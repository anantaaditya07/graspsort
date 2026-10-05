// pick_place_test: one scripted pick-and-place of the nearest perceived object of a class
// (architecture 7.5, Phase 4). Not the sort task node: no action server, one object, then exit.
//
// Selects the object of target_class nearest to the arm base (xy) on /objects_3d and runs ONE
// PickPlaceExecutor::pickAndPlace on it (step sequence, candidate handling and recovery are in
// pick_place_executor.hpp). If the target changes at the freeze (new track id), it re-selects
// until object_wait_timeout + collision_wait_timeout has passed.
//
// Exit 0 only on full success. The last stdout line is "PICK_PLACE_RESULT {json}" for
// scripts/pick_place_trials.py. The robot never reads Gazebo ground truth here.
#include <chrono>
#include <cstdio>
#include <limits>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "graspsort_manipulation/pick_place_executor.hpp"

namespace gm = graspsort::manipulation;
using namespace std::chrono_literals;

namespace {

double secondsSince(std::chrono::steady_clock::time_point t0) {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
}

std::string jsonEscape(const std::string& s) {
  std::string out;
  for (const char c : s) {
    if (c == '"' || c == '\\') {
      out += '\\';
      out += c;
    } else if (c == '\n') {
      out += "\\n";
    } else {
      out += c;
    }
  }
  return out;
}

class PickPlaceTest {
 public:
  explicit PickPlaceTest(const rclcpp::Node::SharedPtr& node) : node_(node), exec_(node) {
    target_class_ = node_->declare_parameter<std::string>("target_class", "sports ball");
    object_wait_timeout_ = node_->declare_parameter<double>("object_wait_timeout", 30.0);
    // Already declared by the executor; read here for the overall selection deadline.
    collision_wait_timeout_ = node_->get_parameter("collision_wait_timeout").as_double();
  }

  int run() {
    const auto t_total = std::chrono::steady_clock::now();
    std::vector<gm::StepRecord> pre;
    gm::PickOutcome out;
    std::string failure;
    std::string failed_step;
    std::string bin_name;
    std::uint32_t object_id = 0;
    bool success = false;
    try {
      timed(pre, "connect", [&](gm::StepRecord& r) {
        exec_.connect();
        r.detail = "connected";
      });
      const auto bin = exec_.binAssignment().binFor(target_class_);
      if (!bin) {
        throw std::runtime_error("no bin for class '" + target_class_ + "'");
      }
      bin_name = *bin;
      // Select and pick; re-select while the target changes at the freeze.
      const auto t0 = std::chrono::steady_clock::now();
      while (true) {
        gm::PickRequest req;
        timed(pre, "select", [&](gm::StepRecord& r) { req = select(r); });
        object_id = req.target.id;
        out = exec_.pickAndPlace(req);
        const bool retry = !out.success && !out.canceled && out.failed_step == "lock_target" &&
                           secondsSince(t0) < object_wait_timeout_ + collision_wait_timeout_;
        if (!retry) {
          break;
        }
        RCLCPP_WARN(node_->get_logger(), "target changed (%s); re-selecting", out.message.c_str());
        std::this_thread::sleep_for(500ms);
      }
      success = out.success;
      if (!success) {
        failed_step = out.failed_step;
        failure = std::string(gm::toString(out.category)) + ": " + out.message;
      }
    } catch (const std::exception& e) {
      failed_step = pre.empty() ? "?" : pre.back().name;
      failure = e.what();
      RCLCPP_ERROR(node_->get_logger(), "%s FAILED: %s", failed_step.c_str(), e.what());
    }
    std::vector<gm::StepRecord> all = pre;
    all.insert(all.end(), out.steps.begin(), out.steps.end());
    printReport(success, object_id, bin_name, failed_step, failure, out.recovery, all,
                secondsSince(t_total));
    return success ? 0 : 1;
  }

 private:
  template <typename Fn>
  void timed(std::vector<gm::StepRecord>& v, const std::string& name, Fn fn) {
    v.push_back(gm::StepRecord{name, false, 0.0, 0.0, ""});
    const auto t0 = std::chrono::steady_clock::now();
    try {
      fn(v.back());
    } catch (...) {
      v.back().seconds = secondsSince(t0);
      throw;
    }
    v.back().ok = true;
    v.back().seconds = secondsSince(t0);
  }

  // Nearest object of target_class to the arm base (xy); scene = all perceived objects.
  gm::PickRequest select(gm::StepRecord& r) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                              std::chrono::duration<double>(object_wait_timeout_));
    const Eigen::Vector3d base = exec_.basePosition();
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      const auto msg = exec_.latestObjects();
      if (msg) {
        gm::PickRequest req;
        double best = std::numeric_limits<double>::infinity();
        std::optional<gm::Object> chosen;
        for (const auto& o : msg->objects) {
          const auto obj = gm::PickPlaceExecutor::toObject(o);
          req.scene.push_back(obj);
          if (o.class_name != target_class_) {
            continue;
          }
          const double d = gm::distanceXY(obj.center, base);
          if (d < best) {
            best = d;
            chosen = obj;
          }
        }
        if (chosen) {
          req.target = *chosen;
          std::ostringstream s;
          s << "id " << chosen->id << ", " << best << " m from base";
          r.detail = s.str();
          return req;
        }
      }
      std::this_thread::sleep_for(100ms);
    }
    throw std::runtime_error("no '" + target_class_ + "' on " + exec_.objectsTopic());
  }

  void printReport(bool success, std::uint32_t object_id, const std::string& bin,
                   const std::string& failed_step, const std::string& failure,
                   const std::string& recovery, const std::vector<gm::StepRecord>& steps,
                   double total) {
    std::printf("\n%-22s %-4s %8s %8s  %s\n", "stage", "ok", "time_s", "plan_s", "detail");
    std::ostringstream js;
    js << "{\"success\":" << (success ? "true" : "false") << ",\"target_class\":\""
       << jsonEscape(target_class_) << "\",\"object_id\":" << object_id << ",\"bin\":\""
       << jsonEscape(bin) << "\",\"failed_stage\":\"" << jsonEscape(failed_step)
       << "\",\"failure\":\"" << jsonEscape(failure) << "\",\"recovery\":\"" << jsonEscape(recovery)
       << "\",\"total_s\":" << total << ",\"stages\":[";
    for (std::size_t i = 0; i < steps.size(); ++i) {
      const auto& r = steps[i];
      std::printf("%-22s %-4s %8.2f %8.3f  %s\n", r.name.c_str(), r.ok ? "yes" : "NO", r.seconds,
                  r.planning_seconds, r.detail.c_str());
      js << (i ? "," : "") << "{\"name\":\"" << r.name << "\",\"ok\":" << (r.ok ? "true" : "false")
         << ",\"s\":" << r.seconds << ",\"plan_s\":" << r.planning_seconds << ",\"detail\":\""
         << jsonEscape(r.detail) << "\"}";
    }
    js << "]}";
    std::printf("%s in %.1f s%s%s\n", success ? "SUCCESS" : "FAILURE", total,
                success ? "" : ", failed stage: ", failed_step.c_str());
    std::printf("PICK_PLACE_RESULT %s\n", js.str().c_str());
    std::fflush(stdout);
  }

  rclcpp::Node::SharedPtr node_;
  gm::PickPlaceExecutor exec_;
  std::string target_class_;
  double object_wait_timeout_{30.0};
  double collision_wait_timeout_{10.0};
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("pick_place_test");
  int code = 2;
  {
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor]() { executor.spin(); });
    try {
      PickPlaceTest test(node);
      code = test.run();
    } catch (const std::exception& e) {
      RCLCPP_FATAL(node->get_logger(), "pick_place_test: %s", e.what());
      code = 2;
    }
    executor.cancel();
    spinner.join();
  }
  rclcpp::shutdown();
  return code;
}
