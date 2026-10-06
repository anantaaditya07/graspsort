// sort_task_node: SortObjects action server on /sort_objects (architecture 6, 7.5, Phase 5).
//
// Per goal, until no candidate remains, max_objects objects are picked or time_limit passes:
//   detect:  wait until /objects_3d has settled (same track ids, no motion > settle_tolerance for
//            settle_time); candidates = perceived objects of the goal's classes (empty = every
//            class with a bin) that are not inside a bin footprint (+ bin_margin) and not given
//            up (matched by position, track ids change).
//   select:  (feedback stage "detect") the object that failed its last attempt first (same class
//   within match_radius;
//            retry k starts at grasp candidate k); otherwise the candidate closest to the arm
//            base that has a grasp candidate (grasp_planner, all other perceived objects as
//            neighbours) and a collision-free IK solution for its pre-grasp and grasp.
//   pick:    PickPlaceExecutor::pickAndPlace (freeze ... place ... ready, unfreeze).
//   retry:   a failed object is retried up to max_attempts - 1 more times, then given up.
// Objects that are never attempted because they are unreachable or have no grasp candidate are
// reported as failed with that category when nothing else is left.
//
// Feedback: current_object "object_<id> (<class>)", stage detect / plan_grasp / approach / grasp
// / transport / place. Result: picked, failed, failure_reasons "<object> (<class>): <category>:
// <detail>" (one per failed object). Cancel: checked between steps; the executor recovers (open
// gripper, detach, unfreeze) and the goal ends canceled with the counts so far.
//
// Evaluation hook (architecture 7.5/8): metrics_log_path != "" appends one JSON line per attempt.
#include <atomic>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <graspsort_msgs/action/sort_objects.hpp>
#include <iomanip>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "graspsort_manipulation/pick_place_executor.hpp"
#include "graspsort_manipulation/sort_logic.hpp"

namespace gm = graspsort::manipulation;
using namespace std::chrono_literals;
using SortObjects = graspsort_msgs::action::SortObjects;
using GoalHandle = rclcpp_action::ServerGoalHandle<SortObjects>;

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
    } else if (static_cast<unsigned char>(c) >= 0x20) {
      out += c;
    }
  }
  return out;
}

std::string uuidHex(const rclcpp_action::GoalUUID& id) {
  std::ostringstream s;
  for (const auto b : id) {
    s << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(b);
  }
  return s.str();
}

}  // namespace

class SortTask {
 public:
  explicit SortTask(const rclcpp::Node::SharedPtr& node) : node_(node), exec_(node) {
    auto& n = *node_;
    action_name_ = n.declare_parameter<std::string>("sort.action_name", "/sort_objects");
    settle_time_ = n.declare_parameter<double>("sort.settle_time", 2.0);
    settle_tolerance_ = n.declare_parameter<double>("sort.settle_tolerance", 0.01);
    settle_timeout_ = n.declare_parameter<double>("sort.settle_timeout", 30.0);
    bin_margin_ = n.declare_parameter<double>("sort.bin_margin", 0.02);
    match_radius_ = n.declare_parameter<double>("sort.match_radius", 0.05);
    max_attempts_ = n.declare_parameter<int>("sort.max_attempts", 3);
    max_objects_ = n.declare_parameter<int>("sort.max_objects", 20);
    time_limit_ = n.declare_parameter<double>("sort.time_limit", 900.0);
    start_at_ready_ = n.declare_parameter<bool>("sort.start_at_ready", true);
    ready_tolerance_ = n.declare_parameter<double>("sort.ready_tolerance", 0.01);
    metrics_log_path_ = n.declare_parameter<std::string>("metrics_log_path", "");
    bin_names_ = n.get_parameter("bin_names").as_string_array();
    static_cast<void>(gm::SortBook(max_attempts_, match_radius_));  // validates the values
  }

  void connect() {
    while (rclcpp::ok()) {
      try {
        exec_.connect();
        break;
      } catch (const std::exception& e) {
        RCLCPP_WARN(node_->get_logger(), "connect: %s; retrying", e.what());
        std::this_thread::sleep_for(2s);
      }
    }
    group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    server_ = rclcpp_action::create_server<SortObjects>(
        node_, action_name_,
        [this](const rclcpp_action::GoalUUID&, std::shared_ptr<const SortObjects::Goal> goal) {
          return onGoal(*goal);
        },
        [this](const std::shared_ptr<GoalHandle>&) {
          RCLCPP_INFO(node_->get_logger(), "cancel requested");
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<GoalHandle> gh) {
          if (worker_.joinable()) {
            worker_.join();
          }
          worker_ = std::thread([this, gh]() { execute(gh); });
        },
        rcl_action_server_get_default_options(), group_);
    RCLCPP_INFO(node_->get_logger(), "SortObjects action server ready on %s", action_name_.c_str());
  }

  ~SortTask() {
    if (worker_.joinable()) {
      worker_.join();
    }
  }

 private:
  rclcpp_action::GoalResponse onGoal(const SortObjects::Goal& goal) {
    if (busy_) {
      RCLCPP_WARN(node_->get_logger(), "goal rejected: a sort is already running");
      return rclcpp_action::GoalResponse::REJECT;
    }
    try {
      gm::goalClasses(goal.classes, exec_.binAssignment());
    } catch (const std::exception& e) {
      RCLCPP_WARN(node_->get_logger(), "goal rejected: %s", e.what());
      return rclcpp_action::GoalResponse::REJECT;
    }
    busy_ = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  using PickPlaceMsgPtr = gm::PickPlaceExecutor::ObjectsMsg::ConstSharedPtr;

  struct GoalState {
    std::shared_ptr<GoalHandle> gh;
    std::string goal_id;
    std::shared_ptr<SortObjects::Feedback> feedback{std::make_shared<SortObjects::Feedback>()};
    void publish(const std::string& object, const std::string& stage) {
      if (feedback->current_object == object && feedback->stage == stage) {
        return;
      }
      feedback->current_object = object;
      feedback->stage = stage;
      gh->publish_feedback(feedback);
    }
  };

  // Waits until /objects_3d has settled (or settle_timeout); returns the latest objects.
  std::vector<gm::Object> waitSettled(const std::shared_ptr<GoalHandle>& gh) {
    gm::SettleDetector settle(settle_time_, settle_tolerance_);
    const auto t0 = std::chrono::steady_clock::now();
    PickPlaceMsgPtr last;
    PickPlaceMsgPtr seen;
    while (rclcpp::ok() && !gh->is_canceling()) {
      const auto msg = exec_.latestObjects();
      if (msg && msg != seen) {
        seen = msg;
        std::map<std::uint32_t, Eigen::Vector3d> m;
        for (const auto& o : msg->objects) {
          m[o.id] = Eigen::Vector3d(o.pose.position.x, o.pose.position.y, o.pose.position.z);
        }
        if (settle.update(secondsSince(t0), m)) {
          last = msg;
          break;
        }
      }
      if (secondsSince(t0) > settle_timeout_) {
        RCLCPP_WARN(node_->get_logger(), "perception did not settle in %.1f s; using the latest",
                    settle_timeout_);
        last = msg;
        break;
      }
      std::this_thread::sleep_for(50ms);
    }
    std::vector<gm::Object> out;
    if (last) {
      for (const auto& o : last->objects) {
        out.push_back(gm::PickPlaceExecutor::toObject(o));
      }
    }
    return out;
  }

  std::string label(const gm::Object& o) const {
    return gm::objectLabel(exec_.objectIdPrefix(), o.id);
  }

  void execute(const std::shared_ptr<GoalHandle> gh) {
    GoalState g;
    g.gh = gh;
    g.goal_id = uuidHex(gh->get_goal_id());
    auto result = std::make_shared<SortObjects::Result>();
    const auto classes = gm::goalClasses(gh->get_goal()->classes, exec_.binAssignment());
    std::string cls_list;
    for (const auto& c : classes) {
      cls_list += (cls_list.empty() ? "" : ", ") + c;
    }
    RCLCPP_INFO(node_->get_logger(), "goal %s: sort [%s]", g.goal_id.substr(0, 8).c_str(),
                cls_list.c_str());
    gm::SortBook book(max_attempts_, match_radius_);
    std::map<std::string, std::vector<gm::Object>> bin_contents;  // placed this goal
    const auto t_goal = std::chrono::steady_clock::now();
    std::uint32_t picked = 0;
    bool canceled = false;
    std::string stop_reason = "no candidates left";

    try {
      std::vector<gm::NamedBin> bins;
      for (const auto& b : bin_names_) {
        bins.push_back(gm::NamedBin{b, exec_.binGeometry(b)});
      }
      // D-24: from the spawn pose (home) the reach check finds no collision-free IK, so every
      // object of the first goal after launch was skipped in 2 s. Start each goal at ready.
      if (start_at_ready_) {
        try {
          if (exec_.moveToReadyIfAway(ready_tolerance_)) {
            RCLCPP_INFO(node_->get_logger(), "moved to the ready state before sorting");
          }
        } catch (const std::exception& e) {
          RCLCPP_WARN(node_->get_logger(), "move to the ready state failed: %s", e.what());
        }
      }
      while (rclcpp::ok()) {
        if (gh->is_canceling()) {
          canceled = true;
          stop_reason = "canceled";
          break;
        }
        if (static_cast<int>(picked) >= max_objects_) {
          stop_reason = "max_objects reached";
          break;
        }
        if (secondsSince(t_goal) > time_limit_) {
          stop_reason = "time_limit reached";
          break;
        }
        // detect
        g.publish("", "detect");
        const auto t_detect = std::chrono::steady_clock::now();
        const auto objects = waitSettled(gh);
        if (gh->is_canceling()) {
          canceled = true;
          stop_reason = "canceled";
          break;
        }
        const auto cand = gm::filterCandidates(objects, classes, bins, bin_margin_, book);
        const Eigen::Vector3d base = exec_.basePosition();

        std::optional<std::size_t> chosen;
        std::size_t first_candidate = 0;
        if (book.retryTarget()) {
          const auto& rt = *book.retryTarget();
          for (const auto i : gm::orderByDistance(objects, cand, rt.position)) {
            if (book.matchesRetry(objects[i].center, objects[i].class_name)) {
              chosen = i;
              break;
            }
          }
          if (!chosen) {
            RCLCPP_WARN(node_->get_logger(), "retry target %s not perceived again; giving up",
                        rt.label.c_str());
            book.giveUpRetry(gm::FailureCategory::kMisdetection,
                             "not perceived again after a failed attempt");
            continue;
          }
          first_candidate = static_cast<std::size_t>(rt.attempts);
          const auto& o = objects[*chosen];
          g.publish(label(o) + " (" + o.class_name + ")", "detect");
          const auto rc = exec_.checkReachable(o, objects, first_candidate);
          if (!rc.ok) {
            const bool gave_up =
                book.recordFailure(o.center, o.class_name, label(o), rc.category, rc.detail);
            logAttempt(g, o, rt.attempts + 1, false, rc.category, "reach_check", {}, 0.0,
                       secondsSince(t_detect));
            RCLCPP_WARN(node_->get_logger(), "retry of %s not reachable: %s%s", label(o).c_str(),
                        rc.detail.c_str(), gave_up ? " (given up)" : "");
            continue;
          }
        } else {
          std::vector<std::pair<std::size_t, gm::ReachCheck>> rejected;
          for (const auto i : gm::orderByDistance(objects, cand, base)) {
            const auto& o = objects[i];
            g.publish(label(o) + " (" + o.class_name + ")", "detect");
            const auto rc = exec_.checkReachable(o, objects, 0);
            if (rc.ok) {
              chosen = i;
              break;
            }
            RCLCPP_INFO(node_->get_logger(), "%s (%s) skipped: %s: %s", label(o).c_str(),
                        o.class_name.c_str(), gm::toString(rc.category), rc.detail.c_str());
            rejected.emplace_back(i, rc);
          }
          if (!chosen) {
            for (const auto& [i, rc] : rejected) {
              const auto& o = objects[i];
              book.giveUp(o.center, o.class_name, label(o), rc.category, rc.detail);
              logAttempt(g, o, 0, false, rc.category, "reach_check", {}, 0.0,
                         secondsSince(t_detect));
            }
            stop_reason = rejected.empty() ? "no candidates left"
                                           : "only unreachable / ungraspable objects left";
            break;
          }
        }

        // pick and place
        const auto& target = objects[*chosen];
        const int attempt = book.attemptsSoFar(target.center, target.class_name) + 1;
        const std::string obj_label = label(target) + " (" + target.class_name + ")";
        RCLCPP_INFO(node_->get_logger(), "attempt %d on %s, candidate %zu", attempt,
                    obj_label.c_str(), first_candidate);
        gm::PickRequest req;
        req.target = target;
        req.scene = objects;
        req.first_candidate = first_candidate;
        const auto bin = exec_.binAssignment().binFor(target.class_name);
        req.bin_occupied = bin_contents[*bin];
        const auto t_attempt = std::chrono::steady_clock::now();
        const double detect_s = std::chrono::duration<double>(t_attempt - t_detect).count();
        const auto out = exec_.pickAndPlace(
            req,
            [&](const std::string& step) {
              const auto stage = gm::actionStageFor(step);
              if (!stage.empty()) {
                g.publish(obj_label, stage);
              }
            },
            [&gh]() { return gh->is_canceling(); });
        const double cycle = detect_s + out.to_retreat_time;
        logAttempt(g, out.target, attempt, out.placed, out.category,
                   out.success ? "" : out.failed_step, out.steps, out.planning_time, cycle,
                   detect_s);
        if (out.placed) {
          ++picked;
          book.recordSuccess();
          gm::Object placed = out.target;
          placed.center.x() = out.release_xy.x();
          placed.center.y() = out.release_xy.y();
          bin_contents[out.bin].push_back(placed);
          RCLCPP_INFO(node_->get_logger(), "%s placed in %s (cycle %.1f s)%s", obj_label.c_str(),
                      out.bin.c_str(), cycle,
                      out.success ? "" : (" but step " + out.failed_step + " failed").c_str());
        } else if (out.canceled) {
          canceled = true;
          stop_reason = "canceled";
          break;
        } else {
          const bool gave_up =
              book.recordFailure(out.target.center, out.target.class_name, label(out.target),
                                 out.category, out.failed_step + ": " + out.message);
          RCLCPP_WARN(node_->get_logger(), "attempt %d on %s failed at %s (%s)%s", attempt,
                      obj_label.c_str(), out.failed_step.c_str(), gm::toString(out.category),
                      gave_up ? "; given up" : "; will retry");
        }
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(node_->get_logger(), "sort aborted: %s", e.what());
      stop_reason = std::string("error: ") + e.what();
      if (book.retryTarget()) {
        book.giveUpRetry(gm::FailureCategory::kOther, stop_reason);
      }
      result->picked = picked;
      result->failed = static_cast<std::uint32_t>(book.givenUpObjects().size());
      result->failure_reasons = book.failureReasons();
      gh->abort(result);
      busy_ = false;
      return;
    }
    if (book.retryTarget() && !canceled) {
      book.giveUpRetry(gm::FailureCategory::kOther, "not retried: " + stop_reason);
    }
    result->picked = picked;
    result->failed = static_cast<std::uint32_t>(book.givenUpObjects().size());
    result->failure_reasons = book.failureReasons();
    RCLCPP_INFO(node_->get_logger(), "goal %s %s: picked %u, failed %u (%s) in %.1f s",
                g.goal_id.substr(0, 8).c_str(), canceled ? "CANCELED" : "done", result->picked,
                result->failed, stop_reason.c_str(), secondsSince(t_goal));
    for (const auto& r : result->failure_reasons) {
      RCLCPP_WARN(node_->get_logger(), "  failed: %s", r.c_str());
    }
    if (canceled) {
      gh->canceled(result);
    } else {
      gh->succeed(result);
    }
    busy_ = false;
  }

  // One JSON line per attempt (or per never-attempted object) to metrics_log_path.
  void logAttempt(const GoalState& g, const gm::Object& o, int attempt, bool success,
                  gm::FailureCategory category, const std::string& stage,
                  const std::vector<gm::StepRecord>& steps, double planning_time, double cycle,
                  double detect_s = -1.0) {
    if (metrics_log_path_.empty()) {
      return;
    }
    std::ostringstream js;
    js << std::setprecision(6) << "{\"stamp\":" << std::fixed << node_->now().seconds()
       << std::defaultfloat << ",\"goal_id\":\"" << g.goal_id << "\",\"object\":\"" << label(o)
       << "\",\"class\":\"" << jsonEscape(o.class_name) << "\",\"attempt\":" << attempt
       << ",\"success\":" << (success ? "true" : "false") << ",\"category\":\""
       << (success ? "none" : gm::toString(category)) << "\",\"stage\":\"" << jsonEscape(stage)
       << "\",\"stage_times\":{";
    bool first = true;
    if (detect_s >= 0.0) {
      js << "\"detect\":" << detect_s;
      first = false;
    }
    for (const auto& s : steps) {
      js << (first ? "" : ",") << "\"" << s.name << "\":" << s.seconds;
      first = false;
    }
    js << "},\"planning_time\":" << planning_time << ",\"cycle_time\":" << cycle
       << ",\"target_pose\":{\"x\":" << o.center.x() << ",\"y\":" << o.center.y()
       << ",\"z\":" << o.center.z() << ",\"yaw\":" << o.yaw << "}}";
    std::lock_guard<std::mutex> lock(log_mutex_);
    std::ofstream f(metrics_log_path_, std::ios::app);
    if (!f) {
      RCLCPP_ERROR(node_->get_logger(), "cannot append to %s", metrics_log_path_.c_str());
      return;
    }
    f << js.str() << "\n";
  }

  rclcpp::Node::SharedPtr node_;
  gm::PickPlaceExecutor exec_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp_action::Server<SortObjects>::SharedPtr server_;
  std::thread worker_;
  std::atomic<bool> busy_{false};
  std::mutex log_mutex_;

  std::string action_name_;
  double settle_time_{2.0}, settle_tolerance_{0.01}, settle_timeout_{30.0}, bin_margin_{0.02},
      match_radius_{0.05}, time_limit_{900.0}, ready_tolerance_{0.01};
  bool start_at_ready_{true};
  int max_attempts_{3}, max_objects_{20};
  std::string metrics_log_path_;
  std::vector<std::string> bin_names_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("sort_task_node");
  int code = 0;
  {
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor]() { executor.spin(); });
    try {
      SortTask task(node);
      task.connect();
      while (rclcpp::ok()) {
        std::this_thread::sleep_for(200ms);
      }
    } catch (const std::exception& e) {
      RCLCPP_FATAL(node->get_logger(), "sort_task_node: %s", e.what());
      code = 1;
    }
    executor.cancel();
    spinner.join();
  }
  rclcpp::shutdown();
  return code;
}
