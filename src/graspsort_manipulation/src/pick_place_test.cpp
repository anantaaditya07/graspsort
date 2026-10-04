// pick_place_test: one scripted pick-and-place of the nearest perceived object of a class
// (architecture 7.5, Phase 4). Not the sort task node: no action server, one object, then exit.
//
// Sequence: select object (/objects_3d) -> wait for its collision object -> freeze scene ->
// open gripper -> OMPL to pre-grasp -> Pilz LIN approach -> close to width - squeeze (D-02) ->
// Gazebo /attach -> MoveIt attachObject -> LIN lift -> OMPL above bin -> LIN lower -> open ->
// Gazebo /detach -> MoveIt detachObject -> remove the collision object -> LIN retreat ->
// OMPL to "ready" -> unfreeze.
// Grasp candidates come from grasp_planner.hpp (ranked); a candidate whose pre-grasp or approach
// cannot be planned is skipped, up to max_candidates.
//
// On any failure: open the gripper, detach (Gazebo and MoveIt) if attached, unfreeze, print the
// failing stage, exit 1. Exit 0 only on full success. The last stdout line is
// "PICK_PLACE_RESULT {json}" for scripts/pick_place_trials.py.
//
// The robot never reads Gazebo ground truth here; the object comes from perception only.
// D-10: no *_mimic joint name is ever put into a MoveIt request (start states are built from the
// RobotModel, whose mimic joint is gripper_right_finger_joint).
#include <moveit/collision_detection/collision_matrix.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <control_msgs/action/gripper_command.hpp>
#include <cstdio>
#include <functional>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <graspsort_msgs/msg/object_pose_array.hpp>
#include <graspsort_msgs/srv/attach_link.hpp>
#include <limits>
#include <memory>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sstream>
#include <std_srvs/srv/set_bool.hpp>
#include <string>
#include <thread>
#include <vector>

#include "graspsort_manipulation/bin_assignment.hpp"
#include "graspsort_manipulation/grasp_planner.hpp"
#include "graspsort_manipulation/pick_place_geometry.hpp"

namespace gm = graspsort::manipulation;
using namespace std::chrono_literals;
using MoveGroup = moveit::planning_interface::MoveGroupInterface;
using GripperCommand = control_msgs::action::GripperCommand;
using AttachLink = graspsort_msgs::srv::AttachLink;
using SetBool = std_srvs::srv::SetBool;
using GetPlanningScene = moveit_msgs::srv::GetPlanningScene;

namespace {

struct StageResult {
  std::string name;
  bool ok{false};
  double seconds{0.0};
  double planning_seconds{0.0};
  std::string detail;
};

class StageFailure : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

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

geometry_msgs::msg::Pose toMsg(const Eigen::Isometry3d& t) {
  geometry_msgs::msg::Pose p;
  p.position.x = t.translation().x();
  p.position.y = t.translation().y();
  p.position.z = t.translation().z();
  const Eigen::Quaterniond q(t.linear());
  p.orientation.x = q.x();
  p.orientation.y = q.y();
  p.orientation.z = q.z();
  p.orientation.w = q.w();
  return p;
}

double yawOf(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

gm::Object toObject(const graspsort_msgs::msg::ObjectPose& o) {
  gm::Object out;
  out.id = o.id;
  out.class_name = o.class_name;
  out.center = {o.pose.position.x, o.pose.position.y, o.pose.position.z};
  out.size = {o.size.x, o.size.y, o.size.z};
  out.yaw = yawOf(o.pose.orientation);
  out.shape = o.shape;
  return out;
}

}  // namespace

class PickPlaceTest {
 public:
  explicit PickPlaceTest(const rclcpp::Node::SharedPtr& node) : node_(node) {
    loadParameters();
    objects_sub_ = node_->create_subscription<graspsort_msgs::msg::ObjectPoseArray>(
        objects_topic_, rclcpp::QoS(10),
        [this](graspsort_msgs::msg::ObjectPoseArray::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(objects_mutex_);
          objects_ = msg;
        });
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    gripper_client_ = rclcpp_action::create_client<GripperCommand>(node_, gripper_action_);
    attach_client_ = node_->create_client<AttachLink>(attach_service_);
    detach_client_ = node_->create_client<AttachLink>(detach_service_);
    freeze_client_ = node_->create_client<SetBool>(freeze_service_);
    scene_client_ = node_->create_client<GetPlanningScene>(scene_service_);
  }

  // Must be called after the node spins (MoveGroupInterface needs /joint_states and /clock).
  int run() {
    const auto t_total = std::chrono::steady_clock::now();
    bool success = false;
    std::string failed_stage;
    std::string failure;
    try {
      stage("connect", [&](StageResult& r) { connect(r); });
      stage("select_and_freeze", [&](StageResult& r) { selectAndFreeze(r); });
      stage("open_gripper", [&](StageResult& r) { moveGripper(cfg_.open_width, false, r); });
      stage("pregrasp", [&](StageResult& r) { pregraspWithCandidates(r); });
      stage("approach", [&](StageResult& r) { moveLin(grasp_pose_, r); });
      stage("close_gripper", [&](StageResult& r) { closeGripper(r); });
      stage("attach_gazebo", [&](StageResult& r) {
        callAttach(attach_client_, r);
        gazebo_attached_ = true;
      });
      stage("attach_moveit", [&](StageResult& r) {
        allowSupportContact(true, r);
        attachMoveIt(r);
      });
      stage("lift", [&](StageResult& r) { moveLin(gm::raised(grasp_pose_, cfg_.lift_height), r); });
      stage("move_above_bin",
            [&](StageResult& r) { moveFree(above_bin_pose_, &release_pose_, r); });
      stage("lower", [&](StageResult& r) { moveLin(release_pose_, r); });
      stage("release_gripper", [&](StageResult& r) { moveGripper(cfg_.open_width, false, r); });
      stage("detach_gazebo", [&](StageResult& r) {
        callAttach(detach_client_, r);
        gazebo_attached_ = false;
      });
      stage("detach_moveit", [&](StageResult& r) { detachMoveIt(r); });
      stage("remove_object", [&](StageResult& r) { removeObject(r); });
      stage("retreat",
            [&](StageResult& r) { moveLin(gm::raised(release_pose_, cfg_.retreat_height), r); });
      stage("ready", [&](StageResult& r) { moveNamed(ready_state_, r); });
      stage("unfreeze_scene", [&](StageResult& r) {
        callSetBool(freeze_client_, false, r);
        frozen_ = false;
      });
      success = true;
    } catch (const StageFailure& e) {
      failed_stage = results_.empty() ? "?" : results_.back().name;
      failure = e.what();
      RCLCPP_ERROR(node_->get_logger(), "stage '%s' FAILED: %s", failed_stage.c_str(), e.what());
      recover();
    }
    if (!success) {
      RCLCPP_WARN(node_->get_logger(), "recovery: %s", recovery_log_.c_str());
    }
    printReport(success, failed_stage, failure, secondsSince(t_total));
    return success ? 0 : 1;
  }

 private:
  // ------------------------------------------------------------------------------------------
  // Parameters
  // ------------------------------------------------------------------------------------------
  void loadParameters() {
    auto& n = *node_;
    target_class_ = n.declare_parameter<std::string>("target_class", "sports ball");
    objects_topic_ = n.declare_parameter<std::string>("objects_topic", "/objects_3d");
    object_wait_timeout_ = n.declare_parameter<double>("object_wait_timeout", 30.0);
    world_frame_ = n.declare_parameter<std::string>("world_frame", "world");
    base_frame_ = n.declare_parameter<std::string>("base_frame", "base_link");
    arm_group_ = n.declare_parameter<std::string>("arm_group", "ur_manipulator");
    tcp_link_ = n.declare_parameter<std::string>("tcp_link", "gripper_tcp");
    attach_link_ = n.declare_parameter<std::string>("attach_link", "gripper_tcp");
    touch_links_ = n.declare_parameter<std::vector<std::string>>(
        "touch_links", {"gripper_base_link", "gripper_left_finger_link",
                        "gripper_right_finger_link", "gripper_tcp"});
    ready_state_ = n.declare_parameter<std::string>("ready_state", "ready");
    object_id_prefix_ = n.declare_parameter<std::string>("object_id_prefix", "object_");
    collision_wait_timeout_ = n.declare_parameter<double>("collision_wait_timeout", 10.0);
    scene_wait_timeout_ = n.declare_parameter<double>("scene_wait_timeout", 3.0);
    freeze_recheck_delay_ = n.declare_parameter<double>("freeze_recheck_delay", 0.3);

    const auto bin_classes =
        n.declare_parameter<std::vector<std::string>>("bin_classes", gm::defaultBinClasses());
    const auto bin_names =
        n.declare_parameter<std::vector<std::string>>("bin_names", gm::defaultBinNames());
    bins_ =
        std::make_unique<gm::BinAssignment>(gm::BinAssignment::fromLists(bin_classes, bin_names));
    // Bin geometry from graspsort_gazebo/config/world_layout.yaml (passed as a params file).
    const auto bin_size = n.declare_parameter<std::vector<double>>("bins.size", {0.20, 0.20, 0.08});
    if (bin_size.size() != 3) {
      throw std::invalid_argument("bins.size must have 3 values");
    }
    bin_geometry_.size = {bin_size[0], bin_size[1], bin_size[2]};
    bin_geometry_.wall_thickness = n.declare_parameter<double>("bins.wall_thickness", 0.01);
    bin_geometry_.floor_thickness = n.declare_parameter<double>("bins.floor_thickness", 0.01);
    tf_timeout_ = n.declare_parameter<double>("tf_timeout", 5.0);

    gm::GraspPlannerConfig g;
    g.cylinder_yaw_count = static_cast<std::size_t>(n.declare_parameter<int>(
        "grasp.cylinder_yaw_count", static_cast<int>(g.cylinder_yaw_count)));
    g.finger_height_fraction =
        n.declare_parameter<double>("grasp.finger_height_fraction", g.finger_height_fraction);
    g.max_opening = n.declare_parameter<double>("grasp.max_opening", g.max_opening);
    g.width_margin = n.declare_parameter<double>("grasp.width_margin", g.width_margin);
    g.max_grasp_width = n.declare_parameter<double>("grasp.max_grasp_width", g.max_grasp_width);
    g.finger_thickness = n.declare_parameter<double>("grasp.finger_thickness", g.finger_thickness);
    g.finger_width = n.declare_parameter<double>("grasp.finger_width", g.finger_width);
    g.min_finger_clearance =
        n.declare_parameter<double>("grasp.min_finger_clearance", g.min_finger_clearance);
    g.clearance_cap = n.declare_parameter<double>("grasp.clearance_cap", g.clearance_cap);
    g.yaw_weight = n.declare_parameter<double>("grasp.yaw_weight", g.yaw_weight);
    g.clearance_weight = n.declare_parameter<double>("grasp.clearance_weight", g.clearance_weight);
    planner_ = std::make_unique<gm::GraspPlanner>(g);
    max_candidates_ = n.declare_parameter<int>("grasp.max_candidates", 3);

    cfg_.pregrasp_height =
        n.declare_parameter<double>("motion.pregrasp_height", cfg_.pregrasp_height);
    cfg_.lift_height = n.declare_parameter<double>("motion.lift_height", cfg_.lift_height);
    cfg_.retreat_height = n.declare_parameter<double>("motion.retreat_height", cfg_.retreat_height);
    cfg_.min_fingertip_clearance =
        n.declare_parameter<double>("motion.min_fingertip_clearance", cfg_.min_fingertip_clearance);
    cfg_.max_fingertip_above_center = n.declare_parameter<double>(
        "motion.max_fingertip_above_center", cfg_.max_fingertip_above_center);
    cfg_.release_clearance =
        n.declare_parameter<double>("motion.release_clearance", cfg_.release_clearance);
    cfg_.transport_clearance =
        n.declare_parameter<double>("motion.transport_clearance", cfg_.transport_clearance);
    cfg_.tcp_to_fingertip =
        n.declare_parameter<double>("gripper.tcp_to_fingertip", cfg_.tcp_to_fingertip);
    cfg_.open_width = n.declare_parameter<double>("gripper.open_width", cfg_.open_width);
    cfg_.max_close_position =
        n.declare_parameter<double>("gripper.max_close_position", cfg_.max_close_position);
    cfg_.squeeze = n.declare_parameter<double>("gripper.squeeze", cfg_.squeeze);
    gm::validate(cfg_);
    gripper_action_ =
        n.declare_parameter<std::string>("gripper.action", "/gripper_controller/gripper_cmd");
    gripper_max_effort_ = n.declare_parameter<double>("gripper.max_effort", 50.0);
    gripper_timeout_ = n.declare_parameter<double>("gripper.timeout", 10.0);
    gripper_settle_ = n.declare_parameter<double>("gripper.settle_time", 0.3);
    gripper_open_tolerance_ = n.declare_parameter<double>("gripper.open_tolerance", 0.003);

    ompl_pipeline_ = n.declare_parameter<std::string>("planning.ompl_pipeline", "ompl");
    ompl_planner_ =
        n.declare_parameter<std::string>("planning.ompl_planner", "RRTConnectkConfigDefault");
    lin_pipeline_ =
        n.declare_parameter<std::string>("planning.lin_pipeline", "pilz_industrial_motion_planner");
    lin_planner_ = n.declare_parameter<std::string>("planning.lin_planner", "LIN");
    planning_time_ = n.declare_parameter<double>("planning.planning_time", 5.0);
    planning_attempts_ = n.declare_parameter<int>("planning.planning_attempts", 1);
    free_plan_retries_ = n.declare_parameter<int>("planning.free_plan_retries", 2);
    free_velocity_ = n.declare_parameter<double>("planning.free_velocity_scaling", 0.5);
    free_acceleration_ = n.declare_parameter<double>("planning.free_acceleration_scaling", 0.5);
    lin_velocity_ = n.declare_parameter<double>("planning.lin_velocity_scaling", 0.1);
    lin_acceleration_ = n.declare_parameter<double>("planning.lin_acceleration_scaling", 0.1);
    goal_position_tolerance_ =
        n.declare_parameter<double>("planning.goal_position_tolerance", 0.001);
    goal_orientation_tolerance_ =
        n.declare_parameter<double>("planning.goal_orientation_tolerance", 0.01);

    freeze_service_ = n.declare_parameter<std::string>("services.freeze", "/scene_manager/freeze");
    attach_service_ = n.declare_parameter<std::string>("services.attach", "/attach");
    detach_service_ = n.declare_parameter<std::string>("services.detach", "/detach");
    attach_parent_model_ = n.declare_parameter<std::string>("services.attach_parent_model", "ur");
    attach_parent_link_ =
        n.declare_parameter<std::string>("services.attach_parent_link", "gripper_left_finger_link");
    service_timeout_ = n.declare_parameter<double>("services.timeout", 10.0);
    scene_service_ =
        n.declare_parameter<std::string>("services.get_planning_scene", "/get_planning_scene");
    support_ids_ = n.declare_parameter<std::vector<std::string>>("support_ids", {"table"});
    ik_timeout_ = n.declare_parameter<double>("planning.ik_timeout", 0.1);
  }

  // ------------------------------------------------------------------------------------------
  // Stage bookkeeping
  // ------------------------------------------------------------------------------------------
  void stage(const std::string& name, const std::function<void(StageResult&)>& fn) {
    results_.push_back(StageResult{name, false, 0.0, 0.0, ""});
    RCLCPP_INFO(node_->get_logger(), "--> %s", name.c_str());
    const auto t0 = std::chrono::steady_clock::now();
    try {
      fn(results_.back());
    } catch (const StageFailure& e) {
      results_.back().seconds = secondsSince(t0);
      results_.back().detail = e.what();
      throw;
    } catch (const std::exception& e) {
      results_.back().seconds = secondsSince(t0);
      results_.back().detail = e.what();
      throw StageFailure(e.what());
    }
    results_.back().ok = true;
    results_.back().seconds = secondsSince(t0);
    RCLCPP_INFO(node_->get_logger(), "<-- %s ok (%.2f s) %s", name.c_str(), results_.back().seconds,
                results_.back().detail.c_str());
  }

  void printReport(bool success, const std::string& failed_stage, const std::string& failure,
                   double total) {
    std::printf("\n%-22s %-4s %8s %8s  %s\n", "stage", "ok", "time_s", "plan_s", "detail");
    std::ostringstream js;
    js << "{\"success\":" << (success ? "true" : "false") << ",\"target_class\":\""
       << jsonEscape(target_class_) << "\",\"object_id\":" << target_.id << ",\"bin\":\""
       << jsonEscape(bin_name_) << "\",\"failed_stage\":\"" << jsonEscape(failed_stage)
       << "\",\"failure\":\"" << jsonEscape(failure) << "\",\"recovery\":\""
       << jsonEscape(recovery_log_) << "\",\"total_s\":" << total << ",\"stages\":[";
    for (std::size_t i = 0; i < results_.size(); ++i) {
      const auto& r = results_[i];
      std::printf("%-22s %-4s %8.2f %8.3f  %s\n", r.name.c_str(), r.ok ? "yes" : "NO", r.seconds,
                  r.planning_seconds, r.detail.c_str());
      js << (i ? "," : "") << "{\"name\":\"" << r.name << "\",\"ok\":" << (r.ok ? "true" : "false")
         << ",\"s\":" << r.seconds << ",\"plan_s\":" << r.planning_seconds << ",\"detail\":\""
         << jsonEscape(r.detail) << "\"}";
    }
    js << "]}";
    std::printf("%s in %.1f s%s%s\n", success ? "SUCCESS" : "FAILURE", total,
                success ? "" : ", failed stage: ", failed_stage.c_str());
    std::printf("PICK_PLACE_RESULT %s\n", js.str().c_str());
    std::fflush(stdout);
  }

  // ------------------------------------------------------------------------------------------
  // Stages
  // ------------------------------------------------------------------------------------------
  void connect(StageResult& r) {
    move_group_ = std::make_unique<MoveGroup>(node_, arm_group_);
    move_group_->setPoseReferenceFrame(world_frame_);
    move_group_->setEndEffectorLink(tcp_link_);
    move_group_->setPlanningTime(planning_time_);
    move_group_->setNumPlanningAttempts(planning_attempts_);
    move_group_->setGoalPositionTolerance(goal_position_tolerance_);
    move_group_->setGoalOrientationTolerance(goal_orientation_tolerance_);
    const auto timeout = std::chrono::duration<double>(service_timeout_);
    if (!gripper_client_->wait_for_action_server(timeout)) {
      throw StageFailure("gripper action server " + gripper_action_ + " not available");
    }
    for (const auto& c : {attach_client_, detach_client_}) {
      if (!c->wait_for_service(timeout)) {
        throw StageFailure(std::string("service ") + c->get_service_name() + " not available");
      }
    }
    if (!freeze_client_->wait_for_service(timeout)) {
      throw StageFailure("service " + freeze_service_ + " not available");
    }
    if (!move_group_->getCurrentState(tf_timeout_)) {
      throw StageFailure("no current robot state");
    }
    r.detail = "planning frame " + move_group_->getPlanningFrame();
  }

  // Selects the target, freezes the scene and re-reads the target at freeze time: a moved object
  // gets a new track id in the scene manager, so the target is accepted only if, after the
  // freeze, the latest /objects_3d still holds the same id and object_<id> is in the scene.
  void selectAndFreeze(StageResult& r) {
    const auto t0 = std::chrono::steady_clock::now();
    std::string log;
    while (rclcpp::ok() && secondsSince(t0) < collision_wait_timeout_ + object_wait_timeout_) {
      selectObject(r);
      if (psi_.getObjects({object_id_}).empty()) {
        std::this_thread::sleep_for(200ms);
        continue;
      }
      StageResult f;
      callSetBool(freeze_client_, true, f);
      frozen_ = true;
      const auto selected = target_.id;
      std::this_thread::sleep_for(std::chrono::duration<double>(freeze_recheck_delay_));
      selectObject(r);
      if (target_.id == selected && !psi_.getObjects({object_id_}).empty()) {
        r.detail += "; frozen";
        return;
      }
      log += " target changed after freeze (" + std::to_string(selected) + " -> " +
             std::to_string(target_.id) + ");";
      callSetBool(freeze_client_, false, f);
      frozen_ = false;
      std::this_thread::sleep_for(500ms);
    }
    throw StageFailure("no stable target with a collision object:" + log);
  }

  void selectObject(StageResult& r) {
    const auto bin = bins_->binFor(target_class_);
    if (!bin) {
      throw StageFailure("no bin for class '" + target_class_ + "'");
    }
    bin_name_ = *bin;
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                              std::chrono::duration<double>(object_wait_timeout_));
    const Eigen::Vector3d base = lookup(base_frame_).translation();
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      graspsort_msgs::msg::ObjectPoseArray::ConstSharedPtr msg;
      {
        std::lock_guard<std::mutex> lock(objects_mutex_);
        msg = objects_;
      }
      if (msg) {
        double best = std::numeric_limits<double>::infinity();
        std::optional<gm::Object> chosen;
        scene_objects_.clear();
        for (const auto& o : msg->objects) {
          scene_objects_.push_back(toObject(o));
          if (o.class_name != target_class_) {
            continue;
          }
          const double d = std::hypot(o.pose.position.x - base.x(), o.pose.position.y - base.y());
          if (d < best) {
            best = d;
            chosen = toObject(o);
          }
        }
        if (chosen) {
          target_ = *chosen;
          std::ostringstream s;
          s << "id " << target_.id << " at (" << target_.center.x() << ", " << target_.center.y()
            << ", " << target_.center.z() << ") size (" << target_.size.x() << ", "
            << target_.size.y() << ", " << target_.size.z() << ") yaw " << target_.yaw << ", "
            << best << " m from base; bin " << bin_name_;
          r.detail = s.str();
          object_id_ = object_id_prefix_ + std::to_string(target_.id);
          return;
        }
      }
      std::this_thread::sleep_for(100ms);
    }
    throw StageFailure("no '" + target_class_ + "' on " + objects_topic_);
  }

  // Plans pre-grasp (OMPL) and approach (LIN, from the planned pre-grasp end state) for each
  // ranked candidate; executes the pre-grasp of the first candidate for which both plan.
  void pregraspWithCandidates(StageResult& r) {
    const Eigen::Isometry3d tcp = currentTcp();
    const double wrist_yaw = gm::closingYaw(Eigen::Quaterniond(tcp.linear()));
    const auto cands = planner_->plan(target_, scene_objects_, wrist_yaw);
    if (cands.empty()) {
      throw StageFailure("grasp planner rejected every candidate");
    }
    const auto grasp_z =
        gm::adjustedGraspZ(cands.front().position.z(), target_.center.z(), target_.size.z(), cfg_);
    if (!grasp_z) {
      throw StageFailure("object too flat for the fingertip clearance rules");
    }
    const auto q_close = gm::closePosition(cands.front().width, cfg_);
    if (!q_close) {
      throw StageFailure("no valid close position for width " +
                         std::to_string(cands.front().width));
    }
    close_position_ = *q_close;
    const Eigen::Isometry3d bin_pose = lookup(bin_name_);
    bin_geometry_.pose = bin_pose;
    const double hang = gm::hangBelowTcp(*grasp_z, target_.center.z(), target_.size.z());

    std::string log;
    const int n = std::min<int>(max_candidates_, static_cast<int>(cands.size()));
    for (int i = 0; i < n; ++i) {
      const auto& c = cands[static_cast<std::size_t>(i)];
      const auto q = gm::topDownOrientation(c.yaw);
      const Eigen::Vector3d p(c.position.x(), c.position.y(), *grasp_z);
      const auto grasp = gm::makePose(p, q);
      const auto pre = gm::raised(grasp, cfg_.pregrasp_height);
      if (!gm::openFingersFitInBin(bin_geometry_, c.yaw, planner_->config().finger_thickness,
                                   planner_->config().finger_width, cfg_)) {
        log += " cand" + std::to_string(i) + ": open fingers do not fit in the bin;";
        continue;
      }
      // Pre-grasp (OMPL), accepted only if the LIN approach is plannable from its end state.
      MoveGroup::Plan pre_plan;
      double t_plan = 0.0;
      const bool ok = planFree(pre, pre_plan, t_plan, &grasp);
      r.planning_seconds += t_plan;
      if (!ok) {
        log += " cand" + std::to_string(i) + " (yaw " + std::to_string(c.yaw) +
               "): pre-grasp or approach not plannable;";
        continue;
      }
      if (!execute(pre_plan)) {
        throw StageFailure("pre-grasp execution failed;" + log);
      }
      grasp_pose_ = grasp;
      release_pose_ = gm::releasePose(bin_geometry_, hang, q, cfg_);
      above_bin_pose_ = gm::aboveBinPose(bin_geometry_, hang, q, cfg_);
      std::ostringstream s;
      s << "candidate " << i << " yaw " << c.yaw << " grasp z " << *grasp_z << " width " << c.width
        << " close q " << close_position_ << ";" << log;
      r.detail = s.str();
      return;
    }
    throw StageFailure("no candidate plannable:" + log);
  }

  void closeGripper(StageResult& r) {
    const double gap = cfg_.open_width - 2.0 * close_position_;
    moveGripper(gap, true, r);
  }

  void attachMoveIt(StageResult& r) {
    if (!move_group_->attachObject(object_id_, attach_link_, touch_links_)) {
      throw StageFailure("attachObject(" + object_id_ + ") failed");
    }
    moveit_attached_ = true;
    if (!waitAttached(true)) {
      throw StageFailure(object_id_ + " not attached in the planning scene");
    }
    r.detail += object_id_ + " -> " + attach_link_;
  }

  void detachMoveIt(StageResult& r) {
    if (!move_group_->detachObject(object_id_)) {
      throw StageFailure("detachObject(" + object_id_ + ") failed");
    }
    if (!waitAttached(false)) {
      throw StageFailure(object_id_ + " still attached in the planning scene");
    }
    moveit_attached_ = false;
    r.detail = object_id_;
  }

  // While the object is held, MoveIt may not see it touching its support (the perceived box can
  // reach a few mm into the table): allow object_<id> vs support_ids in the ACM; removed again
  // with the object. The ACM is replaced as a whole in a diff, so read it first.
  void allowSupportContact(bool allow, StageResult& r) {
    if (support_ids_.empty()) {
      return;
    }
    auto req = std::make_shared<GetPlanningScene::Request>();
    req->components.components =
        moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
    auto fut = scene_client_->async_send_request(req);
    if (fut.wait_for(std::chrono::duration<double>(service_timeout_)) !=
        std::future_status::ready) {
      throw StageFailure("get_planning_scene timeout");
    }
    collision_detection::AllowedCollisionMatrix acm(fut.get()->scene.allowed_collision_matrix);
    if (allow) {
      for (const auto& id : support_ids_) {
        acm.setEntry(object_id_, id, true);
      }
    } else {
      acm.removeEntry(object_id_);
    }
    moveit_msgs::msg::PlanningScene ps;
    ps.is_diff = true;
    acm.getMessage(ps.allowed_collision_matrix);
    if (!psi_.applyPlanningScene(ps)) {
      throw StageFailure("applying the ACM failed");
    }
    acm_modified_ = allow;
    if (allow) {
      r.detail += "ACM: " + object_id_ + " may touch its support; ";
    }
  }

  void removeObject(StageResult& r) {
    allowSupportContact(false, r);
    moveit_msgs::msg::CollisionObject co;
    co.header.frame_id = world_frame_;
    co.id = object_id_;
    co.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    if (!psi_.applyCollisionObject(co)) {
      throw StageFailure("removing " + object_id_ + " failed");
    }
    r.detail = object_id_;
  }

  void moveNamed(const std::string& name, StageResult& r) {
    useOmpl();
    for (int attempt = 0; attempt <= free_plan_retries_; ++attempt) {
      move_group_->setStartStateToCurrentState();
      move_group_->setNamedTarget(name);
      MoveGroup::Plan plan;
      const bool ok = static_cast<bool>(move_group_->plan(plan));
      r.planning_seconds += plan.planning_time_;
      if (ok) {
        if (!execute(plan)) {
          throw StageFailure("execution to '" + name + "' failed");
        }
        return;
      }
    }
    throw StageFailure("planning to '" + name + "' failed");
  }

  // OMPL move; with `followup`, the plan is accepted only if a LIN to `followup` can be planned
  // from its end state (e.g. above the bin -> lower into it).
  void moveFree(const Eigen::Isometry3d& target, const Eigen::Isometry3d* followup,
                StageResult& r) {
    MoveGroup::Plan plan;
    double t = 0.0;
    const bool ok = planFree(target, plan, t, followup);
    r.planning_seconds += t;
    if (!ok) {
      throw StageFailure(followup ? "OMPL planning (with a valid follow-up LIN) failed"
                                  : "OMPL planning failed");
    }
    if (!execute(plan)) {
      throw StageFailure("execution failed");
    }
  }

  void moveLin(const Eigen::Isometry3d& target, StageResult& r) {
    MoveGroup::Plan plan;
    double t = 0.0;
    const bool ok = planLin(target, plan, t, nullptr);
    r.planning_seconds += t;
    if (!ok) {
      throw StageFailure("Pilz LIN planning failed");
    }
    if (!execute(plan)) {
      throw StageFailure("execution failed");
    }
    const Eigen::Isometry3d reached = currentTcp();
    std::ostringstream s;
    s << "tcp error " << (reached.translation() - target.translation()).norm() * 1000.0 << " mm";
    r.detail = s.str();
  }

  // ------------------------------------------------------------------------------------------
  // Recovery: leave the robot safe after a failure.
  // ------------------------------------------------------------------------------------------
  void recover() {
    RCLCPP_WARN(node_->get_logger(), "recovery: open gripper, detach, unfreeze");
    if (move_group_) {
      move_group_->stop();
    }
    StageResult dummy;
    try {
      moveGripper(cfg_.open_width, false, dummy);
      recovery_log_ += "gripper opened; ";
    } catch (const std::exception& e) {
      RCLCPP_ERROR(node_->get_logger(), "recovery: opening the gripper failed: %s", e.what());
    }
    if (gazebo_attached_) {
      try {
        callAttach(detach_client_, dummy);
        gazebo_attached_ = false;
        recovery_log_ += "gazebo " + dummy.detail + "; ";
      } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "recovery: Gazebo detach failed: %s", e.what());
      }
    }
    if (moveit_attached_ && move_group_) {
      move_group_->detachObject(object_id_);
      moveit_attached_ = !waitAttached(false);
      recovery_log_ += moveit_attached_ ? "MoveIt detach FAILED; " : "MoveIt detached; ";
      if (moveit_attached_) {
        RCLCPP_ERROR(node_->get_logger(), "recovery: MoveIt detach of %s failed",
                     object_id_.c_str());
      }
    }
    if (acm_modified_) {
      try {
        allowSupportContact(false, dummy);
        recovery_log_ += "ACM restored; ";
      } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "recovery: ACM restore failed: %s", e.what());
      }
    }
    if (frozen_) {
      try {
        callSetBool(freeze_client_, false, dummy);
        frozen_ = false;
        recovery_log_ += "scene unfrozen; ";
      } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "recovery: unfreeze failed: %s", e.what());
      }
    }
  }

  // ------------------------------------------------------------------------------------------
  // Helpers
  // ------------------------------------------------------------------------------------------
  Eigen::Isometry3d lookup(const std::string& frame) {
    const auto tf = tf_buffer_->lookupTransform(world_frame_, frame, tf2::TimePointZero,
                                                tf2::durationFromSec(tf_timeout_));
    Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
    t.translation() = Eigen::Vector3d(tf.transform.translation.x, tf.transform.translation.y,
                                      tf.transform.translation.z);
    t.linear() = Eigen::Quaterniond(tf.transform.rotation.w, tf.transform.rotation.x,
                                    tf.transform.rotation.y, tf.transform.rotation.z)
                     .toRotationMatrix();
    return t;
  }

  Eigen::Isometry3d currentTcp() {
    const auto state = move_group_->getCurrentState(tf_timeout_);
    if (!state) {
      throw StageFailure("no current robot state");
    }
    return state->getGlobalLinkTransform(tcp_link_);
  }

  moveit::core::RobotStatePtr endState(const MoveGroup::Plan& plan) {
    auto state = move_group_->getCurrentState(tf_timeout_);
    if (!state) {
      throw StageFailure("no current robot state");
    }
    const auto& jt = plan.trajectory_.joint_trajectory;
    if (!jt.points.empty()) {
      state->setVariablePositions(jt.joint_names, jt.points.back().positions);
      state->update();
    }
    return state;
  }

  void useOmpl() {
    move_group_->setPlanningPipelineId(ompl_pipeline_);
    move_group_->setPlannerId(ompl_planner_);
    move_group_->setMaxVelocityScalingFactor(free_velocity_);
    move_group_->setMaxAccelerationScalingFactor(free_acceleration_);
  }

  geometry_msgs::msg::PoseStamped stamped(const Eigen::Isometry3d& t) const {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = world_frame_;
    ps.pose = toMsg(t);
    return ps;
  }

  // OMPL to a joint goal from IK of `target`: the first attempt seeds IK with the current state
  // (keeps the arm configuration, e.g. elbow up), later attempts with random seeds. With
  // `followup`, a LIN to it must also be plannable from the plan's end state.
  bool planFree(const Eigen::Isometry3d& target, MoveGroup::Plan& plan, double& t_plan,
                const Eigen::Isometry3d* followup = nullptr) {
    useOmpl();
    const auto* jmg = move_group_->getRobotModel()->getJointModelGroup(arm_group_);
    for (int attempt = 0; attempt <= free_plan_retries_; ++attempt) {
      auto goal = move_group_->getCurrentState(tf_timeout_);
      if (!goal) {
        return false;
      }
      if (attempt > 0) {
        goal->setToRandomPositions(jmg);
      }
      if (!goal->setFromIK(jmg, target, tcp_link_, ik_timeout_)) {
        RCLCPP_WARN(node_->get_logger(), "IK failed (attempt %d)", attempt);
        continue;
      }
      useOmpl();
      move_group_->setStartStateToCurrentState();
      move_group_->clearPoseTargets();
      move_group_->setJointValueTarget(*goal);
      const bool ok = static_cast<bool>(move_group_->plan(plan));
      t_plan += plan.planning_time_;
      if (!ok) {
        continue;
      }
      if (followup) {
        auto end = endState(plan);
        MoveGroup::Plan check;
        if (!planLin(*followup, check, t_plan, end.get())) {
          RCLCPP_WARN(node_->get_logger(), "follow-up LIN not plannable (attempt %d)", attempt);
          continue;
        }
        useOmpl();
      }
      return true;
    }
    return false;
  }

  bool planLin(const Eigen::Isometry3d& target, MoveGroup::Plan& plan, double& t_plan,
               const moveit::core::RobotState* start) {
    move_group_->setPlanningPipelineId(lin_pipeline_);
    move_group_->setPlannerId(lin_planner_);
    move_group_->setMaxVelocityScalingFactor(lin_velocity_);
    move_group_->setMaxAccelerationScalingFactor(lin_acceleration_);
    if (start) {
      move_group_->setStartState(*start);
    } else {
      move_group_->setStartStateToCurrentState();
    }
    move_group_->clearPoseTargets();
    move_group_->setPoseTarget(stamped(target), tcp_link_);
    const bool ok = static_cast<bool>(move_group_->plan(plan));
    t_plan += plan.planning_time_;
    move_group_->setStartStateToCurrentState();
    return ok;
  }

  bool execute(const MoveGroup::Plan& plan) {
    return static_cast<bool>(move_group_->execute(plan));
  }

  // Sends the gripper to a jaw gap. `grasping`: stalling on the object is success.
  void moveGripper(double gap, bool grasping, StageResult& r) {
    GripperCommand::Goal goal;
    goal.command.position = (cfg_.open_width - gap) / 2.0;
    goal.command.max_effort = gripper_max_effort_;
    if (!gripper_client_->wait_for_action_server(std::chrono::duration<double>(service_timeout_))) {
      throw StageFailure("gripper action server not available");
    }
    auto gh_future = gripper_client_->async_send_goal(goal);
    const auto timeout = std::chrono::duration<double>(gripper_timeout_);
    if (gh_future.wait_for(timeout) != std::future_status::ready || !gh_future.get()) {
      throw StageFailure("gripper goal rejected or not accepted in time");
    }
    auto res_future = gripper_client_->async_get_result(gh_future.get());
    if (res_future.wait_for(timeout) != std::future_status::ready) {
      throw StageFailure("gripper result timeout");
    }
    const auto res = res_future.get();
    std::this_thread::sleep_for(std::chrono::duration<double>(gripper_settle_));
    std::ostringstream s;
    s << "target q " << goal.command.position << " (gap " << gap << "), reached q "
      << res.result->position << (res.result->reached_goal ? " reached" : "")
      << (res.result->stalled ? " stalled" : "");
    r.detail = s.str();
    if (grasping) {
      if (res.code != rclcpp_action::ResultCode::SUCCEEDED && !res.result->stalled) {
        throw StageFailure("gripper close failed: " + r.detail);
      }
    } else if (std::abs(res.result->position - goal.command.position) > gripper_open_tolerance_) {
      throw StageFailure("gripper did not open: " + r.detail);
    }
  }

  void callAttach(const rclcpp::Client<AttachLink>::SharedPtr& client, StageResult& r) {
    auto req = std::make_shared<AttachLink::Request>();
    req->parent_model = attach_parent_model_;
    req->parent_link = attach_parent_link_;
    req->child_model = "";  // nearest model (user decision, Phase 4)
    req->child_link = "";
    auto fut = client->async_send_request(req);
    if (fut.wait_for(std::chrono::duration<double>(service_timeout_)) !=
        std::future_status::ready) {
      throw StageFailure(std::string(client->get_service_name()) + " timeout");
    }
    const auto res = fut.get();
    r.detail = res->message;
    if (!res->success) {
      throw StageFailure(std::string(client->get_service_name()) + ": " + res->message);
    }
  }

  void callSetBool(const rclcpp::Client<SetBool>::SharedPtr& client, bool value, StageResult& r) {
    auto req = std::make_shared<SetBool::Request>();
    req->data = value;
    auto fut = client->async_send_request(req);
    if (fut.wait_for(std::chrono::duration<double>(service_timeout_)) !=
        std::future_status::ready) {
      throw StageFailure(std::string(client->get_service_name()) + " timeout");
    }
    const auto res = fut.get();
    r.detail = res->message;
    if (!res->success) {
      throw StageFailure(std::string(client->get_service_name()) + ": " + res->message);
    }
  }

  bool waitAttached(bool attached) {
    const auto t0 = std::chrono::steady_clock::now();
    while (rclcpp::ok() && secondsSince(t0) < scene_wait_timeout_) {
      const bool now_attached = !psi_.getAttachedObjects({object_id_}).empty();
      if (now_attached == attached) {
        return true;
      }
      std::this_thread::sleep_for(100ms);
    }
    return false;
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<MoveGroup> move_group_;
  moveit::planning_interface::PlanningSceneInterface psi_;
  rclcpp::Subscription<graspsort_msgs::msg::ObjectPoseArray>::SharedPtr objects_sub_;
  std::mutex objects_mutex_;
  graspsort_msgs::msg::ObjectPoseArray::ConstSharedPtr objects_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_client_;
  rclcpp::Client<AttachLink>::SharedPtr attach_client_;
  rclcpp::Client<AttachLink>::SharedPtr detach_client_;
  rclcpp::Client<SetBool>::SharedPtr freeze_client_;
  rclcpp::Client<GetPlanningScene>::SharedPtr scene_client_;

  // Parameters
  std::string target_class_, objects_topic_, world_frame_, base_frame_, arm_group_, tcp_link_,
      attach_link_, ready_state_, object_id_prefix_;
  std::vector<std::string> touch_links_;
  double object_wait_timeout_{30.0}, collision_wait_timeout_{10.0}, scene_wait_timeout_{3.0},
      tf_timeout_{5.0}, freeze_recheck_delay_{0.3};
  std::unique_ptr<gm::BinAssignment> bins_;
  gm::BinGeometry bin_geometry_;
  std::unique_ptr<gm::GraspPlanner> planner_;
  int max_candidates_{3};
  gm::PickPlaceConfig cfg_;
  std::string gripper_action_;
  double gripper_max_effort_{50.0}, gripper_timeout_{10.0}, gripper_settle_{0.3},
      gripper_open_tolerance_{0.003};
  std::string ompl_pipeline_, ompl_planner_, lin_pipeline_, lin_planner_;
  double planning_time_{5.0};
  int planning_attempts_{1}, free_plan_retries_{2};
  double free_velocity_{0.5}, free_acceleration_{0.5}, lin_velocity_{0.1}, lin_acceleration_{0.1},
      goal_position_tolerance_{0.001}, goal_orientation_tolerance_{0.01};
  std::string freeze_service_, attach_service_, detach_service_, attach_parent_model_,
      attach_parent_link_;
  double service_timeout_{10.0};
  std::string scene_service_;
  std::vector<std::string> support_ids_;
  double ik_timeout_{0.1};

  // Run state
  std::vector<StageResult> results_;
  gm::Object target_;
  std::vector<gm::Object> scene_objects_;
  std::string object_id_;
  std::string bin_name_;
  double close_position_{0.0};
  Eigen::Isometry3d grasp_pose_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d release_pose_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d above_bin_pose_{Eigen::Isometry3d::Identity()};
  bool frozen_{false};
  bool gazebo_attached_{false};
  bool moveit_attached_{false};
  bool acm_modified_{false};
  std::string recovery_log_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<rclcpp::Node>("pick_place_test", options);
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
