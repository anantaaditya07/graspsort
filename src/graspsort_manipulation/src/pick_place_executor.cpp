// PickPlaceExecutor (see the header). The step sequence and motion code come from the Phase 4
// pick_place_test (D-16); this file makes it reusable by sort_task_node.
#include "graspsort_manipulation/pick_place_executor.hpp"

#include <moveit/collision_detection/collision_matrix.h>
#include <moveit/robot_state/robot_state.h>

#include <algorithm>
#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <limits>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace graspsort::manipulation {

using namespace std::chrono_literals;
using MoveGroup = moveit::planning_interface::MoveGroupInterface;
using GripperCommand = control_msgs::action::GripperCommand;
using AttachLink = graspsort_msgs::srv::AttachLink;
using SetBool = std_srvs::srv::SetBool;
using GetPlanningScene = moveit_msgs::srv::GetPlanningScene;
using GetPositionIK = moveit_msgs::srv::GetPositionIK;

namespace {

double secondsSince(std::chrono::steady_clock::time_point t0) {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
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

bool isMimic(const std::string& joint) {
  const std::string suffix = "_mimic";
  return joint.size() >= suffix.size() &&
         joint.compare(joint.size() - suffix.size(), suffix.size(), suffix) == 0;
}

}  // namespace

// A failed step; category kNone = derived from the step name (categoryForStep).
class PickPlaceExecutor::StepFailure : public std::runtime_error {
 public:
  explicit StepFailure(const std::string& what, FailureCategory c = FailureCategory::kNone,
                       bool canceled = false)
      : std::runtime_error(what), category(c), cancel(canceled) {}
  FailureCategory category;
  bool cancel;
};

// State of one pickAndPlace call.
struct PickPlaceExecutor::Run {
  PickRequest req;
  PickOutcome out;
  StepCallback on_step;
  CancelCheck canceled;
  std::string object_id;
  BinGeometry bin;
  double close_position{0.0};
  double release_gap{0.0};
  MoveGroup::Plan pregrasp_plan;
  Eigen::Isometry3d grasp_pose{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d release_pose{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d above_bin_pose{Eigen::Isometry3d::Identity()};
  std::vector<std::string> contact_ids;  // ACM: held object may touch these
  bool frozen{false};
  bool gazebo_attached{false};
  bool moveit_attached{false};
  bool acm_modified{false};
  bool at_release{false};  // lowered to the release pose in the bin
  std::chrono::steady_clock::time_point t0;
};

PickPlaceExecutor::PickPlaceExecutor(const rclcpp::Node::SharedPtr& node) : node_(node) {
  loadParameters();
  cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = cb_group_;
  objects_sub_ = node_->create_subscription<ObjectsMsg>(
      objects_topic_, rclcpp::QoS(10),
      [this](ObjectsMsg::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(objects_mutex_);
        objects_ = msg;
      },
      sub_opts);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  gripper_client_ = rclcpp_action::create_client<GripperCommand>(node_, gripper_action_, cb_group_);
  attach_client_ = node_->create_client<AttachLink>(attach_service_,
                                                    rmw_qos_profile_services_default, cb_group_);
  detach_client_ = node_->create_client<AttachLink>(detach_service_,
                                                    rmw_qos_profile_services_default, cb_group_);
  freeze_client_ =
      node_->create_client<SetBool>(freeze_service_, rmw_qos_profile_services_default, cb_group_);
  scene_client_ = node_->create_client<GetPlanningScene>(
      scene_service_, rmw_qos_profile_services_default, cb_group_);
  ik_client_ =
      node_->create_client<GetPositionIK>(ik_service_, rmw_qos_profile_services_default, cb_group_);
}

// ------------------------------------------------------------------------------------------
// Parameters
// ------------------------------------------------------------------------------------------
void PickPlaceExecutor::loadParameters() {
  auto& n = *node_;
  objects_topic_ = n.declare_parameter<std::string>("objects_topic", "/objects_3d");
  world_frame_ = n.declare_parameter<std::string>("world_frame", "world");
  base_frame_ = n.declare_parameter<std::string>("base_frame", "base_link");
  arm_group_ = n.declare_parameter<std::string>("arm_group", "ur_manipulator");
  tcp_link_ = n.declare_parameter<std::string>("tcp_link", "gripper_tcp");
  attach_link_ = n.declare_parameter<std::string>("attach_link", "gripper_tcp");
  touch_links_ = n.declare_parameter<std::vector<std::string>>(
      "touch_links", {"gripper_base_link", "gripper_left_finger_link", "gripper_right_finger_link",
                      "gripper_tcp"});
  ready_state_ = n.declare_parameter<std::string>("ready_state", "ready");
  object_id_prefix_ = n.declare_parameter<std::string>("object_id_prefix", "object_");
  collision_wait_timeout_ = n.declare_parameter<double>("collision_wait_timeout", 10.0);
  scene_wait_timeout_ = n.declare_parameter<double>("scene_wait_timeout", 3.0);
  freeze_recheck_delay_ = n.declare_parameter<double>("freeze_recheck_delay", 0.3);

  const auto bin_classes =
      n.declare_parameter<std::vector<std::string>>("bin_classes", defaultBinClasses());
  const auto bin_names =
      n.declare_parameter<std::vector<std::string>>("bin_names", defaultBinNames());
  bins_ = std::make_unique<BinAssignment>(BinAssignment::fromLists(bin_classes, bin_names));
  // Bin geometry from graspsort_gazebo/config/world_layout.yaml (passed as a params file).
  const auto bin_size = n.declare_parameter<std::vector<double>>("bins.size", {0.20, 0.20, 0.08});
  if (bin_size.size() != 3) {
    throw std::invalid_argument("bins.size must have 3 values");
  }
  bin_template_.size = {bin_size[0], bin_size[1], bin_size[2]};
  bin_template_.wall_thickness = n.declare_parameter<double>("bins.wall_thickness", 0.01);
  bin_template_.floor_thickness = n.declare_parameter<double>("bins.floor_thickness", 0.01);
  tf_timeout_ = n.declare_parameter<double>("tf_timeout", 5.0);

  GraspPlannerConfig g;
  g.cylinder_yaw_count = static_cast<std::size_t>(
      n.declare_parameter<int>("grasp.cylinder_yaw_count", static_cast<int>(g.cylinder_yaw_count)));
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
  planner_ = std::make_unique<GraspPlanner>(g);
  max_candidates_ = n.declare_parameter<int>("grasp.max_candidates", 3);
  reach_check_candidates_ = n.declare_parameter<int>("grasp.reach_check_candidates", 1);

  cfg_.pregrasp_height =
      n.declare_parameter<double>("motion.pregrasp_height", cfg_.pregrasp_height);
  cfg_.lift_height = n.declare_parameter<double>("motion.lift_height", cfg_.lift_height);
  cfg_.retreat_height = n.declare_parameter<double>("motion.retreat_height", cfg_.retreat_height);
  cfg_.min_fingertip_clearance =
      n.declare_parameter<double>("motion.min_fingertip_clearance", cfg_.min_fingertip_clearance);
  cfg_.max_fingertip_above_center = n.declare_parameter<double>("motion.max_fingertip_above_center",
                                                                cfg_.max_fingertip_above_center);
  cfg_.release_clearance =
      n.declare_parameter<double>("motion.release_clearance", cfg_.release_clearance);
  cfg_.transport_clearance =
      n.declare_parameter<double>("motion.transport_clearance", cfg_.transport_clearance);
  slot_cfg_.grid_step = n.declare_parameter<double>("release_slot.grid_step", slot_cfg_.grid_step);
  slot_cfg_.wall_margin =
      n.declare_parameter<double>("release_slot.wall_margin", slot_cfg_.wall_margin);
  slot_cfg_.clearance_cap =
      n.declare_parameter<double>("release_slot.clearance_cap", slot_cfg_.clearance_cap);
  slot_cfg_.box_max_across_offset = n.declare_parameter<double>(
      "release_slot.box_max_across_offset", slot_cfg_.box_max_across_offset);
  cfg_.tcp_to_fingertip =
      n.declare_parameter<double>("gripper.tcp_to_fingertip", cfg_.tcp_to_fingertip);
  cfg_.open_width = n.declare_parameter<double>("gripper.open_width", cfg_.open_width);
  cfg_.max_close_position =
      n.declare_parameter<double>("gripper.max_close_position", cfg_.max_close_position);
  cfg_.squeeze = n.declare_parameter<double>("gripper.squeeze", cfg_.squeeze);
  validate(cfg_);
  release_opening_margin_ =
      n.declare_parameter<double>("gripper.release_opening_margin", release_opening_margin_);
  gripper_action_ =
      n.declare_parameter<std::string>("gripper.action", "/gripper_controller/gripper_cmd");
  gripper_joint_ = n.declare_parameter<std::string>("gripper.joint", "gripper_left_finger_joint");
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
  goal_position_tolerance_ = n.declare_parameter<double>("planning.goal_position_tolerance", 0.001);
  goal_orientation_tolerance_ =
      n.declare_parameter<double>("planning.goal_orientation_tolerance", 0.01);
  ik_timeout_ = n.declare_parameter<double>("planning.ik_timeout", 0.1);
  reach_ik_timeout_ = n.declare_parameter<double>("planning.reach_ik_timeout", 0.5);

  freeze_service_ = n.declare_parameter<std::string>("services.freeze", "/scene_manager/freeze");
  attach_service_ = n.declare_parameter<std::string>("services.attach", "/attach");
  detach_service_ = n.declare_parameter<std::string>("services.detach", "/detach");
  attach_parent_model_ = n.declare_parameter<std::string>("services.attach_parent_model", "ur");
  attach_parent_link_ =
      n.declare_parameter<std::string>("services.attach_parent_link", "gripper_left_finger_link");
  service_timeout_ = n.declare_parameter<double>("services.timeout", 10.0);
  scene_service_ =
      n.declare_parameter<std::string>("services.get_planning_scene", "/get_planning_scene");
  ik_service_ = n.declare_parameter<std::string>("services.compute_ik", "/compute_ik");
  support_ids_ = n.declare_parameter<std::vector<std::string>>("support_ids", {"table"});
  recovery_to_ready_ = n.declare_parameter<bool>("recovery.move_to_ready", true);
}

// ------------------------------------------------------------------------------------------
// Public helpers
// ------------------------------------------------------------------------------------------
void PickPlaceExecutor::connect() {
  move_group_ = std::make_unique<MoveGroup>(node_, arm_group_);
  move_group_->setPoseReferenceFrame(world_frame_);
  move_group_->setEndEffectorLink(tcp_link_);
  move_group_->setPlanningTime(planning_time_);
  move_group_->setNumPlanningAttempts(planning_attempts_);
  move_group_->setGoalPositionTolerance(goal_position_tolerance_);
  move_group_->setGoalOrientationTolerance(goal_orientation_tolerance_);
  const auto timeout = std::chrono::duration<double>(service_timeout_);
  if (!gripper_client_->wait_for_action_server(timeout)) {
    throw std::runtime_error("gripper action server " + gripper_action_ + " not available");
  }
  const std::vector<rclcpp::ClientBase::SharedPtr> clients = {
      attach_client_, detach_client_, freeze_client_, scene_client_, ik_client_};
  for (const auto& c : clients) {
    if (!c->wait_for_service(timeout)) {
      throw std::runtime_error(std::string("service ") + c->get_service_name() + " not available");
    }
  }
  if (!move_group_->getCurrentState(tf_timeout_)) {
    throw std::runtime_error("no current robot state");
  }
}

PickPlaceExecutor::ObjectsMsg::ConstSharedPtr PickPlaceExecutor::latestObjects() const {
  std::lock_guard<std::mutex> lock(objects_mutex_);
  return objects_;
}

Object PickPlaceExecutor::toObject(const graspsort_msgs::msg::ObjectPose& o) {
  Object out;
  out.id = o.id;
  out.class_name = o.class_name;
  out.center = {o.pose.position.x, o.pose.position.y, o.pose.position.z};
  out.size = {o.size.x, o.size.y, o.size.z};
  out.yaw = yawOf(o.pose.orientation);
  out.shape = o.shape;
  return out;
}

BinGeometry PickPlaceExecutor::binGeometry(const std::string& bin_name) {
  BinGeometry b = bin_template_;
  b.pose = lookup(bin_name);
  return b;
}

Eigen::Isometry3d PickPlaceExecutor::lookup(const std::string& frame) {
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

std::optional<double> PickPlaceExecutor::releaseGap(double width) const {
  if (release_opening_margin_ <= 0.0) {
    return cfg_.open_width;
  }
  return std::min(cfg_.open_width, width + release_opening_margin_);
}

ReachCheck PickPlaceExecutor::checkReachable(const Object& target, const std::vector<Object>& scene,
                                             std::size_t first_candidate) {
  ReachCheck out;
  const Eigen::Isometry3d tcp = currentTcp();
  const double wrist_yaw = closingYaw(Eigen::Quaterniond(tcp.linear()));
  const auto all = planner_->evaluate(target, scene, wrist_yaw);
  const auto cands = rotated(planner_->plan(target, scene, wrist_yaw), first_candidate);
  if (cands.empty()) {
    out.category = FailureCategory::kNoGraspCandidate;
    out.detail = std::string("grasp planner rejected all ") + std::to_string(all.size()) +
                 " candidates (first: " + (all.empty() ? "none" : toString(all.front().rejection)) +
                 ")";
    return out;
  }
  const auto grasp_z =
      adjustedGraspZ(cands.front().position.z(), target.center.z(), target.size.z(), cfg_);
  if (!grasp_z) {
    out.category = FailureCategory::kNoGraspCandidate;
    out.detail = "object too flat for the fingertip clearance rules";
    return out;
  }
  if (!closePosition(cands.front().width, cfg_)) {
    out.category = FailureCategory::kNoGraspCandidate;
    out.detail = "no valid close position for width " + std::to_string(cands.front().width);
    return out;
  }
  std::string log;
  const int n = std::min<int>(std::max(1, reach_check_candidates_), static_cast<int>(cands.size()));
  for (int i = 0; i < n; ++i) {
    const auto& c = cands[static_cast<std::size_t>(i)];
    const auto grasp = makePose(Eigen::Vector3d(c.position.x(), c.position.y(), *grasp_z),
                                topDownOrientation(c.yaw));
    const auto pre_sol = computeIk(raised(grasp, cfg_.pregrasp_height), nullptr);
    if (!pre_sol) {
      log += " cand" + std::to_string(i) + ": no collision-free IK for the pre-grasp;";
      continue;
    }
    if (!computeIk(grasp, &*pre_sol)) {
      log += " cand" + std::to_string(i) + ": no collision-free IK for the grasp;";
      continue;
    }
    out.ok = true;
    out.detail = "candidate " + std::to_string(i) + " reachable;" + log;
    return out;
  }
  out.category = FailureCategory::kUnreachable;
  if (!log.empty() && log.back() == ';') {
    log.pop_back();
  }
  out.detail = log.empty() ? "no reachable candidate" : log.substr(1);
  return out;
}

std::optional<std::vector<double>> PickPlaceExecutor::computeIk(const Eigen::Isometry3d& pose,
                                                                const std::vector<double>* seed) {
  auto req = std::make_shared<GetPositionIK::Request>();
  auto& ik = req->ik_request;
  ik.group_name = arm_group_;
  ik.avoid_collisions = true;
  ik.ik_link_name = tcp_link_;
  ik.pose_stamped = stamped(pose);
  ik.timeout = rclcpp::Duration::from_seconds(reach_ik_timeout_);
  // Seed: the current state (is_diff) with the gripper open; arm joints from `seed`.
  // D-10: only real joints, never *_mimic names.
  ik.robot_state.is_diff = true;
  ik.robot_state.joint_state.name.push_back(gripper_joint_);
  ik.robot_state.joint_state.position.push_back(0.0);
  const auto arm_joints = move_group_->getActiveJoints();
  if (seed && seed->size() == arm_joints.size()) {
    for (std::size_t i = 0; i < arm_joints.size(); ++i) {
      ik.robot_state.joint_state.name.push_back(arm_joints[i]);
      ik.robot_state.joint_state.position.push_back((*seed)[i]);
    }
  }
  auto fut = ik_client_->async_send_request(req);
  if (fut.wait_for(std::chrono::duration<double>(service_timeout_ + reach_ik_timeout_)) !=
      std::future_status::ready) {
    RCLCPP_WARN(node_->get_logger(), "%s timeout", ik_service_.c_str());
    return std::nullopt;
  }
  const auto res = fut.get();
  if (res->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    return std::nullopt;
  }
  std::vector<double> sol;
  const auto& js = res->solution.joint_state;
  for (const auto& j : arm_joints) {
    const auto it = std::find(js.name.begin(), js.name.end(), j);
    if (it == js.name.end() || isMimic(j)) {
      return std::nullopt;
    }
    sol.push_back(js.position[static_cast<std::size_t>(it - js.name.begin())]);
  }
  return sol;
}

// ------------------------------------------------------------------------------------------
// pickAndPlace
// ------------------------------------------------------------------------------------------
PickOutcome PickPlaceExecutor::pickAndPlace(const PickRequest& request, const StepCallback& on_step,
                                            const CancelCheck& canceled) {
  Run run;
  run.req = request;
  run.on_step = on_step;
  run.canceled = canceled;
  run.t0 = std::chrono::steady_clock::now();
  run.out.target = request.target;
  run.object_id = objectId(request.target.id);
  run.out.object_id = run.object_id;
  try {
    if (!move_group_) {
      throw StepFailure("not connected", FailureCategory::kOther);
    }
    const auto bin = bins_->binFor(request.target.class_name);
    if (!bin) {
      throw StepFailure("no bin for class '" + request.target.class_name + "'",
                        FailureCategory::kOther);
    }
    run.out.bin = *bin;
    step(run, "lock_target", [&](StepRecord& r) { lockTarget(run, r); });
    step(run, "open_gripper", [&](StepRecord& r) { moveGripper(cfg_.open_width, false, r); });
    step(run, "plan_pregrasp", [&](StepRecord& r) { planPregrasp(run, r); });
    step(run, "pregrasp", [&](StepRecord& r) {
      if (!execute(run.pregrasp_plan)) {
        throw StepFailure("pre-grasp execution failed", FailureCategory::kExecutionFailed);
      }
      r.detail = "executed";
    });
    step(run, "approach", [&](StepRecord& r) { moveLin(run.grasp_pose, r); });
    step(run, "close_gripper",
         [&](StepRecord& r) { moveGripper(cfg_.open_width - 2.0 * run.close_position, true, r); });
    step(run, "attach_gazebo", [&](StepRecord& r) {
      callAttach(attach_client_, r);
      run.gazebo_attached = true;
    });
    step(run, "attach_moveit", [&](StepRecord& r) {
      allowContact(run, true, r);
      attachMoveIt(run, r);
    });
    step(run, "lift", [&](StepRecord& r) { moveLin(raised(run.grasp_pose, cfg_.lift_height), r); });
    step(run, "move_above_bin",
         [&](StepRecord& r) { moveFree(run.above_bin_pose, &run.release_pose, r); });
    step(run, "lower", [&](StepRecord& r) {
      moveLin(run.release_pose, r);
      run.at_release = true;
    });
    step(run, "release_gripper", [&](StepRecord& r) { moveGripper(run.release_gap, false, r); });
    step(run, "detach_gazebo", [&](StepRecord& r) {
      callAttach(detach_client_, r);
      run.gazebo_attached = false;
      run.out.placed = true;
    });
    step(run, "detach_moveit", [&](StepRecord& r) { detachMoveIt(run, r); });
    step(run, "remove_object", [&](StepRecord& r) {
      allowContact(run, false, r);
      removeObject(run, run.object_id, r);
    });
    step(run, "retreat",
         [&](StepRecord& r) { moveLin(raised(run.release_pose, cfg_.retreat_height), r); });
    run.out.to_retreat_time = secondsSince(run.t0);
    step(run, "ready", [&](StepRecord& r) { moveNamed(ready_state_, r); });
    step(run, "unfreeze_scene", [&](StepRecord& r) {
      callSetBool(freeze_client_, false, r);
      run.frozen = false;
    });
    run.out.success = true;
  } catch (const StepFailure& e) {
    run.out.failed_step = run.out.steps.empty() ? "start" : run.out.steps.back().name;
    if (!run.out.steps.empty() && run.out.steps.back().ok) {
      run.out.failed_step = "after " + run.out.steps.back().name;  // e.g. canceled
    }
    run.out.canceled = e.cancel;
    run.out.category =
        e.category != FailureCategory::kNone ? e.category : categoryForStep(run.out.failed_step);
    run.out.message = e.what();
    if (e.cancel) {
      RCLCPP_WARN(node_->get_logger(), "canceled (%s)", run.out.failed_step.c_str());
    } else {
      RCLCPP_ERROR(node_->get_logger(), "step '%s' FAILED (%s): %s", run.out.failed_step.c_str(),
                   toString(run.out.category), e.what());
    }
    recover(run);
    RCLCPP_WARN(node_->get_logger(), "recovery: %s", run.out.recovery.c_str());
  }
  run.out.total_time = secondsSince(run.t0);
  if (run.out.to_retreat_time <= 0.0) {
    run.out.to_retreat_time = run.out.total_time;
  }
  for (const auto& s : run.out.steps) {
    run.out.planning_time += s.planning_seconds;
  }
  return run.out;
}

void PickPlaceExecutor::step(Run& run, const std::string& name,
                             const std::function<void(StepRecord&)>& fn) {
  if (run.canceled && run.canceled()) {
    throw StepFailure("canceled", FailureCategory::kOther, true);
  }
  if (run.on_step) {
    run.on_step(name);
  }
  run.out.steps.push_back(StepRecord{name, false, 0.0, 0.0, ""});
  RCLCPP_INFO(node_->get_logger(), "--> %s", name.c_str());
  const auto t0 = std::chrono::steady_clock::now();
  try {
    fn(run.out.steps.back());
  } catch (const StepFailure& e) {
    run.out.steps.back().seconds = secondsSince(t0);
    run.out.steps.back().detail = e.what();
    throw;
  } catch (const std::exception& e) {
    run.out.steps.back().seconds = secondsSince(t0);
    run.out.steps.back().detail = e.what();
    throw StepFailure(e.what());
  }
  auto& rec = run.out.steps.back();
  rec.ok = true;
  rec.seconds = secondsSince(t0);
  RCLCPP_INFO(node_->get_logger(), "<-- %s ok (%.2f s) %s", name.c_str(), rec.seconds,
              rec.detail.c_str());
}

// Waits for object_<id> in the planning scene, freezes the scene and re-reads /objects_3d: the
// target is accepted only if, after the freeze, the latest /objects_3d still holds its id (a
// moved object gets a new track id) and object_<id> is in the scene. The target is refreshed
// from that message.
void PickPlaceExecutor::lockTarget(Run& run, StepRecord& r) {
  const auto t0 = std::chrono::steady_clock::now();
  while (psi_.getObjects({run.object_id}).empty()) {
    if (secondsSince(t0) > collision_wait_timeout_) {
      throw StepFailure(run.object_id + " not in the planning scene after " +
                            std::to_string(collision_wait_timeout_) + " s",
                        FailureCategory::kMisdetection);
    }
    std::this_thread::sleep_for(200ms);
  }
  StepRecord f;
  callSetBool(freeze_client_, true, f);
  run.frozen = true;
  std::this_thread::sleep_for(std::chrono::duration<double>(freeze_recheck_delay_));
  const auto msg = latestObjects();
  std::optional<Object> now;
  if (msg) {
    for (const auto& o : msg->objects) {
      if (o.id == run.req.target.id) {
        now = toObject(o);
      }
    }
  }
  if (!now) {
    throw StepFailure("target id " + std::to_string(run.req.target.id) + " no longer on " +
                          objects_topic_ + " after the freeze",
                      FailureCategory::kMisdetection);
  }
  if (psi_.getObjects({run.object_id}).empty()) {
    throw StepFailure(run.object_id + " left the planning scene at the freeze",
                      FailureCategory::kMisdetection);
  }
  run.req.target = *now;
  run.out.target = *now;
  for (auto& o : run.req.scene) {
    if (o.id == now->id) {
      o = *now;
    }
  }
  std::ostringstream s;
  s << run.object_id << " (" << now->class_name << ") at (" << now->center.x() << ", "
    << now->center.y() << ", " << now->center.z() << ") size (" << now->size.x() << ", "
    << now->size.y() << ", " << now->size.z() << ") yaw " << now->yaw << "; bin " << run.out.bin
    << "; frozen";
  r.detail = s.str();
}

// Plans pre-grasp (OMPL) and approach (LIN, from the planned pre-grasp end state) for the ranked
// candidates (rotated to first_candidate), up to max_candidates; keeps the first plannable one.
void PickPlaceExecutor::planPregrasp(Run& run, StepRecord& r) {
  const auto& target = run.req.target;
  const Eigen::Isometry3d tcp = currentTcp();
  const double wrist_yaw = closingYaw(Eigen::Quaterniond(tcp.linear()));
  const auto cands =
      rotated(planner_->plan(target, run.req.scene, wrist_yaw), run.req.first_candidate);
  if (cands.empty()) {
    throw StepFailure("grasp planner rejected every candidate", FailureCategory::kNoGraspCandidate);
  }
  const auto grasp_z =
      adjustedGraspZ(cands.front().position.z(), target.center.z(), target.size.z(), cfg_);
  if (!grasp_z) {
    throw StepFailure("object too flat for the fingertip clearance rules",
                      FailureCategory::kNoGraspCandidate);
  }
  const auto q_close = closePosition(cands.front().width, cfg_);
  if (!q_close) {
    throw StepFailure("no valid close position for width " + std::to_string(cands.front().width),
                      FailureCategory::kNoGraspCandidate);
  }
  run.close_position = *q_close;
  run.release_gap = *releaseGap(cands.front().width);
  run.bin = binGeometry(run.out.bin);
  const double hang = hangBelowTcp(*grasp_z, target.center.z(), target.size.z());

  std::string log;
  int ik_failures = 0;
  int slot_failures = 0;
  const int n = std::min<int>(max_candidates_, static_cast<int>(cands.size()));
  for (int i = 0; i < n; ++i) {
    const auto& c = cands[static_cast<std::size_t>(i)];
    const std::string tag = " cand" + std::to_string(i) + " (yaw " + std::to_string(c.yaw) + ")";
    const auto slot = chooseReleaseSlot(
        run.bin, target, c.yaw, run.release_gap, planner_->config().finger_thickness,
        planner_->config().finger_width, run.req.bin_occupied, slot_cfg_);
    if (!slot) {
      ++slot_failures;
      log += tag + ": no release slot in " + run.out.bin + ";";
      continue;
    }
    const auto q = topDownOrientation(c.yaw);
    const auto grasp = makePose(Eigen::Vector3d(c.position.x(), c.position.y(), *grasp_z), q);
    const auto pre = raised(grasp, cfg_.pregrasp_height);
    BinGeometry at_slot = run.bin;
    at_slot.pose.translation().x() = slot->xy.x();
    at_slot.pose.translation().y() = slot->xy.y();
    double t_plan = 0.0;
    const auto status = planFree(pre, run.pregrasp_plan, t_plan, &grasp);
    r.planning_seconds += t_plan;
    if (status != PlanStatus::kOk) {
      ik_failures += status == PlanStatus::kIkFailed ? 1 : 0;
      log += tag + (status == PlanStatus::kIkFailed ? ": no IK for the pre-grasp;"
                                                    : ": pre-grasp or approach not plannable;");
      continue;
    }
    run.grasp_pose = grasp;
    run.release_pose = releasePose(at_slot, hang, q, cfg_);
    run.above_bin_pose = aboveBinPose(at_slot, hang, q, cfg_);
    run.out.release_xy = slot->xy;
    std::ostringstream s;
    s << "candidate " << i << " (rank " << (run.req.first_candidate + static_cast<std::size_t>(i))
      << ") yaw " << c.yaw << " grasp z " << *grasp_z << " width " << c.width << " close q "
      << run.close_position << " release slot (" << slot->xy.x() << ", " << slot->xy.y()
      << ") clearance " << slot->clearance << ";" << log;
    r.detail = s.str();
    return;
  }
  FailureCategory cat = FailureCategory::kPlanFailed;
  if (ik_failures == n) {
    cat = FailureCategory::kNoIk;
  } else if (slot_failures == n) {
    cat = FailureCategory::kOther;
  }
  throw StepFailure("no candidate plannable:" + log, cat);
}

void PickPlaceExecutor::attachMoveIt(Run& run, StepRecord& r) {
  if (!move_group_->attachObject(run.object_id, attach_link_, touch_links_)) {
    throw StepFailure("attachObject(" + run.object_id + ") failed");
  }
  run.moveit_attached = true;
  if (!waitAttached(run.object_id, true)) {
    throw StepFailure(run.object_id + " not attached in the planning scene");
  }
  r.detail += run.object_id + " -> " + attach_link_;
}

void PickPlaceExecutor::detachMoveIt(Run& run, StepRecord& r) {
  if (!move_group_->detachObject(run.object_id)) {
    throw StepFailure("detachObject(" + run.object_id + ") failed", FailureCategory::kOther);
  }
  if (!waitAttached(run.object_id, false)) {
    throw StepFailure(run.object_id + " still attached in the planning scene",
                      FailureCategory::kOther);
  }
  run.moveit_attached = false;
  r.detail = run.object_id;
}

// While the object is held, MoveIt may not see it touching its support (the perceived box can
// reach a few mm into the table): allow object_<id> vs support_ids in the ACM (D-16); removed
// again with the object. The ACM is replaced as a whole in a diff, so read it first.
void PickPlaceExecutor::allowContact(Run& run, bool allow, StepRecord& r) {
  if (support_ids_.empty()) {
    return;
  }
  auto req = std::make_shared<GetPlanningScene::Request>();
  req->components.components = moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
  auto fut = scene_client_->async_send_request(req);
  if (fut.wait_for(std::chrono::duration<double>(service_timeout_)) != std::future_status::ready) {
    throw StepFailure("get_planning_scene timeout", FailureCategory::kOther);
  }
  collision_detection::AllowedCollisionMatrix acm(fut.get()->scene.allowed_collision_matrix);
  if (allow) {
    for (const auto& id : support_ids_) {
      acm.setEntry(run.object_id, id, true);
    }
  } else {
    acm.removeEntry(run.object_id);
  }
  moveit_msgs::msg::PlanningScene ps;
  ps.is_diff = true;
  acm.getMessage(ps.allowed_collision_matrix);
  if (!psi_.applyPlanningScene(ps)) {
    throw StepFailure("applying the ACM failed", FailureCategory::kOther);
  }
  run.acm_modified = allow;
  if (allow) {
    r.detail += "ACM: " + run.object_id + " may touch its support; ";
  }
}

void PickPlaceExecutor::removeObject(Run& /*run*/, const std::string& id, StepRecord& r) {
  moveit_msgs::msg::CollisionObject co;
  co.header.frame_id = world_frame_;
  co.id = id;
  co.operation = moveit_msgs::msg::CollisionObject::REMOVE;
  if (!psi_.applyCollisionObject(co)) {
    throw StepFailure("removing " + id + " failed", FailureCategory::kOther);
  }
  r.detail = id;
}

bool PickPlaceExecutor::moveToReadyIfAway(double tolerance) {
  const auto joints = move_group_->getActiveJoints();
  const auto named = move_group_->getNamedTargetValues(ready_state_);
  std::vector<double> target;
  for (const auto& j : joints) {
    const auto it = named.find(j);
    if (it == named.end()) {
      throw std::runtime_error("ready state '" + ready_state_ + "' has no value for " + j);
    }
    target.push_back(it->second);
  }
  if (jointsWithin(move_group_->getCurrentJointValues(), target, tolerance)) {
    return false;
  }
  StepRecord r;
  moveNamed(ready_state_, r);  // throws StepFailure (a std::runtime_error)
  return true;
}

void PickPlaceExecutor::moveNamed(const std::string& name, StepRecord& r) {
  useOmpl();
  for (int attempt = 0; attempt <= free_plan_retries_; ++attempt) {
    move_group_->setStartStateToCurrentState();
    move_group_->setNamedTarget(name);
    MoveGroup::Plan plan;
    const bool ok = static_cast<bool>(move_group_->plan(plan));
    r.planning_seconds += plan.planning_time_;
    if (ok) {
      if (!execute(plan)) {
        throw StepFailure("execution to '" + name + "' failed", FailureCategory::kExecutionFailed);
      }
      return;
    }
  }
  throw StepFailure("planning to '" + name + "' failed", FailureCategory::kPlanFailed);
}

// OMPL move; with `followup`, the plan is accepted only if a LIN to `followup` can be planned
// from its end state (e.g. above the bin -> lower into it).
void PickPlaceExecutor::moveFree(const Eigen::Isometry3d& target, const Eigen::Isometry3d* followup,
                                 StepRecord& r) {
  MoveGroup::Plan plan;
  double t = 0.0;
  const auto status = planFree(target, plan, t, followup);
  r.planning_seconds += t;
  if (status == PlanStatus::kIkFailed) {
    throw StepFailure("no IK solution for the goal pose", FailureCategory::kNoIk);
  }
  if (status != PlanStatus::kOk) {
    throw StepFailure(
        followup ? "OMPL planning (with a valid follow-up LIN) failed" : "OMPL planning failed",
        FailureCategory::kPlanFailed);
  }
  if (!execute(plan)) {
    throw StepFailure("execution failed", FailureCategory::kExecutionFailed);
  }
}

void PickPlaceExecutor::moveLin(const Eigen::Isometry3d& target, StepRecord& r) {
  MoveGroup::Plan plan;
  double t = 0.0;
  const bool ok = planLin(target, plan, t, nullptr);
  r.planning_seconds += t;
  if (!ok) {
    throw StepFailure("Pilz LIN planning failed", FailureCategory::kPlanFailed);
  }
  if (!execute(plan)) {
    throw StepFailure("execution failed", FailureCategory::kExecutionFailed);
  }
  const Eigen::Isometry3d reached = currentTcp();
  std::ostringstream s;
  s << "tcp error " << (reached.translation() - target.translation()).norm() * 1000.0 << " mm";
  r.detail = s.str();
}

// ------------------------------------------------------------------------------------------
// Recovery: leave the robot safe after a failure or cancel.
// ------------------------------------------------------------------------------------------
void PickPlaceExecutor::recover(Run& run) {
  RCLCPP_WARN(node_->get_logger(), "recovery: open gripper, detach, unfreeze");
  std::string& log = run.out.recovery;
  if (!move_group_) {
    return;
  }
  move_group_->stop();
  StepRecord dummy;
  bool opened = false;
  try {
    moveGripper(cfg_.open_width, false, dummy);
    opened = true;
    log += "gripper opened; ";
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "recovery: opening the gripper failed: %s", e.what());
    log += "gripper open FAILED; ";
  }
  if (run.gazebo_attached) {
    try {
      callAttach(detach_client_, dummy);
      run.gazebo_attached = false;
      log += "gazebo " + dummy.detail + "; ";
    } catch (const std::exception& e) {
      RCLCPP_ERROR(node_->get_logger(), "recovery: Gazebo detach failed: %s", e.what());
      log += "gazebo detach FAILED; ";
    }
  }
  if (run.at_release && opened && !run.gazebo_attached && !run.out.placed) {
    // The failure came after lowering into the bin: opening there releases the object at the
    // release pose, i.e. it is placed.
    run.out.placed = true;
    log += "released at the release pose (placed); ";
  }
  bool dropped = false;
  if (run.moveit_attached) {
    move_group_->detachObject(run.object_id);
    run.moveit_attached = !waitAttached(run.object_id, false);
    log += run.moveit_attached ? "MoveIt detach FAILED; " : "MoveIt detached; ";
    dropped = !run.moveit_attached;
  }
  if (run.acm_modified) {
    try {
      allowContact(run, false, dummy);
      log += "ACM restored; ";
    } catch (const std::exception& e) {
      RCLCPP_ERROR(node_->get_logger(), "recovery: ACM restore failed: %s", e.what());
    }
  }
  if (dropped) {
    // The detached object sits at the gripper in the scene; perception re-adds it where it
    // really lies after the unfreeze.
    try {
      removeObject(run, run.object_id, dummy);
      log += "dropped " + run.object_id + " removed from the scene; ";
    } catch (const std::exception& e) {
      RCLCPP_ERROR(node_->get_logger(), "recovery: remove failed: %s", e.what());
    }
  }
  if (recovery_to_ready_) {
    // Raise the TCP first if possible (e.g. fingers around the object), then go to ready.
    try {
      moveLin(raised(currentTcp(), cfg_.retreat_height), dummy);
      log += "raised; ";
    } catch (const std::exception&) {
      log += "raise skipped; ";
    }
    try {
      moveNamed(ready_state_, dummy);
      log += "at " + ready_state_ + "; ";
    } catch (const std::exception& e) {
      RCLCPP_ERROR(node_->get_logger(), "recovery: move to %s failed: %s", ready_state_.c_str(),
                   e.what());
      log += "move to " + ready_state_ + " FAILED; ";
    }
  }
  if (run.frozen) {
    try {
      callSetBool(freeze_client_, false, dummy);
      run.frozen = false;
      log += "scene unfrozen; ";
    } catch (const std::exception& e) {
      RCLCPP_ERROR(node_->get_logger(), "recovery: unfreeze failed: %s", e.what());
      log += "unfreeze FAILED; ";
    }
  }
}

// ------------------------------------------------------------------------------------------
// Motion helpers
// ------------------------------------------------------------------------------------------
Eigen::Isometry3d PickPlaceExecutor::currentTcp() {
  const auto state = move_group_->getCurrentState(tf_timeout_);
  if (!state) {
    throw StepFailure("no current robot state", FailureCategory::kOther);
  }
  return state->getGlobalLinkTransform(tcp_link_);
}

moveit::core::RobotStatePtr PickPlaceExecutor::endState(const MoveGroup::Plan& plan) {
  auto state = move_group_->getCurrentState(tf_timeout_);
  if (!state) {
    throw StepFailure("no current robot state", FailureCategory::kOther);
  }
  const auto& jt = plan.trajectory_.joint_trajectory;
  if (!jt.points.empty()) {
    state->setVariablePositions(jt.joint_names, jt.points.back().positions);
    state->update();
  }
  return state;
}

void PickPlaceExecutor::useOmpl() {
  move_group_->setPlanningPipelineId(ompl_pipeline_);
  move_group_->setPlannerId(ompl_planner_);
  move_group_->setMaxVelocityScalingFactor(free_velocity_);
  move_group_->setMaxAccelerationScalingFactor(free_acceleration_);
}

geometry_msgs::msg::PoseStamped PickPlaceExecutor::stamped(const Eigen::Isometry3d& t) const {
  geometry_msgs::msg::PoseStamped ps;
  ps.header.frame_id = world_frame_;
  ps.pose = toMsg(t);
  return ps;
}

// OMPL to a joint goal from IK of `target` (D-16): the first attempt seeds IK with the current
// state (keeps the arm configuration), later attempts with random seeds. With `followup`, a LIN
// to it must also be plannable from the plan's end state.
PickPlaceExecutor::PlanStatus PickPlaceExecutor::planFree(const Eigen::Isometry3d& target,
                                                          MoveGroup::Plan& plan, double& t_plan,
                                                          const Eigen::Isometry3d* followup) {
  useOmpl();
  const auto* jmg = move_group_->getRobotModel()->getJointModelGroup(arm_group_);
  bool any_ik = false;
  for (int attempt = 0; attempt <= free_plan_retries_; ++attempt) {
    auto goal = move_group_->getCurrentState(tf_timeout_);
    if (!goal) {
      return PlanStatus::kPlanFailed;
    }
    if (attempt > 0) {
      goal->setToRandomPositions(jmg);
    }
    if (!goal->setFromIK(jmg, target, tcp_link_, ik_timeout_)) {
      RCLCPP_WARN(node_->get_logger(), "IK failed (attempt %d)", attempt);
      continue;
    }
    any_ik = true;
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
    return PlanStatus::kOk;
  }
  return any_ik ? PlanStatus::kPlanFailed : PlanStatus::kIkFailed;
}

bool PickPlaceExecutor::planLin(const Eigen::Isometry3d& target, MoveGroup::Plan& plan,
                                double& t_plan, const moveit::core::RobotState* start) {
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

bool PickPlaceExecutor::execute(const MoveGroup::Plan& plan) {
  return static_cast<bool>(move_group_->execute(plan));
}

// Sends the gripper to a jaw gap. `grasping`: stalling on the object is success.
void PickPlaceExecutor::moveGripper(double gap, bool grasping, StepRecord& r) {
  GripperCommand::Goal goal;
  goal.command.position = (cfg_.open_width - gap) / 2.0;
  goal.command.max_effort = gripper_max_effort_;
  if (!gripper_client_->wait_for_action_server(std::chrono::duration<double>(service_timeout_))) {
    throw StepFailure("gripper action server not available", FailureCategory::kOther);
  }
  auto gh_future = gripper_client_->async_send_goal(goal);
  const auto timeout = std::chrono::duration<double>(gripper_timeout_);
  if (gh_future.wait_for(timeout) != std::future_status::ready || !gh_future.get()) {
    throw StepFailure("gripper goal rejected or not accepted in time");
  }
  auto res_future = gripper_client_->async_get_result(gh_future.get());
  if (res_future.wait_for(timeout) != std::future_status::ready) {
    throw StepFailure("gripper result timeout");
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
      throw StepFailure("gripper close failed: " + r.detail);
    }
  } else if (std::abs(res.result->position - goal.command.position) > gripper_open_tolerance_) {
    throw StepFailure("gripper did not open: " + r.detail);
  }
}

void PickPlaceExecutor::callAttach(const rclcpp::Client<AttachLink>::SharedPtr& client,
                                   StepRecord& r) {
  auto req = std::make_shared<AttachLink::Request>();
  req->parent_model = attach_parent_model_;
  req->parent_link = attach_parent_link_;
  req->child_model = "";  // nearest model (D-15)
  req->child_link = "";
  auto fut = client->async_send_request(req);
  if (fut.wait_for(std::chrono::duration<double>(service_timeout_)) != std::future_status::ready) {
    throw StepFailure(std::string(client->get_service_name()) + " timeout");
  }
  const auto res = fut.get();
  r.detail = res->message;
  if (!res->success) {
    throw StepFailure(std::string(client->get_service_name()) + ": " + res->message);
  }
}

void PickPlaceExecutor::callSetBool(const rclcpp::Client<SetBool>::SharedPtr& client, bool value,
                                    StepRecord& r) {
  auto req = std::make_shared<SetBool::Request>();
  req->data = value;
  auto fut = client->async_send_request(req);
  if (fut.wait_for(std::chrono::duration<double>(service_timeout_)) != std::future_status::ready) {
    throw StepFailure(std::string(client->get_service_name()) + " timeout",
                      FailureCategory::kOther);
  }
  const auto res = fut.get();
  r.detail = res->message;
  if (!res->success) {
    throw StepFailure(std::string(client->get_service_name()) + ": " + res->message,
                      FailureCategory::kOther);
  }
}

bool PickPlaceExecutor::waitAttached(const std::string& id, bool attached) {
  const auto t0 = std::chrono::steady_clock::now();
  while (rclcpp::ok() && secondsSince(t0) < scene_wait_timeout_) {
    const bool now_attached = !psi_.getAttachedObjects({id}).empty();
    if (now_attached == attached) {
      return true;
    }
    std::this_thread::sleep_for(100ms);
  }
  return false;
}

}  // namespace graspsort::manipulation
