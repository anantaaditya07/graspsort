// PickPlaceExecutor: ONE pick-and-place of a perceived object (architecture 7.5), shared by
// pick_place_test (Phase 4) and sort_task_node (Phase 5). Needs ROS (MoveGroupInterface).
//
// Steps (sort_logic.hpp pickPlaceSteps): lock_target (object_<id> in the planning scene, freeze
// the scene, re-check the id on /objects_3d) -> open_gripper -> plan_pregrasp (ranked grasp
// candidates; OMPL to the pre-grasp, accepted only if the LIN approach plans from its end) ->
// pregrasp -> approach (Pilz LIN) -> close_gripper (width - squeeze, D-02) -> attach_gazebo ->
// attach_moveit -> lift (LIN) -> move_above_bin (OMPL) -> lower (LIN) -> release_gripper ->
// detach_gazebo -> detach_moveit -> remove_object -> retreat (LIN) -> ready -> unfreeze_scene.
//
// On a failure or cancel request (checked between steps): stop, open the gripper, detach
// (Gazebo, MoveIt), remove a dropped object from the scene, restore the ACM, raise the TCP and go
// to the ready state (parameter), unfreeze. The outcome holds the failing step, a category
// (sort_logic.hpp), per-step times and the total planning time.
//
// The robot never reads Gazebo ground truth; targets come from perception only. D-10: no *_mimic
// joint name is ever sent to MoveIt.
#ifndef GRASPSORT_MANIPULATION__PICK_PLACE_EXECUTOR_HPP_
#define GRASPSORT_MANIPULATION__PICK_PLACE_EXECUTOR_HPP_

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <control_msgs/action/gripper_command.hpp>
#include <functional>
#include <graspsort_msgs/msg/object_pose_array.hpp>
#include <graspsort_msgs/srv/attach_link.hpp>
#include <memory>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <string>
#include <vector>

#include "graspsort_manipulation/bin_assignment.hpp"
#include "graspsort_manipulation/grasp_planner.hpp"
#include "graspsort_manipulation/pick_place_geometry.hpp"
#include "graspsort_manipulation/sort_logic.hpp"

namespace graspsort::manipulation {

struct StepRecord {
  std::string name;
  bool ok{false};
  double seconds{0.0};
  double planning_seconds{0.0};
  std::string detail;
};

struct PickRequest {
  Object target;              // perceived target, world frame
  std::vector<Object> scene;  // all perceived objects (grasp planner neighbours; may hold target)
  std::vector<Object> bin_occupied;  // objects already in the target bin (release slot choice)
  std::size_t first_candidate{0};    // ranked grasp candidates start here (retry k: k)
};

struct PickOutcome {
  bool success{false};
  bool canceled{false};
  bool placed{false};  // the object was released in its bin (detach_gazebo done)
  std::string failed_step;
  FailureCategory category{FailureCategory::kNone};
  std::string message;
  std::string recovery;
  std::vector<StepRecord> steps;
  double planning_time{0.0};
  double total_time{0.0};
  // Time from the start of pickAndPlace to the end of the retreat step (or to the end).
  double to_retreat_time{0.0};
  Object target;  // as locked (refreshed from /objects_3d after the freeze)
  std::string object_id;
  std::string bin;
  Eigen::Vector2d release_xy{Eigen::Vector2d::Zero()};
};

struct ReachCheck {
  bool ok{false};
  FailureCategory category{FailureCategory::kNone};  // kNoGraspCandidate or kUnreachable
  std::string detail;
};

class PickPlaceExecutor {
 public:
  using StepCallback = std::function<void(const std::string& step)>;
  using CancelCheck = std::function<bool()>;
  using ObjectsMsg = graspsort_msgs::msg::ObjectPoseArray;

  // Declares and reads every parameter (config/pick_place.yaml) and creates the clients.
  explicit PickPlaceExecutor(const rclcpp::Node::SharedPtr& node);

  // Creates the MoveGroupInterface and waits for the servers. The node must already spin.
  // Throws std::runtime_error.
  void connect();

  ObjectsMsg::ConstSharedPtr latestObjects() const;
  static Object toObject(const graspsort_msgs::msg::ObjectPose& o);
  std::string objectId(std::uint32_t id) const { return object_id_prefix_ + std::to_string(id); }
  const std::string& objectIdPrefix() const { return object_id_prefix_; }
  const std::string& objectsTopic() const { return objects_topic_; }
  const BinAssignment& binAssignment() const { return *bins_; }
  // Bin geometry (world_layout.yaml size + TF pose bin_<x>). Throws on TF failure.
  BinGeometry binGeometry(const std::string& bin_name);
  Eigen::Isometry3d lookup(const std::string& frame);
  Eigen::Vector3d basePosition() { return lookup(base_frame_).translation(); }

  // Ordering pre-check (7.5 "reachable (IK check)"): the best grasp candidate (after rotating to
  // first_candidate) must pass grasp_planner and have a collision-free IK solution (move_group
  // /compute_ik, gripper open) for its pre-grasp and grasp poses. Checks up to
  // reach_check_candidates candidates.
  ReachCheck checkReachable(const Object& target, const std::vector<Object>& scene,
                            std::size_t first_candidate);

  // Moves the arm to the ready state (OMPL) unless every arm joint is already within `tolerance`
  // rad of it (D-24). Returns true if a move was made. Throws std::runtime_error on failure.
  bool moveToReadyIfAway(double tolerance);

  PickOutcome pickAndPlace(const PickRequest& request, const StepCallback& on_step = {},
                           const CancelCheck& canceled = {});

 private:
  class StepFailure;
  struct Run;

  void loadParameters();
  void step(Run& run, const std::string& name, const std::function<void(StepRecord&)>& fn);
  void lockTarget(Run& run, StepRecord& r);
  void planPregrasp(Run& run, StepRecord& r);
  void attachMoveIt(Run& run, StepRecord& r);
  void detachMoveIt(Run& run, StepRecord& r);
  void allowContact(Run& run, bool allow, StepRecord& r);
  void removeObject(Run& run, const std::string& id, StepRecord& r);
  void moveNamed(const std::string& name, StepRecord& r);
  void moveFree(const Eigen::Isometry3d& target, const Eigen::Isometry3d* followup, StepRecord& r);
  void moveLin(const Eigen::Isometry3d& target, StepRecord& r);
  void recover(Run& run);

  enum class PlanStatus { kOk, kIkFailed, kPlanFailed };
  Eigen::Isometry3d currentTcp();
  moveit::core::RobotStatePtr endState(const moveit::planning_interface::MoveGroupInterface::Plan&);
  void useOmpl();
  geometry_msgs::msg::PoseStamped stamped(const Eigen::Isometry3d& t) const;
  PlanStatus planFree(const Eigen::Isometry3d& target,
                      moveit::planning_interface::MoveGroupInterface::Plan& plan, double& t_plan,
                      const Eigen::Isometry3d* followup = nullptr);
  bool planLin(const Eigen::Isometry3d& target,
               moveit::planning_interface::MoveGroupInterface::Plan& plan, double& t_plan,
               const moveit::core::RobotState* start);
  bool execute(const moveit::planning_interface::MoveGroupInterface::Plan& plan);
  void moveGripper(double gap, bool grasping, StepRecord& r);
  void callAttach(const rclcpp::Client<graspsort_msgs::srv::AttachLink>::SharedPtr& client,
                  StepRecord& r);
  void callSetBool(const rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr& client, bool value,
                   StepRecord& r);
  bool waitAttached(const std::string& id, bool attached);
  // Collision-aware IK via /compute_ik; seed = current state with the gripper open, overridden by
  // `seed` (arm joints only). Returns the arm joint solution.
  std::optional<std::vector<double>> computeIk(const Eigen::Isometry3d& pose,
                                               const std::vector<double>* seed);
  std::optional<double> releaseGap(double width) const;

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  moveit::planning_interface::PlanningSceneInterface psi_;
  rclcpp::CallbackGroup::SharedPtr cb_group_;
  rclcpp::Subscription<ObjectsMsg>::SharedPtr objects_sub_;
  mutable std::mutex objects_mutex_;
  ObjectsMsg::ConstSharedPtr objects_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr gripper_client_;
  rclcpp::Client<graspsort_msgs::srv::AttachLink>::SharedPtr attach_client_;
  rclcpp::Client<graspsort_msgs::srv::AttachLink>::SharedPtr detach_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr freeze_client_;
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr scene_client_;
  rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr ik_client_;

  // Parameters
  std::string objects_topic_, world_frame_, base_frame_, arm_group_, tcp_link_, attach_link_,
      ready_state_, object_id_prefix_;
  std::vector<std::string> touch_links_;
  double collision_wait_timeout_{10.0}, scene_wait_timeout_{3.0}, tf_timeout_{5.0},
      freeze_recheck_delay_{0.3};
  std::unique_ptr<BinAssignment> bins_;
  BinGeometry bin_template_;
  std::unique_ptr<GraspPlanner> planner_;
  int max_candidates_{3};
  int reach_check_candidates_{1};
  PickPlaceConfig cfg_;
  ReleaseSlotConfig slot_cfg_;
  double release_opening_margin_{0.006};
  std::string gripper_action_, gripper_joint_;
  double gripper_max_effort_{50.0}, gripper_timeout_{10.0}, gripper_settle_{0.3},
      gripper_open_tolerance_{0.003};
  std::string ompl_pipeline_, ompl_planner_, lin_pipeline_, lin_planner_;
  double planning_time_{5.0};
  int planning_attempts_{1}, free_plan_retries_{2};
  double free_velocity_{0.5}, free_acceleration_{0.5}, lin_velocity_{0.1}, lin_acceleration_{0.1},
      goal_position_tolerance_{0.001}, goal_orientation_tolerance_{0.01};
  std::string freeze_service_, attach_service_, detach_service_, attach_parent_model_,
      attach_parent_link_, scene_service_, ik_service_;
  double service_timeout_{10.0};
  std::vector<std::string> support_ids_;
  double ik_timeout_{0.1};
  double reach_ik_timeout_{0.5};
  bool recovery_to_ready_{true};
};

}  // namespace graspsort::manipulation

#endif  // GRASPSORT_MANIPULATION__PICK_PLACE_EXECUTOR_HPP_
