// scene_manager_node (architecture 7.3): keeps the MoveIt planning scene in step with the world.
//
// - At start-up (and whenever move_group lost them) it adds the fixed collision objects from
//   world_layout.yaml (passed with --params-file): "table" (top), "pedestal" and one object per
//   bin (id = bin name, floor + 4 walls).
// - Perceived objects from /objects_3d become "object_<id>" cylinders or boxes. They are added,
//   updated when they moved (update_distance / update_yaw) and removed when they have not been
//   perceived for remove_timeout s. The world file's object poses are never read.
// - Perceived objects whose centre is over a bin (outer footprint + bin_exclusion_margin) are
//   already sorted and are ignored, so they never block placing into that bin (D-23).
// - Objects attached to the robot are never added, modified or removed.
// - ~/freeze (std_srvs/SetBool, true = freeze): while frozen /objects_3d is ignored completely.
//   The freeze response is sent only after any running sync step has finished, so no scene
//   change from perception happens after a successful freeze call.
//
// Decisions are made by the ROS-free graspsort::scene::SceneTracker (scene_logic.hpp). Each
// sync step reads the world object ids and the attached object ids in one /get_planning_scene
// call and applies the changes synchronously with PlanningSceneInterface. If the read fails the
// step is skipped (never act on an unknown attached set).
//
// Subscribes:  objects_3d (graspsort_msgs/ObjectPoseArray)
// Services:    ~/freeze (std_srvs/SetBool)
// Clients:     get_planning_scene (moveit_msgs/GetPlanningScene), apply_planning_scene (via PSI)
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <chrono>
#include <cmath>
#include <future>
#include <geometry_msgs/msg/pose.hpp>
#include <graspsort_msgs/msg/object_pose_array.hpp>
#include <graspsort_scene/scene_logic.hpp>
#include <memory>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <set>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <stdexcept>
#include <string>
#include <vector>

namespace graspsort_scene {

namespace gs = graspsort::scene;
using moveit_msgs::msg::CollisionObject;
using shape_msgs::msg::SolidPrimitive;

namespace {

std::array<double, 3> toArray3(const std::vector<double>& v, const std::string& name) {
  if (v.size() != 3) throw std::invalid_argument(name + " must have 3 elements");
  return {v[0], v[1], v[2]};
}

geometry_msgs::msg::Pose toPose(double x, double y, double z, double yaw) {
  geometry_msgs::msg::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.position.z = z;
  const auto q = gs::yawToQuaternion(yaw);
  p.orientation.x = q[0];
  p.orientation.y = q[1];
  p.orientation.z = q[2];
  p.orientation.w = q[3];
  return p;
}

SolidPrimitive boxPrimitive(const gs::Box& b, geometry_msgs::msg::Pose* pose) {
  SolidPrimitive prim;
  prim.type = SolidPrimitive::BOX;
  prim.dimensions = {b.size[0], b.size[1], b.size[2]};
  *pose = toPose(b.center[0], b.center[1], b.center[2], 0.0);
  return prim;
}

// Fixed object made of boxes given in its own frame, placed at `frame` in the world.
CollisionObject fixedObject(const std::string& id, const std::string& world_frame,
                            const gs::Pose2D& frame, const std::vector<gs::Box>& boxes) {
  CollisionObject co;
  co.id = id;
  co.header.frame_id = world_frame;
  co.pose = toPose(frame.x, frame.y, frame.z, frame.yaw);
  for (const auto& b : boxes) {
    geometry_msgs::msg::Pose sub;
    co.primitives.push_back(boxPrimitive(b, &sub));
    co.primitive_poses.push_back(sub);
  }
  co.operation = CollisionObject::ADD;
  return co;
}

double yawOf(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}  // namespace

class SceneManagerNode : public rclcpp::Node {
 public:
  SceneManagerNode() : rclcpp::Node("scene_manager") {
    // Fixed geometry (world_layout.yaml, '/**' parameters).
    world_frame_ = declare_parameter<std::string>("world_frame", "world");
    declareFixedObjects();

    // Tunables (config/scene.yaml).
    gs::TrackerParams tp;
    tp.update_distance = declare_parameter<double>("update_distance", tp.update_distance);
    tp.update_yaw = declare_parameter<double>("update_yaw", tp.update_yaw);
    tp.remove_timeout = declare_parameter<double>("remove_timeout", tp.remove_timeout);
    if (auto err = gs::checkParams(tp)) throw std::invalid_argument(*err);
    padding_ = declare_parameter<double>("object_padding", 0.0);
    if (padding_ < 0.0) throw std::invalid_argument("object_padding must be >= 0");
    const double sync_period = declare_parameter<double>("sync_period", 0.2);
    if (!(sync_period > 0.0)) throw std::invalid_argument("sync_period must be > 0");
    scene_timeout_ = declare_parameter<double>("scene_service_timeout", 2.0);
    if (!(scene_timeout_ > 0.0)) throw std::invalid_argument("scene_service_timeout must be > 0");
    bin_margin_ = declare_parameter<double>("bin_exclusion_margin", 0.02);
    const auto objects_topic = declare_parameter<std::string>("objects_topic", "objects_3d");
    const auto qos_depth = declare_parameter<std::int64_t>("objects_qos_depth", 1);
    if (qos_depth < 1) throw std::invalid_argument("objects_qos_depth must be >= 1");
    tracker_ = std::make_unique<gs::SceneTracker>(tp);

    // The planning-scene client runs in its own group so the sync timer can wait on it while
    // the subscription, timer and freeze service share one mutually exclusive group.
    main_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    client_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    scene_client_ = create_client<moveit_msgs::srv::GetPlanningScene>(
        "get_planning_scene", rmw_qos_profile_services_default, client_group_);

    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = main_group_;
    objects_sub_ = create_subscription<graspsort_msgs::msg::ObjectPoseArray>(
        objects_topic, rclcpp::QoS(static_cast<std::size_t>(qos_depth)),
        [this](graspsort_msgs::msg::ObjectPoseArray::ConstSharedPtr msg) { onObjects(msg); },
        sub_opts);
    freeze_srv_ = create_service<std_srvs::srv::SetBool>(
        "~/freeze",
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
               std::shared_ptr<std_srvs::srv::SetBool::Response> res) { onFreeze(*req, *res); },
        rmw_qos_profile_services_default, main_group_);
    timer_ = create_wall_timer(
        std::chrono::duration<double>(sync_period), [this] { sync(); }, main_group_);
    RCLCPP_INFO(get_logger(),
                "scene_manager: %zu fixed objects, update_distance %.3f m, update_yaw %.3f rad, "
                "remove_timeout %.2f s, padding %.3f m, bin_exclusion_margin %.3f m",
                fixed_.size(), tp.update_distance, tp.update_yaw, tp.remove_timeout, padding_,
                bin_margin_);
  }

 private:
  void declareFixedObjects() {
    // Table top (legs are under the top and out of the arm's reach; not modelled).
    const gs::Pose2D table{
        declare_parameter<double>("table.x", 0.0), declare_parameter<double>("table.y", 0.0),
        declare_parameter<double>("table.z", 0.0), declare_parameter<double>("table.yaw", 0.0)};
    const auto table_size = toArray3(
        declare_parameter<std::vector<double>>("table.size", std::vector<double>{}), "table.size");
    fixed_.push_back(
        fixedObject("table", world_frame_, table, {gs::tableTopPrimitive(table_size)}));

    const gs::Pose2D pedestal{declare_parameter<double>("pedestal.x", 0.0),
                              declare_parameter<double>("pedestal.y", 0.0),
                              declare_parameter<double>("pedestal.z", 0.0), 0.0};
    const auto pedestal_size =
        toArray3(declare_parameter<std::vector<double>>("pedestal.size", std::vector<double>{}),
                 "pedestal.size");
    fixed_.push_back(
        fixedObject("pedestal", world_frame_, pedestal, {gs::pedestalPrimitive(pedestal_size)}));

    const auto names =
        declare_parameter<std::vector<std::string>>("bins.names", std::vector<std::string>{});
    const auto bin_size = toArray3(
        declare_parameter<std::vector<double>>("bins.size", std::vector<double>{}), "bins.size");
    const double wall = declare_parameter<double>("bins.wall_thickness", 0.0);
    const double floor = declare_parameter<double>("bins.floor_thickness", 0.0);
    if (auto err = gs::checkBin(bin_size, wall, floor)) throw std::invalid_argument(*err);
    const auto boxes = gs::binPrimitives(bin_size, wall, floor);
    for (const auto& n : names) {
      const std::string p = "bins." + n + ".";
      const gs::Pose2D pose{
          declare_parameter<double>(p + "x", 0.0), declare_parameter<double>(p + "y", 0.0),
          declare_parameter<double>(p + "z", 0.0), declare_parameter<double>(p + "yaw", 0.0)};
      fixed_.push_back(fixedObject(n, world_frame_, pose, boxes));
      bin_poses_.push_back(pose);
    }
    bin_size_ = bin_size;
  }

  void onObjects(const graspsort_msgs::msg::ObjectPoseArray::ConstSharedPtr& msg) {
    if (tracker_->frozen()) return;  // frozen: /objects_3d is ignored completely
    if (msg->header.frame_id != world_frame_) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), kLogThrottleMs,
                            "objects_3d frame '%s' is not '%s'; ignored",
                            msg->header.frame_id.c_str(), world_frame_.c_str());
      return;
    }
    std::vector<gs::ObjectEstimate> objs;
    std::size_t in_bins = 0;
    for (const auto& o : msg->objects) {
      if (gs::insideAnyBinArea(o.pose.position.x, o.pose.position.y, bin_poses_, bin_size_,
                               bin_margin_)) {
        ++in_bins;  // already sorted (D-23)
        continue;
      }
      gs::ObjectEstimate e;
      e.id = o.id;
      e.shape = o.shape == static_cast<std::uint8_t>(gs::Shape::kBox) ? gs::Shape::kBox
                                                                      : gs::Shape::kCylinder;
      e.x = o.pose.position.x;
      e.y = o.pose.position.y;
      e.z = o.pose.position.z;
      e.yaw = yawOf(o.pose.orientation);
      e.size = {o.size.x, o.size.y, o.size.z};
      objs.push_back(e);
    }
    if (in_bins > 0) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), kLogThrottleMs,
                           "ignoring %zu perceived objects inside bin areas (D-23)", in_bins);
    }
    latest_ = std::move(objs);
  }

  void onFreeze(const std_srvs::srv::SetBool::Request& req, std_srvs::srv::SetBool::Response& res) {
    tracker_->setFrozen(req.data, now().seconds());
    if (req.data) latest_.reset();  // a message received while frozen is never applied
    res.success = true;
    res.message = req.data ? "scene frozen" : "scene unfrozen";
    RCLCPP_INFO(get_logger(), "%s", res.message.c_str());
  }

  // Reads world object ids and attached object ids; nullopt on failure.
  struct SceneIds {
    std::set<std::string> world;
    std::set<std::string> attached;
  };
  std::optional<SceneIds> readScene() {
    if (!scene_client_->service_is_ready()) return std::nullopt;
    auto req = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    req->components.components =
        moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_NAMES |
        moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS;
    auto fut = scene_client_->async_send_request(req);
    if (fut.wait_for(std::chrono::duration<double>(scene_timeout_)) != std::future_status::ready) {
      scene_client_->remove_pending_request(fut);
      return std::nullopt;
    }
    const auto res = fut.get();
    SceneIds ids;
    for (const auto& co : res->scene.world.collision_objects) ids.world.insert(co.id);
    for (const auto& aco : res->scene.robot_state.attached_collision_objects) {
      ids.attached.insert(aco.object.id);
    }
    return ids;
  }

  CollisionObject perceivedObject(const gs::ObjectEstimate& o) const {
    CollisionObject co;
    co.id = gs::objectId(o.id);
    co.header.frame_id = world_frame_;
    co.pose = toPose(o.x, o.y, o.z, o.shape == gs::Shape::kBox ? o.yaw : 0.0);
    SolidPrimitive prim;
    const auto d = gs::objectDimensions(o, padding_);
    if (o.shape == gs::Shape::kBox) {
      prim.type = SolidPrimitive::BOX;
      prim.dimensions = {d[0], d[1], d[2]};
    } else {
      prim.type = SolidPrimitive::CYLINDER;
      prim.dimensions = {d[0], d[1]};
    }
    co.primitives.push_back(prim);
    co.primitive_poses.push_back(toPose(0.0, 0.0, 0.0, 0.0));
    co.operation = CollisionObject::ADD;  // ADD replaces an existing object with the same id
    return co;
  }

  void sync() {
    if (!psi_) {
      if (!scene_client_->service_is_ready()) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), kLogThrottleMs,
                             "waiting for move_group (get_planning_scene)");
        return;
      }
      psi_ = std::make_unique<moveit::planning_interface::PlanningSceneInterface>();
    }
    const auto ids = readScene();
    if (!ids) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kLogThrottleMs,
                           "get_planning_scene failed; sync step skipped");
      return;
    }

    // Fixed objects: at start-up and whenever they are missing (e.g. move_group restarted).
    std::vector<CollisionObject> fixed_missing;
    for (const auto& co : fixed_) {
      if (!ids->world.count(co.id)) fixed_missing.push_back(co);
    }
    if (!fixed_missing.empty()) {
      if (psi_->applyCollisionObjects(fixed_missing)) {
        RCLCPP_INFO(get_logger(), "added %zu fixed collision objects", fixed_missing.size());
      } else {
        RCLCPP_ERROR(get_logger(), "failed to add the fixed collision objects; retrying");
      }
    }

    if (tracker_->frozen()) return;
    std::optional<std::vector<gs::ObjectEstimate>> seen;
    seen.swap(latest_);
    const auto actions =
        tracker_->step(seen ? &*seen : nullptr, now().seconds(), ids->world, ids->attached);
    if (actions.empty()) return;

    std::vector<CollisionObject> cos;
    for (const auto& o : actions.add) cos.push_back(perceivedObject(o));
    for (const auto& o : actions.update) cos.push_back(perceivedObject(o));
    for (const auto& id : actions.remove) {
      CollisionObject co;
      co.id = id;
      co.header.frame_id = world_frame_;
      co.operation = CollisionObject::REMOVE;
      cos.push_back(co);
    }
    if (!psi_->applyCollisionObjects(cos)) {
      RCLCPP_ERROR(get_logger(), "applying %zu scene changes failed (healed next step)",
                   cos.size());
      return;
    }
    for (const auto& o : actions.add) {
      RCLCPP_INFO(get_logger(), "added %s (%s) at (%.3f, %.3f, %.3f) size (%.3f, %.3f, %.3f)",
                  gs::objectId(o.id).c_str(), o.shape == gs::Shape::kBox ? "box" : "cylinder", o.x,
                  o.y, o.z, o.size[0], o.size[1], o.size[2]);
    }
    for (const auto& o : actions.update) {
      RCLCPP_INFO(get_logger(), "updated %s at (%.3f, %.3f, %.3f)", gs::objectId(o.id).c_str(), o.x,
                  o.y, o.z);
    }
    for (const auto& id : actions.remove) {
      RCLCPP_INFO(get_logger(), "removed %s (not perceived)", id.c_str());
    }
  }

  static constexpr int kLogThrottleMs = 5000;

  std::string world_frame_;
  double padding_{0.0};
  double bin_margin_{0.0};
  std::vector<gs::Pose2D> bin_poses_;
  std::array<double, 3> bin_size_{0.0, 0.0, 0.0};
  double scene_timeout_{0.0};
  std::vector<CollisionObject> fixed_;
  std::unique_ptr<gs::SceneTracker> tracker_;
  std::optional<std::vector<gs::ObjectEstimate>> latest_;
  std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> psi_;
  rclcpp::CallbackGroup::SharedPtr main_group_;
  rclcpp::CallbackGroup::SharedPtr client_group_;
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr scene_client_;
  rclcpp::Subscription<graspsort_msgs::msg::ObjectPoseArray>::SharedPtr objects_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr freeze_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace graspsort_scene

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<graspsort_scene::SceneManagerNode>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
