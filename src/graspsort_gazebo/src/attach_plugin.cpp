// Gazebo Classic world plugin for GraspSort (D-03, architecture 7.6).
// /attach creates a fixed joint between two links (gripper finger <-> object), /detach removes
// it. Based on the Phase 0 B v2 prototype (4200 attach/detach cycles, 0 failures).
// SDF parameters: joint_type (default "fixed"), attach_service (default "/attach"),
// detach_service (default "/detach"), max_attach_distance (m, default 0.02).
// Nearest mode (Phase 4): an empty child_model in /attach attaches the nearest non-static model
// whose link bounding box is within max_attach_distance of the parent link's bounding box (the
// object between the fingers); an empty child_model in /detach detaches everything attached to
// the parent link. The response message names the attachment(s).
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_ros/node.hpp>
#include <graspsort_gazebo/attachment_registry.hpp>
#include <graspsort_msgs/srv/attach_link.hpp>
#include <ignition/math/AxisAlignedBox.hh>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace graspsort {

using AttachLink = graspsort_msgs::srv::AttachLink;

namespace {
constexpr const char* kDefaultJointType = "fixed";
constexpr const char* kDefaultAttachService = "/attach";
constexpr const char* kDefaultDetachService = "/detach";
constexpr const char* kJointNamePrefix = "graspsort_attach__";
constexpr double kDefaultMaxAttachDistance = 0.02;  // m, bounding box gap

Aabb ToAabb(const ignition::math::AxisAlignedBox& b) {
  return Aabb{{b.Min().X(), b.Min().Y(), b.Min().Z()}, {b.Max().X(), b.Max().Y(), b.Max().Z()}};
}

LinkPair ToLinkPair(const AttachLink::Request& r) {
  return LinkPair{r.parent_model, r.parent_link, r.child_model, r.child_link};
}
}  // namespace

// Pauses the world for the lifetime of the guard. World::SetPaused() takes the world update
// mutex, so on return no physics step is in progress and none starts until the guard is
// released. Do NOT lock Physics()->GetPhysicsUpdateMutex() here instead. Model::RemoveJoint()
// calls World::SetPaused() internally, and the physics thread locks world-update -> physics,
// so physics -> world-update from a service thread deadlocks (observed in Phase 0 testing).
class PauseGuard {
 public:
  explicit PauseGuard(gazebo::physics::WorldPtr world)
      : world_(std::move(world)), was_paused_(world_->IsPaused()) {
    world_->SetPaused(true);
  }
  ~PauseGuard() { world_->SetPaused(was_paused_); }
  PauseGuard(const PauseGuard&) = delete;
  PauseGuard& operator=(const PauseGuard&) = delete;

 private:
  gazebo::physics::WorldPtr world_;
  bool was_paused_;
};

class AttachPlugin : public gazebo::WorldPlugin {
 public:
  void Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf) override {
    world_ = world;
    node_ = gazebo_ros::Node::Get(sdf);
    joint_type_ = sdf->Get<std::string>("joint_type", kDefaultJointType).first;
    const auto attach_name = sdf->Get<std::string>("attach_service", kDefaultAttachService).first;
    const auto detach_name = sdf->Get<std::string>("detach_service", kDefaultDetachService).first;
    max_attach_distance_ = sdf->Get<double>("max_attach_distance", kDefaultMaxAttachDistance).first;
    attach_srv_ = node_->create_service<AttachLink>(
        attach_name, [this](const std::shared_ptr<AttachLink::Request> req,
                            std::shared_ptr<AttachLink::Response> res) { OnAttach(*req, *res); });
    detach_srv_ = node_->create_service<AttachLink>(
        detach_name, [this](const std::shared_ptr<AttachLink::Request> req,
                            std::shared_ptr<AttachLink::Response> res) { OnDetach(*req, *res); });
    RCLCPP_INFO(node_->get_logger(),
                "Attach plugin ready (joint type '%s', services %s, %s, nearest within %.3f m)",
                joint_type_.c_str(), attach_srv_->get_service_name(),
                detach_srv_->get_service_name(), max_attach_distance_);
  }

 private:
  struct Attachment {
    gazebo::physics::ModelPtr parent_model;
    gazebo::physics::JointPtr joint;
  };

  // Returns an empty string on success, otherwise the error message.
  std::string FindLink(const std::string& model_name, const std::string& link_name,
                       gazebo::physics::ModelPtr& model, gazebo::physics::LinkPtr& link) const {
    model = world_->ModelByName(model_name);
    link = model ? model->GetLink(link_name) : nullptr;
    return LookupError(model_name, link_name, model != nullptr, link != nullptr);
  }

  // Nearest mode: fills child_model / child_link with the nearest non-static model's link to the
  // parent link. Returns an empty string on success, otherwise the error message.
  std::string ResolveNearest(gazebo::physics::LinkPtr parent_link, LinkPair& pair) const {
    const Aabb parent_box = ToAabb(parent_link->BoundingBox());
    std::vector<AttachCandidate> candidates;
    for (const auto& model : world_->Models()) {
      for (const auto& link : model->GetLinks()) {
        candidates.push_back({model->GetName(), link->GetName(),
                              AabbDistance(parent_box, ToAabb(link->BoundingBox())),
                              model->IsStatic()});
      }
    }
    const auto best = SelectNearest(candidates, pair.parent_model, max_attach_distance_);
    if (!best) {
      return "no non-static model within " + std::to_string(max_attach_distance_) + " m of " +
             pair.parent_model + kScopeSeparator + pair.parent_link;
    }
    pair.child_model = best->model;
    pair.child_link = best->link;
    return "";
  }

  static void Fail(AttachLink::Response& res, const std::string& message) {
    res.success = false;
    res.message = message;
  }

  void OnAttach(const AttachLink::Request& req, AttachLink::Response& res) {
    LinkPair pair = ToLinkPair(req);
    std::string err = ValidateRequest(pair);
    if (!err.empty()) {
      Fail(res, err);
      return;
    }
    // Service callbacks run on the gazebo_ros executor thread. Keep physics stopped while
    // the joint graph is modified.
    const PauseGuard pause(world_);
    gazebo::physics::ModelPtr parent_model, child_model;
    gazebo::physics::LinkPtr parent_link, child_link;
    err = FindLink(pair.parent_model, pair.parent_link, parent_model, parent_link);
    if (err.empty() && IsNearestRequest(pair)) {
      err = ResolveNearest(parent_link, pair);
    }
    const std::string key = MakeKey(pair);
    if (err.empty()) {
      err = registry_.CheckAttach(key);
    }
    if (err.empty()) {
      err = FindLink(pair.child_model, pair.child_link, child_model, child_link);
    }
    if (!err.empty()) {
      Fail(res, err);
      return;
    }
    // CreateJoint() calls Attach() + Load() with the links' current poses, so the existing
    // relative offset is preserved (no snap). Init() makes it active.
    const std::string joint_name = MakeJointName(kJointNamePrefix, key);
    gazebo::physics::JointPtr joint =
        parent_model->CreateJoint(joint_name, joint_type_, parent_link, child_link);
    if (!joint) {
      Fail(res, "failed to create joint '" + joint_name + "'");
      return;
    }
    joint->Init();
    registry_.Add(key, Attachment{parent_model, joint});
    res.success = true;
    res.message = "attached " + key;
    RCLCPP_INFO(node_->get_logger(), "%s", res.message.c_str());
  }

  void OnDetach(const AttachLink::Request& req, AttachLink::Response& res) {
    const LinkPair pair = ToLinkPair(req);
    const std::string err = ValidateRequest(pair);
    if (!err.empty()) {
      Fail(res, err);
      return;
    }
    const PauseGuard pause(world_);
    std::vector<std::string> keys;
    if (IsNearestRequest(pair)) {
      keys = registry_.KeysForParent(pair.parent_model, pair.parent_link);
      if (keys.empty()) {
        Fail(res, "nothing attached to " + pair.parent_model + kScopeSeparator + pair.parent_link);
        return;
      }
    } else {
      keys = {MakeKey(pair)};
    }
    std::string detached;
    for (const auto& key : keys) {
      std::optional<Attachment> attachment = registry_.Remove(key);
      if (!attachment) {
        Fail(res, registry_.CheckDetach(key));
        return;
      }
      const std::string joint_name = attachment->joint->GetName();
      attachment->joint->Detach();
      attachment->parent_model->RemoveJoint(joint_name);
      detached += (detached.empty() ? "" : ", ") + key;
    }
    res.success = true;
    res.message = "detached " + detached;
    RCLCPP_INFO(node_->get_logger(), "%s", res.message.c_str());
  }

  gazebo::physics::WorldPtr world_;
  gazebo_ros::Node::SharedPtr node_;
  std::string joint_type_;
  double max_attach_distance_{kDefaultMaxAttachDistance};
  rclcpp::Service<AttachLink>::SharedPtr attach_srv_;
  rclcpp::Service<AttachLink>::SharedPtr detach_srv_;
  AttachmentRegistry<Attachment> registry_;
};

GZ_REGISTER_WORLD_PLUGIN(AttachPlugin)

}  // namespace graspsort
