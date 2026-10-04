// Gazebo Classic world plugin for GraspSort (D-03, architecture 7.6).
// /attach creates a fixed joint between two links (gripper finger <-> object), /detach removes
// it. Based on the Phase 0 B v2 prototype (4200 attach/detach cycles, 0 failures).
// SDF parameters: joint_type (default "fixed"), attach_service (default "/attach"),
// detach_service (default "/detach").
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_ros/node.hpp>
#include <graspsort_gazebo/attachment_registry.hpp>
#include <graspsort_msgs/srv/attach_link.hpp>
#include <memory>
#include <optional>
#include <string>
#include <utility>

namespace graspsort {

using AttachLink = graspsort_msgs::srv::AttachLink;

namespace {
constexpr const char* kDefaultJointType = "fixed";
constexpr const char* kDefaultAttachService = "/attach";
constexpr const char* kDefaultDetachService = "/detach";
constexpr const char* kJointNamePrefix = "graspsort_attach__";

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
    attach_srv_ = node_->create_service<AttachLink>(
        attach_name, [this](const std::shared_ptr<AttachLink::Request> req,
                            std::shared_ptr<AttachLink::Response> res) { OnAttach(*req, *res); });
    detach_srv_ = node_->create_service<AttachLink>(
        detach_name, [this](const std::shared_ptr<AttachLink::Request> req,
                            std::shared_ptr<AttachLink::Response> res) { OnDetach(*req, *res); });
    RCLCPP_INFO(node_->get_logger(), "Attach plugin ready (joint type '%s', services %s, %s)",
                joint_type_.c_str(), attach_srv_->get_service_name(),
                detach_srv_->get_service_name());
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

  static void Fail(AttachLink::Response& res, const std::string& message) {
    res.success = false;
    res.message = message;
  }

  void OnAttach(const AttachLink::Request& req, AttachLink::Response& res) {
    const LinkPair pair = ToLinkPair(req);
    std::string err = ValidateRequest(pair);
    if (!err.empty()) {
      Fail(res, err);
      return;
    }
    // Service callbacks run on the gazebo_ros executor thread. Keep physics stopped while
    // the joint graph is modified.
    const PauseGuard pause(world_);
    const std::string key = MakeKey(pair);
    err = registry_.CheckAttach(key);
    gazebo::physics::ModelPtr parent_model, child_model;
    gazebo::physics::LinkPtr parent_link, child_link;
    if (err.empty()) {
      err = FindLink(req.parent_model, req.parent_link, parent_model, parent_link);
    }
    if (err.empty()) {
      err = FindLink(req.child_model, req.child_link, child_model, child_link);
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
    const std::string key = MakeKey(pair);
    std::optional<Attachment> attachment = registry_.Remove(key);
    if (!attachment) {
      Fail(res, registry_.CheckDetach(key));
      return;
    }
    const std::string joint_name = attachment->joint->GetName();
    attachment->joint->Detach();
    attachment->parent_model->RemoveJoint(joint_name);
    res.success = true;
    res.message = "detached " + key;
    RCLCPP_INFO(node_->get_logger(), "%s", res.message.c_str());
  }

  gazebo::physics::WorldPtr world_;
  gazebo_ros::Node::SharedPtr node_;
  std::string joint_type_;
  rclcpp::Service<AttachLink>::SharedPtr attach_srv_;
  rclcpp::Service<AttachLink>::SharedPtr detach_srv_;
  AttachmentRegistry<Attachment> registry_;
};

GZ_REGISTER_WORLD_PLUGIN(AttachPlugin)

}  // namespace graspsort
