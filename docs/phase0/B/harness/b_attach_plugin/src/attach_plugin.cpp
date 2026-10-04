// Minimal Gazebo Classic world plugin: /attach creates a fixed joint between two links,
// /detach removes it. Phase 0 (D-03) prototype for GraspSort.
#include <b_attach_msgs/srv/attach_link.hpp>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_ros/node.hpp>
#include <map>
#include <memory>
#include <string>
#include <utility>

namespace graspsort {

using AttachLink = b_attach_msgs::srv::AttachLink;

// Pauses the world for the lifetime of the guard. World::SetPaused() takes the world update
// mutex, so on return no physics step is in progress and none starts until the guard is
// released. Do NOT lock Physics()->GetPhysicsUpdateMutex() here instead: Model::RemoveJoint()
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
    joint_type_ = sdf->Get<std::string>("joint_type", "fixed").first;
    const auto attach_name = sdf->Get<std::string>("attach_service", "attach").first;
    const auto detach_name = sdf->Get<std::string>("detach_service", "detach").first;
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

  static std::string Key(const AttachLink::Request& r) {
    return r.parent_model + "::" + r.parent_link + "__" + r.child_model + "::" + r.child_link;
  }

  // Returns an empty string on success, otherwise the error message.
  std::string FindLink(const std::string& model_name, const std::string& link_name,
                       gazebo::physics::ModelPtr& model, gazebo::physics::LinkPtr& link) const {
    model = world_->ModelByName(model_name);
    if (!model) {
      return "unknown model '" + model_name + "'";
    }
    link = model->GetLink(link_name);
    if (!link) {
      return "unknown link '" + link_name + "' in model '" + model_name + "'";
    }
    return "";
  }

  void OnAttach(const AttachLink::Request& req, AttachLink::Response& res) {
    // Service callbacks run on the gazebo_ros executor thread: keep physics stopped while
    // the joint graph is modified.
    const PauseGuard pause(world_);
    const std::string key = Key(req);
    if (attachments_.count(key) != 0) {
      res.success = false;
      res.message = "already attached: " + key;
      return;
    }
    gazebo::physics::ModelPtr parent_model, child_model;
    gazebo::physics::LinkPtr parent_link, child_link;
    std::string err = FindLink(req.parent_model, req.parent_link, parent_model, parent_link);
    if (err.empty()) {
      err = FindLink(req.child_model, req.child_link, child_model, child_link);
    }
    if (err.empty() && parent_link == child_link) {
      err = "parent and child are the same link";
    }
    if (!err.empty()) {
      res.success = false;
      res.message = err;
      return;
    }
    // CreateJoint() calls Attach() + Load() with the links' current poses, so the
    // existing relative offset is preserved (no snap). Init() makes it active.
    const std::string joint_name = "graspsort_attach__" + key;
    gazebo::physics::JointPtr joint =
        parent_model->CreateJoint(joint_name, joint_type_, parent_link, child_link);
    if (!joint) {
      res.success = false;
      res.message = "failed to create joint '" + joint_name + "'";
      return;
    }
    joint->Init();
    attachments_[key] = Attachment{parent_model, joint};
    res.success = true;
    res.message = "attached " + key;
    RCLCPP_INFO(node_->get_logger(), "%s", res.message.c_str());
  }

  void OnDetach(const AttachLink::Request& req, AttachLink::Response& res) {
    const PauseGuard pause(world_);
    const std::string key = Key(req);
    const auto it = attachments_.find(key);
    if (it == attachments_.end()) {
      res.success = false;
      res.message = "not attached: " + key;
      return;
    }
    const std::string joint_name = it->second.joint->GetName();
    it->second.joint->Detach();
    it->second.parent_model->RemoveJoint(joint_name);
    attachments_.erase(it);
    res.success = true;
    res.message = "detached " + key;
    RCLCPP_INFO(node_->get_logger(), "%s", res.message.c_str());
  }

  gazebo::physics::WorldPtr world_;
  gazebo_ros::Node::SharedPtr node_;
  std::string joint_type_;
  rclcpp::Service<AttachLink>::SharedPtr attach_srv_;
  rclcpp::Service<AttachLink>::SharedPtr detach_srv_;
  std::map<std::string, Attachment> attachments_;
};

GZ_REGISTER_WORLD_PLUGIN(AttachPlugin)

}  // namespace graspsort
