// ROS-free, Gazebo-free bookkeeping for the GraspSort attach plugin (D-03).
// Builds attachment keys and joint names, validates requests, and tracks active
// attachments. The plugin stores Gazebo handles as the Entry type; tests use plain types.
#ifndef GRASPSORT_GAZEBO__ATTACHMENT_REGISTRY_HPP_
#define GRASPSORT_GAZEBO__ATTACHMENT_REGISTRY_HPP_

#include <cstddef>
#include <map>
#include <optional>
#include <string>
#include <utility>

namespace graspsort {

// Mirrors the fields of graspsort_msgs/srv/AttachLink (request part).
struct LinkPair {
  std::string parent_model;
  std::string parent_link;
  std::string child_model;
  std::string child_link;
};

// Separators used in keys. "::" matches Gazebo's scoped-name convention.
inline constexpr const char* kScopeSeparator = "::";
inline constexpr const char* kPairSeparator = "__";

// "parent_model::parent_link__child_model::child_link"
inline std::string MakeKey(const LinkPair& p) {
  return p.parent_model + kScopeSeparator + p.parent_link + kPairSeparator + p.child_model +
         kScopeSeparator + p.child_link;
}

// Name of the Gazebo joint created for an attachment.
inline std::string MakeJointName(const std::string& prefix, const std::string& key) {
  return prefix + key;
}

// Checks a request before any lookup in the simulator.
// Returns an empty string if valid, otherwise the error message.
inline std::string ValidateRequest(const LinkPair& p) {
  if (p.parent_model.empty()) {
    return "parent_model is empty";
  }
  if (p.parent_link.empty()) {
    return "parent_link is empty";
  }
  if (p.child_model.empty()) {
    return "child_model is empty";
  }
  if (p.child_link.empty()) {
    return "child_link is empty";
  }
  if (p.parent_model == p.child_model && p.parent_link == p.child_link) {
    return "parent and child are the same link";
  }
  return "";
}

// Error message for a failed model/link lookup in the simulator.
// Returns an empty string if both were found.
inline std::string LookupError(const std::string& model_name, const std::string& link_name,
                               bool model_found, bool link_found) {
  if (!model_found) {
    return "unknown model '" + model_name + "'";
  }
  if (!link_found) {
    return "unknown link '" + link_name + "' in model '" + model_name + "'";
  }
  return "";
}

template <typename Entry>
class AttachmentRegistry {
 public:
  bool Contains(const std::string& key) const { return entries_.count(key) != 0; }

  // Returns an empty string if `key` may be attached, otherwise the error message.
  std::string CheckAttach(const std::string& key) const {
    return Contains(key) ? "already attached: " + key : "";
  }

  // Returns an empty string if `key` may be detached, otherwise the error message.
  std::string CheckDetach(const std::string& key) const {
    return Contains(key) ? "" : "not attached: " + key;
  }

  // Returns false (and changes nothing) if the key already exists.
  bool Add(const std::string& key, Entry entry) {
    return entries_.emplace(key, std::move(entry)).second;
  }

  // Pointer to the stored entry, or nullptr if absent. Valid until the next Add/Remove.
  const Entry* Find(const std::string& key) const {
    const auto it = entries_.find(key);
    return it == entries_.end() ? nullptr : &it->second;
  }

  // Removes and returns the entry, or std::nullopt if absent.
  std::optional<Entry> Remove(const std::string& key) {
    const auto it = entries_.find(key);
    if (it == entries_.end()) {
      return std::nullopt;
    }
    Entry entry = std::move(it->second);
    entries_.erase(it);
    return entry;
  }

  std::size_t Size() const { return entries_.size(); }

 private:
  std::map<std::string, Entry> entries_;
};

}  // namespace graspsort

#endif  // GRASPSORT_GAZEBO__ATTACHMENT_REGISTRY_HPP_
