// ROS-free, Gazebo-free bookkeeping for the GraspSort attach plugin (D-03).
// Builds attachment keys and joint names, validates requests, and tracks active
// attachments. The plugin stores Gazebo handles as the Entry type; tests use plain types.
//
// Nearest mode (Phase 4 user decision): an empty child_model in /attach means "the nearest
// non-static model to parent_link within max_attach_distance" (sim-side emulation of a physical
// grasp, so the robot never needs ground-truth model names); an empty child_model in /detach
// means "everything attached to parent_link".
#ifndef GRASPSORT_GAZEBO__ATTACHMENT_REGISTRY_HPP_
#define GRASPSORT_GAZEBO__ATTACHMENT_REGISTRY_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <map>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

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

// True if the request asks for the nearest model (empty child_model).
inline bool IsNearestRequest(const LinkPair& p) { return p.child_model.empty(); }

// Checks a request before any lookup in the simulator.
// Returns an empty string if valid, otherwise the error message. An empty child_model selects
// nearest mode, in which child_link is ignored.
inline std::string ValidateRequest(const LinkPair& p) {
  if (p.parent_model.empty()) {
    return "parent_model is empty";
  }
  if (p.parent_link.empty()) {
    return "parent_link is empty";
  }
  if (IsNearestRequest(p)) {
    return "";
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

// World-frame axis-aligned box (m), as Gazebo's Link::BoundingBox().
struct Aabb {
  std::array<double, 3> min{};
  std::array<double, 3> max{};
};

// Euclidean distance (m) between two boxes; 0 if they touch or overlap.
inline double AabbDistance(const Aabb& a, const Aabb& b) {
  double sq = 0.0;
  for (std::size_t i = 0; i < 3; ++i) {
    const double gap = std::max({0.0, a.min[i] - b.max[i], b.min[i] - a.max[i]});
    sq += gap * gap;
  }
  return std::sqrt(sq);
}

// A link the parent could attach to in nearest mode.
struct AttachCandidate {
  std::string model;
  std::string link;
  double distance{0.0};  // AabbDistance to the parent link (m)
  bool is_static{false};
};

// Nearest non-static candidate of another model within max_distance (m), or nullopt. Ties are
// broken by model name, then link name, so the choice is deterministic.
inline std::optional<AttachCandidate> SelectNearest(const std::vector<AttachCandidate>& candidates,
                                                    const std::string& parent_model,
                                                    double max_distance) {
  std::optional<AttachCandidate> best;
  for (const auto& c : candidates) {
    if (c.is_static || c.model == parent_model || !(c.distance <= max_distance)) {
      continue;
    }
    if (!best ||
        std::tie(c.distance, c.model, c.link) < std::tie(best->distance, best->model, best->link)) {
      best = c;
    }
  }
  return best;
}

// Key prefix shared by every attachment of one parent link: "parent_model::parent_link__".
inline std::string ParentKeyPrefix(const std::string& parent_model,
                                   const std::string& parent_link) {
  return parent_model + kScopeSeparator + parent_link + kPairSeparator;
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

  // Keys of all attachments of one parent link, in key order.
  std::vector<std::string> KeysForParent(const std::string& parent_model,
                                         const std::string& parent_link) const {
    const std::string prefix = ParentKeyPrefix(parent_model, parent_link);
    std::vector<std::string> keys;
    for (auto it = entries_.lower_bound(prefix);
         it != entries_.end() && it->first.compare(0, prefix.size(), prefix) == 0; ++it) {
      keys.push_back(it->first);
    }
    return keys;
  }

 private:
  std::map<std::string, Entry> entries_;
};

}  // namespace graspsort

#endif  // GRASPSORT_GAZEBO__ATTACHMENT_REGISTRY_HPP_
