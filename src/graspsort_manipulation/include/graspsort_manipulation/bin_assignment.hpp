// ROS-free class -> bin assignment for GraspSort (architecture 7.5 "Transport and place").
//
// Class names are the detector's COCO names (D-04); bin names are the bin TF frames from
// graspsort_gazebo/config/world_layout.yaml. The map comes from configuration: ROS parameters
// cannot hold a map, so the node passes two parallel string arrays (classes[i] -> bins[i]).
// Unknown classes get no bin unless a default bin is configured. Matching is exact
// (case-sensitive).
#ifndef GRASPSORT_MANIPULATION__BIN_ASSIGNMENT_HPP_
#define GRASPSORT_MANIPULATION__BIN_ASSIGNMENT_HPP_

#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace graspsort::manipulation {

// Documented defaults (D-04 classes, Phase 1 bins).
inline std::vector<std::string> defaultBinClasses() { return {"sports ball", "cup", "bottle"}; }
inline std::vector<std::string> defaultBinNames() { return {"bin_ball", "bin_cup", "bin_bottle"}; }

class BinAssignment {
 public:
  // `default_bin` empty = unknown classes have no bin. Throws std::invalid_argument on empty
  // class or bin names.
  explicit BinAssignment(std::map<std::string, std::string> class_to_bin,
                         std::string default_bin = "")
      : map_(std::move(class_to_bin)), default_bin_(std::move(default_bin)) {
    for (const auto& [cls, bin] : map_) {
      if (cls.empty() || bin.empty()) {
        throw std::invalid_argument("BinAssignment: class and bin names must be non-empty");
      }
    }
  }

  // From parallel arrays (ROS parameters). Throws std::invalid_argument if the lengths differ, a
  // name is empty or a class appears twice.
  static BinAssignment fromLists(const std::vector<std::string>& classes,
                                 const std::vector<std::string>& bins,
                                 const std::string& default_bin = "") {
    if (classes.size() != bins.size()) {
      throw std::invalid_argument("BinAssignment: classes and bins must have the same length");
    }
    std::map<std::string, std::string> m;
    for (std::size_t i = 0; i < classes.size(); ++i) {
      if (!m.emplace(classes[i], bins[i]).second) {
        throw std::invalid_argument("BinAssignment: duplicate class '" + classes[i] + "'");
      }
    }
    return BinAssignment(std::move(m), default_bin);
  }

  // The configured defaults: sports ball -> bin_ball, cup -> bin_cup, bottle -> bin_bottle.
  static BinAssignment defaults() { return fromLists(defaultBinClasses(), defaultBinNames()); }

  // Bin for a class, the default bin for an unknown class if configured, else nullopt.
  std::optional<std::string> binFor(const std::string& class_name) const {
    const auto it = map_.find(class_name);
    if (it != map_.end()) {
      return it->second;
    }
    if (!default_bin_.empty()) {
      return default_bin_;
    }
    return std::nullopt;
  }

  // True if the class has its own bin (not just the default).
  bool knows(const std::string& class_name) const { return map_.count(class_name) > 0; }

  // Configured classes, sorted.
  std::vector<std::string> classes() const {
    std::vector<std::string> out;
    for (const auto& kv : map_) {
      out.push_back(kv.first);
    }
    return out;
  }

 private:
  std::map<std::string, std::string> map_;
  std::string default_bin_;
};

}  // namespace graspsort::manipulation

#endif  // GRASPSORT_MANIPULATION__BIN_ASSIGNMENT_HPP_
