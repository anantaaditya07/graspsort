// GraspSort scene logic (architecture 7.3), header-only and ROS-free so it can be unit-tested.
//
// 1. Fixed geometry from world_layout.yaml: the table top, the pedestal and each open-top bin as
//    box primitives (centre + size, in the object's own frame; the caller places the frame).
// 2. SceneTracker: decides which perceived objects to add, update or remove in the planning
//    scene, given the latest /objects_3d set, the time, the ids currently in the world and the
//    ids currently attached to the robot.
//    - add:    a perceived object that is not tracked, or tracked but missing from the world
//              (removed by someone else, or a failed apply).
//    - update: a tracked object whose estimate moved more than update_distance, whose yaw
//              changed more than update_yaw (boxes only, modulo pi), or whose size changed more
//              than update_distance.
//    - remove: a tracked object that has not been perceived for more than remove_timeout.
//    - attached objects are never added, updated or removed, and are dropped from tracking so
//      that after a detach they are reconciled from scratch.
//    - frozen: no decisions at all (the /objects_3d input is ignored). On unfreeze every tracked
//      object gets a fresh last-seen time, so the freeze itself never causes a removal.
//    - object ids already in the world with the "object_<n>" form that the tracker does not
//      know (for example after a restart) are adopted: they expire like any other object.
// 3. insideBinArea: perceived objects whose centre lies over a bin (outer footprint grown by a
//    margin) are already sorted; the node keeps them out of the planning scene (D-23), so they
//    never block placing into that bin.
#ifndef GRASPSORT_SCENE__SCENE_LOGIC_HPP_
#define GRASPSORT_SCENE__SCENE_LOGIC_HPP_

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace graspsort::scene {

/// Axis-aligned box primitive in its parent frame: centre and full extents (m).
struct Box {
  std::array<double, 3> center{0.0, 0.0, 0.0};
  std::array<double, 3> size{0.0, 0.0, 0.0};
};

/// Planar pose of a fixed object frame in the world: position and yaw (rad).
struct Pose2D {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw{0.0};
};

/// Unit quaternion (x, y, z, w) of a rotation by yaw about z.
inline std::array<double, 4> yawToQuaternion(double yaw) {
  return {0.0, 0.0, std::sin(yaw / 2.0), std::cos(yaw / 2.0)};
}

/// Open-top bin, frame at the centre of its bottom face (world_layout.yaml convention).
/// size = outer {x, y, height}. One floor plate plus 4 walls; the x walls span the full y
/// extent and the y walls fit between them (as in worlds/graspsort.world).
inline std::vector<Box> binPrimitives(const std::array<double, 3>& size, double wall_thickness,
                                      double floor_thickness) {
  const double sx = size[0];
  const double sy = size[1];
  const double h = size[2];
  const double wx = sx / 2.0 - wall_thickness / 2.0;
  const double wy = sy / 2.0 - wall_thickness / 2.0;
  std::vector<Box> out;
  out.push_back({{0.0, 0.0, floor_thickness / 2.0}, {sx, sy, floor_thickness}});
  out.push_back({{wx, 0.0, h / 2.0}, {wall_thickness, sy, h}});
  out.push_back({{-wx, 0.0, h / 2.0}, {wall_thickness, sy, h}});
  out.push_back({{0.0, wy, h / 2.0}, {sx - 2.0 * wall_thickness, wall_thickness, h}});
  out.push_back({{0.0, -wy, h / 2.0}, {sx - 2.0 * wall_thickness, wall_thickness, h}});
  return out;
}

/// Table top, frame at the centre of the top surface. size = {x, y, top thickness}.
inline Box tableTopPrimitive(const std::array<double, 3>& size) {
  return {{0.0, 0.0, -size[2] / 2.0}, size};
}

/// Pedestal, frame at the centre of its top face. size = {x, y, height}.
inline Box pedestalPrimitive(const std::array<double, 3>& size) {
  return {{0.0, 0.0, -size[2] / 2.0}, size};
}

/// Validates a bin description; returns an error text or nullopt.
inline std::optional<std::string> checkBin(const std::array<double, 3>& size, double wall_thickness,
                                           double floor_thickness) {
  for (double s : size) {
    if (!(s > 0.0)) return std::string("bin size must be > 0");
  }
  if (!(wall_thickness > 0.0) || 2.0 * wall_thickness >= std::min(size[0], size[1])) {
    return std::string("bin wall_thickness must be > 0 and < half the bin footprint");
  }
  if (!(floor_thickness > 0.0) || floor_thickness >= size[2]) {
    return std::string("bin floor_thickness must be > 0 and < the bin height");
  }
  return std::nullopt;
}

/// True if (x, y) lies over the outer footprint {size x, size y} of a bin at `bin`, grown by
/// `margin` on every side (margin < 0 shrinks it). D-23: such an object is already in the bin.
inline bool insideBinArea(double x, double y, const Pose2D& bin, const std::array<double, 3>& size,
                          double margin) {
  const double dx = x - bin.x;
  const double dy = y - bin.y;
  const double c = std::cos(bin.yaw);
  const double s = std::sin(bin.yaw);
  const double lx = c * dx + s * dy;  // in the bin frame
  const double ly = -s * dx + c * dy;
  return std::fabs(lx) <= size[0] / 2.0 + margin && std::fabs(ly) <= size[1] / 2.0 + margin;
}

/// True if (x, y) is inside the area of any of the bins (see insideBinArea).
inline bool insideAnyBinArea(double x, double y, const std::vector<Pose2D>& bins,
                             const std::array<double, 3>& size, double margin) {
  return std::any_of(bins.begin(), bins.end(),
                     [&](const Pose2D& b) { return insideBinArea(x, y, b, size, margin); });
}

/// Shape codes of graspsort_msgs/ObjectPose.
enum class Shape : std::uint8_t { kCylinder = 0, kBox = 1 };

/// One perceived object (mirror of graspsort_msgs/ObjectPose, world frame, centre pose).
struct ObjectEstimate {
  std::uint32_t id{0};
  Shape shape{Shape::kCylinder};
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw{0.0};
  std::array<double, 3> size{0.0, 0.0, 0.0};  // footprint x, y and height
};

/// Collision primitive for a perceived object, padded on every side by `padding`.
/// Cylinder: dims {height, radius} (shape_msgs/SolidPrimitive order); radius = size.x / 2.
/// Box: dims {x, y, z}.
inline std::array<double, 3> objectDimensions(const ObjectEstimate& o, double padding) {
  if (o.shape == Shape::kCylinder) {
    return {o.size[2] + 2.0 * padding, o.size[0] / 2.0 + padding, 0.0};
  }
  return {o.size[0] + 2.0 * padding, o.size[1] + 2.0 * padding, o.size[2] + 2.0 * padding};
}

inline constexpr const char* kObjectPrefix = "object_";
inline constexpr double kPi = 3.14159265358979323846;

/// Planning scene id of a perceived object.
inline std::string objectId(std::uint32_t id) {
  return std::string(kObjectPrefix) + std::to_string(id);
}

/// True for ids of the form "object_<digits>".
inline bool isObjectId(const std::string& id) {
  const std::string prefix(kObjectPrefix);
  if (id.size() <= prefix.size() || id.compare(0, prefix.size(), prefix) != 0) return false;
  for (std::size_t i = prefix.size(); i < id.size(); ++i) {
    if (!std::isdigit(static_cast<unsigned char>(id[i]))) return false;
  }
  return true;
}

/// Smallest absolute difference between two yaws of a box (a box is symmetric under pi).
inline double boxYawDifference(double a, double b) {
  double d = std::fmod(std::fabs(a - b), kPi);
  return std::min(d, kPi - d);
}

struct TrackerParams {
  double update_distance{0.01};  // m
  double update_yaw{0.1};        // rad, boxes only
  double remove_timeout{2.0};    // s
};

/// Validates tracker parameters; returns an error text or nullopt.
inline std::optional<std::string> checkParams(const TrackerParams& p) {
  if (!(p.update_distance > 0.0)) return std::string("update_distance must be > 0");
  if (!(p.update_yaw > 0.0)) return std::string("update_yaw must be > 0");
  if (!(p.remove_timeout > 0.0)) return std::string("remove_timeout must be > 0");
  return std::nullopt;
}

struct SceneActions {
  std::vector<ObjectEstimate> add;
  std::vector<ObjectEstimate> update;
  std::vector<std::string> remove;
  bool empty() const { return add.empty() && update.empty() && remove.empty(); }
};

class SceneTracker {
 public:
  explicit SceneTracker(const TrackerParams& params) : params_(params) {}

  bool frozen() const { return frozen_; }

  /// Freeze (true) or unfreeze (false). Unfreezing restarts every remove timeout at `now`.
  void setFrozen(bool frozen, double now) {
    if (frozen_ && !frozen) {
      for (auto& kv : tracked_) kv.second.last_seen = now;
    }
    frozen_ = frozen;
  }

  /// One decision step.
  /// @param seen      latest perceived set, or nullptr if no new /objects_3d message since the
  ///                  last step (then only timeouts are evaluated)
  /// @param now       current time (s)
  /// @param world_ids ids of the world collision objects currently in the planning scene
  /// @param attached  ids of the objects currently attached to the robot
  /// The caller applies the returned actions; failed adds/updates heal on the next step
  /// because the object is then missing from world_ids.
  SceneActions step(const std::vector<ObjectEstimate>* seen, double now,
                    const std::set<std::string>& world_ids, const std::set<std::string>& attached) {
    SceneActions out;
    if (frozen_) return out;

    // Adopt unknown "object_<n>" world objects so they expire if they are not perceived.
    for (const auto& id : world_ids) {
      if (isObjectId(id) && !attached.count(id) && !tracked_.count(id)) {
        tracked_[id] = Tracked{std::nullopt, now};
      }
    }

    std::set<std::string> seen_ids;
    if (seen != nullptr) {
      for (const auto& o : *seen) {
        const std::string id = objectId(o.id);
        if (!seen_ids.insert(id).second) continue;  // duplicate id in one message: first wins
        if (attached.count(id)) continue;           // never touch an attached object
        auto it = tracked_.find(id);
        if (it == tracked_.end() || !world_ids.count(id)) {
          out.add.push_back(o);
          tracked_[id] = Tracked{o, now};
          continue;
        }
        it->second.last_seen = now;
        if (!it->second.applied || changed(*it->second.applied, o)) {
          out.update.push_back(o);
          it->second.applied = o;
        }
      }
    }

    for (auto it = tracked_.begin(); it != tracked_.end();) {
      const std::string& id = it->first;
      if (attached.count(id)) {
        it = tracked_.erase(it);  // untouchable while attached; reconcile after detach
      } else if (seen_ids.count(id)) {
        ++it;
      } else if (now - it->second.last_seen > params_.remove_timeout) {
        if (world_ids.count(id)) out.remove.push_back(id);
        it = tracked_.erase(it);
      } else if (!world_ids.count(id)) {
        it = tracked_.erase(it);  // removed by someone else; re-added when perceived again
      } else {
        ++it;
      }
    }
    return out;
  }

  /// Ids currently tracked (for diagnostics and tests).
  std::vector<std::string> trackedIds() const {
    std::vector<std::string> ids;
    for (const auto& kv : tracked_) ids.push_back(kv.first);
    return ids;
  }

 private:
  struct Tracked {
    std::optional<ObjectEstimate> applied;  // what is in the scene (nullopt: adopted, unknown)
    double last_seen{0.0};
  };

  bool changed(const ObjectEstimate& a, const ObjectEstimate& b) const {
    if (a.shape != b.shape) return true;
    const double d = std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) +
                               (a.z - b.z) * (a.z - b.z));
    if (d > params_.update_distance) return true;
    for (std::size_t i = 0; i < a.size.size(); ++i) {
      if (std::fabs(a.size[i] - b.size[i]) > params_.update_distance) return true;
    }
    return b.shape == Shape::kBox && boxYawDifference(a.yaw, b.yaw) > params_.update_yaw;
  }

  TrackerParams params_;
  bool frozen_{false};
  std::map<std::string, Tracked> tracked_;
};

}  // namespace graspsort::scene

#endif  // GRASPSORT_SCENE__SCENE_LOGIC_HPP_
