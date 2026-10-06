// ROS-free decision logic of the GraspSort sort task (architecture 7.5, Phase 5).
//
// - Failure categories (architecture 8 "failure breakdown") and the action's failure_reasons
//   string format "<object> (<class>): <category>: <detail>".
// - Mapping of the pick-and-place steps (PickPlaceExecutor) to the action feedback stages
//   (detect / plan_grasp / approach / grasp / transport / place) and to default failure
//   categories.
// - Candidate filtering: requested classes with a bin, not inside a bin footprint, not given up.
// - SortBook: attempts per object and given-up objects, matched by position (perception track
//   ids change when an object moves or is re-detected).
// - Ordering: closest to the arm base first (xy).
// - SettleDetector: /objects_3d is "settled" when the set of track ids is unchanged and no object
//   moved more than a tolerance for a minimum time.
// - Release slot inside a bin: objects are released side by side so a later object of the same
//   class does not land on an earlier one (the bins fit two objects, see chooseReleaseSlot).
// - Start pose: jointsWithin decides whether the arm is already at the ready state before a goal
//   (from the spawn pose the reach check finds no collision-free IK, D-24).
//
// Units: metres, radians, seconds. Frame: world, z up.
#ifndef GRASPSORT_MANIPULATION__SORT_LOGIC_HPP_
#define GRASPSORT_MANIPULATION__SORT_LOGIC_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "graspsort_manipulation/bin_assignment.hpp"
#include "graspsort_manipulation/grasp_planner.hpp"
#include "graspsort_manipulation/pick_place_geometry.hpp"

namespace graspsort::manipulation {

// ---------------------------------------------------------------------------------------------
// Failure categories and reason strings
// ---------------------------------------------------------------------------------------------

enum class FailureCategory : std::uint8_t {
  kNone = 0,
  kNoIk,              // no IK solution for a goal pose during the pick
  kPlanFailed,        // a motion could not be planned
  kGraspSlipped,      // object not closed on / not attached / lost
  kExecutionFailed,   // a planned motion or gripper command failed during execution
  kMisdetection,      // the target vanished or became invalid
  kUnreachable,       // never attempted: no collision-free IK for the best candidate
  kNoGraspCandidate,  // never attempted: grasp_planner rejected every candidate
  kOther
};

inline const char* toString(FailureCategory c) {
  switch (c) {
    case FailureCategory::kNone:
      return "none";
    case FailureCategory::kNoIk:
      return "no_ik";
    case FailureCategory::kPlanFailed:
      return "plan_failed";
    case FailureCategory::kGraspSlipped:
      return "grasp_slipped";
    case FailureCategory::kExecutionFailed:
      return "execution_failed";
    case FailureCategory::kMisdetection:
      return "misdetection";
    case FailureCategory::kUnreachable:
      return "unreachable";
    case FailureCategory::kNoGraspCandidate:
      return "no_grasp_candidate";
    case FailureCategory::kOther:
      return "other";
  }
  return "other";
}

// One line, no newlines (detail newlines become spaces).
inline std::string formatFailureReason(const std::string& object, const std::string& class_name,
                                       FailureCategory category, const std::string& detail) {
  std::string d = detail;
  std::replace(d.begin(), d.end(), '\n', ' ');
  return object + " (" + class_name + "): " + toString(category) + ": " + d;
}

// "object_7 (bottle)": the action feedback's current_object.
inline std::string objectLabel(const std::string& id_prefix, std::uint32_t id) {
  return id_prefix + std::to_string(id);
}

// Pick-and-place steps of PickPlaceExecutor, in execution order.
inline const std::vector<std::string>& pickPlaceSteps() {
  static const std::vector<std::string> steps = {
      "lock_target",   "open_gripper",    "plan_pregrasp", "pregrasp",      "approach",
      "close_gripper", "attach_gazebo",   "attach_moveit", "lift",          "move_above_bin",
      "lower",         "release_gripper", "detach_gazebo", "detach_moveit", "remove_object",
      "retreat",       "ready",           "unfreeze_scene"};
  return steps;
}

// Action feedback stage (SortObjects.action: detect / plan_grasp / approach / grasp / transport /
// place) of a pick-and-place step; "" for an unknown step.
inline std::string actionStageFor(const std::string& step) {
  static const std::map<std::string, std::string> m = {{"lock_target", "detect"},
                                                       {"open_gripper", "plan_grasp"},
                                                       {"plan_pregrasp", "plan_grasp"},
                                                       {"pregrasp", "approach"},
                                                       {"approach", "approach"},
                                                       {"close_gripper", "grasp"},
                                                       {"attach_gazebo", "grasp"},
                                                       {"attach_moveit", "grasp"},
                                                       {"lift", "transport"},
                                                       {"move_above_bin", "transport"},
                                                       {"lower", "place"},
                                                       {"release_gripper", "place"},
                                                       {"detach_gazebo", "place"},
                                                       {"detach_moveit", "place"},
                                                       {"remove_object", "place"},
                                                       {"retreat", "place"},
                                                       {"ready", "place"},
                                                       {"unfreeze_scene", "place"}};
  const auto it = m.find(step);
  return it == m.end() ? std::string() : it->second;
}

// Default category of a failure in a step when the step did not report a more specific one
// (planning failures report kPlanFailed / kNoIk themselves).
inline FailureCategory categoryForStep(const std::string& step) {
  if (step == "lock_target") {
    return FailureCategory::kMisdetection;
  }
  if (step == "plan_pregrasp") {
    return FailureCategory::kPlanFailed;
  }
  if (step == "close_gripper" || step == "attach_gazebo" || step == "attach_moveit") {
    return FailureCategory::kGraspSlipped;
  }
  if (step == "open_gripper" || step == "pregrasp" || step == "approach" || step == "lift" ||
      step == "move_above_bin" || step == "lower" || step == "release_gripper" ||
      step == "retreat" || step == "ready") {
    return FailureCategory::kExecutionFailed;
  }
  return FailureCategory::kOther;
}

// ---------------------------------------------------------------------------------------------
// Bins and candidate filtering
// ---------------------------------------------------------------------------------------------

struct NamedBin {
  std::string name;
  BinGeometry geometry;
};

// True if the world point is inside the bin's outer footprint (xy, bin frame), grown by `margin`.
inline bool insideFootprint(const BinGeometry& bin, const Eigen::Vector3d& p, double margin) {
  const Eigen::Vector3d local = bin.pose.inverse() * p;
  return std::abs(local.x()) <= bin.size.x() / 2.0 + margin &&
         std::abs(local.y()) <= bin.size.y() / 2.0 + margin;
}

// Name of the first bin whose footprint (+ margin) contains p, or nullopt.
inline std::optional<std::string> binContaining(const std::vector<NamedBin>& bins,
                                                const Eigen::Vector3d& p, double margin) {
  for (const auto& b : bins) {
    if (insideFootprint(b.geometry, p, margin)) {
      return b.name;
    }
  }
  return std::nullopt;
}

// Classes a goal sorts: the requested ones, or every class with its own bin if none requested.
// Throws std::invalid_argument if a requested class has no bin of its own.
inline std::vector<std::string> goalClasses(const std::vector<std::string>& requested,
                                            const BinAssignment& bins) {
  if (requested.empty()) {
    return bins.classes();
  }
  std::vector<std::string> out;
  for (const auto& c : requested) {
    if (!bins.knows(c)) {
      throw std::invalid_argument("no bin for class '" + c + "'");
    }
    if (std::find(out.begin(), out.end(), c) == out.end()) {
      out.push_back(c);
    }
  }
  return out;
}

// xy distance.
inline double distanceXY(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
  return (a.head<2>() - b.head<2>()).norm();
}

// Per-object bookkeeping of one goal. Objects are identified by class and position (xy within
// match_radius), because track ids change.
struct ObjectRecord {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  std::string class_name;
  std::string label;  // e.g. "object_7" (the id at the last attempt)
  int attempts{0};
  FailureCategory category{FailureCategory::kNone};
  std::string detail;
};

class SortBook {
 public:
  SortBook(int max_attempts, double match_radius)
      : max_attempts_(max_attempts), match_radius_(match_radius) {
    if (max_attempts_ < 1) {
      throw std::invalid_argument("max_attempts must be >= 1");
    }
    if (match_radius_ <= 0.0) {
      throw std::invalid_argument("match_radius must be > 0");
    }
  }

  int maxAttempts() const { return max_attempts_; }

  bool givenUp(const Eigen::Vector3d& p, const std::string& class_name) const {
    return std::any_of(given_up_.begin(), given_up_.end(), [&](const ObjectRecord& r) {
      return r.class_name == class_name && distanceXY(r.position, p) <= match_radius_;
    });
  }

  // The object that failed its last attempt and must be re-selected first, if any.
  const std::optional<ObjectRecord>& retryTarget() const { return retry_; }

  bool matchesRetry(const Eigen::Vector3d& p, const std::string& class_name) const {
    return retry_ && retry_->class_name == class_name &&
           distanceXY(retry_->position, p) <= match_radius_;
  }

  // Attempt number (0-based) the next attempt on an object at p would be.
  int attemptsSoFar(const Eigen::Vector3d& p, const std::string& class_name) const {
    return matchesRetry(p, class_name) ? retry_->attempts : 0;
  }

  // Records a failed attempt. Returns true if the object is now given up (max_attempts reached).
  bool recordFailure(const Eigen::Vector3d& p, const std::string& class_name,
                     const std::string& label, FailureCategory category,
                     const std::string& detail) {
    ObjectRecord r;
    if (matchesRetry(p, class_name)) {
      r = *retry_;
    }
    r.position = p;
    r.class_name = class_name;
    r.label = label;
    r.attempts += 1;
    r.category = category;
    r.detail = detail;
    if (r.attempts >= max_attempts_) {
      given_up_.push_back(r);
      retry_.reset();
      return true;
    }
    retry_ = r;
    return false;
  }

  // The retried object was placed: forget it.
  void recordSuccess() { retry_.reset(); }

  // Gives up an object without (further) attempts, e.g. unreachable, or the retry target that
  // was not perceived again.
  void giveUp(const Eigen::Vector3d& p, const std::string& class_name, const std::string& label,
              FailureCategory category, const std::string& detail) {
    ObjectRecord r;
    if (matchesRetry(p, class_name)) {
      r = *retry_;
      retry_.reset();
    }
    r.position = p;
    r.class_name = class_name;
    r.label = label;
    r.category = category;
    r.detail = detail;
    given_up_.push_back(r);
  }

  // Gives up the current retry target (e.g. it vanished), keeping its last failure unless a
  // category is given.
  void giveUpRetry(FailureCategory category, const std::string& detail) {
    if (!retry_) {
      return;
    }
    ObjectRecord r = *retry_;
    r.category = category;
    r.detail = detail + " (after " + std::to_string(r.attempts) + " failed attempt" +
               (r.attempts == 1 ? "" : "s") + "; last: " + toString(retry_->category) + ": " +
               retry_->detail + ")";
    given_up_.push_back(r);
    retry_.reset();
  }

  const std::vector<ObjectRecord>& givenUpObjects() const { return given_up_; }

  std::vector<std::string> failureReasons() const {
    std::vector<std::string> out;
    for (const auto& r : given_up_) {
      out.push_back(formatFailureReason(r.label, r.class_name, r.category, r.detail));
    }
    return out;
  }

 private:
  int max_attempts_;
  double match_radius_;
  std::optional<ObjectRecord> retry_;
  std::vector<ObjectRecord> given_up_;
};

// Indices of the objects a goal may pick: class in `classes`, centre not inside any bin
// footprint grown by `bin_margin`, not given up.
/// True if both vectors have the same size and every |current[i] - target[i]| <= tolerance.
inline bool jointsWithin(const std::vector<double>& current, const std::vector<double>& target,
                         double tolerance) {
  if (current.size() != target.size()) return false;
  for (std::size_t i = 0; i < current.size(); ++i) {
    if (!(std::fabs(current[i] - target[i]) <= tolerance)) return false;
  }
  return true;
}

inline std::vector<std::size_t> filterCandidates(const std::vector<Object>& objects,
                                                 const std::vector<std::string>& classes,
                                                 const std::vector<NamedBin>& bins,
                                                 double bin_margin, const SortBook& book) {
  std::vector<std::size_t> out;
  for (std::size_t i = 0; i < objects.size(); ++i) {
    const auto& o = objects[i];
    if (std::find(classes.begin(), classes.end(), o.class_name) == classes.end()) {
      continue;
    }
    if (binContaining(bins, o.center, bin_margin)) {
      continue;
    }
    if (book.givenUp(o.center, o.class_name)) {
      continue;
    }
    out.push_back(i);
  }
  return out;
}

// Candidate indices sorted by xy distance to the arm base, closest first (ties: input order).
inline std::vector<std::size_t> orderByDistance(const std::vector<Object>& objects,
                                                std::vector<std::size_t> indices,
                                                const Eigen::Vector3d& base) {
  std::stable_sort(indices.begin(), indices.end(), [&](std::size_t a, std::size_t b) {
    return distanceXY(objects[a].center, base) < distanceXY(objects[b].center, base);
  });
  return indices;
}

// Rotates a ranked candidate list so it starts at index `first` (mod size): retry k starts with
// the next candidate, and every candidate stays available.
template <typename T>
std::vector<T> rotated(const std::vector<T>& v, std::size_t first) {
  if (v.empty()) {
    return v;
  }
  std::vector<T> out;
  const std::size_t n = v.size();
  for (std::size_t i = 0; i < n; ++i) {
    out.push_back(v[(first + i) % n]);
  }
  return out;
}

// ---------------------------------------------------------------------------------------------
// Perception settling
// ---------------------------------------------------------------------------------------------

class SettleDetector {
 public:
  SettleDetector(double settle_time, double tolerance)
      : settle_time_(settle_time), tolerance_(tolerance) {}

  void reset() {
    ref_.clear();
    window_start_.reset();
  }

  // Feeds one /objects_3d message (time t, id -> position). Returns true once the id set has
  // been unchanged and no object moved more than `tolerance` from its position at the window
  // start for at least `settle_time`.
  bool update(double t, const std::map<std::uint32_t, Eigen::Vector3d>& objects) {
    bool same = window_start_.has_value() && objects.size() == ref_.size();
    if (same) {
      for (const auto& [id, p] : objects) {
        const auto it = ref_.find(id);
        if (it == ref_.end() || (it->second - p).norm() > tolerance_) {
          same = false;
          break;
        }
      }
    }
    if (!same) {
      ref_ = objects;
      window_start_ = t;
      return settle_time_ <= 0.0;
    }
    return t - *window_start_ >= settle_time_;
  }

 private:
  double settle_time_;
  double tolerance_;
  std::map<std::uint32_t, Eigen::Vector3d> ref_;
  std::optional<double> window_start_;
};

// ---------------------------------------------------------------------------------------------
// Release slot inside a bin
// ---------------------------------------------------------------------------------------------

struct ReleaseSlotConfig {
  double grid_step{0.0025};    // slot search grid in the bin frame (m)
  double wall_margin{0.006};   // object and fingers stay this far inside the inner walls (m)
  double clearance_cap{0.05};  // clearances above this are equally good (m)
  // Boxes: max slot offset across the closing axis (m). The footprint along that direction is
  // the least reliable (D-14: the oblique camera sees only the front half of a bottle, so its
  // length is underestimated by up to 3 cm), so by default box slots move only along the closing
  // axis, where the width is the measured grasp width.
  double box_max_across_offset{0.0};
};

struct ReleaseSlot {
  Eigen::Vector2d xy{Eigen::Vector2d::Zero()};  // world xy of the held object's centre (= TCP)
  double clearance{0.0};  // min distance object/fingers to occupied objects, capped
};

// Distance between the footprints of two upright objects (box: rectangle, else circle); 0 if
// they overlap.
inline double footprintDistance(const Object& a, const Object& b) {
  const Eigen::Vector2d ca = a.center.head<2>();
  const Eigen::Vector2d cb = b.center.head<2>();
  const double ra = std::max(a.size.x(), a.size.y()) / 2.0;
  const double rb = std::max(b.size.x(), b.size.y()) / 2.0;
  if (a.shape == kShapeBox && b.shape == kShapeBox) {
    return distance(Rect2{ca, a.size.x() / 2.0, a.size.y() / 2.0, a.yaw},
                    Rect2{cb, b.size.x() / 2.0, b.size.y() / 2.0, b.yaw});
  }
  if (a.shape == kShapeBox) {
    return distance(Rect2{ca, a.size.x() / 2.0, a.size.y() / 2.0, a.yaw}, cb, rb);
  }
  if (b.shape == kShapeBox) {
    return distance(Rect2{cb, b.size.x() / 2.0, b.size.y() / 2.0, b.yaw}, ca, ra);
  }
  return std::max(0.0, (ca - cb).norm() - ra - rb);
}

inline double rectDistanceToObject(const Rect2& r, const Object& o) {
  if (o.shape == kShapeBox) {
    return distance(r, Rect2{o.center.head<2>(), o.size.x() / 2.0, o.size.y() / 2.0, o.yaw});
  }
  return distance(r, o.center.head<2>(), std::max(o.size.x(), o.size.y()) / 2.0);
}

// True if every corner of r is inside the axis-aligned box |x| <= hx, |y| <= hy of the frame
// `local_from_world` (2D).
inline bool rectInside(const Rect2& r, const Eigen::Isometry2d& local_from_world, double hx,
                       double hy) {
  for (const auto& c : r.corners()) {
    const Eigen::Vector2d l = local_from_world * c;
    if (std::abs(l.x()) > hx || std::abs(l.y()) > hy) {
      return false;
    }
  }
  return true;
}

// Chooses where in the bin to release `held` (its yaw and size; centre ignored) held with the
// jaws at world closing yaw `closing_yaw`, opened to `release_gap` at release. Valid slots keep
// the object footprint and both fingers (thickness along the closing axis, width across it)
// inside the inner walls minus wall_margin; for boxes the offset across the closing axis is at
// most box_max_across_offset. Among valid slots on the grid: largest clearance to
// `occupied` (capped at clearance_cap), then farthest from the bin centre (packs objects to the
// side, leaving room for the next one), then grid order. nullopt if no slot is valid.
inline std::optional<ReleaseSlot> chooseReleaseSlot(const BinGeometry& bin, const Object& held,
                                                    double closing_yaw, double release_gap,
                                                    double finger_thickness, double finger_width,
                                                    const std::vector<Object>& occupied,
                                                    const ReleaseSlotConfig& cfg) {
  if (cfg.grid_step <= 0.0) {
    throw std::invalid_argument("release slot grid_step must be > 0");
  }
  const Eigen::Vector2d inner = bin.innerHalf();
  const double hx = inner.x() - cfg.wall_margin;
  const double hy = inner.y() - cfg.wall_margin;
  if (hx <= 0.0 || hy <= 0.0) {
    return std::nullopt;
  }
  const double bin_yaw = std::atan2(bin.pose.linear()(1, 0), bin.pose.linear()(0, 0));
  Eigen::Isometry2d world_from_bin = Eigen::Isometry2d::Identity();
  world_from_bin.linear() = Eigen::Rotation2Dd(bin_yaw).toRotationMatrix();
  world_from_bin.translation() = bin.pose.translation().head<2>();
  const Eigen::Isometry2d bin_from_world = world_from_bin.inverse();
  const Eigen::Vector2d axis(std::cos(closing_yaw), std::sin(closing_yaw));
  const double finger_offset = release_gap / 2.0 + finger_thickness / 2.0;
  const double held_r = std::max(held.size.x(), held.size.y()) / 2.0;

  std::optional<ReleaseSlot> best;
  double best_centre = -1.0;
  // Grid in the closing-axis frame (u along the closing axis, v across it), centred on the bin;
  // slots outside the inner walls are rejected below.
  const Eigen::Vector2d across(-axis.y(), axis.x());
  const double reach = std::hypot(hx, hy);
  const int nu = static_cast<int>(std::floor(reach / cfg.grid_step));
  const double v_max = held.shape == kShapeBox ? std::min(reach, cfg.box_max_across_offset) : reach;
  const int nv = static_cast<int>(std::floor(v_max / cfg.grid_step + 1e-9));
  for (int iu = -nu; iu <= nu; ++iu) {
    for (int iv = -nv; iv <= nv; ++iv) {
      const Eigen::Vector2d p =
          world_from_bin.translation() + iu * cfg.grid_step * axis + iv * cfg.grid_step * across;
      const Eigen::Vector2d local = bin_from_world * p;
      std::vector<Rect2> rects = {
          Rect2{p + finger_offset * axis, finger_thickness / 2.0, finger_width / 2.0, closing_yaw},
          Rect2{p - finger_offset * axis, finger_thickness / 2.0, finger_width / 2.0, closing_yaw}};
      bool inside = true;
      if (held.shape == kShapeBox) {
        rects.push_back(Rect2{p, held.size.x() / 2.0, held.size.y() / 2.0, held.yaw});
      } else {
        inside =
            std::abs(local.x()) + held_r <= hx + 1e-9 && std::abs(local.y()) + held_r <= hy + 1e-9;
      }
      for (const auto& r : rects) {
        inside = inside && rectInside(r, bin_from_world, hx, hy);
      }
      if (!inside) {
        continue;
      }
      Object at = held;
      at.center = Eigen::Vector3d(p.x(), p.y(), held.center.z());
      double clearance = std::numeric_limits<double>::infinity();
      for (const auto& o : occupied) {
        clearance = std::min(clearance, footprintDistance(at, o));
        clearance = std::min(clearance, rectDistanceToObject(rects[0], o));
        clearance = std::min(clearance, rectDistanceToObject(rects[1], o));
      }
      clearance = std::min(clearance, cfg.clearance_cap);
      const double centre = local.norm();
      const double eps = 1e-9;
      if (!best || clearance > best->clearance + eps ||
          (std::abs(clearance - best->clearance) <= eps && centre > best_centre + eps)) {
        best = ReleaseSlot{p, clearance};
        best_centre = centre;
      }
    }
  }
  return best;
}

}  // namespace graspsort::manipulation

#endif  // GRASPSORT_MANIPULATION__SORT_LOGIC_HPP_
