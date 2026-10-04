// ROS-free top-down grasp planner for GraspSort (architecture 7.4).
//
// Units: metres, radians. Frame: world, z up; objects stand upright on the table.
//
// Grasp yaw convention: a candidate's `yaw` is the world-frame angle (about +z, wrapped to
// [-pi, pi)) of the gripper's closing axis, i.e. the direction from the right finger to the left
// finger (gripper_base_link +y in the D-02 gripper). yaw and yaw + pi are the same grasp line with
// the wrist turned 180 deg, so both are distinct candidates. `wrist_yaw` passed to the planner
// uses the same convention. The sort task converts (position, yaw) to a full top-down pose.
//
// Every threshold is a GraspPlannerConfig field (later mapped 1:1 to ROS parameters).
#ifndef GRASPSORT_MANIPULATION__GRASP_PLANNER_HPP_
#define GRASPSORT_MANIPULATION__GRASP_PLANNER_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace graspsort::manipulation {

// Shape codes, identical to graspsort_msgs/ObjectPose.shape.
inline constexpr std::uint8_t kShapeCylinder = 0;
inline constexpr std::uint8_t kShapeBox = 1;

inline constexpr double kPi = EIGEN_PI;

// Upright object in the world frame (mirrors graspsort_msgs/ObjectPose).
struct Object {
  std::uint32_t id{0};
  std::string class_name;
  Eigen::Vector3d center{Eigen::Vector3d::Zero()};  // centre of the object (m)
  // x, y: footprint sides along the object's yaw; z: height (m). Cylinder: x = y = diameter.
  Eigen::Vector3d size{Eigen::Vector3d::Zero()};
  double yaw{0.0};  // rad; ignored for cylinders
  std::uint8_t shape{kShapeCylinder};
};

struct GraspPlannerConfig {
  // Number of evenly spaced closing-axis yaws for a cylinder. Architecture: 8.
  std::size_t cylinder_yaw_count{8};
  // Grasp point (between the finger pads) at this fraction of the object height above its
  // bottom. Architecture: fingers at 60% of height.
  double finger_height_fraction{0.6};
  // Gripper opening when fully open (m). D-02: 90 mm (2 x open_half 0.045).
  double max_opening{0.09};
  // Reject if object width > max_opening - width_margin (m).
  double width_margin{0.01};
  // Safety check (D-14): reject a candidate if the object footprint's extent along its closing
  // axis exceeds this (m). Independent of the candidate generation, so a mis-oriented candidate
  // or estimate can never command the fingers onto an object wider than this.
  double max_grasp_width{0.085};
  // Finger pad footprint (m), D-02 URDF: finger_t along the closing axis, finger_w across it.
  double finger_thickness{0.01};
  double finger_width{0.02};
  // Reject if an open finger is closer than this to a neighbour footprint (m). Overlap is always
  // rejected.
  double min_finger_clearance{0.005};
  // Clearance above this (m) gives no extra ranking benefit; also the reported clearance when
  // there are no neighbours.
  double clearance_cap{0.10};
  // Ranking cost = yaw_weight * |yaw - wrist_yaw| (rad, wrapped)
  //              - clearance_weight * min(clearance, clearance_cap) (m). Lower is better.
  double yaw_weight{1.0};
  double clearance_weight{10.0};
  // Costs closer than this are a tie; ties keep generation order.
  double cost_tie_tolerance{1e-9};
};

enum class Rejection : std::uint8_t {
  kNone = 0,
  kTooWide,
  kFingerCollision,
  kExceedsMaxGraspWidth
};

inline const char* toString(Rejection r) {
  switch (r) {
    case Rejection::kNone:
      return "none";
    case Rejection::kTooWide:
      return "object wider than gripper opening";
    case Rejection::kFingerCollision:
      return "finger would hit a neighbouring object";
    case Rejection::kExceedsMaxGraspWidth:
      return "footprint across the fingers exceeds max_grasp_width";
  }
  return "unknown";
}

struct GraspCandidate {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};  // grasp point between the pads (m)
  double yaw{0.0};                                    // closing-axis yaw, [-pi, pi)
  double width{0.0};                                  // object width across the closing axis (m)
  double clearance{0.0};  // min finger-to-neighbour distance, capped at clearance_cap (m)
  double cost{0.0};       // ranking cost (lower is better)
  std::size_t index{0};   // generation order
  Rejection rejection{Rejection::kNone};
};

// ---------------------------------------------------------------------------------------------
// 2D geometry helpers
// ---------------------------------------------------------------------------------------------

// Wraps an angle to [-pi, pi).
inline double wrapToPi(double a) {
  const double two_pi = 2.0 * kPi;
  double w = std::fmod(a + kPi, two_pi);
  if (w < 0.0) {
    w += two_pi;
  }
  w -= kPi;
  if (w >= kPi) {
    w -= two_pi;
  }
  return w;
}

// Oriented rectangle in the table plane.
struct Rect2 {
  Eigen::Vector2d center{Eigen::Vector2d::Zero()};
  double half_x{0.0};
  double half_y{0.0};
  double yaw{0.0};

  std::array<Eigen::Vector2d, 4> corners() const {
    const Eigen::Rotation2Dd r(yaw);
    return {center + r * Eigen::Vector2d(half_x, half_y),
            center + r * Eigen::Vector2d(-half_x, half_y),
            center + r * Eigen::Vector2d(-half_x, -half_y),
            center + r * Eigen::Vector2d(half_x, -half_y)};
  }
};

inline double pointSegmentDistance(const Eigen::Vector2d& p, const Eigen::Vector2d& a,
                                   const Eigen::Vector2d& b) {
  const Eigen::Vector2d ab = b - a;
  const double l2 = ab.squaredNorm();
  const double t = l2 > 0.0 ? std::clamp((p - a).dot(ab) / l2, 0.0, 1.0) : 0.0;
  return (a + t * ab - p).norm();
}

// Separating-axis test for two rectangles (touching counts as overlap).
inline bool overlaps(const Rect2& a, const Rect2& b) {
  const auto ca = a.corners();
  const auto cb = b.corners();
  for (const double yaw : {a.yaw, a.yaw + kPi / 2.0, b.yaw, b.yaw + kPi / 2.0}) {
    const Eigen::Vector2d axis(std::cos(yaw), std::sin(yaw));
    double amin = std::numeric_limits<double>::infinity();
    double amax = -amin;
    double bmin = amin;
    double bmax = -amin;
    for (const auto& c : ca) {
      amin = std::min(amin, c.dot(axis));
      amax = std::max(amax, c.dot(axis));
    }
    for (const auto& c : cb) {
      bmin = std::min(bmin, c.dot(axis));
      bmax = std::max(bmax, c.dot(axis));
    }
    if (amax < bmin || bmax < amin) {
      return false;
    }
  }
  return true;
}

// Distance between two rectangles; 0 if they overlap.
inline double distance(const Rect2& a, const Rect2& b) {
  if (overlaps(a, b)) {
    return 0.0;
  }
  const auto ca = a.corners();
  const auto cb = b.corners();
  double d = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < 4; ++i) {
    for (std::size_t j = 0; j < 4; ++j) {
      d = std::min(d, pointSegmentDistance(ca[i], cb[j], cb[(j + 1) % 4]));
      d = std::min(d, pointSegmentDistance(cb[i], ca[j], ca[(j + 1) % 4]));
    }
  }
  return d;
}

// Distance from a rectangle to a circle; 0 if they overlap.
inline double distance(const Rect2& r, const Eigen::Vector2d& circle_center, double radius) {
  const Eigen::Vector2d local = Eigen::Rotation2Dd(-r.yaw) * (circle_center - r.center);
  const Eigen::Vector2d closest(std::clamp(local.x(), -r.half_x, r.half_x),
                                std::clamp(local.y(), -r.half_y, r.half_y));
  return std::max(0.0, (local - closest).norm() - radius);
}

// Width (m) the gripper must span to grasp the object (box: short side; cylinder: diameter).
inline double graspWidth(const Object& o) {
  return o.shape == kShapeBox ? std::min(o.size.x(), o.size.y()) : std::max(o.size.x(), o.size.y());
}

// Extent (m) of the object's footprint along a closing axis at world yaw `closing_yaw`: for a box
// the projection of its rectangle (|sx cos d| + |sy sin d|, d = closing_yaw - object yaw), for a
// cylinder its diameter.
inline double footprintWidthAcross(const Object& o, double closing_yaw) {
  if (o.shape != kShapeBox) {
    return std::max(o.size.x(), o.size.y());
  }
  const double d = closing_yaw - o.yaw;
  return std::abs(o.size.x() * std::cos(d)) + std::abs(o.size.y() * std::sin(d));
}

// ---------------------------------------------------------------------------------------------
// Planner
// ---------------------------------------------------------------------------------------------

class GraspPlanner {
 public:
  explicit GraspPlanner(const GraspPlannerConfig& cfg = {}) : cfg_(cfg) {
    if (cfg_.cylinder_yaw_count == 0) {
      throw std::invalid_argument("cylinder_yaw_count must be >= 1");
    }
    if (cfg_.finger_height_fraction < 0.0 || cfg_.finger_height_fraction > 1.0) {
      throw std::invalid_argument("finger_height_fraction must be in [0, 1]");
    }
    if (cfg_.max_opening <= 0.0 || cfg_.finger_thickness <= 0.0 || cfg_.finger_width <= 0.0) {
      throw std::invalid_argument("gripper dimensions must be > 0");
    }
    if (cfg_.max_grasp_width <= 0.0) {
      throw std::invalid_argument("max_grasp_width must be > 0");
    }
  }

  const GraspPlannerConfig& config() const { return cfg_; }

  // Unfiltered, unranked candidates in generation order.
  // Box: closing axis along the short side, and the same + pi. Cylinder: cylinder_yaw_count
  // yaws k * 2 pi / n (object yaw ignored). Position: object centre x, y; z at
  // finger_height_fraction of the height above the object's bottom.
  std::vector<GraspCandidate> generateCandidates(const Object& o) const {
    std::vector<double> yaws;
    if (o.shape == kShapeBox) {
      const double short_axis = o.size.x() <= o.size.y() ? o.yaw : o.yaw + kPi / 2.0;
      yaws = {wrapToPi(short_axis), wrapToPi(short_axis + kPi)};
    } else {
      const double step = 2.0 * kPi / static_cast<double>(cfg_.cylinder_yaw_count);
      for (std::size_t k = 0; k < cfg_.cylinder_yaw_count; ++k) {
        yaws.push_back(wrapToPi(static_cast<double>(k) * step));
      }
    }
    const double bottom = o.center.z() - o.size.z() / 2.0;
    std::vector<GraspCandidate> out;
    for (std::size_t i = 0; i < yaws.size(); ++i) {
      GraspCandidate c;
      c.position = {o.center.x(), o.center.y(), bottom + cfg_.finger_height_fraction * o.size.z()};
      c.yaw = yaws[i];
      c.width = graspWidth(o);
      c.index = i;
      out.push_back(c);
    }
    return out;
  }

  // Footprints of the two fully open fingers of a candidate: [0] at +closing axis (left),
  // [1] at -closing axis (right). Inner faces at +-max_opening / 2.
  std::array<Rect2, 2> fingerFootprints(const GraspCandidate& c) const {
    const Eigen::Vector2d axis(std::cos(c.yaw), std::sin(c.yaw));
    const double offset = cfg_.max_opening / 2.0 + cfg_.finger_thickness / 2.0;
    const Eigen::Vector2d p = c.position.head<2>();
    return {Rect2{p + offset * axis, cfg_.finger_thickness / 2.0, cfg_.finger_width / 2.0, c.yaw},
            Rect2{p - offset * axis, cfg_.finger_thickness / 2.0, cfg_.finger_width / 2.0, c.yaw}};
  }

  // Smallest distance (m) from the open fingers to any neighbour footprint (box: rectangle,
  // cylinder: circle); +inf without neighbours. Objects with target.id are skipped.
  double fingerClearance(const GraspCandidate& c, const Object& target,
                         const std::vector<Object>& scene) const {
    double d = std::numeric_limits<double>::infinity();
    for (const auto& finger : fingerFootprints(c)) {
      for (const auto& n : scene) {
        if (n.id == target.id) {
          continue;
        }
        const Eigen::Vector2d nc = n.center.head<2>();
        if (n.shape == kShapeBox) {
          d = std::min(d, distance(finger, Rect2{nc, n.size.x() / 2.0, n.size.y() / 2.0, n.yaw}));
        } else {
          d = std::min(d, distance(finger, nc, std::max(n.size.x(), n.size.y()) / 2.0));
        }
      }
    }
    return d;
  }

  // All candidates with rejection reason, clearance and cost, ranked: accepted first by cost
  // (ties keep generation order), then rejected in generation order. `scene` may contain the
  // target itself (skipped by id).
  std::vector<GraspCandidate> evaluate(const Object& target, const std::vector<Object>& scene,
                                       double wrist_yaw) const {
    auto cands = generateCandidates(target);
    for (auto& c : cands) {
      const double raw = fingerClearance(c, target, scene);
      c.clearance = std::min(raw, cfg_.clearance_cap);
      c.cost = cfg_.yaw_weight * std::abs(wrapToPi(c.yaw - wrist_yaw)) -
               cfg_.clearance_weight * c.clearance;
      if (footprintWidthAcross(target, c.yaw) > cfg_.max_grasp_width) {
        c.rejection = Rejection::kExceedsMaxGraspWidth;
      } else if (c.width > cfg_.max_opening - cfg_.width_margin) {
        c.rejection = Rejection::kTooWide;
      } else if (raw <= 0.0 || raw < cfg_.min_finger_clearance) {
        c.rejection = Rejection::kFingerCollision;
      }
    }
    const double tol = cfg_.cost_tie_tolerance;
    std::stable_sort(cands.begin(), cands.end(),
                     [tol](const GraspCandidate& a, const GraspCandidate& b) {
                       const bool ra = a.rejection != Rejection::kNone;
                       const bool rb = b.rejection != Rejection::kNone;
                       if (ra != rb) {
                         return !ra;
                       }
                       if (ra) {
                         return false;  // rejected: keep generation order
                       }
                       return a.cost < b.cost - tol;
                     });
    return cands;
  }

  // Accepted candidates only, best first. Empty if every candidate is rejected.
  std::vector<GraspCandidate> plan(const Object& target, const std::vector<Object>& scene,
                                   double wrist_yaw) const {
    auto all = evaluate(target, scene, wrist_yaw);
    all.erase(
        std::remove_if(all.begin(), all.end(),
                       [](const GraspCandidate& c) { return c.rejection != Rejection::kNone; }),
        all.end());
    return all;
  }

 private:
  GraspPlannerConfig cfg_;
};

}  // namespace graspsort::manipulation

#endif  // GRASPSORT_MANIPULATION__GRASP_PLANNER_HPP_
