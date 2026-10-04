// ROS-free pick-and-place geometry for GraspSort (architecture 7.5).
//
// Units: metres, radians. Frame: world, z up. Objects stand upright on a horizontal support.
//
// Gripper conventions (D-02 gripper, graspsort_robot.urdf.xacro):
// - gripper_tcp is the grasp point between the finger pads. Its z axis points along the fingers,
//   so for a top-down grasp it points straight down (world -z).
// - The fingers close along gripper_base_link y (= gripper_tcp y). A grasp candidate's yaw
//   (grasp_planner.hpp) is the world yaw of that closing axis, so the TCP y axis is
//   (cos yaw, sin yaw, 0) and x = y cross z = (-sin yaw, cos yaw, 0).
// - The fingertips are `tcp_to_fingertip` below the TCP along its z axis.
// - Finger joint position q: 0 = fully open (gap = open_width), gap = open_width - 2 q.
//
// Every value is a field of PickPlaceConfig (mapped 1:1 to ROS parameters by pick_place_test).
#ifndef GRASPSORT_MANIPULATION__PICK_PLACE_GEOMETRY_HPP_
#define GRASPSORT_MANIPULATION__PICK_PLACE_GEOMETRY_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>

namespace graspsort::manipulation {

struct PickPlaceConfig {
  // TCP height of the pre-grasp above the grasp (architecture 7.5: 10 cm).
  double pregrasp_height{0.10};
  // Straight-line lift of the TCP after the grasp.
  double lift_height{0.15};
  // Straight-line retreat of the TCP after the release.
  double retreat_height{0.10};
  // Fingertips must stay at least this far above the object's bottom (= the support surface).
  double min_fingertip_clearance{0.01};
  // The fingertips must reach at least down to the object's mid-height plus this (negative =
  // below the centre). A sphere is held only if the pads reach below its equator.
  double max_fingertip_above_center{0.0};
  // Release: the held object's bottom this far above the bin floor's top surface.
  double release_clearance{0.02};
  // Transport: the held object's bottom this far above the bin rim at the above-bin pose.
  double transport_clearance{0.05};
  // Gripper (D-02).
  double tcp_to_fingertip{0.02};     // fingertips below the TCP along the fingers
  double open_width{0.09};           // jaw gap at finger position 0
  double max_close_position{0.038};  // never command beyond this (stroke 0.04 = fully closed)
  double squeeze{0.0005};            // close to object width minus this (D-02)
};

// Bin geometry (graspsort_gazebo world_layout.yaml): the bin frame is the centre of its bottom
// face; size = outer x, y, height.
struct BinGeometry {
  Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
  Eigen::Vector3d size{0.20, 0.20, 0.08};
  double wall_thickness{0.01};
  double floor_thickness{0.01};

  double floorTopZ() const { return pose.translation().z() + floor_thickness; }
  double rimZ() const { return pose.translation().z() + size.z(); }
  // Half extents of the inner area (inside the walls).
  Eigen::Vector2d innerHalf() const {
    return {size.x() / 2.0 - wall_thickness, size.y() / 2.0 - wall_thickness};
  }
  // True if a world point is inside the inner area in xy, shrunk by `margin` on every side.
  bool insideInner(const Eigen::Vector3d& p, double margin = 0.0) const {
    const Eigen::Vector3d local = pose.inverse() * p;
    const Eigen::Vector2d h = innerHalf();
    return std::abs(local.x()) <= h.x() - margin && std::abs(local.y()) <= h.y() - margin;
  }
};

inline void validate(const PickPlaceConfig& c) {
  if (c.pregrasp_height <= 0.0 || c.lift_height <= 0.0 || c.retreat_height <= 0.0) {
    throw std::invalid_argument("pregrasp/lift/retreat heights must be > 0");
  }
  if (c.tcp_to_fingertip < 0.0 || c.open_width <= 0.0 || c.max_close_position <= 0.0) {
    throw std::invalid_argument("gripper geometry must be positive");
  }
  if (c.min_fingertip_clearance < 0.0 || c.release_clearance < 0.0 || c.transport_clearance < 0.0 ||
      c.squeeze < 0.0) {
    throw std::invalid_argument("clearances and squeeze must be >= 0");
  }
}

// Orientation of gripper_tcp for a top-down grasp whose closing axis has world yaw `closing_yaw`.
inline Eigen::Quaterniond topDownOrientation(double closing_yaw) {
  const double c = std::cos(closing_yaw);
  const double s = std::sin(closing_yaw);
  Eigen::Matrix3d r;
  r.col(0) = Eigen::Vector3d(-s, c, 0.0);
  r.col(1) = Eigen::Vector3d(c, s, 0.0);
  r.col(2) = Eigen::Vector3d(0.0, 0.0, -1.0);
  return Eigen::Quaterniond(r).normalized();
}

// World yaw of the closing axis (TCP y axis projected on the table plane) for any TCP orientation.
inline double closingYaw(const Eigen::Quaterniond& tcp) {
  const Eigen::Vector3d y = tcp.toRotationMatrix().col(1);
  return std::atan2(y.y(), y.x());
}

inline Eigen::Isometry3d makePose(const Eigen::Vector3d& p, const Eigen::Quaterniond& q) {
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.linear() = q.toRotationMatrix();
  t.translation() = p;
  return t;
}

// Translates a pose straight up (world z).
inline Eigen::Isometry3d raised(const Eigen::Isometry3d& pose, double dz) {
  Eigen::Isometry3d t = pose;
  t.translation().z() += dz;
  return t;
}

// TCP height for an upright object of centre z `center_z` and height `height`, starting from the
// planner's grasp z: lowered if the fingertips would not reach the object's mid-height (+
// max_fingertip_above_center), but never so low that the fingertips come closer than
// min_fingertip_clearance to the object's bottom. nullopt if both cannot hold.
inline std::optional<double> adjustedGraspZ(double planner_z, double center_z, double height,
                                            const PickPlaceConfig& c) {
  const double bottom = center_z - height / 2.0;
  const double z_min = bottom + c.min_fingertip_clearance + c.tcp_to_fingertip;
  const double z_max = center_z + c.max_fingertip_above_center + c.tcp_to_fingertip;
  if (z_min > z_max) {
    return std::nullopt;
  }
  return std::clamp(planner_z, z_min, z_max);
}

// Vertical distance from the TCP down to the bottom of the held object.
inline double hangBelowTcp(double grasp_tcp_z, double center_z, double height) {
  return grasp_tcp_z - (center_z - height / 2.0);
}

// Finger joint position that closes the jaws to `object_width - squeeze` (D-02). nullopt if the
// object is wider than the opening, or if reaching that gap would need more than
// max_close_position (never close fully).
inline std::optional<double> closePosition(double object_width, const PickPlaceConfig& c) {
  const double gap = object_width - c.squeeze;
  const double q = (c.open_width - gap) / 2.0;
  if (object_width <= 0.0 || q < 0.0 || q > c.max_close_position) {
    return std::nullopt;
  }
  return q;
}

// TCP pose that releases the held object above the bin floor: the bin centre in xy, the held
// object's bottom release_clearance above the floor top, orientation `tcp_orientation`.
inline Eigen::Isometry3d releasePose(const BinGeometry& bin, double hang,
                                     const Eigen::Quaterniond& tcp_orientation,
                                     const PickPlaceConfig& c) {
  Eigen::Vector3d p = bin.pose.translation();
  p.z() = bin.floorTopZ() + c.release_clearance + hang;
  return makePose(p, tcp_orientation);
}

// TCP pose above the bin: the held object's bottom transport_clearance above the rim.
inline Eigen::Isometry3d aboveBinPose(const BinGeometry& bin, double hang,
                                      const Eigen::Quaterniond& tcp_orientation,
                                      const PickPlaceConfig& c) {
  Eigen::Vector3d p = bin.pose.translation();
  p.z() = bin.rimZ() + c.transport_clearance + hang;
  return makePose(p, tcp_orientation);
}

// True if the open jaws (gap open_width, finger thickness `finger_thickness` outside the gap)
// stay inside the bin's inner area at the release pose with yaw `closing_yaw`, so the fingers
// can never hit the walls while lowering. finger_width is the pad size across the closing axis.
inline bool openFingersFitInBin(const BinGeometry& bin, double closing_yaw, double finger_thickness,
                                double finger_width, const PickPlaceConfig& c) {
  const double half_span = c.open_width / 2.0 + finger_thickness;
  const double half_w = finger_width / 2.0;
  const double d = closing_yaw - std::atan2(bin.pose.linear()(1, 0), bin.pose.linear()(0, 0));
  const double ex = std::abs(half_span * std::cos(d)) + std::abs(half_w * std::sin(d));
  const double ey = std::abs(half_span * std::sin(d)) + std::abs(half_w * std::cos(d));
  const Eigen::Vector2d h = bin.innerHalf();
  return ex <= h.x() && ey <= h.y();
}

}  // namespace graspsort::manipulation

#endif  // GRASPSORT_MANIPULATION__PICK_PLACE_GEOMETRY_HPP_
