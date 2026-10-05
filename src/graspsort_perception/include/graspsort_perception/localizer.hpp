// ROS-free per-detection localization for object_localizer_node (architecture 7.2 as amended by
// D-12). Builds on projection.hpp.
//
// D-12 (oblique camera, D-05): the object depth is the 20th-percentile depth in the central part
// of the box (as 7.2); the object's points are the pixels within +/- depth_band of that depth,
// with a WIDE band (default 0.10 m instead of 7.2's 0.02 m) so the whole visible object is kept;
// the position is the centre of the table-plane footprint (x, y) at z = table height + height / 2
// (instead of 7.2's back-projection + push-in, which is centimetres off for tall objects seen
// obliquely). Footprint size and yaw come from cv::minAreaRect as in 7.2.
//
// D-20 (overnight): for a class with a known footprint (KnownFootprintTable, e.g. the bottle),
// the known rectangle is fitted to the visible points (fitKnownFootprint): minAreaRect axes, the
// known size decides which axis is long, and the centre is anchored on the camera side. The oblique
// camera sees only the near half of an object (D-14), so minAreaRect undersizes it along the view
// and tilts its yaw; the undersized width made the gripper close inside the bottle.
//
// Units: metres, radians, pixels. Frames: optical (camera) and world (z up).
#ifndef GRASPSORT_PERCEPTION__LOCALIZER_HPP_
#define GRASPSORT_PERCEPTION__LOCALIZER_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <graspsort_perception/projection.hpp>
#include <limits>
#include <map>
#include <opencv2/core.hpp>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace graspsort::perception {

// Values of graspsort_msgs/ObjectPose.shape (architecture section 6).
enum class ObjectShape : std::uint8_t { kCylinder = 0, kBox = 1 };

// Known table-plane footprint of a box-shaped class (m): size_x is the long side, size_y the short.
struct KnownFootprint {
  double size_x{0.0};
  double size_y{0.0};
};

// D-20 known-footprint fit.
struct KnownFitConfig {
  // The visible extent may exceed a known side by this much (m): depth noise and edge pixels.
  double max_overflow{0.015};
};

struct LocalizerConfig {
  DepthSamplingConfig depth{};
  // depth_band is overridden to the D-12 value in the constructor below.
  FootprintConfig footprint{};
  KnownFitConfig known_fit{};
  // Height of the table top in the world frame (m); world_layout.yaml table.z.
  double table_height{0.75};
  // Default D-12 band (m).
  static constexpr double kD12DepthBand = 0.10;

  LocalizerConfig() { footprint.depth_band = kD12DepthBand; }
};

// Detection box (vision_msgs/BoundingBox2D centre and size, pixels) -> pixel rectangle. Inverse of
// object_detector_node's conversion, which publishes a cv::Rect (x, y, w, h) covering pixels
// x .. x + w - 1 as centre (x + w / 2, y + h / 2) and size (w, h). Non-finite values or a size
// that rounds to zero give an empty rectangle.
inline cv::Rect boxFromCenterSize(double cx, double cy, double w, double h) {
  if (!std::isfinite(cx) || !std::isfinite(cy) || !std::isfinite(w) || !std::isfinite(h)) {
    return {};
  }
  const auto width = static_cast<int>(std::lround(w));
  const auto height = static_cast<int>(std::lround(h));
  if (width <= 0 || height <= 0) {
    return {};
  }
  return {static_cast<int>(std::lround(cx - w / 2.0)), static_cast<int>(std::lround(cy - h / 2.0)),
          width, height};
}

// D-20: fits a rectangle of the known size to the table-plane points `xy` of an object seen from
// `camera_xy` (camera position on the table plane).
// 1. Axes: cv::minAreaRect of the points (as 7.2). On the real bottle its axes are right to within
//    the yaw error of the visible shape, but which side is the long one is not: seen end-on, the
//    visible part (short face plus a strip of the top) is longer across the view than along it.
// 2. Long axis: each of the two axes is tried as the long side (a). The visible extents (du along
//    a, dv along b) must fit the known rectangle within cfg.max_overflow. From a raised camera
//    every face that points towards the camera is fully visible, so the hypothesis is scored by how
//    far the extent of each camera-facing face differs from its known length, weighted by how
//    directly the face points at the camera (dir: unit vector from the centroid to the camera):
//      |dir . b| * |du - size_x| / size_x + |dir . a| * |dv - size_y| / size_y + overflow / size_x
// 3. Centre: along an axis whose extent is shorter than the known side, the rectangle is anchored
//    on the camera side, so the hidden part lies away from the camera.
// size_x/size_y are the known sizes, yaw is the long-side angle in [-pi/2, pi/2). nullopt if `xy`
// is empty or neither hypothesis fits (e.g. two merged objects). Throws std::invalid_argument
// unless size_x >= size_y > 0 and max_overflow >= 0.
inline std::optional<Footprint> fitKnownFootprint(const std::vector<cv::Point2f>& xy,
                                                  const Eigen::Vector2d& camera_xy,
                                                  const KnownFootprint& known,
                                                  const KnownFitConfig& cfg, double height) {
  if (!(known.size_x > 0.0) || !(known.size_y > 0.0) || known.size_x < known.size_y) {
    throw std::invalid_argument("known footprint needs size_x >= size_y > 0");
  }
  if (!(cfg.max_overflow >= 0.0)) {
    throw std::invalid_argument("known footprint fit max_overflow must be >= 0");
  }
  if (xy.empty()) {
    return std::nullopt;
  }
  Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
  for (const auto& p : xy) {
    centroid += Eigen::Vector2d(p.x, p.y);
  }
  centroid /= static_cast<double>(xy.size());
  Eigen::Vector2d dir = camera_xy - centroid;
  dir = dir.norm() > 0.0 ? Eigen::Vector2d(dir.normalized()) : Eigen::Vector2d::Zero();

  const double pi = EIGEN_PI;
  const double axis0 = static_cast<double>(cv::minAreaRect(xy).angle) * pi / 180.0;
  const double lx = known.size_x;
  const double ly = known.size_y;
  struct Hypothesis {
    double cost{0.0};
    double yaw{0.0};
    double umin{0.0}, umax{0.0}, vmin{0.0}, vmax{0.0};
  };
  std::optional<Hypothesis> best;
  for (const double yaw : {axis0, axis0 + pi / 2.0}) {
    const Eigen::Vector2d a(std::cos(yaw), std::sin(yaw));
    const Eigen::Vector2d b(-a.y(), a.x());
    Hypothesis h;
    h.yaw = yaw;
    h.umin = h.vmin = std::numeric_limits<double>::infinity();
    h.umax = h.vmax = -std::numeric_limits<double>::infinity();
    for (const auto& p : xy) {
      const Eigen::Vector2d q = Eigen::Vector2d(p.x, p.y) - centroid;
      h.umin = std::min(h.umin, q.dot(a));
      h.umax = std::max(h.umax, q.dot(a));
      h.vmin = std::min(h.vmin, q.dot(b));
      h.vmax = std::max(h.vmax, q.dot(b));
    }
    const double du = h.umax - h.umin;
    const double dv = h.vmax - h.vmin;
    const double overflow = std::max(0.0, du - lx) + std::max(0.0, dv - ly);
    if (overflow > cfg.max_overflow) {
      continue;
    }
    h.cost = std::abs(dir.dot(b)) * std::abs(du - lx) / lx +
             std::abs(dir.dot(a)) * std::abs(dv - ly) / ly + overflow / lx;
    if (!best || h.cost < best->cost) {
      best = h;
    }
  }
  if (!best) {
    return std::nullopt;
  }
  const Eigen::Vector2d a(std::cos(best->yaw), std::sin(best->yaw));
  const Eigen::Vector2d b(-a.y(), a.x());
  // Centre along one axis: the middle if the whole side is visible, else anchored on the camera
  // side (the near edge is seen, the far edge is hidden).
  const auto centre = [](double lo, double hi, double side, double towards_camera) {
    if (hi - lo >= side) {
      return (lo + hi) / 2.0;
    }
    return towards_camera >= 0.0 ? hi - side / 2.0 : lo + side / 2.0;
  };
  Footprint f;
  f.center_xy = centroid + centre(best->umin, best->umax, lx, dir.dot(a)) * a +
                centre(best->vmin, best->vmax, ly, dir.dot(b)) * b;
  f.size_x = lx;
  f.size_y = ly;
  f.yaw = wrapHalfPi(best->yaw);
  f.height = height;
  f.num_points = xy.size();
  return f;
}

// One frame's estimate for one detection (D-12). With `known` (D-20), the known footprint is
// fitted to the points (fitKnownFootprint) and the minAreaRect result is used only if that fit
// fails (then *known_fit_failed is set, if given). nullopt if the box has no valid depth or too
// few object points above the table. Throws
// std::invalid_argument unless depth is CV_32FC1.
inline std::optional<ObjectSample> localizeBox(const cv::Mat& depth, const cv::Rect& box,
                                               const CameraIntrinsics& k,
                                               const Eigen::Isometry3d& world_T_optical,
                                               const LocalizerConfig& cfg, double confidence,
                                               const std::optional<KnownFootprint>& known = {},
                                               bool* known_fit_failed = nullptr) {
  const auto object_depth = percentileDepth(depth, box, cfg.depth);
  if (!object_depth) {
    return std::nullopt;
  }
  const auto points_world =
      transformPoints(world_T_optical, bandPoints(depth, box, k, *object_depth, cfg.footprint));
  auto f = footprintFromWorldPoints(points_world, cfg.table_height, cfg.footprint);
  if (!f) {
    return std::nullopt;
  }
  if (known) {
    const auto pts = footprintPoints(points_world, cfg.table_height, cfg.footprint);
    const auto fit = fitKnownFootprint(pts->xy, world_T_optical.translation().head<2>(), *known,
                                       cfg.known_fit, f->height);
    if (fit) {
      f = fit;
    } else if (known_fit_failed != nullptr) {
      *known_fit_failed = true;
    }
  }
  ObjectSample s;
  s.position = {f->center_xy.x(), f->center_xy.y(), cfg.table_height + f->height / 2.0};
  s.size_x = f->size_x;
  s.size_y = f->size_y;
  s.height = f->height;
  s.yaw = f->yaw;
  s.confidence = confidence;
  return s;
}

// Cylinders (and spheres, reported as cylinders) are rotationally symmetric: yaw is set to 0 and
// both footprint sizes to the longer side, which is the diameter (the visible half of a round
// object gives a full-width but shallower rectangle). Boxes are returned unchanged.
inline ObjectSample applyShape(ObjectSample s, ObjectShape shape) {
  if (shape == ObjectShape::kCylinder) {
    const double diameter = std::max(s.size_x, s.size_y);
    s.size_x = diameter;
    s.size_y = diameter;
    s.yaw = 0.0;
  }
  return s;
}

// Class name -> shape, from two parallel parameter arrays (ROS parameters cannot hold a map).
class ShapeTable {
 public:
  // Throws std::invalid_argument on length mismatch, empty or duplicate class names, or shape
  // values other than 0 (cylinder) / 1 (box).
  ShapeTable(const std::vector<std::string>& classes, const std::vector<std::int64_t>& shapes) {
    if (classes.size() != shapes.size()) {
      throw std::invalid_argument("shape_classes and shape_types must have the same length");
    }
    for (std::size_t i = 0; i < classes.size(); ++i) {
      if (classes[i].empty()) {
        throw std::invalid_argument("empty class name in shape_classes");
      }
      if (shapes[i] != static_cast<std::int64_t>(ObjectShape::kCylinder) &&
          shapes[i] != static_cast<std::int64_t>(ObjectShape::kBox)) {
        throw std::invalid_argument("shape_types values must be 0 (cylinder) or 1 (box): '" +
                                    classes[i] + "'");
      }
      if (!table_.emplace(classes[i], static_cast<ObjectShape>(shapes[i])).second) {
        throw std::invalid_argument("duplicate class in shape_classes: '" + classes[i] + "'");
      }
    }
  }

  // nullopt for a class without a configured shape.
  std::optional<ObjectShape> find(const std::string& class_name) const {
    const auto it = table_.find(class_name);
    if (it == table_.end()) {
      return std::nullopt;
    }
    return it->second;
  }

 private:
  std::map<std::string, ObjectShape> table_;
};

// D-20: class name -> known footprint, from three parallel parameter arrays. Empty arrays = no
// class has a known footprint (minAreaRect for all, as before D-20).
class KnownFootprintTable {
 public:
  // Throws std::invalid_argument on length mismatch, empty or duplicate class names, or sizes
  // that are not size_x >= size_y > 0.
  KnownFootprintTable(const std::vector<std::string>& classes, const std::vector<double>& size_x,
                      const std::vector<double>& size_y) {
    if (classes.size() != size_x.size() || classes.size() != size_y.size()) {
      throw std::invalid_argument(
          "known_footprint classes, size_x and size_y must have the same length");
    }
    for (std::size_t i = 0; i < classes.size(); ++i) {
      if (classes[i].empty()) {
        throw std::invalid_argument("empty class name in known_footprint.classes");
      }
      if (!(size_x[i] > 0.0) || !(size_y[i] > 0.0) || size_x[i] < size_y[i]) {
        throw std::invalid_argument("known footprint of '" + classes[i] +
                                    "' needs size_x >= size_y > 0");
      }
      if (!table_.emplace(classes[i], KnownFootprint{size_x[i], size_y[i]}).second) {
        throw std::invalid_argument("duplicate class in known_footprint.classes: '" + classes[i] +
                                    "'");
      }
    }
  }

  // nullopt for a class without a known footprint.
  std::optional<KnownFootprint> find(const std::string& class_name) const {
    const auto it = table_.find(class_name);
    if (it == table_.end()) {
      return std::nullopt;
    }
    return it->second;
  }

 private:
  std::map<std::string, KnownFootprint> table_;
};

}  // namespace graspsort::perception

#endif  // GRASPSORT_PERCEPTION__LOCALIZER_HPP_
