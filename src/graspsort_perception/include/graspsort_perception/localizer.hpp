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
#include <map>
#include <opencv2/core.hpp>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace graspsort::perception {

// Values of graspsort_msgs/ObjectPose.shape (architecture section 6).
enum class ObjectShape : std::uint8_t { kCylinder = 0, kBox = 1 };

struct LocalizerConfig {
  DepthSamplingConfig depth{};
  // depth_band is overridden to the D-12 value in the constructor below.
  FootprintConfig footprint{};
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

// One frame's estimate for one detection (D-12). nullopt if the box has no valid depth or too
// few object points above the table. Throws std::invalid_argument unless depth is CV_32FC1.
inline std::optional<ObjectSample> localizeBox(const cv::Mat& depth, const cv::Rect& box,
                                               const CameraIntrinsics& k,
                                               const Eigen::Isometry3d& world_T_optical,
                                               const LocalizerConfig& cfg, double confidence) {
  const auto object_depth = percentileDepth(depth, box, cfg.depth);
  if (!object_depth) {
    return std::nullopt;
  }
  const auto points_world =
      transformPoints(world_T_optical, bandPoints(depth, box, k, *object_depth, cfg.footprint));
  const auto f = footprintFromWorldPoints(points_world, cfg.table_height, cfg.footprint);
  if (!f) {
    return std::nullopt;
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

}  // namespace graspsort::perception

#endif  // GRASPSORT_PERCEPTION__LOCALIZER_HPP_
