// ROS-free geometry for the GraspSort object localizer (architecture 7.2).
//
// Units: metres, radians, pixels. Frames:
//   optical  camera optical frame (x right, y down, z forward), depth = optical z.
//   world    z up, table top is the horizontal plane z = table_height (D-05: the camera is
//            oblique, so nothing here assumes a top-down view; footprints are computed from
//            world-frame 3D points projected vertically onto the table plane).
// Pixel convention (ROS/OpenCV): integer (u, v) = (column, row) is the centre of that pixel.
//
// Depth images are CV_32FC1 in metres (Gazebo depth camera). Invalid pixels are 0, NaN or inf.
// Every threshold is a field of a config struct; the localizer maps them 1:1 to ROS parameters.
#ifndef GRASPSORT_PERCEPTION__PROJECTION_HPP_
#define GRASPSORT_PERCEPTION__PROJECTION_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <optional>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

namespace graspsort::perception {

// ---------------------------------------------------------------------------------------------
// Camera model
// ---------------------------------------------------------------------------------------------

// Pinhole intrinsics from sensor_msgs/CameraInfo K (pixels).
struct CameraIntrinsics {
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
};

// Pixel (u, v) at optical depth `depth` (m) -> 3D point in the optical frame (m).
inline Eigen::Vector3d backProject(const CameraIntrinsics& k, double u, double v, double depth) {
  return {(u - k.cx) * depth / k.fx, (v - k.cy) * depth / k.fy, depth};
}

// Moves `point_optical` by `distance` (m) along `ray_direction` (need not be unit length).
// Architecture 7.2: the back-projected box centre lies on the nearest surface; pushing it in by
// half the object depth along the viewing ray approximates the object centre. A zero ray
// direction returns the point unchanged.
inline Eigen::Vector3d pushIn(const Eigen::Vector3d& point_optical,
                              const Eigen::Vector3d& ray_direction, double distance) {
  const double n = ray_direction.norm();
  if (n <= 0.0 || !std::isfinite(n)) {
    return point_optical;
  }
  return point_optical + ray_direction * (distance / n);
}

// ---------------------------------------------------------------------------------------------
// Depth sampling
// ---------------------------------------------------------------------------------------------

struct DepthSamplingConfig {
  // Fraction of the box width and height (centred) whose pixels are sampled. Architecture: 50%.
  double central_fraction{0.5};
  // Percentile of the valid depths used as the object depth (0 = nearest). Architecture: 20th,
  // so table pixels behind a thin object do not pull the estimate (SemNav D-22).
  double percentile{0.20};
  // Valid depth range (m). Defaults match the Gazebo camera clip range (camera_clip_near/far).
  double min_depth{0.1};
  double max_depth{5.0};
};

// True if `d` is a usable depth: finite, non-zero and within [min_depth, max_depth].
inline bool isValidDepth(float d, double min_depth, double max_depth) {
  return std::isfinite(d) && d > 0.0F && d >= min_depth && d <= max_depth;
}

// Throws std::invalid_argument unless `depth` is CV_32FC1.
inline void requireDepth32F(const cv::Mat& depth) {
  if (depth.type() != CV_32FC1) {
    throw std::invalid_argument("depth image must be CV_32FC1 (metres)");
  }
}

// Centred sub-rectangle covering `fraction` of the box in each dimension (at least 1 px if the
// box is non-empty), NOT clipped to the image.
inline cv::Rect centralRegion(const cv::Rect& box, double fraction) {
  if (box.width <= 0 || box.height <= 0) {
    return {};
  }
  const int w = std::max(1, static_cast<int>(std::lround(box.width * fraction)));
  const int h = std::max(1, static_cast<int>(std::lround(box.height * fraction)));
  return {box.x + (box.width - w) / 2, box.y + (box.height - h) / 2, w, h};
}

// Returns the `p`-th percentile (0..1) of `values` using the lower nearest rank
// index floor(p * (n - 1)). Reorders `values`. Empty input -> nullopt.
inline std::optional<float> percentileOf(std::vector<float>& values, double p) {
  if (values.empty()) {
    return std::nullopt;
  }
  const double pc = std::clamp(p, 0.0, 1.0);
  const auto k = static_cast<std::size_t>(std::floor(pc * static_cast<double>(values.size() - 1)));
  std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(k), values.end());
  return values[k];
}

// Object depth (m) for a detection box (pixels): the configured percentile of the valid depths in
// the central region of the box. The region is clipped to the image, so a box partly outside the
// image uses its visible pixels. Returns nullopt for an empty box, a region outside the image or
// no valid depth. Throws std::invalid_argument if `depth` is not CV_32FC1.
inline std::optional<float> percentileDepth(const cv::Mat& depth, const cv::Rect& box,
                                            const DepthSamplingConfig& cfg = {}) {
  requireDepth32F(depth);
  const cv::Rect roi =
      centralRegion(box, cfg.central_fraction) & cv::Rect(0, 0, depth.cols, depth.rows);
  if (roi.empty()) {
    return std::nullopt;
  }
  std::vector<float> values;
  values.reserve(static_cast<std::size_t>(roi.area()));
  for (int r = roi.y; r < roi.y + roi.height; ++r) {
    const float* row = depth.ptr<float>(r);
    for (int c = roi.x; c < roi.x + roi.width; ++c) {
      if (isValidDepth(row[c], cfg.min_depth, cfg.max_depth)) {
        values.push_back(row[c]);
      }
    }
  }
  return percentileOf(values, cfg.percentile);
}

// ---------------------------------------------------------------------------------------------
// Footprint, yaw and height
// ---------------------------------------------------------------------------------------------

struct FootprintConfig {
  // Pixels within +/- depth_band (m) of the object depth belong to the object. Architecture: 2 cm.
  double depth_band{0.02};
  // World points below table_height + table_clearance (m) are table, not object. Needed because
  // with the oblique camera (D-05) table pixels next to the object base share its depth.
  double table_clearance{0.005};
  // Only points up to this fraction of the object height (above the table) are used for the
  // footprint rectangle; height itself still uses all points. 1.0 = no cut (default).
  // D-14: a cut below a bottle's neck was evaluated and is NOT enabled. From the oblique camera
  // the top surfaces carry the object's far edge: on the real sim bottle 0.8 gave the same result
  // as 1.0 and <= 0.7 broke yaw (47-66 deg errors); for a flat-topped box any cut < 1.0 removes
  // the top face and the footprint collapses to the front face.
  double max_height_fraction{1.0};
  // Minimum number of object points for a footprint (3 = smallest set with an area).
  std::size_t min_points{3};
  // Valid depth range (m), as in DepthSamplingConfig.
  double min_depth{0.1};
  double max_depth{5.0};
};

// Optical-frame 3D points of every valid pixel inside `box` (clipped to the image) whose depth is
// within +/- cfg.depth_band of `object_depth`. Throws std::invalid_argument unless CV_32FC1.
inline std::vector<Eigen::Vector3d> bandPoints(const cv::Mat& depth, const cv::Rect& box,
                                               const CameraIntrinsics& k, double object_depth,
                                               const FootprintConfig& cfg = {}) {
  requireDepth32F(depth);
  std::vector<Eigen::Vector3d> out;
  const cv::Rect roi = box & cv::Rect(0, 0, depth.cols, depth.rows);
  if (roi.empty()) {
    return out;
  }
  for (int r = roi.y; r < roi.y + roi.height; ++r) {
    const float* row = depth.ptr<float>(r);
    for (int c = roi.x; c < roi.x + roi.width; ++c) {
      const float d = row[c];
      if (isValidDepth(d, cfg.min_depth, cfg.max_depth) &&
          std::abs(static_cast<double>(d) - object_depth) <= cfg.depth_band) {
        out.push_back(backProject(k, c, r, d));
      }
    }
  }
  return out;
}

// Applies `world_T_optical` (pose of the optical frame in the world, from TF) to each point.
inline std::vector<Eigen::Vector3d> transformPoints(const Eigen::Isometry3d& world_T_optical,
                                                    const std::vector<Eigen::Vector3d>& points) {
  std::vector<Eigen::Vector3d> out;
  out.reserve(points.size());
  for (const auto& p : points) {
    out.push_back(world_T_optical * p);
  }
  return out;
}

// Wraps an angle to [-pi/2, pi/2): a rectangle's yaw is defined modulo pi.
inline double wrapHalfPi(double a) {
  const double pi = EIGEN_PI;
  double w = std::fmod(a + pi / 2.0, pi);
  if (w < 0.0) {
    w += pi;
  }
  w -= pi / 2.0;
  if (w >= pi / 2.0) {  // guard against rounding at the upper edge
    w -= pi;
  }
  return w;
}

// Footprint of an upright object on the table plane, world frame.
// Convention: size_x >= size_y; yaw (rad, [-pi/2, pi/2)) is the world-frame angle of the
// rectangle's long side (size_x) about +z. For a square both sides tie and the first edge
// returned by cv::RotatedRect::points is used. Callers ignore yaw for cylinders.
struct Footprint {
  Eigen::Vector2d center_xy{Eigen::Vector2d::Zero()};
  double size_x{0.0};
  double size_y{0.0};
  double yaw{0.0};
  double height{0.0};  // max(z) - table_height over the object points (m)
  std::size_t num_points{0};
};

// Height (m) of the highest point above the table plane; nullopt for no points.
inline std::optional<double> heightAboveTable(const std::vector<Eigen::Vector3d>& points_world,
                                              double table_height) {
  if (points_world.empty()) {
    return std::nullopt;
  }
  double zmax = -std::numeric_limits<double>::infinity();
  for (const auto& p : points_world) {
    zmax = std::max(zmax, p.z());
  }
  return zmax - table_height;
}

// Table-plane points of an upright object: world points above the table
// (z >= table_height + table_clearance) and below table_height + max_height_fraction * height,
// projected vertically onto the table plane. height = highest point above the table (all points).
struct FootprintPoints {
  std::vector<cv::Point2f> xy;
  double height{0.0};
};

// nullopt if no point is above the table.
inline std::optional<FootprintPoints> footprintPoints(
    const std::vector<Eigen::Vector3d>& points_world, double table_height,
    const FootprintConfig& cfg = {}) {
  std::vector<Eigen::Vector3d> above;
  above.reserve(points_world.size());
  for (const auto& p : points_world) {
    if (p.allFinite() && p.z() >= table_height + cfg.table_clearance) {
      above.push_back(p);
    }
  }
  const auto height = heightAboveTable(above, table_height);
  if (!height) {
    return std::nullopt;
  }
  const double z_max = table_height + cfg.max_height_fraction * *height;
  FootprintPoints out;
  out.height = *height;
  out.xy.reserve(above.size());
  for (const auto& p : above) {
    if (p.z() <= z_max) {
      out.xy.emplace_back(static_cast<float>(p.x()), static_cast<float>(p.y()));
    }
  }
  return out;
}

// Fits cv::minAreaRect to footprintPoints(). nullopt if fewer than cfg.min_points remain for the
// rectangle.
inline std::optional<Footprint> footprintFromWorldPoints(
    const std::vector<Eigen::Vector3d>& points_world, double table_height,
    const FootprintConfig& cfg = {}) {
  const auto pts = footprintPoints(points_world, table_height, cfg);
  if (!pts) {
    return std::nullopt;
  }
  const auto& xy = pts->xy;
  if (xy.size() < std::max<std::size_t>(cfg.min_points, 1)) {
    return std::nullopt;
  }
  const cv::RotatedRect rr = cv::minAreaRect(xy);
  cv::Point2f v[4];
  rr.points(v);
  const Eigen::Vector2d e0(v[1].x - v[0].x, v[1].y - v[0].y);
  const Eigen::Vector2d e1(v[2].x - v[1].x, v[2].y - v[1].y);
  const bool e0_long = e0.norm() >= e1.norm();
  const Eigen::Vector2d& long_edge = e0_long ? e0 : e1;
  Footprint f;
  f.center_xy = {rr.center.x, rr.center.y};
  f.size_x = std::max(e0.norm(), e1.norm());
  f.size_y = std::min(e0.norm(), e1.norm());
  f.yaw = wrapHalfPi(std::atan2(long_edge.y(), long_edge.x()));
  f.height = pts->height;
  f.num_points = xy.size();
  return f;
}

// ---------------------------------------------------------------------------------------------
// Stability gating and track association
// ---------------------------------------------------------------------------------------------

// One per-frame estimate of an object (world frame, m / rad).
struct ObjectSample {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  double size_x{0.0};
  double size_y{0.0};
  double height{0.0};
  double yaw{0.0};  // rectangle yaw, modulo pi
  double confidence{0.0};
};

struct StabilityConfig {
  // Sliding window of the most recent samples that are averaged ("average over N frames").
  std::size_t window_size{10};
  // Samples needed before an estimate may be published (<= window_size).
  std::size_t min_samples{10};
  // Publish only when the largest per-axis population std-dev of position is below this (m).
  double max_position_stddev{0.005};
};

// Accumulates samples of one static object and decides when the average is stable.
// Averages: arithmetic for position, sizes, height and confidence; yaw uses the doubled-angle
// circular mean (yaw is defined modulo pi), result in [-pi/2, pi/2).
class StabilityGate {
 public:
  explicit StabilityGate(const StabilityConfig& cfg = {}) : cfg_(cfg) {
    if (cfg_.window_size == 0 || cfg_.min_samples == 0 || cfg_.min_samples > cfg_.window_size) {
      throw std::invalid_argument("StabilityConfig: need 0 < min_samples <= window_size");
    }
  }

  void addSample(const ObjectSample& s) {
    samples_.push_back(s);
    while (samples_.size() > cfg_.window_size) {
      samples_.pop_front();
    }
  }

  void reset() { samples_.clear(); }
  std::size_t size() const { return samples_.size(); }

  // Mean of the window regardless of readiness; nullopt if empty.
  std::optional<ObjectSample> mean() const {
    if (samples_.empty()) {
      return std::nullopt;
    }
    ObjectSample m;
    double s2 = 0.0;
    double c2 = 0.0;
    for (const auto& s : samples_) {
      m.position += s.position;
      m.size_x += s.size_x;
      m.size_y += s.size_y;
      m.height += s.height;
      m.confidence += s.confidence;
      s2 += std::sin(2.0 * s.yaw);
      c2 += std::cos(2.0 * s.yaw);
    }
    const double n = static_cast<double>(samples_.size());
    m.position /= n;
    m.size_x /= n;
    m.size_y /= n;
    m.height /= n;
    m.confidence /= n;
    m.yaw = wrapHalfPi(0.5 * std::atan2(s2, c2));
    return m;
  }

  // Largest per-axis population standard deviation of position (m); 0 if empty.
  double positionStdDev() const {
    const auto m = mean();
    if (!m) {
      return 0.0;
    }
    Eigen::Vector3d var = Eigen::Vector3d::Zero();
    for (const auto& s : samples_) {
      var += (s.position - m->position).cwiseAbs2();
    }
    var /= static_cast<double>(samples_.size());
    return std::sqrt(var.maxCoeff());
  }

  bool ready() const {
    return samples_.size() >= cfg_.min_samples && positionStdDev() < cfg_.max_position_stddev;
  }

  // Mean if ready(), else nullopt.
  std::optional<ObjectSample> stableEstimate() const { return ready() ? mean() : std::nullopt; }

 private:
  StabilityConfig cfg_;
  std::deque<ObjectSample> samples_;
};

struct TrackConfig {
  // A measurement joins the nearest track within this 3D distance (m) of the track's mean.
  double max_association_distance{0.05};
  // A track is deleted after this many consecutive updates without a match (>= 1).
  std::size_t max_missed_updates{10};
  // Only associate measurements with tracks of the same class.
  bool require_same_class{true};
  // Id of the first track. Ids increase by one per new track and are never reused.
  std::uint32_t first_id{1};
  StabilityConfig stability{};
};

struct ObjectMeasurement {
  std::string class_name;
  ObjectSample sample;
};

struct Track {
  std::uint32_t id;
  std::string class_name;
  StabilityGate gate;
  std::size_t missed_updates;
};

// Nearest-neighbour association in 3D with stable uint32 ids.
// update(): pairs (track, measurement) within max_association_distance (and same class if
// configured) are matched greedily in order of increasing distance, ties broken by lower track
// id, then lower measurement index; one-to-one. Unmatched measurements start new tracks.
// Unmatched tracks count a miss and are deleted after max_missed_updates consecutive misses.
// Id policy: monotonically increasing from first_id, never reused (uint32 wrap is not handled;
// it needs ~4e9 tracks).
class TrackAssociator {
 public:
  explicit TrackAssociator(const TrackConfig& cfg = {}) : cfg_(cfg), next_id_(cfg.first_id) {
    if (cfg_.max_missed_updates == 0) {
      throw std::invalid_argument("TrackConfig: max_missed_updates must be >= 1");
    }
    StabilityGate check(cfg_.stability);  // validates the stability config
  }

  // Returns the track id assigned to each measurement (same order as the input).
  std::vector<std::uint32_t> update(const std::vector<ObjectMeasurement>& measurements) {
    struct Pair {
      double dist;
      std::uint32_t track_id;
      std::size_t track_idx;
      std::size_t meas_idx;
    };
    std::vector<Pair> pairs;
    for (std::size_t t = 0; t < tracks_.size(); ++t) {
      const auto tm = tracks_[t].gate.mean();
      if (!tm) {
        continue;
      }
      for (std::size_t m = 0; m < measurements.size(); ++m) {
        if (cfg_.require_same_class && measurements[m].class_name != tracks_[t].class_name) {
          continue;
        }
        const double d = (measurements[m].sample.position - tm->position).norm();
        if (d <= cfg_.max_association_distance) {
          pairs.push_back({d, tracks_[t].id, t, m});
        }
      }
    }
    std::sort(pairs.begin(), pairs.end(), [](const Pair& a, const Pair& b) {
      return std::tie(a.dist, a.track_id, a.meas_idx) < std::tie(b.dist, b.track_id, b.meas_idx);
    });

    std::vector<bool> track_used(tracks_.size(), false);
    std::vector<std::optional<std::uint32_t>> assigned(measurements.size());
    for (const auto& p : pairs) {
      if (track_used[p.track_idx] || assigned[p.meas_idx]) {
        continue;
      }
      track_used[p.track_idx] = true;
      assigned[p.meas_idx] = p.track_id;
      tracks_[p.track_idx].gate.addSample(measurements[p.meas_idx].sample);
      tracks_[p.track_idx].missed_updates = 0;
    }
    for (std::size_t t = 0; t < tracks_.size(); ++t) {
      if (!track_used[t]) {
        ++tracks_[t].missed_updates;
      }
    }
    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                                 [this](const Track& tr) {
                                   return tr.missed_updates >= cfg_.max_missed_updates;
                                 }),
                  tracks_.end());

    std::vector<std::uint32_t> ids(measurements.size());
    for (std::size_t m = 0; m < measurements.size(); ++m) {
      if (!assigned[m]) {
        Track tr{next_id_++, measurements[m].class_name, StabilityGate(cfg_.stability), 0};
        tr.gate.addSample(measurements[m].sample);
        tracks_.push_back(tr);
        assigned[m] = tr.id;
      }
      ids[m] = *assigned[m];
    }
    return ids;
  }

  const std::vector<Track>& tracks() const { return tracks_; }

  // Track with this id, or nullptr.
  const Track* find(std::uint32_t id) const {
    for (const auto& t : tracks_) {
      if (t.id == id) {
        return &t;
      }
    }
    return nullptr;
  }

 private:
  TrackConfig cfg_;
  std::uint32_t next_id_;
  std::vector<Track> tracks_;
};

}  // namespace graspsort::perception

#endif  // GRASPSORT_PERCEPTION__PROJECTION_HPP_
