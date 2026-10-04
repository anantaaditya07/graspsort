// Synthetic depth renderer for GraspSort perception tests (ray casting, noise-free).
// Renders the table plane and upright objects as seen by the fixed oblique RGB-D camera from
// world_layout.yaml (D-05), with the Phase 1 intrinsics (640x480, fx = fy = 462.17).
// Moved out of test_projection.cpp so test_localizer.cpp can reuse it.
#ifndef GRASPSORT_PERCEPTION__TEST__OBLIQUE_RENDER_HPP_
#define GRASPSORT_PERCEPTION__TEST__OBLIQUE_RENDER_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <functional>
#include <graspsort_perception/projection.hpp>
#include <limits>
#include <opencv2/core.hpp>
#include <vector>

namespace graspsort_test {

namespace gp = graspsort::perception;

inline constexpr double kPi = EIGEN_PI;
inline constexpr int kWidth = 640;
inline constexpr int kHeight = 480;
inline constexpr double kTableZ = 0.75;
inline const gp::CameraIntrinsics kK{462.17, 462.17, 320.0, 240.0};

inline double deg(double d) { return d * kPi / 180.0; }

// ---- Synthetic depth renderer (ray casting) for the oblique camera ----

// Pose of the optical frame in the world, built from world_layout.yaml camera_link pose
// (x 0.9721, z 1.3245, pitch 0.8727, yaw pi) and the optical rotation rpy(-pi/2, 0, -pi/2).
inline Eigen::Isometry3d cameraPose() {
  Eigen::Isometry3d world_T_link = Eigen::Isometry3d::Identity();
  world_T_link.translation() = Eigen::Vector3d(0.9721, 0.0, 1.3245);
  world_T_link.linear() = (Eigen::AngleAxisd(kPi, Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(0.8727, Eigen::Vector3d::UnitY()))
                              .toRotationMatrix();
  Eigen::Isometry3d link_T_optical = Eigen::Isometry3d::Identity();
  link_T_optical.linear() = (Eigen::AngleAxisd(-kPi / 2.0, Eigen::Vector3d::UnitZ()) *
                             Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(-kPi / 2.0, Eigen::Vector3d::UnitX()))
                                .toRotationMatrix();
  return world_T_link * link_T_optical;
}

struct BoxObj {
  Eigen::Vector2d xy;
  double yaw, sx, sy, h;
};
struct CylObj {
  Eigen::Vector2d xy;
  double r, h;
};

// Ray o + t d (t >= 0) against an upright box standing on the table; returns t or +inf.
inline double hitBox(const Eigen::Vector3d& o, const Eigen::Vector3d& d, const BoxObj& b) {
  const Eigen::Matrix3d r_inv =
      Eigen::AngleAxisd(-b.yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Vector3d c(b.xy.x(), b.xy.y(), kTableZ + b.h / 2.0);
  const Eigen::Vector3d lo = r_inv * (o - c);
  const Eigen::Vector3d ld = r_inv * d;
  const Eigen::Vector3d half(b.sx / 2.0, b.sy / 2.0, b.h / 2.0);
  double t0 = 0.0;
  double t1 = std::numeric_limits<double>::infinity();
  for (int i = 0; i < 3; ++i) {
    if (std::abs(ld[i]) < 1e-12) {
      if (std::abs(lo[i]) > half[i]) return std::numeric_limits<double>::infinity();
      continue;
    }
    double a = (-half[i] - lo[i]) / ld[i];
    double bb = (half[i] - lo[i]) / ld[i];
    if (a > bb) std::swap(a, bb);
    t0 = std::max(t0, a);
    t1 = std::min(t1, bb);
  }
  return t0 <= t1 ? t0 : std::numeric_limits<double>::infinity();
}

// Ray against an upright closed cylinder standing on the table.
inline double hitCyl(const Eigen::Vector3d& o, const Eigen::Vector3d& d, const CylObj& c) {
  double best = std::numeric_limits<double>::infinity();
  const Eigen::Vector2d p(o.x() - c.xy.x(), o.y() - c.xy.y());
  const Eigen::Vector2d q(d.x(), d.y());
  const double a = q.squaredNorm();
  const double b = 2.0 * p.dot(q);
  const double cc = p.squaredNorm() - c.r * c.r;
  const double disc = b * b - 4.0 * a * cc;
  if (a > 0.0 && disc >= 0.0) {
    const double t = (-b - std::sqrt(disc)) / (2.0 * a);
    const double z = o.z() + t * d.z();
    if (t > 0.0 && z >= kTableZ && z <= kTableZ + c.h) best = t;
  }
  if (std::abs(d.z()) > 1e-12) {  // top cap
    const double t = (kTableZ + c.h - o.z()) / d.z();
    const Eigen::Vector2d hp = p + t * q;
    if (t > 0.0 && hp.norm() <= c.r) best = std::min(best, t);
  }
  return best;
}

// Ray against a sphere resting on the table (ball).
inline double hitSphere(const Eigen::Vector3d& o, const Eigen::Vector3d& d,
                        const Eigen::Vector2d& xy, double r) {
  const Eigen::Vector3d oc = o - Eigen::Vector3d(xy.x(), xy.y(), kTableZ + r);
  const double a = d.squaredNorm();
  const double b = 2.0 * oc.dot(d);
  const double disc = b * b - 4.0 * a * (oc.squaredNorm() - r * r);
  if (disc < 0.0) return std::numeric_limits<double>::infinity();
  const double t = (-b - std::sqrt(disc)) / (2.0 * a);
  return t > 0.0 ? t : std::numeric_limits<double>::infinity();
}

using HitFn = std::function<double(const Eigen::Vector3d&, const Eigen::Vector3d&)>;

struct Rendered {
  cv::Mat depth;                // CV_32FC1, metres
  std::vector<cv::Rect> boxes;  // tight pixel box of each object (ideal detector)
};

// Renders the table plane plus the objects. Optical rays have z = 1, so the ray parameter equals
// the optical depth.
inline Rendered render(const std::vector<HitFn>& objects) {
  const Eigen::Isometry3d T = cameraPose();
  const Eigen::Vector3d o = T.translation();
  const std::size_t n = objects.size();
  Rendered out;
  out.depth = cv::Mat(kHeight, kWidth, CV_32FC1, cv::Scalar(0.0F));
  std::vector<int> minu(n, kWidth), minv(n, kHeight), maxu(n, -1), maxv(n, -1);
  for (int v = 0; v < kHeight; ++v) {
    for (int u = 0; u < kWidth; ++u) {
      const Eigen::Vector3d dir_opt((u - kK.cx) / kK.fx, (v - kK.cy) / kK.fy, 1.0);
      const Eigen::Vector3d d = T.linear() * dir_opt;
      double t =
          (d.z() < 0.0) ? (kTableZ - o.z()) / d.z() : std::numeric_limits<double>::infinity();
      int hit = -1;
      for (std::size_t i = 0; i < n; ++i) {
        const double ti = objects[i](o, d);
        if (ti < t) {
          t = ti;
          hit = static_cast<int>(i);
        }
      }
      out.depth.at<float>(v, u) = std::isfinite(t) ? static_cast<float>(t) : 0.0F;
      if (hit >= 0) {
        const auto i = static_cast<std::size_t>(hit);
        minu[i] = std::min(minu[i], u);
        maxu[i] = std::max(maxu[i], u);
        minv[i] = std::min(minv[i], v);
        maxv[i] = std::max(maxv[i], v);
      }
    }
  }
  for (std::size_t i = 0; i < n; ++i) {
    out.boxes.emplace_back(minu[i], minv[i], maxu[i] - minu[i] + 1, maxv[i] - minv[i] + 1);
  }
  return out;
}

}  // namespace graspsort_test

#endif  // GRASPSORT_PERCEPTION__TEST__OBLIQUE_RENDER_HPP_
