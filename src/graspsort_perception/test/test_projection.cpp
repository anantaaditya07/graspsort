// GoogleTests for graspsort_perception/projection.hpp (architecture 7.2): back-projection,
// percentile sampling with table pixels present, minAreaRect yaw, height, stability gating and
// track association. Fixtures use the Phase 1 numbers: 640x480 camera with fx = fy = 462.17,
// table top at z = 0.75 m, camera from world_layout.yaml (oblique, 50 deg pitch, D-05), and the
// D-04 objects (ball r 0.0375, cup r 0.0324 h 0.13, bottle box 0.097 x 0.067 x 0.191).
#include <gtest/gtest.h>

#include <cmath>
#include <functional>
#include <graspsort_perception/projection.hpp>
#include <limits>
#include <vector>

namespace gp = graspsort::perception;

namespace {

constexpr double kPi = EIGEN_PI;
constexpr int kWidth = 640;
constexpr int kHeight = 480;
constexpr double kTableZ = 0.75;
const gp::CameraIntrinsics kK{462.17, 462.17, 320.0, 240.0};

double deg(double d) { return d * kPi / 180.0; }

// ---- Synthetic depth renderer (ray casting) for the oblique camera ----

// Pose of the optical frame in the world, built from world_layout.yaml camera_link pose
// (x 0.9721, z 1.3245, pitch 0.8727, yaw pi) and the optical rotation rpy(-pi/2, 0, -pi/2).
Eigen::Isometry3d cameraPose() {
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
double hitBox(const Eigen::Vector3d& o, const Eigen::Vector3d& d, const BoxObj& b) {
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
double hitCyl(const Eigen::Vector3d& o, const Eigen::Vector3d& d, const CylObj& c) {
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
double hitSphere(const Eigen::Vector3d& o, const Eigen::Vector3d& d, const Eigen::Vector2d& xy,
                 double r) {
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
Rendered render(const std::vector<HitFn>& objects) {
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

// Points on the side faces and top of an upright rectangle (world frame) for footprint tests.
std::vector<Eigen::Vector3d> boxSurface(const Eigen::Vector2d& c, double yaw, double sx, double sy,
                                        double h, double step) {
  std::vector<Eigen::Vector3d> pts;
  const Eigen::Rotation2Dd r(yaw);
  for (double z = kTableZ + step; z <= kTableZ + h + 1e-9; z += step) {
    for (double a = -sx / 2.0; a <= sx / 2.0 + 1e-9; a += step) {
      for (double b = -sy / 2.0; b <= sy / 2.0 + 1e-9; b += step) {
        const bool edge = std::abs(std::abs(a) - sx / 2.0) < step / 2.0 ||
                          std::abs(std::abs(b) - sy / 2.0) < step / 2.0;
        if (!edge && z < kTableZ + h - 1e-9) continue;
        const Eigen::Vector2d p = c + r * Eigen::Vector2d(a, b);
        pts.emplace_back(p.x(), p.y(), z);
      }
    }
  }
  // Exact corners at the top so the hull is the true rectangle.
  for (int sa : {-1, 1}) {
    for (int sb : {-1, 1}) {
      const Eigen::Vector2d p = c + r * Eigen::Vector2d(sa * sx / 2.0, sb * sy / 2.0);
      pts.emplace_back(p.x(), p.y(), kTableZ + h);
    }
  }
  return pts;
}

gp::ObjectSample sampleAt(double x, double y, double z) {
  gp::ObjectSample s;
  s.position = {x, y, z};
  s.size_x = 0.067;
  s.size_y = 0.067;
  s.height = 0.13;
  s.yaw = 0.0;
  s.confidence = 0.8;
  return s;
}

gp::ObjectMeasurement meas(const std::string& cls, double x, double y) {
  return {cls, sampleAt(x, y, 0.815)};
}

}  // namespace

// ================================ back-projection ============================================

TEST(BackProject, PrincipalPointLiesOnOpticalAxis) {
  const Eigen::Vector3d p = gp::backProject(kK, 320.0, 240.0, 0.75);
  EXPECT_DOUBLE_EQ(p.x(), 0.0);
  EXPECT_DOUBLE_EQ(p.y(), 0.0);
  EXPECT_DOUBLE_EQ(p.z(), 0.75);
}

TEST(BackProject, OffAxisPixelMatchesPinholeModel) {
  // 100 px right and 100 px up at 0.8 m: x = 100 * 0.8 / 462.17 = 0.173097 m.
  const Eigen::Vector3d p = gp::backProject(kK, 420.0, 140.0, 0.8);
  EXPECT_NEAR(p.x(), 0.173097, 1e-6);
  EXPECT_NEAR(p.y(), -0.173097, 1e-6);
  EXPECT_DOUBLE_EQ(p.z(), 0.8);
  // Re-projection returns the pixel.
  EXPECT_NEAR(kK.fx * p.x() / p.z() + kK.cx, 420.0, 1e-9);
  EXPECT_NEAR(kK.fy * p.y() / p.z() + kK.cy, 140.0, 1e-9);
}

TEST(PushIn, MovesAlongNormalisedRay) {
  const Eigen::Vector3d p(0.1, -0.05, 0.7);
  const Eigen::Vector3d q = gp::pushIn(p, p, 0.0335);  // half the bottle's 0.067 m depth
  EXPECT_NEAR((q - p).norm(), 0.0335, 1e-12);
  EXPECT_NEAR((q - p).normalized().dot(p.normalized()), 1.0, 1e-12);
  const Eigen::Vector3d axial = gp::pushIn(Eigen::Vector3d(0, 0, 1.0), {0, 0, 2.0}, 0.05);
  EXPECT_NEAR(axial.z(), 1.05, 1e-12);
}

TEST(PushIn, ZeroRayLeavesPointUnchanged) {
  const Eigen::Vector3d p(0.1, 0.2, 0.3);
  EXPECT_TRUE(gp::pushIn(p, Eigen::Vector3d::Zero(), 0.05).isApprox(p));
}

// ================================ depth sampling =============================================

TEST(PercentileOf, LowerNearestRank) {
  std::vector<float> v{10, 9, 8, 7, 6, 5, 4, 3, 2, 1};
  EXPECT_FLOAT_EQ(*gp::percentileOf(v, 0.20), 2.0F);  // floor(0.2 * 9) = 1 -> 2nd smallest
  EXPECT_FLOAT_EQ(*gp::percentileOf(v, 0.0), 1.0F);
  EXPECT_FLOAT_EQ(*gp::percentileOf(v, 1.0), 10.0F);
  std::vector<float> empty;
  EXPECT_FALSE(gp::percentileOf(empty, 0.2).has_value());
}

TEST(CentralRegion, CentredHalfOfTheBox) {
  EXPECT_EQ(gp::centralRegion(cv::Rect(100, 50, 40, 80), 0.5), cv::Rect(110, 70, 20, 40));
  EXPECT_EQ(gp::centralRegion(cv::Rect(100, 50, 1, 1), 0.5), cv::Rect(100, 50, 1, 1));
  EXPECT_TRUE(gp::centralRegion(cv::Rect(100, 50, 0, 10), 0.5).empty());
}

TEST(PercentileDepth, TablePixelsDoNotPullTheEstimate) {
  // Thin object (8 px wide, 0.70 m) inside a 40 px wide box; table behind it at 0.85 m.
  // The central 50% (20 px) is 40% object, 60% table: the median is the table, the 20th
  // percentile is the object.
  cv::Mat depth(kHeight, kWidth, CV_32FC1, cv::Scalar(0.85F));
  depth(cv::Rect(316, 200, 8, 80)).setTo(0.70F);
  const cv::Rect box(300, 200, 40, 80);
  const auto p20 = gp::percentileDepth(depth, box);
  ASSERT_TRUE(p20.has_value());
  EXPECT_FLOAT_EQ(*p20, 0.70F);
  gp::DepthSamplingConfig median;
  median.percentile = 0.5;
  EXPECT_FLOAT_EQ(gp::percentileDepth(depth, box, median).value_or(-1.0F), 0.85F);
}

TEST(PercentileDepth, RejectsZeroNanInfAndOutOfRange) {
  cv::Mat depth(kHeight, kWidth, CV_32FC1, cv::Scalar(0.72F));
  const cv::Rect box(200, 200, 40, 60);  // central region (210, 215, 20, 30)
  depth(cv::Rect(210, 215, 20, 4)).setTo(0.0F);
  depth(cv::Rect(210, 219, 20, 4)).setTo(std::numeric_limits<float>::quiet_NaN());
  depth(cv::Rect(210, 223, 20, 4)).setTo(std::numeric_limits<float>::infinity());
  depth(cv::Rect(210, 227, 20, 4)).setTo(0.05F);  // below min_depth 0.1
  depth(cv::Rect(210, 231, 20, 4)).setTo(6.0F);   // above max_depth 5.0
  // Only rows 235..244 (200 px) are valid: nearest and farthest are both 0.72.
  gp::DepthSamplingConfig nearest;
  nearest.percentile = 0.0;
  const auto lo = gp::percentileDepth(depth, box, nearest);
  ASSERT_TRUE(lo.has_value());
  EXPECT_FLOAT_EQ(*lo, 0.72F);
  gp::DepthSamplingConfig farthest;
  farthest.percentile = 1.0;
  const auto hi = gp::percentileDepth(depth, box, farthest);
  ASSERT_TRUE(hi.has_value());
  EXPECT_FLOAT_EQ(*hi, 0.72F);
}

TEST(PercentileDepth, AllNanGivesNoDepth) {
  cv::Mat depth(kHeight, kWidth, CV_32FC1, cv::Scalar(std::numeric_limits<float>::quiet_NaN()));
  EXPECT_FALSE(gp::percentileDepth(depth, cv::Rect(100, 100, 50, 50)).has_value());
}

TEST(PercentileDepth, EmptyOrOutsideBoxGivesNoDepth) {
  cv::Mat depth(kHeight, kWidth, CV_32FC1, cv::Scalar(0.8F));
  EXPECT_FALSE(gp::percentileDepth(depth, cv::Rect(100, 100, 0, 0)).has_value());
  EXPECT_FALSE(gp::percentileDepth(depth, cv::Rect(700, 100, 40, 40)).has_value());
  EXPECT_FALSE(gp::percentileDepth(depth, cv::Rect(-100, -100, 40, 40)).has_value());
}

TEST(PercentileDepth, BoxPartlyOutsideImageUsesVisiblePixels) {
  // Box x = -40..39: central region x = -20..19, visible part x = 0..19 at 0.60 m.
  cv::Mat depth(kHeight, kWidth, CV_32FC1, cv::Scalar(0.90F));
  depth(cv::Rect(0, 100, 20, 60)).setTo(0.60F);
  gp::DepthSamplingConfig farthest;
  farthest.percentile = 1.0;
  const auto d = gp::percentileDepth(depth, cv::Rect(-40, 100, 80, 40), farthest);
  ASSERT_TRUE(d.has_value());
  EXPECT_FLOAT_EQ(*d, 0.60F);
}

TEST(PercentileDepth, RejectsNonFloatImage) {
  cv::Mat depth(kHeight, kWidth, CV_16UC1, cv::Scalar(800));
  EXPECT_THROW(gp::percentileDepth(depth, cv::Rect(0, 0, 10, 10)), std::invalid_argument);
}

// ================================ band points ================================================

TEST(BandPoints, KeepsPixelsWithinTwoCentimetres) {
  cv::Mat depth(kHeight, kWidth, CV_32FC1, cv::Scalar(0.85F));
  depth(cv::Rect(300, 200, 10, 10)).setTo(0.70F);  // 100 object pixels
  depth.at<float>(205, 320) = 0.715F;              // inside the band
  depth.at<float>(205, 321) = 0.725F;              // outside the band
  depth.at<float>(205, 322) = std::numeric_limits<float>::quiet_NaN();
  const auto pts = gp::bandPoints(depth, cv::Rect(290, 190, 40, 40), kK, 0.70);
  EXPECT_EQ(pts.size(), 101U);
  const Eigen::Vector3d expect = gp::backProject(kK, 300.0, 200.0, 0.70F);
  EXPECT_TRUE(pts.front().isApprox(expect, 1e-6));
}

TEST(TransformPoints, AppliesWorldFromOptical) {
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);
  t.linear() = Eigen::AngleAxisd(kPi / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const auto out = gp::transformPoints(t, {Eigen::Vector3d(1.0, 0.0, 0.0)});
  ASSERT_EQ(out.size(), 1U);
  EXPECT_TRUE(out[0].isApprox(Eigen::Vector3d(1.0, 3.0, 3.0), 1e-12));
}

// ================================ footprint / yaw / height ===================================

TEST(WrapHalfPi, MapsToHalfOpenInterval) {
  EXPECT_NEAR(gp::wrapHalfPi(0.3), 0.3, 1e-12);
  EXPECT_NEAR(gp::wrapHalfPi(kPi / 2.0), -kPi / 2.0, 1e-12);
  EXPECT_NEAR(gp::wrapHalfPi(-kPi / 2.0), -kPi / 2.0, 1e-12);
  EXPECT_NEAR(gp::wrapHalfPi(deg(135)), deg(-45), 1e-12);
  EXPECT_NEAR(gp::wrapHalfPi(0.1 + kPi), 0.1, 1e-12);
  EXPECT_NEAR(gp::wrapHalfPi(0.1 - 3.0 * kPi), 0.1, 1e-12);
}

TEST(Footprint, BottleSizeYawAndHeight) {
  const auto pts = boxSurface({0.38, -0.08}, deg(30), 0.097, 0.067, 0.191, 0.004);
  const auto f = gp::footprintFromWorldPoints(pts, kTableZ);
  ASSERT_TRUE(f.has_value());
  EXPECT_NEAR(f->size_x, 0.097, 1e-4);
  EXPECT_NEAR(f->size_y, 0.067, 1e-4);
  EXPECT_NEAR(f->yaw, deg(30), deg(0.1));
  EXPECT_NEAR(f->center_xy.x(), 0.38, 1e-4);
  EXPECT_NEAR(f->center_xy.y(), -0.08, 1e-4);
  EXPECT_NEAR(f->height, 0.191, 1e-6);
}

TEST(Footprint, SizeXIsTheLongSideWhateverTheInputOrder) {
  // Same physical bottle described with swapped sides and yaw + 90 deg.
  const auto pts = boxSurface({0.4, 0.0}, deg(30 + 90), 0.067, 0.097, 0.191, 0.004);
  const auto f = gp::footprintFromWorldPoints(pts, kTableZ);
  ASSERT_TRUE(f.has_value());
  EXPECT_NEAR(f->size_x, 0.097, 1e-4);
  EXPECT_NEAR(f->size_y, 0.067, 1e-4);
  EXPECT_NEAR(f->yaw, deg(30), deg(0.1));
}

TEST(Footprint, YawWrapsAroundPlusMinusHalfPi) {
  struct Case {
    double in_deg, out_deg;
  };
  for (const Case c : {Case{89.5, 89.5}, Case{90.0, -90.0}, Case{90.5, -89.5}, Case{-89.5, -89.5},
                       Case{120.0, -60.0}, Case{-100.0, 80.0}, Case{180.0, 0.0}}) {
    const auto pts = boxSurface({0.45, 0.1}, deg(c.in_deg), 0.097, 0.067, 0.05, 0.004);
    const auto f = gp::footprintFromWorldPoints(pts, kTableZ);
    ASSERT_TRUE(f.has_value());
    EXPECT_GE(f->yaw, -kPi / 2.0) << c.in_deg;
    EXPECT_LT(f->yaw, kPi / 2.0) << c.in_deg;
    // Compare modulo pi, and exactly within the interval away from the +-90 deg seam.
    EXPECT_NEAR(gp::wrapHalfPi(f->yaw - deg(c.out_deg)), 0.0, deg(0.1)) << c.in_deg;
    if (std::abs(c.out_deg) < 89.0) {
      EXPECT_NEAR(f->yaw, deg(c.out_deg), deg(0.1)) << c.in_deg;
    }
  }
}

TEST(Footprint, TablePointsAreIgnored) {
  auto pts = boxSurface({0.48, 0.2}, deg(10), 0.097, 0.067, 0.191, 0.004);
  // Table pixels in the band all around the object (z at and just above the table top).
  for (double x = 0.38; x <= 0.58; x += 0.01) {
    pts.emplace_back(x, 0.30, kTableZ);
    pts.emplace_back(x, 0.10, kTableZ + 0.004);  // below the 5 mm clearance
  }
  const auto f = gp::footprintFromWorldPoints(pts, kTableZ);
  ASSERT_TRUE(f.has_value());
  EXPECT_NEAR(f->size_x, 0.097, 1e-4);
  EXPECT_NEAR(f->size_y, 0.067, 1e-4);
}

TEST(Footprint, CupFootprintIsSquareSoYawIsMeaningless) {
  // Cylinder (cup r 0.0324, h 0.13): the rectangle is ~a square of side 2r; callers ignore yaw.
  std::vector<Eigen::Vector3d> pts;
  for (int i = 0; i < 72; ++i) {
    const double a = 2.0 * kPi * i / 72.0;
    for (double z = kTableZ + 0.01; z <= kTableZ + 0.13 + 1e-9; z += 0.03) {
      pts.emplace_back(0.48 + 0.0324 * std::cos(a), -0.2 + 0.0324 * std::sin(a), z);
    }
  }
  const auto f = gp::footprintFromWorldPoints(pts, kTableZ);
  ASSERT_TRUE(f.has_value());
  EXPECT_NEAR(f->size_x, 0.0648, 1e-3);
  EXPECT_NEAR(f->size_y, 0.0648, 1e-3);
  EXPECT_LT(f->size_x - f->size_y, 1e-3);
  EXPECT_NEAR(f->height, 0.13, 1e-6);
}

TEST(Footprint, TooFewPointsGivesNothing) {
  const std::vector<Eigen::Vector3d> two{{0.4, 0.0, 0.8}, {0.41, 0.0, 0.8}};
  EXPECT_FALSE(gp::footprintFromWorldPoints(two, kTableZ).has_value());
  EXPECT_FALSE(gp::footprintFromWorldPoints({}, kTableZ).has_value());
  EXPECT_FALSE(gp::heightAboveTable({}, kTableZ).has_value());
}

// ======================= end to end on a rendered oblique depth image ========================

TEST(ObliqueRender, CameraPoseLooksAtTheAimPoint) {
  // The optical axis passes through the world_layout aim point (0.49, 0, 0.75) at ~0.75 m.
  const Eigen::Isometry3d t = cameraPose();
  const Eigen::Vector3d aim(0.49, 0.0, 0.75);
  const Eigen::Vector3d in_optical = t.inverse() * aim;
  EXPECT_NEAR(in_optical.x(), 0.0, 2e-3);
  EXPECT_NEAR(in_optical.y(), 0.0, 2e-3);
  EXPECT_NEAR(in_optical.z(), 0.75, 2e-3);
}

// Ideal-detector scenes from the world layout (bottle_1, cup_2, ball_1 positions).
struct Scene {
  const char* name;
  HitFn hit;
  Eigen::Vector2d xy;
  double half_depth;  // pushIn distance: half the object's extent along the view
  double height;
  double size_x, size_y;
  double yaw_deg;
};

Scene bottleScene(double yaw_deg) {
  const BoxObj b{{0.38, -0.08}, deg(yaw_deg), 0.097, 0.067, 0.191};
  return {
      "bottle", [b](const Eigen::Vector3d& o, const Eigen::Vector3d& d) { return hitBox(o, d, b); },
      b.xy,     0.067 / 2.0,
      0.191,    0.097,
      0.067,    yaw_deg};
}

Scene cupScene() {
  const CylObj c{{0.48, 0.20}, 0.0324, 0.13};
  return {
      "cup",  [c](const Eigen::Vector3d& o, const Eigen::Vector3d& d) { return hitCyl(o, d, c); },
      c.xy,   0.0324,
      0.13,   0.0648,
      0.0648, 0.0};
}

Scene ballScene() {
  const Eigen::Vector2d xy(0.60, -0.12);
  return {"ball",
          [xy](const Eigen::Vector3d& o, const Eigen::Vector3d& d) {
            return hitSphere(o, d, xy, 0.0375);
          },
          xy,
          0.0375,
          0.075,
          0.075,
          0.075,
          0.0};
}

struct PipelineResult {
  Eigen::Vector3d push_in_centre;
  gp::Footprint footprint;
};

// Runs the 7.2 chain on a rendered scene: percentile depth, back-projection + push-in, band
// points, world transform, footprint.
PipelineResult runPipeline(const Scene& sc, double depth_band) {
  const Rendered r = render({sc.hit});
  const cv::Rect box = r.boxes.at(0);
  EXPECT_GT(box.area(), 0) << sc.name;
  const auto d = gp::percentileDepth(r.depth, box);
  EXPECT_TRUE(d.has_value()) << sc.name;
  const Eigen::Isometry3d t = cameraPose();
  const Eigen::Vector3d surf =
      gp::backProject(kK, box.x + box.width / 2.0, box.y + box.height / 2.0, d.value_or(0.0F));
  gp::FootprintConfig fc;
  fc.depth_band = depth_band;
  const auto f = gp::footprintFromWorldPoints(
      gp::transformPoints(t, gp::bandPoints(r.depth, box, kK, d.value_or(0.0F), fc)), kTableZ, fc);
  EXPECT_TRUE(f.has_value()) << sc.name;
  return {t * gp::pushIn(surf, surf, sc.half_depth), f.value_or(gp::Footprint{})};
}

TEST(ObliqueRender, WideBandRecoversFootprintCentreYawAndHeight) {
  // With the oblique camera a band wide enough to cover the whole visible object gives the true
  // footprint for every bottle yaw. (Default band 0.02 m does not, see the next test.)
  const double wide_band = 0.10;
  for (double yaw : {0.0, 30.0, 60.0, -45.0}) {
    const Scene sc = bottleScene(yaw);
    const auto res = runPipeline(sc, wide_band);
    const auto& f = res.footprint;
    EXPECT_NEAR(f.size_x, sc.size_x, 0.0015) << yaw;
    EXPECT_NEAR(f.size_y, sc.size_y, 0.0015) << yaw;
    EXPECT_NEAR(gp::wrapHalfPi(f.yaw - deg(yaw)), 0.0, deg(0.5)) << yaw;
    EXPECT_LT((f.center_xy - sc.xy).norm(), 0.002) << yaw;
    EXPECT_NEAR(f.height, sc.height, 0.001) << yaw;
  }
  const auto cup = runPipeline(cupScene(), wide_band).footprint;
  EXPECT_NEAR(cup.size_x, 0.0648, 0.002);
  EXPECT_NEAR(cup.size_y, 0.0648, 0.002);
  EXPECT_LT((cup.center_xy - cupScene().xy).norm(), 0.002);
  EXPECT_NEAR(cup.height, 0.13, 0.001);
  // The ball's far hemisphere is hidden: centre biased ~5 mm towards the camera.
  const auto ball = runPipeline(ballScene(), wide_band).footprint;
  EXPECT_LT((ball.center_xy - ballScene().xy).norm(), 0.007);
  EXPECT_NEAR(ball.height, 0.075, 0.001);
}

TEST(ObliqueRender, TwoCentimetreBandUnderestimatesFootprint) {
  // Records a D-05 finding: with the architecture's +-2 cm band and the oblique camera only a
  // strip of the front face is kept. Bottle face-on (yaw 0): the 0.097 m side is lost and the long
  // axis is reported 90 deg off; the cup comes out 0.064 x 0.045 m instead of 0.065 x 0.065 m.
  const auto bottle = runPipeline(bottleScene(0.0), gp::FootprintConfig{}.depth_band).footprint;
  EXPECT_LT(bottle.size_x, 0.075);
  EXPECT_NEAR(std::abs(bottle.yaw), kPi / 2.0, deg(1.0));
  const auto cup = runPipeline(cupScene(), gp::FootprintConfig{}.depth_band).footprint;
  EXPECT_LT(cup.size_y, 0.05);
  // Height is unaffected: the top of the band slice is the top edge of the object.
  EXPECT_NEAR(bottle.height, 0.191, 0.001);
}

TEST(ObliqueRender, PushInCentreIsCentimetresOffForTallObjects) {
  // Records a D-05 finding: the box-centre pixel hits the front face high up; pushing in along
  // the downward-slanted ray moves the estimate along the view, not horizontally.
  const Scene bottle = bottleScene(30.0);
  const auto res = runPipeline(bottle, 0.10);
  EXPECT_GT((res.push_in_centre.head<2>() - bottle.xy).norm(), 0.03);
  EXPECT_LT((res.footprint.center_xy - bottle.xy).norm(), 0.002);
  // For the ball (centre pixel near the sphere centre) push-in is accurate.
  const auto ball = runPipeline(ballScene(), 0.10);
  EXPECT_LT((ball.push_in_centre.head<2>() - ballScene().xy).norm(), 0.002);
}

// ================================ stability gating ===========================================

TEST(StabilityGate, NotReadyBeforeMinSamples) {
  gp::StabilityGate g;  // window 10, min 10, 5 mm
  for (int i = 0; i < 9; ++i) g.addSample(sampleAt(0.48, -0.2, 0.815));
  EXPECT_FALSE(g.ready());
  EXPECT_FALSE(g.stableEstimate().has_value());
  g.addSample(sampleAt(0.48, -0.2, 0.815));
  EXPECT_TRUE(g.ready());
}

TEST(StabilityGate, ReadyWithMillimetreNoiseAndReturnsMean) {
  gp::StabilityGate g;
  for (int i = 0; i < 10; ++i) {
    const double e = (i % 2 == 0) ? 0.001 : -0.001;  // std-dev 1 mm
    g.addSample(sampleAt(0.48 + e, -0.2 - e, 0.815));
  }
  EXPECT_NEAR(g.positionStdDev(), 0.001, 1e-9);
  const auto m = g.stableEstimate();
  ASSERT_TRUE(m.has_value());
  EXPECT_NEAR(m->position.x(), 0.48, 1e-12);
  EXPECT_NEAR(m->position.y(), -0.2, 1e-12);
  EXPECT_NEAR(m->size_x, 0.067, 1e-12);
  EXPECT_NEAR(m->confidence, 0.8, 1e-12);
}

TEST(StabilityGate, NotReadyWithCentimetreNoise) {
  gp::StabilityGate g;
  for (int i = 0; i < 10; ++i) {
    const double e = (i % 2 == 0) ? 0.01 : -0.01;  // std-dev 10 mm > 5 mm
    g.addSample(sampleAt(0.48, -0.2, 0.815 + e));
  }
  EXPECT_NEAR(g.positionStdDev(), 0.01, 1e-9);
  EXPECT_FALSE(g.ready());
}

TEST(StabilityGate, WindowSlidesAfterTheObjectMoves) {
  gp::StabilityGate g;
  for (int i = 0; i < 10; ++i) g.addSample(sampleAt(0.48, -0.2, 0.815));
  ASSERT_TRUE(g.ready());
  for (int i = 0; i < 5; ++i) g.addSample(sampleAt(0.52, -0.2, 0.815));
  EXPECT_FALSE(g.ready());  // half old, half new: std-dev 20 mm
  for (int i = 0; i < 5; ++i) g.addSample(sampleAt(0.52, -0.2, 0.815));
  ASSERT_TRUE(g.ready());
  EXPECT_NEAR(g.stableEstimate()->position.x(), 0.52, 1e-12);
  EXPECT_EQ(g.size(), 10U);
}

TEST(StabilityGate, YawMeanWrapsAtHalfPi) {
  gp::StabilityGate g;
  for (int i = 0; i < 10; ++i) {
    auto s = sampleAt(0.38, -0.08, 0.845);
    s.yaw = (i % 2 == 0) ? deg(89) : deg(-89);  // same physical orientation +-1 deg
    g.addSample(s);
  }
  const double y = g.mean()->yaw;
  EXPECT_NEAR(std::abs(y), kPi / 2.0, 1e-9);  // not 0, which an arithmetic mean would give
  EXPECT_GE(y, -kPi / 2.0);
  EXPECT_LT(y, kPi / 2.0);
}

TEST(StabilityGate, InvalidConfigThrows) {
  gp::StabilityConfig c;
  c.min_samples = 11;  // > window_size 10
  EXPECT_THROW(gp::StabilityGate{c}, std::invalid_argument);
  c.min_samples = 0;
  EXPECT_THROW(gp::StabilityGate{c}, std::invalid_argument);
}

// ================================ track association ==========================================

TEST(TrackAssociator, StableIdsForStaticObjects) {
  gp::TrackAssociator ta;
  const auto first = ta.update({meas("cup", 0.48, -0.2), meas("cup", 0.48, 0.2)});
  ASSERT_EQ(first, (std::vector<std::uint32_t>{1, 2}));
  // Reordered input with 3 mm jitter keeps the ids.
  const auto second = ta.update({meas("cup", 0.483, 0.2), meas("cup", 0.477, -0.2)});
  EXPECT_EQ(second, (std::vector<std::uint32_t>{2, 1}));
  EXPECT_EQ(ta.tracks().size(), 2U);
  EXPECT_EQ(ta.find(1)->gate.size(), 2U);
}

TEST(TrackAssociator, FarOrOtherClassMeasurementStartsANewTrack) {
  gp::TrackAssociator ta;
  ta.update({meas("cup", 0.48, -0.2)});
  EXPECT_EQ(ta.update({meas("cup", 0.48, -0.149)}), (std::vector<std::uint32_t>{2}));  // 51 mm
  EXPECT_EQ(ta.update({meas("bottle", 0.48, -0.2)}), (std::vector<std::uint32_t>{3}));
  EXPECT_EQ(ta.update({meas("cup", 0.48, -0.2)}), (std::vector<std::uint32_t>{1}));
}

TEST(TrackAssociator, NearestMeasurementWinsAndTiesGoToLowerIndex) {
  gp::TrackAssociator ta;
  ta.update({meas("cup", 0.48, -0.2)});
  // Nearer measurement (index 1) takes track 1, the other starts track 2.
  EXPECT_EQ(ta.update({meas("cup", 0.48, -0.17), meas("cup", 0.48, -0.19)}),
            (std::vector<std::uint32_t>{2, 1}));

  gp::TrackAssociator tie;
  tie.update({meas("ball", 0.60, 0.0)});
  // Two measurements exactly 20 mm either side: the lower index wins.
  EXPECT_EQ(tie.update({meas("ball", 0.60, 0.02), meas("ball", 0.60, -0.02)}),
            (std::vector<std::uint32_t>{1, 2}));
}

TEST(TrackAssociator, TrackExpiresAfterMaxMissedAndIdsAreNotReused) {
  gp::TrackConfig cfg;
  cfg.max_missed_updates = 3;
  gp::TrackAssociator ta(cfg);
  ta.update({meas("bottle", 0.38, -0.08), meas("ball", 0.60, 0.12)});
  for (int i = 0; i < 2; ++i) ta.update({meas("ball", 0.60, 0.12)});
  ASSERT_NE(ta.find(1), nullptr);  // 2 misses: still alive
  EXPECT_EQ(ta.find(1)->missed_updates, 2U);
  ta.update({meas("ball", 0.60, 0.12)});
  EXPECT_EQ(ta.find(1), nullptr);  // 3rd miss: deleted
  EXPECT_EQ(ta.tracks().size(), 1U);
  // The bottle reappears at the same place: new id 3, not 1.
  EXPECT_EQ(ta.update({meas("bottle", 0.38, -0.08), meas("ball", 0.60, 0.12)}),
            (std::vector<std::uint32_t>{3, 2}));
}

TEST(TrackAssociator, MatchResetsMissCounter) {
  gp::TrackConfig cfg;
  cfg.max_missed_updates = 2;
  gp::TrackAssociator ta(cfg);
  ta.update({meas("cup", 0.48, 0.2)});
  ta.update({});
  ta.update({meas("cup", 0.48, 0.2)});
  ta.update({});
  ASSERT_NE(ta.find(1), nullptr);
  EXPECT_EQ(ta.find(1)->missed_updates, 1U);
}

TEST(TrackAssociator, InvalidConfigThrows) {
  gp::TrackConfig cfg;
  cfg.max_missed_updates = 0;
  EXPECT_THROW(gp::TrackAssociator{cfg}, std::invalid_argument);
}
