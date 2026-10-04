// GoogleTests for graspsort_perception/localizer.hpp (architecture 7.2 as amended by D-12):
// box conversion, per-detection localization on rendered oblique depth images, shape handling
// and the class -> shape table. Scenes use the world_layout.yaml object positions.
#include <gtest/gtest.h>

#include <cmath>
#include <graspsort_perception/localizer.hpp>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "oblique_render.hpp"

namespace gp = graspsort::perception;

namespace {

using namespace graspsort_test;  // NOLINT: test-only renderer (oblique_render.hpp)

constexpr double kBottleSx = 0.097;
constexpr double kBottleSy = 0.067;
constexpr double kBottleH = 0.191;
constexpr double kBallR = 0.0375;

HitFn bottleAt(const Eigen::Vector2d& xy, double yaw) {
  const BoxObj b{xy, yaw, kBottleSx, kBottleSy, kBottleH};
  return [b](const Eigen::Vector3d& o, const Eigen::Vector3d& d) { return hitBox(o, d, b); };
}

HitFn ballAt(const Eigen::Vector2d& xy) {
  return [xy](const Eigen::Vector3d& o, const Eigen::Vector3d& d) {
    return hitSphere(o, d, xy, kBallR);
  };
}

std::optional<gp::ObjectSample> localizeOnly(const HitFn& hit, double confidence = 0.7) {
  const Rendered r = render({hit});
  return gp::localizeBox(r.depth, r.boxes.at(0), kK, cameraPose(), gp::LocalizerConfig{},
                         confidence);
}

}  // namespace

// ================================ box conversion =============================================

TEST(BoxFromCenterSize, InvertsTheDetectorConversion) {
  // object_detector_node: centre = (x + w / 2, y + h / 2), size = (w, h) for cv::Rect(x, y, w, h).
  for (const cv::Rect& in :
       {cv::Rect(10, 5, 11, 5), cv::Rect(0, 0, 1, 1), cv::Rect(37, 120, 64, 33)}) {
    const cv::Rect out =
        gp::boxFromCenterSize(in.x + in.width / 2.0, in.y + in.height / 2.0, in.width, in.height);
    EXPECT_EQ(out, in);
  }
}

TEST(BoxFromCenterSize, DegenerateBoxesAreEmpty) {
  EXPECT_TRUE(gp::boxFromCenterSize(10.0, 10.0, 0.0, 5.0).empty());
  EXPECT_TRUE(gp::boxFromCenterSize(10.0, 10.0, -3.0, 5.0).empty());
  EXPECT_TRUE(gp::boxFromCenterSize(10.0, 10.0, 0.4, 5.0).empty());  // rounds to zero width
  EXPECT_TRUE(
      gp::boxFromCenterSize(std::numeric_limits<double>::quiet_NaN(), 10.0, 5.0, 5.0).empty());
}

// ================================ localizeBox (D-12) =========================================

TEST(LocalizeBox, DefaultConfigUsesTheD12WideBand) {
  EXPECT_DOUBLE_EQ(gp::LocalizerConfig{}.footprint.depth_band, 0.10);
  EXPECT_DOUBLE_EQ(gp::LocalizerConfig{}.depth.percentile, 0.20);
  EXPECT_DOUBLE_EQ(gp::LocalizerConfig{}.depth.central_fraction, 0.5);
}

TEST(LocalizeBox, BottleCentreSizeYawAndHeightAtSeveralYaws) {
  const Eigen::Vector2d xy(0.38, -0.08);  // bottle_1
  for (double yaw_deg : {0.0, 30.0, 60.0, 90.0, -45.0}) {
    const auto s = localizeOnly(bottleAt(xy, deg(yaw_deg)));
    ASSERT_TRUE(s.has_value()) << yaw_deg;
    EXPECT_LT((s->position.head<2>() - xy).norm(), 0.002) << yaw_deg;
    EXPECT_NEAR(s->position.z(), kTableZ + kBottleH / 2.0, 0.001) << yaw_deg;
    EXPECT_NEAR(s->size_x, kBottleSx, 0.0015) << yaw_deg;
    EXPECT_NEAR(s->size_y, kBottleSy, 0.0015) << yaw_deg;
    EXPECT_NEAR(s->height, kBottleH, 0.001) << yaw_deg;
    EXPECT_NEAR(gp::wrapHalfPi(s->yaw - deg(yaw_deg)), 0.0, deg(0.5)) << yaw_deg;
  }
}

TEST(LocalizeBox, BallCentreWithinSevenMillimetres) {
  // The far hemisphere is hidden from the oblique camera, so the footprint centre is biased
  // towards the camera by a few millimetres (D-12 table: 5.4 mm).
  for (const Eigen::Vector2d& xy : {Eigen::Vector2d(0.60, -0.12), Eigen::Vector2d(0.60, 0.12)}) {
    const auto s = localizeOnly(ballAt(xy));
    ASSERT_TRUE(s.has_value());
    EXPECT_LT((s->position.head<2>() - xy).norm(), 0.007);
    EXPECT_NEAR(s->position.z(), kTableZ + kBallR, 0.002);
    EXPECT_NEAR(s->height, 2.0 * kBallR, 0.001);
  }
}

TEST(LocalizeBox, ConfidenceIsPassedThrough) {
  const auto s = localizeOnly(ballAt({0.60, 0.12}), 0.42);
  ASSERT_TRUE(s.has_value());
  EXPECT_DOUBLE_EQ(s->confidence, 0.42);
}

TEST(LocalizeBox, TwoObjectsInOneImageAreLocalizedIndependently) {
  const Eigen::Vector2d bottle_xy(0.38, 0.10);  // bottle_2
  const Eigen::Vector2d ball_xy(0.60, 0.12);    // ball_2
  const Rendered r = render({bottleAt(bottle_xy, 0.0), ballAt(ball_xy)});
  const auto b = gp::localizeBox(r.depth, r.boxes.at(0), kK, cameraPose(), {}, 0.7);
  const auto s = gp::localizeBox(r.depth, r.boxes.at(1), kK, cameraPose(), {}, 0.7);
  ASSERT_TRUE(b.has_value());
  ASSERT_TRUE(s.has_value());
  EXPECT_LT((b->position.head<2>() - bottle_xy).norm(), 0.002);
  EXPECT_LT((s->position.head<2>() - ball_xy).norm(), 0.007);
}

TEST(LocalizeBox, BoxOnBareTableGivesNothing) {
  // Only table pixels: they are all below table_height + table_clearance.
  const Rendered r = render({});
  EXPECT_FALSE(
      gp::localizeBox(r.depth, cv::Rect(300, 220, 40, 40), kK, cameraPose(), {}, 0.7).has_value());
}

TEST(LocalizeBox, NoValidDepthGivesNothing) {
  const cv::Mat depth(kHeight, kWidth, CV_32FC1, cv::Scalar(0.0F));
  EXPECT_FALSE(
      gp::localizeBox(depth, cv::Rect(300, 220, 40, 40), kK, cameraPose(), {}, 0.7).has_value());
}

TEST(LocalizeBox, NonFloatDepthThrows) {
  const cv::Mat depth(kHeight, kWidth, CV_16UC1, cv::Scalar(1000));
  EXPECT_THROW(gp::localizeBox(depth, cv::Rect(300, 220, 40, 40), kK, cameraPose(), {}, 0.7),
               std::invalid_argument);
}

// ================================ shapes ======================================================

TEST(ApplyShape, CylinderGetsDiameterAndZeroYaw) {
  gp::ObjectSample s;
  s.size_x = 0.075;
  s.size_y = 0.041;
  s.yaw = 0.7;
  const auto c = gp::applyShape(s, gp::ObjectShape::kCylinder);
  EXPECT_DOUBLE_EQ(c.size_x, 0.075);
  EXPECT_DOUBLE_EQ(c.size_y, 0.075);
  EXPECT_DOUBLE_EQ(c.yaw, 0.0);
}

TEST(ApplyShape, BoxIsUnchanged) {
  gp::ObjectSample s;
  s.size_x = 0.097;
  s.size_y = 0.067;
  s.yaw = 0.5;
  const auto b = gp::applyShape(s, gp::ObjectShape::kBox);
  EXPECT_DOUBLE_EQ(b.size_x, 0.097);
  EXPECT_DOUBLE_EQ(b.size_y, 0.067);
  EXPECT_DOUBLE_EQ(b.yaw, 0.5);
}

TEST(ShapeTable, LooksUpConfiguredClasses) {
  const gp::ShapeTable t({"bottle", "sports ball"}, {1, 0});
  EXPECT_EQ(t.find("bottle"), gp::ObjectShape::kBox);
  EXPECT_EQ(t.find("sports ball"), gp::ObjectShape::kCylinder);
  EXPECT_FALSE(t.find("cup").has_value());
}

TEST(ShapeTable, RejectsInvalidConfig) {
  EXPECT_THROW(gp::ShapeTable({"bottle"}, {1, 0}), std::invalid_argument);
  EXPECT_THROW(gp::ShapeTable({"bottle"}, {2}), std::invalid_argument);
  EXPECT_THROW(gp::ShapeTable({""}, {1}), std::invalid_argument);
  EXPECT_THROW(gp::ShapeTable({"bottle", "bottle"}, {1, 1}), std::invalid_argument);
}
