#include <gtest/gtest.h>

#include <cmath>

#include "graspsort_manipulation/grasp_planner.hpp"
#include "graspsort_manipulation/pick_place_geometry.hpp"

namespace gm = graspsort::manipulation;

namespace {

constexpr double kEps = 1e-9;
// World layout values (graspsort_gazebo/config/world_layout.yaml) and D-02/D-14 objects.
constexpr double kTableZ = 0.75;
constexpr double kBallRadius = 0.0375;
constexpr double kBottleHeight = 0.1913;
constexpr double kBottleShort = 0.0582;

gm::BinGeometry binAt(double x, double y, double yaw = 0.0) {
  gm::BinGeometry b;
  b.pose = Eigen::Isometry3d::Identity();
  b.pose.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  b.pose.translation() = Eigen::Vector3d(x, y, kTableZ);
  return b;
}

}  // namespace

TEST(PickPlaceGeometry, TopDownOrientationPointsDownWithClosingAxisAtYaw) {
  for (const double yaw : {0.0, 0.3, gm::kPi / 2.0, -2.5, 3.0}) {
    const Eigen::Matrix3d r = gm::topDownOrientation(yaw).toRotationMatrix();
    EXPECT_NEAR((r.col(2) - Eigen::Vector3d(0, 0, -1)).norm(), 0.0, kEps);
    EXPECT_NEAR((r.col(1) - Eigen::Vector3d(std::cos(yaw), std::sin(yaw), 0)).norm(), 0.0, kEps);
    EXPECT_NEAR(r.determinant(), 1.0, kEps);
    EXPECT_NEAR(gm::wrapToPi(gm::closingYaw(gm::topDownOrientation(yaw)) - yaw), 0.0, 1e-9);
  }
}

TEST(PickPlaceGeometry, YawZeroIsTcpXAlongWorldMinusY) {
  const Eigen::Matrix3d r = gm::topDownOrientation(0.0).toRotationMatrix();
  EXPECT_NEAR((r.col(0) - Eigen::Vector3d(0, 1, 0)).norm(), 0.0, kEps);
  EXPECT_NEAR((r.col(1) - Eigen::Vector3d(1, 0, 0)).norm(), 0.0, kEps);
}

TEST(PickPlaceGeometry, RaisedMovesOnlyZ) {
  const auto p = gm::makePose({0.5, -0.1, 0.8}, gm::topDownOrientation(0.7));
  const auto q = gm::raised(p, 0.1);
  EXPECT_NEAR((q.translation() - Eigen::Vector3d(0.5, -0.1, 0.9)).norm(), 0.0, kEps);
  EXPECT_TRUE(q.linear().isApprox(p.linear()));
}

// A 75 mm ball on the table: the planner's 60% grasp height puts the TCP 7.5 mm above the
// centre; the fingertips (20 mm below the TCP) are 12.5 mm below the equator and 25 mm above the
// table, so the default config keeps the planner's z.
TEST(PickPlaceGeometry, BallGraspFingertipsBelowEquatorAndAboveTable) {
  gm::PickPlaceConfig c;
  gm::Object ball;
  ball.center = {0.6, 0.12, kTableZ + kBallRadius};
  ball.size = {2 * kBallRadius, 2 * kBallRadius, 2 * kBallRadius};
  const auto cands = gm::GraspPlanner().generateCandidates(ball);
  ASSERT_FALSE(cands.empty());
  const double pz = cands.front().position.z();
  EXPECT_NEAR(pz, kTableZ + 0.045, kEps);
  const auto z = gm::adjustedGraspZ(pz, ball.center.z(), ball.size.z(), c);
  ASSERT_TRUE(z.has_value());
  EXPECT_NEAR(*z, pz, kEps);
  const double fingertip = *z - c.tcp_to_fingertip;
  EXPECT_LT(fingertip, ball.center.z() - 0.01);
  EXPECT_GT(fingertip, kTableZ + 0.02);
}

TEST(PickPlaceGeometry, GraspZLoweredToReachMidHeight) {
  gm::PickPlaceConfig c;
  c.max_fingertip_above_center = -0.005;
  // Planner z far above: fingertips must come down to centre - 5 mm.
  const auto z = gm::adjustedGraspZ(1.0, 0.80, 0.10, c);
  ASSERT_TRUE(z.has_value());
  EXPECT_NEAR(*z, 0.80 - 0.005 + c.tcp_to_fingertip, kEps);
}

TEST(PickPlaceGeometry, GraspZRaisedForTableClearance) {
  gm::PickPlaceConfig c;
  const auto z = gm::adjustedGraspZ(0.70, 0.80, 0.10, c);
  ASSERT_TRUE(z.has_value());
  EXPECT_NEAR(*z, 0.75 + c.min_fingertip_clearance + c.tcp_to_fingertip, kEps);
}

TEST(PickPlaceGeometry, GraspZRejectedForFlatObject) {
  gm::PickPlaceConfig c;
  // 10 mm high object: fingertips cannot be both 10 mm above the bottom and at the mid-height.
  EXPECT_FALSE(gm::adjustedGraspZ(0.76, 0.755, 0.01, c).has_value());
}

TEST(PickPlaceGeometry, BottleGraspKeepsPlannerZ) {
  gm::PickPlaceConfig c;
  const double cz = kTableZ + kBottleHeight / 2.0;
  const double pz = kTableZ + 0.6 * kBottleHeight;
  const auto z = gm::adjustedGraspZ(pz, cz, kBottleHeight, c);
  ASSERT_TRUE(z.has_value());
  EXPECT_NEAR(*z, pz, kEps);
}

TEST(PickPlaceGeometry, PregraspFingertipsAboveObjectTopForBall) {
  gm::PickPlaceConfig c;
  const double grasp_z = kTableZ + 0.045;
  const double pre_fingertip = grasp_z + c.pregrasp_height - c.tcp_to_fingertip;
  EXPECT_NEAR(pre_fingertip - (kTableZ + 2 * kBallRadius), 0.05, kEps);
}

// Bottle (D-14): with the pre-grasp 10 cm above a grasp at 60% height the fingertips are only
// 3.5 mm above the bottle top, but the open fingers (inner faces at +-45 mm) straddle the 58 mm
// body, so the pre-grasp and the approach are collision-free in the footprint.
TEST(PickPlaceGeometry, PregraspBottleFingersStraddleBody) {
  gm::PickPlaceConfig c;
  const double grasp_z = kTableZ + 0.6 * kBottleHeight;
  const double pre_fingertip = grasp_z + c.pregrasp_height - c.tcp_to_fingertip;
  EXPECT_NEAR(pre_fingertip - (kTableZ + kBottleHeight), 0.08 - 0.4 * kBottleHeight, kEps);
  EXPECT_GT(c.open_width / 2.0, kBottleShort / 2.0 + 0.015);
}

TEST(PickPlaceGeometry, HangBelowTcp) {
  EXPECT_NEAR(gm::hangBelowTcp(0.795, 0.7875, 0.075), 0.045, kEps);
}

TEST(PickPlaceGeometry, ClosePositionIsWidthMinusSqueeze) {
  gm::PickPlaceConfig c;
  const auto q = gm::closePosition(kBottleShort, c);
  ASSERT_TRUE(q.has_value());
  EXPECT_NEAR(c.open_width - 2.0 * *q, kBottleShort - c.squeeze, kEps);
  const auto qb = gm::closePosition(2 * kBallRadius, c);
  ASSERT_TRUE(qb.has_value());
  EXPECT_NEAR(*qb, (0.09 - 0.075 + c.squeeze) / 2.0, kEps);
}

TEST(PickPlaceGeometry, ClosePositionRejectsTooWideOrTooThin) {
  gm::PickPlaceConfig c;
  EXPECT_FALSE(gm::closePosition(0.095, c).has_value());
  EXPECT_FALSE(gm::closePosition(0.005, c).has_value());  // would need a full close
  EXPECT_FALSE(gm::closePosition(0.0, c).has_value());
}

TEST(PickPlaceGeometry, ReleaseAndAboveBinPoses) {
  gm::PickPlaceConfig c;
  const auto bin = binAt(0.5, 0.42);
  const auto q = gm::topDownOrientation(0.4);
  const double hang = 0.045;
  const auto rel = gm::releasePose(bin, hang, q, c);
  EXPECT_NEAR((rel.translation() - Eigen::Vector3d(0.5, 0.42, 0.76 + 0.02 + 0.045)).norm(), 0.0,
              kEps);
  const auto above = gm::aboveBinPose(bin, hang, q, c);
  EXPECT_NEAR(above.translation().z(), 0.83 + 0.05 + 0.045, kEps);
  EXPECT_TRUE(above.linear().isApprox(q.toRotationMatrix()));
  // Held object bottom is between floor and rim at release.
  const double bottom = rel.translation().z() - hang;
  EXPECT_GT(bottom, bin.floorTopZ());
  EXPECT_LT(bottom, bin.rimZ());
}

TEST(PickPlaceGeometry, BinInnerArea) {
  const auto bin = binAt(0.35, -0.42);
  EXPECT_TRUE(bin.insideInner({0.35, -0.42, 0.8}));
  EXPECT_TRUE(bin.insideInner({0.35 + 0.089, -0.42 - 0.089, 0.8}));
  EXPECT_FALSE(bin.insideInner({0.35 + 0.091, -0.42, 0.8}));
  EXPECT_FALSE(bin.insideInner({0.35 + 0.08, -0.42, 0.8}, 0.02));
  const auto rotated = binAt(0.0, 0.0, gm::kPi / 4.0);
  // (0.06, 0.06) is 0.085 m along the rotated bin's x axis: inside; (0.07, 0.07) is 0.099 m.
  EXPECT_TRUE(rotated.insideInner({0.06, 0.06, 0.8}));
  EXPECT_FALSE(rotated.insideInner({0.07, 0.07, 0.8}));
  // (0, 0.1) is (0.071, 0.071) in the bin frame: inside although |y| > 0.09 in the world.
  EXPECT_TRUE(rotated.insideInner({0.0, 0.1, 0.8}));
}

TEST(PickPlaceGeometry, OpenFingersFitInBin) {
  gm::PickPlaceConfig c;
  const auto bin = binAt(0.5, 0.42);
  // 90 mm gap + 2 x 10 mm fingers = 110 mm < 180 mm inner, at any yaw.
  for (const double yaw : {0.0, 0.5, gm::kPi / 4.0, gm::kPi / 2.0, -2.0}) {
    EXPECT_TRUE(gm::openFingersFitInBin(bin, yaw, 0.01, 0.02, c));
  }
  c.open_width = 0.17;
  EXPECT_FALSE(gm::openFingersFitInBin(bin, 0.0, 0.01, 0.02, c));
}

TEST(PickPlaceGeometry, ValidateRejectsBadConfig) {
  gm::PickPlaceConfig c;
  EXPECT_NO_THROW(gm::validate(c));
  c.pregrasp_height = 0.0;
  EXPECT_THROW(gm::validate(c), std::invalid_argument);
  c = gm::PickPlaceConfig{};
  c.squeeze = -0.001;
  EXPECT_THROW(gm::validate(c), std::invalid_argument);
}
