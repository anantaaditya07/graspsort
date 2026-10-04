// GoogleTests for graspsort_manipulation/grasp_planner.hpp (architecture 7.4): box and cylinder
// candidates, width rejection, neighbour collision rejection and ranking order. Objects are the
// D-04 set on the Phase 1 table (top at z = 0.75 m): ball r 0.0375, cup r 0.0324 h 0.13,
// bottle box 0.097 x 0.067 x 0.191. Gripper: D-02, 90 mm max opening, 10 x 20 mm finger pads.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <graspsort_manipulation/grasp_planner.hpp>
#include <stdexcept>
#include <vector>

namespace gm = graspsort::manipulation;

namespace {

constexpr double kPi = EIGEN_PI;
constexpr double kTableZ = 0.75;
constexpr double kAngTol = 1e-9;

double deg(double d) { return d * kPi / 180.0; }

gm::Object cup(std::uint32_t id, double x, double y, double yaw = 0.0) {
  return {id, "cup", {x, y, kTableZ + 0.065}, {0.0648, 0.0648, 0.13}, yaw, gm::kShapeCylinder};
}
gm::Object ball(std::uint32_t id, double x, double y) {
  return {id,  "sports ball",     {x, y, kTableZ + 0.0375}, {0.075, 0.075, 0.075},
          0.0, gm::kShapeCylinder};
}
gm::Object bottle(std::uint32_t id, double x, double y, double yaw) {
  return {id, "bottle", {x, y, kTableZ + 0.0955}, {0.097, 0.067, 0.191}, yaw, gm::kShapeBox};
}

std::vector<double> yawsDeg(const std::vector<gm::GraspCandidate>& c) {
  std::vector<double> out;
  for (const auto& g : c) out.push_back(std::round(g.yaw * 180.0 / kPi * 1e6) / 1e6);
  return out;
}

}  // namespace

// ================================ geometry helpers ===========================================

TEST(Geometry, WrapToPiHalfOpen) {
  EXPECT_NEAR(gm::wrapToPi(kPi), -kPi, kAngTol);
  EXPECT_NEAR(gm::wrapToPi(-kPi), -kPi, kAngTol);
  EXPECT_NEAR(gm::wrapToPi(deg(270)), deg(-90), kAngTol);
  EXPECT_NEAR(gm::wrapToPi(deg(-190)), deg(170), kAngTol);
  EXPECT_NEAR(gm::wrapToPi(0.3 + 4.0 * kPi), 0.3, kAngTol);
}

TEST(Geometry, RectangleDistanceAndOverlap) {
  const gm::Rect2 a{{0.0, 0.0}, 0.05, 0.02, 0.0};
  const gm::Rect2 b{{0.10, 0.0}, 0.03, 0.03, 0.0};  // gap 0.10 - 0.05 - 0.03 = 0.02
  EXPECT_FALSE(gm::overlaps(a, b));
  EXPECT_NEAR(gm::distance(a, b), 0.02, 1e-12);
  const gm::Rect2 c{{0.07, 0.0}, 0.03, 0.03, deg(45)};  // corner reaches x = 0.07 - 0.0424
  EXPECT_TRUE(gm::overlaps(a, c));
  EXPECT_DOUBLE_EQ(gm::distance(a, c), 0.0);
  // Diagonal: rotated square whose corner points at a's corner.
  const gm::Rect2 d{{0.05 + 0.03, 0.02 + 0.03}, 0.01, 0.01, 0.0};
  EXPECT_NEAR(gm::distance(a, d), std::hypot(0.02, 0.02), 1e-12);
}

TEST(Geometry, RectangleToCircleDistance) {
  const gm::Rect2 a{{0.0, 0.0}, 0.005, 0.01, deg(90)};  // rotated: extends 0.01 along x
  EXPECT_NEAR(gm::distance(a, {0.05, 0.0}, 0.0375), 0.05 - 0.01 - 0.0375, 1e-12);
  EXPECT_DOUBLE_EQ(gm::distance(a, {0.03, 0.0}, 0.0375), 0.0);
}

// ================================ candidates =================================================

TEST(Candidates, BoxClosesAcrossShortSidePlus180) {
  const gm::GraspPlanner p;
  const auto c = p.generateCandidates(bottle(1, 0.38, -0.08, deg(30)));
  ASSERT_EQ(c.size(), 2U);
  // Short side (0.067) is the object's y axis: closing axis at 30 + 90 = 120 deg, and -60 deg.
  EXPECT_NEAR(c[0].yaw, deg(120), kAngTol);
  EXPECT_NEAR(c[1].yaw, deg(-60), kAngTol);
  for (const auto& g : c) {
    EXPECT_DOUBLE_EQ(g.width, 0.067);
    EXPECT_NEAR(g.position.x(), 0.38, 1e-12);
    EXPECT_NEAR(g.position.y(), -0.08, 1e-12);
    EXPECT_NEAR(g.position.z(), kTableZ + 0.6 * 0.191, 1e-12);  // fingers at 60% of height
  }
}

TEST(Candidates, BoxShortSideAlongXUsesObjectYaw) {
  const gm::GraspPlanner p;
  gm::Object o = bottle(1, 0.4, 0.0, deg(170));
  o.size = {0.067, 0.097, 0.191};
  const auto c = p.generateCandidates(o);
  ASSERT_EQ(c.size(), 2U);
  EXPECT_NEAR(c[0].yaw, deg(170), kAngTol);
  EXPECT_NEAR(c[1].yaw, deg(-10), kAngTol);  // 350 deg wrapped
}

TEST(Candidates, BoxYawWrapsAtPlusMinusPi) {
  const gm::GraspPlanner p;
  // Closing axis at 90 + 90 = 180 deg -> -180 deg, and 0 deg.
  const auto c = p.generateCandidates(bottle(1, 0.4, 0.0, deg(90)));
  EXPECT_NEAR(c[0].yaw, -kPi, kAngTol);
  EXPECT_NEAR(c[1].yaw, 0.0, kAngTol);
  for (const auto& g : c) {
    EXPECT_GE(g.yaw, -kPi);
    EXPECT_LT(g.yaw, kPi);
  }
}

TEST(Candidates, CylinderEightYawsAndObjectYawIgnored) {
  const gm::GraspPlanner p;
  const auto c = p.generateCandidates(cup(1, 0.48, 0.2));
  EXPECT_EQ(yawsDeg(c), (std::vector<double>{0, 45, 90, 135, -180, -135, -90, -45}));
  for (const auto& g : c) {
    EXPECT_DOUBLE_EQ(g.width, 0.0648);
    EXPECT_NEAR(g.position.z(), kTableZ + 0.6 * 0.13, 1e-12);
  }
  EXPECT_EQ(yawsDeg(p.generateCandidates(cup(1, 0.48, 0.2, 0.7))), yawsDeg(c));
}

TEST(Candidates, CylinderYawCountIsAParameter) {
  gm::GraspPlannerConfig cfg;
  cfg.cylinder_yaw_count = 4;
  const auto c = gm::GraspPlanner(cfg).generateCandidates(ball(1, 0.6, -0.12));
  EXPECT_EQ(yawsDeg(c), (std::vector<double>{0, 90, -180, -90}));
  EXPECT_NEAR(c[0].position.z(), kTableZ + 0.6 * 0.075, 1e-12);
}

// ================================ width rejection ============================================

TEST(WidthRejection, D04ObjectsFitTheGripper) {
  const gm::GraspPlanner p;
  EXPECT_EQ(p.plan(cup(1, 0.48, 0.2), {}, 0.0).size(), 8U);
  EXPECT_EQ(p.plan(ball(2, 0.6, -0.12), {}, 0.0).size(), 8U);
  EXPECT_EQ(p.plan(bottle(3, 0.38, -0.08, 0.0), {}, 0.0).size(), 2U);
}

TEST(WidthRejection, TooWideObjectIsRejectedAtTheMarginBoundary) {
  const gm::GraspPlanner p;  // limit = 0.09 - 0.01 = 0.08
  gm::Object at_limit = bottle(1, 0.4, 0.0, 0.0);
  at_limit.size = {0.12, 0.08, 0.191};
  EXPECT_EQ(p.plan(at_limit, {}, 0.0).size(), 2U);
  gm::Object over = at_limit;
  over.size = {0.12, 0.0801, 0.191};
  EXPECT_TRUE(p.plan(over, {}, 0.0).empty());
  const auto all = p.evaluate(over, {}, 0.0);
  ASSERT_EQ(all.size(), 2U);
  for (const auto& c : all) EXPECT_EQ(c.rejection, gm::Rejection::kTooWide);
  // A cylinder wider than the limit (e.g. a 0.085 m can) loses every yaw.
  gm::Object wide_cyl = cup(2, 0.48, 0.2);
  wide_cyl.size = {0.085, 0.085, 0.12};
  EXPECT_TRUE(p.plan(wide_cyl, {}, 0.0).empty());
}

TEST(MaxGraspWidth, FootprintWidthAcrossTheClosingAxis) {
  const gm::Object b = bottle(1, 0.4, 0.0, 0.0);                           // 0.097 x 0.067 at yaw 0
  EXPECT_NEAR(gm::footprintWidthAcross(b, kPi / 2.0), b.size.y(), 1e-12);  // across short side
  EXPECT_NEAR(gm::footprintWidthAcross(b, 0.0), b.size.x(), 1e-12);        // across long side
  // 30 deg off the short side: |sx sin 30| + |sy cos 30|.
  EXPECT_NEAR(gm::footprintWidthAcross(b, kPi / 2.0 + deg(30.0)),
              b.size.x() * std::sin(deg(30.0)) + b.size.y() * std::cos(deg(30.0)), 1e-12);
  // Cylinder: the diameter at any yaw.
  EXPECT_NEAR(gm::footprintWidthAcross(ball(2, 0.6, 0.1), deg(73.0)), 0.075, 1e-12);
}

TEST(MaxGraspWidth, DefaultIs85MillimetresAndD04ObjectsPass) {
  EXPECT_DOUBLE_EQ(gm::GraspPlannerConfig{}.max_grasp_width, 0.085);
  const gm::GraspPlanner p;
  for (const auto& c : p.evaluate(bottle(1, 0.38, -0.08, deg(20.0)), {}, 0.0)) {
    EXPECT_NE(c.rejection, gm::Rejection::kExceedsMaxGraspWidth);
  }
}

TEST(MaxGraspWidth, RejectsCandidateWiderThanTheLimitEvenIfItFitsTheOpening) {
  // Opening check relaxed (0.09 - 0 = 0.09) so only the max_grasp_width safety check can fire.
  gm::GraspPlannerConfig cfg;
  cfg.width_margin = 0.0;
  const gm::GraspPlanner p(cfg);
  gm::Object cyl = ball(1, 0.6, 0.1);
  cyl.size = {0.088, 0.088, 0.12};  // fits the 0.09 opening, exceeds 0.085
  EXPECT_TRUE(p.plan(cyl, {}, 0.0).empty());
  for (const auto& c : p.evaluate(cyl, {}, 0.0)) {
    EXPECT_EQ(c.rejection, gm::Rejection::kExceedsMaxGraspWidth);
  }
  // Exactly at the limit is accepted (the check is "exceeds").
  cyl.size = {0.085, 0.085, 0.12};
  EXPECT_EQ(p.plan(cyl, {}, 0.0).size(), cfg.cylinder_yaw_count);
}

TEST(MaxGraspWidth, IsAParameter) {
  gm::GraspPlannerConfig cfg;
  cfg.max_grasp_width = 0.06;  // narrower than the bottle's 0.067 short side
  const gm::GraspPlanner p(cfg);
  const auto all = p.evaluate(bottle(1, 0.38, -0.08, 0.0), {}, 0.0);
  ASSERT_EQ(all.size(), 2U);
  for (const auto& c : all) EXPECT_EQ(c.rejection, gm::Rejection::kExceedsMaxGraspWidth);
}

// ================================ neighbour rejection ========================================

TEST(NeighbourRejection, BallTouchingTheFingerLineRejectsThoseYaws) {
  const gm::GraspPlanner p;
  const gm::Object target = cup(1, 0.48, 0.2);
  // Ball surface at 0.09 - 0.0375 = 0.0525 m along +x: overlaps the finger (0.045..0.055 m) of
  // the 0 and 180 deg candidates.
  const std::vector<gm::Object> scene{target, ball(2, 0.48 + 0.09, 0.2)};
  const auto all = p.evaluate(target, scene, 0.0);
  std::vector<double> rejected;
  for (const auto& c : all) {
    if (c.rejection == gm::Rejection::kFingerCollision) {
      rejected.push_back(std::round(c.yaw * 180.0 / kPi));
      EXPECT_DOUBLE_EQ(c.clearance, 0.0);
    }
  }
  EXPECT_EQ(rejected, (std::vector<double>{0, -180}));
  EXPECT_EQ(p.plan(target, scene, 0.0).size(), 6U);
}

TEST(NeighbourRejection, BoxNeighbourCloserThanMinClearanceIsRejected) {
  const gm::GraspPlanner p;  // min_finger_clearance 0.005
  const gm::Object target = cup(1, 0.48, 0.2);
  // Bottle near face at 0.09 - 0.0335 = 0.0565 m along +y: 1.5 mm from the +-90 deg fingers
  // (outer face 0.055 m), no overlap but under the 5 mm clearance.
  const gm::Object nb = bottle(2, 0.48, 0.2 + 0.09, 0.0);
  const auto all = p.evaluate(target, {nb}, 0.0);
  std::vector<double> rejected;
  for (const auto& c : all) {
    if (c.rejection == gm::Rejection::kFingerCollision) {
      rejected.push_back(std::round(c.yaw * 180.0 / kPi));
      EXPECT_NEAR(c.clearance, 0.0015, 1e-9);
    }
  }
  EXPECT_EQ(rejected, (std::vector<double>{90, -90}));
  // With zero required clearance only overlap counts: all 8 accepted.
  gm::GraspPlannerConfig cfg;
  cfg.min_finger_clearance = 0.0;
  EXPECT_EQ(gm::GraspPlanner(cfg).plan(target, {nb}, 0.0).size(), 8U);
}

TEST(NeighbourRejection, BoxedInObjectHasNoGrasp) {
  const gm::GraspPlanner p;
  const gm::Object target = cup(1, 0.48, 0.0);
  const std::vector<gm::Object> ring{ball(2, 0.48 + 0.09, 0.0),     ball(3, 0.48 - 0.09, 0.0),
                                     ball(4, 0.48, 0.09),           ball(5, 0.48, -0.09),
                                     ball(6, 0.48 + 0.064, 0.064),  ball(7, 0.48 - 0.064, 0.064),
                                     ball(8, 0.48 + 0.064, -0.064), ball(9, 0.48 - 0.064, -0.064)};
  EXPECT_TRUE(p.plan(target, ring, 0.0).empty());
  for (const auto& c : p.evaluate(target, ring, 0.0)) {
    EXPECT_EQ(c.rejection, gm::Rejection::kFingerCollision);
  }
}

TEST(NeighbourRejection, TargetInSceneIsSkippedById) {
  const gm::GraspPlanner p;
  const gm::Object target = bottle(7, 0.38, -0.08, deg(30));
  const auto c = p.plan(target, {target}, 0.0);
  ASSERT_EQ(c.size(), 2U);
  EXPECT_DOUBLE_EQ(c[0].clearance, p.config().clearance_cap);
}

// ================================ ranking ====================================================

TEST(Ranking, ClosestYawToWristFirst) {
  const gm::GraspPlanner p;
  const auto c = p.plan(cup(1, 0.48, 0.2), {}, deg(100));
  // |yaw - 100 deg| wrapped: 90:10, 135:35, 45:55, -180:80, 0:100, -135:125, -45:145, -90:170.
  EXPECT_EQ(yawsDeg(c), (std::vector<double>{90, 135, 45, -180, 0, -135, -45, -90}));
}

TEST(Ranking, YawDifferenceWrapsAcrossPi) {
  const gm::GraspPlanner p;
  // Wrist at 170 deg: the -60 deg candidate is 130 deg away, the 120 deg one 50 deg.
  const auto c = p.plan(bottle(1, 0.38, -0.08, deg(30)), {}, deg(170));
  EXPECT_EQ(yawsDeg(c), (std::vector<double>{120, -60}));
  // Wrist at -170 deg: 120 deg is 70 deg away (across the seam), -60 deg 110 deg.
  EXPECT_EQ(yawsDeg(p.plan(bottle(1, 0.38, -0.08, deg(30)), {}, deg(-170))),
            (std::vector<double>{120, -60}));
}

TEST(Ranking, LargerClearanceBreaksYawTie) {
  const gm::GraspPlanner p;
  const gm::Object target = cup(1, 0.48, 0.2);
  // Wrist at 22.5 deg: 0 and 45 deg tie on yaw. A ball 2 cm beyond the 0 deg finger lowers the
  // clearance of 0 / 180 deg, so 45 deg ranks first.
  const std::vector<gm::Object> scene{ball(2, 0.48 + 0.055 + 0.02 + 0.0375, 0.2)};
  const auto c = p.plan(target, scene, deg(22.5));
  ASSERT_EQ(c.size(), 8U);
  EXPECT_NEAR(c[0].yaw, deg(45), kAngTol);
  EXPECT_NEAR(c[1].yaw, 0.0, kAngTol);
  EXPECT_NEAR(c[1].clearance, 0.02, 1e-9);
  EXPECT_GT(c[0].clearance, c[1].clearance);
}

TEST(Ranking, ExactTieKeepsGenerationOrder) {
  const gm::GraspPlanner p;
  // No neighbours, wrist at 22.5 deg: 0 deg (index 0) and 45 deg (index 1) have equal cost.
  const auto c = p.plan(cup(1, 0.48, 0.2), {}, deg(22.5));
  EXPECT_EQ(c[0].index, 0U);
  EXPECT_EQ(c[1].index, 1U);
  EXPECT_NEAR(c[0].cost, c[1].cost, 1e-12);
}

TEST(Ranking, WeightsTradeYawAgainstClearance) {
  const gm::Object target = cup(1, 0.48, 0.2);
  // Neighbour 2 cm beyond the 0 deg finger; wrist at 0 deg.
  const std::vector<gm::Object> scene{ball(2, 0.48 + 0.055 + 0.02 + 0.0375, 0.2)};
  gm::GraspPlannerConfig yaw_only;
  yaw_only.clearance_weight = 0.0;
  EXPECT_NEAR(gm::GraspPlanner(yaw_only).plan(target, scene, 0.0)[0].yaw, 0.0, kAngTol);
  // Clearance dominates: 45 or -45 deg (more clearance) beats 0 deg.
  gm::GraspPlannerConfig clear_only;
  clear_only.yaw_weight = 0.0;
  const auto c = gm::GraspPlanner(clear_only).plan(target, scene, 0.0);
  EXPECT_NE(std::abs(std::round(c[0].yaw * 180.0 / kPi)), 0.0);
  EXPECT_NE(std::abs(std::round(c[0].yaw * 180.0 / kPi)), 180.0);
  // Default weights (1 / rad, 10 / m): 0 deg costs 0 - 10 * 0.02 = -0.2; +-45 deg cost
  // 0.785 - 10 * clearance(~0.04) > 0, so the wrist-aligned grasp still wins with 2 cm clearance.
  const auto d = gm::GraspPlanner().plan(target, scene, 0.0);
  EXPECT_NEAR(d[0].yaw, 0.0, kAngTol);
  EXPECT_NEAR(d[0].cost, -10.0 * 0.02, 1e-9);
}

TEST(Planner, InvalidConfigThrows) {
  gm::GraspPlannerConfig c;
  c.cylinder_yaw_count = 0;
  EXPECT_THROW(gm::GraspPlanner{c}, std::invalid_argument);
  c = {};
  c.finger_height_fraction = 1.5;
  EXPECT_THROW(gm::GraspPlanner{c}, std::invalid_argument);
  c = {};
  c.max_opening = 0.0;
  EXPECT_THROW(gm::GraspPlanner{c}, std::invalid_argument);
  c = {};
  c.max_grasp_width = 0.0;
  EXPECT_THROW(gm::GraspPlanner{c}, std::invalid_argument);
}

TEST(Planner, RejectionReasonStrings) {
  EXPECT_STREQ(gm::toString(gm::Rejection::kTooWide), "object wider than gripper opening");
  EXPECT_STREQ(gm::toString(gm::Rejection::kFingerCollision),
               "finger would hit a neighbouring object");
  EXPECT_STREQ(gm::toString(gm::Rejection::kExceedsMaxGraspWidth),
               "footprint across the fingers exceeds max_grasp_width");
}
