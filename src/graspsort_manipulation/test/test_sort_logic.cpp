#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "graspsort_manipulation/sort_logic.hpp"

namespace gm = graspsort::manipulation;

namespace {

// World layout values (graspsort_gazebo/config/world_layout.yaml) and D-10/D-14 objects.
constexpr double kTableZ = 0.75;
constexpr double kBottleLong = 0.0958;
constexpr double kBottleShort = 0.0582;
constexpr double kBottleHeight = 0.1913;
constexpr double kBallDiameter = 0.075;
constexpr double kFingerT = 0.01;
constexpr double kFingerW = 0.02;

gm::BinGeometry binAt(double x, double y, double yaw = 0.0) {
  gm::BinGeometry b;
  b.pose = Eigen::Isometry3d::Identity();
  b.pose.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  b.pose.translation() = Eigen::Vector3d(x, y, kTableZ);
  return b;
}

gm::Object bottle(std::uint32_t id, double x, double y, double yaw = 0.0) {
  gm::Object o;
  o.id = id;
  o.class_name = "bottle";
  o.center = {x, y, kTableZ + kBottleHeight / 2.0};
  o.size = {kBottleLong, kBottleShort, kBottleHeight};
  o.yaw = yaw;
  o.shape = gm::kShapeBox;
  return o;
}

gm::Object ball(std::uint32_t id, double x, double y) {
  gm::Object o;
  o.id = id;
  o.class_name = "sports ball";
  o.center = {x, y, kTableZ + kBallDiameter / 2.0};
  o.size = {kBallDiameter, kBallDiameter, kBallDiameter};
  o.shape = gm::kShapeCylinder;
  return o;
}

std::vector<gm::NamedBin> layoutBins() {
  return {{"bin_cup", binAt(0.25, 0.42)},
          {"bin_bottle", binAt(0.35, -0.42)},
          {"bin_ball", binAt(0.50, 0.42)}};
}

// Every corner of the held object footprint and of both fingers inside the inner walls - margin.
bool slotInside(const gm::BinGeometry& bin, const gm::Object& held, const Eigen::Vector2d& xy,
                double closing_yaw, double gap, double margin) {
  const Eigen::Vector2d axis(std::cos(closing_yaw), std::sin(closing_yaw));
  const double off = gap / 2.0 + kFingerT / 2.0;
  std::vector<gm::Rect2> rects = {
      gm::Rect2{xy + off * axis, kFingerT / 2.0, kFingerW / 2.0, closing_yaw},
      gm::Rect2{xy - off * axis, kFingerT / 2.0, kFingerW / 2.0, closing_yaw},
      gm::Rect2{xy, held.size.x() / 2.0, held.size.y() / 2.0, held.yaw}};
  const Eigen::Vector2d h = bin.innerHalf();
  for (const auto& r : rects) {
    for (const auto& c : r.corners()) {
      const Eigen::Vector3d l = bin.pose.inverse() * Eigen::Vector3d(c.x(), c.y(), kTableZ);
      if (std::abs(l.x()) > h.x() - margin + 1e-9 || std::abs(l.y()) > h.y() - margin + 1e-9) {
        return false;
      }
    }
  }
  return true;
}

}  // namespace

// ---------------------------------------------------------------------------------------------
// Categories, reason strings, stages
// ---------------------------------------------------------------------------------------------

TEST(SortLogic, CategoryNamesMatchTheActionContract) {
  EXPECT_STREQ(gm::toString(gm::FailureCategory::kNoIk), "no_ik");
  EXPECT_STREQ(gm::toString(gm::FailureCategory::kPlanFailed), "plan_failed");
  EXPECT_STREQ(gm::toString(gm::FailureCategory::kGraspSlipped), "grasp_slipped");
  EXPECT_STREQ(gm::toString(gm::FailureCategory::kExecutionFailed), "execution_failed");
  EXPECT_STREQ(gm::toString(gm::FailureCategory::kMisdetection), "misdetection");
  EXPECT_STREQ(gm::toString(gm::FailureCategory::kUnreachable), "unreachable");
  EXPECT_STREQ(gm::toString(gm::FailureCategory::kNoGraspCandidate), "no_grasp_candidate");
  EXPECT_STREQ(gm::toString(gm::FailureCategory::kOther), "other");
}

TEST(SortLogic, FailureReasonFormatIsOneLine) {
  EXPECT_EQ(gm::formatFailureReason("object_7", "bottle", gm::FailureCategory::kNoIk, "pre\ngrasp"),
            "object_7 (bottle): no_ik: pre grasp");
  EXPECT_EQ(gm::objectLabel("object_", 12), "object_12");
}

TEST(SortLogic, EveryStepHasAnActionStage) {
  const std::set<std::string> allowed = {"detect", "plan_grasp", "approach",
                                         "grasp",  "transport",  "place"};
  for (const auto& s : gm::pickPlaceSteps()) {
    EXPECT_EQ(allowed.count(gm::actionStageFor(s)), 1u) << s;
  }
  EXPECT_EQ(gm::actionStageFor("lock_target"), "detect");
  EXPECT_EQ(gm::actionStageFor("plan_pregrasp"), "plan_grasp");
  EXPECT_EQ(gm::actionStageFor("approach"), "approach");
  EXPECT_EQ(gm::actionStageFor("close_gripper"), "grasp");
  EXPECT_EQ(gm::actionStageFor("move_above_bin"), "transport");
  EXPECT_EQ(gm::actionStageFor("retreat"), "place");
  EXPECT_EQ(gm::actionStageFor("unknown"), "");
}

TEST(SortLogic, StagesAreInActionOrder) {
  const std::vector<std::string> order = {"detect", "plan_grasp", "approach",
                                          "grasp",  "transport",  "place"};
  std::size_t last = 0;
  for (const auto& s : gm::pickPlaceSteps()) {
    const auto it = std::find(order.begin(), order.end(), gm::actionStageFor(s));
    const auto idx = static_cast<std::size_t>(it - order.begin());
    EXPECT_GE(idx, last) << s;
    last = idx;
  }
}

TEST(SortLogic, DefaultCategoryPerStep) {
  EXPECT_EQ(gm::categoryForStep("lock_target"), gm::FailureCategory::kMisdetection);
  EXPECT_EQ(gm::categoryForStep("plan_pregrasp"), gm::FailureCategory::kPlanFailed);
  EXPECT_EQ(gm::categoryForStep("close_gripper"), gm::FailureCategory::kGraspSlipped);
  EXPECT_EQ(gm::categoryForStep("attach_gazebo"), gm::FailureCategory::kGraspSlipped);
  EXPECT_EQ(gm::categoryForStep("attach_moveit"), gm::FailureCategory::kGraspSlipped);
  EXPECT_EQ(gm::categoryForStep("approach"), gm::FailureCategory::kExecutionFailed);
  EXPECT_EQ(gm::categoryForStep("lower"), gm::FailureCategory::kExecutionFailed);
  EXPECT_EQ(gm::categoryForStep("unfreeze_scene"), gm::FailureCategory::kOther);
  EXPECT_EQ(gm::categoryForStep("after lift"), gm::FailureCategory::kOther);
}

// ---------------------------------------------------------------------------------------------
// Bins, classes, filtering, ordering
// ---------------------------------------------------------------------------------------------

TEST(SortLogic, InsideFootprintUsesOuterSizeAndMargin) {
  const auto b = binAt(0.5, 0.42);
  EXPECT_TRUE(gm::insideFootprint(b, {0.5, 0.42, 0.8}, 0.0));
  EXPECT_TRUE(gm::insideFootprint(b, {0.599, 0.42, 0.8}, 0.0));
  EXPECT_FALSE(gm::insideFootprint(b, {0.61, 0.42, 0.8}, 0.0));
  EXPECT_TRUE(gm::insideFootprint(b, {0.61, 0.42, 0.8}, 0.02));
  EXPECT_FALSE(gm::insideFootprint(b, {0.5, 0.42 - 0.125, 0.8}, 0.02));
}

TEST(SortLogic, InsideFootprintRespectsBinYaw) {
  const auto b = binAt(0.0, 0.0, gm::kPi / 4.0);
  // (0.12, 0) is outside a 0.2 m axis-aligned square but inside the rotated one's corner reach.
  EXPECT_TRUE(gm::insideFootprint(b, {0.12, 0.0, 0.0}, 0.0));
  EXPECT_FALSE(gm::insideFootprint(b, {0.10, 0.10, 0.0}, 0.0));
}

TEST(SortLogic, BinContainingNamesTheBin) {
  const auto bins = layoutBins();
  EXPECT_EQ(gm::binContaining(bins, {0.52, 0.40, 0.8}, 0.02).value(), "bin_ball");
  EXPECT_EQ(gm::binContaining(bins, {0.35, -0.38, 0.8}, 0.02).value(), "bin_bottle");
  EXPECT_FALSE(gm::binContaining(bins, {0.60, 0.12, 0.8}, 0.02).has_value());
}

TEST(SortLogic, GoalClassesEmptyMeansAllBinClasses) {
  const auto b = gm::BinAssignment::defaults();
  EXPECT_EQ(gm::goalClasses({}, b), (std::vector<std::string>{"bottle", "cup", "sports ball"}));
  EXPECT_EQ(gm::goalClasses({"bottle", "bottle"}, b), (std::vector<std::string>{"bottle"}));
  EXPECT_THROW(gm::goalClasses({"chair"}, b), std::invalid_argument);
}

TEST(SortLogic, FilterKeepsRequestedClassesOutsideBinsNotGivenUp) {
  const auto bins = layoutBins();
  gm::SortBook book(3, 0.05);
  const std::vector<gm::Object> objs = {ball(1, 0.60, 0.12), ball(2, 0.52, 0.40),  // 2 in bin
                                        bottle(3, 0.38, 0.10), bottle(4, 0.38, -0.08)};
  EXPECT_EQ(gm::filterCandidates(objs, {"sports ball", "bottle"}, bins, 0.02, book),
            (std::vector<std::size_t>{0, 2, 3}));
  EXPECT_EQ(gm::filterCandidates(objs, {"bottle"}, bins, 0.02, book),
            (std::vector<std::size_t>{2, 3}));
  book.giveUp({0.385, 0.095, 0.8}, "bottle", "object_3", gm::FailureCategory::kUnreachable, "x");
  EXPECT_EQ(gm::filterCandidates(objs, {"sports ball", "bottle"}, bins, 0.02, book),
            (std::vector<std::size_t>{0, 3}));
}

TEST(SortLogic, OrderByDistanceClosestToBaseFirst) {
  const std::vector<gm::Object> objs = {ball(1, 0.60, 0.12), ball(2, 0.60, -0.12),
                                        bottle(3, 0.38, 0.10), bottle(4, 0.38, -0.08)};
  EXPECT_EQ(gm::orderByDistance(objs, {0, 1, 2, 3}, Eigen::Vector3d(0, 0, 0.75)),
            (std::vector<std::size_t>{3, 2, 0, 1}));
  EXPECT_EQ(gm::orderByDistance(objs, {1, 0}, Eigen::Vector3d(0, 0, 0.75)),
            (std::vector<std::size_t>{1, 0}));  // tie keeps input order
}

TEST(SortLogic, RotatedStartsAtTheRetryCandidate) {
  const std::vector<int> v = {10, 11, 12};
  EXPECT_EQ(gm::rotated(v, 0), v);
  EXPECT_EQ(gm::rotated(v, 1), (std::vector<int>{11, 12, 10}));
  EXPECT_EQ(gm::rotated(v, 4), (std::vector<int>{11, 12, 10}));
  EXPECT_TRUE(gm::rotated(std::vector<int>{}, 2).empty());
}

// ---------------------------------------------------------------------------------------------
// SortBook: retries and give-up by position
// ---------------------------------------------------------------------------------------------

TEST(SortLogic, ThreeFailedAttemptsGiveUp) {
  gm::SortBook book(3, 0.05);
  const Eigen::Vector3d p(0.4, 0.1, 0.8);
  EXPECT_FALSE(book.recordFailure(p, "bottle", "object_3", gm::FailureCategory::kPlanFailed, "a"));
  EXPECT_TRUE(book.matchesRetry({0.42, 0.11, 0.8}, "bottle"));  // moved 2 cm, new id
  EXPECT_EQ(book.attemptsSoFar({0.42, 0.11, 0.8}, "bottle"), 1);
  EXPECT_FALSE(
      book.recordFailure({0.42, 0.11, 0.8}, "bottle", "object_9", gm::FailureCategory::kNoIk, "b"));
  EXPECT_FALSE(book.givenUp(p, "bottle"));
  EXPECT_TRUE(book.recordFailure({0.42, 0.11, 0.8}, "bottle", "object_9",
                                 gm::FailureCategory::kExecutionFailed, "c"));
  EXPECT_FALSE(book.retryTarget().has_value());
  EXPECT_TRUE(book.givenUp({0.43, 0.12, 0.8}, "bottle"));
  EXPECT_FALSE(book.givenUp({0.43, 0.12, 0.8}, "sports ball"));
  EXPECT_FALSE(book.givenUp({0.6, 0.12, 0.8}, "bottle"));
  ASSERT_EQ(book.failureReasons().size(), 1u);
  EXPECT_EQ(book.failureReasons()[0], "object_9 (bottle): execution_failed: c");
  EXPECT_EQ(book.givenUpObjects()[0].attempts, 3);
}

TEST(SortLogic, SuccessAfterRetryClearsTheRetryTarget) {
  gm::SortBook book(3, 0.05);
  book.recordFailure({0.4, 0.1, 0.8}, "bottle", "object_3", gm::FailureCategory::kPlanFailed, "a");
  ASSERT_TRUE(book.retryTarget().has_value());
  book.recordSuccess();
  EXPECT_FALSE(book.retryTarget().has_value());
  EXPECT_TRUE(book.failureReasons().empty());
  EXPECT_EQ(book.attemptsSoFar({0.4, 0.1, 0.8}, "bottle"), 0);
}

TEST(SortLogic, OtherObjectDoesNotInheritRetryAttempts) {
  gm::SortBook book(3, 0.05);
  book.recordFailure({0.4, 0.1, 0.8}, "bottle", "object_3", gm::FailureCategory::kPlanFailed, "a");
  EXPECT_FALSE(book.matchesRetry({0.4, -0.08, 0.8}, "bottle"));
  EXPECT_EQ(book.attemptsSoFar({0.4, -0.08, 0.8}, "bottle"), 0);
}

TEST(SortLogic, GiveUpRetryKeepsHistory) {
  gm::SortBook book(3, 0.05);
  book.recordFailure({0.4, 0.1, 0.8}, "bottle", "object_3", gm::FailureCategory::kGraspSlipped,
                     "attach failed");
  book.giveUpRetry(gm::FailureCategory::kMisdetection, "not perceived again");
  ASSERT_EQ(book.failureReasons().size(), 1u);
  EXPECT_EQ(book.failureReasons()[0],
            "object_3 (bottle): misdetection: not perceived again (after 1 failed attempt; "
            "last: grasp_slipped: attach failed)");
  book.giveUpRetry(gm::FailureCategory::kOther, "no-op without a retry target");
  EXPECT_EQ(book.failureReasons().size(), 1u);
}

TEST(SortLogic, NeverAttemptedObjectsCountAsFailed) {
  gm::SortBook book(3, 0.05);
  book.giveUp({0.9, 0.0, 0.8}, "sports ball", "object_5", gm::FailureCategory::kUnreachable,
              "no collision-free IK for the pre-grasp");
  EXPECT_EQ(book.failureReasons(),
            (std::vector<std::string>{"object_5 (sports ball): unreachable: no collision-free IK "
                                      "for the pre-grasp"}));
  EXPECT_EQ(book.givenUpObjects()[0].attempts, 0);
}

TEST(SortLogic, SortBookValidatesParameters) {
  EXPECT_THROW(gm::SortBook(0, 0.05), std::invalid_argument);
  EXPECT_THROW(gm::SortBook(3, 0.0), std::invalid_argument);
}

// ---------------------------------------------------------------------------------------------
// SettleDetector
// ---------------------------------------------------------------------------------------------

TEST(SortLogic, SettlesAfterStableWindow) {
  gm::SettleDetector s(2.0, 0.01);
  std::map<std::uint32_t, Eigen::Vector3d> m = {{1, {0.6, 0.1, 0.8}}, {2, {0.4, 0.0, 0.8}}};
  EXPECT_FALSE(s.update(0.0, m));
  EXPECT_FALSE(s.update(1.0, m));
  m[1].x() += 0.005;  // within tolerance
  EXPECT_FALSE(s.update(1.9, m));
  EXPECT_TRUE(s.update(2.0, m));
}

TEST(SortLogic, SettleRestartsOnMotionOrIdChange) {
  gm::SettleDetector s(2.0, 0.01);
  std::map<std::uint32_t, Eigen::Vector3d> m = {{1, {0.6, 0.1, 0.8}}};
  s.update(0.0, m);
  m[1].x() += 0.02;
  EXPECT_FALSE(s.update(1.5, m));  // moved: window restarts at 1.5
  EXPECT_FALSE(s.update(3.0, m));
  EXPECT_TRUE(s.update(3.5, m));
  auto m2 = m;
  m2[2] = {0.4, 0.0, 0.8};  // new track
  EXPECT_FALSE(s.update(4.0, m2));
  EXPECT_FALSE(s.update(5.0, m));  // track gone
  EXPECT_TRUE(s.update(7.0, m));
}

TEST(SortLogic, EmptyTableSettles) {
  gm::SettleDetector s(1.0, 0.01);
  EXPECT_FALSE(s.update(0.0, {}));
  EXPECT_TRUE(s.update(1.0, {}));
}

// ---------------------------------------------------------------------------------------------
// Release slots
// ---------------------------------------------------------------------------------------------

TEST(SortLogic, FootprintDistanceShapes) {
  EXPECT_NEAR(gm::footprintDistance(ball(1, 0, 0), ball(2, 0.1, 0)), 0.025, 1e-9);
  EXPECT_NEAR(gm::footprintDistance(bottle(1, 0, 0), bottle(2, 0, 0.07)), 0.07 - kBottleShort,
              1e-9);
  EXPECT_NEAR(gm::footprintDistance(bottle(1, 0, 0), ball(2, 0, 0.1)),
              0.1 - kBottleShort / 2.0 - kBallDiameter / 2.0, 1e-9);
  EXPECT_NEAR(gm::footprintDistance(ball(2, 0, 0.1), bottle(1, 0, 0)),
              0.1 - kBottleShort / 2.0 - kBallDiameter / 2.0, 1e-9);
  EXPECT_DOUBLE_EQ(gm::footprintDistance(bottle(1, 0, 0), bottle(2, 0.05, 0)), 0.0);
}

TEST(SortLogic, EmptyBinSlotIsInsideAndToTheSide) {
  const auto bin = binAt(0.35, -0.42);
  const auto b = bottle(1, 0.38, 0.10);
  const double gap = kBottleShort + 0.006;
  const gm::ReleaseSlotConfig cfg;
  const auto slot = gm::chooseReleaseSlot(bin, b, gm::kPi / 2.0, gap, kFingerT, kFingerW, {}, cfg);
  ASSERT_TRUE(slot.has_value());
  EXPECT_TRUE(slotInside(bin, b, slot->xy, gm::kPi / 2.0, gap, cfg.wall_margin));
  EXPECT_GT((slot->xy - bin.pose.translation().head<2>()).norm(), 0.03);
  EXPECT_DOUBLE_EQ(slot->clearance, cfg.clearance_cap);
}

TEST(SortLogic, SecondBottleFitsBesideTheFirst) {
  const auto bin = binAt(0.35, -0.42);
  const double gap = kBottleShort + 0.006;
  const gm::ReleaseSlotConfig cfg;
  const auto b1 = bottle(1, 0.38, 0.10);
  const auto s1 = gm::chooseReleaseSlot(bin, b1, gm::kPi / 2.0, gap, kFingerT, kFingerW, {}, cfg);
  ASSERT_TRUE(s1.has_value());
  auto placed = b1;
  placed.center.head<2>() = s1->xy;
  const auto b2 = bottle(2, 0.38, -0.08);
  const auto s2 =
      gm::chooseReleaseSlot(bin, b2, -gm::kPi / 2.0, gap, kFingerT, kFingerW, {placed}, cfg);
  ASSERT_TRUE(s2.has_value());
  EXPECT_TRUE(slotInside(bin, b2, s2->xy, -gm::kPi / 2.0, gap, cfg.wall_margin));
  auto second = b2;
  second.center.head<2>() = s2->xy;
  EXPECT_GE(gm::footprintDistance(placed, second), 0.015);  // object gap
  EXPECT_GE(s2->clearance, 0.005);                          // fingers included
}

TEST(SortLogic, FullyOpenJawsLeaveNoRoomForASecondBottle) {
  // Why gripper.release_opening_margin exists: with the jaws opened to 0.09 m the outer finger
  // of the second bottle would touch the first bottle.
  const auto bin = binAt(0.35, -0.42);
  const gm::ReleaseSlotConfig cfg;
  const auto b1 = bottle(1, 0.38, 0.10);
  const auto s1 = gm::chooseReleaseSlot(bin, b1, gm::kPi / 2.0, 0.09, kFingerT, kFingerW, {}, cfg);
  ASSERT_TRUE(s1.has_value());
  auto placed = b1;
  placed.center.head<2>() = s1->xy;
  const auto s2 =
      gm::chooseReleaseSlot(bin, b1, gm::kPi / 2.0, 0.09, kFingerT, kFingerW, {placed}, cfg);
  ASSERT_TRUE(s2.has_value());
  EXPECT_LT(s2->clearance, 0.005);
}

TEST(SortLogic, TwoBallsFitInTheBallBin) {
  const auto bin = binAt(0.50, 0.42);
  const double gap = kBallDiameter + 0.006;
  const gm::ReleaseSlotConfig cfg;
  for (const double yaw : {0.0, gm::kPi / 4.0, gm::kPi / 2.0, -3.0 * gm::kPi / 4.0}) {
    const auto b1 = ball(1, 0.6, 0.12);
    const auto s1 = gm::chooseReleaseSlot(bin, b1, yaw, gap, kFingerT, kFingerW, {}, cfg);
    ASSERT_TRUE(s1.has_value());
    auto placed = b1;
    placed.center.head<2>() = s1->xy;
    const auto s2 = gm::chooseReleaseSlot(bin, b1, yaw, gap, kFingerT, kFingerW, {placed}, cfg);
    ASSERT_TRUE(s2.has_value());
    auto second = b1;
    second.center.head<2>() = s2->xy;
    EXPECT_GE(gm::footprintDistance(placed, second), 0.01) << "yaw " << yaw;
    EXPECT_GE(s2->clearance, 0.0) << "yaw " << yaw;
    // Ball stays inside the inner walls - margin.
    const Eigen::Vector3d l = bin.pose.inverse() * Eigen::Vector3d(s2->xy.x(), s2->xy.y(), 0.8);
    EXPECT_LE(std::abs(l.x()) + kBallDiameter / 2.0, bin.innerHalf().x() - cfg.wall_margin + 1e-9);
    EXPECT_LE(std::abs(l.y()) + kBallDiameter / 2.0, bin.innerHalf().y() - cfg.wall_margin + 1e-9);
  }
}

TEST(SortLogic, NoSlotForAnObjectWiderThanTheBin) {
  const auto bin = binAt(0.35, -0.42);
  auto big = bottle(1, 0, 0);
  big.size = {0.19, 0.05, 0.1};
  EXPECT_FALSE(
      gm::chooseReleaseSlot(bin, big, gm::kPi / 2.0, 0.06, kFingerT, kFingerW, {}, {}).has_value());
  gm::ReleaseSlotConfig bad;
  bad.grid_step = 0.0;
  EXPECT_THROW(gm::chooseReleaseSlot(bin, bottle(1, 0, 0), 0.0, 0.06, kFingerT, kFingerW, {}, bad),
               std::invalid_argument);
}

TEST(SortLogic, SlotRespectsARotatedBin) {
  const auto bin = binAt(0.35, -0.42, 0.5);
  const auto b = bottle(1, 0, 0, 0.5);
  const double gap = kBottleShort + 0.006;
  const gm::ReleaseSlotConfig cfg;
  const auto s =
      gm::chooseReleaseSlot(bin, b, 0.5 + gm::kPi / 2.0, gap, kFingerT, kFingerW, {}, cfg);
  ASSERT_TRUE(s.has_value());
  EXPECT_TRUE(slotInside(bin, b, s->xy, 0.5 + gm::kPi / 2.0, gap, cfg.wall_margin));
}

TEST(SortLogic, BoxSlotsStayOnTheClosingAxisLine) {
  // Perceived bottle length 0.0625 (D-14: back half hidden) vs the real 0.0958: with the default
  // box_max_across_offset 0 the slot is on the bin's centre line across the closing axis, so the
  // real bottle still fits; allowing an across offset pushes the real bottle into the wall.
  const auto bin = binAt(0.35, -0.42);
  auto seen = bottle(1, 0.38, -0.08, 0.058);
  seen.size.x() = 0.0625;
  auto real = seen;
  real.size.x() = kBottleLong;
  const double closing = 0.058 + gm::kPi / 2.0;
  const double gap = kBottleShort + 0.006;
  gm::ReleaseSlotConfig cfg;
  const auto s = gm::chooseReleaseSlot(bin, seen, closing, gap, kFingerT, kFingerW, {}, cfg);
  ASSERT_TRUE(s.has_value());
  const Eigen::Vector2d axis(std::cos(closing), std::sin(closing));
  const Eigen::Vector2d off = s->xy - bin.pose.translation().head<2>();
  EXPECT_NEAR(off.x() * -axis.y() + off.y() * axis.x(), 0.0, 1e-9);
  EXPECT_TRUE(slotInside(bin, real, s->xy, closing, gap, 0.0));
  cfg.box_max_across_offset = 1.0;
  const auto loose = gm::chooseReleaseSlot(bin, seen, closing, gap, kFingerT, kFingerW, {}, cfg);
  ASSERT_TRUE(loose.has_value());
  EXPECT_FALSE(slotInside(bin, real, loose->xy, closing, gap, 0.0));
}

TEST(SortLogic, TwoPerceivedBottlesWithTiltedClosingAxesFitSideBySide) {
  // Values from a live run: closing yaws 1.4877 and 1.6290 rad (not exactly pi/2), perceived
  // footprints 0.0748 x 0.0552 and 0.0625 x 0.0551; real bottles 0.0958 x 0.0582.
  const auto bin = binAt(0.35, -0.42);
  gm::ReleaseSlotConfig cfg;
  auto b1 = bottle(3, 0.39, 0.098, -0.0831);
  b1.size.head<2>() = Eigen::Vector2d(0.0748, 0.0552);
  auto b2 = bottle(6, 0.396, -0.079, 0.0582);
  b2.size.head<2>() = Eigen::Vector2d(0.0625, 0.0551);
  const double gap1 = 0.0552 + 0.006;
  const double gap2 = 0.0551 + 0.006;
  const auto s1 = gm::chooseReleaseSlot(bin, b1, 1.4877, gap1, kFingerT, kFingerW, {}, cfg);
  ASSERT_TRUE(s1.has_value());
  EXPECT_GT((s1->xy - bin.pose.translation().head<2>()).norm(), 0.03);  // not the centre
  auto placed = b1;
  placed.center.head<2>() = s1->xy;
  const auto s2 = gm::chooseReleaseSlot(bin, b2, 1.6290, gap2, kFingerT, kFingerW, {placed}, cfg);
  ASSERT_TRUE(s2.has_value());
  EXPECT_GE(s2->clearance, 0.005);
  // Real-size bottles at both slots: inside the walls and apart.
  auto r1 = placed;
  r1.size.head<2>() = Eigen::Vector2d(kBottleLong, kBottleShort);
  auto r2 = b2;
  r2.center.head<2>() = s2->xy;
  r2.size.head<2>() = Eigen::Vector2d(kBottleLong, kBottleShort);
  EXPECT_TRUE(slotInside(bin, r1, s1->xy, 1.4877, gap1, 0.0));
  EXPECT_TRUE(slotInside(bin, r2, s2->xy, 1.6290, gap2, 0.0));
  EXPECT_GT(gm::footprintDistance(r1, r2), 0.0);
}
