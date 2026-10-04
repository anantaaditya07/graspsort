// Unit tests for scene_logic.hpp (architecture 7.3): fixed primitives and the scene diffing.
#include <gtest/gtest.h>

#include <graspsort_scene/scene_logic.hpp>
#include <set>
#include <string>
#include <vector>

namespace gs = graspsort::scene;

namespace {

constexpr double kEps = 1e-12;

// Axis-aligned bounds of a box: {min x, max x, min y, max y, min z, max z}.
std::array<double, 6> bounds(const gs::Box& b) {
  return {b.center[0] - b.size[0] / 2, b.center[0] + b.size[0] / 2, b.center[1] - b.size[1] / 2,
          b.center[1] + b.size[1] / 2, b.center[2] - b.size[2] / 2, b.center[2] + b.size[2] / 2};
}

bool overlaps(const gs::Box& a, const gs::Box& b) {
  const auto p = bounds(a);
  const auto q = bounds(b);
  const double tol = 1e-9;
  for (int i = 0; i < 3; ++i) {
    if (p[2 * i + 1] <= q[2 * i] + tol || q[2 * i + 1] <= p[2 * i] + tol) return false;
  }
  return true;
}

gs::ObjectEstimate ball(std::uint32_t id, double x, double y) {
  gs::ObjectEstimate o;
  o.id = id;
  o.shape = gs::Shape::kCylinder;
  o.x = x;
  o.y = y;
  o.z = 0.7875;
  o.size = {0.075, 0.075, 0.075};
  return o;
}

gs::ObjectEstimate bottle(std::uint32_t id, double x, double y, double yaw) {
  gs::ObjectEstimate o;
  o.id = id;
  o.shape = gs::Shape::kBox;
  o.x = x;
  o.y = y;
  o.z = 0.85;
  o.yaw = yaw;
  o.size = {0.0958, 0.0582, 0.1913};
  return o;
}

gs::TrackerParams params() {
  gs::TrackerParams p;
  p.update_distance = 0.01;
  p.update_yaw = 0.1;
  p.remove_timeout = 2.0;
  return p;
}

std::set<std::string> ids(const std::vector<gs::ObjectEstimate>& v) {
  std::set<std::string> out;
  for (const auto& o : v) out.insert(gs::objectId(o.id));
  return out;
}

// Applies actions to a simulated world-id set, like the node does.
void applyActions(const gs::SceneActions& a, std::set<std::string>* world) {
  for (const auto& o : a.add) world->insert(gs::objectId(o.id));
  for (const auto& o : a.update) world->insert(gs::objectId(o.id));
  for (const auto& id : a.remove) world->erase(id);
}

}  // namespace

// ---- fixed geometry ----

TEST(BinPrimitives, MatchesWorldFile) {
  // worlds/graspsort.world: floor 0.20x0.20x0.01 at z 0.005; x walls 0.01x0.20x0.08 at
  // x +-0.095, z 0.04; y walls 0.18x0.01x0.08 at y +-0.095.
  const auto b = gs::binPrimitives({0.20, 0.20, 0.08}, 0.01, 0.01);
  ASSERT_EQ(b.size(), 5u);
  const std::vector<gs::Box> expected = {{{0, 0, 0.005}, {0.20, 0.20, 0.01}},
                                         {{0.095, 0, 0.04}, {0.01, 0.20, 0.08}},
                                         {{-0.095, 0, 0.04}, {0.01, 0.20, 0.08}},
                                         {{0, 0.095, 0.04}, {0.18, 0.01, 0.08}},
                                         {{0, -0.095, 0.04}, {0.18, 0.01, 0.08}}};
  for (std::size_t i = 0; i < b.size(); ++i) {
    for (int k = 0; k < 3; ++k) {
      EXPECT_NEAR(b[i].center[k], expected[i].center[k], kEps) << i << " " << k;
      EXPECT_NEAR(b[i].size[k], expected[i].size[k], kEps) << i << " " << k;
    }
  }
}

TEST(BinPrimitives, StaysInsideOuterBoxAndLeavesInnerCavity) {
  const std::array<double, 3> size{0.30, 0.22, 0.10};
  const double wall = 0.012;
  const double floor = 0.015;
  const auto b = gs::binPrimitives(size, wall, floor);
  for (const auto& box : b) {
    const auto q = bounds(box);
    EXPECT_GE(q[0], -size[0] / 2 - kEps);
    EXPECT_LE(q[1], size[0] / 2 + kEps);
    EXPECT_GE(q[2], -size[1] / 2 - kEps);
    EXPECT_LE(q[3], size[1] / 2 + kEps);
    EXPECT_GE(q[4], -kEps);
    EXPECT_LE(q[5], size[2] + kEps);
  }
  // The inner cavity (above the floor, inside the walls) is free.
  const gs::Box cavity{{0, 0, (floor + size[2]) / 2},
                       {size[0] - 2 * wall, size[1] - 2 * wall, size[2] - floor}};
  for (const auto& box : b) EXPECT_FALSE(overlaps(box, cavity));
  // Walls do not overlap each other (the y walls fit between the x walls).
  for (std::size_t i = 1; i < b.size(); ++i) {
    for (std::size_t j = i + 1; j < b.size(); ++j) EXPECT_FALSE(overlaps(b[i], b[j]));
  }
}

TEST(BinPrimitives, CheckRejectsBadValues) {
  EXPECT_FALSE(gs::checkBin({0.2, 0.2, 0.08}, 0.01, 0.01).has_value());
  EXPECT_TRUE(gs::checkBin({0.2, 0.0, 0.08}, 0.01, 0.01).has_value());
  EXPECT_TRUE(gs::checkBin({0.2, 0.2, 0.08}, 0.0, 0.01).has_value());
  EXPECT_TRUE(gs::checkBin({0.2, 0.2, 0.08}, 0.1, 0.01).has_value());
  EXPECT_TRUE(gs::checkBin({0.2, 0.2, 0.08}, 0.01, 0.08).has_value());
}

TEST(FixedPrimitives, TableTopAndPedestalHangBelowTheirFrame) {
  const auto t = gs::tableTopPrimitive({0.70, 1.10, 0.05});
  EXPECT_NEAR(t.center[2], -0.025, kEps);
  EXPECT_NEAR(t.size[0], 0.70, kEps);
  const auto p = gs::pedestalPrimitive({0.20, 0.20, 0.75});
  EXPECT_NEAR(p.center[2], -0.375, kEps);  // pedestal reaches from z 0 to its top (0.75)
  EXPECT_NEAR(p.size[2], 0.75, kEps);
}

TEST(Geometry, YawQuaternion) {
  const auto q = gs::yawToQuaternion(gs::kPi / 2);
  EXPECT_NEAR(q[2], std::sqrt(0.5), kEps);
  EXPECT_NEAR(q[3], std::sqrt(0.5), kEps);
  EXPECT_NEAR(q[0], 0.0, kEps);
}

TEST(Geometry, ObjectDimensions) {
  const auto c = gs::objectDimensions(ball(1, 0, 0), 0.0);
  EXPECT_NEAR(c[0], 0.075, kEps);   // height
  EXPECT_NEAR(c[1], 0.0375, kEps);  // radius = size.x / 2
  const auto cp = gs::objectDimensions(ball(1, 0, 0), 0.005);
  EXPECT_NEAR(cp[0], 0.085, kEps);
  EXPECT_NEAR(cp[1], 0.0425, kEps);
  const auto b = gs::objectDimensions(bottle(2, 0, 0, 0), 0.005);
  EXPECT_NEAR(b[0], 0.1058, kEps);
  EXPECT_NEAR(b[1], 0.0682, kEps);
  EXPECT_NEAR(b[2], 0.2013, kEps);
}

TEST(Geometry, ObjectIds) {
  EXPECT_EQ(gs::objectId(7), "object_7");
  EXPECT_TRUE(gs::isObjectId("object_12"));
  EXPECT_FALSE(gs::isObjectId("object_"));
  EXPECT_FALSE(gs::isObjectId("object_1a"));
  EXPECT_FALSE(gs::isObjectId("bin_ball"));
  EXPECT_FALSE(gs::isObjectId("table"));
}

TEST(Geometry, BoxYawDifferenceIsModuloPi) {
  EXPECT_NEAR(gs::boxYawDifference(0.0, gs::kPi), 0.0, 1e-9);
  EXPECT_NEAR(gs::boxYawDifference(0.05, gs::kPi - 0.05), 0.1, 1e-9);
  EXPECT_NEAR(gs::boxYawDifference(-1.5, 1.5), gs::kPi - 3.0, 1e-9);
  EXPECT_NEAR(gs::boxYawDifference(0.0, gs::kPi / 2), gs::kPi / 2, 1e-9);
}

TEST(Params, Check) {
  EXPECT_FALSE(gs::checkParams(params()).has_value());
  auto p = params();
  p.remove_timeout = 0.0;
  EXPECT_TRUE(gs::checkParams(p).has_value());
  p = params();
  p.update_distance = -1.0;
  EXPECT_TRUE(gs::checkParams(p).has_value());
}

// ---- tracker ----

TEST(SceneTracker, AddsNewObjectsOnce) {
  gs::SceneTracker t(params());
  std::set<std::string> world{"table", "bin_ball"};
  const std::vector<gs::ObjectEstimate> seen{ball(1, 0.6, -0.12), bottle(3, 0.38, 0.1, 0.0)};
  auto a = t.step(&seen, 0.0, world, {});
  EXPECT_EQ(ids(a.add), (std::set<std::string>{"object_1", "object_3"}));
  EXPECT_TRUE(a.update.empty());
  EXPECT_TRUE(a.remove.empty());
  applyActions(a, &world);
  a = t.step(&seen, 0.1, world, {});
  EXPECT_TRUE(a.empty());
}

TEST(SceneTracker, UpdatesOnlyBeyondThresholds) {
  gs::SceneTracker t(params());
  std::set<std::string> world;
  std::vector<gs::ObjectEstimate> seen{ball(1, 0.6, 0.0), bottle(2, 0.4, 0.0, 0.0)};
  applyActions(t.step(&seen, 0.0, world, {}), &world);

  seen = {ball(1, 0.605, 0.0), bottle(2, 0.4, 0.005, 0.05)};  // below both thresholds
  EXPECT_TRUE(t.step(&seen, 0.1, world, {}).empty());

  seen = {ball(1, 0.62, 0.0), bottle(2, 0.4, 0.0, 0.0)};  // ball moved 2 cm
  auto a = t.step(&seen, 0.2, world, {});
  EXPECT_EQ(ids(a.update), (std::set<std::string>{"object_1"}));
  EXPECT_TRUE(a.add.empty());

  // Distance is measured from the last APPLIED pose: two 6 mm steps trigger on the second.
  seen = {ball(1, 0.626, 0.0), bottle(2, 0.4, 0.0, 0.0)};
  EXPECT_TRUE(t.step(&seen, 0.3, world, {}).empty());
  seen = {ball(1, 0.632, 0.0), bottle(2, 0.4, 0.0, 0.0)};
  EXPECT_EQ(ids(t.step(&seen, 0.4, world, {}).update), (std::set<std::string>{"object_1"}));

  // Box yaw change > update_yaw triggers; the same yaw + pi does not.
  seen = {ball(1, 0.632, 0.0), bottle(2, 0.4, 0.0, gs::kPi)};
  EXPECT_TRUE(t.step(&seen, 0.5, world, {}).empty());
  seen = {ball(1, 0.632, 0.0), bottle(2, 0.4, 0.0, 0.3)};
  EXPECT_EQ(ids(t.step(&seen, 0.6, world, {}).update), (std::set<std::string>{"object_2"}));
}

TEST(SceneTracker, CylinderYawIsIgnoredButSizeCounts) {
  gs::SceneTracker t(params());
  std::set<std::string> world;
  std::vector<gs::ObjectEstimate> seen{ball(1, 0.6, 0.0)};
  applyActions(t.step(&seen, 0.0, world, {}), &world);
  seen[0].yaw = 1.0;
  EXPECT_TRUE(t.step(&seen, 0.1, world, {}).empty());
  seen[0].size[2] = 0.09;  // 15 mm taller
  EXPECT_EQ(t.step(&seen, 0.2, world, {}).update.size(), 1u);
  seen[0].shape = gs::Shape::kBox;
  EXPECT_EQ(t.step(&seen, 0.3, world, {}).update.size(), 1u);
}

TEST(SceneTracker, RemovesAfterTimeoutOnly) {
  gs::SceneTracker t(params());
  std::set<std::string> world;
  const std::vector<gs::ObjectEstimate> both{ball(1, 0.6, 0.0), ball(2, 0.6, 0.12)};
  const std::vector<gs::ObjectEstimate> one{ball(1, 0.6, 0.0)};
  applyActions(t.step(&both, 0.0, world, {}), &world);
  EXPECT_TRUE(t.step(&one, 1.0, world, {}).empty());
  EXPECT_TRUE(t.step(&one, 2.0, world, {}).empty());  // exactly at the timeout: kept
  auto a = t.step(&one, 2.1, world, {});
  EXPECT_EQ(a.remove, (std::vector<std::string>{"object_2"}));
  applyActions(a, &world);
  EXPECT_EQ(world.count("object_2"), 0u);
  // Comes back: re-added.
  a = t.step(&both, 3.0, world, {});
  EXPECT_EQ(ids(a.add), (std::set<std::string>{"object_2"}));
}

TEST(SceneTracker, TimeoutsRunWithoutNewMessages) {
  gs::SceneTracker t(params());
  std::set<std::string> world;
  const std::vector<gs::ObjectEstimate> seen{ball(1, 0.6, 0.0)};
  applyActions(t.step(&seen, 0.0, world, {}), &world);
  EXPECT_TRUE(t.step(nullptr, 1.5, world, {}).empty());
  EXPECT_EQ(t.step(nullptr, 2.5, world, {}).remove.size(), 1u);
}

TEST(SceneTracker, FreezeBlocksEverythingAndRestartsTimeouts) {
  gs::SceneTracker t(params());
  std::set<std::string> world;
  std::vector<gs::ObjectEstimate> seen{ball(1, 0.6, 0.0), ball(2, 0.6, 0.12)};
  applyActions(t.step(&seen, 0.0, world, {}), &world);

  t.setFrozen(true, 1.0);
  EXPECT_TRUE(t.frozen());
  const std::vector<gs::ObjectEstimate> moved{ball(1, 0.7, 0.0), ball(3, 0.4, 0.0)};
  EXPECT_TRUE(t.step(&moved, 1.5, world, {}).empty());    // no add/update
  EXPECT_TRUE(t.step(nullptr, 10.0, world, {}).empty());  // no remove
  t.setFrozen(false, 10.0);
  EXPECT_FALSE(t.frozen());
  // Unfreeze restarted the timeouts: nothing removed right away.
  EXPECT_TRUE(t.step(nullptr, 11.0, world, {}).empty());
  // After unfreeze the moved/new objects are applied.
  const auto a = t.step(&moved, 11.1, world, {});
  EXPECT_EQ(ids(a.update), (std::set<std::string>{"object_1"}));
  EXPECT_EQ(ids(a.add), (std::set<std::string>{"object_3"}));
  // object_2 expires remove_timeout after the unfreeze (10.0 + 2.0), not before.
  EXPECT_TRUE(t.step(&moved, 11.9, world, {}).remove.empty());
  EXPECT_EQ(t.step(&moved, 12.1, world, {}).remove, (std::vector<std::string>{"object_2"}));
}

TEST(SceneTracker, UnfreezeWithoutFreezeDoesNotTouchTimeouts) {
  gs::SceneTracker t(params());
  std::set<std::string> world;
  const std::vector<gs::ObjectEstimate> seen{ball(1, 0.6, 0.0)};
  applyActions(t.step(&seen, 0.0, world, {}), &world);
  t.setFrozen(false, 5.0);  // already unfrozen: no-op
  EXPECT_EQ(t.step(nullptr, 5.0, world, {}).remove.size(), 1u);
}

TEST(SceneTracker, NeverTouchesAttachedObjects) {
  gs::SceneTracker t(params());
  std::set<std::string> world;
  std::vector<gs::ObjectEstimate> seen{ball(1, 0.6, 0.0), ball(2, 0.6, 0.12)};
  applyActions(t.step(&seen, 0.0, world, {}), &world);

  // object_1 attached: MoveIt moved it out of the world into the robot state.
  world.erase("object_1");
  const std::set<std::string> attached{"object_1"};
  // Seen (moving with the gripper): no add, no update.
  seen = {ball(1, 0.5, 0.0), ball(2, 0.6, 0.12)};
  auto a = t.step(&seen, 0.5, world, attached);
  EXPECT_TRUE(a.empty());
  // Not seen for a long time: no remove.
  const std::vector<gs::ObjectEstimate> only2{ball(2, 0.6, 0.12)};
  for (double now = 1.0; now < 20.0; now += 1.0) {
    a = t.step(&only2, now, world, attached);
    EXPECT_TRUE(a.empty()) << now;
  }
  // Detached and removed by the pick code; perceived again in the bin: added fresh.
  seen = {ball(1, 0.5, 0.42), ball(2, 0.6, 0.12)};
  a = t.step(&seen, 21.0, world, {});
  EXPECT_EQ(ids(a.add), (std::set<std::string>{"object_1"}));
}

TEST(SceneTracker, AttachedObjectStillInWorldIsNotRemoved) {
  // Even if an attached id also appears in the world list it is never removed or updated.
  gs::SceneTracker t(params());
  std::set<std::string> world;
  std::vector<gs::ObjectEstimate> seen{ball(1, 0.6, 0.0)};
  applyActions(t.step(&seen, 0.0, world, {}), &world);
  const std::set<std::string> attached{"object_1"};
  EXPECT_TRUE(t.step(nullptr, 30.0, world, attached).empty());
  seen = {ball(1, 0.3, 0.0)};
  EXPECT_TRUE(t.step(&seen, 31.0, world, attached).empty());
}

TEST(SceneTracker, ReAddsObjectsRemovedBySomeoneElse) {
  gs::SceneTracker t(params());
  std::set<std::string> world;
  const std::vector<gs::ObjectEstimate> seen{ball(1, 0.6, 0.0)};
  applyActions(t.step(&seen, 0.0, world, {}), &world);
  world.erase("object_1");  // e.g. pick code removed it, or a failed apply
  EXPECT_EQ(ids(t.step(&seen, 0.2, world, {}).add), (std::set<std::string>{"object_1"}));
  // Removed externally and not perceived: forgotten, no remove action for a missing id.
  gs::SceneTracker t2(params());
  std::set<std::string> w2;
  applyActions(t2.step(&seen, 0.0, w2, {}), &w2);
  w2.clear();
  EXPECT_TRUE(t2.step(nullptr, 0.5, w2, {}).empty());
  EXPECT_TRUE(t2.trackedIds().empty());
}

TEST(SceneTracker, AdoptsUnknownObjectIdsAndExpiresThem) {
  gs::SceneTracker t(params());
  const std::set<std::string> world{"table", "bin_ball", "object_9", "my_box"};
  EXPECT_TRUE(t.step(nullptr, 0.0, world, {}).empty());
  EXPECT_EQ(t.trackedIds(), (std::vector<std::string>{"object_9"}));
  auto a = t.step(nullptr, 2.5, world, {});
  EXPECT_EQ(a.remove, (std::vector<std::string>{"object_9"}));  // fixed/foreign ids untouched
  // An adopted object that is perceived is re-applied (its geometry is unknown).
  gs::SceneTracker t2(params());
  const std::vector<gs::ObjectEstimate> seen{ball(9, 0.6, 0.0)};
  a = t2.step(&seen, 0.0, world, {});
  EXPECT_TRUE(a.add.empty());
  EXPECT_EQ(ids(a.update), (std::set<std::string>{"object_9"}));
}

TEST(SceneTracker, DuplicateIdsInOneMessageUseTheFirst) {
  gs::SceneTracker t(params());
  const std::vector<gs::ObjectEstimate> seen{ball(1, 0.6, 0.0), ball(1, 0.2, 0.0)};
  const auto a = t.step(&seen, 0.0, {}, {});
  ASSERT_EQ(a.add.size(), 1u);
  EXPECT_NEAR(a.add[0].x, 0.6, kEps);
}
