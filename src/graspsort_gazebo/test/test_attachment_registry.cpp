#include <gtest/gtest.h>

#include <graspsort_gazebo/attachment_registry.hpp>
#include <string>
#include <vector>

using graspsort::Aabb;
using graspsort::AabbDistance;
using graspsort::AttachCandidate;
using graspsort::AttachmentRegistry;
using graspsort::IsNearestRequest;
using graspsort::LinkPair;
using graspsort::LookupError;
using graspsort::MakeJointName;
using graspsort::MakeKey;
using graspsort::SelectNearest;
using graspsort::ValidateRequest;

namespace {
LinkPair Pair() { return LinkPair{"ur", "gripper_left_finger_link", "ball_1", "link"}; }
}  // namespace

TEST(AttachmentKey, BuildsScopedKey) {
  EXPECT_EQ(MakeKey(Pair()), "ur::gripper_left_finger_link__ball_1::link");
}

TEST(AttachmentKey, DifferentChildGivesDifferentKey) {
  LinkPair other = Pair();
  other.child_model = "ball_2";
  EXPECT_NE(MakeKey(Pair()), MakeKey(other));
}

TEST(AttachmentKey, JointNameIsPrefixPlusKey) {
  EXPECT_EQ(MakeJointName("p__", "a::b__c::d"), "p__a::b__c::d");
}

TEST(ValidateRequest, AcceptsValidRequest) { EXPECT_EQ(ValidateRequest(Pair()), ""); }

TEST(ValidateRequest, RejectsEmptyFields) {
  LinkPair p = Pair();
  p.parent_model.clear();
  EXPECT_EQ(ValidateRequest(p), "parent_model is empty");
  p = Pair();
  p.parent_link.clear();
  EXPECT_EQ(ValidateRequest(p), "parent_link is empty");
  p = Pair();
  p.child_link.clear();
  EXPECT_EQ(ValidateRequest(p), "child_link is empty");
}

TEST(ValidateRequest, EmptyChildModelIsNearestModeAndIgnoresChildLink) {
  LinkPair p = Pair();
  p.child_model.clear();
  p.child_link.clear();
  EXPECT_TRUE(IsNearestRequest(p));
  EXPECT_EQ(ValidateRequest(p), "");
  EXPECT_FALSE(IsNearestRequest(Pair()));
}

TEST(AabbDistance, ZeroWhenTouchingOrOverlapping) {
  const Aabb a{{0.0, 0.0, 0.0}, {1.0, 1.0, 1.0}};
  EXPECT_DOUBLE_EQ(AabbDistance(a, Aabb{{1.0, 0.0, 0.0}, {2.0, 1.0, 1.0}}), 0.0);
  EXPECT_DOUBLE_EQ(AabbDistance(a, Aabb{{0.5, 0.5, 0.5}, {2.0, 2.0, 2.0}}), 0.0);
}

TEST(AabbDistance, EuclideanGapAcrossAxes) {
  const Aabb a{{0.0, 0.0, 0.0}, {1.0, 1.0, 1.0}};
  EXPECT_NEAR(AabbDistance(a, Aabb{{1.03, 0.0, 0.0}, {2.0, 1.0, 1.0}}), 0.03, 1e-12);
  EXPECT_NEAR(AabbDistance(a, Aabb{{1.03, 1.04, 0.0}, {2.0, 2.0, 1.0}}), 0.05, 1e-12);
  EXPECT_NEAR(AabbDistance(Aabb{{1.03, 0.0, 0.0}, {2.0, 1.0, 1.0}}, a), 0.03, 1e-12);  // symmetric
}

TEST(SelectNearest, PicksClosestDynamicModelOfAnotherModel) {
  const std::vector<AttachCandidate> c{
      {"ur", "wrist_3_link", 0.0, false},  // the parent model itself: skipped
      {"table", "link", 0.0, true},        // static: skipped
      {"ball_2", "link", 0.015, false},
      {"ball_1", "link", 0.001, false},
  };
  const auto best = SelectNearest(c, "ur", 0.02);
  ASSERT_TRUE(best.has_value());
  EXPECT_EQ(best->model, "ball_1");
}

TEST(SelectNearest, NothingWithinMaxDistance) {
  const std::vector<AttachCandidate> c{{"ball_1", "link", 0.05, false}};
  EXPECT_FALSE(SelectNearest(c, "ur", 0.02).has_value());
  EXPECT_TRUE(SelectNearest(c, "ur", 0.05).has_value());  // boundary is inclusive
}

TEST(SelectNearest, TiesBrokenByModelName) {
  const std::vector<AttachCandidate> c{{"bottle_2", "link", 0.0, false},
                                       {"bottle_1", "link", 0.0, false}};
  EXPECT_EQ(SelectNearest(c, "ur", 0.02)->model, "bottle_1");
}

TEST(AttachmentRegistry, KeysForParentListsOnlyThatParentLink) {
  AttachmentRegistry<int> r;
  r.Add(MakeKey(Pair()), 1);
  r.Add(MakeKey(LinkPair{"ur", "gripper_left_finger_link", "ball_2", "link"}), 2);
  r.Add(MakeKey(LinkPair{"ur", "gripper_right_finger_link", "ball_3", "link"}), 3);
  const auto keys = r.KeysForParent("ur", "gripper_left_finger_link");
  ASSERT_EQ(keys.size(), 2U);
  EXPECT_EQ(keys[0], "ur::gripper_left_finger_link__ball_1::link");
  EXPECT_EQ(keys[1], "ur::gripper_left_finger_link__ball_2::link");
  EXPECT_TRUE(r.KeysForParent("ur", "tool0").empty());
}

TEST(ValidateRequest, RejectsSameLink) {
  const LinkPair p{"ur", "wrist_3_link", "ur", "wrist_3_link"};
  EXPECT_EQ(ValidateRequest(p), "parent and child are the same link");
}

TEST(ValidateRequest, AllowsTwoLinksOfSameModel) {
  const LinkPair p{"ur", "wrist_3_link", "ur", "tool0"};
  EXPECT_EQ(ValidateRequest(p), "");
}

TEST(LookupError, ReportsUnknownModelFirst) {
  EXPECT_EQ(LookupError("ghost", "link", false, false), "unknown model 'ghost'");
}

TEST(LookupError, ReportsUnknownLink) {
  EXPECT_EQ(LookupError("ur", "no_link", true, false), "unknown link 'no_link' in model 'ur'");
}

TEST(LookupError, EmptyWhenFound) { EXPECT_EQ(LookupError("ur", "tool0", true, true), ""); }

TEST(AttachmentRegistry, AddFindRemove) {
  AttachmentRegistry<int> reg;
  const std::string key = MakeKey(Pair());
  EXPECT_EQ(reg.Size(), 0u);
  EXPECT_EQ(reg.CheckAttach(key), "");
  EXPECT_EQ(reg.CheckDetach(key), "not attached: " + key);
  ASSERT_TRUE(reg.Add(key, 7));
  EXPECT_TRUE(reg.Contains(key));
  ASSERT_NE(reg.Find(key), nullptr);
  EXPECT_EQ(*reg.Find(key), 7);
  EXPECT_EQ(reg.CheckDetach(key), "");
  const auto removed = reg.Remove(key);
  ASSERT_TRUE(removed.has_value());
  EXPECT_EQ(*removed, 7);
  EXPECT_EQ(reg.Size(), 0u);
  EXPECT_EQ(reg.Find(key), nullptr);
}

TEST(AttachmentRegistry, RejectsDuplicateAttach) {
  AttachmentRegistry<int> reg;
  const std::string key = MakeKey(Pair());
  ASSERT_TRUE(reg.Add(key, 1));
  EXPECT_EQ(reg.CheckAttach(key), "already attached: " + key);
  EXPECT_FALSE(reg.Add(key, 2));
  EXPECT_EQ(*reg.Find(key), 1);
  EXPECT_EQ(reg.Size(), 1u);
}

TEST(AttachmentRegistry, RemoveUnknownReturnsNullopt) {
  AttachmentRegistry<int> reg;
  EXPECT_FALSE(reg.Remove("nope").has_value());
}

TEST(AttachmentRegistry, SeveralAttachmentsAtOnce) {
  AttachmentRegistry<int> reg;
  LinkPair a = Pair();
  LinkPair b = Pair();
  b.child_model = "cup_1";
  ASSERT_TRUE(reg.Add(MakeKey(a), 1));
  ASSERT_TRUE(reg.Add(MakeKey(b), 2));
  EXPECT_EQ(reg.Size(), 2u);
  ASSERT_TRUE(reg.Remove(MakeKey(a)).has_value());
  EXPECT_TRUE(reg.Contains(MakeKey(b)));
  EXPECT_FALSE(reg.Contains(MakeKey(a)));
}

TEST(AttachmentRegistry, ReattachAfterDetach) {
  AttachmentRegistry<int> reg;
  const std::string key = MakeKey(Pair());
  ASSERT_TRUE(reg.Add(key, 1));
  ASSERT_TRUE(reg.Remove(key).has_value());
  EXPECT_EQ(reg.CheckAttach(key), "");
  EXPECT_TRUE(reg.Add(key, 2));
}
