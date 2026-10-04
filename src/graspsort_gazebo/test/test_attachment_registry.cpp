#include <gtest/gtest.h>

#include <graspsort_gazebo/attachment_registry.hpp>
#include <string>

using graspsort::AttachmentRegistry;
using graspsort::LinkPair;
using graspsort::LookupError;
using graspsort::MakeJointName;
using graspsort::MakeKey;
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
  p.child_model.clear();
  EXPECT_EQ(ValidateRequest(p), "child_model is empty");
  p = Pair();
  p.child_link.clear();
  EXPECT_EQ(ValidateRequest(p), "child_link is empty");
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
