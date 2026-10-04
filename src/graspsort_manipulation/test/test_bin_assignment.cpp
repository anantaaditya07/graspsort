// GoogleTests for graspsort_manipulation/bin_assignment.hpp: class -> bin mapping from config.
#include <gtest/gtest.h>

#include <graspsort_manipulation/bin_assignment.hpp>
#include <stdexcept>
#include <string>
#include <vector>

namespace gm = graspsort::manipulation;

TEST(BinAssignment, DefaultsMapTheThreeD04Classes) {
  const auto b = gm::BinAssignment::defaults();
  EXPECT_EQ(b.binFor("sports ball").value_or(""), "bin_ball");
  EXPECT_EQ(b.binFor("cup").value_or(""), "bin_cup");
  EXPECT_EQ(b.binFor("bottle").value_or(""), "bin_bottle");
  EXPECT_EQ(b.classes(), (std::vector<std::string>{"bottle", "cup", "sports ball"}));
}

TEST(BinAssignment, UnknownClassHasNoBinByDefault) {
  const auto b = gm::BinAssignment::defaults();
  EXPECT_FALSE(b.binFor("can").has_value());
  EXPECT_FALSE(b.binFor("").has_value());
  EXPECT_FALSE(b.binFor("Cup").has_value());  // exact, case-sensitive match
  EXPECT_FALSE(b.knows("can"));
  EXPECT_TRUE(b.knows("cup"));
}

TEST(BinAssignment, ConfiguredDefaultBinCatchesUnknownClasses) {
  const auto b = gm::BinAssignment::fromLists({"cup"}, {"bin_cup"}, "bin_misc");
  EXPECT_EQ(b.binFor("cup").value_or(""), "bin_cup");
  EXPECT_EQ(b.binFor("can").value_or(""), "bin_misc");
  EXPECT_FALSE(b.knows("can"));
}

TEST(BinAssignment, ParallelListsFromParameters) {
  const auto b = gm::BinAssignment::fromLists({"cup", "bottle", "sports ball"},
                                              {"bin_bottle", "bin_bottle", "bin_ball"});
  // Several classes may share one bin.
  EXPECT_EQ(b.binFor("cup").value_or(""), "bin_bottle");
  EXPECT_EQ(b.binFor("bottle").value_or(""), "bin_bottle");
  EXPECT_EQ(b.classes().size(), 3U);
}

TEST(BinAssignment, InvalidConfigThrows) {
  EXPECT_THROW(gm::BinAssignment::fromLists({"cup", "bottle"}, {"bin_cup"}), std::invalid_argument);
  EXPECT_THROW(gm::BinAssignment::fromLists({"cup", "cup"}, {"bin_cup", "bin_ball"}),
               std::invalid_argument);
  EXPECT_THROW(gm::BinAssignment::fromLists({""}, {"bin_cup"}), std::invalid_argument);
  EXPECT_THROW(gm::BinAssignment::fromLists({"cup"}, {""}), std::invalid_argument);
}

TEST(BinAssignment, EmptyConfigAssignsNothing) {
  const auto b = gm::BinAssignment::fromLists({}, {});
  EXPECT_FALSE(b.binFor("cup").has_value());
  EXPECT_TRUE(b.classes().empty());
}
