#include <gtest/gtest.h>
#include "readiness.hpp"
#include <vector>
using namespace litesim3d::control;

TEST(Readiness, TimeoutMustBeFinitePositiveAndBounded) {
  for (const auto *value : {".001", "1", "3600"}) EXPECT_GT(startup_timeout(value), 0);
  for (const auto *value : {"0", "-1", "3600.1", "nan", "inf", "-inf", "1junk", "bad"}) EXPECT_ANY_THROW(startup_timeout(value));
}
TEST(Readiness, ClockCannotBypassReadinessAndRewindsResetProgress) {
  Readiness state; state.clock(100); EXPECT_FALSE(state.ready());
  state.status({{"ready", "false"}}); state.clock(101); EXPECT_FALSE(state.ready());
  state.status({{"ready", "true"}}); state.clock(10); EXPECT_FALSE(state.ready());
  state.clock(10); EXPECT_FALSE(state.ready()); state.clock(5); EXPECT_FALSE(state.ready());
  state.clock(6); EXPECT_TRUE(state.ready());
  state.clock(100); state.clock(80); EXPECT_FALSE(state.ready());
  state.clock(81); EXPECT_TRUE(state.ready());
  state.status({{"ready", "false"}}); EXPECT_FALSE(state.ready());
  state.status({{"ready", "true"}}); state.clock(200); EXPECT_FALSE(state.ready()); state.clock(201); EXPECT_TRUE(state.ready());
}
TEST(Readiness, NewEpochAndFaultDiscardReadyState) {
  Readiness state; state.status({{"ready", "true"}, {"run_id", "first"}});
  state.clock(1); state.clock(2); EXPECT_TRUE(state.ready());
  state.status({{"ready", "true"}, {"run_id", "second"}}); EXPECT_FALSE(state.ready());
  state.clock(3); EXPECT_FALSE(state.ready()); state.clock(4); EXPECT_TRUE(state.ready());
  state.status({{"ready", "true"}, {"run_id", "second"}, {"state", "fault"}});
  EXPECT_TRUE(state.fault()); EXPECT_FALSE(state.ready());
}

TEST(Readiness, TimeoutParsingRejectsOverflowAndTrailingCharacters) {
  EXPECT_DOUBLE_EQ(startup_timeout("1e-3"), .001);
  EXPECT_DOUBLE_EQ(startup_timeout("+3600"), 3600);
  EXPECT_DOUBLE_EQ(startup_timeout("\t2"), 2);
  for (const auto *value : {"", " ", "2 ", "2s", "1e9999", "-0", "NaN", "INF"}) {
    SCOPED_TRACE(value);
    EXPECT_ANY_THROW(startup_timeout(value));
  }
}

TEST(Readiness, MissingOrNonLiteralReadyStatusCannotEnableClockProgress) {
  for (const auto &values : std::vector<std::map<std::string, std::string>>{
         {}, {{"ready", "True"}}, {{"ready", "1"}}, {{"ready", "true "}}, {{"state", "ready"}}}) {
    Readiness state;
    const auto before = values;
    state.status(values);
    state.clock(-2);
    state.clock(-1);
    EXPECT_FALSE(state.ready());
    EXPECT_FALSE(state.fault());
    EXPECT_EQ(values, before);
  }
}

TEST(Readiness, RepeatedSameEpochStatusPreservesPreviouslyObservedProgress) {
  Readiness state;
  const std::map<std::string, std::string> status{{"ready", "true"}, {"run_id", "epoch"}};
  state.status(status);
  state.clock(-2);
  state.clock(-1);
  ASSERT_TRUE(state.ready());
  state.status(status);
  EXPECT_TRUE(state.ready());
  state.clock(-1);
  EXPECT_TRUE(state.ready());
  state.clock(-3);
  EXPECT_FALSE(state.ready());
  state.clock(-2);
  EXPECT_TRUE(state.ready());
}

TEST(Readiness, LostStatusAndEpochChangeRequireFreshClockSamples) {
  Readiness state;
  state.status({{"ready", "true"}, {"run_id", "first"}});
  state.clock(10);
  state.clock(11);
  ASSERT_TRUE(state.ready());
  state.status({});
  state.clock(1000);
  EXPECT_FALSE(state.ready());
  state.status({{"ready", "true"}, {"run_id", "second"}});
  state.clock(1001);
  EXPECT_FALSE(state.ready());
  state.clock(1002);
  EXPECT_TRUE(state.ready());
}

TEST(Readiness, FaultFlagIsReportedIndependentlyOfReadyAndClock) {
  Readiness state;
  state.status({{"state", "fault"}, {"ready", "false"}, {"run_id", "failed"}});
  EXPECT_TRUE(state.fault());
  state.clock(1);
  state.clock(2);
  EXPECT_FALSE(state.ready());
  state.status({{"state", "running"}, {"ready", "true"}, {"run_id", "recovered"}});
  EXPECT_FALSE(state.fault());
  EXPECT_FALSE(state.ready());
  state.clock(0);
  state.clock(1);
  EXPECT_TRUE(state.ready());
}
