#include <gtest/gtest.h>
#include "readiness.hpp"
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
