// Copyright 2026 LiteSim3D contributors
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include "control_loop_sleep.hpp"

using litesim3d_ros2_control::sleep_until_or_shutdown;

TEST(ControlLoopSleep, ShutdownBetweenUpdateAndSleep) {
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  rclcpp::Clock clock(RCL_ROS_TIME);
  const auto deadline = clock.now() + rclcpp::Duration::from_seconds(0.01);
  ASSERT_TRUE(context->is_valid());  // The control loop has entered its last update.
  context->shutdown("SIGINT during update");
  // Reproduce the exact failure in the unguarded Humble control loop.
  EXPECT_THROW(clock.sleep_until(deadline, context), std::runtime_error);
  EXPECT_FALSE(sleep_until_or_shutdown(clock, deadline, context));
}

TEST(ControlLoopSleep, LiveContextErrorsAreNotSuppressed) {
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  rclcpp::Clock clock(RCL_ROS_TIME);
  EXPECT_THROW(sleep_until_or_shutdown(clock, rclcpp::Time(1, 0, RCL_STEADY_TIME), context),
               std::runtime_error);
  context->shutdown("test complete");
}

TEST(ControlLoopSleep, LiveContextCanReachDeadline) {
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  rclcpp::Clock clock(RCL_ROS_TIME);
  EXPECT_TRUE(sleep_until_or_shutdown(clock, clock.now(), context));
  context->shutdown("test complete");
}
