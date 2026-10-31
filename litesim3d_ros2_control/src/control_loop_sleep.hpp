// Copyright 2026 LiteSim3D contributors
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <stdexcept>
#include <rclcpp/rclcpp.hpp>

namespace litesim3d_ros2_control {

inline bool sleep_until_or_shutdown(
    rclcpp::Clock &clock, const rclcpp::Time &until,
    const rclcpp::Context::SharedPtr &context) {
  try {
    return clock.sleep_until(until, context);
  } catch (const std::runtime_error &) {
    // Humble throws if shutdown invalidates the context between the loop's
    // rclcpp::ok() check and entry to sleep_until(). A second pre-check alone
    // cannot close that race. Keep errors on a live context visible.
    if (context && !context->is_valid()) return false;
    throw;
  }
}

}  // namespace litesim3d_ros2_control
