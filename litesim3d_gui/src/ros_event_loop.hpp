#pragma once

#include <rclcpp/context.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <exception>
#include <utility>

namespace litesim3d_render {

inline rclcpp::ExecutorOptions ros_executor_options(
    const rclcpp::Context::SharedPtr &context) {
  rclcpp::ExecutorOptions options;
  options.context = context;
  return options;
}

// Construct one executor while starting the GUI and reuse it for every tick.
// Free rclcpp::spin_some(node) constructs a new executor, whose constructor can
// fail if ROS shuts down between the check and the call.
// A ROS signal handler can still invalidate the context before spinning an
// existing executor. End that tick normally on shutdown;
// errors while ROS is still active must reach the GUI's failure handler.
template<class Work>
bool process_ros_events(const rclcpp::Context::SharedPtr &context, Work &&work) {
  if (!context->is_valid()) return false;
  try {
    std::forward<Work>(work)();
  } catch (const std::exception &) {
    if (context->is_valid()) throw;
    return false;
  }
  return context->is_valid();
}

}  // namespace litesim3d_render
