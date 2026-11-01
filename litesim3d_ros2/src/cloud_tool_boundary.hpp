#pragma once
#include <cstdlib>
#include <rclcpp/rclcpp.hpp>

namespace litesim3d_ros2 {
// The production boundary retains the normal ROS node, QoS and message types.
// Unit tests substitute only these I/O/lifecycle ports; conversion, validation,
// PCL processing and status serialization remain the same implementation.
struct CloudToolBoundary {
  using Node = rclcpp::Node;
  template<class Message> using Publisher = typename rclcpp::Publisher<Message>::SharedPtr;
  template<class Message> using Subscription = typename rclcpp::Subscription<Message>::SharedPtr;
  using Timer = rclcpp::TimerBase::SharedPtr;
  static const char *environment(const char *name) { return std::getenv(name); }
  static void initialize(int argc, char **argv) { rclcpp::init(argc, argv); }
  template<class NodeType> static void spin(const std::shared_ptr<NodeType> &node) { rclcpp::spin(node); }
  static void shutdown() { rclcpp::shutdown(); }
};
} // namespace litesim3d_ros2
