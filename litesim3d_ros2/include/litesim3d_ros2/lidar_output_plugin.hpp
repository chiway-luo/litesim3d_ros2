#pragma once

#include <litesim3d/sim.hpp>
#include <rclcpp/rclcpp.hpp>

namespace litesim3d_ros {
// Optional ROS serialization for a simulated lidar. The host owns the node and
// calls this interface on its executor; plugins must not start hardware drivers.
class LidarOutputPlugin {
public:
  virtual ~LidarOutputPlugin() = default;
  virtual void initialize(rclcpp::Node &node, const litesim3d::Config &config,
                          const std::string &topic_prefix, const std::string &frame_prefix,
                          const litesim3d::Json &settings,
                          const rclcpp::PublisherOptions &options) = 0;
  virtual bool handles(const std::string &topic) const = 0;
  virtual bool publish(const litesim3d::Cloud &cloud) = 0;
  virtual void matches(litesim3d::Json &result) const = 0;
  virtual const litesim3d::Json &description() const = 0;
};
} // namespace litesim3d_ros
