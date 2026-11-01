#pragma once

#include <litesim3d_ros2/lidar_output_plugin.hpp>
#include <pluginlib/class_loader.hpp>
#include <functional>

namespace litesim3d_ros {
// Retain the existing launch selectors while loading vendor serialization only
// when a matching sensor actually requests it. Native output needs no plugin.
class LidarOutputs {
public:
  using PluginFactory = std::function<std::shared_ptr<LidarOutputPlugin>()>;
  LidarOutputs(rclcpp::Node &node, const litesim3d::Config &config,
               const std::string &topic_prefix, const std::string &frame_prefix,
               const std::string &interface, int xfer_format, int lidar_id,
               const rclcpp::PublisherOptions &options, PluginFactory factory = {});
  bool handles(const std::string &topic) const { return plugin_ && plugin_->handles(topic); }
  bool publish(const litesim3d::Cloud &cloud) { return plugin_ && plugin_->publish(cloud); }
  void matches(litesim3d::Json &result) const { if (plugin_) plugin_->matches(result); }
  const litesim3d::Json &description() const {
    return plugin_ ? plugin_->description() : description_;
  }
private:
  // Destroy plugin publishers, then their QoS callback weak control blocks,
  // before unloading the library that instantiated their deleters.
  std::unique_ptr<pluginlib::ClassLoader<LidarOutputPlugin>> loader_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  std::shared_ptr<LidarOutputPlugin> plugin_;
  litesim3d::Json description_;
};
} // namespace litesim3d_ros
