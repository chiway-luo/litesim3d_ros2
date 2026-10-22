#include "lidar_outputs.hpp"
#include <algorithm>

namespace litesim3d_ros {
LidarOutputs::LidarOutputs(rclcpp::Node &node, const litesim3d::Config &config,
    const std::string &topic_prefix, const std::string &frame_prefix,
    const std::string &interface, int xfer_format, int lidar_id,
    const rclcpp::PublisherOptions &options) {
  using litesim3d::require;
  require(interface == "native" || interface == "auto" || interface == "livox",
          "lidar_interface must be native, auto or livox");
  require(xfer_format == 0 || xfer_format == 1, "xfer_format must be 0 (PointCloud2) or 1 (CustomMsg)");
  require(lidar_id >= 0 && lidar_id <= 255, "livox_lidar_id must be in [0,255]");
  description_ = {{"lidar_interface", interface}, {"xfer_format", xfer_format},
                  {"lidars", litesim3d::Json::array()}};
  if (interface == "native") return;
  const bool mid360 = std::any_of(config.lidars.begin(), config.lidars.end(),
      [](const auto &lidar) { return lidar.scan_pattern == "livox_mid360"; });
  require(interface != "livox" || mid360, "livox output requires a Mid-360 lidar");
  if (!mid360) return;
  try {
    loader_ = std::make_unique<pluginlib::ClassLoader<LidarOutputPlugin>>(
        "litesim3d_ros2", "litesim3d_ros::LidarOutputPlugin");
    plugin_ = loader_->createSharedInstance("litesim3d_mid360/LivoxOutput");
  } catch (const std::exception &error) {
    throw std::runtime_error(std::string("Mid-360 Livox output requires the litesim3d_mid360 plugin; "
        "build/source plugins/litesim3d_mid360, or select lidar_interface:=native: ") + error.what());
  }
  // The node's default callback group outlives this member. It must not retain
  // weak_ptr control blocks whose deleters live in an already unloaded plugin.
  callback_group_ = node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto plugin_options = options;
  plugin_options.callback_group = callback_group_;
  plugin_->initialize(node, config, topic_prefix, frame_prefix,
      {{"lidar_interface", interface}, {"xfer_format", xfer_format}, {"livox_lidar_id", lidar_id}}, plugin_options);
}
} // namespace litesim3d_ros
