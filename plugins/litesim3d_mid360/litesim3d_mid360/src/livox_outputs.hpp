#pragma once
#include "litesim3d/sim.hpp"
#include <litesim3d_ros2/lidar_output_plugin.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace litesim3d_ros {
// ROS serialization only: core sampling and geometry remain unchanged.
class LivoxOutputs final : public LidarOutputPlugin {
public:
  void initialize(rclcpp::Node &node, const litesim3d::Config &config,
                  const std::string &topic_prefix, const std::string &frame_prefix,
                  const litesim3d::Json &settings,
                  const rclcpp::PublisherOptions &options) override;
  bool handles(const std::string &topic) const override { return lidars_.count(topic) != 0; }
  bool publish(const litesim3d::Cloud &cloud) override;
  void matches(litesim3d::Json &result) const override;
  const litesim3d::Json &description() const override { return description_; }
private:
  struct Lidar {
    litesim3d::LidarConfig config;
    uint8_t lidar_id = 0;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud, boundary;
    rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr custom;
  };
  std::map<std::string, Lidar> lidars_;
  std::string frame_prefix_;
  litesim3d::Json description_;
};
} // namespace litesim3d_ros
