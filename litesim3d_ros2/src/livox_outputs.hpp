#pragma once
#include "litesim3d/sim.hpp"
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace litesim3d_ros {
// ROS serialization only: core sampling and geometry remain unchanged.
class LivoxOutputs {
public:
  LivoxOutputs(rclcpp::Node &node, const litesim3d::Config &config,
                std::string topic_prefix, std::string frame_prefix,
                const std::string &interface, int xfer_format, int lidar_id,
                const rclcpp::PublisherOptions &options);
  bool handles(const std::string &topic) const { return lidars_.count(topic) != 0; }
  bool publish(const litesim3d::Cloud &cloud);
  void matches(litesim3d::Json &result) const;
  const litesim3d::Json &description() const { return description_; }
private:
  struct Lidar {
    litesim3d::LidarConfig config;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud, boundary;
    rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr custom;
  };
  std::map<std::string, Lidar> lidars_;
  std::string frame_prefix_;
  uint8_t lidar_id_ = 0;
  litesim3d::Json description_;
};
} // namespace litesim3d_ros
