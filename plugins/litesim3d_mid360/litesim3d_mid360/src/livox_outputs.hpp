#pragma once
#include "litesim3d/sim.hpp"
#include <litesim3d_ros2/lidar_output_plugin.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <functional>

namespace litesim3d_ros {
// ROS serialization only: core sampling and geometry remain unchanged.
class LivoxOutputs final : public LidarOutputPlugin {
public:
  // Publisher capabilities keep serialization independently testable. The
  // default factory below still creates the same real ROS publishers and QoS.
  template<class Message> struct Output {
    std::string topic;
    std::function<void(const Message &)> send;
    std::function<size_t()> subscriptions;
    explicit operator bool() const { return bool(send); }
    void publish(const Message &message) const { send(message); }
    const char *get_topic_name() const { return topic.c_str(); }
    size_t get_subscription_count() const { return subscriptions(); }
  };
  using CloudOutput = Output<sensor_msgs::msg::PointCloud2>;
  using CustomOutput = Output<livox_ros_driver2::msg::CustomMsg>;
  struct Transport {
    std::function<std::string(rclcpp::Node &,const std::string &)> resolve;
    std::function<CloudOutput(rclcpp::Node &,const std::string &,const rclcpp::QoS &,
                             const rclcpp::PublisherOptions &)> cloud;
    std::function<CustomOutput(rclcpp::Node &,const std::string &,const rclcpp::QoS &,
                              const rclcpp::PublisherOptions &)> custom;
  };
  explicit LivoxOutputs(Transport transport = {}) : transport_(std::move(transport)) {}
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
    CloudOutput cloud, boundary;
    CustomOutput custom;
  };
  std::map<std::string, Lidar> lidars_;
  std::string frame_prefix_;
  litesim3d::Json description_;
  Transport transport_;
};
} // namespace litesim3d_ros
