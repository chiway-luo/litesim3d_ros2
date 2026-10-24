#pragma once
#include <litesim3d/sim.hpp>
#include <litesim3d_interfaces/msg/joint_command.hpp>
#include <litesim3d_interfaces/msg/joint_feedback.hpp>
#include <rclcpp/rclcpp.hpp>

namespace litesim3d_ros {
// Transport only: the core owns limits, integration, collision and feedback.
class JointControl {
public:
  using Command = litesim3d_interfaces::msg::JointCommand;
  using Feedback = litesim3d_interfaces::msg::JointFeedback;
  JointControl(rclcpp::Node &node, const litesim3d::Simulator &sim,
      std::function<bool(const litesim3d::JointCommand &)> accept, const std::string &prefix = "")
      : node_(node), sim_(sim) {
    feedback_ = node.create_publisher<Feedback>(prefix + "sim/joint_feedback", rclcpp::QoS(1).reliable());
    command_ = node.create_subscription<Command>(prefix + "sim/joint_commands", rclcpp::QoS(1).reliable(),
        [this, accept](Command::ConstSharedPtr msg) {
          if (msg->run_id != sim_.run_id || msg->header.stamp.sec < 0 ||
              msg->header.stamp.nanosec >= 1000000000 || msg->name.size() != msg->mode.size() ||
              msg->name.size() != msg->command.size() || command_->get_publisher_count() > 1) {
            RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000,
                "Rejected joint command: session, layout, timestamp or multiple command publishers");
            return;
          }
          litesim3d::JointCommand cmd;
          cmd.stamp = uint64_t(msg->header.stamp.sec) * 1000000000 + msg->header.stamp.nanosec;
          for (size_t i = 0; i < msg->name.size(); ++i)
            if (!cmd.joints.emplace(msg->name[i], litesim3d::JointTarget{
                static_cast<litesim3d::JointCommandMode>(msg->mode[i]), msg->command[i]}).second) return;
          if (msg->body_velocity_control) {
            if (msg->body_velocity.angular.x != 0 || msg->body_velocity.angular.y != 0) return;
            cmd.body_velocity = litesim3d::plugins::Twist{msg->body_velocity.linear.x,
                msg->body_velocity.linear.y, msg->body_velocity.linear.z, msg->body_velocity.angular.z};
          }
          if (!accept(cmd)) RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000,
              "Rejected joint command: %s", sim_.diagnostics().at("last_error").get<std::string>().c_str());
        });
  }
  void publish(const sensor_msgs::msg::JointState &state, const litesim3d::Snapshot &snapshot) {
    Feedback msg;
    msg.run_id = sim_.run_id;
    msg.operational = sim_.state() != litesim3d::State::Fault && sim_.state() != litesim3d::State::Exited;
    msg.state = state;
    msg.body_command_supported = sim_.config().drive.model == "multicopter";
    const auto &p = snapshot.world_base.translation();
    const Eigen::Quaterniond q(snapshot.world_base.linear());
    msg.body_pose.position.x = p.x(); msg.body_pose.position.y = p.y(); msg.body_pose.position.z = p.z();
    msg.body_pose.orientation.x = q.x(); msg.body_pose.orientation.y = q.y();
    msg.body_pose.orientation.z = q.z(); msg.body_pose.orientation.w = q.w();
    msg.body_velocity.linear.x = snapshot.v; msg.body_velocity.linear.y = snapshot.lateral;
    msg.body_velocity.linear.z = snapshot.vertical; msg.body_velocity.angular.z = snapshot.w;
    feedback_->publish(msg);
  }
private:
  rclcpp::Node &node_;
  const litesim3d::Simulator &sim_;
  rclcpp::Publisher<Feedback>::SharedPtr feedback_;
  rclcpp::Subscription<Command>::SharedPtr command_;
};
} // namespace litesim3d_ros
