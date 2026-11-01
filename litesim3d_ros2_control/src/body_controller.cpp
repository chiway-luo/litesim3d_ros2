#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>

namespace litesim3d_ros2_control {
// A velocity-level controller for the kinematic multicopter, not a rotor/force controller.
class BodyVelocityController : public controller_interface::ControllerInterface {
  friend struct ControllerUnitAccess;
  struct Reference { std::array<double, 4> value{}; int64_t stamp = 0; uint64_t epoch = 0; bool valid = false; };
  const std::array<std::string, 4> interfaces_{
      "linear.x.velocity", "linear.y.velocity", "linear.z.velocity", "angular.z.velocity"};
  std::array<size_t, 4> indices_{};
  std::string body_;
  double timeout_ = .5, max_linear_ = 2, max_angular_ = 1.2;
  realtime_tools::RealtimeBuffer<Reference> reference_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  std::atomic<bool> active_{false};
  std::atomic<uint64_t> epoch_{0};
  int64_t previous_time_ = 0;
  void halt() { for (auto &interface : command_interfaces_) interface.set_value(0); }
public:
  controller_interface::CallbackReturn on_init() override {
    try {
      auto_declare<std::string>("body_name", "body");
      auto_declare<double>("command_timeout", .5);
      auto_declare<double>("max_linear_velocity", 2.0);
      auto_declare<double>("max_angular_velocity", 1.2);
      return controller_interface::CallbackReturn::SUCCESS;
    } catch (const std::exception &) { return controller_interface::CallbackReturn::ERROR; }
  }
  controller_interface::InterfaceConfiguration command_interface_configuration() const override {
    controller_interface::InterfaceConfiguration result{controller_interface::interface_configuration_type::INDIVIDUAL, {}};
    for (const auto &name : interfaces_) result.names.push_back(body_ + "/" + name);
    return result;
  }
  controller_interface::InterfaceConfiguration state_interface_configuration() const override {
    return {controller_interface::interface_configuration_type::NONE, {}};
  }
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
    try {
      auto node = get_node();
      body_ = node->get_parameter("body_name").as_string();
      timeout_ = node->get_parameter("command_timeout").as_double();
      max_linear_ = node->get_parameter("max_linear_velocity").as_double();
      max_angular_ = node->get_parameter("max_angular_velocity").as_double();
      if (body_.empty()) throw std::runtime_error("body_name must not be empty");
      for (double value : {timeout_, max_linear_, max_angular_})
        if (!std::isfinite(value) || value <= 0) throw std::runtime_error("timeout and limits must be finite and positive");
      subscription_ = node->create_subscription<geometry_msgs::msg::Twist>("~/cmd_vel", rclcpp::QoS(1),
        [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
          const auto epoch = epoch_.load();
          if (!active_.load()) return;
          Reference next;
          next.value = {msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.z};
          next.valid = msg->angular.x == 0 && msg->angular.y == 0 &&
              std::all_of(next.value.begin(), next.value.end(), [](double v) { return std::isfinite(v); });
          next.stamp = get_node()->now().nanoseconds(); next.epoch = epoch;
          reference_.writeFromNonRT(next);
        });
      return controller_interface::CallbackReturn::SUCCESS;
    } catch (const std::exception &error) {
      RCLCPP_ERROR(get_node()->get_logger(), "%s", error.what());
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
    for (size_t i = 0; i < interfaces_.size(); ++i) {
      const auto it = std::find_if(command_interfaces_.begin(), command_interfaces_.end(),
          [&](const auto &value) { return value.get_name() == body_ + "/" + interfaces_[i]; });
      if (it == command_interfaces_.end()) return controller_interface::CallbackReturn::ERROR;
      indices_[i] = std::distance(command_interfaces_.begin(), it);
    }
    reference_.writeFromNonRT(Reference{}); previous_time_ = 0;
    halt(); epoch_.fetch_add(1); active_.store(true);
    return controller_interface::CallbackReturn::SUCCESS;
  }
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
    active_.store(false); epoch_.fetch_add(1); halt();
    return controller_interface::CallbackReturn::SUCCESS;
  }
  controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &) override {
    const auto now = time.nanoseconds();
    auto *reference = reference_.readFromRT(); const auto command = *reference;
    const bool rollback = previous_time_ > now; previous_time_ = now;
    if (rollback) reference->valid = false;
    if (!active_.load() || command.epoch != epoch_.load() || !command.valid || rollback || now < command.stamp ||
        double(now - command.stamp) * 1e-9 > timeout_) {
      halt(); return controller_interface::return_type::OK;
    }
    // Scale before computing length to avoid overflow for very large finite inputs.
    auto value = command.value;
    const double largest = std::max({std::abs(value[0]), std::abs(value[1]), std::abs(value[2])});
    if (largest > 0) {
      const double norm = std::hypot(value[0] / largest, value[1] / largest, value[2] / largest);
      if (largest > max_linear_ / norm)
        for (size_t i = 0; i < 3; ++i) value[i] = (value[i] / largest) * (max_linear_ / norm);
    }
    value[3] = std::clamp(value[3], -max_angular_, max_angular_);
    for (size_t i = 0; i < indices_.size(); ++i) command_interfaces_[indices_[i]].set_value(value[i]);
    return controller_interface::return_type::OK;
  }
};
}
// Only direct-class units omit registration; the installed DSO still exports.
#ifndef LITESIM3D_NO_PLUGIN_REGISTRATION
PLUGINLIB_EXPORT_CLASS(litesim3d_ros2_control::BodyVelocityController, controller_interface::ControllerInterface)
#endif
