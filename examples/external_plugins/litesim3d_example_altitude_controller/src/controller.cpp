#include <controller_interface/controller_interface.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <std_msgs/msg/float64.hpp>
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>

namespace example {
class AltitudeController : public controller_interface::ControllerInterface {
  struct Target { double height = 0; int64_t stamp = 0; uint64_t epoch = 0; bool valid = false; };
  const std::array<std::string, 4> axes_{
      "linear.x.velocity", "linear.y.velocity", "linear.z.velocity", "angular.z.velocity"};
  std::string body_;
  size_t z_command_ = 0, z_state_ = 0;
  double gain_ = 1, limit_ = .4, timeout_ = .5;
  int64_t previous_time_ = 0;
  std::atomic<uint64_t> epoch_{0};
  std::atomic<bool> active_{false};
  realtime_tools::RealtimeBuffer<Target> target_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
  void halt() { for (auto &handle : command_interfaces_) handle.set_value(0); }
public:
  controller_interface::CallbackReturn on_init() override {
    try {
      auto_declare<std::string>("body_name", "body");
      auto_declare<std::string>("target_topic", "~/target_height");
      auto_declare<double>("gain", 1.0);
      auto_declare<double>("max_vertical_velocity", .4);
      auto_declare<double>("command_timeout", .5);
      return controller_interface::CallbackReturn::SUCCESS;
    } catch (const std::exception &) { return controller_interface::CallbackReturn::ERROR; }
  }
  controller_interface::InterfaceConfiguration command_interface_configuration() const override {
    controller_interface::InterfaceConfiguration out{controller_interface::interface_configuration_type::INDIVIDUAL, {}};
    for (const auto &name : axes_) out.names.push_back(body_ + "/" + name);
    return out;
  }
  controller_interface::InterfaceConfiguration state_interface_configuration() const override {
    return {controller_interface::interface_configuration_type::INDIVIDUAL, {body_ + "/position.z"}};
  }
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
    try {
      auto node = get_node(); body_ = node->get_parameter("body_name").as_string();
      gain_ = node->get_parameter("gain").as_double();
      limit_ = node->get_parameter("max_vertical_velocity").as_double();
      timeout_ = node->get_parameter("command_timeout").as_double();
      if (body_.empty()) throw std::runtime_error("empty body_name");
      for (auto value : {gain_, limit_, timeout_})
        if (!std::isfinite(value) || value <= 0) throw std::runtime_error("gain, limit and timeout must be finite and positive");
      subscription_ = node->create_subscription<std_msgs::msg::Float64>(node->get_parameter("target_topic").as_string(), 1,
        [this](std_msgs::msg::Float64::ConstSharedPtr msg) {
          const auto epoch = epoch_.load(); if (!active_.load()) return;
          target_.writeFromNonRT({msg->data, get_node()->now().nanoseconds(), epoch, std::isfinite(msg->data)});
        });
      return controller_interface::CallbackReturn::SUCCESS;
    } catch (const std::exception &error) {
      RCLCPP_ERROR(get_node()->get_logger(), "%s", error.what());
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
    auto command = std::find_if(command_interfaces_.begin(), command_interfaces_.end(),
        [&](const auto &h) { return h.get_name() == body_ + "/linear.z.velocity"; });
    auto state = std::find_if(state_interfaces_.begin(), state_interfaces_.end(),
        [&](const auto &h) { return h.get_name() == body_ + "/position.z"; });
    if (command == command_interfaces_.end() || state == state_interfaces_.end() || command_interfaces_.size() != 4)
      return controller_interface::CallbackReturn::ERROR;
    z_command_ = std::distance(command_interfaces_.begin(), command);
    z_state_ = std::distance(state_interfaces_.begin(), state);
    target_.writeFromNonRT(Target{}); previous_time_ = 0; halt(); epoch_.fetch_add(1); active_.store(true);
    return controller_interface::CallbackReturn::SUCCESS;
  }
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
    active_.store(false); epoch_.fetch_add(1); halt(); return controller_interface::CallbackReturn::SUCCESS;
  }
  controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &) override {
    halt();
    const auto now = time.nanoseconds(); auto *buffer = target_.readFromRT(); const auto target = *buffer;
    const bool rollback = now < previous_time_; previous_time_ = now;
    if (rollback) buffer->valid = false;
    const double height = state_interfaces_[z_state_].get_value();
    if (!std::isfinite(height)) return controller_interface::return_type::ERROR;
    if (!active_.load() || target.epoch != epoch_.load() || !target.valid || rollback || now < target.stamp ||
        (now - target.stamp) * 1e-9 > timeout_) return controller_interface::return_type::OK;
    // The resource supplies world height in metres. All other body axes remain stopped.
    const double velocity = std::clamp(gain_ * (target.height - height), -limit_, limit_);
    command_interfaces_[z_command_].set_value(velocity);
    return controller_interface::return_type::OK;
  }
};
}
PLUGINLIB_EXPORT_CLASS(example::AltitudeController, controller_interface::ControllerInterface)
