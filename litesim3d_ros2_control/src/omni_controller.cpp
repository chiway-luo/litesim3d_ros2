#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <Eigen/Dense>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <set>

namespace litesim3d_ros2_control {
// Ordinary omni wheels: each roller releases motion perpendicular to the wheel's
// rolling direction. This is not a mecanum-wheel controller.
class OmniDriveController : public controller_interface::ControllerInterface {
  friend struct ControllerUnitAccess;
  struct Reference { double x = 0, y = 0, yaw = 0; int64_t stamp = 0; bool valid = false; uint64_t epoch = 0; };
  std::vector<std::string> joints_;
  Eigen::MatrixXd matrix_;
  std::vector<size_t> indices_;
  std::vector<double> wheel_limits_;
  double timeout_ = .5, max_linear_ = 1, max_angular_ = 1;
  realtime_tools::RealtimeBuffer<Reference> reference_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  std::atomic<bool> active_{false};
  std::atomic<uint64_t> epoch_{0};
  int64_t previous_time_ = 0;

  void halt() { for (auto &interface : command_interfaces_) interface.set_value(0); }
public:
  controller_interface::CallbackReturn on_init() override {
    try {
      auto_declare<std::vector<std::string>>("wheel_joints", {});
      for (const auto *name : {"wheel_x", "wheel_y", "wheel_radius", "wheel_angle", "max_wheel_velocity"})
        auto_declare<std::vector<double>>(name, {});
      auto_declare<double>("command_timeout", .5);
      auto_declare<double>("max_linear_velocity", 1.0);
      auto_declare<double>("max_angular_velocity", 1.0);
      return controller_interface::CallbackReturn::SUCCESS;
    } catch (const std::exception &error) {
      RCLCPP_ERROR(get_node()->get_logger(), "%s", error.what());
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  controller_interface::InterfaceConfiguration command_interface_configuration() const override {
    controller_interface::InterfaceConfiguration result{controller_interface::interface_configuration_type::INDIVIDUAL, {}};
    for (const auto &name : joints_) result.names.push_back(name + "/velocity");
    return result;
  }
  controller_interface::InterfaceConfiguration state_interface_configuration() const override {
    // This controller performs inverse kinematics. Feedback and encoder odometry
    // remain with the broadcaster and the independently selected odometry source.
    return {controller_interface::interface_configuration_type::NONE, {}};
  }
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
    try {
      auto node = get_node();
      joints_ = node->get_parameter("wheel_joints").as_string_array();
      const auto x = node->get_parameter("wheel_x").as_double_array();
      const auto y = node->get_parameter("wheel_y").as_double_array();
      const auto radius = node->get_parameter("wheel_radius").as_double_array();
      const auto angle = node->get_parameter("wheel_angle").as_double_array();
      wheel_limits_ = node->get_parameter("max_wheel_velocity").as_double_array();
      timeout_ = node->get_parameter("command_timeout").as_double();
      max_linear_ = node->get_parameter("max_linear_velocity").as_double();
      max_angular_ = node->get_parameter("max_angular_velocity").as_double();
      const size_t n = joints_.size();
      if ((n != 3 && n != 4) || std::set<std::string>(joints_.begin(), joints_.end()).size() != n ||
          x.size() != n || y.size() != n || radius.size() != n || angle.size() != n || wheel_limits_.size() != n)
        throw std::runtime_error("expected 3 or 4 unique wheels and equally sized geometry/limit arrays");
      for (double value : {timeout_, max_linear_, max_angular_})
        if (!std::isfinite(value) || value <= 0) throw std::runtime_error("timeout and body limits must be finite and positive");
      matrix_.resize(n, 3);
      for (size_t i = 0; i < n; ++i) {
        if (joints_[i].empty() || !std::isfinite(x[i]) || !std::isfinite(y[i]) ||
            !std::isfinite(angle[i]) || !std::isfinite(radius[i]) || radius[i] <= 0 ||
            !std::isfinite(wheel_limits_[i]) || wheel_limits_[i] <= 0)
          throw std::runtime_error("invalid wheel geometry or velocity limit");
        const double c = std::cos(angle[i]), s = std::sin(angle[i]);
        matrix_.row(i) << c / radius[i], s / radius[i], (x[i] * s - y[i] * c) / radius[i];
      }
      if (!matrix_.allFinite()) throw std::runtime_error("wheel geometry overflows the constraint matrix");
      Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix_);
      if (svd.singularValues()[2] <= svd.singularValues()[0] * 1e-8)
        throw std::runtime_error("wheel layout cannot independently control vx, vy and wz");
      subscription_ = node->create_subscription<geometry_msgs::msg::Twist>("~/cmd_vel", rclcpp::QoS(1),
        [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
          const auto epoch = epoch_.load();
          if (!active_.load()) return;
          Reference next;
          if (std::isfinite(msg->linear.x) && std::isfinite(msg->linear.y) && std::isfinite(msg->angular.z))
            next = {msg->linear.x, msg->linear.y, msg->angular.z, get_node()->now().nanoseconds(), true, epoch};
          // Invalid messages actively replace the last command with a stop.
          reference_.writeFromNonRT(next);
        });
      return controller_interface::CallbackReturn::SUCCESS;
    } catch (const std::exception &error) {
      RCLCPP_ERROR(get_node()->get_logger(), "%s", error.what());
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
    indices_.clear();
    for (const auto &joint : joints_) {
      const auto it = std::find_if(command_interfaces_.begin(), command_interfaces_.end(),
          [&](const auto &value) { return value.get_name() == joint + "/velocity"; });
      if (it == command_interfaces_.end()) return controller_interface::CallbackReturn::ERROR;
      indices_.push_back(std::distance(command_interfaces_.begin(), it));
    }
    // A callback from the previous activation may still be finishing. Use a
    // thread-safe buffer write and an epoch instead of deleting its storage.
    reference_.writeFromNonRT(Reference{});
    previous_time_ = 0; halt(); epoch_.fetch_add(1); active_.store(true);
    return controller_interface::CallbackReturn::SUCCESS;
  }
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
    active_.store(false); epoch_.fetch_add(1); halt();
    return controller_interface::CallbackReturn::SUCCESS;
  }
  controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &) override {
    const int64_t now = time.nanoseconds();
    auto *reference = reference_.readFromRT();
    const auto command = *reference;
    const bool rollback = previous_time_ > now;
    previous_time_ = now;
    if (rollback) reference->valid = false;
    if (!active_.load() || command.epoch != epoch_.load() || rollback || !command.valid || now < command.stamp ||
        double(now - command.stamp) * 1e-9 > timeout_) {
      halt(); return controller_interface::return_type::OK;
    }
    Eigen::Vector3d body(command.x, command.y, std::clamp(command.yaw, -max_angular_, max_angular_));
    const double norm = body.head<2>().stableNorm();
    if (norm > max_linear_) body.head<2>() *= max_linear_ / norm;
    // Uniformly scale all wheels so wheel limiting preserves the requested direction.
    double scale = 1;
    for (size_t i = 0; i < indices_.size(); ++i) {
      const double rate = matrix_.row(i).dot(body);
      if (std::abs(rate) > wheel_limits_[i]) scale = std::min(scale, wheel_limits_[i] / std::abs(rate));
    }
    for (size_t i = 0; i < indices_.size(); ++i)
      command_interfaces_[indices_[i]].set_value(matrix_.row(i).dot(body) * scale);
    return controller_interface::return_type::OK;
  }
};
}
PLUGINLIB_EXPORT_CLASS(litesim3d_ros2_control::OmniDriveController, controller_interface::ControllerInterface)
