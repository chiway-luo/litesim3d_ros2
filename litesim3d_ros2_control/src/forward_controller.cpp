#include <forward_command_controller/forward_command_controller.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace litesim3d_ros2_control {

// Retain the upstream parameter validation, interface claims, realtime update
// and activation/deactivation buffer resets. Only command delivery differs:
// SystemDefaultsQoS can select best effort on Humble and lose a one-shot target.
class ReliableForwardCommandController
    : public forward_command_controller::ForwardCommandController {
public:
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
    const auto result = read_parameters();
    if (result != controller_interface::CallbackReturn::SUCCESS) return result;
    using Command = forward_command_controller::CmdType;
    joints_command_subscriber_ = get_node()->create_subscription<Command>(
        "~/commands", rclcpp::QoS(1).reliable().durability_volatile(),
        [this](const Command::SharedPtr message) { rt_command_ptr_.writeFromNonRT(message); });
    return controller_interface::CallbackReturn::SUCCESS;
  }
};

}  // namespace litesim3d_ros2_control

PLUGINLIB_EXPORT_CLASS(litesim3d_ros2_control::ReliableForwardCommandController,
                      controller_interface::ControllerInterface)
