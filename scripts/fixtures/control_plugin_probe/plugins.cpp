#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/system_interface.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <cstdlib>

namespace probe {
__attribute__((noinline)) void instance_allocation() {
  if (std::getenv("LITESIM_PROBE_LEAK")) {
    volatile int *p = static_cast<int *>(std::malloc(64));
    *p = 7;  // Deliberate instance leak: registration suppressions must not hide it.
  }
}
class Hardware : public hardware_interface::SystemInterface {
public:
  Hardware() { instance_allocation(); }
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override { return {}; }
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override { return {}; }
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override {
    return hardware_interface::return_type::OK;
  }
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override {
    return hardware_interface::return_type::OK;
  }
};
class Controller : public controller_interface::ControllerInterface {
public:
  Controller() { instance_allocation(); }
  controller_interface::CallbackReturn on_init() override {
    return controller_interface::CallbackReturn::SUCCESS;
  }
  controller_interface::InterfaceConfiguration command_interface_configuration() const override { return {}; }
  controller_interface::InterfaceConfiguration state_interface_configuration() const override { return {}; }
  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override {
    return controller_interface::return_type::OK;
  }
};
}
PLUGINLIB_EXPORT_CLASS(probe::Hardware, hardware_interface::SystemInterface)
PLUGINLIB_EXPORT_CLASS(probe::Controller, controller_interface::ControllerInterface)
