#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/system_interface.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <gtest/gtest.h>
#include <cmath>
#ifdef __linux__
#include <sched.h>
#endif

class BodyConfiguration : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  pluginlib::ClassLoader<hardware_interface::SystemInterface> hardware_loader_{
      "hardware_interface", "hardware_interface::SystemInterface"};
  pluginlib::ClassLoader<controller_interface::ControllerInterface> controller_loader_{
      "controller_interface", "controller_interface::ControllerInterface"};
  hardware_interface::HardwareInfo info() {
    hardware_interface::HardwareInfo info; info.name = "drone"; info.type = "system";
    hardware_interface::ComponentInfo gpio; gpio.name = "airframe"; gpio.type = "gpio";
    for (const auto *name : {"linear.x.velocity", "linear.y.velocity", "linear.z.velocity", "angular.z.velocity"}) {
      hardware_interface::InterfaceInfo interface; interface.name = name;
      gpio.command_interfaces.push_back(interface); gpio.state_interfaces.push_back(interface);
    }
    hardware_interface::InterfaceInfo height; height.name = "position.z"; gpio.state_interfaces.push_back(height);
    info.gpios.push_back(gpio); return info;
  }
};
TEST_F(BodyConfiguration, GPIORequiresNoFakeJointsAndEnforcesExclusiveOwnership) {
  auto hardware = hardware_loader_.createSharedInstance("litesim3d_ros2_control/LiteSimSystem");
  ASSERT_EQ(hardware->on_init(info()), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(hardware->export_command_interfaces().size(), 4U);
  EXPECT_EQ(hardware->export_state_interfaces().size(), 5U);
  EXPECT_EQ(hardware->export_state_interfaces().back().get_name(), "airframe/position.z");
  EXPECT_EQ(hardware->perform_command_mode_switch({"airframe/linear.z.velocity"}, {}), hardware_interface::return_type::OK);
  EXPECT_EQ(hardware->prepare_command_mode_switch({"airframe/linear.z.velocity"}, {}), hardware_interface::return_type::ERROR);
  EXPECT_EQ(hardware->perform_command_mode_switch({"airframe/linear.z.velocity"}, {"airframe/linear.z.velocity"}), hardware_interface::return_type::OK);
}
TEST_F(BodyConfiguration, RejectsUnsupportedAndDuplicateGPIOInterfaces) {
  for (int scenario = 0; scenario < 4; ++scenario) {
    auto config = info();
    if (scenario == 0) config.gpios[0].command_interfaces[0].name = "effort";
    if (scenario == 1) config.gpios[0].state_interfaces[0].name = "unknown";
    if (scenario == 2) config.gpios[0].command_interfaces.push_back(config.gpios[0].command_interfaces[0]);
    if (scenario == 3) config.gpios.push_back(config.gpios[0]);
    auto hardware = hardware_loader_.createSharedInstance("litesim3d_ros2_control/LiteSimSystem");
    EXPECT_EQ(hardware->on_init(config), hardware_interface::CallbackReturn::ERROR);
  }
}
TEST_F(BodyConfiguration, ImmediateDestructionDoesNotNeedGlobalShutdown) {
#ifdef __linux__
  // A single CPU makes the worker-start/destructor ordering easy to exercise.
  // Restore only this test thread's affinity; no system configuration changes.
  cpu_set_t original, single;
  ASSERT_EQ(sched_getaffinity(0, sizeof(original), &original), 0);
  CPU_ZERO(&single);
  for (int cpu = 0; cpu < CPU_SETSIZE; ++cpu) if (CPU_ISSET(cpu, &original)) {
    CPU_SET(cpu, &single); break;
  }
  ASSERT_EQ(sched_setaffinity(0, sizeof(single), &single), 0);
  struct RestoreAffinity {
    cpu_set_t mask;
    ~RestoreAffinity() { sched_setaffinity(0, sizeof(mask), &mask); }
  } restore{original};
#endif
  for (int cycle = 0; cycle < 64; ++cycle) {
    auto hardware = hardware_loader_.createSharedInstance("litesim3d_ros2_control/LiteSimSystem");
    ASSERT_EQ(hardware->on_init(info()), hardware_interface::CallbackReturn::SUCCESS);
    hardware.reset();
    ASSERT_TRUE(rclcpp::ok()) << "destroying a plugin must not shut down the manager context";
  }
}
TEST_F(BodyConfiguration, ControllerResolvesResourceAndRejectsInvalidLimits) {
  for (double limit : {1.0, 0.0, -1.0, std::numeric_limits<double>::infinity()}) {
    auto controller = controller_loader_.createSharedInstance("litesim3d_ros2_control/BodyVelocityController");
    ASSERT_EQ(controller->init("body_configuration_test"), controller_interface::return_type::OK);
    controller->get_node()->set_parameters({rclcpp::Parameter("body_name", "airframe"),
        rclcpp::Parameter("max_linear_velocity", limit)});
    EXPECT_EQ(controller->on_configure(rclcpp_lifecycle::State{}),
        limit == 1 ? controller_interface::CallbackReturn::SUCCESS : controller_interface::CallbackReturn::ERROR);
    if (limit == 1) EXPECT_EQ(controller->command_interface_configuration().names[2], "airframe/linear.z.velocity");
  }
}
