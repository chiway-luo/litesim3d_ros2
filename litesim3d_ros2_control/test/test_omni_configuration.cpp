#include <controller_interface/controller_interface.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <gtest/gtest.h>
#include <cmath>

class OmniConfiguration : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  pluginlib::ClassLoader<controller_interface::ControllerInterface> loader_{
      "controller_interface", "controller_interface::ControllerInterface"};
  std::shared_ptr<controller_interface::ControllerInterface> make(int n) {
    auto controller = loader_.createSharedInstance("litesim3d_ros2_control/OmniDriveController");
    EXPECT_EQ(controller->init("omni_configuration_test"), controller_interface::return_type::OK);
    std::vector<std::string> names;
    std::vector<double> x, y, angles;
    for (int i = 0; i < n; ++i) {
      const double a = 2 * 3.141592653589793 * i / n;
      names.push_back("wheel_" + std::to_string(i));
      x.push_back(.3 * std::cos(a)); y.push_back(.3 * std::sin(a)); angles.push_back(a + 3.141592653589793/2);
    }
    controller->get_node()->set_parameters({rclcpp::Parameter("wheel_joints", names),
      rclcpp::Parameter("wheel_x", x), rclcpp::Parameter("wheel_y", y),
      rclcpp::Parameter("wheel_angle", angles), rclcpp::Parameter("wheel_radius", std::vector<double>(n, .1)),
      rclcpp::Parameter("max_wheel_velocity", std::vector<double>(n, 20))});
    return controller;
  }
};
TEST_F(OmniConfiguration, ThreeAndFourWheelsExposeOnlyVelocityCommands) {
  for (int n : {3, 4}) {
    auto controller = make(n);
    ASSERT_EQ(controller->on_configure(rclcpp_lifecycle::State{}), controller_interface::CallbackReturn::SUCCESS);
    const auto interfaces = controller->command_interface_configuration();
    ASSERT_EQ(interfaces.names.size(), size_t(n));
    for (int i = 0; i < n; ++i) EXPECT_EQ(interfaces.names[i], "wheel_" + std::to_string(i) + "/velocity");
    EXPECT_EQ(controller->state_interface_configuration().type, controller_interface::interface_configuration_type::NONE);
  }
}
TEST_F(OmniConfiguration, RejectsIncompleteOrDuplicateJointArrays) {
  for (const auto &parameter : {rclcpp::Parameter("wheel_x", std::vector<double>{0, 1}),
                               rclcpp::Parameter("wheel_joints", std::vector<std::string>{"a", "a", "b"})}) {
    auto controller = make(3); controller->get_node()->set_parameter(parameter);
    EXPECT_EQ(controller->on_configure(rclcpp_lifecycle::State{}), controller_interface::CallbackReturn::ERROR);
  }
}
TEST_F(OmniConfiguration, RejectsSingularNonFiniteAndOverflowingGeometry) {
  for (const auto &parameter : {rclcpp::Parameter("wheel_angle", std::vector<double>(3, 0)),
                               rclcpp::Parameter("wheel_x", std::vector<double>(3, NAN)),
                               rclcpp::Parameter("wheel_radius", std::vector<double>(3, 1e-320))}) {
    auto controller = make(3); controller->get_node()->set_parameter(parameter);
    EXPECT_EQ(controller->on_configure(rclcpp_lifecycle::State{}), controller_interface::CallbackReturn::ERROR);
  }
}
TEST_F(OmniConfiguration, RejectsInvalidTimeoutAndLimits) {
  for (const auto &parameter : {rclcpp::Parameter("command_timeout", 0.0),
                               rclcpp::Parameter("max_linear_velocity", INFINITY),
                               rclcpp::Parameter("max_angular_velocity", -1.0),
                               rclcpp::Parameter("max_wheel_velocity", std::vector<double>{1, -1, 1})}) {
    auto controller = make(3); controller->get_node()->set_parameter(parameter);
    EXPECT_EQ(controller->on_configure(rclcpp_lifecycle::State{}), controller_interface::CallbackReturn::ERROR);
  }
}
