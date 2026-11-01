// Direct controller unit tests: callback dispatch and update operate in-process,
// without a controller manager, simulator, publisher peer or ROS transport.
#include "../src/body_controller.cpp"
#include "../src/omni_controller.cpp"
#include <class_loader/class_loader_core.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <gtest/gtest.h>
#include <limits>

namespace litesim3d_ros2_control {
struct ControllerUnitAccess {
  template<class Controller>
  static void deliver(Controller &controller, const geometry_msgs::msg::Twist &message) {
    std::shared_ptr<void> erased = std::make_shared<geometry_msgs::msg::Twist>(message);
    controller.subscription_->handle_message(erased, rclcpp::MessageInfo{});
  }
  static void reference(BodyVelocityController &controller, std::array<double, 4> value,
                        int64_t stamp, bool valid = true, bool old_epoch = false) {
    BodyVelocityController::Reference next;
    next.value = value; next.stamp = stamp; next.valid = valid;
    next.epoch = controller.epoch_.load() - unsigned(old_epoch);
    controller.reference_.writeFromNonRT(next);
  }
  static void reference(OmniDriveController &controller, std::array<double, 4> value,
                        int64_t stamp, bool valid = true, bool old_epoch = false) {
    OmniDriveController::Reference next;
    next.x = value[0]; next.y = value[1]; next.yaw = value[3];
    next.stamp = stamp; next.valid = valid;
    next.epoch = controller.epoch_.load() - unsigned(old_epoch);
    controller.reference_.writeFromNonRT(next);
  }
};
}

namespace {
using namespace litesim3d_ros2_control;
using Callback = controller_interface::CallbackReturn;
using Return = controller_interface::return_type;
constexpr int64_t second = 1000000000;
TEST(ControllerRegistrationUnit, DirectClassesDoNotInstallUnownedPluginFactories) {
  const auto classes = class_loader::impl::getAvailableClasses<
      controller_interface::ControllerInterface>(nullptr);
  EXPECT_EQ(std::count(classes.begin(), classes.end(),
      "litesim3d_ros2_control::BodyVelocityController"), 0);
  EXPECT_EQ(std::count(classes.begin(), classes.end(),
      "litesim3d_ros2_control::OmniDriveController"), 0);
  EXPECT_FALSE(class_loader::impl::hasANonPurePluginLibraryBeenOpened());
}
struct Commands {
  std::vector<double> values;
  std::vector<hardware_interface::CommandInterface> owned;
  std::vector<std::string> names;
  void assign(controller_interface::ControllerInterface &controller, const std::vector<std::string> &requested) {
    names = requested;
    std::reverse(names.begin(), names.end());
    names.push_back("extra/unused");
    values.assign(names.size(), 99);
    owned.reserve(names.size());
    for (size_t i = 0; i < names.size(); ++i) {
      const auto separator = names[i].find('/');
      owned.emplace_back(names[i].substr(0, separator), names[i].substr(separator + 1), &values[i]);
    }
    std::vector<hardware_interface::LoanedCommandInterface> loans;
    for (auto &interface : owned) loans.emplace_back(interface);
    controller.assign_interfaces(std::move(loans), {});
  }
  double at(const std::string &name) const {
    const auto found = std::find(names.begin(), names.end(), name);
    if (found == names.end()) throw std::logic_error("test requested an unknown interface");
    return values[std::distance(names.begin(), found)];
  }
  void stopped() const { for (double value : values) EXPECT_EQ(value, 0); }
};
class ControllerUnits : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void body(BodyVelocityController &controller) {
    ASSERT_EQ(controller.init("body_unit"), Return::OK);
    ASSERT_EQ(controller.on_configure(rclcpp_lifecycle::State{}), Callback::SUCCESS);
  }
  void omni(OmniDriveController &controller, int count = 3) {
    ASSERT_EQ(controller.init("omni_unit"), Return::OK);
    std::vector<std::string> names;
    std::vector<double> x, y, angle;
    for (int i = 0; i < count; ++i) {
      const double theta = 2 * M_PI * i / count;
      names.push_back("wheel_" + std::to_string(i));
      x.push_back(.3 * std::cos(theta)); y.push_back(.3 * std::sin(theta)); angle.push_back(theta + M_PI / 2);
    }
    controller.get_node()->set_parameters({rclcpp::Parameter("wheel_joints", names),
      rclcpp::Parameter("wheel_x", x), rclcpp::Parameter("wheel_y", y),
      rclcpp::Parameter("wheel_angle", angle), rclcpp::Parameter("wheel_radius", std::vector<double>(count, .1)),
      rclcpp::Parameter("max_wheel_velocity", std::vector<double>(count, 2))});
    ASSERT_EQ(controller.on_configure(rclcpp_lifecycle::State{}), Callback::SUCCESS);
  }
  template<class Controller>
  void update(Controller &controller, int64_t now) {
    EXPECT_EQ(controller.update(rclcpp::Time(now, RCL_ROS_TIME), rclcpp::Duration(0, 10000000)), Return::OK);
  }
  template<class Controller>
  void staleCases(Controller &controller, Commands &commands) {
    ControllerUnitAccess::reference(controller, {.1, .1, .1, .1}, second);
    update(controller, second);
    EXPECT_TRUE(std::any_of(commands.values.begin(), commands.values.end(), [](double v) { return v != 0; }));
    ControllerUnitAccess::reference(controller, {.1, .1, .1, .1}, second, true, true);
    update(controller, second); commands.stopped();  // previous activation's epoch
    ControllerUnitAccess::reference(controller, {.1, .1, .1, .1}, second, false);
    update(controller, second); commands.stopped();
    ControllerUnitAccess::reference(controller, {.1, .1, .1, .1}, 2 * second);
    update(controller, second); commands.stopped();  // future command
    ControllerUnitAccess::reference(controller, {.1, .1, .1, .1}, second);
    update(controller, 2 * second); commands.stopped();  // watchdog expiration
    ControllerUnitAccess::reference(controller, {.1, .1, .1, .1}, second);
    update(controller, second); commands.stopped();  // clock rollback latches invalidity
    update(controller, second + 1); commands.stopped();
    EXPECT_EQ(controller.on_deactivate(rclcpp_lifecycle::State{}), Callback::SUCCESS);
    ControllerUnitAccess::reference(controller, {.1, .1, .1, .1}, second + 1);
    update(controller, second + 1); commands.stopped();  // inactive controller
    ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State{}), Callback::SUCCESS);
    update(controller, second + 1); commands.stopped();  // no stale command after activation
  }
};
}

TEST_F(ControllerUnits, BodyLimitsThreeDimensionalNormAndAvoidsFiniteOverflow) {
  BodyVelocityController controller; body(controller);
  Commands commands; commands.assign(controller, controller.command_interface_configuration().names);
  EXPECT_EQ(controller.state_interface_configuration().type, controller_interface::interface_configuration_type::NONE);
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State{}), Callback::SUCCESS); commands.stopped();
  ControllerUnitAccess::reference(controller, {3, 4, 0, 9}, second);
  update(controller, second);
  EXPECT_NEAR(commands.at("body/linear.x.velocity"), 1.2, 1e-12);
  EXPECT_NEAR(commands.at("body/linear.y.velocity"), 1.6, 1e-12);
  EXPECT_EQ(commands.at("body/linear.z.velocity"), 0);
  EXPECT_EQ(commands.at("body/angular.z.velocity"), 1.2);
  ControllerUnitAccess::reference(controller, {0, 0, 0, -9}, second);
  update(controller, second); EXPECT_EQ(commands.at("body/angular.z.velocity"), -1.2);
  ControllerUnitAccess::reference(controller, {.1, -.2, .3, -.4}, second);
  update(controller, second);
  EXPECT_EQ(commands.at("body/linear.x.velocity"), .1);
  EXPECT_EQ(commands.at("body/linear.y.velocity"), -.2);
  EXPECT_EQ(commands.at("body/linear.z.velocity"), .3);
  const double large = std::numeric_limits<double>::max();
  ControllerUnitAccess::reference(controller, {large, -large, large, 0}, second);
  update(controller, second);
  const double expected = 2 / std::sqrt(3.0);
  EXPECT_NEAR(commands.at("body/linear.x.velocity"), expected, 1e-12);
  EXPECT_NEAR(commands.at("body/linear.y.velocity"), -expected, 1e-12);
  EXPECT_NEAR(commands.at("body/linear.z.velocity"), expected, 1e-12);
}

TEST_F(ControllerUnits, BodyRejectsMissingInterfacesAndStaleReferences) {
  BodyVelocityController controller; body(controller);
  EXPECT_EQ(controller.on_activate(rclcpp_lifecycle::State{}), Callback::ERROR);
  Commands commands; commands.assign(controller, controller.command_interface_configuration().names);
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State{}), Callback::SUCCESS);
  staleCases(controller, commands);
}

TEST_F(ControllerUnits, BodyCallbackAcceptsFiniteCommandsAndReplacesInvalidInputsWithStop) {
  BodyVelocityController controller; body(controller);
  Commands commands; commands.assign(controller, controller.command_interface_configuration().names);
  geometry_msgs::msg::Twist message; message.linear.x = .25;
  ControllerUnitAccess::deliver(controller, message);  // inactive callback must not arm a command
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State{}), Callback::SUCCESS);
  update(controller, controller.get_node()->now().nanoseconds()); commands.stopped();
  for (int defect = -1; defect < 6; ++defect) {
    geometry_msgs::msg::Twist input; input.linear.x = .25;
    if (defect == 0) input.angular.x = .1;
    if (defect == 1) input.angular.y = .1;
    if (defect == 2) input.linear.x = NAN;
    if (defect == 3) input.linear.y = INFINITY;
    if (defect == 4) input.linear.z = -INFINITY;
    if (defect == 5) input.angular.z = NAN;
    ControllerUnitAccess::deliver(controller, input);
    update(controller, controller.get_node()->now().nanoseconds());
    if (defect < 0) EXPECT_EQ(commands.at("body/linear.x.velocity"), .25);
    else commands.stopped();
  }
}

TEST_F(ControllerUnits, OmniPreservesDirectionWhileApplyingBodyAndWheelLimits) {
  for (int count : {3, 4}) {
    OmniDriveController controller; omni(controller, count);
    Commands commands; commands.assign(controller, controller.command_interface_configuration().names);
    ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State{}), Callback::SUCCESS);
    for (const std::array<double, 4> input : {std::array<double, 4>{0, 0, 0, 0}, {.03, -.02, 0, .04}, {3, 4, 0, 4}, {-3, -4, 0, -4}}) {
      ControllerUnitAccess::reference(controller, input, second); update(controller, second);
      double x = input[0], y = input[1];
      const double norm = std::hypot(x, y);
      if (norm > 1) { x /= norm; y /= norm; }
      const double yaw = std::clamp(input[3], -1.0, 1.0);
      std::vector<double> expected(count);
      double scale = 1;
      for (int i = 0; i < count; ++i) {
        const double theta = 2 * M_PI * i / count;
        expected[i] = (-std::sin(theta) * x + std::cos(theta) * y + .3 * yaw) / .1;
        if (std::abs(expected[i]) > 2) scale = std::min(scale, 2 / std::abs(expected[i]));
      }
      for (int i = 0; i < count; ++i)
        EXPECT_NEAR(commands.at("wheel_" + std::to_string(i) + "/velocity"), expected[i] * scale, 1e-12);
    }
  }
}

TEST_F(ControllerUnits, OmniRejectsMissingInterfacesStaleReferencesAndInvalidCallbacks) {
  OmniDriveController controller; omni(controller);
  EXPECT_EQ(controller.on_activate(rclcpp_lifecycle::State{}), Callback::ERROR);
  Commands commands; commands.assign(controller, controller.command_interface_configuration().names);
  geometry_msgs::msg::Twist message; message.linear.x = .03;
  ControllerUnitAccess::deliver(controller, message);
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State{}), Callback::SUCCESS);
  staleCases(controller, commands);
  for (int defect = -1; defect < 3; ++defect) {
    auto input = message;
    if (defect == 0) input.linear.x = NAN;
    if (defect == 1) input.linear.y = INFINITY;
    if (defect == 2) input.angular.z = -INFINITY;
    ControllerUnitAccess::deliver(controller, input);
    update(controller, controller.get_node()->now().nanoseconds());
    if (defect < 0) EXPECT_NE(commands.at("wheel_1/velocity"), 0);
    else commands.stopped();
  }
}

TEST_F(ControllerUnits, EveryConfigurationArrayAndFiniteGeometryGuardIsValidated) {
  BodyVelocityController body_controller; body(body_controller);
  body_controller.get_node()->set_parameter(rclcpp::Parameter("body_name", ""));
  EXPECT_EQ(body_controller.on_configure(rclcpp_lifecycle::State{}), Callback::ERROR);
  const std::vector<rclcpp::Parameter> invalid{
    rclcpp::Parameter("wheel_joints", std::vector<std::string>{"one", "two"}),
    rclcpp::Parameter("wheel_y", std::vector<double>{0, 0}),
    rclcpp::Parameter("wheel_radius", std::vector<double>{.1, .1}),
    rclcpp::Parameter("wheel_angle", std::vector<double>{0, 0}),
    rclcpp::Parameter("max_wheel_velocity", std::vector<double>{2, 2}),
    rclcpp::Parameter("wheel_joints", std::vector<std::string>{"", "one", "two"}),
    rclcpp::Parameter("wheel_y", std::vector<double>{NAN, 0, 0}),
    rclcpp::Parameter("wheel_angle", std::vector<double>{NAN, 0, 0}),
    rclcpp::Parameter("wheel_radius", std::vector<double>{NAN, .1, .1}),
    rclcpp::Parameter("wheel_radius", std::vector<double>{0, .1, .1}),
    rclcpp::Parameter("max_wheel_velocity", std::vector<double>{NAN, 2, 2})};
  for (const auto &parameter : invalid) {
    OmniDriveController controller; omni(controller);
    controller.get_node()->set_parameter(parameter);
    EXPECT_EQ(controller.on_configure(rclcpp_lifecycle::State{}), Callback::ERROR) << parameter.get_name();
  }
}
