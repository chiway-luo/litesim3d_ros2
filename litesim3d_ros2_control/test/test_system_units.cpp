// Validation/state-machine units: stop the private executor immediately, inject
// feedback directly and assert exported scalar state. No ROS peer is launched.
#include "../src/system.cpp"
#include <gtest/gtest.h>
#include <limits>

namespace litesim3d_ros2_control {
struct SystemUnitAccess {
  static void stopTransport(LiteSimSystem &system) {
    system.stop_transport_.store(true); system.executor_.cancel();
    if (system.transport_thread_.joinable()) system.transport_thread_.join();
  }
  static void receive(LiteSimSystem &system, const litesim3d_interfaces::msg::JointFeedback &message) {
    system.receive(message);
  }
  static bool fault(const LiteSimSystem &system) { return system.fault_; }
  static bool received(const LiteSimSystem &system) { return system.received_; }
  static void active(LiteSimSystem &system, bool value) { system.active_ = value; }
  static void expire(LiteSimSystem &system) { system.received_at_ -= std::chrono::seconds(10); }
  static uint8_t mode(const LiteSimSystem &system, size_t index) { return system.mode_.at(index); }
  static bool reset(const LiteSimSystem &system, size_t index) { return system.reset_on_write_.at(index); }
  static void watchdog(LiteSimSystem &system) { system.watchdog_->execute_callback(); }
  static void deliver(LiteSimSystem &system, const litesim3d_interfaces::msg::JointFeedback &message) {
    std::shared_ptr<void> erased = std::make_shared<litesim3d_interfaces::msg::JointFeedback>(message);
    system.subscription_->handle_message(erased, rclcpp::MessageInfo{});
  }
  static size_t feedbackPublishers(const LiteSimSystem &system) { return system.feedbackPublisherCount(); }
  static size_t commandSubscribers(const LiteSimSystem &system) { return system.commandSubscriberCount(); }
};
}
namespace {
using namespace litesim3d_ros2_control;
using Feedback = litesim3d_interfaces::msg::JointFeedback;
using Command = litesim3d_interfaces::msg::JointCommand;
// The unit doubles replace only external endpoint discovery and sends. All
// configuration, feedback validation, ownership and write logic is production.
class IsolatedSystem final : public LiteSimSystem {
public:
  size_t feedback_sources = 1, command_sinks = 1;
  std::vector<litesim3d_interfaces::msg::JointCommand> published;
protected:
  size_t feedbackPublisherCount() const override { return feedback_sources; }
  size_t commandSubscriberCount() const override { return command_sinks; }
  void publishCommand(const litesim3d_interfaces::msg::JointCommand &message) override { published.push_back(message); }
};
hardware_interface::InterfaceInfo interface(const char *name) {
  hardware_interface::InterfaceInfo result; result.name = name; return result;
}
hardware_interface::HardwareInfo info(bool body = true) {
  hardware_interface::HardwareInfo result; result.name = "system_unit"; result.type = "system";
  hardware_interface::ComponentInfo joint; joint.name = "arm"; joint.type = "joint";
  joint.state_interfaces = {interface("position"), interface("velocity")};
  joint.command_interfaces = joint.state_interfaces; result.joints.push_back(joint);
  joint.name = "mimic"; joint.command_interfaces.clear(); result.joints.push_back(joint);
  if (body) {
    hardware_interface::ComponentInfo gpio; gpio.name = "airframe"; gpio.type = "gpio";
    gpio.command_interfaces = {interface("linear.x.velocity"), interface("linear.y.velocity"),
                               interface("linear.z.velocity"), interface("angular.z.velocity")};
    gpio.state_interfaces = gpio.command_interfaces;
    for (const char *name : {"position.x", "position.y", "position.z", "orientation.x", "orientation.y", "orientation.z", "orientation.w"})
      gpio.state_interfaces.push_back(interface(name));
    result.gpios.push_back(gpio);
  }
  result.hardware_parameters["activation_timeout"] = "0.001";
  return result;
}
Feedback feedback() {
  Feedback result; result.run_id = "unit-session"; result.operational = true;
  result.body_command_supported = true;
  result.state.header.stamp.sec = 2; result.state.header.stamp.nanosec = 3;
  // Feedback order deliberately differs from hardware resource order.
  result.state.name = {"mimic", "arm", "other"};
  result.state.position = {-2, 1.5, 12}; result.state.velocity = {-.5, .25, 9};
  result.body_pose.position.x = 1; result.body_pose.position.y = 2; result.body_pose.position.z = 3;
  result.body_pose.orientation.w = 1;
  result.body_velocity.linear.x = .1; result.body_velocity.linear.y = .2;
  result.body_velocity.linear.z = .3; result.body_velocity.angular.z = .4;
  return result;
}
class SystemUnits : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void initialize(LiteSimSystem &system, bool body = true) {
    ASSERT_EQ(system.on_init(info(body)), hardware_interface::CallbackReturn::SUCCESS);
    SystemUnitAccess::stopTransport(system);
    ASSERT_EQ(system.on_configure(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
  }
  hardware_interface::return_type read(LiteSimSystem &system) {
    return system.read(rclcpp::Time(int64_t{0}, RCL_ROS_TIME), rclcpp::Duration(0, 0));
  }
  hardware_interface::return_type write(LiteSimSystem &system) {
    return system.write(rclcpp::Time(int64_t{0}, RCL_ROS_TIME), rclcpp::Duration(0, 0));
  }
};
}

TEST_F(SystemUnits, RejectsUnsupportedHardwareInterfacesAndTimeouts) {
  for (unsigned defect = 0; defect < 17; ++defect) {
    auto config = info();
    switch (defect) {
    case 0: config.joints.clear(); config.gpios.clear(); break;
    case 1: config.sensors.push_back({}); break;
    case 2: config.gpios.push_back(config.gpios.front()); break;
    case 3: config.gpios[0].state_interfaces[0].name = "effort"; break;
    case 4: config.gpios[0].state_interfaces.push_back(config.gpios[0].state_interfaces[0]); break;
    case 5: config.gpios[0].command_interfaces[0].name = "position.x"; break;
    case 6: config.gpios[0].command_interfaces.push_back(config.gpios[0].command_interfaces[0]); break;
    case 7: config.gpios[0].name.clear(); break;
    case 8: config.gpios[0].state_interfaces.clear(); config.gpios[0].command_interfaces.clear(); break;
    case 9: config.hardware_parameters["feedback_timeout"] = "nan"; break;
    case 10: config.hardware_parameters["feedback_timeout"] = "0"; break;
    case 11: config.hardware_parameters["activation_timeout"] = "inf"; break;
    case 12: config.hardware_parameters["activation_timeout"] = "-1"; break;
    case 13: config.joints.push_back(config.joints.front()); break;
    case 14: config.joints[0].state_interfaces.pop_back(); break;
    case 15: config.joints[0].command_interfaces[0].name = "effort"; break;
    case 16: config.joints[0].command_interfaces.push_back(config.joints[0].command_interfaces[0]); break;
    }
    LiteSimSystem system;
    EXPECT_EQ(system.on_init(config), hardware_interface::CallbackReturn::ERROR) << defect;
  }
  for (const char *invalid : {"not-a-number", "1e9999"}) {
    auto config = info(); config.hardware_parameters["feedback_timeout"] = invalid;
    LiteSimSystem system; EXPECT_EQ(system.on_init(config), hardware_interface::CallbackReturn::ERROR);
  }
}

TEST_F(SystemUnits, ExportsActualStateAndReadOnlyJointsInHardwareOrder) {
  LiteSimSystem system; initialize(system);
  auto states = system.export_state_interfaces();
  auto commands = system.export_command_interfaces();
  ASSERT_EQ(states.size(), 15U); ASSERT_EQ(commands.size(), 6U);
  EXPECT_EQ(states[0].get_name(), "arm/position"); EXPECT_EQ(states[2].get_name(), "mimic/position");
  EXPECT_EQ(commands[0].get_name(), "arm/position"); EXPECT_EQ(commands[1].get_name(), "arm/velocity");
  EXPECT_EQ(commands.back().get_name(), "airframe/angular.z.velocity");
  EXPECT_EQ(read(system), hardware_interface::return_type::OK);
  SystemUnitAccess::receive(system, feedback());
  ASSERT_TRUE(SystemUnitAccess::received(system)); ASSERT_FALSE(SystemUnitAccess::fault(system));
  EXPECT_EQ(read(system), hardware_interface::return_type::OK);
  EXPECT_EQ(states[0].get_value(), 1.5); EXPECT_EQ(states[1].get_value(), .25);
  EXPECT_EQ(states[2].get_value(), -2); EXPECT_EQ(states[3].get_value(), -.5);
  EXPECT_EQ(states[4].get_value(), .1); EXPECT_EQ(states[5].get_value(), .2);
  EXPECT_EQ(states[6].get_value(), .3); EXPECT_EQ(states[7].get_value(), .4);
  EXPECT_EQ(states[8].get_value(), 1); EXPECT_EQ(states[9].get_value(), 2); EXPECT_EQ(states[10].get_value(), 3);
  EXPECT_EQ(states.back().get_value(), 1);
  EXPECT_EQ(write(system), hardware_interface::return_type::OK);  // inactive never emits a command
  EXPECT_EQ(system.on_deactivate(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(system.on_error(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
}

TEST_F(SystemUnits, FeedbackRejectsMalformedLayoutSessionTimeAndFiniteValues) {
  LiteSimSystem system; initialize(system);
  for (unsigned defect = 0; defect < 13; ++defect) {
    ASSERT_EQ(system.on_configure(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
    auto message = feedback();
    switch (defect) {
    case 0: message.run_id.clear(); break;
    case 1: message.operational = false; break;
    case 2: message.state.header.stamp.sec = -1; break;
    case 3: message.state.header.stamp.nanosec = 1000000000; break;
    case 4: message.state.position.pop_back(); break;
    case 5: message.state.velocity.pop_back(); break;
    case 6: message.state.name[2] = "arm"; break;
    case 7: message.state.position[0] = NAN; break;
    case 8: message.state.velocity[0] = INFINITY; break;
    case 9: message.state.name[1] = "missing_arm"; break;
    case 10: message.body_command_supported = false; break;
    case 11: message.body_velocity.linear.z = NAN; break;
    case 12: message.body_pose.orientation.w = 2; break;
    }
    SystemUnitAccess::receive(system, message);
    EXPECT_TRUE(SystemUnitAccess::fault(system)) << defect;
    EXPECT_FALSE(SystemUnitAccess::received(system)) << defect;
  }
  for (unsigned defect = 0; defect < 3; ++defect) {
    system.on_configure(rclcpp_lifecycle::State{});
    SystemUnitAccess::receive(system, feedback()); SystemUnitAccess::active(system, true);
    auto message = feedback();
    if (defect == 0) message.run_id = "a-new-session";
    if (defect == 1) --message.state.header.stamp.sec;
    if (defect == 2) SystemUnitAccess::expire(system);
    SystemUnitAccess::receive(system, message);
    EXPECT_TRUE(SystemUnitAccess::fault(system)) << defect;
  }
}

TEST_F(SystemUnits, JointAndBodyOwnershipSwitchesAreTransactional) {
  LiteSimSystem system; initialize(system);
  const std::string position = "arm/position", velocity = "arm/velocity", body = "airframe/linear.z.velocity";
  EXPECT_EQ(system.prepare_command_mode_switch({position, velocity}, {}), hardware_interface::return_type::ERROR);
  EXPECT_EQ(system.perform_command_mode_switch({position, velocity}, {}), hardware_interface::return_type::ERROR);
  EXPECT_EQ(SystemUnitAccess::mode(system, 0), Command::HOLD);
  EXPECT_EQ(system.prepare_command_mode_switch({velocity, body}, {}), hardware_interface::return_type::OK);
  EXPECT_EQ(SystemUnitAccess::mode(system, 0), Command::HOLD);  // prepare cannot mutate ownership
  EXPECT_EQ(system.perform_command_mode_switch({velocity, body}, {}), hardware_interface::return_type::OK);
  EXPECT_EQ(SystemUnitAccess::mode(system, 0), Command::VELOCITY); EXPECT_TRUE(SystemUnitAccess::reset(system, 0));
  EXPECT_EQ(system.perform_command_mode_switch({velocity}, {}), hardware_interface::return_type::ERROR);
  EXPECT_EQ(system.perform_command_mode_switch({body}, {}), hardware_interface::return_type::ERROR);
  EXPECT_EQ(system.perform_command_mode_switch({position, body}, {velocity, body}), hardware_interface::return_type::OK);
  EXPECT_EQ(SystemUnitAccess::mode(system, 0), Command::POSITION);
  EXPECT_EQ(system.perform_command_mode_switch({}, {position, body}), hardware_interface::return_type::OK);
  EXPECT_EQ(SystemUnitAccess::mode(system, 0), Command::HOLD);
  EXPECT_EQ(system.prepare_command_mode_switch({"another_system/joint/velocity"}, {}), hardware_interface::return_type::OK);
  EXPECT_EQ(system.on_error(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
}

TEST_F(SystemUnits, MissingPeersCannotActivateAndFaultsNeverBecomeHealthyViaLateFeedback) {
  LiteSimSystem system; initialize(system, false);
  EXPECT_EQ(system.export_state_interfaces().size(), 4U);
  EXPECT_EQ(system.export_command_interfaces().size(), 2U);
  EXPECT_EQ(SystemUnitAccess::feedbackPublishers(system), 0U);
  EXPECT_EQ(SystemUnitAccess::commandSubscribers(system), 0U);
  EXPECT_EQ(system.prepare_command_mode_switch({"arm/velocity"}, {}), hardware_interface::return_type::OK);
  EXPECT_EQ(system.on_activate(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::ERROR);
  SystemUnitAccess::receive(system, feedback());
  EXPECT_EQ(system.on_activate(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::ERROR);
  SystemUnitAccess::active(system, true);
  EXPECT_EQ(write(system), hardware_interface::return_type::ERROR);
  EXPECT_EQ(read(system), hardware_interface::return_type::ERROR); EXPECT_TRUE(SystemUnitAccess::fault(system));
  SystemUnitAccess::receive(system, feedback());
  EXPECT_TRUE(SystemUnitAccess::fault(system));
  EXPECT_EQ(system.on_configure(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_FALSE(SystemUnitAccess::fault(system)); EXPECT_FALSE(SystemUnitAccess::received(system));
}

TEST_F(SystemUnits, AdapterCountsGateActivationAndWritesPreserveSessionAndReadonlyPolicy) {
  IsolatedSystem system; initialize(system);
  SystemUnitAccess::deliver(system, feedback());
  system.command_sinks = 0;
  EXPECT_EQ(system.on_activate(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::ERROR);
  system.command_sinks = 1;
  ASSERT_EQ(system.on_activate(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(read(system), hardware_interface::return_type::OK);
  auto states = system.export_state_interfaces(); EXPECT_EQ(states[0].get_value(), 1.5);
  ASSERT_EQ(write(system), hardware_interface::return_type::OK);
  ASSERT_EQ(system.published.size(), 1U);
  EXPECT_EQ(system.published.back().run_id, "unit-session");
  EXPECT_EQ(system.published.back().header.stamp, feedback().state.header.stamp);
  EXPECT_EQ(system.published.back().name, std::vector<std::string>{"arm"});  // no command for mimic
  EXPECT_EQ(system.published.back().mode, std::vector<uint8_t>{Command::HOLD});
  EXPECT_EQ(system.published.back().command, std::vector<double>{0});
  EXPECT_FALSE(system.published.back().body_velocity_control);
  system.feedback_sources = 2;
  EXPECT_EQ(write(system), hardware_interface::return_type::ERROR);  // duplicate source must stop
  EXPECT_TRUE(system.published.back().name.empty());
}

TEST_F(SystemUnits, OwnershipHandoffResetsOnceThenSerializesJointAndBodyCommands) {
  IsolatedSystem system; initialize(system);
  SystemUnitAccess::receive(system, feedback());
  ASSERT_EQ(system.on_activate(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
  auto commands = system.export_command_interfaces();
  const std::vector<std::string> start{"arm/velocity", "airframe/linear.x.velocity", "airframe/linear.y.velocity",
      "airframe/linear.z.velocity", "airframe/angular.z.velocity"};
  ASSERT_EQ(system.perform_command_mode_switch(start, {}), hardware_interface::return_type::OK);
  for (auto &command : commands) command.set_value(NAN);  // old controller's deactivation output
  ASSERT_EQ(write(system), hardware_interface::return_type::OK);
  EXPECT_EQ(system.published.back().command, std::vector<double>{0});
  EXPECT_EQ(system.published.back().body_velocity.linear.x, 0);
  EXPECT_FALSE(SystemUnitAccess::reset(system, 0));
  commands[1].set_value(.25);
  for (size_t i = 2; i < commands.size(); ++i) commands[i].set_value(double(i) / 10);
  ASSERT_EQ(write(system), hardware_interface::return_type::OK);
  EXPECT_EQ(system.published.back().mode, std::vector<uint8_t>{Command::VELOCITY});
  EXPECT_EQ(system.published.back().command, std::vector<double>{.25});
  EXPECT_TRUE(system.published.back().body_velocity_control);
  EXPECT_EQ(system.published.back().body_velocity.linear.x, .2);
  EXPECT_EQ(system.published.back().body_velocity.linear.y, .3);
  EXPECT_EQ(system.published.back().body_velocity.linear.z, .4);
  EXPECT_EQ(system.published.back().body_velocity.angular.z, .5);
  ASSERT_EQ(system.perform_command_mode_switch({"arm/position"}, {"arm/velocity"}), hardware_interface::return_type::OK);
  commands[0].set_value(NAN);
  ASSERT_EQ(write(system), hardware_interface::return_type::OK);
  EXPECT_EQ(system.published.back().command, std::vector<double>{1.5});
  commands[0].set_value(2.5);
  ASSERT_EQ(write(system), hardware_interface::return_type::OK);
  EXPECT_EQ(system.published.back().mode, std::vector<uint8_t>{Command::POSITION});
  EXPECT_EQ(system.published.back().command, std::vector<double>{2.5});
  EXPECT_EQ(system.prepare_command_mode_switch({}, {"arm/velocity"}), hardware_interface::return_type::OK);
}

TEST_F(SystemUnits, NonFiniteJointAndBodyCommandsLatchFaultAndEmitAnEmptyHold) {
  for (unsigned invalid = 0; invalid < 3; ++invalid) {
    IsolatedSystem system; initialize(system);
    SystemUnitAccess::receive(system, feedback());
    ASSERT_EQ(system.on_activate(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
    auto commands = system.export_command_interfaces();
    const std::string claimed = invalid == 0 ? "arm/position" : invalid == 1 ? "arm/velocity" : "airframe/linear.z.velocity";
    ASSERT_EQ(system.perform_command_mode_switch({claimed}, {}), hardware_interface::return_type::OK);
    ASSERT_EQ(write(system), hardware_interface::return_type::OK);  // clear ownership handoff reset
    commands[invalid == 2 ? 4 : invalid].set_value(NAN);
    EXPECT_EQ(write(system), hardware_interface::return_type::ERROR);
    ASSERT_FALSE(system.published.empty()); EXPECT_TRUE(system.published.back().name.empty());
    EXPECT_TRUE(SystemUnitAccess::fault(system));
    EXPECT_EQ(write(system), hardware_interface::return_type::ERROR);
  }
}

TEST_F(SystemUnits, WatchdogCoversInactiveNoSampleFreshAndLateSampleCasesDeterministically) {
  IsolatedSystem system; initialize(system);
  SystemUnitAccess::watchdog(system); EXPECT_FALSE(SystemUnitAccess::fault(system));
  SystemUnitAccess::active(system, true);
  SystemUnitAccess::watchdog(system); EXPECT_FALSE(SystemUnitAccess::fault(system));
  SystemUnitAccess::active(system, false);  // initial feedback arrives before activation
  SystemUnitAccess::receive(system, feedback());
  SystemUnitAccess::active(system, true);
  SystemUnitAccess::watchdog(system); EXPECT_FALSE(SystemUnitAccess::fault(system));
  SystemUnitAccess::expire(system);
  SystemUnitAccess::watchdog(system); EXPECT_TRUE(SystemUnitAccess::fault(system));
  EXPECT_EQ(read(system), hardware_interface::return_type::ERROR);
  EXPECT_EQ(system.on_configure(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
  SystemUnitAccess::receive(system, feedback()); SystemUnitAccess::active(system, true);
  SystemUnitAccess::expire(system);
  EXPECT_EQ(write(system), hardware_interface::return_type::ERROR);  // elapsed guard, without waiting for timer
}

TEST_F(SystemUnits, StateOnlyBodyAndDuplicateJointStatesHaveExplicitValidation) {
  auto config = info(); config.gpios[0].command_interfaces.clear();
  LiteSimSystem system; ASSERT_EQ(system.on_init(config), hardware_interface::CallbackReturn::SUCCESS);
  SystemUnitAccess::stopTransport(system);
  auto message = feedback(); message.body_command_supported = false;
  SystemUnitAccess::receive(system, message); EXPECT_FALSE(SystemUnitAccess::fault(system));
  config = info(); config.gpios[0].state_interfaces.clear();
  LiteSimSystem command_only; ASSERT_EQ(command_only.on_init(config), hardware_interface::CallbackReturn::SUCCESS);
  SystemUnitAccess::stopTransport(command_only);
  config = info(); config.joints[0].state_interfaces.push_back(interface("position"));
  LiteSimSystem duplicate; EXPECT_EQ(duplicate.on_init(config), hardware_interface::CallbackReturn::ERROR);
  LiteSimSystem uninitialized;
  EXPECT_EQ(uninitialized.on_deactivate(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
}
