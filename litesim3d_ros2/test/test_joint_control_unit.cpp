// Unit boundary: JointControl transport-adapter validation/serialization only.
// The Simulator is an inert, in-memory session metadata fixture: no physics tick,
// command execution, executor, publisher peer or service is used by these tests.
#include "../src/joint_control.hpp"
#include <gtest/gtest.h>

namespace litesim3d_ros {
struct JointControlUnitAccess {
  static void deliver(JointControl &control, const JointControl::Command &message) {
    std::shared_ptr<void> erased = std::make_shared<JointControl::Command>(message);
    control.command_->handle_message(erased, rclcpp::MessageInfo{});
  }
  static JointControl::Feedback feedback(const std::string &run_id, litesim3d::State operational,
      bool body_command_supported, const sensor_msgs::msg::JointState &state,
      const litesim3d::Snapshot &snapshot) {
    return JointControl::makeFeedback(run_id, operational, body_command_supported, state, snapshot);
  }
};
}
namespace {
using litesim3d_ros::JointControl;
using Access = litesim3d_ros::JointControlUnitAccess;
std::unique_ptr<litesim3d::Simulator> metadata() {
  litesim3d::Config config; config.drive.model = "multicopter";
  config.drive.support_links.clear(); config.control_mode = "ros2_control";
  litesim3d::Model model; model.root = "base_link"; model.links.insert("base_link");
  model.collision.push_back({"fixture", "base_link", litesim3d::Shape::box({.1, .1, .1})});
  return std::make_unique<litesim3d::Simulator>(config, model, litesim3d::Scene{});
}
class JointControlUnits : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void SetUp() override {
    sim = metadata(); node = std::make_shared<rclcpp::Node>("joint_adapter_unit");
    control = std::make_unique<JointControl>(*node, *sim, [&](const litesim3d::JointCommand &command) {
      accepted.push_back(command); return accept;
    }, "unit/");
  }
  JointControl::Command command() {
    JointControl::Command message; message.run_id = sim->run_id;
    message.header.stamp.sec = 3; message.header.stamp.nanosec = 4;
    message.name = {"joint_b", "joint_a"};
    message.mode = {JointControl::Command::POSITION, JointControl::Command::VELOCITY};
    message.command = {1.5, -.25}; return message;
  }
  std::unique_ptr<litesim3d::Simulator> sim;
  rclcpp::Node::SharedPtr node;
  bool accept = true;
  std::vector<litesim3d::JointCommand> accepted;
  std::unique_ptr<JointControl> control;
};
}

TEST_F(JointControlUnits, ValidCommandsPreserveModesNamesAndExactNanosecondStamp) {
  auto message = command(); Access::deliver(*control, message);
  ASSERT_EQ(accepted.size(), 1U); const auto &result = accepted.front();
  ASSERT_TRUE(result.stamp.has_value()); EXPECT_EQ(*result.stamp, 3000000004U);
  ASSERT_EQ(result.joints.size(), 2U);
  EXPECT_EQ(result.joints.at("joint_b").mode, litesim3d::JointCommandMode::Position);
  EXPECT_EQ(result.joints.at("joint_b").value, 1.5);
  EXPECT_EQ(result.joints.at("joint_a").mode, litesim3d::JointCommandMode::Velocity);
  EXPECT_EQ(result.joints.at("joint_a").value, -.25);
  EXPECT_FALSE(result.body_velocity.has_value());
  EXPECT_EQ(sim->stepIndex(), 0U);
  EXPECT_EQ(sim->snapshot().stamp, 1U);
  // The publishing adapter is a thin call to the independently asserted pure
  // conversion below; no subscriber or delivery claim is involved here.
  EXPECT_NO_THROW(control->publish(sensor_msgs::msg::JointState{}, sim->snapshot()));
}

TEST_F(JointControlUnits, LayoutSessionTimestampAndDuplicatesAreRejectedBeforeAcceptor) {
  for (int defect = 0; defect < 6; ++defect) {
    auto message = command();
    if (defect == 0) message.run_id = "previous-session";
    if (defect == 1) message.header.stamp.sec = -1;
    if (defect == 2) message.header.stamp.nanosec = 1000000000;
    if (defect == 3) message.mode.pop_back();
    if (defect == 4) message.command.pop_back();
    if (defect == 5) message.name[1] = message.name[0];
    Access::deliver(*control, message); EXPECT_TRUE(accepted.empty()) << defect;
  }
}

TEST_F(JointControlUnits, BodyVelocityIsOptionalAndDisallowsUnsupportedAngularAxes) {
  for (int angular_axis = 0; angular_axis < 2; ++angular_axis) {
    auto invalid = command(); invalid.body_velocity_control = true;
    if (angular_axis == 0) invalid.body_velocity.angular.x = .1;
    else invalid.body_velocity.angular.y = .1;
    Access::deliver(*control, invalid); EXPECT_TRUE(accepted.empty());
  }
  auto message = command(); message.name.clear(); message.mode.clear(); message.command.clear();
  message.body_velocity_control = true; message.body_velocity.linear.x = .1;
  message.body_velocity.linear.y = .2; message.body_velocity.linear.z = .3; message.body_velocity.angular.z = -.4;
  Access::deliver(*control, message);
  ASSERT_EQ(accepted.size(), 1U); ASSERT_TRUE(accepted[0].body_velocity.has_value());
  const auto value = *accepted[0].body_velocity;
  EXPECT_EQ(value.vx, .1); EXPECT_EQ(value.vy, .2); EXPECT_EQ(value.vz, .3); EXPECT_EQ(value.wz, -.4);
  EXPECT_TRUE(accepted[0].joints.empty());
  accept = false; Access::deliver(*control, message); EXPECT_EQ(accepted.size(), 2U);
}

TEST(JointFeedbackUnits, SerializationPreservesMetadataPoseAndBodyTwistForEveryState) {
  sensor_msgs::msg::JointState state;
  state.header.frame_id = "joint_frame"; state.header.stamp.sec = 4; state.header.stamp.nanosec = 5;
  state.name = {"joint"}; state.position = {1.2}; state.velocity = {-.4}; state.effort = {0};
  litesim3d::Snapshot snapshot; snapshot.world_base = litesim3d::pose({1, 2, 3}, {.1, -.2, .3});
  snapshot.v = .4; snapshot.lateral = -.5; snapshot.vertical = .6; snapshot.w = -.7;
  const Eigen::Quaterniond quaternion(snapshot.world_base.linear());
  for (auto operational : {litesim3d::State::Ready, litesim3d::State::Running, litesim3d::State::Paused,
                          litesim3d::State::Fault, litesim3d::State::Exited}) for (bool body : {false, true}) {
    const auto result = Access::feedback("unit-run", operational, body, state, snapshot);
    EXPECT_EQ(result.run_id, "unit-run"); EXPECT_EQ(result.state, state);
    EXPECT_EQ(result.operational, operational != litesim3d::State::Fault && operational != litesim3d::State::Exited);
    EXPECT_EQ(result.body_command_supported, body);
    EXPECT_EQ(result.body_pose.position.x, 1); EXPECT_EQ(result.body_pose.position.y, 2); EXPECT_EQ(result.body_pose.position.z, 3);
    EXPECT_EQ(result.body_pose.orientation.x, quaternion.x()); EXPECT_EQ(result.body_pose.orientation.y, quaternion.y());
    EXPECT_EQ(result.body_pose.orientation.z, quaternion.z()); EXPECT_EQ(result.body_pose.orientation.w, quaternion.w());
    EXPECT_EQ(result.body_velocity.linear.x, .4); EXPECT_EQ(result.body_velocity.linear.y, -.5);
    EXPECT_EQ(result.body_velocity.linear.z, .6); EXPECT_EQ(result.body_velocity.angular.z, -.7);
    EXPECT_EQ(result.body_velocity.angular.x, 0); EXPECT_EQ(result.body_velocity.angular.y, 0);
  }
}
