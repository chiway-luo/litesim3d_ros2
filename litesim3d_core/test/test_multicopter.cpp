#include "litesim3d/sim.hpp"
#include <gtest/gtest.h>
#include <limits>
using namespace litesim3d;
namespace {
Config flightConfig() {
  Scene ignored;
  auto c = Config::load(std::filesystem::path(SIM_RESOURCES) / "config/demo.yaml", ignored);
  c.drive.model = "multicopter"; c.drive.support_links.clear();
  c.terrain.enabled = false; c.odom_error.enabled = false;
  c.initial_pose = pose(Vec3(0, 0, 1)); c.initial_joints.clear();
  c.drive.max_v = 2; c.drive.max_w = 2; c.drive.unlimited_acceleration = true;
  c.timeout = 10; c.lidars.resize(1); c.lidars[0].link = c.base_link;
  c.lidars[0].columns = 8; c.lidars[0].vertical_angles = {0};
  c.imus.clear(); ImuConfig imu; imu.link = c.base_link; c.imus.push_back(imu);
  return c;
}
Model body() {
  Model m; m.root = "base_link"; m.links.insert(m.root);
  Geometry g{"body", m.root, Shape::box(Vec3(.4, .4, .2)), Pose::Identity()};
  m.collision = m.visual = m.sensor = {g};
  m.report = {{"undriven_joints", Json::array()}};
  return m;
}
void run(Simulator &sim, const Command &command, size_t steps) {
  sim.pause(false);
  for (size_t i = 0; i < steps; ++i) { ASSERT_TRUE(sim.command(command)); sim.tick(); }
}
}
TEST(Multicopter, NoWheelJointsAndExplicitModeValidation) {
  auto c = flightConfig(); EXPECT_NO_THROW(Simulator(c, body(), {}));
  c.terrain.enabled = true; EXPECT_THROW(c.validate(), std::runtime_error);
  c.terrain.enabled = false; c.odom_error.enabled = true; EXPECT_THROW(c.validate(), std::runtime_error);
  c.odom_error.enabled = false; c.drive.support_links.insert("base_link");
  EXPECT_THROW(c.validate(), std::runtime_error);
}
TEST(Multicopter, BodyXYZAndYawMatchAnalyticHelix) {
  auto c = flightConfig(); Simulator sim(c, body(), {});
  Command cmd; cmd.v = .3; cmd.lateral = .4; cmd.vertical = .5; cmd.w = 1;
  run(sim, cmd, 200);
  const auto p = sim.snapshot().world_base;
  EXPECT_NEAR(p.translation().x(), .3*std::sin(1)-.4*(1-std::cos(1)), 1e-12);
  EXPECT_NEAR(p.translation().y(), .3*(1-std::cos(1))+.4*std::sin(1), 1e-12);
  EXPECT_NEAR(p.translation().z(), 1.5, 1e-12);
  EXPECT_NEAR(std::atan2(p.linear()(1,0),p.linear()(0,0)),1,1e-12);
  EXPECT_TRUE(sim.snapshot().joints.empty());
  EXPECT_NEAR(sim.snapshot().odom_base.translation().z(),.5,1e-12);
  const auto journal=sim.manifest()["inputs"];
  EXPECT_FALSE(journal.empty());
  EXPECT_DOUBLE_EQ(journal.back()["vertical"],.5);
}
TEST(Multicopter, InitialYawRotatesTranslationAndRejectsRollPitch) {
  auto c=flightConfig(); c.initial_pose=pose(Vec3(0,0,1),Vec3(0,0,pi/2));
  Simulator sim(c,body(),{}); Command cmd;cmd.v=1;run(sim,cmd,200);
  EXPECT_NEAR(sim.snapshot().world_base.translation().x(),0,1e-12);
  EXPECT_NEAR(sim.snapshot().world_base.translation().y(),1,1e-12);
  cmd.roll_rate=.1;EXPECT_FALSE(sim.command(cmd));cmd.roll_rate=0;cmd.pitch_rate=.1;
  EXPECT_FALSE(sim.command(cmd));cmd.pitch_rate=0;cmd.vertical=std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(sim.command(cmd));
}
TEST(Multicopter, VectorSpeedAndAccelerationLimitsIncludeDiagonalMotion) {
  auto c=flightConfig();c.drive.unlimited_acceleration=false;c.drive.acceleration=1;
  Simulator sim(c,body(),{});Command cmd;cmd.v=20;cmd.lateral=20;cmd.vertical=20;
  Vec3 previous=Vec3::Zero();sim.pause(false);
  for(int i=0;i<500;++i) {
    ASSERT_TRUE(sim.command(cmd));sim.tick();const auto v=sim.snapshot().linearVelocity();
    EXPECT_LE(v.norm(),2+1e-12);EXPECT_LE((v-previous).norm(),.005+1e-12);previous=v;
  }
  EXPECT_NEAR(previous.norm(),2,1e-12);
}
TEST(Multicopter, TimeoutBrakesAllAxesAndHoldsAltitude) {
  auto c=flightConfig();c.timeout=.1;c.drive.unlimited_acceleration=false;
  Simulator sim(c,body(),{});Command cmd;cmd.v=.4;cmd.lateral=.3;cmd.vertical=.5;
  run(sim,cmd,100);
  for(int i=0;i<240;++i)sim.tick();
  EXPECT_LT(sim.snapshot().linearVelocity().norm(),1e-12);
  const auto before=sim.snapshot().world_base;
  for(int i=0;i<100;++i)sim.tick();
  EXPECT_TRUE(before.matrix()==sim.snapshot().world_base.matrix());
}
TEST(Multicopter, ThinCeilingCannotBeTunneledThrough) {
  auto c=flightConfig();c.drive.max_v=1000;c.step_ns=100000000;
  Scene scene;scene.collision={{"ceiling","",Shape::box(Vec3(10,10,.002)),pose(Vec3(0,0,2))}};
  Simulator sim(c,body(),scene);Command cmd;cmd.vertical=500;run(sim,cmd,2);
  EXPECT_LT(sim.snapshot().world_base.translation().z(),1.9);
  EXPECT_GT(sim.snapshot().world_base.translation().z(),1.8);
  EXPECT_EQ(sim.snapshot().linearVelocity().norm(),0);EXPECT_GT(sim.contactCount(),0U);
}
TEST(Multicopter, SideWallAndFloorStopAllVelocityComponents) {
  for (bool floor : {false,true}) {
    auto c=flightConfig();c.drive.max_v=100;
    Scene scene;scene.collision={{"obstacle","",Shape::box(floor?Vec3(10,10,.002):Vec3(10,.002,10)),
        pose(floor?Vec3(0,0,0):Vec3(0,1,1))}};
    Simulator sim(c,body(),scene);Command cmd;cmd.lateral=floor?0:50;cmd.vertical=floor?-50:1;
    run(sim,cmd,10);
    EXPECT_GT(sim.contactCount(),0U);EXPECT_EQ(sim.snapshot().linearVelocity().norm(),0);
    if(floor)EXPECT_GT(sim.snapshot().world_base.translation().z(),.1);
    else EXPECT_LT(sim.snapshot().world_base.translation().y(),.8);
  }
}
TEST(Multicopter, ImuIncludesVerticalAccelerationAndLateralCentripetalTerm) {
  auto c=flightConfig();c.drive.unlimited_acceleration=false;c.drive.acceleration=1;
  Simulator sim(c,body(),{});Command cmd;cmd.vertical=1;sim.pause(false);ASSERT_TRUE(sim.command(cmd));
  auto events=sim.tick();ASSERT_FALSE(events.back().imus.empty());
  EXPECT_NEAR(events.back().imus[0].acceleration.z(),10.81,1e-10);
  c.drive.unlimited_acceleration=true;Simulator circling(c,body(),{});cmd={};cmd.lateral=1;cmd.w=1;
  circling.pause(false);ASSERT_TRUE(circling.command(cmd));events=circling.tick();
  const auto imu=events.back().imus[0];
  EXPECT_NEAR(imu.acceleration.x(),-1,1e-10);EXPECT_NEAR(imu.acceleration.y(),0,1e-10);
  EXPECT_NEAR(imu.angular_velocity.z(),1,1e-10);
}

TEST(Multicopter, VerticalClosingSpeedStopsBeforeMovingObstacleOverlap) {
  auto c=flightConfig();c.drive.max_v=100;
  Scene scene;
  Geometry obstacle{"descending", "", Shape::box(Vec3(1,1,.2)), Pose::Identity()};
  obstacle.motion=std::make_shared<Motion>();
  obstacle.motion->initial=pose(Vec3(0,0,1.8));
  obstacle.motion->velocity=Vec3(0,0,-50);
  scene.collision={obstacle};
  Simulator sim(c,body(),scene);Command cmd;cmd.vertical=100;run(sim,cmd,1);
  // In 5 ms the closing distance would be .75 m, exceeding the .6 m gap.
  EXPECT_NEAR(sim.snapshot().world_base.translation().z(),1,1e-12);
  EXPECT_EQ(sim.snapshot().linearVelocity().norm(),0);
  EXPECT_EQ(sim.scene().collision[0].motion->frozen_at,1U);
  EXPECT_GT(sim.contactCount(),0U);
}

TEST(MulticopterControl, BodyCommandsMatchLegacyMotionAndStopOnTimeout) {
  auto legacy_config = flightConfig(); legacy_config.timeout = .1;
  auto control_config = legacy_config; control_config.control_mode = "ros2_control";
  Simulator legacy(legacy_config, body(), {}), control(control_config, body(), {});
  legacy.pause(false); control.pause(false);
  Command twist; twist.v = .3; twist.lateral = .2; twist.vertical = .4; twist.w = .2;
  JointCommand command; command.body_velocity = plugins::Twist{.3, .2, .4, .2};
  EXPECT_FALSE(control.command(twist)); EXPECT_FALSE(legacy.jointCommand(command));
  for (int i = 0; i < 200; ++i) {
    ASSERT_TRUE(legacy.command(twist)); ASSERT_TRUE(control.jointCommand(command));
    legacy.tick(); control.tick();
    EXPECT_TRUE(legacy.snapshot().world_base.matrix().isApprox(control.snapshot().world_base.matrix(), 1e-12));
  }
  EXPECT_TRUE(control.manifest()["inputs"].back().contains("body_velocity"));
  for (int i = 0; i < 100; ++i) control.tick();
  EXPECT_NEAR(control.snapshot().linearVelocity().norm(), 0, 1e-12);
  EXPECT_NEAR(control.snapshot().w, 0, 1e-12);
  const auto stopped = control.snapshot().world_base;
  for (int i = 0; i < 30; ++i) control.tick();
  EXPECT_TRUE(stopped.matrix() == control.snapshot().world_base.matrix());
}
TEST(MulticopterControl, RejectsInvalidCommandsAndEmptyPacketHolds) {
  auto c = flightConfig(); c.control_mode = "ros2_control"; Simulator sim(c, body(), {}); sim.pause(false);
  JointCommand command; command.body_velocity = plugins::Twist{0, 0, .5, 0};
  ASSERT_TRUE(sim.jointCommand(command)); sim.tick(); EXPECT_GT(sim.snapshot().vertical, 0);
  command.body_velocity->vz = NAN; EXPECT_FALSE(sim.jointCommand(command));
  command.body_velocity->vz = .5; command.stamp = sim.snapshot().stamp + 1; EXPECT_FALSE(sim.jointCommand(command));
  command.stamp.reset(); command.joints["missing"] = {JointCommandMode::Velocity, 1}; EXPECT_FALSE(sim.jointCommand(command));
  ASSERT_TRUE(sim.jointCommand(JointCommand{})); sim.tick(); EXPECT_DOUBLE_EQ(sim.snapshot().vertical, 0);
  sim.fault("test"); EXPECT_FALSE(sim.jointCommand(JointCommand{}));
}
TEST(MulticopterControl, CollisionFeedbackReflectsAcceptedHeightAndVelocity) {
  auto c = flightConfig(); c.control_mode = "ros2_control"; c.drive.max_v = 100;
  Scene scene; scene.collision = {{"floor", "", Shape::box(Vec3(10,10,.002)), pose(Vec3(0,0,0))}};
  Simulator sim(c, body(), scene); sim.pause(false);
  JointCommand command; command.body_velocity = plugins::Twist{0, 0, -50, 0};
  for (int i = 0; i < 20; ++i) { ASSERT_TRUE(sim.jointCommand(command)); sim.tick(); }
  EXPECT_GT(sim.contactCount(), 0U); EXPECT_GT(sim.snapshot().world_base.translation().z(), .1);
  EXPECT_LT(sim.snapshot().world_base.translation().z(), .11); EXPECT_DOUBLE_EQ(sim.snapshot().vertical, 0);
}
TEST(MulticopterControl, IndependentJointMovesWithBodyAndRespectsLimits) {
  auto c = flightConfig(); c.control_mode = "ros2_control";
  auto model = body();
  Joint joint; joint.name = "gimbal"; joint.type = "revolute";
  joint.parent = "base_link"; joint.child = "camera"; joint.axis = Vec3::UnitY();
  joint.lower = -.5; joint.upper = .5; joint.velocity = .2;
  model.links.insert(joint.child); model.joints[joint.name] = joint; model.order.push_back(joint.name);
  Simulator sim(c, model, {}); sim.pause(false);
  JointCommand command; command.body_velocity = plugins::Twist{0,0,.3,0};
  command.joints["gimbal"] = {JointCommandMode::Position, .4};
  for (int i = 0; i < 200; ++i) { ASSERT_TRUE(sim.jointCommand(command)); sim.tick(); }
  EXPECT_NEAR(sim.snapshot().world_base.translation().z(), 1.3, 1e-10);
  EXPECT_NEAR(sim.snapshot().joints.at("gimbal"), .2, 1e-10);
  EXPECT_NEAR(sim.snapshot().joint_velocities.at("gimbal"), .2, 1e-10);
  command.joints["gimbal"].value = .6; EXPECT_FALSE(sim.jointCommand(command));
  ASSERT_TRUE(sim.jointCommand({})); sim.tick();
  EXPECT_DOUBLE_EQ(sim.snapshot().joint_velocities.at("gimbal"), 0);
}
