#include "litesim3d/sim.hpp"
#include <gtest/gtest.h>

using namespace litesim3d;
namespace {
Config wheelConfig(bool keep_driving = true) {
  Scene ignored;
  auto c = Config::load(std::filesystem::path(SIM_RESOURCES) / "config/demo.yaml", ignored);
  c.drive.continue_on_collision = keep_driving;
  c.drive.unlimited_acceleration = true;
  c.timeout = 60;
  auto &l = c.lidars[0];
  l.columns = 1; l.vertical_angles = {0}; l.horizontal_start = 0;
  l.horizontal_end = .01; l.filter = "robot";
  c.imus = {ImuConfig{}};
  return c;
}
Scene wallScene(double x = .8, double thickness = .1, bool terrain = false) {
  Scene s;
  s.visual = s.collision = s.sensor = {
      Geometry{"wall", "", Shape::box({thickness, 10, 3}), pose({x, 0, 1})}};
  if (terrain) {
    Geometry floor{"floor", "", Shape::box({20, 20, .2}), pose({0, 0, -.1})};
    floor.support = true;
    s.visual.push_back(floor); s.collision.push_back(floor); s.sensor.push_back(floor);
  }
  s.rebuild();
  return s;
}
Simulator wheeled(Config c = wheelConfig(), Scene s = wallScene()) {
  return Simulator(c, Model::load(c.urdf, c.model_options), s);
}
std::vector<Event> step(Simulator &s, uint32_t count) {
  s.pause(true);
  std::vector<Event> events;
  EXPECT_TRUE(s.singleStep(std::to_string(s.stepIndex()), count, events).success);
  return events;
}
} // namespace

TEST(CollisionOdometry, ToggleSeparatesWheelEncodersFromBlockedTruth) {
  for (bool enabled : {false, true}) {
    auto s = wheeled(wheelConfig(enabled));
    ASSERT_TRUE(s.command({.8, 0}));
    step(s, 400);
    const auto &p = s.snapshot();
    EXPECT_GT(s.contactCount(), 0u);
    EXPECT_NEAR(p.world_base.translation().x(), .35, 2e-5);
    EXPECT_DOUBLE_EQ(p.v, 0);
    const double distance = enabled ? 1.6 : p.world_base.translation().x();
    EXPECT_NEAR(p.odom_base.translation().x(), distance, 1e-9);
    EXPECT_NEAR(p.joints.at("left_wheel_joint") * .1, distance, 1e-9);
    EXPECT_NEAR(p.joints.at("right_wheel_joint") * .1, distance, 1e-9);
    EXPECT_NEAR(p.odom_v, enabled ? .8 : 0, 1e-12);
    EXPECT_NEAR(p.joint_velocities.at("left_wheel_joint"), enabled ? 8 : 0, 1e-12);
    EXPECT_DOUBLE_EQ(s.scene().collision[0].at(p.stamp).translation().x(), .8);
    EXPECT_EQ(s.manifest()["config"]["drive"]["continue_on_collision"], enabled);
  }
}

TEST(CollisionOdometry, ReverseEscapesContactMarginWithAndWithoutTerrain) {
  for (bool terrain : {false, true}) for (bool enabled : {false, true}) {
    auto c = wheelConfig(enabled); c.terrain.enabled = terrain;
    auto s = wheeled(c, wallScene(.8, .1, terrain));
    ASSERT_TRUE(s.command({.8, 0})); step(s, 200);
    const auto before = s.snapshot();
    ASSERT_GT(s.contactCount(), 0u);
    ASSERT_TRUE(s.command({-.5, 0})); step(s, 100);
    EXPECT_NEAR(s.snapshot().world_base.translation().x(), before.world_base.translation().x() - .25, 1e-7);
    EXPECT_NEAR(s.snapshot().odom_base.translation().x(), before.odom_base.translation().x() - .25, 1e-7);
    EXPECT_DOUBLE_EQ(s.snapshot().v, -.5);
  }
}

TEST(CollisionOdometry, AccelerationAndTimeoutApplyToWheelsWhileBlocked) {
  for (bool noisy : {false, true}) {
    auto c = wheelConfig(); c.step_ns = 10000000; c.timeout = 1.2;
    c.odom_error.enabled = noisy;
    c.drive.unlimited_acceleration = false; c.drive.acceleration = c.drive.deceleration = 1;
    auto s = wheeled(c, wallScene(.55));
    ASSERT_TRUE(s.command({.8, 0})); step(s, 20);
    EXPECT_NEAR(s.snapshot().odom_v, .2, 1e-12);
    step(s, 80);
    EXPECT_NEAR(s.snapshot().world_base.translation().x(), .1, 2e-5);
    EXPECT_NEAR(s.snapshot().odom_base.translation().x(), .48, 1e-9);
    EXPECT_NEAR(s.snapshot().odom_v, .8, 1e-12);
    step(s, 110);
    EXPECT_NEAR(s.snapshot().odom_base.translation().x(), .96, 1e-9);
    EXPECT_NEAR(s.snapshot().odom_v, 0, 1e-12);
    EXPECT_NEAR(s.snapshot().joint_velocities.at("left_wheel_joint"), 0, 1e-12);
    const auto stopped = s.snapshot(); step(s, 100);
    EXPECT_TRUE(s.snapshot().odom_base.matrix().isApprox(stopped.odom_base.matrix(), 1e-12));
  }
}

TEST(CollisionOdometry, TurningAndNoisyOdometryIntegrateEncoderMotion) {
  for (bool noisy : {false, true}) {
    auto c = wheelConfig();
    c.odom_error.enabled = noisy;
    c.odom_error.linear_scale = noisy ? 1.2 : 1;
    auto s = wheeled(c);
    ASSERT_TRUE(s.command({.8, .2})); step(s, 400);
    ASSERT_GT(s.contactCount(), 0u);
    const Pose expected = integrate(Pose::Identity(), .8 * c.odom_error.linear_scale, .2, 2);
    EXPECT_TRUE(s.snapshot().odom_base.matrix().isApprox(expected.matrix(), 1e-9));
    EXPECT_NEAR(s.snapshot().joints.at("left_wheel_joint"), 15, 1e-9);
    EXPECT_NEAR(s.snapshot().joints.at("right_wheel_joint"), 17, 1e-9);
    EXPECT_DOUBLE_EQ(s.snapshot().v, 0);
    EXPECT_DOUBLE_EQ(s.snapshot().w, 0);
  }
}

TEST(CollisionOdometry, SensorsStayAtBlockedPoseWhileEncodersAccumulate) {
  auto s = wheeled();
  ASSERT_TRUE(s.command({.8, .2})); step(s, 200);
  const Pose blocked = s.snapshot().world_base;
  const auto events = step(s, 100);
  size_t imus = 0, clouds = 0;
  std::vector<uint8_t> cloud;
  for (const auto &e : events) {
    EXPECT_TRUE(e.snapshot.world_base.matrix().isApprox(blocked.matrix(), 1e-10));
    for (const auto &imu : e.imus) {
      ++imus;
      EXPECT_TRUE(imu.angular_velocity.isZero(1e-12));
      EXPECT_TRUE(imu.acceleration.isApprox(Vec3(0, 0, 9.80665), 1e-12));
    }
    for (const auto &points : e.clouds) {
      if (clouds++) EXPECT_EQ(points.data, cloud);
      cloud = points.data;
    }
  }
  EXPECT_GT(imus, 0u); EXPECT_GT(clouds, 1u);
  EXPECT_GT(s.snapshot().odom_base.translation().x(), blocked.translation().x() + .5);
}

TEST(CollisionOdometry, ThinObstacleCannotBeCrossedAtHighSpeed) {
  auto c = wheelConfig(); c.drive.max_v = 99; c.step_ns = 100000000;
  // Stay within the URDF's 1000 rad/s wheel limit, including angular limits.
  auto s = wheeled(c, wallScene(1, .000001));
  ASSERT_TRUE(s.command({99, 0})); step(s, 20);
  EXPECT_NEAR(s.snapshot().world_base.translation().x(), .5999995, 2e-5);
  EXPECT_NEAR(s.snapshot().odom_base.translation().x(), 198, 1e-7);
}

TEST(CollisionOdometry, AckermannKeepsSteeringLimitsWhileBlocked) {
  auto c = wheelConfig(); c.drive.model = "ackermann";
  auto s = wheeled(c);
  ASSERT_TRUE(s.command({.8, 2})); step(s, 400);
  const double angular = .8 * std::tan(c.drive.max_steering) / c.drive.wheelbase;
  EXPECT_GT(s.contactCount(), 0u);
  EXPECT_NEAR(s.snapshot().odom_w, angular, 1e-12);
  EXPECT_TRUE(s.snapshot().odom_base.matrix().isApprox(integrate(Pose::Identity(), .8, angular, 2).matrix(), 1e-9));
}

TEST(CollisionOdometry, MultiRobotCollisionPreservesWheelMotion) {
  auto c = wheelConfig();
  World world(c, Model::load(c.urdf), {}, {{"a", pose({-1, 0, 0})}, {"b", pose({1, 0, 0}, {0, 0, pi})}});
  ASSERT_TRUE(world.command(0, {.8, 0})); ASSERT_TRUE(world.command(1, {.8, 0}));
  std::vector<std::vector<Event>> events;
  ASSERT_TRUE(world.singleStep("collision", 400, events).success);
  for (size_t i = 0; i < 2; ++i) {
    EXPECT_GT(world.robot(i).contactCount(), 0u);
    EXPECT_DOUBLE_EQ(world.robot(i).snapshot().v, 0);
    EXPECT_NEAR(world.robot(i).snapshot().odom_base.translation().x(), 1.6, 1e-9);
    EXPECT_NEAR(world.robot(i).snapshot().joint_velocities.at("left_wheel_joint"), 8, 1e-12);
  }
  EXPECT_GE(world.robot(1).snapshot().world_base.translation().x() - world.robot(0).snapshot().world_base.translation().x(), .8);
}

TEST(CollisionOdometry, FreshSessionClearsDriveStateAndDroneRejectsWheelOption) {
  auto s = wheeled(); ASSERT_TRUE(s.command({.8, 0})); step(s, 400);
  auto fresh = wheeled(s.config(), s.scene()); step(fresh, 20);
  EXPECT_TRUE(fresh.snapshot().odom_base.matrix().isApprox(Pose::Identity().matrix()));
  EXPECT_DOUBLE_EQ(fresh.snapshot().joints.at("left_wheel_joint"), 0);
  auto c = wheelConfig(); c.drive.model = "multicopter"; c.drive.support_links.clear();
  EXPECT_THROW(c.validate(), std::runtime_error);
}
