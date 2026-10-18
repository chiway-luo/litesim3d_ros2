#include "litesim3d/sim.hpp"
#include <gtest/gtest.h>
using namespace litesim3d;
namespace {
Config config() {
  Scene unused;
  auto c = Config::load(std::filesystem::path(SIM_RESOURCES) / "config/demo.yaml", unused);
  c.step_ns = 100000000;
  c.odom_rate = 200;
  c.lidars[0].columns = 16;
  c.lidars[0].vertical_angles = {0};
  c.lidars[0].scan_mode = "rolling";
  c.drive.unlimited_acceleration = true;
  c.timeout = 10;
  return c;
}
Scene scene(Geometry geometry) {
  Scene s;
  s.visual = s.collision = s.sensor = {geometry};
  s.rebuild();
  return s;
}
Scene platform(double width, double length = 10, double x = 0) {
  return scene({"platform", "", Shape::box({length, width, .2}), pose({x, 0, -.1}), .5, true});
}
void sameEvents(const std::vector<Event> &a, const std::vector<Event> &b) {
  ASSERT_EQ(a.size(), b.size());
  for (size_t i = 0; i < a.size(); ++i) {
    EXPECT_EQ(a[i].snapshot.stamp, b[i].snapshot.stamp);
    EXPECT_TRUE(a[i].snapshot.world_base.isApprox(b[i].snapshot.world_base, 1e-12));
    EXPECT_EQ(a[i].snapshot.joints, b[i].snapshot.joints);
    ASSERT_EQ(a[i].clouds.size(), b[i].clouds.size());
    for (size_t j = 0; j < a[i].clouds.size(); ++j) {
      EXPECT_EQ(a[i].clouds[j].stamp, b[i].clouds[j].stamp);
      EXPECT_EQ(a[i].clouds[j].data, b[i].clouds[j].data);
    }
  }
}
}

TEST(ReviewWorld, DynamicStopAndPointCloudsAreIndependentOfRobotOrder) {
  auto c = config();
  Geometry wall{"moving", "", Shape::box({.1, 100, 2})};
  wall.motion = std::make_shared<Motion>();
  wall.motion->initial = pose({.85, 0, .5});
  wall.motion->velocity = {-100, 0, 0};
  // Different x exposes retroactive freezing; equal x checks that all robots
  // independently report the same simultaneous contact before it is committed.
  for (double far_x : {-1., 0.}) {
    std::vector<RobotSpec> specs{{"near", pose()}, {"far", pose({far_x, 10, 0})}};
    World a(c, Model::load(c.urdf), scene(wall), specs);
    std::reverse(specs.begin(), specs.end());
    World b(c, Model::load(c.urdf), scene(wall), specs);
    a.pause(false); b.pause(false);
    for (size_t i = 0; i < 2; ++i) { a.command(i, {1, 0}); b.command(i, {1, 0}); }
    for (int step = 0; step < 3; ++step) {
      auto x = a.tick(), y = b.tick();
      for (size_t i = 0; i < 2; ++i) {
        sameEvents(x[i], y[1 - i]);
        EXPECT_EQ(a.robot(i).contactCount(), b.robot(1 - i).contactCount());
        EXPECT_EQ(a.robot(i).scene().collision[0].motion->frozen_at, 1u);
        ASSERT_EQ(a.robot(i).contacts().size(), b.robot(1 - i).contacts().size());
        for (size_t k = 0; k < a.robot(i).contacts().size(); ++k) {
          EXPECT_EQ(a.robot(i).contacts()[k].stamp, b.robot(1 - i).contacts()[k].stamp);
          EXPECT_EQ(a.robot(i).contacts()[k].entity, b.robot(1 - i).contacts()[k].entity);
        }
      }
      if (step == 0) {
        EXPECT_NEAR(a.robot(1).snapshot().world_base.translation().x(), far_x < 0 ? -.9 : 0, 1e-12);
        EXPECT_EQ(a.robot(1).contactCount(), far_x < 0 ? 0u : 1u);
      }
    }
    EXPECT_EQ(wall.motion->frozen_at, UINT64_MAX);
  }
}

TEST(ReviewWorld, SingleRobotSchedulingMatchesStandalone) {
  auto c = config();
  c.drive.unlimited_acceleration = false;
  c.odom_rate = 30;
  c.timeout = .037;
  c.lidars[0].rate = 13;
  c.imus.push_back(ImuConfig{});
  auto model = Model::load(c.urdf);
  Simulator alone(c, model, {});
  World world(c, model, {}, {{c.robot_id, pose()}});
  alone.pause(false); world.pause(false);
  for (int step = 0; step < 10; ++step) {
    if (step % 3 == 0) { alone.command({.5, .2}); world.command(0, {.5, .2}); }
    sameEvents(alone.tick(), world.tick()[0]);
    EXPECT_EQ(alone.snapshot().stamp, world.robot(0).snapshot().stamp);
    EXPECT_TRUE(alone.snapshot().world_base.isApprox(world.robot(0).snapshot().world_base, 1e-12));
  }
}

TEST(ReviewWorld, CallbackFailureMakesFaultTerminal) {
  for (bool standard_exception : {true, false}) {
    auto c = config();
    World world(c, Model::load(c.urdf), {}, {{"a", pose()}, {"b", pose({0, 3, 0})}});
    world.pause(false);
    world.command(0, {.5, 0});
    EXPECT_ANY_THROW(world.tick([&](size_t, const std::vector<Event> &) {
      if (standard_exception) throw std::runtime_error("sink failed");
      throw 7;
    }));
    const auto before = world.robot(0).snapshot();
    size_t delivered = 0;
    for (int i = 0; i < 3; ++i) {
      const auto batch = world.tick([&](size_t, const std::vector<Event> &) { ++delivered; });
      for (const auto &events : batch) EXPECT_TRUE(events.empty());
    }
    EXPECT_EQ(delivered, 0u);
    EXPECT_EQ(world.robot(0).snapshot().stamp, before.stamp);
    EXPECT_TRUE(world.robot(0).snapshot().world_base.isApprox(before.world_base));
    EXPECT_EQ(world.robot(0).snapshot().joints, before.joints);
    for (size_t i = 0; i < world.size(); ++i) {
      EXPECT_EQ(world.robot(i).state(), State::Fault);
      EXPECT_FALSE(world.command(i, {1, 0}));
    }
    std::vector<std::vector<Event>> output;
    EXPECT_FALSE(world.singleStep("after-failure", 1, output).success);
    EXPECT_THROW(world.pause(false), std::runtime_error);
  }
}

TEST(ReviewWorld, NoninteractingCommandsDoNotChangeOdometryNoiseOrTrajectory) {
  auto c = config();
  c.timeout = .037; c.odom_rate = 30; c.lidars[0].rate = 13;
  c.odom_error.enabled = true;
  c.odom_error.linear_sigma = c.odom_error.angular_sigma = .01;
  auto m = Model::load(c.urdf);
  World alone(c, m, {}, {{"r0", pose()}});
  World peers(c, m, {}, {{"r0", pose()}, {"far", pose({1000, 0, 0})}});
  alone.pause(false); peers.pause(false);
  for (int step = 0; step < 10; ++step) {
    if (step % 4 == 0) { alone.command(0, {.5, .2}); peers.command(0, {.5, .2}); }
    if (step % 2 == 1) peers.command(1, {.3, 0});
    sameEvents(alone.tick()[0], peers.tick()[0]);
    EXPECT_TRUE(alone.robot(0).snapshot().odom_base.isApprox(
        peers.robot(0).snapshot().odom_base, 1e-12)) << "step=" << step;
  }
}

TEST(ReviewTerrain, SupportTracksWheelSeparationAndFullWidth) {
  for (double separation : {.5, 1.0}) {
    auto c = config(); c.terrain.enabled = true; c.drive.wheel_separation = separation;
    auto m = Model::load(c.urdf);
    m.joints.at(c.drive.left_joint).origin.translation().y() = separation / 2;
    m.joints.at(c.drive.right_joint).origin.translation().y() = -separation / 2;
    // Both wheel centers miss this platform; the old +/-0.20 probes all fit.
    EXPECT_THROW(Simulator(c, m, platform(separation - .08)), std::runtime_error);
    // Even the outer half of the tire must be supported, not just its center.
    EXPECT_THROW(Simulator(c, m, platform(separation + .04)), std::runtime_error);
    Simulator supported(c, m, platform(separation + .06));
    supported.pause(false); supported.command({1, 0}); supported.tick();
    EXPECT_NEAR(supported.snapshot().world_base.translation().x(), .1, 1e-12);
    EXPECT_EQ(supported.contactCount(), 0u);
  }
}

TEST(ReviewTerrain, LostSideSupportStopsAndInvalidPosePreservesState) {
  auto c = config(); c.terrain.enabled = true;
  const auto m = Model::load(c.urdf);
  Simulator robot(c, m, platform(.6));
  const auto before = robot.snapshot();
  EXPECT_THROW(robot.setPose(pose({0, .05, 0})), std::runtime_error);
  EXPECT_TRUE(robot.snapshot().world_base.isApprox(before.world_base));
  EXPECT_EQ(robot.snapshot().stamp, before.stamp);
  robot.pause(false); robot.command({1, 1});
  for (int i = 0; i < 20; ++i) robot.tick();
  EXPECT_NE(robot.diagnostics()["last_error"].get<std::string>().find("terrain stop"), std::string::npos);
  for (const auto &part : m.placed(m.collision, robot.snapshot().world_base,
                                   c.base_link, robot.snapshot().joints)) {
    if (!c.drive.support_links.count(part.link)) continue;
    const Vec3 edge = part.transform.linear().col(2) * (part.shape->size.z() / 2);
    for (double sign : {-1., 1.})
      EXPECT_LE(std::abs((part.transform.translation() + sign * edge).y()), .3 + 1e-9);
  }
}

TEST(ReviewTerrain, EveryDeclaredWheelNeedsSupportIncludingMimicWheels) {
  auto c = config(); c.terrain.enabled = true;
  auto m = Model::load(c.urdf);
  const auto original = m.collision;
  for (const auto &name : {c.drive.left_joint, c.drive.right_joint}) {
    auto &rear = m.joints.at(name);
    rear.origin.translation().x() = -.24;
    auto front = rear;
    front.name = "front_" + rear.name;
    front.child = "front_" + rear.child;
    front.mimic = rear.name;
    front.origin.translation().x() = .24;
    m.links.insert(front.child); m.joints.emplace(front.name, front); m.order.push_back(front.name);
    c.drive.support_links.insert(front.child);
    for (auto part : original) if (part.link == rear.child) {
      part.link = front.child; m.collision.push_back(part);
    }
  }
  // This deck ends at x=0.20: both rear wheels fit, both front wheels miss it.
  EXPECT_THROW(Simulator(c, m, platform(2, 2, -.8)), std::runtime_error);
  EXPECT_NO_THROW(Simulator(c, m, platform(2)));
  c.drive.support_links.insert("lidar_link");
  m.collision.erase(std::remove_if(m.collision.begin(), m.collision.end(),
      [](const auto &g) { return g.link == "lidar_link"; }), m.collision.end());
  EXPECT_THROW(Simulator(c, m, platform(2)), std::runtime_error);
}
