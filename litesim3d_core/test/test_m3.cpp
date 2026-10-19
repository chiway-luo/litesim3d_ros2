#include "litesim3d/sim.hpp"
#include <chrono>
#include <cstring>
#include <fstream>
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
using namespace litesim3d;
namespace {
Config config3() {
  Scene s;
  auto c = Config::load(
      std::filesystem::path(SIM_RESOURCES) / "config/demo.yaml", s);
  c.lidars[0].columns = 32;
  c.lidars[0].vertical_angles = {0};
  c.lidars[0].scan_mode = "rolling";
  c.drive.unlimited_acceleration = true;
  c.timeout = 1000;
  return c;
}
float value3(const Cloud &c, size_t n, size_t field) {
  uint32_t v = 0;
  for (int i = 0; i < 4; ++i)
    v |= uint32_t(c.data.at(n * 24 + field + i)) << (8 * i);
  float f;
  std::memcpy(&f, &v, 4);
  return f;
}
std::vector<Event> step3(Simulator &s, uint32_t count) {
  s.pause(true);
  std::vector<Event> e;
  EXPECT_TRUE(s.singleStep(std::to_string(s.stepIndex()), count, e).success);
  return e;
}
Scene moving3(double velocity = 1, double omega = 0) {
  Scene s;
  Geometry g{"moving", "", Shape::box({.1, 200, 200})};
  g.motion = std::make_shared<Motion>();
  g.motion->initial = pose({5.05, 0, 0});
  g.motion->velocity = {velocity, 0, 0};
  g.motion->yaw_rate = omega;
  s.visual = s.sensor = s.collision = {g};
  s.rebuild();
  return s;
}
World world3(const std::vector<RobotSpec> &specs, Config c = config3(),
             Scene scene = {}) {
  return World(c, Model::load(c.urdf), scene, specs);
}
} // namespace
TEST(EXT04M3, StreamingDeliveryKeepsCommittedPeerStateAndFaultsTogether) {
  auto world = world3({{"r0", pose()},
                       {"r1", pose({3, 0, 0})},
                       {"r2", pose({0, 3, 0})},
                       {"r3", pose({3, 3, 0})}});
  size_t calls = 0;
  world.tick([&](size_t, const std::vector<Event> &) { ++calls; });
  EXPECT_EQ(calls, 0u);
  std::vector<std::vector<Event>> prior;
  ASSERT_TRUE(world.singleStep("before-scan", 19, prior).success);
  world.pause(false);
  std::vector<uint64_t> delivered;
  std::vector<std::vector<uint8_t>> bytes;
  auto events = world.tick([&](size_t i, const std::vector<Event> &batch) {
    EXPECT_EQ(i, calls++);
    for (size_t peer = 0; peer < world.size(); ++peer)
      EXPECT_EQ(world.robot(peer).snapshot().stamp, 100000001u);
    ASSERT_EQ(batch.size(), 1u);
    ASSERT_EQ(batch[0].clouds.size(), 1u);
    bytes.push_back(batch[0].clouds[0].data);
    delivered.push_back(std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::steady_clock::now().time_since_epoch())
                            .count());
  });
  ASSERT_EQ(calls, 4u);
  ASSERT_EQ(delivered.size(), 4u);
  for (size_t i = 0; i < world.size(); ++i) {
    const auto &cloud = events[i][0].clouds[0];
    EXPECT_EQ(cloud.data, bytes[i]);
    EXPECT_LE(cloud.compute_end_ns, delivered[i]);
    if (i > 0)
      EXPECT_GE(cloud.compute_end_ns, delivered[i - 1]);
  }
  EXPECT_THROW(world.tick([](size_t, const std::vector<Event> &) {
    throw std::runtime_error("consumer failed");
  }),
               std::runtime_error);
  for (size_t i = 0; i < world.size(); ++i)
    EXPECT_EQ(world.robot(i).state(), State::Fault);
}
TEST(EXT03M3, DynamicStopStateIsSharedOnlyInsideOneWorld) {
  auto c = config3();
  auto scene = moving3(-100);
  scene.sensor[0].motion->initial = pose({.65, 0, .3});
  auto a = world3({{"near", pose()}, {"far", pose({0, 10, 0})}}, c, scene);
  auto b = world3({{"independent", pose({1000, 0, 0})}}, c, scene);
  a.pause(false);
  b.pause(false);
  a.tick();
  b.tick();
  EXPECT_EQ(a.robot(0).scene().sensor[0].motion.get(),
            a.robot(1).scene().sensor[0].motion.get());
  EXPECT_EQ(a.robot(0).scene().sensor[0].motion->frozen_at, 1u);
  EXPECT_EQ(b.robot(0).scene().sensor[0].motion->frozen_at, UINT64_MAX);
  EXPECT_EQ(scene.sensor[0].motion->frozen_at, UINT64_MAX);
}
TEST(EXT01M3, TranslationAndRotationUsePerPointTime) {
  for (double omega : {0., 1.}) {
    auto scene = moving3(1, omega);
    LidarConfig l;
    l.vertical_angles = {0};
    l.columns = 100;
    l.scan_mode = "rolling";
    l.horizontal_start = 0;
    l.horizontal_end = .01;
    l.filter = "robot";
    auto cloud =
        scanTrajectory(scene, l, 1, [](uint64_t) { return Pose::Identity(); });
    for (size_t i = 0; i < 100; ++i) {
      double t = i * .001, a = i * .0001;
      double expected =
          (std::cos(omega * t) * (5.05 + t) - .05) / std::cos(a - omega * t);
      EXPECT_NEAR(std::hypot(value3(cloud, i, 0), value3(cloud, i, 4)),
                  expected, 1e-5);
    }
    EXPECT_EQ(scene.build_count, 1u);
  }
}
TEST(EXT02M3, FastObstacleCannotCrossStationaryOrOncomingRobot) {
  for (double v : {0., 1.}) {
    auto c = config3();
    auto scene = moving3(-100);
    scene.sensor[0].motion->initial = pose({.65, 0, .3});
    Simulator s(c, Model::load(c.urdf), scene);
    s.command({v, 0});
    step3(s, 40);
    EXPECT_EQ(s.scene().sensor[0].motion->frozen_at, 1u);
    EXPECT_EQ(scene.sensor[0].motion->frozen_at, UINT64_MAX);
    EXPECT_NEAR(s.snapshot().world_base.translation().x(), 0, 1e-12);
    EXPECT_EQ(s.snapshot().v, 0);
    EXPECT_GT(s.contactCount(), 0u);
    auto h = s.scene().ray({0, 0, .3}, Vec3::UnitX(), 10, {}, {},
                           s.snapshot().stamp);
    ASSERT_TRUE(h);
    EXPECT_NEAR(h->distance, .6, 1e-9);
  }
}
TEST(EXT03M3, WorldClockCommandsFramesAndMutualPerception) {
  auto c = config3();
  c.lidars[0].filter = "robot";
  auto world = world3({{"r0", pose({0, 0, 0})}, {"r1", pose({2, 0, 0})}}, c);
  world.command(0, {.2, 0});
  world.pause(false);
  bool peer_hit = false;
  for (int n = 0; n < 200; ++n) {
    auto e = world.tick();
    EXPECT_EQ(world.robot(0).snapshot().stamp, world.robot(1).snapshot().stamp);
    for (auto &event : e[0])
      for (auto &p : event.clouds)
        for (size_t slot = 0; slot < p.width; ++slot)
          if (std::isfinite(value3(p, slot, 0)))
            peer_hit = true;
  }
  EXPECT_NEAR(world.robot(0).snapshot().world_base.translation().x(), .2, 1e-9);
  EXPECT_NEAR(world.robot(1).snapshot().world_base.translation().x(), 2, 1e-9);
  EXPECT_TRUE(peer_hit);
  EXPECT_NE(world.robot(0).run_id, world.robot(1).run_id);
  EXPECT_EQ(world.robot(0).model().collision[0].shape.get(),
            world.robot(1).model().collision[0].shape.get());
}
TEST(EXT02M3, RelativeRobotSweepIsOrderIndependent) {
  auto c = config3();
  c.drive.max_v = 10;
  c.drive.max_w = 1;
  c.drive.wheel_radius = .1;
  auto w =
      world3({{"a", pose({-1, 0, 0})}, {"b", pose({1, 0, 0}, {0, 0, pi})}}, c);
  w.pause(false);
  w.command(0, {10, 0});
  w.command(1, {10, 0});
  for (int i = 0; i < 100; ++i)
    w.tick();
  EXPECT_LT(w.robot(0).snapshot().world_base.translation().x(),
            w.robot(1).snapshot().world_base.translation().x());
  EXPECT_NEAR(w.robot(0).snapshot().world_base.translation().x(),
              -w.robot(1).snapshot().world_base.translation().x(), 1e-9);
  EXPECT_GT(w.robot(0).contactCount(), 0u);
  EXPECT_GT(w.robot(1).contactCount(), 0u);
}
TEST(EXT04M3, AddingNoninteractingRobotPreservesRandomStream) {
  auto c = config3();
  c.lidars[0].range_sigma = .01;
  c.lidars[0].dropout = .1;
  auto scene = moving3(0);
  auto a = world3({{"r0", pose()}}, c, scene),
       b = world3({{"r0", pose()}, {"far", pose({1000, 0, 0})}}, c, scene);
  a.pause(false);
  b.pause(false);
  unsigned frames = 0;
  for (int n = 0; n < 2000; ++n) {
    auto x = a.tick(), y = b.tick();
    ASSERT_EQ(x[0].size(), y[0].size());
    for (size_t j = 0; j < x[0].size(); ++j) {
      ASSERT_EQ(x[0][j].clouds.size(), y[0][j].clouds.size());
      for (size_t k = 0; k < x[0][j].clouds.size(); ++k) {
        EXPECT_EQ(x[0][j].clouds[k].data, y[0][j].clouds[k].data);
        ++frames;
      }
    }
  }
  EXPECT_EQ(frames, 100u);
}
TEST(EXT05M3, AckermannAnalyticLimitsReverseAndNoPivot) {
  for (double v : {1., -1.}) {
    auto c = config3();
    c.drive.model = "ackermann";
    Simulator s(c, Model::load(c.urdf), {});
    s.command({v, 1});
    step3(s, 200);
    double w = std::tan(.6);
    EXPECT_NEAR(s.snapshot().world_base.translation().x(), v * std::sin(w) / w,
                1e-8);
    EXPECT_NEAR(s.snapshot().world_base.translation().y(),
                v * (1 - std::cos(w)) / w, 1e-8);
    EXPECT_NEAR(s.snapshot().w, w, 1e-12);
  }
  auto c = config3();
  c.drive.model = "ackermann";
  Simulator s(c, Model::load(c.urdf), {});
  s.command({0, 2});
  step3(s, 200);
  EXPECT_NEAR(yaw(s.snapshot().world_base), 0, 1e-12);
  EXPECT_EQ(s.snapshot().w, 0);
  c.drive.wheelbase = 0;
  EXPECT_THROW(c.validate(), std::runtime_error);
  c.drive.wheelbase = 1;
  c.drive.max_steering = pi / 2;
  EXPECT_THROW(c.validate(), std::runtime_error);
  c.drive.max_steering = .6;
  c.drive.model = "unimplemented";
  EXPECT_THROW(c.validate(), std::runtime_error);
}
TEST(EXT03M3, InvalidWorldAndStepIdempotency) {
  EXPECT_THROW(world3({{"same", pose()}, {"same", pose({2, 0, 0})}}),
               std::runtime_error);
  EXPECT_THROW(world3({{"r0", pose()}, {"r1", pose()}}), std::runtime_error);
  EXPECT_THROW(world3({{"/bad", pose()}}), std::runtime_error);
  auto w = world3({{"r0", pose()}});
  std::vector<std::vector<Event>> out;
  auto a = w.singleStep("a", 20, out);
  ASSERT_TRUE(a.success);
  auto b = w.singleStep("a", 20, out);
  EXPECT_EQ(a.time_ns, b.time_ns);
  EXPECT_FALSE(w.singleStep("a", 21, out).success);
  EXPECT_FALSE(w.singleStep("bad", 0, out).success);
  w.pause(false);
  EXPECT_FALSE(w.singleStep("busy", 1, out).success);
}

TEST(EXT01M3, DynamicSurfaceOutsideInitialBoundsAndSnapshot) {
  auto scene = moving3(1);
  const uint64_t time = 5'000'000'001;
  auto hit = scene.ray({6, 0, 0}, Vec3::UnitX(), 100, {}, {}, time);
  ASSERT_TRUE(hit);
  EXPECT_NEAR(hit->distance, 4, 1e-10);
  LidarConfig l;
  l.columns = 10;
  l.vertical_angles = {0};
  l.horizontal_start = 0;
  l.horizontal_end = .01;
  auto cloud =
      scanTrajectory(scene, l, time, [](uint64_t) { return Pose::Identity(); });
  for (unsigned i = 0; i < 10; ++i)
    EXPECT_NEAR(value3(cloud, i, 0), 10, 1e-6);
  EXPECT_EQ(scene.build_count, 1u);
}

TEST(EXT02M3, RotatingObstacleStopsBeforeCornerSweepAndRecordsTime) {
  auto c = config3();
  Scene scene;
  Geometry wall{"rotor", "", Shape::box({3, .05, .4})};
  wall.motion = std::make_shared<Motion>();
  wall.motion->initial = pose({0, 1, .3});
  wall.motion->yaw_rate = 100;
  scene.sensor = scene.visual = scene.collision = {wall};
  scene.rebuild();
  Simulator s(c, Model::load(c.urdf), scene);
  step3(s, 40);
  EXPECT_GT(s.contactCount(), 0u);
  EXPECT_LT(s.scene().sensor[0].motion->frozen_at, 200'000'001u);
  EXPECT_TRUE(s.snapshot().world_base.isApprox(Pose::Identity()));
  auto record = s.manifest()["dynamic_entities"][0];
  EXPECT_EQ(record["frozen_at_ns"].get<uint64_t>(),
            s.scene().sensor[0].motion->frozen_at);
  // A support seam can rotate a tall sensor farther than v*dt + yaw_rate*r*dt.
  auto ramp = [](const char *id, double pitch, double offset) {
    Geometry g{id, "",  Shape::box({1, 6, .2}), pose({}, {0, pitch, 0}),
               .5, true};
    g.transform.translation() = g.transform.linear() * Vec3(offset, 0, -.1);
    return g;
  };
  Geometry thin{"moving_thin_wall", "", Shape::box({.001, 3, .02})};
  thin.motion = std::make_shared<Motion>();
  thin.motion->initial = pose({.2, 0, 2});
  thin.motion->velocity = {.01, 0, 0};
  Scene seam;
  seam.visual = seam.sensor = seam.collision = {
      ramp("left", -2 * pi / 180, -.5), ramp("right", 2 * pi / 180, .5), thin};
  seam.rebuild();
  ASSERT_TRUE(seam.tilted_support);
  c.terrain.enabled = true;
  c.initial_pose = pose({-.003, 0, -.003 * std::tan(2 * pi / 180)});
  auto model = Model::load(c.urdf);
  model.joints.at("lidar_fixed").origin = pose({.2, 0, 2});
  Simulator on_seam(c, model, seam);
  on_seam.command({1, 0});
  step3(on_seam, 1);
  EXPECT_EQ(on_seam.scene().collision.back().motion->frozen_at, 1u);
  EXPECT_NEAR(on_seam.snapshot().world_base.translation().x(), -.003, 1e-12);
}

TEST(EXT03M3, FourRobotsOrderIndependentAndAllStatesIndependent) {
  auto c = config3();
  c.lidars[0].horizontal_start = 0;
  c.lidars[0].horizontal_end = .01;
  std::vector<RobotSpec> specs = {{"a", pose()},
                                  {"b", pose({2, 0, 0})},
                                  {"c", pose({0, 3, 0})},
                                  {"d", pose({2, 3, 0})}};
  auto a = world3(specs, c);
  std::reverse(specs.begin(), specs.end());
  auto b = world3(specs, c);
  a.pause(false);
  b.pause(false);
  a.command(0, {.2, .1});
  b.command(3, {.2, .1});
  for (int i = 0; i < 200; ++i) {
    auto x = a.tick(), y = b.tick();
    for (size_t r = 0; r < 4; ++r) {
      EXPECT_TRUE(a.robot(r).snapshot().world_base.isApprox(
          b.robot(3 - r).snapshot().world_base));
      ASSERT_EQ(x[r].size(), y[3 - r].size());
      for (size_t j = 0; j < x[r].size(); ++j)
        for (size_t k = 0; k < x[r][j].clouds.size(); ++k)
          EXPECT_EQ(x[r][j].clouds[k].data, y[3 - r][j].clouds[k].data);
    }
  }
  EXPECT_NE(a.robot(0).snapshot().joints.at("left_wheel_joint"),
            a.robot(1).snapshot().joints.at("left_wheel_joint"));
  auto report = a.manifest();
  EXPECT_EQ(report["robots"].size(), 4u);
  EXPECT_EQ(report["clock_publishers_required"], 1);
}

TEST(EXT05M3, StraightClippingAccelerationAndInvalidInput) {
  auto c = config3();
  c.drive.model = "ackermann";
  c.drive.unlimited_acceleration = false;
  Simulator s(c, Model::load(c.urdf), {});
  s.command({100, 0});
  step3(s, 400);
  EXPECT_NEAR(s.snapshot().v, 1, 1e-8);
  EXPECT_NEAR(s.snapshot().world_base.translation().x(), 1, 1e-7);
  EXPECT_NEAR(s.snapshot().world_base.translation().y(), 0, 1e-12);
  EXPECT_FALSE(s.command({NAN, 0}));
  auto bad = Command{.1, 0};
  bad.lateral = .01;
  EXPECT_FALSE(s.command(bad));
  s.command({0, 100});
  step3(s, 200);
  EXPECT_EQ(s.snapshot().v, 0);
  EXPECT_EQ(s.snapshot().w, 0);
}

TEST(EXT05M3, ReversalCannotPivotAtZeroMidpointSpeed) {
  auto c = config3();
  c.drive.model = "ackermann";
  c.drive.unlimited_acceleration = false;
  c.drive.acceleration = c.drive.deceleration = 2;
  Simulator s(c, Model::load(c.urdf), {});
  s.command({.005, 1});
  step3(s, 1);
  const double before = yaw(s.snapshot().world_base);
  s.command({-.005, 1});
  step3(s, 1);
  EXPECT_NEAR(yaw(s.snapshot().world_base), before, 1e-12);
  EXPECT_NEAR(s.snapshot().v, -.005, 1e-12);
}

TEST(EXT03M3, WorldBudgetsTerminalStateAndMotionSharing) {
  auto c = config3();
  auto w = world3({{"r0", pose()}});
  std::vector<std::vector<Event>> out;
  EXPECT_TRUE(w.tick()[0].empty());
  EXPECT_FALSE(w.singleStep("", 1, out).success);
  EXPECT_FALSE(w.singleStep(std::string(129, 'a'), 1, out).success);
  EXPECT_FALSE(w.singleStep("large", 10001, out).success);
  w.fault("test fault");
  EXPECT_FALSE(w.command(0, {.1, 0}));
  EXPECT_EQ(w.robot(0).state(), State::Fault);
  EXPECT_FALSE(w.singleStep("fault", 1, out).success);
  auto other = world3({{"r0", pose()}});
  other.exit();
  EXPECT_EQ(other.robot(0).state(), State::Exited);
  c.lidars[0].columns = 8192;
  c.lidars[0].vertical_angles = std::vector<double>(16, 0);
  auto large = world3({{"r0", pose()}}, c);
  EXPECT_FALSE(large.singleStep("budget", 10000, out).success);
  auto model = Model::load(c.urdf);
  while (model.collision.size() < 257)
    model.collision.push_back(model.collision[0]);
  EXPECT_NO_THROW(World(c, model, {}, {{"r0", pose()}}));
  c.collision_budget_enabled = true;
  EXPECT_THROW(World(c, model, {}, {{"r0", pose()}}), std::runtime_error);
  c = config3();
  c.imus.push_back(ImuConfig{});
  c.imus.back().topic = "/absolute";
  EXPECT_THROW(world3({{"r0", pose()}}, c), std::runtime_error);
  c = config3();
  c.terrain.enabled = true;
  Scene flat;
  flat.visual = flat.sensor = flat.collision = {Geometry{
      "ground", "", Shape::box({20, 20, .2}), pose({0, 0, -.1}), .5, true}};
  flat.rebuild();
  EXPECT_FALSE(flat.tilted_support);
  auto supported = world3({{"a", pose()}, {"b", pose({2, 0, 0})}}, c, flat);
  supported.pause(false);
  supported.command(0, {.2, 0});
  supported.tick();
  EXPECT_GT(supported.robot(0).snapshot().world_base.translation().x(), 0);
}

TEST(EXT01M3, ConfiguredObstaclesAreFixedAndWorldValidation) {
  const auto root =
      std::filesystem::temp_directory_path() / "lws-m3-validation";
  std::filesystem::create_directories(root);
  auto n = YAML::LoadFile(
      (std::filesystem::path(SIM_RESOURCES) / "config/demo.yaml").string());
  n["robot"]["urdf"] =
      (std::filesystem::path(SIM_RESOURCES) / "urdf/r01.urdf").string();
  n["entities"] = YAML::Node(YAML::NodeType::Sequence);
  n["entities"].push_back(YAML::Load(
      "{id: moving, pose: {xyz: [5,0,0]}, geometry: {type: box, size: "
      "[0.1,20,20]}, motion: {linear_velocity: [1,0,0], yaw_rate: 0.1}}"));
  auto path = root / "robot.yaml";
  auto save = [&] { std::ofstream(path) << n; };
  save();
  Scene scene;
  auto c = Config::load(path, scene);
  ASSERT_FALSE(scene.sensor[0].motion);
  EXPECT_NEAR(scene.sensor[0].at(1'000'000'001).translation().x(), 5, 1e-9);
  EXPECT_EQ(c.resolved["ignored_entity_motion"], Json::array({"moving"}));
  EXPECT_EQ(c.resolved["obstacles_fixed"], true);
  // Legacy visual-only obstacles must still block the robot while retaining
  // their initial pose, even when the old motion binding requested movement.
  n["entities"][0]["visual"] = n["entities"][0]["geometry"];
  n["entities"][0]["sensor_geometry"] = n["entities"][0]["geometry"];
  n["entities"][0].remove("geometry");
  n["robot"]["drive"]["continue_on_collision"] = true;
  save();
  c = Config::load(path, scene);
  ASSERT_EQ(scene.collision.size(), 1u);
  c.timeout = 60; c.drive.unlimited_acceleration = true;
  c.lidars[0].columns = 1; c.lidars[0].vertical_angles = {0};
  Simulator fixed(c, Model::load(c.urdf), scene);
  ASSERT_TRUE(fixed.command({1, 0})); step3(fixed, 1000);
  EXPECT_GT(fixed.contactCount(), 0u);
  EXPECT_NEAR(fixed.snapshot().world_base.translation().x(), 4.55, 2e-5);
  EXPECT_NEAR(fixed.snapshot().odom_base.translation().x(), 5, 1e-9);
  EXPECT_DOUBLE_EQ(fixed.scene().collision[0].at(fixed.snapshot().stamp).translation().x(), 5);
  n["entities"][0]["support"] = true;
  save();
  EXPECT_THROW(Config::load(path, scene), std::runtime_error);
  n["entities"][0].remove("support");
  n["entities"][0]["motion"]["yaw_rate"] = 101.;
  save();
  EXPECT_THROW(Config::load(path, scene), std::runtime_error);
  n["entities"][0]["motion"]["yaw_rate"] = 0.;
  n["entities"][0]["motion"]["typo"] = 1;
  save();
  EXPECT_THROW(Config::load(path, scene), std::runtime_error);
  n["entities"][0]["motion"].remove("typo");
  n["entities"][0]["motion"]["linear_velocity"] = YAML::Load("[101,0,0]");
  save();
  EXPECT_THROW(Config::load(path, scene), std::runtime_error);
  n["entities"] = YAML::Node(YAML::NodeType::Sequence);
  save();
  auto wpath = root / "world.yaml";
  auto w = YAML::Load("{schema_version: 1, config: robot.yaml, robots: [{id: "
                      "r0, xyz: [0,0,0]}]}");
  auto savew = [&] { std::ofstream(wpath) << w; };
  savew();
  auto world = World::load(wpath);
  EXPECT_EQ(world->size(), 1u);
  w["unexpected"] = 0;
  savew();
  EXPECT_THROW(World::load(wpath), std::runtime_error);
  w.remove("unexpected");
  w["schema_version"] = 2;
  savew();
  EXPECT_THROW(World::load(wpath), std::runtime_error);
  w["schema_version"] = 1;
  w["robots"][0]["xyz"] = YAML::Load("[0,0]");
  savew();
  EXPECT_THROW(World::load(wpath), std::runtime_error);
  w["robots"][0]["xyz"] = YAML::Load("[.nan,0,0]");
  savew();
  EXPECT_THROW(World::load(wpath), std::runtime_error);
  c = config3();
  c.lidars[0].topic = "/absolute";
  EXPECT_THROW(world3({{"r0", pose()}}, c), std::runtime_error);
  EXPECT_THROW(world3({}), std::runtime_error);
  c = config3();
  c.history_limit = 8;
  c.step_ns = 100'000'000;
  c.lidars[0].rate = 10;
  c.odom_rate = 1;
  EXPECT_THROW(world3({{"r0", pose()}}, c), std::runtime_error);
}
