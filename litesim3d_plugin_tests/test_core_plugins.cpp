#include <litesim3d/sim.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gtest/gtest.h>
#include <fstream>
#include <chrono>

using namespace litesim3d;
namespace {
Json parametersFor(const std::string &package) {
  return plugins::readParameterFile(std::filesystem::path(ament_index_cpp::get_package_share_directory(package)) / "config/parameters.yaml");
}
Model baseModel() {
  Model m; m.root = "base_link"; m.links.insert(m.root);
  m.report = {{"undriven_joints", Json::array()}, {"resources", Json::object()}};
  m.collision.push_back({"robot", m.root, Shape::box({.4, .3, .2}), pose({0, 0, .25})});
  return m;
}
void wheel(Model &m, const std::string &name, const Vec3 &position, double angle = 0,
           const std::string &parent = "base_link") {
  Joint j; j.name = name; j.parent = parent; j.child = name + "_link"; j.type = "continuous";
  j.origin = pose(position); j.axis = {-std::sin(angle), std::cos(angle), 0}; j.velocity = 100;
  m.joints[name] = j; m.order.push_back(name); m.links.insert(j.child);
  Pose shape = Pose::Identity();
  shape.linear() = Eigen::Quaterniond::FromTwoVectors(Vec3::UnitZ(), j.axis).toRotationMatrix();
  m.collision.push_back({"robot", j.child, Shape::cylinder(.1, .04), shape});
}
Model diffModel() {
  auto m = baseModel(); wheel(m, "left_wheel_joint", {0, .25, .1}); wheel(m, "right_wheel_joint", {0, -.25, .1});
  m.sensor = m.visual = m.collision; return m;
}
Model omniModel(int n) {
  auto m = baseModel();
  for (int i = 0; i < n; ++i) {
    const double angle = 2 * pi * i / n + (n == 4 ? pi / 4 : 0), radius = n == 4 ? .3 * std::sqrt(2) : .3;
    wheel(m, "wheel_" + std::to_string(i) + "_joint", {radius * std::cos(angle), radius * std::sin(angle), .1}, angle + pi / 2);
  }
  m.sensor = m.visual = m.collision; return m;
}
Model ackModel() {
  auto m = baseModel();
  for (const auto &side : {std::string("left"), std::string("right")}) {
    const double y = side == "left" ? .2 : -.2;
    wheel(m, "rear_" + side + "_wheel_joint", {0, y, .1});
    Joint j; j.name = side + "_steering_joint"; j.parent = "base_link"; j.child = side + "_steering_link";
    j.type = "revolute"; j.axis = Vec3::UnitZ(); j.lower = -.7; j.upper = .7; j.velocity = 10; j.origin = pose({.6, y, .1});
    m.joints[j.name] = j; m.order.push_back(j.name); m.links.insert(j.child);
    wheel(m, "front_" + side + "_wheel_joint", Vec3::Zero(), 0, j.child);
  }
  m.sensor = m.visual = m.collision; return m;
}
Config configuration(const std::string &drive_package, const std::string &drive_class,
                     const std::string &odom_package, const std::string &odom_class, bool keep_driving) {
  Config c; c.step_ns = 10000000; c.timeout = 100; c.drive.model = "plugin";
  c.drive.continue_on_collision = keep_driving; c.drive.support_links.clear();
  auto p = parametersFor(drive_package); p["limits"]["unlimited_acceleration"] = true;
  auto o = parametersFor(odom_package); if (p.contains("wheels")) o["wheels"] = p["wheels"];
  c.plugins = {{"chassis", "drive", drive_package + "/" + drive_class, p, {}},
               {"odometry", "encoders", odom_package + "/" + odom_class, o, {}}};
  return c;
}
Scene wall(bool lateral = false) {
  Scene scene;
  scene.collision = {{"wall", "", Shape::box(lateral ? Vec3(10, .1, 2) : Vec3(.1, 10, 2)),
                    pose(lateral ? Vec3(0, 1, 1) : Vec3(1, 0, 1))}};
  scene.visual = scene.sensor = scene.collision; scene.rebuild(); return scene;
}
std::vector<Event> advance(Simulator &sim, unsigned steps) {
  sim.pause(true); std::vector<Event> events;
  auto result = sim.singleStep(std::to_string(sim.stepIndex()), steps, events);
  EXPECT_TRUE(result.success) << result.message; return events;
}
Config diffConfig(bool keep_driving) {
  return configuration("litesim3d_diff_drive", "DifferentialDrive", "litesim3d_diff_odometry", "DifferentialOdometry", keep_driving);
}
}
TEST(CorePlugins, DifferentialCollisionToggleAndReverseEscape) {
  for (bool keep_driving : {false, true}) {
    auto c = diffConfig(keep_driving); Simulator sim(c, diffModel(), wall());
    ASSERT_TRUE(sim.command({1, 0})); advance(sim, 200);
    const auto stopped = sim.snapshot(); ASSERT_GT(sim.contactCount(), 0u);
    EXPECT_NEAR(stopped.world_base.translation().x(), .75, 2e-5); EXPECT_EQ(stopped.v, 0);
    EXPECT_NEAR(stopped.odom_base.translation().x(), keep_driving ? 2 : stopped.world_base.translation().x(), 1e-9);
    EXPECT_NEAR(stopped.odom_v, keep_driving ? 1 : 0, 1e-12);
    ASSERT_TRUE(sim.command({-.5, 0})); advance(sim, 20);
    EXPECT_NEAR(sim.snapshot().world_base.translation().x(), stopped.world_base.translation().x() - .1, 1e-8);
    EXPECT_NEAR(sim.snapshot().odom_base.translation().x(), stopped.odom_base.translation().x() - .1, 1e-8);
    EXPECT_DOUBLE_EQ(sim.scene().collision[0].at(sim.snapshot().stamp).translation().x(), 1);
  }
}
TEST(CorePlugins, OmniLateralCollisionKeepsEncoderLateralTwistSeparateFromTruth) {
  for (int n : {3, 4}) for (bool keep_driving : {false, true}) {
    SCOPED_TRACE(std::to_string(n) + "/" + std::to_string(keep_driving));
    auto c = configuration("litesim3d_omni" + std::to_string(n) + "_drive", "Omni" + std::to_string(n) + "Drive",
                           "litesim3d_omni_odometry", "OmniOdometry", keep_driving);
    Simulator sim(c, omniModel(n), wall(true)); Command cmd; cmd.lateral = 1;
    ASSERT_TRUE(sim.command(cmd)); advance(sim, 200);
    const auto stopped = sim.snapshot(); ASSERT_GT(sim.contactCount(), 0u);
    EXPECT_LT(stopped.world_base.translation().y(), .8); EXPECT_GT(stopped.world_base.translation().y(), .5);
    EXPECT_EQ(stopped.lateral, 0); EXPECT_NEAR(stopped.odom_lateral, keep_driving ? 1 : 0, 1e-12);
    EXPECT_NEAR(stopped.odom_base.translation().y(), keep_driving ? 2 : stopped.world_base.translation().y(), 1e-9);
    EXPECT_NEAR(stopped.odom_base.translation().x(), 0, 1e-9);
    cmd.lateral = -.5; ASSERT_TRUE(sim.command(cmd)); advance(sim, 20);
    EXPECT_NEAR(sim.snapshot().world_base.translation().y(), stopped.world_base.translation().y() - .1, 1e-8);
  }
}
TEST(CorePlugins, AckermannControlsSteeringAndMatchesIndependentEncoderOdometry) {
  auto c = configuration("litesim3d_ackermann_drive", "AckermannDrive", "litesim3d_ackermann_odometry", "AckermannOdometry", true);
  Simulator sim(c, ackModel(), {}); ASSERT_TRUE(sim.command({.5, .2})); advance(sim, 100);
  const auto &s = sim.snapshot();
  EXPECT_NEAR(s.joints.at("left_steering_joint"), std::atan(.6 / (2.5 - .2)), 1e-12);
  EXPECT_NEAR(s.joints.at("right_steering_joint"), std::atan(.6 / (2.5 + .2)), 1e-12);
  EXPECT_NEAR(s.world_base.translation().x(), 2.5 * std::sin(.2), 1e-9);
  EXPECT_NEAR(s.world_base.translation().y(), 2.5 * (1 - std::cos(.2)), 1e-9);
  EXPECT_LT((s.world_base.matrix() - s.odom_base.matrix()).norm(), 1e-9);
  EXPECT_NE(s.joint_velocities.at("front_left_wheel_joint"), s.joint_velocities.at("front_right_wheel_joint"));
}
TEST(CorePlugins, AckermannSteeringSweepCannotRotateWheelThroughObstacle) {
  auto c = configuration("litesim3d_ackermann_drive", "AckermannDrive", "litesim3d_ackermann_odometry", "AckermannOdometry", true);
  Scene scene;
  scene.collision = {{"steering_barrier", "", Shape::box({.5, .02, .5}), pose({.6, .27, .25})}};
  scene.sensor = scene.visual = scene.collision; scene.rebuild();
  Simulator sim(c, ackModel(), scene); ASSERT_TRUE(sim.command({.1, 2})); advance(sim, 10);
  EXPECT_GT(sim.contactCount(), 0u);
  EXPECT_GT(sim.snapshot().joints.at("left_steering_joint"), 0);
  EXPECT_LT(sim.snapshot().joints.at("left_steering_joint"), .6);
  EXPECT_LT(sim.snapshot().world_base.translation().norm(), 1e-12);
  const auto placed = sim.model().placed(sim.model().collision, sim.snapshot().world_base, c.base_link, sim.snapshot().joints);
  for (const auto &part : placed) {
    fcl::CollisionObjectd a(part.shape->collision, part.transform);
    fcl::CollisionObjectd b(scene.collision[0].shape->collision, scene.collision[0].transform);
    fcl::CollisionRequestd request; fcl::CollisionResultd result; fcl::collide(&a, &b, request, result);
    EXPECT_FALSE(result.isCollision()) << part.link;
  }
}
TEST(CorePlugins, AckermannExactUrdfSteeringLimitAcceptsSaturatedTurn) {
  auto c = configuration("litesim3d_ackermann_drive", "AckermannDrive", "litesim3d_ackermann_odometry", "AckermannOdometry", true);
  auto model = ackModel();
  for (const auto &name : {"left_steering_joint", "right_steering_joint"}) {
    model.joints.at(name).lower = -.6; model.joints.at(name).upper = .6;
  }
  Simulator sim(c, model, {}); ASSERT_TRUE(sim.command({.2, 1}));
  EXPECT_NO_THROW(advance(sim, 100));
  EXPECT_DOUBLE_EQ(sim.snapshot().joints.at("left_steering_joint"), .6);
  EXPECT_TRUE(sim.snapshot().world_base.matrix().isApprox(sim.snapshot().odom_base.matrix(), 1e-9));
}
TEST(CorePlugins, FreshSessionResetsPluginsAndTrueSensorsRemainBlocked) {
  auto c = diffConfig(true); c.imus = {ImuConfig{}};
  LidarConfig l; l.link = "base_link"; l.columns = 1; l.vertical_angles = {0};
  l.filter = "robot"; l.horizontal_start = 0; l.horizontal_end = .01; c.lidars = {l};
  Simulator sim(c, diffModel(), wall()); ASSERT_TRUE(sim.command({1, 0})); advance(sim, 150);
  const auto stopped = sim.snapshot(); auto events = advance(sim, 50); unsigned count = 0;
  for (const auto &event : events) for (const auto &imu : event.imus) {
    ++count; EXPECT_LT(imu.angular_velocity.norm(), 1e-12);
    EXPECT_LT((imu.acceleration - Vec3(0, 0, 9.80665)).norm(), 1e-12);
    EXPECT_LT((event.snapshot.world_base.matrix() - stopped.world_base.matrix()).norm(), 1e-12);
  }
  EXPECT_GT(count, 0u); EXPECT_GT(sim.snapshot().odom_base.translation().x(), 1.9);
  Simulator fresh(c, diffModel(), wall()); advance(fresh, 10);
  EXPECT_TRUE(fresh.snapshot().world_base.matrix().isApprox(Pose::Identity().matrix()));
  EXPECT_TRUE(fresh.snapshot().odom_base.matrix().isApprox(Pose::Identity().matrix()));
  EXPECT_EQ(fresh.snapshot().joints.at("left_wheel_joint"), 0);
}
TEST(CorePlugins, UnknownClassAndMissingJointAreStartupErrors) {
  auto c = diffConfig(true); c.plugins[0].class_name = "absent/Drive";
  EXPECT_THROW(Simulator(c, diffModel(), {}), std::runtime_error);
  c = diffConfig(true); c.plugins[0].parameters["left_joints"] = {"absent_joint"};
  EXPECT_THROW(Simulator(c, diffModel(), {}), std::runtime_error);
}

TEST(CorePlugins, UrdfPluginTagsLoadPackageParameterFilesAndRejectYamlConflicts) {
  const auto directory = std::filesystem::temp_directory_path() / ("litesim-model-plugins-" +
      std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  ASSERT_TRUE(std::filesystem::create_directory(directory));
  struct Cleanup { std::filesystem::path path; ~Cleanup() { std::filesystem::remove_all(path); } } cleanup{directory};
  auto xml = readText(std::filesystem::path(ament_index_cpp::get_package_share_directory("litesim3d_core")) / "urdf/r01.urdf");
  const auto at = xml.rfind("</robot>"); ASSERT_NE(at, std::string::npos);
  xml.insert(at, R"(<litesim3d version="1">
    <chassis name="drive" class="litesim3d_diff_drive/DifferentialDrive">
      <parameters file="package://litesim3d_diff_drive/config/parameters.yaml"/>
      <overrides>limits: {unlimited_acceleration: true}</overrides>
    </chassis>
    <odometry name="encoder_odom" class="litesim3d_diff_odometry/DifferentialOdometry">
      <parameters file="package://litesim3d_diff_odometry/config/parameters.yaml"/>
    </odometry>
  </litesim3d>)");
  std::ofstream(directory / "composed.urdf") << xml;
  const std::string yaml = "schema_version: 1\nunits: m_rad_s\nrobot: {urdf: composed.urdf, continue_on_collision: true}\nlidars: []\nentities: []\n";
  std::ofstream(directory / "runtime.yaml") << yaml;
  Scene scene; auto c = Config::load(directory / "runtime.yaml", scene);
  ASSERT_EQ(c.plugins.size(), 2u); EXPECT_EQ(c.drive.model, "plugin");
  Simulator sim(c, Model::load(c.urdf, c.model_options), scene);
  ASSERT_TRUE(sim.command({.5, 0})); advance(sim, 20);
  EXPECT_NEAR(sim.snapshot().world_base.translation().x(), .05, 1e-9);
  EXPECT_EQ(sim.manifest()["config"]["plugins"][0]["parameters"]["limits"]["unlimited_acceleration"], true);
  auto conflict = yaml; const auto pos = conflict.find("continue_on_collision: true");
  conflict.insert(pos, "motion_model: differential_drive, ");
  std::ofstream(directory / "conflict.yaml") << conflict;
  EXPECT_THROW(Config::load(directory / "conflict.yaml", scene), std::runtime_error);
}
