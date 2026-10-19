#include "litesim3d/sim.hpp"
#include <chrono>
#include <cstring>
#include <fstream>
#include <gtest/gtest.h>
#include <limits>
#include <thread>
using namespace litesim3d;
namespace {
const std::filesystem::path resources = SIM_RESOURCES;
std::filesystem::path temporary(const std::string &body, const std::string &extension = ".urdf") {
  static uint64_t index = 0;
  auto path =
      std::filesystem::temp_directory_path() /
      ("lws-test-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) +
       "-" + std::to_string(index++) + extension);
  std::ofstream out(path);
  out << body;
  return path;
}
Model importText(const std::string &text, const ModelOptions &options = {}) {
  auto path = temporary(text);
  try {
    auto m = Model::load(path, options);
    std::filesystem::remove(path);
    return m;
  } catch (...) {
    std::filesystem::remove(path);
    throw;
  }
}
std::string replace(std::string text, const std::string &a, const std::string &b) {
  auto pos = text.find(a);
  if (pos == std::string::npos)
    throw std::runtime_error("fixture substring absent: " + a);
  text.replace(pos, a.size(), b);
  return text;
}
Geometry box(const std::string &id, Vec3 center, Vec3 size) {
  return {id, "", Shape::box(size), pose(center)};
}
Scene scene(std::initializer_list<Geometry> shapes) {
  Scene s;
  s.sensor = shapes;
  s.collision = shapes;
  s.visual = shapes;
  s.rebuild();
  return s;
}
Config config() {
  Scene ignored;
  auto c = Config::load(resources / "config/demo.yaml", ignored);
  c.lidars[0].columns = 8;
  c.lidars[0].vertical_angles = {0};
  c.timeout = 60;
  c.drive.unlimited_acceleration = true;
  return c;
}
Simulator simulator(Config c = config(), Scene s = {}) {
  return Simulator(c, Model::load(c.urdf, c.model_options), s);
}
float number(const Cloud &c, size_t slot, size_t field) {
  uint32_t word = 0;
  for (int b = 0; b < 4; ++b)
    word |= uint32_t(c.data[slot * 24 + field + b]) << (8 * b);
  float v;
  std::memcpy(&v, &word, 4);
  return v;
}
LidarConfig lidar() {
  LidarConfig l;
  l.vertical_angles = {0};
  l.columns = 1;
  l.horizontal_start = 0;
  l.horizontal_end = .1;
  return l;
}
void advance(Simulator &s, size_t n, double v = 0, double w = 0) {
  s.pause(false);
  for (size_t i = 0; i < n; ++i) {
    ASSERT_TRUE(s.command({v, w}));
    s.tick();
  }
}
} // namespace

TEST(COR01, HeadlessDeterministic) {
  auto a = simulator(), b = simulator();
  advance(a, 200, .2, .1);
  advance(b, 200, .2, .1);
  EXPECT_TRUE(a.snapshot().world_base.matrix() == b.snapshot().world_base.matrix());
}

TEST(URDFMimic, SimulatorReportsChainedVelocitiesDuringTurnsReverseAndStop) {
  const auto text = replace(readText(resources / "urdf/r01.urdf"), "</robot>", R"(
<link name="follower_link"/><link name="chain_link"/>
<joint name="follower" type="continuous"><parent link="base_link"/><child link="follower_link"/>
<axis xyz="0 1 0"/><limit effort="100" velocity="1000"/>
<mimic joint="left_wheel_joint" multiplier="-2" offset="0.3"/></joint>
<joint name="chain" type="continuous"><parent link="base_link"/><child link="chain_link"/>
<axis xyz="0 1 0"/><limit effort="100" velocity="1000"/>
<mimic joint="follower" multiplier="-0.5" offset="0.1"/></joint></robot>)");
  Simulator s(config(), importText(text), {});
  for (const auto &command : std::vector<std::pair<double, double>>{{.4, .2}, {-.3, -.4}, {0., 0.}}) {
    advance(s, 20, command.first, command.second);
    const auto &state = s.snapshot();
    const double expected = (command.first - command.second * .5 / 2) / .1;
    ASSERT_TRUE(state.joint_velocities.count("follower"));
    ASSERT_TRUE(state.joint_velocities.count("chain"));
    EXPECT_NEAR(state.joint_velocities.at("follower"), -2 * expected, 1e-12);
    EXPECT_NEAR(state.joint_velocities.at("chain"), expected, 1e-12);
    EXPECT_NEAR(state.joints.at("follower"), -2 * state.joints.at("left_wheel_joint") + .3, 1e-12);
    EXPECT_NEAR(state.joints.at("chain"), -.5 * state.joints.at("follower") + .1, 1e-12);
  }
}

TEST(URDFMimic, CollisionStopsFollowerVelocity) {
  const auto text = replace(readText(resources / "urdf/r01.urdf"), "</robot>", R"(
<link name="follower_link"/>
<joint name="follower" type="continuous"><parent link="base_link"/><child link="follower_link"/>
<axis xyz="0 1 0"/><limit effort="100" velocity="1000"/>
<mimic joint="left_wheel_joint" multiplier="1" offset="0"/></joint></robot>)");
  Simulator s(config(), importText(text), scene({box("wall", {.8, 0, .5}, {.1, 2, 1})}));
  advance(s, 200, .8, 0);
  EXPECT_FALSE(s.contacts().empty());
  ASSERT_TRUE(s.snapshot().joint_velocities.count("follower"));
  EXPECT_DOUBLE_EQ(s.snapshot().joint_velocities.at("follower"), 0);
  EXPECT_DOUBLE_EQ(s.snapshot().joint_velocities.at("left_wheel_joint"), 0);
}
TEST(COR02, IntegerClock) {
  auto s = simulator();
  s.pause(false);
  for (int i = 0; i < 100000; ++i)
    s.tick();
  EXPECT_EQ(s.snapshot().stamp, 500000000001ULL);
  EXPECT_EQ(s.stepIndex(), 100000);
}
TEST(COR03, MultiRate) {
  auto c = config();
  const auto prototype = c.lidars[0];
  c.lidars.clear();
  std::map<std::string, double> rates;
  std::map<std::string, uint64_t> counts;
  for (double rate : {7.0, 10.0, 50.0, 75.0, 200.0, 7.3}) {
    auto sensor = prototype;
    sensor.name = sensor.topic = "rate_" + std::to_string(rate);
    sensor.rate = rate;
    sensor.columns = 1;
    rates[sensor.topic] = rate;
    c.lidars.push_back(sensor);
  }
  auto s = simulator(c);
  s.pause(false);
  uint64_t odom = 0;
  for (int i = 0; i < 12000; ++i) {
    for (auto &e : s.tick()) {
      if (e.odom)
        ++odom;
      for (auto &cloud : e.clouds) {
        const auto n = ++counts[cloud.topic];
        EXPECT_EQ(cloud.stamp, 1 + uint64_t(std::llround(n * 1e9 / rates.at(cloud.topic))));
        EXPECT_LE(cloud.stamp, s.snapshot().stamp);
      }
    }
  }
  for (const auto &[topic, rate] : rates)
    EXPECT_EQ(counts[topic], std::llround(60 * rate));
  EXPECT_EQ(odom, 3000);
}
TEST(COR04, PauseResume) {
  auto s = simulator();
  advance(s, 20, .5);
  s.pause(true);
  const auto p = s.snapshot();
  std::this_thread::sleep_for(std::chrono::seconds(2));
  EXPECT_TRUE(s.tick().empty());
  EXPECT_EQ(s.snapshot().stamp, p.stamp);
  EXPECT_TRUE(s.snapshot().world_base.matrix() == p.world_base.matrix());
  s.pause(false);
  s.tick();
  EXPECT_EQ(s.snapshot().stamp, p.stamp + 5000000);
}
TEST(COR05, StepIdempotency) {
  auto s = simulator();
  std::vector<Event> e;
  EXPECT_FALSE(s.singleStep("a", 10, e).success);
  s.pause(true);
  auto a = s.singleStep("a", 10, e);
  EXPECT_TRUE(a.success);
  EXPECT_EQ(a.time_ns, 50000001ULL);
  EXPECT_EQ(s.singleStep("a", 10, e).time_ns, a.time_ns);
  EXPECT_EQ(s.stepIndex(), 10);
  EXPECT_EQ(s.singleStep("a", 11, e).code, 4);
  EXPECT_EQ(s.singleStep("", 10, e).code, 2);
  EXPECT_EQ(s.singleStep("b", 0, e).code, 2);
  s.pause(false);
  EXPECT_EQ(s.singleStep("c", 1, e).code, 1);
}
TEST(COR05, PendingCommandDiagnostics) {
  auto s = simulator();
  s.pause(true);
  const auto before = s.snapshot().stamp;
  EXPECT_EQ(s.diagnostics()["pending_command"], false);
  ASSERT_TRUE(s.command({.4, 0}));
  EXPECT_EQ(s.diagnostics()["pending_command"], true);
  EXPECT_EQ(s.snapshot().stamp, before);
  EXPECT_TRUE(s.manifest()["inputs"].empty());
  EXPECT_FALSE(s.command({std::numeric_limits<double>::quiet_NaN(), 0}));
  EXPECT_EQ(s.diagnostics()["pending_command"], true);
  std::vector<Event> events;
  ASSERT_TRUE(s.singleStep("apply-command", 20, events).success);
  EXPECT_EQ(s.diagnostics()["pending_command"], false);
  EXPECT_EQ(s.snapshot().stamp, before + 100000000ULL);
  EXPECT_NEAR(s.snapshot().world_base.translation().x(), .04, 1e-12);
  EXPECT_EQ(s.manifest()["inputs"].size(), 1);
  ASSERT_TRUE(s.singleStep("apply-command", 20, events).success);
  EXPECT_EQ(s.snapshot().stamp, before + 100000000ULL);
  EXPECT_EQ(s.manifest()["inputs"].size(), 1);
}
TEST(COR06, CommandReplay) {
  std::vector<Json> histories;
  std::vector<Pose> ends;
  for (int repetition = 0; repetition < 3; ++repetition) {
    auto s = simulator();
    s.pause(false);
    for (int i = 0; i < 1000; ++i) {
      s.command({.3, .2});
      s.command({.2 * std::sin(i * .1), .3 * std::cos(i * .1)});
      s.tick();
    }
    histories.push_back(s.manifest()["inputs"]);
    ends.push_back(s.snapshot().world_base);
    EXPECT_EQ(s.diagnostics()["coalesced_commands"], 1000);
  }
  EXPECT_EQ(histories[0], histories[1]);
  EXPECT_EQ(histories[1], histories[2]);
  EXPECT_TRUE(ends[0].matrix() == ends[2].matrix());
}
TEST(COR07, IndependentStreams) {
  std::mt19937_64 a(streamSeed(42, "robot/lidar/noise")), b(streamSeed(42, "robot/lidar/noise")),
      other(streamSeed(43, "robot/lidar/noise")), extra(streamSeed(42, "robot/extra"));
  for (int i = 0; i < 1000; ++i) {
    auto v = a();
    EXPECT_EQ(v, b());
    EXPECT_NE(v, other());
    extra();
  }
}
TEST(COR08, InvalidConfig) {
  auto c = config();
  c.step_ns = 0;
  EXPECT_THROW(c.validate(), std::runtime_error);
  c = config();
  c.odom_rate = -1;
  EXPECT_THROW(c.validate(), std::runtime_error);
  c = config();
  c.timeout = NAN;
  EXPECT_THROW(c.validate(), std::runtime_error);
  c = config();
  c.real_time_factor = INFINITY;
  EXPECT_THROW(c.validate(), std::runtime_error);
  const auto original = readText(resources / "config/demo.yaml");
  for (auto bad : {replace(original, "threads: 1", "threads: 0"),
                   replace(original, "threads: 1", "mystery: 1"),
                   replace(original, "step_size: 0.005", "step_size: .nan"),
                   replace(original, "schema_version: 1", "schema_version: 2")}) {
    auto p = temporary(bad, ".yaml");
    Scene s;
    EXPECT_THROW(Config::load(p, s), std::runtime_error);
    std::filesystem::remove(p);
  }
}
TEST(COR09, TransformInverse) {
  std::mt19937_64 rng(42);
  std::uniform_real_distribution<double> d(-3, 3);
  for (int i = 0; i < 300; ++i) {
    auto p = pose({d(rng), d(rng), d(rng)}, {d(rng), d(rng), d(rng)});
    EXPECT_TRUE((p.inverse() * p).matrix().isApprox(Eigen::Matrix4d::Identity(), 1e-12));
    Eigen::Quaterniond q(p.linear());
    auto qn = q;
    qn.coeffs() *= -1;
    EXPECT_TRUE(quaternionPose(p.translation(), q)
                    .matrix()
                    .isApprox(quaternionPose(p.translation(), qn).matrix(), 1e-12));
    EXPECT_NEAR(q.norm(), 1, 1e-12);
  }
  for (double a : {1e-12, pi - 1e-12})
    EXPECT_TRUE((pose(Vec3::Zero(), {a, 0, 0}).inverse() * pose(Vec3::Zero(), {a, 0, 0}))
                    .matrix()
                    .isApprox(Eigen::Matrix4d::Identity(), 1e-12));
  EXPECT_THROW(quaternionPose(Vec3::Zero(), Eigen::Quaterniond(0, 0, 0, 0)), std::runtime_error);
}
TEST(COR10, ManifestAndHashes) {
  auto s = simulator();
  advance(s, 10, .2);
  auto m = s.manifest("test");
  EXPECT_EQ(m["step_ns"], 5000000);
  EXPECT_EQ(m["seed"], 42);
  EXPECT_EQ(m["inputs"].size(), 10);
  EXPECT_EQ(m["model"]["sha256"], sha256(config().urdf));
  EXPECT_FALSE(m["compiler"].get<std::string>().empty());
  EXPECT_NE(sha256Text("a"), sha256Text("b"));
  EXPECT_EQ(sha256Text("abc"), "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
}
TEST(COR11, PacingDoesNotChangeModel) {
  std::vector<Pose> outputs;
  std::vector<std::vector<uint8_t>> sensor_outputs;
  for (double rtf : {.5, 1.0, 0.0}) {
    auto c = config();
    c.real_time_factor = rtf;
    auto s = simulator(c);
    s.pause(false);
    std::vector<uint8_t> bytes;
    for (int i = 0; i < 200; ++i) {
      ASSERT_TRUE(s.command({.3, .2}));
      for (const auto &event : s.tick())
        for (const auto &cloud : event.clouds)
          bytes.insert(bytes.end(), cloud.data.begin(), cloud.data.end());
    }
    outputs.push_back(s.snapshot().world_base);
    sensor_outputs.push_back(bytes);
  }
  EXPECT_TRUE(outputs[0].matrix() == outputs[1].matrix());
  EXPECT_TRUE(outputs[1].matrix() == outputs[2].matrix());
  EXPECT_FALSE(sensor_outputs[0].empty());
  EXPECT_EQ(sensor_outputs[0], sensor_outputs[1]);
  EXPECT_EQ(sensor_outputs[1], sensor_outputs[2]);
}
TEST(COR12, StateTransactions) {
  auto s = simulator();
  EXPECT_EQ(s.state(), State::Ready);
  s.setPose(pose({2, 1, 0}));
  EXPECT_NEAR(s.snapshot().world_base.translation().x(), 2, 1e-12);
  s.pause(false);
  EXPECT_THROW(s.setPose(pose({9, 0, 0})), std::runtime_error);
  // Candidate loading happens before replacement. A failed runtime request must
  // leave the already running simulator and its scene unchanged.
  const auto before_failed_load = s.snapshot();
  Scene candidate;
  EXPECT_THROW(Config::load(resources / "missing-candidate.yaml", candidate), std::runtime_error);
  EXPECT_EQ(s.state(), State::Running);
  EXPECT_EQ(s.snapshot().stamp, before_failed_load.stamp);
  EXPECT_TRUE(s.snapshot().world_base.matrix() == before_failed_load.world_base.matrix());
  EXPECT_EQ(s.snapshot().stamp, 1);
  EXPECT_NEAR(s.snapshot().world_base.translation().x(), 2, 1e-12);
  s.tick();
  s.pause(true);
  s.fault("test");
  EXPECT_EQ(s.state(), State::Fault);
  EXPECT_THROW(s.pause(false), std::runtime_error);
  EXPECT_FALSE(s.command({1, 0}));
  s.exit();
  EXPECT_EQ(s.state(), State::Exited);
}

TEST(URD01, Tree) {
  auto m = Model::load(resources / "urdf/r01.urdf");
  EXPECT_EQ(m.root, "base_link");
  EXPECT_EQ(m.links.size(), 4);
  EXPECT_EQ(m.joints.size(), 3);
  EXPECT_EQ(m.joints.at("lidar_fixed").parent, "base_link");
}
TEST(URD02, MountTransform) {
  auto xml = readText(resources / "urdf/r01.urdf");
  xml = replace(xml, "xyz=\"0.2 0 0.65\"", "xyz=\"0.2 0 0.5\" rpy=\"0 0 1.5707963267948966\"");
  auto m = importText(xml);
  EXPECT_LT((m.fk({}).at("lidar_link") * Vec3(1, 0, 0) - Vec3(.2, 1, .5)).norm(), 1e-6);
}
TEST(URD03, JointLimits) {
  auto m = Model::load(resources / "urdf/r03.urdf");
  auto p = m.positions({{"main", 3}, {"slide", -3}, {"mimic", 100}});
  EXPECT_EQ(p["main"], 1);
  EXPECT_EQ(p["slide"], 0);
  auto q = m.advance({}, {{"main", 1}, {"slide", 1}}, .1);
  EXPECT_NEAR(q["main"], .2, 1e-12);
  EXPECT_NEAR(q["slide"], .05, 1e-12);
  auto r = Model::load(resources / "urdf/r01.urdf");
  EXPECT_EQ(r.positions({{"left_wheel_joint", 100}})["left_wheel_joint"], 100);
}
TEST(URD04, ArbitraryAxis) {
  auto m = Model::load(resources / "urdf/r03.urdf");
  Vec3 a = Vec3(1, 1, 1).normalized(), v(1, 2, 3);
  const double t = .3;
  Vec3 expected = v * std::cos(t) + a.cross(v) * std::sin(t) + a * a.dot(v) * (1 - std::cos(t)) +
                  Vec3(.2, 0, .5);
  EXPECT_LT((m.fk({{"main", t}}).at("arm") * v - expected).norm(), 1e-6);
  EXPECT_NEAR((m.fk({{"slide", .3}}).at("slider").translation() - Vec3(.5, 0, .5)).norm(), 0, 1e-6);
}
TEST(URD05, MimicChain) {
  auto m = Model::load(resources / "urdf/r03.urdf");
  auto p = m.positions({{"main", .3}});
  EXPECT_NEAR(p["mimic"], .7, 1e-12);
  EXPECT_NEAR(p["mimic_chain"], .55, 1e-12);
}
TEST(URD06, MimicCycles) {
  auto text = readText(resources / "urdf/r03.urdf");
  EXPECT_THROW(
      importText(replace(text, "joint=\"main\" multiplier", "joint=\"mimic_chain\" multiplier")),
      std::runtime_error);
  EXPECT_THROW(importText(replace(text, "joint=\"main\" multiplier", "joint=\"mimic\" multiplier")),
               std::runtime_error);
}
TEST(URD07, SeparateGeometry) {
  std::string xml = "<robot name='g'><link name='base'><visual><origin xyz='2 0 0'/><geometry><box "
                    "size='1 1 1'/></geometry></visual><collision><origin xyz='4 0 "
                    "0'/><geometry><sphere radius='1'/></geometry></collision></link></robot>";
  auto a = importText(xml);
  EXPECT_EQ(a.visual[0].transform.translation().x(), 2);
  EXPECT_EQ(a.collision[0].transform.translation().x(), 4);
  EXPECT_EQ(a.sensor[0].shape->kind, Shape::Kind::Box);
  ModelOptions o;
  o.sensor_overrides["base"] = "collision";
  auto b = importText(xml, o);
  EXPECT_EQ(b.sensor[0].shape->kind, Shape::Kind::Sphere);
  EXPECT_NEAR(b.placed(b.sensor, pose({1, 0, 0}), "base", {})[0].transform.translation().x(), 5,
              1e-12);
}
TEST(URD08, PrimitiveDimensions) {
  EXPECT_NEAR(Shape::box({2, 4, 6})->ray({-5, 0, 0}, {1, 0, 0}, 100), 4, 1e-6);
  EXPECT_NEAR(Shape::sphere(1)->ray({-5, 0, 0}, {1, 0, 0}, 100), 4, 1e-6);
  EXPECT_NEAR(Shape::cylinder(1, 4)->ray({0, 0, -5}, {0, 0, 1}, 100), 3, 1e-6);
  EXPECT_THROW(Shape::box({0, 1, 1}), std::runtime_error);
  EXPECT_THROW(Shape::sphere(-1), std::runtime_error);
  EXPECT_THROW(Shape::cylinder(1, NAN), std::runtime_error);
}
TEST(URD09, MeshFormats) {
  for (auto ext : {"stl", "obj", "dae"}) {
    auto s = Shape::mesh(resources / (std::string("meshes/cube.") + ext), Vec3::Ones());
    EXPECT_LT((s->bounds.hi - s->bounds.lo - Vec3::Ones()).norm(), 1e-6);
    EXPECT_NEAR(s->ray({-5, 0, 0}, {1, 0, 0}, 100), 4.5, 1e-6);
    auto m = Model::load(resources / (std::string("urdf/r02_") + ext + ".urdf"));
    EXPECT_EQ(m.visual[0].shape->kind, Shape::Kind::Mesh);
  }
}
TEST(URD10, ResourceResolution) {
  auto p = resources / "meshes/cube.obj";
  EXPECT_EQ(resolveResource("../meshes/cube.obj", resources / "urdf"),
            std::filesystem::canonical(p));
  EXPECT_EQ(resolveResource(p.string(), "/"), std::filesystem::canonical(p));
  EXPECT_EQ(resolveResource("file://" + p.string(), "/"), std::filesystem::canonical(p));
  auto prefix = temporary("", ".dir");
  std::filesystem::remove(prefix);
  std::filesystem::create_directories(prefix / "share/example/meshes");
  std::filesystem::copy_file(p, prefix / "share/example/meshes/cube.obj");
  setenv("LWS_RESOURCE_PREFIX", prefix.c_str(), 1);
  EXPECT_EQ(sha256(resolveResource("package://example/meshes/cube.obj", "/")), sha256(p));
  EXPECT_THROW(resolveResource("package://example/../cube.obj", "/"), std::runtime_error);
  EXPECT_THROW(resolveResource("package://missing/cube.obj", "/"), std::runtime_error);
  unsetenv("LWS_RESOURCE_PREFIX");
  std::filesystem::remove_all(prefix);
}
TEST(URD12, NonUniformScale) {
  auto s = Shape::mesh(resources / "meshes/cube.obj", {2, 3, 4});
  EXPECT_LT((s->bounds.hi - s->bounds.lo - Vec3(2, 3, 4)).norm(), 1e-6);
  EXPECT_NEAR(s->ray({0, 0, -5}, {0, 0, 1}, 100), 3, 1e-6);
  for (Vec3 scale : {Vec3(0, 1, 1), Vec3(-1, 1, 1), Vec3(NAN, 1, 1)})
    EXPECT_THROW(Shape::mesh(resources / "meshes/cube.obj", scale), std::runtime_error);
}
TEST(URD13, MultipleElements) {
  auto xml = readText(resources / "urdf/r01.urdf");
  auto end = xml.find("</link>");
  xml.insert(end, "<visual><origin xyz='2 0 0'/><geometry><box size='1 1 "
                  "1'/></geometry></visual><collision><origin xyz='2 0 0'/><geometry><box size='1 "
                  "1 1'/></geometry></collision>");
  auto m = importText(xml);
  EXPECT_EQ(m.visual.size(), 5);
  EXPECT_EQ(m.collision.size(), 5);
  Scene s;
  s.sensor = m.placed(m.sensor, Pose::Identity(), m.root, {});
  s.rebuild();
  ASSERT_TRUE(s.ray({3, 0, 0}, {-1, 0, 0}, 10));
  EXPECT_NEAR(s.ray({3, 0, 0}, {-1, 0, 0}, 10)->distance, .5, 1e-6);
}
TEST(URD14, InertialAndExtensions) {
  auto xml = replace(readText(resources / "urdf/r01.urdf"), "</robot>",
                     "<gazebo><plugin name='ignored' filename='never_execute'/></gazebo></robot>");
  auto m = importText(xml);
  EXPECT_EQ(m.report["inertial"]["base_link"]["mass"], 10);
  EXPECT_EQ(m.report["inertial"]["base_link"]["dynamics_enabled"], false);
  EXPECT_EQ(m.report["ignored_extensions"][0], "gazebo");
}
TEST(URD15, InvalidTrees) {
  for (auto xml : {"not xml", "<robot name='x'><link name='a'/><link name='b'/></robot>",
                   "<robot name='x'><link name='a'/><link name='a'/></robot>",
                   "<robot name='x'><link name='a'/><joint name='j' type='fixed'><parent "
                   "link='a'/><child link='missing'/></joint></robot>",
                   "<robot name='x'><link name='a'/><link name='b'/><joint name='a' "
                   "type='fixed'><parent link='a'/><child link='b'/></joint><joint name='b' "
                   "type='fixed'><parent link='b'/><child link='a'/></joint></robot>"})
    EXPECT_THROW(importText(xml), std::runtime_error);
}
TEST(URD16, UnsupportedJoints) {
  for (auto type : {"floating", "planar", "unknown"})
    EXPECT_THROW(importText(replace(readText(resources / "urdf/r01.urdf"), "type=\"continuous\"",
                                    std::string("type=\"") + type + "\"")),
                 std::runtime_error);
}
TEST(URD17, MissingGeometry) {
  std::string xml = "<robot name='r'><link name='base'><visual><geometry><box size='1 1 "
                    "1'/></geometry></visual></link></robot>";
  EXPECT_THROW(importText(xml), std::runtime_error);
  ModelOptions o;
  o.visual_collision_fallback = true;
  EXPECT_EQ(importText(xml, o).report["collision_fallback"].size(), 1);
  EXPECT_THROW(importText(replace(xml, "<box size='1 1 1'/>", "<mesh filename='missing.stl'/>"), o),
               std::runtime_error);
}
TEST(URD20, UndrivenJoint) {
  auto c = config();
  auto xml = replace(readText(c.urdf), "</robot>",
                     "<link name='unused'/><joint name='undriven' type='continuous'><parent "
                     "link='base_link'/><child link='unused'/><axis xyz='0 0 1'/></joint></robot>");
  auto m = importText(xml);
  c.initial_joints["undriven"] = .7;
  Simulator s(c, m, {});
  advance(s, 2000, .1);
  EXPECT_NEAR(s.snapshot().joints.at("undriven"), .7, 1e-12);
  EXPECT_NE(s.model().report["undriven_joints"].dump().find("undriven"), std::string::npos);
}

TEST(GEO01, NearestSurface) {
  for (auto fixture : {box("f01", {5.005, 0, 1}, {.01, 10, 10}), box("f02", {5, 0, 1}, {2, 2, 2}),
                       box("f03", {2.005, 0, 1}, {.01, 10, 10})}) {
    auto s = scene({fixture});
    auto hit = s.ray({0, 0, 1}, {1, 0, 0}, 100);
    ASSERT_TRUE(hit);
    double expected = fixture.entity == "f01" ? 5 : fixture.entity == "f02" ? 4 : 2;
    EXPECT_NEAR(hit->distance, expected, 1e-3);
    EXPECT_EQ(hit->entity, fixture.entity);
  }
}
TEST(GEO02, ParallelAndBoundary) {
  auto s = scene({box("wall", {5.005, 0, 0}, {.01, 2, 2})});
  EXPECT_FALSE(s.ray({0, 0, 0}, {0, 1, 0}, 100));
  EXPECT_FALSE(s.ray({0, 0, 0}, {1e-14, 1, 0}, 100));
  EXPECT_TRUE(s.ray({0, 1, 1}, {1, 0, 0}, 100));
  auto mesh = Shape::mesh(resources / "meshes/cube.obj", Vec3::Ones());
  EXPECT_NEAR(mesh->ray({-5, .5, .5}, {1, 0, 0}, 100), 4.5, 1e-6);
}
TEST(GEO03, InteriorAndSurface) {
  auto b = Shape::box({2, 2, 2});
  EXPECT_NEAR(b->ray(Vec3::Zero(), Vec3::UnitX(), 100), 1, 1e-12);
  EXPECT_EQ(b->ray({1, 0, 0}, Vec3::UnitX(), 100), 0);
  EXPECT_TRUE(std::isinf(b->ray({1.001, 0, 0}, Vec3::UnitX(), 100)));
  EXPECT_NEAR(Shape::sphere(1)->ray(Vec3::Zero(), Vec3::UnitX(), 100), 1, 1e-12);
}
TEST(GEO04, BlindZoneOccludes) {
  auto l = lidar();
  l.min_range = .1;
  l.max_range = 10;
  for (double x : {.099, .1, 10.0, 10.001}) {
    auto s =
        scene({box("near", {x + .005, 0, 0}, {.01, 10, 10}), box("far", {20, 0, 0}, {1, 10, 10})});
    auto c = scan(s, l, Pose::Identity(), 1);
    if (x >= .1 && x <= 10)
      EXPECT_NEAR(number(c, 0, 0), x, 1e-3);
    else
      EXPECT_TRUE(std::isnan(number(c, 0, 0)));
  }
}
TEST(GEO05, OccludedObjects) {
  auto s = scene({box("near", {2.005, 0, 0}, {.01, 4, 4}), box("far", {5, 0, 0}, {1, 1, 1})});
  EXPECT_NEAR(s.ray(Vec3::Zero(), Vec3::UnitX(), 100)->distance, 2, 1e-6);
  s.sensor.pop_back();
  s.rebuild();
  EXPECT_NEAR(s.ray(Vec3::Zero(), Vec3::UnitX(), 100)->distance, 2, 1e-6);
}
TEST(GEO06, SelfFiltering) {
  Scene s;
  auto body = box("robot", {2, 0, 0}, {1, 1, 1});
  body.link = "base_link";
  auto shell = box("robot", {.2, 0, 0}, {.1, .1, .1});
  shell.link = "lidar_link";
  auto l = lidar();
  auto c = scan(s, l, Pose::Identity(), 1, {body, shell});
  EXPECT_NEAR(number(c, 0, 0), 1.5, 1e-6);
  l.filter = "robot";
  EXPECT_FALSE(scan(s, l, Pose::Identity(), 1, {body, shell}).dense);
  l.filter = "links";
  l.excluded_links = {"base_link"};
  EXPECT_NEAR(number(scan(s, l, Pose::Identity(), 1, {body, shell}), 0, 0), .15, 1e-6);
}
TEST(GEO07, RigidEquivariance) {
  std::mt19937_64 rng(41);
  std::uniform_real_distribution<double> d(-2, 2);
  for (int i = 0; i < 50; ++i) {
    auto g = box("wall", {5, 0, 0}, {1, 3, 3});
    Pose p = pose({d(rng), d(rng), d(rng)}, {d(rng), d(rng), d(rng)});
    g.transform = p * g.transform;
    auto s = scene({g});
    auto hit = s.ray(p.translation(), p.linear() * Vec3::UnitX(), 100);
    ASSERT_TRUE(hit);
    EXPECT_NEAR(hit->distance, 4.5, 1e-6);
  }
}
TEST(GEO08, BadMeshes) {
  for (auto text : {"", "v 0 0 0\nv 1 0 0\nv 2 0 0\nf 1 2 3", "v 0 0 0\nf 1 2 9",
                    "v nan 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3"}) {
    auto p = temporary(text, ".obj");
    EXPECT_THROW(Shape::mesh(p, Vec3::Ones()), std::runtime_error);
    std::filesystem::remove(p);
  }
}
TEST(GEO09, HeightDiscrimination) {
  auto s = scene({box("low", {3, 0, .2}, {1, 1, .4}), box("overhead", {3, 0, 2}, {1, 1, .2})});
  EXPECT_FALSE(s.ray({0, 0, 1}, Vec3::UnitX(), 10));
  EXPECT_EQ(s.ray({0, 0, .2}, Vec3::UnitX(), 10)->entity, "low");
  EXPECT_EQ(s.ray({0, 0, 2}, Vec3::UnitX(), 10)->entity, "overhead");
}
TEST(GEO11, ConfigAtomicity) {
  const auto text = replace(readText(resources / "config/demo.yaml"), "../urdf/r01.urdf",
                            (resources / "urdf/r01.urdf").string());
  for (auto invalid : {replace(text, "id: column", "id: ground"),
                       replace(text, "xyz: [5.005, 0, 1.5]", "xyz: [.nan, 0, 1.5]"),
                       replace(text, "type: box", "type: unknown")}) {
    auto p = temporary(invalid, ".yaml");
    Scene previous = scene({box("existing", {1, 0, 0}, {1, 1, 1})});
    EXPECT_THROW(Config::load(p, previous), std::runtime_error);
    EXPECT_EQ(previous.sensor[0].entity, "existing");
    std::filesystem::remove(p);
  }
}

TEST(MOT01, StraightAnalytic) {
  auto s = simulator();
  advance(s, 2000, 1);
  EXPECT_NEAR(s.snapshot().world_base.translation().x(), 10, 1e-3);
  EXPECT_NEAR(s.snapshot().world_base.translation().y(), 0, 1e-9);
  EXPECT_NEAR(yaw(s.snapshot().world_base), 0, 1e-9);
}
TEST(MOT02, ArcAnalytic) {
  auto s = simulator();
  advance(s, 2000, 1, .5);
  EXPECT_NEAR(s.snapshot().world_base.translation().x(), 2 * std::sin(5), 1e-3);
  EXPECT_NEAR(s.snapshot().world_base.translation().y(), 2 * (1 - std::cos(5)), 1e-3);
  EXPECT_NEAR(std::remainder(yaw(s.snapshot().world_base) - 5, 2 * pi), 0, 1e-4);
}
TEST(MOT03, WheelRates) {
  auto s = simulator();
  advance(s, 200, 1, .4);
  EXPECT_NEAR(s.snapshot().joint_velocities.at("left_wheel_joint"), 9, 1e-9);
  EXPECT_NEAR(s.snapshot().joint_velocities.at("right_wheel_joint"), 11, 1e-9);
  EXPECT_NEAR(s.snapshot().joints.at("left_wheel_joint"), 9, 1e-6);
  EXPECT_NEAR(s.snapshot().joints.at("right_wheel_joint"), 11, 1e-6);
}
TEST(MOT04, AccelerationLimits) {
  auto c = config();
  c.drive.unlimited_acceleration = false;
  auto s = simulator(c);
  s.pause(false);
  double old = 0;
  for (int i = 0; i < 800; ++i) {
    s.command({i < 400 ? 5.0 : -5.0, 8});
    s.tick();
    double v = s.snapshot().v;
    EXPECT_LE(std::abs(v), 1);
    EXPECT_LE(std::abs(v - old), .005 + 1e-10);
    old = v;
  }
  EXPECT_NEAR(s.snapshot().v, -.5, 1e-6);
}
TEST(MOT05, Timeout) {
  auto c = config();
  c.timeout = .5;
  c.drive.unlimited_acceleration = false;
  c.drive.acceleration = 10;
  auto s = simulator(c);
  s.pause(false);
  s.command({1, 0});
  for (int i = 0; i < 40; ++i)
    s.tick();
  s.pause(true);
  const auto before_pause = s.snapshot().stamp;
  std::this_thread::sleep_for(std::chrono::seconds(2));
  EXPECT_TRUE(s.tick().empty());
  EXPECT_EQ(s.snapshot().stamp, before_pause);
  EXPECT_EQ(s.diagnostics()["command_timeout"], false);
  s.pause(false);
  for (int i = 0; i < 261; ++i)
    s.tick();
  EXPECT_NEAR(s.snapshot().v, 0, 1e-9);
  EXPECT_EQ(s.diagnostics()["command_timeout"], true);
  auto stamp = s.snapshot().stamp;
  EXPECT_FALSE(s.command({NAN, 0}));
  s.pause(true);
  EXPECT_TRUE(s.tick().empty());
  EXPECT_EQ(s.snapshot().stamp, stamp);
}
TEST(MOT06, SweptThinWall) {
  for (double v : {1.0, 40.0}) {
    auto c = config();
    c.drive.max_v = 40;
    c.step_ns = 100000000;
    auto s = simulator(c, scene({box("thin", {2.005, 0, .5}, {.01, 4, 1})}));
    advance(s, 30, v);
    EXPECT_LE(s.snapshot().world_base.translation().x(), 1.600001);
    EXPECT_GE(s.snapshot().world_base.translation().x(), 1.59);
    EXPECT_FALSE(s.contacts().empty());
  }
}
TEST(MOT07, RotationalSweep) {
  auto c = config();
  c.drive.max_w = 10;
  c.step_ns = 100000000;
  auto s = simulator(c, scene({box("corner", {.49, 0, .3}, {.02, .04, .4})}));
  advance(s, 20, 0, 10);
  EXPECT_FALSE(s.contacts().empty());
  EXPECT_LT(std::abs(yaw(s.snapshot().world_base)), pi / 2);
}
TEST(MOT11, InvalidCommands) {
  auto c = config();
  c.timeout = .1;
  auto s = simulator(c);
  advance(s, 2, .2);
  for (Command bad : std::vector<Command>{{NAN, 0}, {0, INFINITY}, {0, 0, 1}, {0, 0, 0, 1}})
    EXPECT_FALSE(s.command(bad));
  Command bad;
  bad.frame = "map";
  EXPECT_FALSE(s.command(bad));
  bad.frame = "base_link";
  bad.stamp = s.snapshot().stamp + 100;
  EXPECT_FALSE(s.command(bad));
  advance(s, 50, 0);
  bad.stamp = 1;
  EXPECT_FALSE(s.command(bad));
  EXPECT_GE(s.diagnostics()["rejected_commands"].get<int>(), 7);
  auto timeout_test = simulator(c);
  advance(timeout_test, 1, .2);
  for (int i = 0; i < 25; ++i) {
    EXPECT_FALSE(timeout_test.command({NAN, 0}));
    timeout_test.tick();
  }
  EXPECT_EQ(timeout_test.diagnostics()["command_timeout"], true);
  EXPECT_EQ(timeout_test.snapshot().v, 0);
}
TEST(MOT12, SmoothTrajectorySteps) {
  double previous = INFINITY;
  for (double dt : {.01, .005, .0025}) {
    Pose p = Pose::Identity();
    for (int i = 0; i < int(10 / dt); ++i) {
      double t = (i + .5) * dt;
      p = integrate(p, 1 + .1 * std::cos(t), 0, dt);
    }
    const double error = std::abs(p.translation().x() - (10 + .1 * std::sin(10)));
    EXPECT_LT(error, 1e-3);
    EXPECT_LE(error, previous + 1e-12);
    previous = error;
  }
}
TEST(MOT13, GroundAndWheelContacts) {
  auto ground = box("ground", {0, 0, -.1}, {20, 20, .2});
  ground.support = true;
  auto c = config();
  auto s = simulator(c, scene({ground}));
  advance(s, 200, .5);
  EXPECT_NEAR(s.snapshot().world_base.translation().x(), .5, 1e-6);
  EXPECT_TRUE(s.contacts().empty());
  c.initial_pose = pose({0, 0, -.15});
  EXPECT_THROW(simulator(c, scene({ground})), std::runtime_error);
  c = config();
  auto wheel_wall = box("wheel_wall", {1.005, .25, .1}, {.01, .1, .15});
  auto w = simulator(c, scene({ground, wheel_wall}));
  advance(w, 1000, .5);
  EXPECT_FALSE(w.contacts().empty());
  EXPECT_LE(w.snapshot().world_base.translation().x(), .900001);
}

TEST(LID01, ScanSizes) {
  for (int rings : {16, 32, 64}) {
    auto l = lidar();
    l.columns = 1024;
    l.vertical_angles.resize(rings, 0);
    auto c = scan({}, l, Pose::Identity(), 123);
    EXPECT_EQ(c.height, rings);
    EXPECT_EQ(c.width, 1024);
    EXPECT_EQ(c.data.size(), size_t(rings) * 1024 * 24);
  }
}
TEST(LID02, DirectionsAndSeam) {
  auto l = lidar();
  l.columns = 1024;
  l.horizontal_start = -pi;
  l.horizontal_end = pi;
  l.vertical_angles = {-.2, -.01, .3};
  EXPECT_GT((l.direction(1, 0) - l.direction(1, 1023)).norm(), .001);
  EXPECT_NEAR(l.direction(2, 45).z(), std::sin(.3), 1e-12);
  l.horizontal_start = .1;
  l.horizontal_end = 1.2;
  EXPECT_GE(std::atan2(l.direction(0, 0).y(), l.direction(0, 0).x()), .1 - 1e-12);
  EXPECT_LT(std::atan2(l.direction(0, 1023).y(), l.direction(0, 1023).x()), 1.2);
}
TEST(LID03, Real3DSurfaces) {
  auto s = scene({box("bridge", {3, 0, 2}, {2, 4, .2}), box("box", {5, 0, .4}, {2, 2, .8})});
  auto l = lidar();
  l.vertical_angles = {-.2, 0, .4};
  auto c = scan(s, l, pose({0, 0, 1}), 1);
  EXPECT_FALSE(std::isnan(number(c, 0, 0)));
  EXPECT_TRUE(std::isnan(number(c, 1, 0)));
  EXPECT_FALSE(std::isnan(number(c, 2, 0)));
  EXPECT_NEAR(number(c, 2, 8) + 1, 1.9, 1e-3);
}
TEST(LID04, MountLocalCoordinates) {
  auto l = lidar();
  auto s = scene({box("wall", {5.005, 0, 0}, {.01, 20, 20})});
  auto cloud = scan(s, l, pose({.2, 0, .5}), 123);
  EXPECT_NEAR(number(cloud, 0, 0), 4.8, 1e-3);
  EXPECT_EQ(cloud.frame, "lidar_link");
  auto rotated = scan(s, l, pose({.2, 0, .5}, {0, 0, .3}), 123);
  EXPECT_NEAR(number(rotated, 0, 0), 4.8 / std::cos(.3), 1e-3);
}
TEST(LID05, Layout) {
  auto l = lidar();
  auto c = scan({}, l, Pose::Identity(), 1);
  EXPECT_EQ(c.point_step, 24);
  EXPECT_EQ(c.row_step, 24);
  EXPECT_EQ(c.data.size(), 24);
  EXPECT_EQ(c.data[18], 0);
  EXPECT_EQ(c.data[19], 0);
}
TEST(LID06, IndependentBytes) {
  auto l = lidar();
  l.columns = 4;
  l.vertical_angles = {0, .1};
  auto c = scan(scene({box("wall", {5.005, 0, 0}, {.01, 100, 100})}), l, Pose::Identity(), 999);
  EXPECT_EQ(c.data.size(), 192);
  for (size_t i = 0; i < 8; ++i) {
    EXPECT_EQ(c.data[i * 24 + 16] + 256 * c.data[i * 24 + 17], i / 4);
    EXPECT_EQ(c.data[i * 24 + 18], 0);
    EXPECT_EQ(c.data[i * 24 + 19], 0);
    EXPECT_FLOAT_EQ(number(c, i, 20), 0);
    EXPECT_FLOAT_EQ(number(c, i, 12), .5);
    EXPECT_NEAR(number(c, i, 0), 5, 1e-3);
  }
  EXPECT_EQ(c.stamp, 999);
}
TEST(LID07, NoReturnSlots) {
  auto l = lidar();
  l.columns = 4;
  auto empty = scan({}, l, Pose::Identity(), 1);
  EXPECT_FALSE(empty.dense);
  for (int i = 0; i < 4; ++i)
    for (int f : {0, 4, 8})
      EXPECT_TRUE(std::isnan(number(empty, i, f)));
  auto full = scan(scene({box("wall", {5, 0, 0}, {1, 100, 100})}), l, Pose::Identity(), 1);
  EXPECT_TRUE(full.dense);
  l.horizontal_end = 2 * pi;
  auto partial = scan(scene({box("wall", {5, 0, 0}, {1, 100, 100})}), l, Pose::Identity(), 1);
  EXPECT_FALSE(partial.dense);
  EXPECT_TRUE(std::isfinite(number(partial, 0, 0)));
  EXPECT_TRUE(std::isnan(number(partial, 2, 0)));
}
TEST(LID08, RangeBoundaries) {
  auto l = lidar();
  l.min_range = 1;
  l.max_range = 5;
  for (double x : {.999, 1.0, 5.0, 5.001}) {
    auto c = scan(scene({box("surface", {x + .005, 0, 0}, {.01, 2, 2}),
                         box("behind", {10, 0, 0}, {1, 2, 2})}),
                  l, Pose::Identity(), 1);
    EXPECT_EQ(std::isfinite(number(c, 0, 0)), x >= 1 && x <= 5);
  }
}
TEST(LID09, SnapshotTimes) {
  auto c = config();
  auto s = simulator(c);
  s.pause(false);
  s.command({1, .5});
  size_t frames = 0;
  for (int i = 0; i < 40; ++i)
    for (auto &e : s.tick())
      for (auto &cloud : e.clouds) {
        ++frames;
        EXPECT_EQ(cloud.stamp, e.snapshot.stamp);
        for (size_t n = 0; n < cloud.height * cloud.width; ++n)
          EXPECT_EQ(number(cloud, n, 20), 0);
      }
  EXPECT_EQ(frames, 2);
}
TEST(LID20, InvalidScanConfig) {
  auto l = lidar();
  l.vertical_angles.clear();
  EXPECT_THROW(l.validate(), std::runtime_error);
  l = lidar();
  l.vertical_angles = {NAN};
  EXPECT_THROW(l.validate(), std::runtime_error);
  l = lidar();
  l.rate = -1;
  EXPECT_THROW(l.validate(), std::runtime_error);
  l = lidar();
  l.max_range = l.min_range;
  EXPECT_THROW(l.validate(), std::runtime_error);
  l = lidar();
  l.vertical_angles.resize(65537);
  EXPECT_THROW(l.validate(), std::runtime_error);
  l = lidar();
  l.columns = 0;
  EXPECT_THROW(l.validate(), std::runtime_error);
  l = lidar();
  l.horizontal_end = 9;
  EXPECT_THROW(l.validate(), std::runtime_error);
}
TEST(LID21, IndependentLidars) {
  auto c = config();
  auto a = lidar();
  a.name = "a";
  a.topic = "a";
  a.link = "lidar_link";
  auto b = a;
  b.name = "b";
  b.topic = "b";
  b.link = "base_link";
  b.columns = 3;
  c.lidars = {a, b};
  auto s = simulator(c, scene({box("wall", {5.005, 0, .5}, {.01, 4, 4})}));
  s.pause(false);
  std::vector<Cloud> clouds;
  for (int i = 0; i < 20; ++i)
    for (auto &e : s.tick())
      for (auto &cloud : e.clouds)
        clouds.push_back(cloud);
  ASSERT_EQ(clouds.size(), 2);
  EXPECT_EQ(clouds[0].width, 1);
  EXPECT_EQ(clouds[1].width, 3);
  EXPECT_NE(clouds[0].frame, clouds[1].frame);
  EXPECT_NEAR(number(clouds[0], 0, 0), 4.8, 1e-3);
}
TEST(ODO01, InitialOrigin) {
  auto c = config();
  c.initial_pose = pose({3, 4, 0}, {0, 0, pi / 2});
  auto s = simulator(c);
  EXPECT_NEAR(s.snapshot().odom_base.translation().norm(), 0, 1e-12);
  advance(s, 200, 1);
  EXPECT_NEAR(s.snapshot().world_base.translation().y(), 5, 1e-6);
  EXPECT_NEAR(s.snapshot().odom_base.translation().x(), 1, 1e-6);
}
TEST(ODO02, BodyVelocity) {
  auto c = config();
  c.initial_pose = pose({0, 0, 0}, {0, 0, pi / 2});
  auto s = simulator(c);
  advance(s, 100, .5);
  EXPECT_NEAR(s.snapshot().v, .5, 1e-9);
  EXPECT_NEAR(s.snapshot().world_base.translation().x(), 0, 1e-9);
  EXPECT_NEAR(s.snapshot().world_base.translation().y(), .25, 1e-6);
}
TEST(ODO04, Covariance) {
  auto c = config();
  for (int i = 0; i < 6; ++i)
    c.pose_covariance[i * 7] = .1;
  c.pose_covariance[1] = c.pose_covariance[6] = .01;
  EXPECT_NO_THROW(c.validate());
  c.pose_covariance[0] = -1;
  EXPECT_THROW(c.validate(), std::runtime_error);
  c = config();
  c.twist_covariance[2] = 1;
  EXPECT_THROW(c.validate(), std::runtime_error);
}

TEST(URD03, MimicLimitsConstrainDriver) {
  auto m = Model::load(resources / "urdf/r03.urdf");
  m.joints.at("mimic").upper = .7;
  m.joints.at("mimic").lower = -.7;
  m.joints.at("mimic").velocity = .2;
  auto values = m.positions({{"main", 1}});
  EXPECT_NEAR(values["main"], .3, 1e-12);
  EXPECT_NEAR(values["mimic"], .7, 1e-12);
  auto next = m.advance({}, {{"main", 1}}, 1);
  EXPECT_NEAR(next["main"], .1, 1e-12);
  EXPECT_NEAR(next["mimic"], .3, 1e-12);
  m.joints.at("mimic").multiplier = -2;
  EXPECT_NO_THROW(m.positions({{"main", -.8}}));
  m.joints.at("mimic").multiplier = 0;
  m.joints.at("mimic").offset = 2;
  EXPECT_THROW(m.positions({}), std::runtime_error);
  m.joints.at("mimic").offset = .2;
  EXPECT_NEAR(m.positions({})["mimic"], .2, 1e-12);
}

TEST(COR08, FullConfigAndInvalidBindings) {
  auto original = readText(resources / "config/demo.yaml");
  original = replace(original, "../urdf/r01.urdf", (resources / "urdf/r01.urdf").string());
  auto valid = replace(original, "base_link: base_link",
                       "base_link: base_link\n  initial_joints: {left_wheel_joint: 0.2}\n  "
                       "sensor_overrides: {base_link: collision}");
  valid = replace(valid, "self_filter: sensor_link",
                  "self_filter: links\n    excluded_links: [lidar_link]");
  valid += "\nodom:\n  mode: ideal\n  pose_covariance: [";
  for (int i = 0; i < 36; ++i)
    valid += (i ? ", " : "") + std::string(i % 7 == 0 ? "0.1" : "0.0");
  valid += "]\n";
  auto file = temporary(valid, ".yaml");
  Scene output;
  auto loaded = Config::load(file, output);
  std::filesystem::remove(file);
  EXPECT_EQ(loaded.lidars[0].filter, "links");
  EXPECT_EQ(loaded.pose_covariance[0], .1);
  EXPECT_EQ(loaded.initial_joints.at("left_wheel_joint"), .2);
  for (const auto &bad : std::vector<std::string>{
           replace(original, "rings: 32", "rings: 31"), replace(original, "snapshot", "unknown_scan"),
           replace(original, "xyzirt_v1", "bad"), replace(original, "m_rad_s", "millimeter"),
           replace(original, "seed: 42", "seed: 42\n  seed: 43"),
           replace(original, "base_link: base_link", "base_link: absent"),
           replace(original, "link: lidar_link", "link: absent"),
           replace(original, "wheel_radius: 0.1", "wheel_radius: 0.2"),
           replace(original, "wheel_separation: 0.5", "wheel_separation: 0.6"),
           replace(original, "support_links: [left_wheel, right_wheel]",
                   "support_links: [absent]")}) {
    auto p = temporary(bad, ".yaml");
    EXPECT_THROW(
        {
          Scene s;
          auto c = Config::load(p, s);
          Simulator simulation(c, Model::load(c.urdf, c.model_options), s);
        },
        std::runtime_error);
    std::filesystem::remove(p);
  }
}

TEST(GEO11, MeshSceneAndSeparateRoles) {
  auto text = readText(resources / "config/demo.yaml");
  text = replace(text, "../urdf/r01.urdf", (resources / "urdf/r01.urdf").string());
  auto start = text.find("entities:");
  text.resize(start);
  text += "entities:\n - id: mesh\n   pose: {xyz: [5, 0, 1]}\n   visual: {type: mesh, uri: '" +
          (resources / "meshes/cube.obj").string() +
          "', scale: [1, 2, 3]}\n   collision: {type: sphere, radius: 1}\n   sensor_geometry: "
          "{type: box, size: [1, 2, 3]}\n";
  auto p = temporary(text, ".yaml");
  Scene s;
  auto c = Config::load(p, s);
  std::filesystem::remove(p);
  EXPECT_EQ(s.visual[0].shape->kind, Shape::Kind::Mesh);
  EXPECT_EQ(s.collision[0].shape->kind, Shape::Kind::Sphere);
  EXPECT_NEAR(s.ray({0, 0, 1}, {1, 0, 0}, 100)->distance, 4.5, 1e-6);
  EXPECT_EQ(c.resources.size(), 2);
}

TEST(URD21, EquivalentFootprintTrajectory) {
  auto a = config(), b = a;
  b.urdf = resources / "urdf/r01_footprint.urdf";
  auto sa = simulator(a), sb = simulator(b);
  advance(sa, 200, .5, .3);
  advance(sb, 200, .5, .3);
  EXPECT_LT((sa.snapshot().odom_base.matrix() - sb.snapshot().odom_base.matrix()).norm(), 1e-9);
  auto tf = sb.model().fk(sb.snapshot().joints).at("lidar_link");
  auto lidar_world = sb.snapshot().odom_root * tf;
  auto expected = sa.snapshot().odom_base * sa.model().fk(sa.snapshot().joints).at("lidar_link");
  EXPECT_LT((lidar_world.matrix() - expected.matrix()).norm(), 1e-9);
}
