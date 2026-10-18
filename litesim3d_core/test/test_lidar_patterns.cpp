#include "litesim3d/sim.hpp"
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <cstring>
#include <fstream>
using namespace litesim3d;
namespace {
Scene enclosure(double radius = 10) {
  Scene s;
  s.sensor = {{"enclosure", "", Shape::sphere(radius)}};
  s.rebuild();
  return s;
}
float field(const Cloud &cloud, size_t slot, size_t offset) {
  uint32_t word = 0;
  for (size_t i = 0; i < 4; ++i) word |= uint32_t(cloud.data.at(slot * 24 + offset + i)) << (8 * i);
  float result;
  std::memcpy(&result, &word, 4);
  return result;
}
Vec3 point(const Cloud &cloud, size_t slot) {
  return {field(cloud, slot, 0), field(cloud, slot, 4), field(cloud, slot, 8)};
}
LidarConfig pattern(const std::string &name) {
  LidarConfig c;
  c.vertical_angles = {0}; c.columns = 2000; c.scan_mode = "rolling";
  c.scan_pattern = name; c.pattern_hz = 37;
  return c;
}
Cloud frame(const Scene &s, const LidarConfig &c, uint64_t index, unsigned threads = 1) {
  return scanTrajectory(s, c, index * 100000000, [](uint64_t) { return Pose::Identity(); }, {}, 42, index, threads);
}
Config load(const YAML::Node &yaml) {
  const auto path = std::filesystem::temp_directory_path() /
    ("lws-pattern-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".yaml");
  { std::ofstream out(path); out << yaml; }
  Scene s;
  try {
    auto config = Config::load(path, s);
    std::filesystem::remove(path);
    return config;
  } catch (...) { std::filesystem::remove(path); throw; }
}
YAML::Node configYaml() {
  auto n = YAML::LoadFile(std::string(SIM_RESOURCES) + "/config/demo.yaml");
  n["robot"]["urdf"] = std::string(SIM_RESOURCES) + "/urdf/r01.urdf";
  return n;
}
}
TEST(LidarPattern, StationarySpinningRepeatsButNoiseChangesReturns) {
  auto c = pattern("spinning"); c.vertical_angles = {-.1, .1};
  auto s = enclosure();
  EXPECT_EQ(frame(s, c, 0).data, frame(s, c, 1).data);
  c.range_sigma = .01;
  EXPECT_NE(frame(s, c, 0).data, frame(s, c, 1).data);
  EXPECT_EQ(frame(s, c, 3).data, frame(s, c, 3, 4).data);
}
TEST(LidarPattern, SphericalFramesChangeWithinPhysicalAngularBounds) {
  auto c = pattern("spherical");
  c.horizontal_start = -.7; c.horizontal_end = .4;
  c.vertical_min = -7 * pi / 180; c.vertical_max = 52 * pi / 180;
  auto s = enclosure(); auto a = frame(s, c, 0); auto b = frame(s, c, 1);
  EXPECT_NE(a.data, b.data);
  EXPECT_EQ(b.data, frame(s, c, 1, 4).data);
  double low = 1, high = -1;
  for (size_t i = 0; i < c.columns; ++i) {
    const auto p = point(b, i);
    const double az = std::atan2(p.y(), p.x()), el = std::asin(p.z() / p.norm());
    EXPECT_NEAR(p.norm(), 10, 2e-6);
    EXPECT_GE(az, c.horizontal_start - 1e-6); EXPECT_LE(az, c.horizontal_end + 1e-6);
    EXPECT_GE(el, c.vertical_min - 1e-6); EXPECT_LE(el, c.vertical_max + 1e-6);
    low = std::min(low, el); high = std::max(high, el);
    EXPECT_NEAR(field(b, i, 20), i / (c.rate * c.columns), 1e-8);
    EXPECT_EQ(b.data[i * 24 + 16], 0); EXPECT_EQ(b.data[i * 24 + 17], 0);
  }
  EXPECT_LT(low, c.vertical_min + .001); EXPECT_GT(high, c.vertical_max - .001);
}
TEST(LidarPattern, RosetteIsACircularConeNotARectangularFov) {
  auto c = pattern("rosette"); c.cone_half_angle = 19.2 * pi / 180;
  auto s = enclosure(); auto a = frame(s, c, 0); auto b = frame(s, c, 1);
  EXPECT_NE(a.data, b.data); EXPECT_EQ(b.data, frame(s, c, 1, 4).data);
  double low = 1, high = 0;
  for (size_t i = 0; i < c.columns; ++i) {
    const auto p = point(b, i);
    const double angle = std::acos(p.x() / p.norm());
    EXPECT_NEAR(p.norm(), 10, 2e-6);
    EXPECT_LE(angle, c.cone_half_angle + 1e-6);
    low = std::min(low, angle); high = std::max(high, angle);
  }
  EXPECT_LT(low, c.cone_half_angle * .05); EXPECT_GT(high, c.cone_half_angle * .99);
}
TEST(LidarPattern, NonRepeatingFramePackingKeepsContinuousTrajectory) {
  for (const auto &name : {"spherical", "rosette"}) {
    auto whole = pattern(name), half = whole;
    half.rate *= 2; half.columns /= 2;
    for (size_t i = 0; i < half.columns; ++i) {
      EXPECT_LT((whole.direction(0, i, 4) - half.direction(0, i, 8)).norm(), 1e-10);
      EXPECT_LT((whole.direction(0, half.columns + i, 4) - half.direction(0, i, 9)).norm(), 1e-10);
    }
  }
}
TEST(LidarPattern, RangeAndOcclusionApplyToEveryPattern) {
  for (const auto &name : {"spinning", "spherical", "rosette"}) {
    auto c = pattern(name); c.columns = 1000;
    c.min_range = 1; c.max_range = 9;
    auto far = frame(enclosure(), c, 0);
    for (size_t i = 0; i < c.columns; ++i) EXPECT_TRUE(std::isnan(field(far, i, 0)));
    c.max_range = 20;
    auto s = enclosure(); s.sensor.push_back({"near", "", Shape::sphere(.5)}); s.rebuild();
    auto blocked = frame(s, c, 0);
    for (size_t i = 0; i < c.columns; ++i) EXPECT_TRUE(std::isnan(field(blocked, i, 0)));
  }
}
TEST(LidarPattern, InvalidCombinationsAreRejected) {
  auto c = pattern("unknown"); EXPECT_THROW(c.validate(), std::runtime_error);
  c = pattern("spherical"); c.vertical_angles = {0, .1}; EXPECT_THROW(c.validate(), std::runtime_error);
  c = pattern("spherical"); c.ring_delays = {0}; EXPECT_THROW(c.validate(), std::runtime_error);
  c = pattern("spherical"); c.vertical_min = c.vertical_max; EXPECT_THROW(c.validate(), std::runtime_error);
  c = pattern("spherical"); c.columns = 1; EXPECT_THROW(c.validate(), std::runtime_error);
  c = pattern("rosette"); c.cone_half_angle = pi; EXPECT_THROW(c.validate(), std::runtime_error);
  c = pattern("rosette"); c.horizontal_start = 0; EXPECT_THROW(c.validate(), std::runtime_error);
}
TEST(LidarPattern, YamlAndManifestRecordResolvedPattern) {
  auto n = configYaml(); auto l = n["lidars"][0];
  l["scan_pattern"] = "spherical"; l.remove("vertical_angles_deg"); l["rings"] = 1;
  l["vertical_min_deg"] = -7; l["vertical_max_deg"] = 52; l["pattern_hz"] = 37;
  l["publish_range_boundary"] = true;
  auto c = load(n);
  EXPECT_EQ(c.lidars[0].scan_pattern, "spherical");
  EXPECT_NEAR(c.lidars[0].vertical_min, -7 * pi / 180, 1e-12);
  Simulator sim(c, Model::load(c.urdf), {});
  auto recorded = sim.manifest()["config"]["lidars"][0];
  EXPECT_EQ(recorded["scan_pattern"], "spherical"); EXPECT_EQ(recorded["pattern_hz"], 37);
  EXPECT_EQ(recorded["publish_range_boundary"], true);
  l["cone_half_angle_deg"] = 20; EXPECT_THROW(load(n), std::runtime_error);
  l.remove("cone_half_angle_deg"); l["scan_pattern"] = "spinning";
  l["vertical_angles_deg"].push_back(0); EXPECT_THROW(load(n), std::runtime_error);
}
TEST(LidarBoundary, EmptyAndOutOfRangeRaysHaveExactSyntheticEndpoints) {
  for (const auto &name : {"spinning", "spherical", "rosette"}) {
    auto c = pattern(name); c.max_range = 9;
    for (const auto &s : {Scene{}, enclosure(10)}) {
      auto raw = frame(s, c, 2);
      EXPECT_TRUE(raw.range_boundary.empty());
      c.publish_range_boundary = true;
      auto cloud = frame(s, c, 2);
      EXPECT_EQ(raw.data, cloud.data);
      EXPECT_EQ(cloud.range_boundary, frame(s, c, 2, 4).range_boundary);
      auto boundary = cloud; boundary.data = cloud.range_boundary;
      ASSERT_EQ(boundary.data.size(), cloud.data.size());
      for (size_t i = 0; i < c.columns; ++i) {
        EXPECT_TRUE(std::isnan(field(cloud, i, 0)));
        EXPECT_LT((point(boundary, i) - 9 * c.direction(0, i, 2)).norm(), 1e-6);
        EXPECT_NEAR(point(boundary, i).norm(), 9, 1e-6);
        EXPECT_EQ(field(boundary, i, 12), 0);
        EXPECT_EQ(field(boundary, i, 20), field(cloud, i, 20));
      }
      c.publish_range_boundary = false;
    }
  }
}
TEST(LidarBoundary, ReturnsBlindOcclusionAndDropoutNeverCreateFarEndpoints) {
  for (const auto &name : {"spinning", "spherical", "rosette"}) {
    for (int scenario = 0; scenario < 5; ++scenario) {
      auto c = pattern(name); c.publish_range_boundary = true;
      c.min_range = 1; c.max_range = 20;
      auto s = enclosure(10);
      if (scenario == 1) { s.sensor.push_back({"near", "", Shape::sphere(.5)}); s.rebuild(); }
      if (scenario == 2) c.dropout = 1;
      if (scenario == 3) c.range_bias = 20;
      if (scenario == 4) c.max_range = 10; // A true hit exactly at the limit is still a return.
      auto cloud = frame(s, c, 0); auto boundary = cloud; boundary.data = cloud.range_boundary;
      for (size_t i = 0; i < c.columns; ++i) {
        EXPECT_TRUE(std::isnan(field(boundary, i, 0)));
        EXPECT_EQ(std::isfinite(field(cloud, i, 0)), scenario == 0 || scenario == 4);
      }
    }
  }
}
TEST(LidarBoundary, AuxiliaryTopicAndMemoryAreIncludedInBudgets) {
  auto c = load(configYaml()); auto duplicate = c.lidars[0];
  c.lidars[0].publish_range_boundary = true;
  duplicate.name = "second"; duplicate.topic = c.lidars[0].topic + "/range_boundary";
  c.lidars.push_back(duplicate);
  EXPECT_THROW(c.validate(), std::runtime_error);
  std::reverse(c.lidars.begin(), c.lidars.end());
  EXPECT_THROW(c.validate(), std::runtime_error);
  c = load(configYaml()); c.lidars[0].publish_range_boundary = true;
  ImuConfig imu; imu.topic = c.lidars[0].topic + "/range_boundary"; c.imus.push_back(imu);
  EXPECT_THROW(c.validate(), std::runtime_error);
  c = load(configYaml()); c.lidars[0].columns = 65536; c.lidars[0].vertical_angles.assign(64, 0);
  EXPECT_NO_THROW(c.validate());
  c.lidars[0].publish_range_boundary = true;
  EXPECT_THROW(c.validate(), std::runtime_error);
}
TEST(LidarBoundary, BufferedStepBudgetCountsAuxiliaryClouds) {
  auto c = load(configYaml()); c.lidars[0].columns = 65536;
  c.lidars[0].vertical_angles = {0}; c.lidars[0].publish_range_boundary = true;
  // 100 + 1 frames fit in 512 MiB individually; 2 robots with auxiliary data exceed it.
  World world(c, Model::load(c.urdf), {}, {{"r0", pose()}, {"r1", pose({3, 0, 0})}});
  world.pause(true); std::vector<std::vector<Event>> batches;
  auto result = world.singleStep("auxiliary-budget", 2000, batches);
  EXPECT_FALSE(result.success); EXPECT_EQ(world.robot(0).snapshot().stamp, 1u);
  // The single-robot budget includes the same doubling at 200 + 1 frames.
  Simulator sim(c, Model::load(c.urdf), {}); sim.pause(true); std::vector<Event> events;
  EXPECT_FALSE(sim.singleStep("auxiliary-budget", 4000, events).success);
  EXPECT_EQ(sim.snapshot().stamp, 1u);
}
