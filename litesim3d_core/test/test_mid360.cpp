#include "litesim3d/sim.hpp"
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <cstring>
#include <fstream>
using namespace litesim3d;
namespace {
const std::string expected_hash = "aa1fc08b6a4400608dbd6ee832b7ea3a9c3c37197e734f60f58fe5abf762269a";
std::filesystem::path reference() { return std::filesystem::path(SIM_RESOURCES) / "lidar_patterns/mid360.csv"; }
std::shared_ptr<const LidarPatternTable> table() {
  static auto value = LidarPatternTable::load(reference()); return value;
}
LidarConfig mid360() {
  LidarConfig c; c.scan_pattern = "livox_mid360"; c.pattern_table = table();
  c.vertical_angles = {0}; c.columns = 20000; c.scan_mode = "rolling";
  c.vertical_min = -7 * pi / 180; c.vertical_max = 52 * pi / 180;
  c.filter = "robot"; return c;
}
float field(const std::vector<uint8_t> &data, size_t i, size_t offset = 0) {
  uint32_t bits = 0;
  for (int k = 0; k < 4; ++k) bits |= uint32_t(data.at(24 * i + offset + k)) << (8 * k);
  float value; std::memcpy(&value, &bits, 4); return value;
}
Vec3 point(const std::vector<uint8_t> &data, size_t i) { return {field(data,i),field(data,i,4),field(data,i,8)}; }
Cloud sample(const Scene &s, const LidarConfig &c, uint64_t index = 0, unsigned threads = 1) {
  return scanTrajectory(s,c,1,[](uint64_t) { return Pose::Identity(); },{},42,index,threads);
}
Scene enclosure(double radius) {
  Scene s; s.sensor = {{"enclosure","",Shape::sphere(radius)}}; s.rebuild(); return s;
}
YAML::Node yaml() {
  auto n = YAML::LoadFile(std::string(SIM_RESOURCES) + "/config/demo.yaml");
  n["robot"]["urdf"] = std::string(SIM_RESOURCES) + "/urdf/r01.urdf";
  auto l = n["lidars"][0]; l["scan_pattern"] = "livox_mid360";
  l["pattern_file"] = reference().string(); l["columns"] = 20000; l["rings"] = 1;
  l.remove("vertical_angles_deg"); return n;
}
struct Temp {
  std::filesystem::path path = std::filesystem::temp_directory_path() /
      ("lws-mid360-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  ~Temp() { std::filesystem::remove(path); }
  void write(const std::string &data) { std::ofstream out(path); out << data; }
};
Config load(const YAML::Node &n) {
  Temp t; t.write(YAML::Dump(n)); Scene s; return Config::load(t.path,s);
}
}
TEST(Mid360, OfficialAssetAndZenithConvention) {
  ASSERT_EQ(table()->samples.size(),800000u);
  EXPECT_EQ(table()->source_sha256,expected_hash);
  // Explicit upstream samples: catches swapped axes, degree/radian and zenith sign mistakes.
  const double az[] = {268.99,269,269,269}, zen[] = {37.838,40.825,43.581,46.152};
  for (size_t i = 0; i < 4; ++i) {
    const auto &p = table()->samples[i].direction;
    EXPECT_NEAR(std::atan2(p.y(),p.x()) * 180 / pi + 360,az[i],1e-10);
    EXPECT_NEAR(std::asin(p.z()) * 180 / pi,90-zen[i],1e-10);
    EXPECT_NEAR(p.norm(),1,1e-12);
  }
}
TEST(Mid360, SingleFrameUsesEverySourceRayWithoutAddingPoints) {
  auto c = mid360(); c.publish_range_boundary = true;
  auto cloud = sample(enclosure(10),c);
  ASSERT_EQ(cloud.width,20000u); ASSERT_EQ(cloud.height,1u);
  size_t valid = 0, cropped = 0;
  for (size_t i = 0; i < 20000; ++i) {
    const auto &ref = table()->samples[i];
    if (ref.elevation >= c.vertical_min && ref.elevation <= c.vertical_max) {
      EXPECT_LT((point(cloud.data,i) - ref.direction * 10).norm(),1.2e-6); ++valid;
    } else { EXPECT_TRUE(std::isnan(field(cloud.data,i))); ++cropped; }
    EXPECT_TRUE(std::isnan(field(cloud.range_boundary,i)));
    EXPECT_NEAR(field(cloud.data,i,20),i * .000005,1e-8);
  }
  EXPECT_GT(valid,19900u); EXPECT_GT(cropped,0u);
  EXPECT_EQ(cloud.data,sample(enclosure(10),c,0,4).data);
}
TEST(Mid360, FramePackingPhaseAndFourSecondCycleAreExplicit) {
  auto full = mid360(), half = full; half.rate = 20; half.columns = 10000;
  full.vertical_min = half.vertical_min = -pi/2; full.vertical_max = half.vertical_max = pi/2;
  for (size_t i = 0; i < 10000; ++i) {
    EXPECT_LT((full.direction(0,i,3)-half.direction(0,i,6)).norm(),1e-12);
    EXPECT_LT((full.direction(0,10000+i,3)-half.direction(0,i,7)).norm(),1e-12);
    EXPECT_LT((full.direction(0,i,40)-full.direction(0,i,0)).norm(),1e-12);
  }
  EXPECT_GT((full.direction(0,100,0)-full.direction(0,100,1)).norm(),.001);
  EXPECT_EQ(full.offsetNs(0,19999),99995000u);
  EXPECT_EQ(half.offsetNs(0,9999),49995000u);
  EXPECT_LT((full.direction(0,19999,39)-table()->samples.back().direction).norm(),1e-12);
}
TEST(Mid360, CroppingPreservesSlotsAndNeverCreatesBoundaryOutsideFov) {
  auto c = mid360(); c.publish_range_boundary = true; c.max_range = 25;
  c.vertical_min = -5 * pi/180; c.vertical_max = 15 * pi/180;
  // Cross the +/-pi seam to test an angular mask, not a stretched pattern.
  c.horizontal_start = 170*pi/180; c.horizontal_end = 190*pi/180;
  auto cloud = sample({},c); size_t emitted = 0;
  for (size_t i = 0; i < 20000; ++i) {
    const auto &ref = table()->samples[i];
    const bool inside = ref.azimuth >= c.horizontal_start && ref.azimuth < c.horizontal_end &&
                        ref.elevation >= c.vertical_min && ref.elevation <= c.vertical_max;
    EXPECT_EQ(std::isfinite(field(cloud.range_boundary,i)),inside);
    EXPECT_TRUE(std::isnan(field(cloud.data,i)));
    if (inside) { EXPECT_LT((point(cloud.range_boundary,i)-25*ref.direction).norm(),2e-6); ++emitted; }
  }
  EXPECT_GT(emitted,10u); EXPECT_LT(emitted,2000u);
}
TEST(Mid360, RangeOcclusionDropoutAndNoiseRetainPhysicalDirections) {
  auto c = mid360(); c.min_range = 1; c.max_range = 9; c.publish_range_boundary = true;
  auto far = sample(enclosure(10),c);
  auto near = sample(enclosure(.5),c);
  c.max_range = 20; c.dropout = 1;
  auto dropped = sample(enclosure(10),c);
  c.dropout = 0; c.range_sigma = .02;
  auto noisy = sample(enclosure(10),c,2);
  EXPECT_EQ(noisy.data,sample(enclosure(10),c,2,4).data);
  for (size_t i = 0; i < 20000; ++i) {
    EXPECT_TRUE(std::isnan(field(far.data,i)));
    EXPECT_TRUE(std::isnan(field(near.data,i))); EXPECT_TRUE(std::isnan(field(near.range_boundary,i)));
    EXPECT_TRUE(std::isnan(field(dropped.data,i))); EXPECT_TRUE(std::isnan(field(dropped.range_boundary,i)));
    if (std::isfinite(field(noisy.data,i)))
      EXPECT_LT((point(noisy.data,i).normalized()-c.direction(0,i,2)).norm(),1e-7);
  }
}
TEST(Mid360, MovingSensorStillUsesPerPointTime) {
  auto c = mid360(); c.horizontal_start = -.5; c.horizontal_end = .5;
  c.vertical_min = -.1; c.vertical_max = .4;
  Scene s; s.sensor = {{"wall","",Shape::box({.2,100,100}),pose({5.1,0,0})}}; s.rebuild();
  auto cloud = scanTrajectory(s,c,0,[](uint64_t t) { return pose({2*t*1e-9,0,0}); });
  size_t hits = 0;
  for (size_t i = 0; i < 20000; ++i) if (std::isfinite(field(cloud.data,i))) {
    EXPECT_NEAR(field(cloud.data,i),5 - 2 * i * .000005,1e-6); ++hits;
  }
  EXPECT_GT(hits,100u);
}
TEST(Mid360, YamlLoadsSharedResourceAndManifestRecordsProvenance) {
  auto n = yaml(); auto second = YAML::Clone(n["lidars"][0]);
  second["name"] = "second"; second["topic"] = "second_points"; n["lidars"].push_back(second);
  auto c = load(n); EXPECT_EQ(c.lidars[0].pattern_table,c.lidars[1].pattern_table);
  EXPECT_NEAR(c.lidars[0].vertical_min,-7*pi/180,1e-12);
  EXPECT_NEAR(c.lidars[0].vertical_max,52*pi/180,1e-12);
  Simulator sim(c,Model::load(c.urdf),{}); auto record = sim.manifest()["config"]["lidars"][0];
  EXPECT_EQ(record["pattern_sha256"],expected_hash); EXPECT_EQ(record["pattern_samples"],800000);
  EXPECT_EQ(record["pattern_cycle_s"],4.0); EXPECT_EQ(record["scan_pattern"],"livox_mid360");
  EXPECT_EQ(c.resources[std::filesystem::canonical(reference()).string()],expected_hash);
}
TEST(Mid360, RejectsWrongPointRateMissingTableAndIncompatibleParameters) {
  auto c = mid360(); c.columns = 40000; EXPECT_THROW(c.validate(),std::runtime_error);
  c = mid360(); c.pattern_table.reset(); EXPECT_THROW(c.validate(),std::runtime_error);
  auto n = yaml(); n["lidars"][0]["pattern_hz"] = 100; EXPECT_THROW(load(n),std::runtime_error);
  n = yaml(); n["lidars"][0]["cone_half_angle_deg"] = 20; EXPECT_THROW(load(n),std::runtime_error);
  n = yaml(); n["lidars"][0]["scan_pattern"] = "spherical"; EXPECT_THROW(load(n),std::runtime_error);
  n = yaml(); n["lidars"][0].remove("pattern_file"); EXPECT_THROW(load(n),std::runtime_error);
}
TEST(Mid360, RejectsMalformedTablesAndHonorsLoadBudgets) {
  const std::string header = "Time/s,Azimuth/deg,Zenith/deg\n";
  for (const std::string &body : {"", "2,0,90\n", "1,nan,90\n", "1,360.1,90\n",
                                "1,0,-1\n", "1,0,90,extra\n", "1,0,90\n1,10,90\n"}) {
    Temp t; t.write(header+body); EXPECT_THROW(LidarPatternTable::load(t.path),std::runtime_error);
  }
  Temp t; t.write("time,az,zen\n1,0,90\n"); EXPECT_THROW(LidarPatternTable::load(t.path),std::runtime_error);
  auto control = std::make_shared<LoadControl>(); control->byte_budget = 100;
  EXPECT_THROW(LidarPatternTable::load(reference(),control),std::runtime_error);
  control = std::make_shared<LoadControl>(); control->cancelled = [] { return true; };
  EXPECT_THROW(LidarPatternTable::load(reference(),control),std::runtime_error);
}
