#include <litesim3d/sensor_plugin.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gtest/gtest.h>
#include <cstring>
#include <set>

namespace lp = litesim3d::plugins;
using namespace litesim3d;
namespace {
const std::vector<std::string> classes = {"litesim3d_lidar/Lidar", "litesim3d_imu/Imu",
    "litesim3d_rgb_camera/RgbCamera", "litesim3d_depth_camera/DepthCamera", "litesim3d_rgbd_camera/RgbdCamera"};
Json config(const std::string &cls) {
  return lp::readParameterFile(std::filesystem::path(ament_index_cpp::get_package_share_directory(
      cls.substr(0,cls.find('/')))) / "config/parameters.yaml");
}
lp::Initialization init(std::string name = "sensor") {
  lp::Initialization i; i.instance = std::move(name); i.seed = 17;
  i.model.base_link = "base_link"; i.model.links = {"base_link","lidar_link","imu_link","camera_optical_frame"}; return i;
}
lp::SensorContext context(const Scene &s) {
  return {s,{},Pose::Identity(),[](uint64_t) { return Pose::Identity(); },{},100000001,100000001,0,1};
}
float value(const std::vector<uint8_t> &data, size_t offset, bool big = false) {
  uint32_t bits = 0; for (unsigned j = 0; j < 4; ++j) bits |= uint32_t(data.at(offset+j)) << (8*(big ? 3-j : j));
  float f; std::memcpy(&f,&bits,4); return f;
}
void put(std::vector<uint8_t> &data, size_t offset, float f, bool big = false) {
  uint32_t bits; std::memcpy(&bits,&f,4);
  for (unsigned j = 0; j < 4; ++j) data.at(offset+j) = uint8_t(bits >> (8*(big ? 3-j : j)));
}
lp::ImageFrame image(const std::string &kind, uint64_t stamp = 1, bool big = false) {
  lp::ImageFrame f; f.stamp_ns = stamp; f.width = f.height = 16; f.depth_big_endian = big;
  if (kind != "depth") f.color.assign(16*16*3,100);
  if (kind != "rgb") {
    f.depth.resize(16*16*4);
    for (size_t i = 0; i < f.depth.size(); i+=4) put(f.depth,i,2,big);
  }
  return f;
}
Model model() {
  Model m; m.root = "base_link"; m.links = {"base_link","lidar_link","imu_link","camera_optical_frame"};
  m.report = {{"undriven_joints",Json::array()},{"resources",Json::object()}};
  m.collision = {{"robot","base_link",Shape::box({.2,.2,.2}),pose({0,0,.2})}};
  for (const auto &name : {"lidar_link","imu_link","camera_optical_frame"}) {
    Joint j; j.name = std::string(name)+"_mount"; j.parent = m.root; j.child = name; j.type = "fixed";
    j.origin = pose({.3,0,.4}); m.joints[j.name] = j; m.order.push_back(j.name);
  }
  m.sensor = m.visual = m.collision; return m;
}
Config base() {
  Config c; c.drive.model = "multicopter"; c.drive.support_links.clear(); c.timeout = 100;
  c.drive.unlimited_acceleration = true; return c;
}
lp::Declaration declaration(size_t cls, const std::string &name, Json p = Json::object()) {
  return {"sensor",name,classes.at(cls),lp::merge(config(classes.at(cls)),p),{}};
}
std::vector<Event> advance(Simulator &sim, unsigned n) {
  sim.pause(true); std::vector<Event> result;
  auto status = sim.singleStep(std::to_string(sim.stepIndex()),n,result);
  EXPECT_TRUE(status.success) << status.message; return result;
}
}
TEST(SensorPlugins, AllFiveInstalledAndCompleteDefaultFiles) {
  lp::Loader loader;
  for (const auto &cls : classes) {
    SCOPED_TRACE(cls);
    auto plugin = loader.sensor(cls); const auto defaults = plugin->defaults(), file = config(cls);
    ASSERT_EQ(defaults.size(),file.size());
    for (auto it = defaults.begin(); it != defaults.end(); ++it) {
      ASSERT_TRUE(file.contains(it.key())) << it.key();
      if (!it.value().is_null()) EXPECT_EQ(it.value(),file.at(it.key())) << it.key();
    }
    plugin->initialize(init(),file); EXPECT_EQ(plugin->description().name,"sensor");
  }
  EXPECT_THROW(loader.sensor("missing/Sensor"),std::runtime_error);
  EXPECT_THROW(loader.sensor("litesim3d_diff_drive/DifferentialDrive"),std::runtime_error);
}
TEST(SensorPlugins, ValidationAndLoaderLifetime) {
  std::shared_ptr<lp::SensorPlugin> held;
  { lp::Loader loader; held = loader.sensor(classes[0]); }
  auto p = config(classes[0]); p["columns"] = 2.5; EXPECT_THROW(held->initialize(init(),p),std::runtime_error);
  p = config(classes[0]); p["link"] = "missing"; EXPECT_THROW(held->initialize(init(),p),std::runtime_error);
  p = config(classes[0]); p.erase("topic"); EXPECT_THROW(held->initialize(init(),p),std::runtime_error);
  p = config(classes[0]); p["unsupported"] = 1; EXPECT_THROW(held->initialize(init(),p),std::runtime_error);
  p = config(classes[0]); p["excluded_links"] = {"missing"}; EXPECT_THROW(held->initialize(init(),p),std::runtime_error);
  p = config(classes[0]); p["columns"] = 4; held->initialize(init(),p);
  Scene scene; scene.rebuild(); auto c = context(scene); lp::SensorOutput output;
  EXPECT_NO_THROW(held->step(c,output)); ASSERT_EQ(output.clouds.size(),1u);
}
TEST(SensorPlugins, LidarUsesTruthTrajectoryRangeAndDeterministicNoise) {
  Scene s; s.sensor = {{"wall","",Shape::box({.2,20,20}),pose({3,0,0})}}; s.rebuild();
  lp::Loader loader; auto plugin = loader.sensor(classes[0]);
  auto p = config(classes[0]); p["columns"] = 4; p["scan_mode"] = "rolling";
  p["horizontal_start_deg"] = 0; p["horizontal_end_deg"] = .00001;
  plugin->initialize(init(),p);
  auto c = context(s); c.stamp_ns = 1; c.sample_index = 2;
  c.world_sensor = [](uint64_t stamp) { return pose({double(stamp-1)*1e-9,0,0}); };
  lp::SensorOutput output; plugin->step(c,output);
  ASSERT_EQ(output.clouds.size(),1u); const auto &cloud = output.clouds[0];
  EXPECT_EQ(cloud.frame_index,2u); EXPECT_EQ(cloud.stamp,1u);
  for (size_t i = 0; i < 4; ++i) EXPECT_NEAR(value(cloud.data,i*24),2.9-i*.025,1e-5);
  EXPECT_LE(cloud.completion_stamp,c.completion_ns);
  p["range_sigma_m"] = .03; plugin->initialize(init(),p);
  lp::SensorOutput a,b; plugin->step(c,a); plugin->reset(1); plugin->step(c,b);
  EXPECT_EQ(a.clouds[0].data,b.clouds[0].data); EXPECT_NE(a.clouds[0].data,cloud.data);
  p["dropout"] = 1.; plugin->initialize(init(),p); lp::SensorOutput none; plugin->step(c,none);
  EXPECT_TRUE(std::isnan(value(none.clouds[0].data,0)));
}
TEST(SensorPlugins, ImuMountBiasRangeCovarianceAndReset) {
  lp::Loader loader; auto plugin = loader.sensor(classes[1]); auto p = config(classes[1]);
  p["accel_bias"] = {1.,2.,3.}; p["gyro_bias"] = {.1,.2,.3}; p["accel_range_mps2"] = 5.;
  plugin->initialize(init(),p); Scene scene; auto c = context(scene);
  c.body.acceleration = {4,0,0}; c.body.omega = {0,0,1}; c.body_sensor = pose({0,0,0},{0,0,pi});
  lp::SensorOutput a; plugin->step(c,a); ASSERT_EQ(a.imus.size(),1u);
  EXPECT_NEAR(a.imus[0].acceleration.x(),-3,1e-12); EXPECT_NEAR(a.imus[0].acceleration.y(),2,1e-12);
  EXPECT_EQ(a.imus[0].acceleration.z(),5); EXPECT_NEAR(a.imus[0].angular_velocity.z(),1.3,1e-12);
  EXPECT_EQ(a.imus[0].orientation_covariance[0],-1);
  p["accel_sigma"] = {.1,.2,.3}; plugin->initialize(init(),p);
  lp::SensorOutput b,d; plugin->step(c,b); plugin->reset(1); plugin->step(c,d);
  EXPECT_EQ(b.imus[0].acceleration,d.imus[0].acceleration);
  EXPECT_NEAR(b.imus[0].acceleration_covariance[8],.09,1e-14);
  p["gyro_range_radps"] = -1; EXPECT_THROW(plugin->initialize(init(),p),std::runtime_error);
}
TEST(SensorPlugins, CameraKindsBiasClippingNanEndianAndIndependentNoise) {
  lp::Loader loader; Scene scene; auto c = context(scene);
  for (size_t n = 2; n < classes.size(); ++n) for (bool big : {false,true}) {
    auto plugin = loader.sensor(classes[n]); auto p = config(classes[n]); p["width"] = p["height"] = 16;
    const auto kind = n == 2 ? "rgb" : n == 3 ? "depth" : "rgbd";
    if (n != 3) p["color_bias_dn"] = {200.,-200.,2.};
    if (n != 2) p["depth_bias_m"] = .5;
    plugin->initialize(init(),p); lp::SensorOutput out; plugin->step(c,out);
    EXPECT_EQ(out.cameras,std::vector<std::string>{"sensor"});
    auto f = image(kind,1,big);
    if (n != 2) { put(f.depth,4,30,big); put(f.depth,8,std::numeric_limits<float>::quiet_NaN(),big); }
    plugin->processImage(f);
    if (n != 3) { EXPECT_EQ(f.color[0],255); EXPECT_EQ(f.color[1],0); EXPECT_EQ(f.color[2],102); }
    if (n != 2) { EXPECT_EQ(value(f.depth,0,big),2.5); EXPECT_TRUE(std::isnan(value(f.depth,4,big))); EXPECT_TRUE(std::isnan(value(f.depth,8,big))); }
    if (n != 3) { p["color_bias_dn"] = {0.,0.,0.}; p["color_sigma_dn"] = {5.,5.,5.}; }
    if (n != 2) { p["depth_bias_m"] = 0.; p["depth_sigma_m"] = .05; }
    plugin->initialize(init(),p); auto a = image(kind,1,big), b = a, d = image(kind,2,big);
    plugin->processImage(a); plugin->reset(1); plugin->processImage(b); plugin->processImage(d);
    EXPECT_EQ(a.color,b.color); EXPECT_EQ(a.depth,b.depth);
    if (n != 3) EXPECT_NE(a.color,d.color);
    if (n != 2) EXPECT_NE(a.depth,d.depth);
    auto bad = image(kind,1,big); bad.width = 17; EXPECT_THROW(plugin->processImage(bad),std::runtime_error);
  }
}
TEST(SensorPlugins, ZeroErrorsPreserveImuAndAllCameraMeasurements) {
  lp::Loader loader; Scene scene; auto c = context(scene);
  auto imu = loader.sensor(classes[1]); imu->initialize(init(), config(classes[1]));
  lp::SensorOutput output; imu->step(c, output); ASSERT_EQ(output.imus.size(), 1u);
  EXPECT_EQ(output.imus[0].acceleration, Vec3(0, 0, 9.81));
  EXPECT_EQ(output.imus[0].angular_velocity, Vec3::Zero());
  for (size_t n = 2; n < classes.size(); ++n) {
    auto camera = loader.sensor(classes[n]); auto p = config(classes[n]); p["width"] = p["height"] = 16;
    camera->initialize(init(), p);
    auto actual = image(n == 2 ? "rgb" : n == 3 ? "depth" : "rgbd"), expected = actual;
    camera->processImage(actual);
    EXPECT_EQ(actual.color, expected.color);
    EXPECT_EQ(actual.depth, expected.depth);
  }
}

TEST(SensorPlugins, SensorNoiseChangesMeasurementsButNotWorldOrOdometry) {
  auto cfg = base();
  cfg.plugins = {declaration(0, "lidar", {{"columns", 4}, {"horizontal_start_deg", 0}, {"horizontal_end_deg", .001}}),
                 declaration(1, "imu")};
  auto noisy_cfg = cfg;
  noisy_cfg.plugins[0].parameters["range_sigma_m"] = .1;
  noisy_cfg.plugins[1].parameters["accel_sigma"] = {.1, .1, .1};
  noisy_cfg.plugins[1].parameters["gyro_sigma"] = {.01, .01, .01};
  Scene scene; scene.sensor = {{"wall", "", Shape::box({.2, 20, 20}), pose({3, 0, 0})}}; scene.rebuild();
  Simulator ideal(cfg, model(), scene), noisy(noisy_cfg, model(), scene);
  ASSERT_TRUE(ideal.command({.2, .1})); ASSERT_TRUE(noisy.command({.2, .1}));
  const auto a = advance(ideal, 100), b = advance(noisy, 100);
  ASSERT_EQ(a.size(), b.size()); size_t changed_clouds = 0, changed_imus = 0;
  for (size_t i = 0; i < a.size(); ++i) {
    EXPECT_EQ(a[i].snapshot.world_base.matrix(), b[i].snapshot.world_base.matrix());
    EXPECT_EQ(a[i].snapshot.odom_base.matrix(), b[i].snapshot.odom_base.matrix());
    ASSERT_EQ(a[i].clouds.size(), b[i].clouds.size());
    ASSERT_EQ(a[i].imus.size(), b[i].imus.size());
    for (size_t j = 0; j < a[i].clouds.size(); ++j) changed_clouds += a[i].clouds[j].data != b[i].clouds[j].data;
    for (size_t j = 0; j < a[i].imus.size(); ++j) changed_imus += a[i].imus[j].acceleration != b[i].imus[j].acceleration;
  }
  EXPECT_GT(changed_clouds, 0u); EXPECT_GT(changed_imus, 0u);
}
TEST(SensorPlugins, MultiInstanceHostSchedulingTruthMountAndLegacyAdapter) {
  auto cfg = base(); cfg.plugins = {
    declaration(0,"front",{{"columns",4},{"rate_hz",10},{"horizontal_start_deg",0},{"horizontal_end_deg",.00001}}),
    declaration(0,"rear",{{"columns",4},{"rate_hz",7},{"topic","rear/points"}}),
    declaration(1,"imu_a",{{"rate_hz",20}}), declaration(1,"imu_b",{{"rate_hz",13},{"topic","rear/imu"}}),
    declaration(2,"rgb",{{"rate_hz",5},{"width",16},{"height",16}}),
    declaration(3,"depth",{{"rate_hz",3},{"width",16},{"height",16},{"topic","camera/depth"}}),
    declaration(4,"rgbd",{{"rate_hz",2},{"width",16},{"height",16},{"topic","camera/rgbd"}}),
    declaration(0,"off",{{"enabled",false},{"topic","off/points"}})};
  Scene scene; scene.sensor = scene.visual = scene.collision = {{"wall","",Shape::box({.2,20,20}),pose({3,0,0})}}; scene.rebuild();
  Simulator sim(cfg,model(),scene); auto events = advance(sim,200);
  std::map<std::string,size_t> counts;
  for (const auto &event : events) {
    for (const auto &cloud : event.clouds) {
      ++counts[cloud.topic]; if (cloud.topic == "points_raw") EXPECT_NEAR(value(cloud.data,0),2.6,1e-6);
    }
    for (const auto &imu : event.imus) ++counts[imu.topic];
    for (const auto camera : event.cameras) ++counts[sim.config().cameras.at(camera).name];
  }
  EXPECT_EQ(counts["points_raw"],10u); EXPECT_EQ(counts["rear/points"],7u);
  EXPECT_EQ(counts["imu/data_raw"],20u); EXPECT_EQ(counts["rear/imu"],13u);
  EXPECT_EQ(counts["rgb"],5u); EXPECT_EQ(counts["depth"],3u); EXPECT_EQ(counts["rgbd"],2u); EXPECT_EQ(counts["off/points"],0u);
  auto frame = image("rgbd"); EXPECT_NO_THROW(sim.processCamera(2,frame));
  // Resolving a config a second time (new run/reset) must not duplicate sensors.
  Simulator reset(sim.config(),model(),scene); EXPECT_EQ(reset.config().plugins.size(),cfg.plugins.size());
  auto legacy = base(); legacy.lidars = sim.config().lidars; legacy.imus = sim.config().imus; legacy.cameras = sim.config().cameras;
  Simulator adapted(legacy,model(),scene); const auto replay = advance(adapted,200);
  ASSERT_EQ(events.size(),replay.size());
  for (size_t i = 0; i < events.size(); ++i) {
    EXPECT_EQ(events[i].snapshot.stamp,replay[i].snapshot.stamp);
    ASSERT_EQ(events[i].clouds.size(),replay[i].clouds.size());
    for (size_t j = 0; j < events[i].clouds.size(); ++j) EXPECT_EQ(events[i].clouds[j].data,replay[i].clouds[j].data);
  }
}
TEST(SensorPlugins, ConflictsBudgetsUnknownMountAndDuplicateTopicsFail) {
  auto cfg = base(); cfg.plugins = {declaration(1,"first")}; ImuConfig legacy; cfg.imus = {legacy};
  EXPECT_THROW(Simulator(cfg,model(),{}),std::runtime_error);
  cfg.imus.clear(); cfg.plugins.push_back(declaration(1,"second")); EXPECT_THROW(Simulator(cfg,model(),{}),std::runtime_error);
  cfg.plugins = {declaration(1,"first",{{"link","missing"}})}; EXPECT_THROW(Simulator(cfg,model(),{}),std::runtime_error);
  cfg.plugins = {declaration(0,"huge",{{"columns",65536},{"vertical_angles_deg",std::vector<double>(65,0)}})};
  EXPECT_THROW(Simulator(cfg,model(),{}),std::runtime_error);
  cfg.plugins = {declaration(1,"same"),declaration(2,"same")}; EXPECT_THROW(Simulator(cfg,model(),{}),std::runtime_error);
}


TEST(SensorPlugins, ConfigurableImuGravityIsUsedAndReported) {
  lp::Loader loader; auto plugin=loader.sensor(classes[1]); auto p=config(classes[1]);
  p["gravity_mps2"]=9.7; plugin->initialize(init(),p);
  Scene scene; auto context = ::context(scene); lp::SensorOutput output;
  plugin->step(context,output); ASSERT_EQ(output.imus.size(),1u);
  EXPECT_DOUBLE_EQ(output.imus[0].acceleration.z(),9.7);
  EXPECT_DOUBLE_EQ(plugin->description().settings.at("gravity_mps2").get<double>(),9.7);
}
