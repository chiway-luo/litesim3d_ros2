#include <litesim3d/sim.hpp>
#include <gtest/gtest.h>
#include <functional>
#include <limits>

using namespace litesim3d;

namespace {
Model hostModel() {
  Model model;
  model.root = "base_link";
  model.links.insert(model.root);
  model.collision = {{"body", model.root, Shape::box({.2, .2, .2}), pose({0, 0, .25})}};
  model.visual = model.sensor = model.collision;
  model.report = {{"undriven_joints", Json::array()}, {"resources", Json::object()}};
  for (const std::string &name : {"drive", "passive", "steer", "mimic", "steer_mimic", "fixed"}) {
    Joint joint;
    joint.name = name;
    joint.parent = model.root;
    joint.child = name + "_link";
    joint.type = name == "fixed" ? "fixed" : name.find("steer") == 0 ? "revolute" : "continuous";
    joint.axis = joint.type == "revolute" ? Vec3::UnitZ() : Vec3::UnitY();
    joint.velocity = 10;
    if (joint.type == "revolute") { joint.lower = -.7; joint.upper = .7; }
    if (name == "mimic") joint.mimic = "drive";
    if (name == "steer_mimic") joint.mimic = "steer";
    model.links.insert(joint.child);
    model.joints[name] = joint;
    model.order.push_back(name);
  }
  return model;
}

Config motionConfig(const std::string &mode, bool joint_control = true) {
  Config cfg;
  cfg.step_ns = 100000000;
  cfg.drive.model = "plugin";
  cfg.drive.support_links.clear();
  cfg.control_mode = joint_control ? "ros2_control" : "twist";
  cfg.plugins = {
      {"chassis", "drive", "litesim3d_plugin_tests/ContractFaultChassis", {{"case", mode}}, {}},
      {"odometry", "encoders", "litesim3d_plugin_tests/ContractFaultOdometry", Json::object(), {}}};
  return cfg;
}

Config sensorConfig(const std::string &mode, const std::string &kind = "imu") {
  Config cfg;
  cfg.step_ns = 100000000;
  cfg.drive.model = "multicopter";
  cfg.drive.support_links.clear();
  cfg.plugins = {{"sensor", "sensor", "litesim3d_plugin_tests/ContractFaultSensor",
                  {{"case", mode}, {"kind", kind}}, {}}};
  return cfg;
}

void rejects(const std::function<void()> &operation, const std::string &message) {
  try {
    operation();
    FAIL() << "Expected host rejection containing: " << message;
  } catch (const std::runtime_error &error) {
    EXPECT_NE(std::string(error.what()).find(message), std::string::npos) << error.what();
  }
}
} // namespace

TEST(HostPluginContracts, InvalidDriveMetadataCannotEnterTheControlLoop) {
  const std::vector<std::pair<std::string, std::string>> cases = {
      {"driven_empty", "no drive joints"},
      {"driven_missing", "invalid joint-driven wheel"},
      {"driven_type", "invalid joint-driven wheel"},
      {"driven_mimic", "invalid joint-driven wheel"},
      {"passive_missing", "invalid passive wheel"},
      {"passive_type", "invalid passive wheel"},
      {"passive_mimic", "invalid passive wheel"},
      {"passive_overlap", "invalid passive wheel"},
      {"limits_missing", "invalid joint position limits"},
      {"limits_type", "invalid joint position limits"},
      {"limits_mimic", "invalid joint position limits"},
      {"limits_nan_lower", "invalid joint position limits"},
      {"limits_nan_upper", "invalid joint position limits"},
      {"limits_initial_low", "initial position outside chassis joint limits"},
      {"limits_initial_high", "initial position outside chassis joint limits"},
  };
  for (const auto &entry : cases) {
    const auto &mode = entry.first;
    const auto &message = entry.second;
    SCOPED_TRACE(mode);
    rejects([&] { Simulator sim(motionConfig(mode), hostModel(), {}); }, message);
  }
  for (double velocity : {0., std::numeric_limits<double>::quiet_NaN()}) {
    auto model = hostModel();
    model.joints.at("drive").velocity = velocity;
    rejects([&] { Simulator sim(motionConfig("ok"), model, {}); }, "finite positive URDF velocity");
  }
  auto cfg = motionConfig("ok");
  cfg.plugins.front().name.clear();
  rejects([&] { Simulator sim(cfg, hostModel(), {}); }, "duplicate/empty plugin name");
  cfg = motionConfig("ok");
  cfg.plugins.back().name = cfg.plugins.front().name;
  rejects([&] { Simulator sim(cfg, hostModel(), {}); }, "duplicate/empty plugin name");
}

TEST(HostPluginContracts, InvalidPredictionsAndPassiveFeedbackMakeFaultTerminal) {
  for (const std::string &mode : {
      "wheel_missing", "wheel_type", "wheel_mimic", "wheel_delta_nan", "wheel_velocity_missing",
      "wheel_velocity_nan", "steering_type", "steering_mimic", "steering_nan", "steering_low", "steering_high",
      "passive_rate_name", "passive_rate_nan"}) {
    SCOPED_TRACE(mode);
    const bool joint_control = mode.find("passive_rate") == 0;
    Simulator sim(motionConfig(mode, joint_control), hostModel(), {});
    sim.pause(false);
    if (joint_control) ASSERT_TRUE(sim.jointCommand({}));
    else ASSERT_TRUE(sim.command({.1, 0}));
    EXPECT_THROW(sim.tick(), std::runtime_error);
    EXPECT_EQ(sim.state(), State::Fault);
    const auto stopped = sim.snapshot().stamp;
    EXPECT_TRUE(sim.tick().empty());
    EXPECT_EQ(sim.snapshot().stamp, stopped);
  }
}

TEST(HostPluginContracts, SensorDescriptorMustMatchBindingsRateAndResolvedSettings) {
  for (const std::string &mode : {"descriptor_name", "rate_nan", "rate_zero", "rate_high",
                                  "settings_link", "settings_topic", "settings_rate", "settings_enabled"}) {
    SCOPED_TRACE(mode);
    rejects([&] { Simulator sim(sensorConfig(mode), hostModel(), {}); }, "sensor plugin 'sensor'");
  }
  Simulator disabled(sensorConfig("disabled"), hostModel(), {});
  EXPECT_TRUE(disabled.config().imus.empty());
  disabled.pause(false);
  for (const auto &event : disabled.tick()) EXPECT_TRUE(event.imus.empty());
  for (bool camera : {false, true}) {
    auto cfg = sensorConfig("ok");
    if (camera) cfg.cameras.emplace_back();
    else {
      LidarConfig lidar;
      lidar.vertical_angles = {0};
      cfg.lidars = {lidar};
    }
    rejects([&] { Simulator sim(cfg, hostModel(), {}); }, "conflict with legacy YAML");
  }
}

TEST(HostPluginContracts, MalformedSensorOutputsNeverContinueAfterFault) {
  for (const std::string &mode : {"imu_empty", "imu_cloud", "imu_camera", "imu_stamp", "imu_frame", "imu_topic",
                                  "camera_cloud", "camera_imu", "camera_name"}) {
    SCOPED_TRACE(mode);
    const std::string kind = mode.find("camera") == 0 ? "rgb" : "imu";
    Simulator sim(sensorConfig(mode, kind), hostModel(), {});
    sim.pause(false);
    EXPECT_THROW(sim.tick(), std::runtime_error);
    EXPECT_EQ(sim.state(), State::Fault);
    EXPECT_TRUE(sim.tick().empty());
  }
}
