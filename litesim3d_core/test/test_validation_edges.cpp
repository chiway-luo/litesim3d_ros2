#include "litesim3d/rate.hpp"
#include "litesim3d/sim.hpp"
#include <gtest/gtest.h>
#include <array>
#include <chrono>
#include <fstream>
#include <functional>
#include <limits>

using namespace litesim3d;

namespace {
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInfinity = std::numeric_limits<double>::infinity();

void rejects(const std::function<void()> &operation, const std::string &diagnostic) {
  try {
    operation();
    FAIL() << "Expected rejection containing: " << diagnostic;
  } catch (const std::runtime_error &error) {
    EXPECT_NE(std::string(error.what()).find(diagnostic), std::string::npos) << error.what();
  }
}

template <class T>
void rejectsEdits(const T &valid,
                 const std::vector<std::pair<std::string, std::function<void(T &)>>> &edits,
                 const std::string &diagnostic) {
  ASSERT_NO_THROW(valid.validate());
  for (const auto &[name, edit] : edits) {
    SCOPED_TRACE(name);
    T value = valid;
    edit(value);
    rejects([&] { value.validate(); }, diagnostic);
  }
}

LidarConfig lidar() {
  LidarConfig value;
  value.columns = 4;
  value.vertical_angles = {0};
  return value;
}

Model mimicModel() {
  Model model;
  model.root = "root";
  model.links = {"root", "driver_link", "follower_link", "constant_link", "fixed_link"};
  Joint driver;
  driver.name = "driver";
  driver.parent = model.root;
  driver.child = "driver_link";
  driver.type = "prismatic";
  driver.axis = Vec3::UnitX();
  driver.lower = -2;
  driver.upper = 2;
  driver.velocity = 2;
  Joint follower = driver;
  follower.name = "follower";
  follower.child = "follower_link";
  follower.type = "revolute";
  follower.mimic = driver.name;
  follower.multiplier = -2;
  follower.offset = .5;
  follower.lower = -1;
  follower.upper = 1;
  follower.velocity = .4;
  Joint constant = follower;
  constant.name = "constant";
  constant.child = "constant_link";
  constant.mimic = follower.name;
  constant.multiplier = 0;
  constant.offset = .25;
  Joint fixed = driver;
  fixed.name = "fixed";
  fixed.child = "fixed_link";
  fixed.type = "fixed";
  for (const auto &joint : {driver, follower, constant, fixed}) {
    model.joints[joint.name] = joint;
    model.order.push_back(joint.name);
  }
  return model;
}

struct TemporaryText {
  std::filesystem::path path;
  TemporaryText(const std::string &text, const std::string &extension) {
    static size_t sequence = 0;
    path = std::filesystem::temp_directory_path() /
        ("litesim-unit-edges-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) +
         "-" + std::to_string(sequence++) + extension);
    std::ofstream(path) << text;
  }
  ~TemporaryText() { std::filesystem::remove(path); }
};

Config loadYaml(const std::string &extra = "", const std::string &robot_extra = "",
                const std::string &entities = "[]") {
  const auto urdf = (std::filesystem::path(SIM_RESOURCES) / "urdf/r01.urdf").string();
  TemporaryText file("schema_version: 1\nunits: m_rad_s\nrobot: {urdf: '" + urdf +
      "'" + robot_extra + "}\nentities: " + entities + "\n" + extra, ".yaml");
  Scene scene;
  return Config::load(file.path, scene);
}

Model loadXml(const std::string &xml, const ModelOptions &options = {}) {
  TemporaryText file(xml, ".urdf");
  return Model::load(file.path, options);
}
} // namespace

TEST(ValidationEdges, ConfigScalarBoundsAndFiniteInputs) {
  const Config valid;
  const std::vector<std::pair<std::string, std::function<void(Config &)>>> edits = {
      {"unknown control mode", [](auto &c) { c.control_mode = "position"; }},
      {"zero step", [](auto &c) { c.step_ns = 0; }},
      {"step over maximum", [](auto &c) { c.step_ns = 100000001; }},
      {"non-finite odom rate", [](auto &c) { c.odom_rate = kNaN; }},
      {"zero odom rate", [](auto &c) { c.odom_rate = 0; }},
      {"odom rate over maximum", [](auto &c) { c.odom_rate = 1001; }},
      {"non-finite timeout", [](auto &c) { c.timeout = kInfinity; }},
      {"zero timeout", [](auto &c) { c.timeout = 0; }},
      {"non-finite real-time factor", [](auto &c) { c.real_time_factor = kNaN; }},
      {"negative real-time factor", [](auto &c) { c.real_time_factor = -1; }},
      {"real-time factor over maximum", [](auto &c) { c.real_time_factor = 101; }},
      {"non-finite initial pose", [](auto &c) { c.initial_pose.translation().x() = kNaN; }},
      {"non-unit determinant", [](auto &c) { c.initial_pose.linear()(0, 0) = 2; }},
      {"pitched M1 pose", [](auto &c) { c.initial_pose = pose(Vec3::Zero(), {0, .1, 0}); }},
      {"rolled M1 pose", [](auto &c) { c.initial_pose = pose(Vec3::Zero(), {.1, 0, 0}); }},
      {"upside-down M1 pose", [](auto &c) { c.initial_pose = pose(Vec3::Zero(), {pi, 0, 0}); }},
      {"empty robot id", [](auto &c) { c.robot_id.clear(); }},
      {"empty base link", [](auto &c) { c.base_link.clear(); }},
      {"zero workers", [](auto &c) { c.threads = 0; }},
      {"too many workers", [](auto &c) { c.threads = 5; }},
      {"history below minimum", [](auto &c) { c.history_limit = 3; }},
      {"history above maximum", [](auto &c) { c.history_limit = 65537; }},
      {"zero journal", [](auto &c) { c.journal_limit = 0; }},
      {"journal above maximum", [](auto &c) { c.journal_limit = 65537; }},
      {"too many IMUs", [](auto &c) { c.imus.resize(17); }},
      {"too many lidars", [](auto &c) { c.lidars.resize(17); }},
      {"too many cameras", [](auto &c) { c.cameras.resize(9); }},
  };
  ASSERT_NO_THROW(valid.validate());
  for (const auto &[name, edit] : edits) {
    SCOPED_TRACE(name);
    Config value = valid;
    edit(value);
    EXPECT_THROW(value.validate(), std::runtime_error);
  }
  Config boundary;
  boundary.step_ns = 100000000;
  boundary.odom_rate = 1000;
  boundary.real_time_factor = 0;
  boundary.threads = 4;
  boundary.history_limit = boundary.journal_limit = 65536;
  EXPECT_NO_THROW(boundary.validate());
  boundary.real_time_factor = 100;
  EXPECT_NO_THROW(boundary.validate());
}

TEST(ValidationEdges, CovarianceRejectsNonFiniteAsymmetricAndNegativeMatrices) {
  for (bool pose_covariance : {false, true}) {
    for (int invalid = 0; invalid < 3; ++invalid) {
      SCOPED_TRACE(pose_covariance);
      SCOPED_TRACE(invalid);
      Config c;
      auto &covariance = pose_covariance ? c.pose_covariance : c.twist_covariance;
      if (invalid == 0) covariance[0] = kNaN;
      if (invalid == 1) covariance[1] = .1;
      if (invalid == 2) covariance[0] = -.1;
      rejects([&] { c.validate(); }, "covariance must be");
    }
  }
  Config c;
  for (size_t axis = 0; axis < 6; ++axis) {
    c.pose_covariance[axis * 7] = double(axis);
    c.twist_covariance[axis * 7] = .25;
  }
  EXPECT_NO_THROW(c.validate());
}

TEST(ValidationEdges, SensorNameTopicAndAllocationBudgetsIncludeEveryOutput) {
  Config c;
  auto first = lidar();
  auto second = first;
  second.name = "second";
  c.lidars = {first, second};
  rejects([&] { c.validate(); }, "duplicate lidar name/topic");
  second.name = first.name;
  second.topic = "second";
  c.lidars = {first, second};
  rejects([&] { c.validate(); }, "duplicate lidar name/topic");
  first.name = "boundary_topic";
  first.topic = "points/range_boundary";
  second.name = "source";
  second.topic = "points";
  second.publish_range_boundary = true;
  c.lidars = {first, second};
  rejects([&] { c.validate(); }, "duplicate range boundary topic");
  first = lidar();
  first.columns = 65536;
  first.vertical_angles.resize(64, 0);
  second = first;
  second.name = second.topic = "second";
  c.lidars = {first, second};
  rejects([&] { c.validate(); }, "aggregate sensor point budget");
  c.lidars = {lidar()};
  ImuConfig imu;
  imu.name = c.lidars[0].name;
  c.imus = {imu};
  rejects([&] { c.validate(); }, "duplicate sensor name/topic");
  c.imus[0].name = "imu";
  c.imus[0].topic = c.lidars[0].topic;
  rejects([&] { c.validate(); }, "duplicate sensor name/topic");
  c.imus.clear();
  CameraConfig camera;
  camera.name = c.lidars[0].name;
  c.cameras = {camera};
  rejects([&] { c.validate(); }, "duplicate sensor name");
  c.cameras[0].name = "camera";
  c.cameras[0].type = "rgbd";
  for (const auto &channel : {"color", "depth"}) for (const auto &suffix : {"image_raw", "camera_info"}) {
    c.lidars[0].topic = camera.topic + "/" + channel + "/" + suffix;
    rejects([&] { c.validate(); }, "duplicate camera output topic");
  }
  c.lidars.clear();
  c.cameras[0].width = 4096;
  c.cameras[0].height = 2025;
  camera = c.cameras[0];
  camera.name = "second_camera";
  camera.topic = "second_camera";
  c.cameras.push_back(camera);
  EXPECT_NO_THROW(c.validate());
  camera.name = camera.topic = "third_camera";
  camera.width = camera.height = 16;
  c.cameras.push_back(camera);
  rejects([&] { c.validate(); }, "aggregate camera pixel allocation");
  c.cameras.back().enabled = false;
  EXPECT_NO_THROW(c.validate());
}

TEST(ValidationEdges, RollingHistoryIncludesNonIntegralTimeoutScheduling) {
  Config c;
  c.step_ns = 100000000;
  c.history_limit = 4;
  c.lidars = {lidar()};
  c.lidars[0].scan_mode = "rolling";
  rejects([&] { c.validate(); }, "trajectory history budget");
  c.history_limit = 11;
  EXPECT_EQ(c.timeoutPhaseNs(), 0u);
  EXPECT_NO_THROW(c.validate());
  c.timeout = .55;
  EXPECT_EQ(c.timeoutPhaseNs(), 50000000u);
  rejects([&] { c.validate(); }, "trajectory history budget");
  c.history_limit = 12;
  EXPECT_NO_THROW(c.validate());
}

TEST(ValidationEdges, DriveDimensionsLimitsAndJointNames) {
  const std::array<double Drive::*, 7> positive = {
      &Drive::wheel_radius, &Drive::wheel_separation, &Drive::max_v, &Drive::max_w,
      &Drive::acceleration, &Drive::deceleration, &Drive::angular_acceleration};
  for (auto field : positive) for (double bad : {0., -1., kNaN, kInfinity}) {
    Drive drive;
    drive.*field = bad;
    rejects([&] { drive.validate(); }, "positive finite");
  }
  rejectsEdits<Drive>({}, {
      {"unknown model", [](auto &d) { d.model = "skid"; }},
      {"non-finite wheelbase", [](auto &d) { d.wheelbase = kNaN; }},
      {"zero wheelbase", [](auto &d) { d.wheelbase = 0; }},
      {"non-finite steering", [](auto &d) { d.max_steering = kInfinity; }},
      {"zero steering", [](auto &d) { d.max_steering = 0; }},
      {"steering at singularity", [](auto &d) { d.max_steering = pi / 2; }},
      {"empty left joint", [](auto &d) { d.left_joint.clear(); }},
      {"empty right joint", [](auto &d) { d.right_joint.clear(); }},
      {"shared wheel joint", [](auto &d) { d.right_joint = d.left_joint; }},
  }, "");
  for (const auto &model : {"differential_drive", "ackermann", "multicopter", "plugin"}) {
    Drive drive;
    drive.model = model;
    EXPECT_NO_THROW(drive.validate());
  }
}

TEST(ValidationEdges, TerrainLimitsAndOdometryErrorBounds) {
  rejectsEdits<TerrainConfig>({}, {
      {"non-finite slope", [](auto &c) { c.max_slope = kNaN; }},
      {"zero slope", [](auto &c) { c.max_slope = 0; }},
      {"vertical slope", [](auto &c) { c.max_slope = pi / 2; }},
      {"non-finite clearance", [](auto &c) { c.clearance = kInfinity; }},
      {"negative clearance", [](auto &c) { c.clearance = -.1; }},
      {"non-finite continuity", [](auto &c) { c.continuity = kNaN; }},
      {"zero continuity", [](auto &c) { c.continuity = 0; }},
      {"continuity over maximum", [](auto &c) { c.continuity = .10001; }},
  }, "terrain: invalid limits");
  const std::array<double OdomError::*, 6> fields = {
      &OdomError::linear_scale, &OdomError::angular_scale, &OdomError::linear_bias,
      &OdomError::angular_bias, &OdomError::linear_sigma, &OdomError::angular_sigma};
  for (auto field : fields) {
    OdomError error;
    error.*field = kNaN;
    rejects([&] { error.validate(); }, "non-finite error model");
  }
  rejectsEdits<OdomError>({}, {
      {"zero linear scale", [](auto &e) { e.linear_scale = 0; }},
      {"zero angular scale", [](auto &e) { e.angular_scale = 0; }},
      {"negative linear sigma", [](auto &e) { e.linear_sigma = -.1; }},
      {"negative angular sigma", [](auto &e) { e.angular_sigma = -.1; }},
  }, "invalid scale/sigma");
}

TEST(ValidationEdges, ImuIdentityNoiseAndMeasurementRanges) {
  rejectsEdits<ImuConfig>({}, {
      {"empty name", [](auto &c) { c.name.clear(); }},
      {"empty link", [](auto &c) { c.link.clear(); }},
      {"empty topic", [](auto &c) { c.topic.clear(); }},
      {"non-finite rate", [](auto &c) { c.rate = kNaN; }},
      {"zero rate", [](auto &c) { c.rate = 0; }},
      {"rate over maximum", [](auto &c) { c.rate = 2001; }},
      {"non-finite gravity", [](auto &c) { c.gravity = kInfinity; }},
      {"zero gravity", [](auto &c) { c.gravity = 0; }},
      {"non-finite acceleration bias", [](auto &c) { c.accel_bias.x() = kNaN; }},
      {"non-finite gyro bias", [](auto &c) { c.gyro_bias.y() = kInfinity; }},
      {"non-finite acceleration sigma", [](auto &c) { c.accel_sigma.z() = kNaN; }},
      {"non-finite gyro sigma", [](auto &c) { c.gyro_sigma.x() = kInfinity; }},
      {"negative acceleration sigma", [](auto &c) { c.accel_sigma.y() = -.1; }},
      {"negative gyro sigma", [](auto &c) { c.gyro_sigma.z() = -.1; }},
      {"non-finite acceleration range", [](auto &c) { c.accel_range = kNaN; }},
      {"negative acceleration range", [](auto &c) { c.accel_range = -1; }},
      {"non-finite gyro range", [](auto &c) { c.gyro_range = kInfinity; }},
      {"negative gyro range", [](auto &c) { c.gyro_range = -1; }},
  }, "imu:");
  ImuConfig c;
  Kinematics body;
  Pose mount = Pose::Identity();
  for (int component = 0; component < 5; ++component) {
    Kinematics invalid = body;
    Pose invalid_mount = mount;
    if (component == 0) invalid.world_body.translation().x() = kNaN;
    if (component == 1) invalid.acceleration.x() = kInfinity;
    if (component == 2) invalid.omega.y() = kNaN;
    if (component == 3) invalid.alpha.z() = kInfinity;
    if (component == 4) invalid_mount.translation().x() = kNaN;
    rejects([&] { sampleImu(c, invalid, invalid_mount, 1); }, "non-finite kinematics");
  }
  c.accel_range = 2;
  c.gyro_range = 3;
  body.acceleration = {-5, 5, -20};
  body.omega = {-5, 5, 0};
  const auto clipped = sampleImu(c, body, mount, 7);
  EXPECT_TRUE(clipped.acceleration.isApprox(Vec3(-2, 2, -2)));
  EXPECT_TRUE(clipped.angular_velocity.isApprox(Vec3(-3, 3, 0)));
  EXPECT_EQ(clipped.stamp, 7u);
}

TEST(ValidationEdges, CameraAllFiniteDimensionsOpticsAndNoiseBranches) {
  rejectsEdits<CameraConfig>({}, {
      {"invalid name", [](auto &c) { c.name = "1camera"; }},
      {"empty link", [](auto &c) { c.link.clear(); }},
      {"invalid topic", [](auto &c) { c.topic = "camera//front"; }},
      {"height below minimum", [](auto &c) { c.height = 15; }},
      {"width above maximum", [](auto &c) { c.width = 4097; }},
      {"height above maximum", [](auto &c) { c.height = 4097; }},
      {"non-finite field of view", [](auto &c) { c.horizontal_fov = kNaN; }},
      {"zero field of view", [](auto &c) { c.horizontal_fov = 0; }},
      {"non-finite near clip", [](auto &c) { c.near_clip = kNaN; }},
      {"non-finite far clip", [](auto &c) { c.far_clip = kInfinity; }},
      {"zero near clip", [](auto &c) { c.near_clip = 0; }},
      {"non-finite color bias", [](auto &c) { c.color_bias.x() = kNaN; }},
      {"non-finite color sigma", [](auto &c) { c.color_sigma.y() = kInfinity; }},
      {"negative color sigma", [](auto &c) { c.color_sigma.z() = -1; }},
      {"non-finite depth bias", [](auto &c) { c.depth_bias = kNaN; }},
      {"non-finite depth sigma", [](auto &c) { c.depth_sigma = kInfinity; }},
      {"negative depth sigma", [](auto &c) { c.depth_sigma = -1; }},
      {"RGB cannot have depth bias", [](auto &c) { c.depth_bias = .1; }},
      {"RGB cannot have depth noise", [](auto &c) { c.depth_sigma = .1; }},
      {"depth cannot have RGB bias", [](auto &c) { c.type = "depth"; c.color_bias.x() = .1; }},
      {"depth cannot have RGB noise", [](auto &c) { c.type = "depth"; c.color_sigma.z() = .1; }},
  }, "camera");
  CameraConfig c;
  c.type = "rgbd";
  c.width = 16;
  c.height = 4096;
  c.rate = 240;
  c.color_sigma = Vec3::Ones();
  c.depth_sigma = .1;
  EXPECT_NO_THROW(c.validate());
  EXPECT_EQ(c.json()["type"], "rgbd");
  EXPECT_EQ(c.json()["height"], 4096);
}

TEST(ValidationEdges, LidarFiniteRangesPatternsAndRingDelays) {
  rejectsEdits<LidarConfig>(lidar(), {
      {"empty name", [](auto &c) { c.name.clear(); }},
      {"empty link", [](auto &c) { c.link.clear(); }},
      {"empty topic", [](auto &c) { c.topic.clear(); }},
      {"too many rings", [](auto &c) { c.vertical_angles.resize(65537, 0); }},
      {"zero columns", [](auto &c) { c.columns = 0; }},
      {"too many columns", [](auto &c) { c.columns = 65537; }},
      {"aggregate point budget", [](auto &c) { c.columns = 65536; c.vertical_angles.resize(65, 0); }},
      {"non-finite vertical angle", [](auto &c) { c.vertical_angles[0] = kNaN; }},
      {"angle below lower pole", [](auto &c) { c.vertical_angles[0] = -pi; }},
      {"non-finite rate", [](auto &c) { c.rate = kNaN; }},
      {"zero rate", [](auto &c) { c.rate = 0; }},
      {"rate over maximum", [](auto &c) { c.rate = 1001; }},
      {"non-finite minimum range", [](auto &c) { c.min_range = kNaN; }},
      {"non-finite maximum range", [](auto &c) { c.max_range = kInfinity; }},
      {"negative minimum range", [](auto &c) { c.min_range = -.1; }},
      {"reversed range", [](auto &c) { c.max_range = c.min_range; }},
      {"non-finite horizontal start", [](auto &c) { c.horizontal_start = kNaN; }},
      {"non-finite horizontal end", [](auto &c) { c.horizontal_end = kInfinity; }},
      {"zero horizontal span", [](auto &c) { c.horizontal_end = c.horizontal_start; }},
      {"horizontal span over full turn", [](auto &c) { c.horizontal_end += .01; }},
      {"unknown scan mode", [](auto &c) { c.scan_mode = "interpolated"; }},
      {"unknown scan pattern", [](auto &c) { c.scan_pattern = "random"; }},
      {"non-finite lower elevation", [](auto &c) { c.vertical_min = kNaN; }},
      {"non-finite upper elevation", [](auto &c) { c.vertical_max = kInfinity; }},
      {"lower elevation outside sphere", [](auto &c) { c.vertical_min = -pi; }},
      {"upper elevation outside sphere", [](auto &c) { c.vertical_max = pi; }},
      {"zero vertical span", [](auto &c) { c.vertical_max = c.vertical_min; }},
      {"non-finite cone", [](auto &c) { c.cone_half_angle = kNaN; }},
      {"zero cone", [](auto &c) { c.cone_half_angle = 0; }},
      {"cone above hemisphere", [](auto &c) { c.cone_half_angle = pi; }},
      {"non-finite pattern frequency", [](auto &c) { c.pattern_hz = kInfinity; }},
      {"zero pattern frequency", [](auto &c) { c.pattern_hz = 0; }},
      {"pattern frequency over maximum", [](auto &c) { c.pattern_hz = 10001; }},
      {"non-finite bias", [](auto &c) { c.range_bias = kNaN; }},
      {"non-finite sigma", [](auto &c) { c.range_sigma = kInfinity; }},
      {"negative sigma", [](auto &c) { c.range_sigma = -1; }},
      {"non-finite dropout", [](auto &c) { c.dropout = kNaN; }},
      {"negative dropout", [](auto &c) { c.dropout = -.1; }},
      {"dropout above one", [](auto &c) { c.dropout = 1.1; }},
      {"wrong delay count", [](auto &c) { c.ring_delays = {0, 0}; }},
      {"non-finite delay", [](auto &c) { c.ring_delays = {kNaN}; }},
      {"negative delay", [](auto &c) { c.ring_delays = {-.01}; }},
      {"delay one whole column", [](auto &c) { c.ring_delays = {1 / (c.rate * c.columns)}; }},
      {"unknown self-filter", [](auto &c) { c.filter = "none"; }},
  }, "lidar:");
  auto c = lidar();
  c.ring_delays = {.001};
  rejects([&] { c.validate(); }, "snapshot cannot have ring delays");
  c.scan_mode = "rolling";
  c.ring_delays = {.001};
  EXPECT_NO_THROW(c.validate());
  EXPECT_EQ(c.offsetNs(0, 0), 1000000u);
  EXPECT_EQ(c.offsetNs(0, 3), 76000000u);
  for (const auto &filter : {"sensor_link", "robot", "links"}) {
    c.filter = filter;
    EXPECT_NO_THROW(c.validate());
  }
}

TEST(ValidationEdges, LoadCheckpointFailureIsAtomicAndBoundaryInclusive) {
  LoadControl control;
  control.byte_budget = 10;
  control.triangle_budget = 2;
  control.checkpoint(10, 2);
  EXPECT_EQ(control.bytes, 10u);
  EXPECT_EQ(control.triangles, 2u);
  EXPECT_EQ(control.checkpoints, 1u);
  rejects([&] { control.checkpoint(1); }, "byte budget");
  rejects([&] { control.checkpoint(0, 1); }, "triangle budget");
  EXPECT_EQ(control.checkpoints, 1u);
  control.bytes = 11;
  rejects([&] { control.checkpoint(); }, "byte budget");
  control.bytes = 10;
  control.triangles = 3;
  rejects([&] { control.checkpoint(); }, "triangle budget");
  control.triangles = 2;
  control.cancelled = [] { return true; };
  rejects([&] { control.checkpoint(); }, "cancelled");
  control.cancelled = [] { return false; };
  control.fail_allocation_after = 1;
  EXPECT_THROW(control.checkpoint(), std::bad_alloc);
  EXPECT_EQ(control.checkpoints, 1u);
  control.fail_allocation_after.reset();
  EXPECT_NO_THROW(control.checkpoint());
  EXPECT_EQ(control.checkpoints, 2u);
}

TEST(ValidationEdges, PoseQuaternionAndSceneRayRejectNonFiniteInputs) {
  rejects([] { pose({kNaN, 0, 0}); }, "non-finite xyz/rpy");
  rejects([] { pose(Vec3::Zero(), {0, kInfinity, 0}); }, "non-finite xyz/rpy");
  rejects([] { quaternionPose({kNaN, 0, 0}, Eigen::Quaterniond::Identity()); }, "invalid quaternion");
  rejects([] { quaternionPose(Vec3::Zero(), Eigen::Quaterniond(kNaN, 0, 0, 0)); }, "invalid quaternion");
  rejects([] { quaternionPose(Vec3::Zero(), Eigen::Quaterniond(0, 0, 0, 0)); }, "invalid quaternion");
  const auto normalized = quaternionPose({1, 2, 3}, Eigen::Quaterniond(2, 0, 0, 0));
  EXPECT_TRUE(normalized.translation().isApprox(Vec3(1, 2, 3)));
  EXPECT_TRUE(normalized.linear().isIdentity());
  Scene scene;
  rejects([&] { scene.ray({kNaN, 0, 0}, Vec3::UnitX(), 1); }, "ray: invalid");
  rejects([&] { scene.ray(Vec3::Zero(), {0, kInfinity, 0}, 1); }, "ray: invalid");
  rejects([&] { scene.ray(Vec3::Zero(), Vec3::Zero(), 1); }, "ray: invalid");
  rejects([&] { scene.ray(Vec3::Zero(), Vec3::UnitX(), kInfinity); }, "ray: invalid");
  rejects([&] { scene.ray(Vec3::Zero(), Vec3::UnitX(), 0); }, "ray: invalid");
  EXPECT_FALSE(scene.ray(Vec3::Zero(), Vec3::UnitX(), 1));
}

TEST(ValidationEdges, RealTimeRateResetsForRepeatedAndBackwardWallClock) {
  RealTimeRate rate;
  EXPECT_FALSE(rate.sample("epoch", 1, 1));
  ASSERT_TRUE(rate.sample("epoch", 100000001, 1.1));
  EXPECT_FALSE(rate.sample("epoch", 200000001, 1.1));
  EXPECT_NEAR(*rate.sample("epoch", 300000001, 1.2), 1, 1e-12);
  EXPECT_FALSE(rate.sample("epoch", 400000001, .5));
  EXPECT_NEAR(*rate.sample("epoch", 600000001, .6), 2, 1e-12);
}

TEST(ModelBoundaries, NegativeAndConstantMimicsConstrainDriverAndVelocity) {
  const auto model = mimicModel();
  const auto high = model.positions({{"driver", 100}, {"follower", 42}, {"fixed", 1}});
  EXPECT_DOUBLE_EQ(high.at("driver"), .75);
  EXPECT_DOUBLE_EQ(high.at("follower"), -1);
  EXPECT_DOUBLE_EQ(high.at("constant"), .25);
  EXPECT_DOUBLE_EQ(high.at("fixed"), 0);
  const auto low = model.positions({{"driver", -100}});
  EXPECT_DOUBLE_EQ(low.at("driver"), -.25);
  EXPECT_DOUBLE_EQ(low.at("follower"), 1);
  const auto advanced = model.advance({}, {{"driver", 1}}, .5);
  // The follower's 0.4 rad/s limit constrains its -2:1 driver to 0.2 m/s.
  EXPECT_NEAR(advanced.at("driver"), .1, 1e-12);
  EXPECT_NEAR(advanced.at("follower"), .3, 1e-12);
  EXPECT_DOUBLE_EQ(advanced.at("constant"), .25);
  const auto frames = model.fk(advanced);
  EXPECT_NEAR(frames.at("driver_link").translation().x(), .1, 1e-12);
  EXPECT_TRUE(frames.at("fixed_link").isApprox(Pose::Identity()));
  const auto before = model.positions({});
  const auto small_target = model.advance(before, {{"driver", .001}}, 1);
  EXPECT_NEAR(small_target.at("driver"), .001, 1e-12);
}

TEST(ModelBoundaries, InvalidTargetsMimicOverflowAndIncompatibleLimitsRejectAtomically) {
  const auto model = mimicModel();
  rejects([&] { model.positions({{"missing", 0}}); }, "unknown joint");
  rejects([&] { model.positions({{"driver", kNaN}}); }, "non-finite position");
  for (double dt : {0., -1., kNaN, kInfinity})
    rejects([&] { model.advance({}, {}, dt); }, "invalid dt");
  rejects([&] { model.advance({}, {{"missing", 0}}, .1); }, "invalid joint target");
  rejects([&] { model.advance({}, {{"driver", kNaN}}, .1); }, "invalid joint target");
  rejects([&] { model.advance({}, {{"follower", 0}}, .1); }, "mimic joint cannot be driven");
  auto invalid = model;
  invalid.joints.at("constant").offset = 2;
  rejects([&] { invalid.positions({}); }, "mimic constant violates limits");
  invalid = model;
  invalid.joints.at("follower").lower = 10;
  invalid.joints.at("follower").upper = 11;
  rejects([&] { invalid.positions({}); }, "incompatible mimic limits");
  invalid = model;
  invalid.joints.at("follower").multiplier = std::numeric_limits<double>::max();
  invalid.joints.at("constant").multiplier = std::numeric_limits<double>::max();
  rejects([&] { invalid.positions({}); }, "mimic chain overflow");
  rejects([&] { model.mechanicalGeometry("missing"); }, "base_link is absent");
  rejects([&] { model.placed({}, Pose::Identity(), "missing", {}); }, "unknown base_link");
  EXPECT_EQ(model.positions({}).at("driver"), 0);
}

TEST(CollisionBudgetBoundaries, DynamicShapeCountMeshSizeAndWorldShapeCount) {
  Config c;
  c.collision_budget_enabled = true;
  Model model;
  Scene scene;
  auto box = Shape::box(Vec3::Ones());
  Geometry moving{"moving", "", box};
  moving.motion = std::make_shared<Motion>();
  scene.collision.assign(64, moving);
  EXPECT_NO_THROW(validateCollisionBudget(c, model, scene));
  scene.collision.push_back(moving);
  rejects([&] { validateCollisionBudget(c, model, scene); }, "dynamic model exceeds 64 shapes");
  scene.collision.assign(1024, Geometry{"fixed", "", box});
  EXPECT_NO_THROW(validateCollisionBudget(c, model, scene));
  scene.collision.push_back(scene.collision.front());
  rejects([&] { validateCollisionBudget(c, model, scene); }, "world exceeds 1024 shapes");
  auto oversized = std::make_shared<Shape>();
  oversized->kind = Shape::Kind::Mesh;
  oversized->triangles.resize(10001);
  scene.collision = {Geometry{"oversized", "", oversized}};
  rejects([&] { validateCollisionBudget(c, model, scene); }, "mesh exceeds 10000 triangles");
}

TEST(ModelBoundaries, ParserReportsMissingNamesTypesAndEveryParentChildComponent) {
  for (const std::string &link : {"<link/>", "<link name=''/>", "<joint name='j'/>"})
    EXPECT_THROW(loadXml("<robot name='r'><link name='base'/>" + link + "</robot>"), std::runtime_error);
  for (const std::string &ends : {
      "<child link='tip'/>", "<parent link='base'/>",
      "<parent/><child link='tip'/>", "<parent link='base'/><child/>",
      "<parent link='missing'/><child link='tip'/>"})
    rejects([&] { loadXml("<robot name='r'><link name='base'/><link name='tip'/>"
        "<joint name='j' type='fixed'>" + ends + "</joint></robot>"); }, "joint j");
  const auto xml = "<robot name='r'><material name='grey'><color rgba='.5 .5 .5 1'/></material>"
      "<link name='base'/><litesim3d version='1'/></robot>";
  EXPECT_TRUE(loadXml(xml).report["ignored_extensions"].empty());
  for (const auto &uri : {"package://missing_slash", "package:///file", "package://pkg//absolute"})
    rejects([&] { resolveResource(uri, std::filesystem::temp_directory_path()); }, "invalid package URI");
}

TEST(ModelBoundaries, VisualOnlyChoiceAndMechanicalWheelGeometryAreExplicit) {
  const std::string visual = "<robot name='r'><link name='base'><visual><geometry>"
      "<box size='1 1 1'/></geometry></visual></link></robot>";
  ModelOptions options;
  options.allow_visual_only = true;
  auto model = loadXml(visual, options);
  EXPECT_TRUE(model.collision.empty());
  EXPECT_EQ(model.visual.size(), 1u);
  options.sensor_geometry = "unknown";
  rejects([&] { loadXml(visual, options); }, "sensor_geometry must be visual or collision");
  auto wheel_model = Model::load(std::filesystem::path(SIM_RESOURCES) / "urdf/r01.urdf");
  const auto wheel_link = wheel_model.joints.at("left_wheel_joint").child;
  for (auto &g : wheel_model.collision) if (g.link == wheel_link)
    g.transform.linear() = Eigen::AngleAxisd(pi / 2, Vec3::UnitY()).toRotationMatrix();
  EXPECT_EQ(wheel_model.mechanicalGeometry("base_link").wheel_radii.count("left_wheel_joint"), 0u);
  wheel_model = Model::load(std::filesystem::path(SIM_RESOURCES) / "urdf/r01.urdf");
  for (const auto &g : wheel_model.collision) if (g.link == wheel_link) {
    auto extra = g;
    extra.shape = Shape::cylinder(g.shape->size.x() * 2, g.shape->size.z());
    wheel_model.collision.push_back(extra);
    break;
  }
  EXPECT_EQ(wheel_model.mechanicalGeometry("base_link").wheel_radii.count("left_wheel_joint"), 0u);
  auto constant = mimicModel();
  constant.joints.at("constant").offset = -2;
  rejects([&] { constant.positions({}); }, "mimic constant violates limits");
  auto overflow = mimicModel();
  overflow.joints.at("follower").offset = std::numeric_limits<double>::max();
  overflow.joints.at("follower").lower = -kInfinity;
  overflow.joints.at("follower").upper = kInfinity;
  overflow.joints.at("constant").multiplier = 2;
  rejects([&] { overflow.positions({}); }, "mimic chain overflow");
}

TEST(TerrainBoundaries, QueryBoundsAndClosestSupportLayerAreExplicit) {
  Scene scene;
  for (double invalid : {-1., kNaN, kInfinity})
    rejects([&] { scene.supportAt(0, 0, 0, invalid); }, "invalid bounds");
  scene.collision = {
      Geometry{"lower", "", Shape::box({4, 4, .2}), pose({0, 0, -.1}), .5, true},
      Geometry{"upper", "", Shape::box({4, 4, .2}), pose({0, 0, -.09}), .5, true}};
  scene.rebuild();
  const auto nearest = scene.supportAt(0, 0, .008, .02);
  ASSERT_TRUE(nearest);
  EXPECT_EQ(nearest->entity, "upper");
  EXPECT_NEAR(nearest->point.z(), .01, 1e-12);
  EXPECT_EQ(scene.supportAt(0, 0, .008, .02, "lower")->entity, "lower");
}

TEST(ConfigYamlBoundaries, TypedSensorsTerrainRobotOptionsAndRingDelayLoad) {
  auto cfg = loadYaml(
      "terrain: {enabled: true, max_slope_deg: 15, clearance_m: 0.01, continuity_m: 0.05}\n"
      "imus: [{name: imu1, link: base_link, topic: imu/raw, rate_hz: 500, gravity_mps2: 9.8, "
      "accel_bias: [1,2,3], gyro_bias: [0.1,0.2,0.3], accel_sigma: [0.01,0.02,0.03], "
      "gyro_sigma: [0.001,0.002,0.003], accel_range_mps2: 20, gyro_range_radps: 5}]\n"
      "cameras: [{name: camera1, link: base_link, topic: camera/front, type: rgbd, enabled: false, "
      "width: 16, height: 32, rate_hz: 60, horizontal_fov_deg: 80, near_clip_m: 0.2, far_clip_m: 8, "
      "color_bias_dn: [1,2,3], color_sigma_dn: [0.1,0.2,0.3], depth_bias_m: 0.01, depth_sigma_m: 0.02}]\n"
      "lidars: [{name: rolling, link: lidar_link, columns: 4, vertical_angles_deg: [0], "
      "scan_mode: rolling, ring_delays_s: [0.001]}]\n",
      ", drive: {}, support_links: [left_wheel, right_wheel], continue_on_collision: true");
  EXPECT_TRUE(cfg.terrain.enabled);
  EXPECT_NEAR(cfg.terrain.clearance, .01, 1e-12);
  ASSERT_EQ(cfg.imus.size(), 1u);
  EXPECT_EQ(cfg.imus[0].rate, 500);
  EXPECT_EQ(cfg.imus[0].gyro_bias.y(), .2);
  ASSERT_EQ(cfg.cameras.size(), 1u);
  EXPECT_EQ(cfg.cameras[0].type, "rgbd");
  EXPECT_FALSE(cfg.cameras[0].enabled);
  EXPECT_EQ(cfg.cameras[0].height, 32u);
  EXPECT_EQ(cfg.lidars[0].offsetNs(0, 0), 1000000u);
  EXPECT_TRUE(cfg.drive.continue_on_collision);
  const auto drone = loadYaml("", ", motion_model: multicopter");
  EXPECT_TRUE(drone.drive.support_links.empty());
}

TEST(ConfigYamlBoundaries, MalformedSensorVectorsSequencesAndCovariancesReject) {
  for (const std::string &extra : {
      "simulation: {step_size: 0}\n", "simulation: {step_size: 0.2}\n",
      "imus: {}\n", "cameras: {}\n", "lidars: {}\n",
      "imus: [{accel_bias: [1,2]}]\n", "imus: [{gyro_bias: 1}]\n",
      "odom: {pose_covariance: 0}\n", "odom: {twist_covariance: [0]}\n",
      "odom: {mode: unsupported}\n", "odom: {mode: noisy, linear_scale_error: .nan}\n",
      "odom: {mode: noisy, angular_scale_error: -1}\n",
      "odom: {mode: noisy, angular_scale_error: .nan}\n",
      "odom: {mode: noisy, angular_scale: 1.1, angular_scale_error: 0.1}\n",
      "odom: {linear_scale: 1.1}\n", "odom: {angular_scale: 1.1}\n",
      "odom: {linear_bias: 0.1}\n", "odom: {angular_bias: 0.1}\n",
      "odom: {linear_sigma: 0.1}\n", "odom: {angular_sigma: 0.1}\n"}) {
    SCOPED_TRACE(extra);
    EXPECT_THROW(loadYaml(extra), std::runtime_error);
  }
  for (const std::string &extra : {"imus: [", "cameras: ["}) {
    auto sequence = extra;
    const int count = extra[0] == 'i' ? 17 : 9;
    for (int i = 0; i < count; ++i) sequence += (i ? ", {}" : "{}");
    EXPECT_THROW(loadYaml(sequence + "]\n"), std::runtime_error);
  }
  for (const std::string &extra : {
      ", drive: {continue_on_collision: false}, continue_on_collision: true",
      ", drive: {support_links: []}, support_links: []",
      ", support_links: base_link"})
    EXPECT_THROW(loadYaml("", extra), std::runtime_error);
}

TEST(ConfigYamlBoundaries, RoleEntriesPreserveSeparateGeometryAndRejectInvalidSupport) {
  const std::string box = "{type: box, size: [1,1,0.2]}";
  EXPECT_NO_THROW(loadYaml("scene_format: role_entries\n", "",
      "[{id: visual, visual: " + box + "}, {id: collision, collision: " + box +
      "}, {id: sensor, sensor_geometry: " + box + "}]"));
  for (const auto &role : {"visual", "collision", "sensor_geometry"})
    EXPECT_THROW(loadYaml("", "", "[{id: mixed, geometry: " + box + ", " + role +
        ": " + box + "}]"), std::runtime_error);
  EXPECT_THROW(loadYaml("scene_format: role_entries\n", "", "[{id: empty}]"), std::runtime_error);
  EXPECT_THROW(loadYaml("scene_format: role_entries\n", "", "[{id: visual, visual: " + box +
      ", support: true}]"), std::runtime_error);
  for (const std::string &reflectivity : {"-0.1", "1.1"})
    EXPECT_THROW(loadYaml("", "", "[{id: bad, geometry: " + box + ", reflectivity: " +
        reflectivity + "}]"), std::runtime_error);
  for (const std::string &motion : {"{yaw_rate: .nan}", "{yaw_rate: 101}"})
    EXPECT_THROW(loadYaml("", "", "[{id: bad, geometry: " + box + ", motion: " +
        motion + "}]"), std::runtime_error);
  for (const std::string &entity : {
      "{id: sphere, geometry: {type: sphere, radius: 1}, support: true}",
      "{id: tilted, pose: {rpy: [0.1,0,0]}, geometry: {type: box, size: [1,1,0.2]}, support: true}",
      "{id: high, geometry: {type: box, size: [1,1,0.2]}, support: true}"})
    EXPECT_THROW(loadYaml("", "", "[" + entity + "]"), std::runtime_error);
}

TEST(SimulatorBoundaries, IntegratorsRejectEveryNonFiniteScalarAndNegativeTime) {
  for (const auto &values : {std::array<double, 3>{kNaN, 0, .1},
                             std::array<double, 3>{0, kNaN, .1},
                             std::array<double, 3>{0, 0, kNaN},
                             std::array<double, 3>{0, 0, -.1}})
    rejects([&] { integrate(Pose::Identity(), values[0], values[1], values[2]); }, "invalid velocity/time");
  EXPECT_TRUE(integrate(Pose::Identity(), 1, 1, 0).isApprox(Pose::Identity()));
}

TEST(SimulatorBoundaries, JointCommandTimestampsHoldModesAndTerminalDiagnostics) {
  Config c;
  c.base_link = "root";
  c.control_mode = "ros2_control";
  c.drive.model = "multicopter";
  c.drive.support_links.clear();
  c.step_ns = 100000000;
  c.timeout = .2;
  auto model = mimicModel();
  model.collision = {{"body", "root", Shape::box({.2, .2, .2})}};
  model.report = {{"undriven_joints", Json::array()}};
  Simulator sim(c, model, {});
  JointCommand cmd;
  cmd.joints["fixed"] = {JointCommandMode::Hold, 0};
  EXPECT_FALSE(sim.jointCommand(cmd));
  cmd.joints.clear();
  cmd.joints["driver"] = {static_cast<JointCommandMode>(255), 0};
  EXPECT_FALSE(sim.jointCommand(cmd));
  cmd.joints["driver"] = {JointCommandMode::Position, -3};
  EXPECT_FALSE(sim.jointCommand(cmd));
  cmd.joints["driver"] = {JointCommandMode::Position, 3};
  EXPECT_FALSE(sim.jointCommand(cmd));
  cmd.joints["driver"] = {JointCommandMode::Hold, 0};
  ASSERT_TRUE(sim.jointCommand(cmd));
  ASSERT_TRUE(sim.jointCommand(cmd));
  EXPECT_GE(sim.diagnostics()["coalesced_commands"].get<unsigned>(), 1u);
  EXPECT_EQ(sim.diagnostics()["pending_command"], true);
  cmd.stamp = sim.snapshot().stamp + 1;
  EXPECT_FALSE(sim.jointCommand(cmd));
  sim.pause(false);
  sim.tick();
  sim.tick();
  cmd.stamp = 1;
  ASSERT_TRUE(sim.jointCommand(cmd)); // exactly the timeout age remains valid
  sim.tick();
  EXPECT_FALSE(sim.jointCommand(cmd));
  EXPECT_EQ(sim.snapshot().joints.at("driver"), 0);
  sim.exit();
  EXPECT_EQ(sim.diagnostics()["state"], "exited");
  EXPECT_FALSE(sim.jointCommand({}));
  EXPECT_THROW(sim.pause(true), std::runtime_error);
  Simulator faulted(c, model, {});
  faulted.fault("unit fault");
  EXPECT_EQ(faulted.diagnostics()["state"], "fault");
}

TEST(SimulatorBoundaries, StepRequestCacheHasInclusiveLimitsAndNoSilentEviction) {
  Config c;
  c.base_link = "root";
  c.drive.model = "multicopter";
  c.drive.support_links.clear();
  c.odom_rate = 1;
  auto model = mimicModel();
  model.collision = {{"body", "root", Shape::box({.2, .2, .2})}};
  model.report = {{"undriven_joints", Json::array()}};
  Simulator sim(c, model, {});
  sim.pause(true);
  std::vector<Event> events;
  EXPECT_EQ(sim.singleStep(std::string(129, 'a'), 1, events).code, 2);
  EXPECT_EQ(sim.singleStep("too-many", 10001, events).code, 2);
  for (size_t i = 0; i < 4096; ++i) {
    ASSERT_TRUE(sim.singleStep(std::to_string(i), 1, events).success) << i;
    events.clear();
  }
  const auto before = sim.snapshot().stamp;
  EXPECT_EQ(sim.singleStep("full", 1, events).code, 3);
  EXPECT_EQ(sim.snapshot().stamp, before);
  EXPECT_TRUE(sim.singleStep("0", 1, events).success);
  EXPECT_EQ(sim.snapshot().stamp, before);
  sim.exit();
  EXPECT_FALSE(sim.command({}));
}

TEST(SimulatorBoundaries, CollidingPoseAndStaleTwistLeaveCommittedStateUnchanged) {
  Config c;
  c.base_link = "root";
  c.drive.model = "multicopter";
  c.drive.support_links.clear();
  c.step_ns = 100000000;
  c.timeout = .1;
  auto model = mimicModel();
  model.collision = {{"body", "root", Shape::box({.2, .2, .2})}};
  model.report = {{"undriven_joints", Json::array()}};
  Scene scene;
  scene.collision = {{"wall", "", Shape::box({.2, 2, 2}), pose({1, 0, 0})}};
  Simulator sim(c, model, scene);
  const auto before = sim.snapshot();
  rejects([&] { sim.setPose(pose({1, 0, 0})); }, "set_pose collides");
  EXPECT_TRUE(sim.snapshot().world_base.isApprox(before.world_base));
  sim.pause(false);
  sim.tick();
  sim.tick();
  Command command;
  command.stamp = 1;
  EXPECT_FALSE(sim.command(command));
  EXPECT_THROW(sim.setPose(Pose::Identity()), std::runtime_error);
}
