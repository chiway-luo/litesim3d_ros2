#pragma once

#include <Eigen/Geometry>
#include <array>
#include <cstdint>
#include <deque>
#include <fcl/fcl.h>
#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <nlohmann/json.hpp>
#include <litesim3d_plugin_api/declarations.hpp>
#include <litesim3d_plugin_api/model_geometry.hpp>
#include <optional>
#include <random>
#include <set>
#include <string>
#include <vector>

namespace litesim3d {
using Vec3 = Eigen::Vector3d;
using Pose = Eigen::Isometry3d;
using Json = nlohmann::json;
constexpr double pi = 3.14159265358979323846;
Pose pose(const Vec3 &xyz = Vec3::Zero(), const Vec3 &rpy = Vec3::Zero());
Pose quaternionPose(const Vec3 &xyz, Eigen::Quaterniond q);
double yaw(const Pose &p);
void require(bool condition, const std::string &message);
// Keep the std::string overload for existing consumers; literal diagnostics
// must not allocate on successful checks in parsing and sensor hot paths.
void require(bool condition, const char *message);
std::string sha256(const std::filesystem::path &path);
std::string sha256Text(const std::string &text);
std::string readText(const std::filesystem::path &path);
std::filesystem::path resolveResource(const std::string &uri, const std::filesystem::path &base);
uint64_t streamSeed(uint64_t seed, const std::string &key);
struct Shape;
struct LoadControl {
  size_t byte_budget = SIZE_MAX, triangle_budget = SIZE_MAX;
  bool collision_budget_enabled = false;
  size_t bytes = 0, triangles = 0, checkpoints = 0;
  std::optional<size_t> fail_allocation_after;
  std::function<bool()> cancelled;
  std::map<std::string, std::weak_ptr<Shape>> meshes;
  void checkpoint(size_t additional_bytes = 0, size_t additional_triangles = 0);
};

struct Bounds {
  Vec3 lo = Vec3::Constant(INFINITY), hi = Vec3::Constant(-INFINITY);
  void add(const Vec3 &p);
  void add(const Bounds &b);
  bool ray(const Vec3 &o, const Vec3 &d, double limit) const;
};
struct Triangle {
  Vec3 a, b, c;
};
struct BvhNode {
  Bounds bounds;
  int left = -1, right = -1;
  size_t begin = 0, end = 0;
};
class Bvh {
public:
  void build(const std::vector<Bounds> &boxes);
  void query(const Vec3 &o, const Vec3 &d, double limit,
             const std::function<void(size_t)> &fn) const;

private:
  int partition(size_t begin, size_t end, const std::vector<Bounds> &boxes);
  std::vector<BvhNode> nodes_;
  std::vector<size_t> order_;
};
struct Shape {
  enum class Kind { Box, Sphere, Cylinder, Mesh };
  Kind kind = Kind::Box;
  Vec3 size = Vec3::Ones(); // box xyz, sphere radius x, cylinder radius x / length z
  std::vector<Triangle> triangles;
  Bounds bounds;
  Bvh bvh;
  std::shared_ptr<fcl::CollisionGeometryd> collision;
  std::string resource;
  static std::shared_ptr<Shape> box(const Vec3 &size);
  static std::shared_ptr<Shape> sphere(double radius);
  static std::shared_ptr<Shape> cylinder(double radius, double length);
  static std::shared_ptr<Shape> mesh(const std::filesystem::path &path, const Vec3 &scale,
                                    const std::shared_ptr<LoadControl> &control = {}, bool build_collision = true);
  void ensureCollision();
  double ray(const Vec3 &origin, const Vec3 &direction, double limit) const;
};
struct Motion {
  Pose initial = Pose::Identity();
  Vec3 velocity = Vec3::Zero(); // world coordinates, m/s
  double yaw_rate = 0;
  uint64_t frozen_at = UINT64_MAX;
  std::function<Pose(uint64_t)> path; // shared-world robot trajectory
  Pose at(uint64_t stamp) const;
};
struct Geometry {
  std::string entity, link;
  std::shared_ptr<Shape> shape;
  Pose transform = Pose::Identity();
  double reflectivity = 0.5;
  bool support = false;
  std::shared_ptr<fcl::CollisionGeometryd> support_barriers = nullptr;
  std::shared_ptr<Motion> motion;
  Pose at(uint64_t stamp) const;
  Bounds worldBounds() const;
};
struct Hit {
  double distance;
  std::string entity, link;
  double reflectivity;
};
struct Support {
  Vec3 point, normal;
  std::string entity;
};
struct Scene {
  std::vector<Geometry> visual, collision, sensor;
  Bvh index;
  uint64_t build_count = 0;
  bool tilted_support = false;
  void rebuild();
  void updateEntity(const std::string &id, const Pose &transform);
  std::optional<Support> supportAt(double x, double y, double reference_z,
                                  double max_delta, const std::string &entity = "") const;
  std::optional<Hit> ray(const Vec3 &o, const Vec3 &d, double max_range,
                         const std::vector<Geometry> &robot = {},
                         const std::set<std::string> &excluded_links = {}, uint64_t stamp = 1) const;
};
struct Joint {
  std::string name, parent, child, type, mimic;
  Pose origin = Pose::Identity();
  Vec3 axis = Vec3::UnitX();
  double lower = -INFINITY, upper = INFINITY, velocity = INFINITY;
  double multiplier = 1, offset = 0;
};
struct ModelOptions {
  bool visual_collision_fallback = false, allow_visual_only = false;
  std::string sensor_geometry = "visual";
  std::map<std::string, std::string> sensor_overrides;
  std::shared_ptr<LoadControl> load_control;
};
struct Model {
  std::string root, xml;
  std::map<std::string, Joint> joints;
  std::vector<std::string> order;
  std::set<std::string> links;
  std::vector<Geometry> visual, collision, sensor;
  Json report;
  static Model load(const std::filesystem::path &file, const ModelOptions &options = {});
  std::map<std::string, double> positions(const std::map<std::string, double> &q) const;
  std::map<std::string, double> advance(const std::map<std::string, double> &q,
                                        const std::map<std::string, double> &targets,
                                        double dt) const;
  std::map<std::string, Pose> fk(const std::map<std::string, double> &q) const;
  plugins::ModelGeometry mechanicalGeometry(const std::string &base_link) const;
  std::vector<Geometry> placed(const std::vector<Geometry> &geometries, const Pose &world_base,
                               const std::string &base_link,
                               const std::map<std::string, double> &q) const;
};
struct LidarPatternTable {
  struct Sample { Vec3 direction; double azimuth, elevation; };
  std::vector<Sample> samples;
  std::string source, source_sha256;
  // Livox's Mid-360 CSV uses consecutive sample ordinals despite its Time/s header.
  static std::shared_ptr<const LidarPatternTable> load(
      const std::filesystem::path &file, const std::shared_ptr<LoadControl> &control = {});
};
struct LidarConfig {
  std::string name = "lidar", link = "lidar_link", topic = "points_raw";
  std::vector<double> vertical_angles;
  uint32_t columns = 1024;
  double rate = 10, min_range = 0.1, max_range = 100;
  double horizontal_start = -pi, horizontal_end = pi;
  std::string filter = "sensor_link";
  std::set<std::string> excluded_links;
  std::string scan_mode = "snapshot";
  // Spatial scan pattern is independent of rolling/snapshot time sampling.
  std::string scan_pattern = "spinning";
  double vertical_min = -pi / 2, vertical_max = pi / 2;
  double cone_half_angle = 19.2 * pi / 180, pattern_hz = 100;
  std::shared_ptr<const LidarPatternTable> pattern_table;
  std::vector<double> ring_delays; // seconds, relative to the beginning of each column
  double range_bias = 0, range_sigma = 0, dropout = 0;
  // Optional synthetic endpoints on topic + "/range_boundary"; never measured returns.
  bool publish_range_boundary = false;
  void validate() const;
  Vec3 direction(size_t ring, size_t column, uint64_t frame = 0) const;
  uint64_t offsetNs(size_t ring, size_t column) const;
};
struct Cloud {
  uint64_t stamp = 0;
  std::string frame, topic;
  uint32_t height = 0, width = 0, point_step = 24, row_step = 0;
  bool dense = true;
  std::vector<uint8_t> data;
  // Same organized xyzirt layout; only geometrically unobstructed rays are finite.
  std::vector<uint8_t> range_boundary;
  uint64_t completion_stamp = 0;
  double compute_ms = 0;
  uint64_t compute_end_ns = 0;
  uint64_t frame_index = 0;
};
Cloud scan(const Scene &scene, const LidarConfig &config, const Pose &world_lidar, uint64_t stamp,
           const std::vector<Geometry> &robot = {});
// Counter-based random streams: scheduling, other sensors and dropout never shift range noise.
double randomUniform(uint64_t key, uint64_t sample, uint64_t lane = 0);
double randomNormal(uint64_t key, uint64_t sample, uint64_t lane = 0);
Cloud scanTrajectory(const Scene &scene, const LidarConfig &config, uint64_t start,
                     const std::function<Pose(uint64_t)> &world_sensor,
                     const std::vector<Geometry> &robot_in_sensor = {},
                     uint64_t seed = 42, uint64_t frame = 0, unsigned threads = 1);
struct TerrainConfig {
  bool enabled = false;
  double max_slope = 20 * pi / 180, clearance = 0, continuity = 0.02;
  void validate() const;
};
Pose supportedPose(const Scene &scene, const Pose &candidate, double reference_z,
                   const TerrainConfig &config, const std::vector<Vec3> &support_points);
struct ImuConfig {
  std::string name = "imu", link = "base_link", topic = "imu/data_raw";
  double rate = 200;
  Vec3 accel_bias = Vec3::Zero(), gyro_bias = Vec3::Zero();
  Vec3 accel_sigma = Vec3::Zero(), gyro_sigma = Vec3::Zero();
  double gravity = 9.81; // world -Z magnitude, m/s^2; configurable per IMU
  double accel_range = 0, gyro_range = 0; // zero disables clipping; SI units
  void validate() const;
};
// Pinhole sensor bound to a ROS optical link: +z forward, +x right, +y down.
struct CameraConfig {
  std::string name = "front_camera", link = "camera_optical_frame", topic = "camera/front";
  std::string type = "rgb";
  bool enabled = true;
  unsigned width = 640, height = 480;
  double rate = 15, horizontal_fov = 70 * pi / 180, near_clip = .1, far_clip = 20;
  Vec3 color_bias = Vec3::Zero(), color_sigma = Vec3::Zero(); // RGB digital numbers [0,255]
  double depth_bias = 0, depth_sigma = 0; // metres
  bool color() const { return type == "rgb" || type == "rgbd"; }
  bool depth() const { return type == "depth" || type == "rgbd"; }
  void validate() const;
  Json json() const;
};
struct Kinematics {
  Pose world_body = Pose::Identity();
  Vec3 acceleration = Vec3::Zero(), omega = Vec3::Zero(), alpha = Vec3::Zero(); // world frame
};
struct ImuSample {
  uint64_t stamp = 0;
  std::string frame, topic;
  Vec3 acceleration = Vec3::Zero(), angular_velocity = Vec3::Zero();
  std::array<double, 9> orientation_covariance{-1, 0, 0, 0, 0, 0, 0, 0, 0};
  std::array<double, 9> acceleration_covariance{}, angular_velocity_covariance{};
};
ImuSample sampleImu(const ImuConfig &config, const Kinematics &body, const Pose &body_imu,
                    uint64_t stamp, uint64_t seed = 42, uint64_t sample = 0);
struct OdomError {
  double linear_scale = 1, angular_scale = 1, linear_bias = 0, angular_bias = 0;
  double linear_sigma = 0, angular_sigma = 0;
  bool enabled = false;
  void validate() const;
};
struct Drive {
  std::string model = "differential_drive";
  double wheelbase = 1, max_steering = 0.6;
  double wheel_radius = 0.1, wheel_separation = 0.5;
  double max_v = 1, max_w = 2, acceleration = 0.5, deceleration = 1, angular_acceleration = 2;
  bool unlimited_acceleration = false;
  // Keep wheel encoders running against a blocked chassis (never disables collision).
  bool continue_on_collision = false;
  std::string left_joint = "left_wheel_joint", right_joint = "right_wheel_joint";
  std::set<std::string> support_links = {"left_wheel", "right_wheel"};
  void validate() const;
};
Pose integrate(const Pose &initial, double v, double w, double dt);
// Level, velocity-controlled multicopter: body XYZ translation and yaw.
Pose integrateFlight(const Pose &initial, const Vec3 &velocity, double yaw_rate, double dt);
struct Config {
  std::string control_mode = "twist";
  uint64_t step_ns = 5000000, seed = 42;
  bool collision_budget_enabled = false;
  double odom_rate = 50, timeout = 0.5, real_time_factor = 1;
  std::string robot_id = "robot_0", base_link = "base_link";
  Pose initial_pose = Pose::Identity();
  Drive drive;
  std::set<std::string> inferred_drive_fields; // omitted legacy YAML geometry, resolved from URDF
  TerrainConfig terrain;
  OdomError odom_error;
  unsigned threads = 1;
  size_t history_limit = 8192, journal_limit = 4096;
  std::map<std::string, double> initial_joints;
  std::vector<LidarConfig> lidars;
  std::vector<ImuConfig> imus;
  std::vector<CameraConfig> cameras;
  std::array<double, 36> pose_covariance{}, twist_covariance{};
  std::filesystem::path urdf;
  ModelOptions model_options;
  Json resolved, resources;
  std::vector<plugins::Declaration> plugins;
  bool sensors_resolved = false; // typed arrays below are the loaded plugin descriptors
  static Config load(const std::filesystem::path &file, Scene &scene,
                     const std::shared_ptr<LoadControl> &control = {});
  uint64_t timeoutPhaseNs() const;
  void validate() const;
};
void validateCollisionBudget(const Config &, const Model &, const Scene &, size_t robots = 1);
enum class State { Ready, Running, Paused, Fault, Exited };
struct Command {
  double v = 0, w = 0, lateral = 0, vertical = 0, roll_rate = 0, pitch_rate = 0;
  std::optional<uint64_t> stamp;
  std::string frame;
};
enum class JointCommandMode : uint8_t { Hold = 0, Position = 1, Velocity = 2 };
struct JointTarget {
  JointCommandMode mode = JointCommandMode::Hold;
  double value = 0;
};
struct JointCommand {
  std::map<std::string, JointTarget> joints;
  std::optional<uint64_t> stamp;
  std::optional<plugins::Twist> body_velocity; // Kinematic multicopter only; base frame.
};
struct Snapshot {
  uint64_t stamp = 1;
  Pose world_base = Pose::Identity(), odom_base = Pose::Identity(), odom_root = Pose::Identity();
  std::map<std::string, double> joints, joint_velocities;
  double v = 0, w = 0;
  double lateral = 0, vertical = 0;
  double odom_v = 0, odom_w = 0, odom_lateral = 0, odom_vertical = 0;
  Vec3 linearVelocity() const { return Vec3(v, lateral, vertical); }
};
struct Event {
  Snapshot snapshot;
  bool odom = false;
  std::vector<Cloud> clouds;
  std::vector<ImuSample> imus;
  std::vector<size_t> cameras; // indexes in Config; poses belong to this exact snapshot
};
struct Contact {
  uint64_t stamp;
  std::string robot_link, entity;
  double clearance;
};
struct StepResult {
  bool success = false;
  uint8_t code = 0;
  std::string message;
  uint64_t time_ns = 0;
};
class Simulator {
public:
  Simulator(Config config, Model model, Scene scene);
  const Config &config() const { return config_; }
  const Model &model() const { return model_; }
  const Scene &scene() const { return scene_; }
  const Snapshot &snapshot() const { return current_; }
  State state() const { return state_; }
  uint64_t stepIndex() const { return step_; }
  bool command(const Command &cmd);
  bool jointCommand(const JointCommand &cmd);
  void pause(bool paused);
  std::vector<Event> tick();
  StepResult singleStep(const std::string &request_id, uint32_t count, std::vector<Event> &output);
  void setPose(const Pose &initial);
  void fault(const std::string &reason);
  void exit();
  Json manifest(const std::string &reason = "running") const;
  Json diagnostics() const;
  const std::vector<Contact> &contacts() const { return contacts_; }
  uint64_t contactCount() const { return contact_count_; }
  void processCamera(size_t index, plugins::ImageFrame &image);
  std::string run_id;

private:
  friend class World;
  void completeScans(std::vector<Event> &events);
  Cloud makeCloud(const Cloud &pending, const Snapshot &snapshot) const;
  void stopForStep(const std::string &entity, const std::string &link, double distance);
  struct DynamicContact { std::shared_ptr<Motion> motion; Contact contact; };
  std::vector<DynamicContact> dynamicContacts(double dt) const;
  struct MotionStep {
    double old_v, old_w, new_v, new_w;
    Pose old_pose;
    Vec3 old_linear = Vec3::Zero(), new_linear = Vec3::Zero();
    plugins::DriveStep plugin_drive;
  };
  void beginStep();
  uint64_t nextEvent(uint64_t end) const;
  MotionStep prepareMotion(uint64_t next);
  plugins::DriveStep prepareJointMotion(double dt) const;
  std::optional<Event> advanceMotion(uint64_t next, const MotionStep &motion);
  void finishStep();
  std::vector<Vec3> supportPoints() const;
  bool defer_scans_ = false, forced_stop_ = false;
  uint64_t peer_history_ns_ = 0;
  std::vector<Event> advanceStep();
  void move(double dt, uint64_t end_stamp);
  std::optional<Contact> clearance(const Pose &p, double &distance,
      const std::map<std::string, double> *joints = nullptr,
      const std::set<std::string> *moving_links = nullptr) const;
  void initializeMotionPlugins();
  void initializeSensorPlugins(const plugins::Initialization &);
  plugins::SensorOutput sampleSensor(const std::string &name, uint64_t stamp,
      uint64_t sample_index, const Snapshot &snapshot) const;
  Kinematics sensorKinematics(const Snapshot &, const std::map<std::string, Pose> &frames,
      const std::string &link) const;
  void moveSteering(const std::map<std::string, double> &targets);
  Pose trajectory(uint64_t stamp) const;
  Config config_;
  Model model_;
  Scene scene_;
  // The loader outlives its instances; the API also protects external owners.
  std::unique_ptr<plugins::Loader> plugin_loader_;
  std::shared_ptr<plugins::ChassisPlugin> chassis_plugin_;
  std::shared_ptr<plugins::OdometryPlugin> odometry_plugin_;
  std::map<std::string, std::shared_ptr<plugins::SensorPlugin>> sensor_plugins_;
  std::map<std::string, plugins::SensorDescription> sensor_descriptions_;
  plugins::Twist plugin_drive_velocity_;
  double accepted_motion_fraction_ = 1;
  Snapshot current_;
  Pose origin_;
  State state_ = State::Ready;
  uint64_t step_ = 0, accepted_stamp_ = 1, sequence_ = 0;
  bool started_ = false, timed_out_ = false;
  Command target_;
  std::optional<Command> pending_;
  JointCommand joint_target_;
  std::optional<JointCommand> pending_joints_;
  plugins::JointDrivenChassis *joint_chassis_ = nullptr;
  bool body_control_ = false;
  std::set<std::string> driven_joints_, passive_joints_;
  std::map<std::string, std::pair<double, double>> joint_position_limits_;
  std::vector<uint64_t> lidar_indices_;
  std::vector<uint64_t> imu_indices_;
  std::vector<uint64_t> camera_indices_;
  struct History { uint64_t stamp; Pose pose; };
  std::deque<History> history_;
  Kinematics kinematics_;
  std::map<std::string, double> joint_accelerations_;
  uint64_t motion_samples_ = 0, contact_count_ = 0, discontinuities_ = 0, evicted_inputs_ = 0;
  bool motion_blocked_ = false, drive_blocked_ = false;
  double drive_v_ = 0, drive_w_ = 0;
  uint64_t odom_index_ = 1;
  std::map<std::string, std::pair<uint32_t, StepResult>> requests_;
  std::vector<Contact> contacts_;
  Json inputs_ = Json::array();
  uint64_t rejected_ = 0, coalesced_ = 0;
  std::string last_error_;
};

struct RobotSpec { std::string id; Pose initial = Pose::Identity(); };
class World {
public:
  // Called synchronously after each robot's sensors finish, with all peer
  // histories already committed. The sink must not mutate/reenter this World.
  using EventSink = std::function<void(size_t, const std::vector<Event> &)>;
  World(Config config, Model model, Scene scene, std::vector<RobotSpec> robots);
  World(const World &) = delete;
  World(World &&) = delete; // Peer trajectories retain this world's stable address.
  static std::unique_ptr<World> load(const std::filesystem::path &file);
  size_t size() const { return robots_.size(); }
  const Simulator &robot(size_t index) const { return *robots_.at(index); }
  bool command(size_t index, const Command &command);
  bool jointCommand(size_t index, const JointCommand &command);
  void pause(bool paused);
  void fault(const std::string &reason);
  void exit();
  std::vector<std::vector<Event>> tick(const EventSink &sink = {});
  StepResult singleStep(const std::string &id, uint32_t count, std::vector<std::vector<Event>> &events);
  Json manifest() const;
  std::string run_id;
private:
  std::vector<std::vector<Event>> advance(const EventSink &sink = {});
  std::vector<std::unique_ptr<Simulator>> robots_;
  std::map<std::string,std::pair<uint32_t,StepResult>> requests_;
  bool paused_ = true;
};
} // namespace litesim3d
