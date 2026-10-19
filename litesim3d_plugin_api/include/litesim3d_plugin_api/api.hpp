#pragma once

#include <Eigen/Geometry>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <map>
#include <memory>
#include <nlohmann/json.hpp>
#include <set>
#include <string>
#include <vector>

namespace litesim3d::plugins {
using Json = nlohmann::json;
using Pose = Eigen::Isometry3d;
constexpr unsigned api_version = 1;

// All vectors use base coordinates: +x forward, +y left, +z up. SI units.
struct Twist {
  double vx = 0, vy = 0, vz = 0, wz = 0;
};
struct JointDescription {
  std::string type, mimic, parent_joint;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d axis = Eigen::Vector3d::UnitY();
  double lower = -std::numeric_limits<double>::infinity();
  double upper = std::numeric_limits<double>::infinity();
  double max_velocity = std::numeric_limits<double>::infinity();
};
struct ModelDescription {
  std::string base_link;
  std::set<std::string> links;
  std::map<std::string, JointDescription> joints; // neutral configuration, in base frame
};
struct Initialization {
  std::string instance;
  uint64_t seed = 42;
  ModelDescription model;
  std::filesystem::path model_directory;
};
struct JointState {
  std::map<std::string, double> position, velocity; // continuous angles are unwrapped
};
struct ChassisInput {
  uint64_t dt_ns = 0;
  Twist command, drive_velocity;
  JointState joints;
};
struct DriveStep {
  Twist average, velocity; // interval-average and endpoint body twist
  std::map<std::string, double> wheel_delta; // full-interval encoder displacement, rad
  std::map<std::string, double> steering_position; // ideal actuator target angle, rad
  std::map<std::string, double> joint_velocity; // endpoint velocity, rad/s
};
// No ground-truth pose, scene, or commanded velocity is available to odometry.
struct EncoderInput {
  uint64_t dt_ns = 0;
  JointState previous, current;
};
struct OdometryOutput {
  Pose pose = Pose::Identity(); // odom -> base, starts at identity after reset
  Twist velocity;
};

// Sensor data/services are defined by the core's public sensor support header.
// Forward declarations keep the motion API independent of collision/rendering.
struct SensorContext;
struct SensorOutput;
struct ImageFrame;
struct SensorDescription {
  std::string name, link, topic, kind;
  double rate_hz = 0;
  bool enabled = true;
  Json settings;
};

class ChassisPlugin {
public:
  virtual ~ChassisPlugin();
  // null values denote required parameters; non-null values are actual defaults.
  virtual Json defaults() const = 0;
  virtual void initialize(const Initialization &, const Json &parameters) = 0;
  virtual bool supportsLateral() const { return false; }
  virtual bool supportsVertical() const { return false; }
  // No state is committed by prediction. The host owns collision acceptance.
  virtual DriveStep step(const ChassisInput &) const = 0;
  virtual void reset(const JointState &) = 0;
};
class OdometryPlugin {
public:
  virtual ~OdometryPlugin();
  virtual Json defaults() const = 0;
  virtual void initialize(const Initialization &, const Json &parameters) = 0;
  virtual OdometryOutput step(const EncoderInput &) = 0;
  virtual void reset(const JointState &) = 0;
};
class SensorPlugin {
public:
  virtual ~SensorPlugin();
  virtual Json defaults() const = 0;
  virtual void initialize(const Initialization &, const Json &parameters) = 0;
  virtual SensorDescription description() const = 0;
  virtual void step(const SensorContext &, SensorOutput &) = 0;
  // Camera plugins postprocess rendered pixels; non-camera plugins may ignore this.
  virtual void processImage(ImageFrame &) {}
  virtual void reset(uint64_t stamp_ns) = 0;
};

class Loader {
public:
  Loader();
  ~Loader();
  Loader(const Loader &) = delete;
  Loader &operator=(const Loader &) = delete;
  std::shared_ptr<ChassisPlugin> chassis(const std::string &class_name);
  std::shared_ptr<OdometryPlugin> odometry(const std::string &class_name);
  std::shared_ptr<SensorPlugin> sensor(const std::string &class_name);
private:
  struct Impl;
  std::shared_ptr<Impl> impl_;
};

// Recursive maps; arrays replace as a whole. Unknown and missing keys fail.
Json parameters(const Json &defaults, const Json &overrides, const std::string &path = "parameters");
Json merge(const Json &base, const Json &overlay);
void require(bool condition, const std::string &message);
double seconds(uint64_t dt_ns);
Pose integrate(const Pose &, const Twist &, double dt);
} // namespace litesim3d::plugins
