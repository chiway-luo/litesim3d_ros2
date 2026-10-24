#pragma once
#include "litesim3d/sim.hpp"

namespace litesim3d::plugins {
// Read-only truth at this sensor deadline. Rolling lidar uses world_sensor(t).
// No wheel odometry is present. Scene and trajectory are valid only during step().
struct SensorContext {
  const Scene &scene;
  Kinematics body;
  Pose body_sensor;
  std::function<Pose(uint64_t)> world_sensor;
  std::vector<Geometry> robot_in_sensor;
  uint64_t stamp_ns, completion_ns, sample_index;
  unsigned threads;
};
struct SensorOutput {
  std::vector<Cloud> clouds;
  std::vector<ImuSample> imus;
  std::vector<std::string> cameras; // request the shared renderer for this instance
};
// Optional capability: keeps SensorPlugin and SensorContext ABI/semantics intact.
// Kinematics describes the mounted link origin/orientation in world coordinates,
// including the accepted joint motion. Context.body remains the base body truth.
class LinkKinematicsSensor {
public:
  virtual ~LinkKinematicsSensor() = default;
  virtual void stepWithKinematics(const SensorContext &, const Kinematics &sensor,
      SensorOutput &) = 0;
};
struct ImageFrame {
  uint64_t stamp_ns = 0;
  unsigned width = 0, height = 0;
  std::vector<uint8_t> color; // packed RGB8
  std::vector<uint8_t> depth; // packed 32FC1
  bool depth_big_endian = false;
};

// Common schemas/conversion for the supported ROS output families. A custom
// plugin may have its own parameters, but describes output with these settings.
Json sensorDefaults(const std::string &kind);
LidarConfig lidarSettings(const std::string &name, const Json &,
    const std::filesystem::path &base = {});
ImuConfig imuSettings(const std::string &name, const Json &);
CameraConfig cameraSettings(const std::string &name, const std::string &kind, const Json &);
Json sensorParameters(const LidarConfig &);
Json sensorParameters(const ImuConfig &);
Json sensorParameters(const CameraConfig &);
void validateSensorBinding(const Initialization &, const std::string &link);
void applyCameraNoise(const CameraConfig &, uint64_t seed, ImageFrame &);
} // namespace litesim3d::plugins
