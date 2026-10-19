#pragma once
#include "litesim3d_plugin_api/api.hpp"
#include <Eigen/Core>

namespace litesim3d::plugins {
double number(const Json &, const std::string &key);
double positive(const Json &, const std::string &key);
std::vector<std::string> jointNames(const Json &, const std::string &key);
const JointDescription &wheelJoint(const ModelDescription &, const std::string &name);
const JointDescription &steeringJoint(const ModelDescription &, const std::string &name, double max_angle);
void uniqueNames(const std::vector<std::string> &names);
double encoderRate(const EncoderInput &, const std::string &joint, bool endpoint);
double encoderPosition(const JointState &, const std::string &joint);
void validateState(const JointState &);

struct MotionLimits {
  double linear = 1, angular = 2, acceleration = .5, deceleration = 1, angular_acceleration = 2;
  bool unlimited = false;
  static Json defaults();
  void configure(const Json &);
  DriveStep predict(const ChassisInput &, bool lateral = false, bool vertical = false) const;
};
void limitWheelSpeed(DriveStep &, const ModelDescription &);

// Generic ordinary omni-wheel constraint: row * [vx,vy,wz] = wheel angular speed.
// Passive rollers permit motion perpendicular to rolling_direction.
struct WheelConstraint {
  std::string joint;
  double x = 0, y = 0, angle = 0, radius = 0;
  Eigen::RowVector3d row() const;
};
std::vector<WheelConstraint> wheelConstraints(const Json &, const ModelDescription &, size_t required_count = 0);
Eigen::MatrixXd wheelMatrix(const std::vector<WheelConstraint> &);
Eigen::MatrixXd inverseWheelMatrix(const Eigen::MatrixXd &);

class EncoderIntegrator {
public:
  static Json defaults();
  void initialize(const Initialization &, const Json &noise);
  OdometryOutput update(Twist interval, Twist endpoint, uint64_t dt_ns);
  void reset();
private:
  Pose pose_ = Pose::Identity();
  Eigen::Vector2d linear_scale_{1, 1}, linear_bias_{0, 0}, linear_sigma_{0, 0};
  double angular_scale_ = 1, angular_bias_ = 0, angular_sigma_ = 0;
  uint64_t seed_ = 42, sample_ = 0;
};
} // namespace litesim3d::plugins
