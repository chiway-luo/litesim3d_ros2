#include "litesim3d_plugin_api/motion.hpp"
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>

namespace litesim3d::plugins {
double number(const Json &p, const std::string &key) {
  require(p.contains(key) && p.at(key).is_number(), key + ": expected number");
  const double value = p.at(key).get<double>();
  require(std::isfinite(value), key + ": must be finite");
  return value;
}
double positive(const Json &p, const std::string &key) {
  const double value = number(p, key);
  require(value > 0, key + ": must be positive");
  return value;
}
std::vector<std::string> jointNames(const Json &p, const std::string &key) {
  require(p.at(key).is_array() && !p.at(key).empty() && p.at(key).size() <= 16,
          key + ": expected 1..16 joint names");
  auto names = p.at(key).get<std::vector<std::string>>();
  uniqueNames(names);
  return names;
}
void uniqueNames(const std::vector<std::string> &names) {
  std::set<std::string> used;
  for (const auto &name : names)
    require(!name.empty() && used.insert(name).second, "empty/duplicate joint name: " + name);
}
const JointDescription &wheelJoint(const ModelDescription &model, const std::string &name) {
  require(model.joints.count(name), "wheel joint not found: " + name);
  const auto &joint = model.joints.at(name);
  require(joint.type == "continuous" && joint.mimic.empty(), "wheel must be independent continuous joint: " + name);
  require(joint.position.allFinite() && joint.axis.allFinite(), "invalid wheel transform: " + name);
  require(joint.max_velocity > 0, "wheel velocity limit must be positive: " + name);
  return joint;
}
const JointDescription &steeringJoint(const ModelDescription &model, const std::string &name, double angle) {
  require(model.joints.count(name), "steering joint not found: " + name);
  const auto &joint = model.joints.at(name);
  require(joint.type == "revolute" && joint.mimic.empty(), "steering must be independent revolute joint: " + name);
  require((joint.axis - Eigen::Vector3d::UnitZ()).norm() < 1e-6, "steering axis must be base +z: " + name);
  require(joint.lower <= -angle && joint.upper >= angle, "steering limits do not cover max_steering_rad: " + name);
  return joint;
}
double encoderPosition(const JointState &state, const std::string &joint) {
  require(state.position.count(joint), "missing encoder position: " + joint);
  const double value = state.position.at(joint);
  require(std::isfinite(value), "non-finite encoder position: " + joint);
  return value;
}
double encoderRate(const EncoderInput &input, const std::string &joint, bool endpoint) {
  if (!endpoint) return (encoderPosition(input.current, joint) - encoderPosition(input.previous, joint)) / seconds(input.dt_ns);
  require(input.current.velocity.count(joint), "missing encoder velocity: " + joint);
  const double value = input.current.velocity.at(joint);
  require(std::isfinite(value), "non-finite encoder velocity: " + joint);
  return value;
}
void validateState(const JointState &state) {
  for (const auto *values : {&state.position, &state.velocity})
    for (const auto &[name, value] : *values) require(std::isfinite(value), "non-finite joint state: " + name);
}
Json MotionLimits::defaults() {
  return {{"max_linear_speed_mps", 1.0}, {"max_angular_speed_radps", 2.0},
          {"acceleration_mps2", .5}, {"deceleration_mps2", 1.0},
          {"angular_acceleration_radps2", 2.0}, {"unlimited_acceleration", false}};
}
void MotionLimits::configure(const Json &p) {
  linear = positive(p, "max_linear_speed_mps"); angular = positive(p, "max_angular_speed_radps");
  acceleration = positive(p, "acceleration_mps2"); deceleration = positive(p, "deceleration_mps2");
  angular_acceleration = positive(p, "angular_acceleration_radps2");
  unlimited = p.at("unlimited_acceleration").get<bool>();
}
DriveStep MotionLimits::predict(const ChassisInput &input, bool lateral, bool vertical) const {
  const double dt = seconds(input.dt_ns);
  for (const auto *v : {&input.command, &input.drive_velocity})
    for (auto value : {v->vx, v->vy, v->vz, v->wz}) require(std::isfinite(value), "non-finite chassis command/state");
  require(lateral || input.command.vy == 0, "chassis does not support lateral motion");
  require(vertical || input.command.vz == 0, "chassis does not support vertical motion");
  Eigen::Vector3d desired(input.command.vx, lateral ? input.command.vy : 0, vertical ? input.command.vz : 0);
  Eigen::Vector3d previous(input.drive_velocity.vx, lateral ? input.drive_velocity.vy : 0, vertical ? input.drive_velocity.vz : 0);
  if (desired.stableNorm() > linear) desired *= linear / desired.stableNorm();
  auto next = desired.eval();
  if (!unlimited) {
    Eigen::Vector3d delta = desired - previous;
    const double bound = ((desired.dot(previous) < 0 || desired.stableNorm() < previous.stableNorm()) ? deceleration : acceleration) * dt;
    if (delta.stableNorm() > bound) delta *= bound / delta.stableNorm();
    next = previous + delta;
  }
  double next_w = std::clamp(input.command.wz, -angular, angular);
  if (!unlimited) next_w = input.drive_velocity.wz + std::clamp(next_w - input.drive_velocity.wz, -angular_acceleration * dt, angular_acceleration * dt);
  Eigen::Vector3d average = unlimited ? next : Eigen::Vector3d((previous + next) / 2);
  DriveStep result;
  result.velocity = {next.x(), next.y(), next.z(), next_w};
  result.average = {average.x(), average.y(), average.z(), unlimited ? next_w : (input.drive_velocity.wz + next_w) / 2};
  return result;
}
void limitWheelSpeed(DriveStep &step, const ModelDescription &model) {
  double factor = 1;
  for (const auto &[name, speed] : step.joint_velocity) {
    require(std::isfinite(speed), "non-finite predicted joint velocity: " + name);
    const double limit = model.joints.at(name).max_velocity;
    if (std::abs(speed) > limit) factor = std::min(factor, limit / std::abs(speed));
  }
  for (auto *v : {&step.average, &step.velocity}) {
    v->vx *= factor; v->vy *= factor; v->vz *= factor; v->wz *= factor;
  }
  for (auto &[name, value] : step.wheel_delta) value *= factor;
  for (auto &[name, value] : step.joint_velocity) value *= factor;
}
Eigen::RowVector3d WheelConstraint::row() const {
  const double c = std::cos(angle), s = std::sin(angle);
  return Eigen::RowVector3d(c / radius, s / radius, (x * s - y * c) / radius);
}
std::vector<WheelConstraint> wheelConstraints(const Json &p, const ModelDescription &model, size_t count) {
  require(p.is_array() && (count ? p.size() == count : (p.size() == 3 || p.size() == 4)), "wheels: wrong ordinary omni wheel count");
  std::vector<WheelConstraint> result;
  std::vector<std::string> names;
  for (const auto &entry : p) {
    const auto item = parameters({{"joint", nullptr}, {"x_m", nullptr}, {"y_m", nullptr},
                                 {"rolling_angle_rad", nullptr}, {"radius_m", nullptr}}, entry, "wheels[]");
    WheelConstraint wheel;
    wheel.joint = item.at("joint").get<std::string>();
    wheel.x = number(item, "x_m"); wheel.y = number(item, "y_m");
    wheel.angle = number(item, "rolling_angle_rad"); wheel.radius = positive(item, "radius_m");
    const auto &joint = wheelJoint(model, wheel.joint);
    require((joint.position.head<2>() - Eigen::Vector2d(wheel.x, wheel.y)).norm() < 1e-6,
            "wheel position disagrees with URDF: " + wheel.joint);
    const Eigen::Vector3d axis(-std::sin(wheel.angle), std::cos(wheel.angle), 0);
    require((axis - joint.axis).norm() < 1e-6, "wheel axis disagrees with rolling_angle_rad: " + wheel.joint);
    names.push_back(wheel.joint); result.push_back(wheel);
  }
  uniqueNames(names);
  inverseWheelMatrix(wheelMatrix(result)); // reject degenerate/nearly singular layouts at initialization
  return result;
}
Eigen::MatrixXd wheelMatrix(const std::vector<WheelConstraint> &wheels) {
  Eigen::MatrixXd result(wheels.size(), 3);
  for (size_t i = 0; i < wheels.size(); ++i) result.row(i) = wheels[i].row();
  return result;
}
Eigen::MatrixXd inverseWheelMatrix(const Eigen::MatrixXd &matrix) {
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
  require(svd.singularValues().size() == 3 && svd.singularValues()[2] > 1e-8 * svd.singularValues()[0],
          "ordinary omni wheel layout is singular or ill-conditioned (requires rank 3)");
  return svd.solve(Eigen::MatrixXd::Identity(matrix.rows(), matrix.rows()));
}
Json EncoderIntegrator::defaults() {
  return {{"linear_scale", {1.0, 1.0}}, {"linear_bias_mps", {0.0, 0.0}}, {"linear_sigma_mps", {0.0, 0.0}},
          {"angular_scale", 1.0}, {"angular_bias_radps", 0.0}, {"angular_sigma_radps", 0.0}};
}
void EncoderIntegrator::initialize(const Initialization &init, const Json &noise) {
  const auto p = parameters(defaults(), noise, "noise");
  auto vector = [&](const std::string &key) {
    require(p.at(key).is_array() && p.at(key).size() == 2, "noise." + key + ": expected [x,y]");
    Eigen::Vector2d v(p.at(key)[0].get<double>(), p.at(key)[1].get<double>());
    require(v.allFinite(), "noise." + key + ": values must be finite"); return v;
  };
  linear_scale_ = vector("linear_scale"); linear_bias_ = vector("linear_bias_mps"); linear_sigma_ = vector("linear_sigma_mps");
  angular_scale_ = positive(p, "angular_scale"); angular_bias_ = number(p, "angular_bias_radps"); angular_sigma_ = number(p, "angular_sigma_radps");
  require((linear_scale_.array() > 0).all() && (linear_sigma_.array() >= 0).all() && angular_sigma_ >= 0, "invalid odometry noise scale/sigma");
  seed_ = init.seed;
  for (unsigned char c : init.instance) seed_ = (seed_ ^ c) * 1099511628211ULL;
  reset();
}
namespace {
uint64_t mix(uint64_t x) {
  x += 0x9e3779b97f4a7c15ULL;
  x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ULL;
  x = (x ^ (x >> 27)) * 0x94d049bb133111ebULL;
  return x ^ (x >> 31);
}
double normal(uint64_t key) {
  const double u = ((mix(key) >> 11) + .5) / 9007199254740992.0;
  const double v = ((mix(key + 1) >> 11) + .5) / 9007199254740992.0;
  return std::sqrt(-2 * std::log(u)) * std::cos(6.28318530717958647692 * v);
}
}
OdometryOutput EncoderIntegrator::update(Twist interval, Twist endpoint, uint64_t dt_ns) {
  const double dt = seconds(dt_ns);
  const double bx = linear_bias_.x() + linear_sigma_.x() * normal(seed_ + sample_ * 6);
  const double by = linear_bias_.y() + linear_sigma_.y() * normal(seed_ + sample_ * 6 + 2);
  const double bw = angular_bias_ + angular_sigma_ * normal(seed_ + sample_ * 6 + 4);
  for (auto *v : {&interval, &endpoint}) {
    v->vx = v->vx * linear_scale_.x() + bx; v->vy = v->vy * linear_scale_.y() + by;
    v->wz = v->wz * angular_scale_ + bw;
  }
  pose_ = integrate(pose_, interval, dt);
  ++sample_;
  return {pose_, endpoint};
}
void EncoderIntegrator::reset() { pose_.setIdentity(); sample_ = 0; }
} // namespace litesim3d::plugins
