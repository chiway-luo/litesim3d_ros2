#include <litesim3d_plugin_api/motion.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <algorithm>
#include <array>
#include <cmath>

namespace litesim3d::plugins {
class AckermannDrive : public ChassisPlugin {
public:
  Json defaults() const override {
    return {{"wheel_radius_m", nullptr}, {"wheelbase_m", nullptr}, {"track_width_m", nullptr},
            {"rear_left_joint", nullptr}, {"rear_right_joint", nullptr},
            {"front_left_joint", nullptr}, {"front_right_joint", nullptr},
            {"left_steering_joint", nullptr}, {"right_steering_joint", nullptr},
            {"max_steering_rad", .6}, {"limits", MotionLimits::defaults()}};
  }
  void initialize(const Initialization &init, const Json &values) override {
    const auto p = parameters(defaults(), values);
    radius_ = positive(p, "wheel_radius_m"); length_ = positive(p, "wheelbase_m"); track_ = positive(p, "track_width_m");
    max_steering_ = positive(p, "max_steering_rad");
    require(max_steering_ < 1.5707963267948966, "max_steering_rad must be less than pi/2");
    wheels_ = {p.at("rear_left_joint"), p.at("rear_right_joint"), p.at("front_left_joint"), p.at("front_right_joint")};
    steering_ = {p.at("left_steering_joint"), p.at("right_steering_joint")};
    std::vector<std::string> names(wheels_.begin(), wheels_.end());
    names.insert(names.end(), steering_.begin(), steering_.end()); uniqueNames(names);
    model_ = init.model; limits_.configure(p.at("limits"));
    for (size_t i = 0; i < wheels_.size(); ++i) {
      const auto &joint = wheelJoint(model_, wheels_[i]);
      const Eigen::Vector2d expected(i < 2 ? 0 : length_, (i % 2 ? -1 : 1) * track_ / 2);
      require((joint.position.head<2>() - expected).norm() < 1e-6, "Ackermann base must be rear-axle midpoint; wheel location mismatch: " + wheels_[i]);
      require((joint.axis - Eigen::Vector3d::UnitY()).norm() < 1e-6, "Ackermann neutral wheel axis must be base +y: " + wheels_[i]);
      if (i >= 2) require(joint.parent_joint == steering_[i - 2], "front wheel must be a child of its steering joint: " + wheels_[i]);
    }
    for (const auto &name : steering_) steeringJoint(model_, name, max_steering_);
    // Limit the inner road wheel, not just the virtual bicycle steering angle.
    max_curvature_ = std::tan(max_steering_) / (length_ + track_ / 2 * std::tan(max_steering_));
  }
  DriveStep step(const ChassisInput &input) const override {
    auto result = limits_.predict(input);
    const double curvature = std::abs(result.velocity.vx) > 1e-12
        ? std::clamp(result.velocity.wz / result.velocity.vx, -max_curvature_, max_curvature_) : 0;
    // Ideal steering actuator: angle is selected at this interval's boundary.
    result.velocity.wz = result.velocity.vx * curvature;
    result.average.wz = result.average.vx * curvature;
    std::array<double, 2> angles{};
    for (size_t i = 0; i < 2; ++i) {
      // atan2(tan(limit)) can exceed the declared limit by one ulp.
      angles[i] = std::clamp(std::atan2(length_ * curvature, 1 - (i ? -1 : 1) * track_ / 2 * curvature),
                             -max_steering_, max_steering_);
      encoderPosition(input.joints, steering_[i]);
      result.steering_position[steering_[i]] = angles[i];
      result.joint_velocity[steering_[i]] = 0; // settled endpoint of an ideal position actuator
    }
    const double dt = seconds(input.dt_ns);
    for (size_t i = 0; i < wheels_.size(); ++i) {
      encoderPosition(input.joints, wheels_[i]);
      const double x = i < 2 ? 0 : length_, y = (i % 2 ? -1 : 1) * track_ / 2;
      const double angle = i < 2 ? 0 : angles[i - 2];
      auto speed = [&](const Twist &v) { return ((v.vx - v.wz * y) * std::cos(angle) + v.wz * x * std::sin(angle)) / radius_; };
      result.wheel_delta[wheels_[i]] = speed(result.average) * dt;
      result.joint_velocity[wheels_[i]] = speed(result.velocity);
    }
    limitWheelSpeed(result, model_);
    return result;
  }
  void reset(const JointState &joints) override {
    validateState(joints);
    for (const auto &name : wheels_) encoderPosition(joints, name);
    for (const auto &name : steering_) encoderPosition(joints, name);
  }
private:
  ModelDescription model_;
  MotionLimits limits_;
  std::array<std::string, 4> wheels_;
  std::array<std::string, 2> steering_;
  double radius_ = 0, length_ = 0, track_ = 0, max_steering_ = 0, max_curvature_ = 0;
};
}
PLUGINLIB_EXPORT_CLASS(litesim3d::plugins::AckermannDrive, litesim3d::plugins::ChassisPlugin)
