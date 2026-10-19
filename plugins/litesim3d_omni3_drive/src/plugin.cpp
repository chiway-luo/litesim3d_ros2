#include <litesim3d_plugin_api/motion.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace litesim3d::plugins {
class Omni3Drive : public ChassisPlugin {
public:
  Json defaults() const override { return {{"wheels", nullptr}, {"limits", MotionLimits::defaults()}}; }
  void initialize(const Initialization &init, const Json &values) override {
    const auto p = parameters(defaults(), values);
    model_ = init.model; wheels_ = wheelConstraints(p.at("wheels"), model_, 3);
    limits_.configure(p.at("limits"));
  }
  bool supportsLateral() const override { return true; }
  DriveStep step(const ChassisInput &input) const override {
    auto result = limits_.predict(input, true);
    const double dt = seconds(input.dt_ns);
    for (const auto &wheel : wheels_) {
      encoderPosition(input.joints, wheel.joint);
      result.wheel_delta[wheel.joint] = wheel.row().dot(Eigen::Vector3d(result.average.vx, result.average.vy, result.average.wz)) * dt;
      result.joint_velocity[wheel.joint] = wheel.row().dot(Eigen::Vector3d(result.velocity.vx, result.velocity.vy, result.velocity.wz));
    }
    limitWheelSpeed(result, model_);
    return result;
  }
  void reset(const JointState &joints) override {
    validateState(joints);
    for (const auto &wheel : wheels_) encoderPosition(joints, wheel.joint);
  }
private:
  ModelDescription model_;
  MotionLimits limits_;
  std::vector<WheelConstraint> wheels_;
};
}
PLUGINLIB_EXPORT_CLASS(litesim3d::plugins::Omni3Drive, litesim3d::plugins::ChassisPlugin)
