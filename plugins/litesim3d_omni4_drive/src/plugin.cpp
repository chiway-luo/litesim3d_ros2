#include <litesim3d_plugin_api/motion.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace litesim3d::plugins {
class Omni4Drive : public ChassisPlugin, public JointDrivenChassis {
public:
  Json defaults() const override { return {{"wheels", nullptr}, {"limits", MotionLimits::defaults()}}; }
  void initialize(const Initialization &init, const Json &values) override {
    const auto p = parameters(defaults(), values);
    model_ = init.model; wheels_ = wheelConstraints(p.at("wheels"), model_, 4);
    limits_.configure(p.at("limits"));
    inverse_ = inverseWheelMatrix(wheelMatrix(wheels_));
  }
  bool supportsLateral() const override { return true; }
  std::set<std::string> drivenJoints() const override {
    std::set<std::string> names;
    for (const auto &wheel : wheels_) names.insert(wheel.joint);
    return names;
  }
  Twist forward(const JointState &state) const override {
    Eigen::VectorXd rates(wheels_.size());
    for (size_t i = 0; i < wheels_.size(); ++i) {
      rates[i] = state.velocity.at(wheels_[i].joint);
      require(std::isfinite(rates[i]), "non-finite omni wheel velocity");
    }
    const Eigen::Vector3d body = inverse_ * rates;
    return {body.x(), body.y(), 0, body.z()};
  }
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
  Eigen::MatrixXd inverse_;
};
}
PLUGINLIB_EXPORT_CLASS(litesim3d::plugins::Omni4Drive, litesim3d::plugins::ChassisPlugin)
