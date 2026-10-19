#include <litesim3d_plugin_api/motion.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace litesim3d::plugins {
class DifferentialDrive : public ChassisPlugin {
public:
  Json defaults() const override {
    return {{"wheel_radius_m", nullptr}, {"wheel_separation_m", nullptr},
            {"left_joints", nullptr}, {"right_joints", nullptr}, {"limits", MotionLimits::defaults()}};
  }
  void initialize(const Initialization &init, const Json &values) override {
    const auto p = parameters(defaults(), values);
    radius_ = positive(p, "wheel_radius_m"); separation_ = positive(p, "wheel_separation_m");
    left_ = jointNames(p, "left_joints"); right_ = jointNames(p, "right_joints");
    auto names = left_; names.insert(names.end(), right_.begin(), right_.end()); uniqueNames(names);
    model_ = init.model; limits_.configure(p.at("limits"));
    for (const auto &side : {std::make_pair(&left_, 1.0), std::make_pair(&right_, -1.0)})
      for (const auto &name : *side.first) {
        const auto &joint = wheelJoint(model_, name);
        require(std::abs(joint.position.y() - side.second * separation_ / 2) < 1e-6,
                "wheel_separation_m disagrees with URDF: " + name);
        require((joint.axis - Eigen::Vector3d::UnitY()).norm() < 1e-6, "differential wheel axis must be base +y: " + name);
      }
  }
  DriveStep step(const ChassisInput &input) const override {
    auto result = limits_.predict(input);
    const double dt = seconds(input.dt_ns);
    for (const auto &side : {std::make_pair(&left_, -1.0), std::make_pair(&right_, 1.0)})
      for (const auto &name : *side.first) {
        encoderPosition(input.joints, name);
        result.wheel_delta[name] = (result.average.vx + side.second * result.average.wz * separation_ / 2) * dt / radius_;
        result.joint_velocity[name] = (result.velocity.vx + side.second * result.velocity.wz * separation_ / 2) / radius_;
      }
    limitWheelSpeed(result, model_);
    return result;
  }
  void reset(const JointState &joints) override {
    validateState(joints);
    for (const auto &side : {left_, right_}) for (const auto &name : side) encoderPosition(joints, name);
  }
private:
  ModelDescription model_;
  MotionLimits limits_;
  double radius_ = 0, separation_ = 0;
  std::vector<std::string> left_, right_;
};
}
PLUGINLIB_EXPORT_CLASS(litesim3d::plugins::DifferentialDrive, litesim3d::plugins::ChassisPlugin)
