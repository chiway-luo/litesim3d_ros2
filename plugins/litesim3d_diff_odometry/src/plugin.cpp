#include <litesim3d_plugin_api/motion.hpp>
#include <litesim3d_plugin_api/model_geometry.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace litesim3d::plugins {
class DifferentialOdometry : public OdometryPlugin, public ModelParameters {
public:
  Json resolveModelParameters(const ModelGeometry &geometry, const Json &values) const override {
    return differentialGeometry(geometry, values);
  }
  Json defaults() const override {
    return {{"wheel_radius_m", nullptr}, {"wheel_separation_m", nullptr},
            {"left_joints", nullptr}, {"right_joints", nullptr}, {"noise", EncoderIntegrator::defaults()}};
  }
  void initialize(const Initialization &init, const Json &values) override {
    const auto p = parameters(defaults(), values);
    radius_ = positive(p, "wheel_radius_m"); separation_ = positive(p, "wheel_separation_m");
    left_ = jointNames(p, "left_joints"); right_ = jointNames(p, "right_joints");
    auto names = left_; names.insert(names.end(), right_.begin(), right_.end()); uniqueNames(names);
    for (const auto &name : names) wheelJoint(init.model, name);
    integrator_.initialize(init, p.at("noise"));
  }
  OdometryOutput step(const EncoderInput &input) override {
    auto estimate = [&](bool endpoint) {
      double left = 0, right = 0;
      for (const auto &name : left_) left += encoderRate(input, name, endpoint);
      for (const auto &name : right_) right += encoderRate(input, name, endpoint);
      left *= radius_ / left_.size(); right *= radius_ / right_.size();
      return Twist{(left + right) / 2, 0, 0, (right - left) / separation_};
    };
    return integrator_.update(estimate(false), estimate(true), input.dt_ns);
  }
  void reset(const JointState &joints) override { validateState(joints); integrator_.reset(); }
private:
  EncoderIntegrator integrator_;
  double radius_ = 0, separation_ = 0;
  std::vector<std::string> left_, right_;
};
}
PLUGINLIB_EXPORT_CLASS(litesim3d::plugins::DifferentialOdometry, litesim3d::plugins::OdometryPlugin)
