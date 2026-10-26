#include <litesim3d_plugin_api/motion.hpp>
#include <litesim3d_plugin_api/model_geometry.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace litesim3d::plugins {
class OmniOdometry : public OdometryPlugin, public ModelParameters {
public:
  Json resolveModelParameters(const ModelGeometry &geometry, const Json &values) const override {
    return omniGeometry(geometry, values);
  }
  Json defaults() const override { return {{"wheels", nullptr}, {"noise", EncoderIntegrator::defaults()}}; }
  void initialize(const Initialization &init, const Json &values) override {
    const auto p = parameters(defaults(), values);
    wheels_ = wheelConstraints(p.at("wheels"), init.model);
    inverse_ = inverseWheelMatrix(wheelMatrix(wheels_));
    integrator_.initialize(init, p.at("noise"));
  }
  OdometryOutput step(const EncoderInput &input) override {
    auto estimate = [&](bool endpoint) {
      Eigen::VectorXd speed(wheels_.size());
      for (size_t i = 0; i < wheels_.size(); ++i) speed[i] = encoderRate(input, wheels_[i].joint, endpoint);
      const Eigen::Vector3d body = inverse_ * speed;
      return Twist{body.x(), body.y(), 0, body.z()};
    };
    return integrator_.update(estimate(false), estimate(true), input.dt_ns);
  }
  void reset(const JointState &joints) override { validateState(joints); integrator_.reset(); }
private:
  std::vector<WheelConstraint> wheels_;
  Eigen::MatrixXd inverse_;
  EncoderIntegrator integrator_;
};
}
PLUGINLIB_EXPORT_CLASS(litesim3d::plugins::OmniOdometry, litesim3d::plugins::OdometryPlugin)
