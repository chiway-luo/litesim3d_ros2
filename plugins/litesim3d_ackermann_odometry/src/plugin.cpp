#include <litesim3d_plugin_api/motion.hpp>
#include <litesim3d_plugin_api/model_geometry.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <array>
#include <cmath>

namespace litesim3d::plugins {
class AckermannOdometry : public OdometryPlugin, public ModelParameters {
public:
  Json resolveModelParameters(const ModelGeometry &geometry, const Json &values) const override {
    return ackermannGeometry(geometry, values);
  }
  Json defaults() const override {
    return {{"wheel_radius_m", nullptr}, {"wheelbase_m", nullptr}, {"track_width_m", nullptr},
            {"rear_left_joint", nullptr}, {"rear_right_joint", nullptr},
            {"left_steering_joint", nullptr}, {"right_steering_joint", nullptr},
            {"max_steering_rad", .6}, {"noise", EncoderIntegrator::defaults()}};
  }
  void initialize(const Initialization &init, const Json &values) override {
    const auto p = parameters(defaults(), values);
    radius_ = positive(p, "wheel_radius_m"); length_ = positive(p, "wheelbase_m"); track_ = positive(p, "track_width_m");
    max_angle_ = positive(p, "max_steering_rad"); require(max_angle_ < 1.5707963267948966, "max_steering_rad must be less than pi/2");
    wheels_ = {p.at("rear_left_joint"), p.at("rear_right_joint")};
    steering_ = {p.at("left_steering_joint"), p.at("right_steering_joint")};
    uniqueNames({wheels_[0], wheels_[1], steering_[0], steering_[1]});
    for (const auto &name : wheels_) wheelJoint(init.model, name);
    for (const auto &name : steering_) steeringJoint(init.model, name, max_angle_);
    integrator_.initialize(init, p.at("noise"));
  }
  OdometryOutput step(const EncoderInput &input) override {
    double curvature = 0;
    for (size_t i = 0; i < 2; ++i) {
      const double angle = encoderPosition(input.current, steering_[i]);
      require(std::abs(angle) <= max_angle_ + 1e-9, "encoder steering angle exceeds configured limit");
      const double t = std::tan(angle), denominator = length_ + (i ? -1 : 1) * track_ / 2 * t;
      require(denominator > 1e-12, "invalid steering geometry");
      curvature += .5 * t / denominator;
    }
    auto estimate = [&](bool endpoint) {
      const double v = radius_ * (encoderRate(input, wheels_[0], endpoint) + encoderRate(input, wheels_[1], endpoint)) / 2;
      return Twist{v, 0, 0, v * curvature};
    };
    return integrator_.update(estimate(false), estimate(true), input.dt_ns);
  }
  void reset(const JointState &joints) override { validateState(joints); integrator_.reset(); }
private:
  EncoderIntegrator integrator_;
  std::array<std::string, 2> wheels_, steering_;
  double radius_ = 0, length_ = 0, track_ = 0, max_angle_ = 0;
};
}
PLUGINLIB_EXPORT_CLASS(litesim3d::plugins::AckermannOdometry, litesim3d::plugins::OdometryPlugin)
