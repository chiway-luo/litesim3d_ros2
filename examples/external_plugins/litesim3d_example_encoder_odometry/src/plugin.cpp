#include <litesim3d_plugin_api/motion.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace litesim3d::plugins {
// This example is deliberately independent of every provided implementation.
// It depends only on the public API and has no simulator source dependency.
class ExampleEncoderOdometry : public OdometryPlugin {
public:
  Json defaults() const override {
    return {{"left_joint", nullptr}, {"right_joint", nullptr}, {"wheel_radius_m", nullptr},
            {"wheel_separation_m", nullptr}, {"distance_scale", 1.0}};
  }
  void initialize(const Initialization &init, const Json &values) override {
    const auto p = parameters(defaults(), values);
    left_ = p.at("left_joint").get<std::string>(); right_ = p.at("right_joint").get<std::string>();
    uniqueNames({left_, right_}); wheelJoint(init.model, left_); wheelJoint(init.model, right_);
    radius_ = positive(p, "wheel_radius_m"); track_ = positive(p, "wheel_separation_m");
    scale_ = positive(p, "distance_scale"); pose_.setIdentity();
  }
  OdometryOutput step(const EncoderInput &input) override {
    auto estimate = [&](bool endpoint) {
      const double left = encoderRate(input, left_, endpoint) * radius_;
      const double right = encoderRate(input, right_, endpoint) * radius_;
      return Twist{scale_ * (left + right) / 2, 0, 0, (right - left) / track_};
    };
    pose_ = integrate(pose_, estimate(false), seconds(input.dt_ns));
    return {pose_, estimate(true)};
  }
  void reset(const JointState &joints) override { validateState(joints); pose_.setIdentity(); }
private:
  std::string left_, right_;
  double radius_ = 0, track_ = 0, scale_ = 1;
  Pose pose_ = Pose::Identity();
};
}
PLUGINLIB_EXPORT_CLASS(litesim3d::plugins::ExampleEncoderOdometry, litesim3d::plugins::OdometryPlugin)
