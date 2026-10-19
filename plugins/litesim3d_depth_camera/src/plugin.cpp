#include "litesim3d/sensor_plugin.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace litesim3d::plugins {
class DepthCamera final : public SensorPlugin {
public:
  Json defaults() const override { return sensorDefaults("depth"); }
  void initialize(const Initialization &init, const Json &input) override {
    const auto p = parameters(defaults(),input);
    config_ = cameraSettings(init.instance,"depth",p);
    validateSensorBinding(init,config_.link);
    enabled_ = p.at("enabled"); seed_ = init.seed;
  }
  SensorDescription description() const override {
    auto settings = sensorParameters(config_); settings["enabled"] = enabled_;
    return {config_.name,config_.link,config_.topic,"depth",config_.rate,enabled_,settings};
  }
  void step(const SensorContext &context, SensorOutput &out) override {
    if (!enabled_) return;
    (void)context;
    out.cameras.push_back(config_.name);
  }
  void processImage(ImageFrame &image) override { applyCameraNoise(config_,seed_,image); }
  // Counter-based noise uses the host's sample index/stamp; there is no hidden clock.
  void reset(uint64_t) override {}
private:
  CameraConfig config_;
  uint64_t seed_ = 42;
  bool enabled_ = true;
};
} // namespace litesim3d::plugins
PLUGINLIB_EXPORT_CLASS(litesim3d::plugins::DepthCamera, litesim3d::plugins::SensorPlugin)
