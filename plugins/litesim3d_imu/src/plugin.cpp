#include "litesim3d/sensor_plugin.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace litesim3d::plugins {
class Imu final : public SensorPlugin {
public:
  Json defaults() const override { return sensorDefaults("imu"); }
  void initialize(const Initialization &init, const Json &input) override {
    const auto p = parameters(defaults(),input);
    config_ = imuSettings(init.instance,p);
    validateSensorBinding(init,config_.link);
    enabled_ = p.at("enabled"); seed_ = init.seed;
  }
  SensorDescription description() const override {
    auto settings = sensorParameters(config_); settings["enabled"] = enabled_;
    return {config_.name,config_.link,config_.topic,"imu",config_.rate,enabled_,settings};
  }
  void step(const SensorContext &context, SensorOutput &out) override {
    if (!enabled_) return;
    out.imus.push_back(sampleImu(config_,context.body,context.body_sensor,
        context.stamp_ns,seed_,context.sample_index));
  }
  // Counter-based noise uses the host's sample index/stamp; there is no hidden clock.
  void reset(uint64_t) override {}
private:
  ImuConfig config_;
  uint64_t seed_ = 42;
  bool enabled_ = true;
};
} // namespace litesim3d::plugins
PLUGINLIB_EXPORT_CLASS(litesim3d::plugins::Imu, litesim3d::plugins::SensorPlugin)
