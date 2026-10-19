#include "litesim3d/sensor_plugin.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace litesim3d::plugins {
class Lidar final : public SensorPlugin {
public:
  Json defaults() const override { return sensorDefaults("lidar"); }
  void initialize(const Initialization &init, const Json &input) override {
    const auto p = parameters(defaults(),input);
    config_ = lidarSettings(init.instance,p,init.model_directory);
    validateSensorBinding(init,config_.link);
    enabled_ = p.at("enabled"); seed_ = init.seed;
    for (const auto &link : config_.excluded_links) validateSensorBinding(init,link);
  }
  SensorDescription description() const override {
    auto settings = sensorParameters(config_); settings["enabled"] = enabled_;
    return {config_.name,config_.link,config_.topic,"lidar",config_.rate,enabled_,settings};
  }
  void step(const SensorContext &context, SensorOutput &out) override {
    if (!enabled_) return;
    out.clouds.push_back(scanTrajectory(context.scene,config_,context.stamp_ns,
        context.world_sensor,context.robot_in_sensor,seed_,context.sample_index,context.threads));
  }
  // Counter-based noise uses the host's sample index/stamp; there is no hidden clock.
  void reset(uint64_t) override {}
private:
  LidarConfig config_;
  uint64_t seed_ = 42;
  bool enabled_ = true;
};
} // namespace litesim3d::plugins
PLUGINLIB_EXPORT_CLASS(litesim3d::plugins::Lidar, litesim3d::plugins::SensorPlugin)
