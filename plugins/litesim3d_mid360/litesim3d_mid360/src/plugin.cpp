#include <litesim3d/sensor_plugin.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace litesim3d::plugins {
class Mid360 final : public SensorPlugin {
public:
  Json defaults() const override {
    return merge(sensorDefaults("lidar"), {
      {"scan_pattern","livox_mid360"}, {"scan_mode","rolling"}, {"columns",20000},
      {"vertical_min_deg",-7.0}, {"vertical_max_deg",52.0}, {"max_range_m",40.0},
      {"pattern_file","package://litesim3d_core/lidar_patterns/mid360.csv"}});
  }
  void initialize(const Initialization &init, const Json &input) override {
    const auto p = parameters(defaults(), input);
    require(p.at("scan_pattern") == "livox_mid360", "Mid360 requires scan_pattern=livox_mid360");
    config_ = lidarSettings(init.instance, p, init.model_directory);
    validateSensorBinding(init, config_.link);
    for (const auto &link : config_.excluded_links) validateSensorBinding(init, link);
    enabled_ = p.at("enabled");
    seed_ = init.seed;
  }
  SensorDescription description() const override {
    auto settings = sensorParameters(config_);
    settings["enabled"] = enabled_;
    return {config_.name, config_.link, config_.topic, "lidar", config_.rate, enabled_, settings};
  }
  void step(const SensorContext &context, SensorOutput &out) override {
    if (!enabled_) return;
    out.clouds.push_back(scanTrajectory(context.scene, config_, context.stamp_ns,
        context.world_sensor, context.robot_in_sensor, seed_, context.sample_index, context.threads));
  }
  void reset(uint64_t) override {}
private:
  LidarConfig config_;
  uint64_t seed_ = 42;
  bool enabled_ = true;
};
} // namespace litesim3d::plugins
PLUGINLIB_EXPORT_CLASS(litesim3d::plugins::Mid360, litesim3d::plugins::SensorPlugin)
