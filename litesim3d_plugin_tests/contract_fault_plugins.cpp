// Deliberately invalid in-process unit-test doubles for the host's plugin
// boundary checks. Built only with BUILD_TESTING; never a simulator plugin.
#include <litesim3d/sensor_plugin.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <limits>

namespace litesim3d::plugins {
class ContractFaultChassis : public ChassisPlugin, public JointDrivenChassis {
public:
  Json defaults() const override { return {{"case", "ok"}}; }
  void initialize(const Initialization &, const Json &p) override { mode_ = p.at("case"); }
  void reset(const JointState &) override {}
  std::set<std::string> drivenJoints() const override {
    if (mode_ == "driven_empty") return {};
    if (mode_ == "driven_missing") return {"missing"};
    if (mode_ == "driven_type") return {"fixed"};
    if (mode_ == "driven_mimic") return {"mimic"};
    return {"drive"};
  }
  std::set<std::string> passiveJoints() const override {
    if (mode_ == "passive_missing") return {"missing"};
    if (mode_ == "passive_type") return {"fixed"};
    if (mode_ == "passive_mimic") return {"mimic"};
    if (mode_ == "passive_overlap") return {"drive"};
    return {"passive"};
  }
  std::map<std::string, std::pair<double, double>> positionLimits() const override {
    const double bad = std::numeric_limits<double>::quiet_NaN();
    if (mode_ == "limits_missing") return {{"missing", {-.5, .5}}};
    if (mode_ == "limits_type") return {{"drive", {-.5, .5}}};
    if (mode_ == "limits_mimic") return {{"steer_mimic", {-.5, .5}}};
    if (mode_ == "limits_nan_lower") return {{"steer", {bad, .5}}};
    if (mode_ == "limits_nan_upper") return {{"steer", {-.5, bad}}};
    if (mode_ == "limits_initial_low") return {{"steer", {.1, .5}}};
    if (mode_ == "limits_initial_high") return {{"steer", {-.5, -.1}}};
    return {{"steer", {-.5, .5}}};
  }
  Twist forward(const JointState &) const override { return {}; }
  std::map<std::string, double> passiveVelocities(const JointState &, const Twist &) const override {
    if (mode_ == "passive_rate_name") return {{"missing", 0}};
    if (mode_ == "passive_rate_nan") return {{"passive", std::numeric_limits<double>::quiet_NaN()}};
    return {{"passive", 0}};
  }
  DriveStep step(const ChassisInput &) const override {
    const double bad = std::numeric_limits<double>::quiet_NaN();
    DriveStep out;
    std::string wheel = "drive";
    if (mode_ == "wheel_missing") wheel = "missing";
    if (mode_ == "wheel_type") wheel = "fixed";
    if (mode_ == "wheel_mimic") wheel = "mimic";
    out.wheel_delta[wheel] = mode_ == "wheel_delta_nan" ? bad : 0;
    if (mode_ != "wheel_velocity_missing") out.joint_velocity[wheel] = mode_ == "wheel_velocity_nan" ? bad : 0;
    if (mode_ == "steering_type") out.steering_position["drive"] = 0;
    if (mode_ == "steering_mimic") out.steering_position["steer_mimic"] = 0;
    if (mode_ == "steering_nan") out.steering_position["steer"] = bad;
    if (mode_ == "steering_low") out.steering_position["steer"] = -1;
    if (mode_ == "steering_high") out.steering_position["steer"] = 1;
    return out;
  }
private:
  std::string mode_;
};

class ContractFaultOdometry : public OdometryPlugin {
public:
  Json defaults() const override { return Json::object(); }
  void initialize(const Initialization &, const Json &) override {}
  void reset(const JointState &) override {}
  OdometryOutput step(const EncoderInput &) override { return {}; }
};

class ContractFaultSensor : public SensorPlugin {
public:
  Json defaults() const override { return {{"case", "ok"}, {"kind", "imu"}}; }
  void initialize(const Initialization &init, const Json &p) override {
    mode_ = p.at("case");
    desc_.name = init.instance;
    desc_.link = init.model.base_link;
    desc_.topic = "fault/output";
    desc_.kind = p.at("kind");
    desc_.rate_hz = 10;
    desc_.enabled = mode_ != "disabled";
    desc_.settings = parameters(sensorDefaults(desc_.kind), {{"link", desc_.link}, {"topic", desc_.topic},
        {"rate_hz", desc_.rate_hz}, {"enabled", desc_.enabled}});
    if (mode_ == "descriptor_name") desc_.name = "wrong";
    if (mode_ == "rate_nan") desc_.rate_hz = std::numeric_limits<double>::quiet_NaN();
    if (mode_ == "rate_zero") desc_.rate_hz = 0;
    if (mode_ == "rate_high") desc_.rate_hz = 2001;
    if (mode_ == "settings_link") desc_.settings["link"] = "wrong";
    if (mode_ == "settings_topic") desc_.settings["topic"] = "wrong";
    if (mode_ == "settings_rate") desc_.settings["rate_hz"] = 11;
    if (mode_ == "settings_enabled") desc_.settings["enabled"] = !desc_.enabled;
  }
  SensorDescription description() const override { return desc_; }
  void reset(uint64_t) override {}
  void step(const SensorContext &ctx, SensorOutput &out) override {
    if (desc_.kind == "imu") {
      ImuSample sample;
      sample.stamp = mode_ == "imu_stamp" ? 0 : ctx.stamp_ns;
      sample.frame = mode_ == "imu_frame" ? "wrong" : desc_.link;
      sample.topic = mode_ == "imu_topic" ? "wrong" : desc_.topic;
      if (mode_ != "imu_empty") out.imus.push_back(sample);
      if (mode_ == "imu_cloud") out.clouds.emplace_back();
      if (mode_ == "imu_camera") out.cameras.push_back(desc_.name);
    } else {
      out.cameras.push_back(mode_ == "camera_name" ? "wrong" : desc_.name);
      if (mode_ == "camera_cloud") out.clouds.emplace_back();
      if (mode_ == "camera_imu") out.imus.emplace_back();
    }
  }
private:
  std::string mode_;
  SensorDescription desc_;
};
} // namespace litesim3d::plugins

PLUGINLIB_EXPORT_CLASS(litesim3d::plugins::ContractFaultChassis, litesim3d::plugins::ChassisPlugin)
PLUGINLIB_EXPORT_CLASS(litesim3d::plugins::ContractFaultOdometry, litesim3d::plugins::OdometryPlugin)
PLUGINLIB_EXPORT_CLASS(litesim3d::plugins::ContractFaultSensor, litesim3d::plugins::SensorPlugin)
