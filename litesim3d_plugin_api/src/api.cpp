#include "litesim3d_plugin_api/api.hpp"
#include <cmath>
#include <pluginlib/class_loader.hpp>
#include <stdexcept>

namespace litesim3d::plugins {
ChassisPlugin::~ChassisPlugin() = default;
OdometryPlugin::~OdometryPlugin() = default;
JointDrivenChassis::~JointDrivenChassis() = default;
SensorPlugin::~SensorPlugin() = default;
struct Loader::Impl {
  pluginlib::ClassLoader<ChassisPlugin> chassis{"litesim3d_plugin_api", "litesim3d::plugins::ChassisPlugin"};
  pluginlib::ClassLoader<OdometryPlugin> odometry{"litesim3d_plugin_api", "litesim3d::plugins::OdometryPlugin"};
  pluginlib::ClassLoader<SensorPlugin> sensor{"litesim3d_plugin_api", "litesim3d::plugins::SensorPlugin"};
};
Loader::Loader() : impl_(std::make_shared<Impl>()) {}
Loader::~Loader() = default;
namespace {
template<class T, class Owner>
std::shared_ptr<T> load(pluginlib::ClassLoader<T> &loader, const std::string &name,
                        const std::shared_ptr<Owner> &owner) {
  try {
    auto plugin = loader.createSharedInstance(name);
    T *pointer = plugin.get();
    // Keep pluginlib alive even when an API consumer destroys its Loader first.
    return std::shared_ptr<T>(pointer, [plugin, owner](T *) mutable { plugin.reset(); });
  } catch (const std::exception &error) {
    throw std::runtime_error("cannot load LiteSim3D plugin '" + name + "': " + error.what());
  }
}
}
std::shared_ptr<ChassisPlugin> Loader::chassis(const std::string &name) { return load(impl_->chassis, name, impl_); }
std::shared_ptr<OdometryPlugin> Loader::odometry(const std::string &name) { return load(impl_->odometry, name, impl_); }
std::shared_ptr<SensorPlugin> Loader::sensor(const std::string &name) { return load(impl_->sensor, name, impl_); }
void require(bool condition, const std::string &message) {
  if (!condition) throw std::runtime_error(message);
}
Json merge(const Json &base, const Json &overlay) {
  if (!base.is_object() || !overlay.is_object()) return overlay;
  Json result = base;
  for (auto it = overlay.begin(); it != overlay.end(); ++it)
    result[it.key()] = result.contains(it.key()) ? merge(result.at(it.key()), it.value()) : it.value();
  return result;
}
Json parameters(const Json &defaults, const Json &overrides, const std::string &path) {
  require(defaults.is_object() && overrides.is_object(), path + ": expected mapping");
  for (auto it = overrides.begin(); it != overrides.end(); ++it)
    require(defaults.contains(it.key()), path + ": unknown parameter " + it.key());
  Json result = defaults;
  for (auto it = defaults.begin(); it != defaults.end(); ++it) {
    const auto key = path + "." + it.key();
    if (it.value().is_object()) {
      result[it.key()] = parameters(it.value(), overrides.value(it.key(), Json::object()), key);
    } else if (overrides.contains(it.key())) result[it.key()] = overrides.at(it.key());
    require(!result.at(it.key()).is_null(), key + ": required parameter is missing");
    const auto &v = result.at(it.key());
    if (!it.value().is_null())
      require((it.value().is_number() && v.is_number()) || it.value().type() == v.type(),
              key + ": incorrect parameter type");
    if (v.is_number()) require(std::isfinite(v.get<double>()), key + ": must be finite");
  }
  return result;
}
double seconds(uint64_t dt_ns) {
  require(dt_ns > 0 && dt_ns <= 1000000000ULL, "plugin dt_ns must be in [1, 1000000000]");
  return static_cast<double>(dt_ns) * 1e-9;
}
Pose integrate(const Pose &initial, const Twist &v, double dt) {
  for (auto value : {v.vx, v.vy, v.vz, v.wz, dt}) require(std::isfinite(value), "non-finite twist/dt");
  require(dt >= 0, "negative dt");
  const double angle = v.wz * dt;
  const double sinc = std::abs(angle) < 1e-8 ? 1 - angle * angle / 6 : std::sin(angle) / angle;
  const double cosc = std::abs(angle) < 1e-8 ? angle / 2 - angle * angle * angle / 24 : (1 - std::cos(angle)) / angle;
  Pose delta = Pose::Identity();
  delta.linear() = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  delta.translation() = Eigen::Vector3d(dt * (v.vx * sinc - v.vy * cosc),
                                       dt * (v.vx * cosc + v.vy * sinc), dt * v.vz);
  return initial * delta;
}
} // namespace litesim3d::plugins
