#include "litesim3d_plugin_api/model_geometry.hpp"
#include "litesim3d_plugin_api/motion.hpp"
#include <algorithm>
#include <cmath>

namespace litesim3d::plugins {
ModelParameters::~ModelParameters() = default;
namespace {
void derived(Json &p, const std::string &key, double value, bool angle = false) {
  require(std::isfinite(value), key + ": invalid URDF geometry");
  if (p.contains(key)) {
    const double old = number(p, key);
    const double difference = angle ? std::atan2(std::sin(old-value), std::cos(old-value)) : old-value;
    require(std::abs(difference) < 1e-6, key + ": configured geometry disagrees with URDF; remove the duplicate value");
  }
  p[key] = value;
}
double radius(const ModelGeometry &g, const std::string &name) {
  wheelJoint(g.model, name);
  require(g.wheel_radii.count(name), "wheel requires an unambiguous centered coaxial collision cylinder: " + name);
  const double r = g.wheel_radii.at(name);
  require(std::isfinite(r) && r > 0, "invalid URDF wheel radius: " + name);
  return r;
}
double commonRadius(const ModelGeometry &g, const std::vector<std::string> &names) {
  uniqueNames(names);
  require(!names.empty(), "wheel bindings must not be empty");
  const double r = radius(g, names.front());
  for (const auto &name : names)
    require(std::abs(radius(g, name) - r) < 1e-6, "this chassis requires equal URDF wheel radii: " + name);
  return r;
}
}

Json differentialGeometry(const ModelGeometry &g, const Json &values) {
  Json p = values;
  const auto left = jointNames(p, "left_joints"), right = jointNames(p, "right_joints");
  auto names = left; names.insert(names.end(), right.begin(), right.end());
  derived(p, "wheel_radius_m", commonRadius(g, names));
  const double ly = wheelJoint(g.model, left.front()).position.y();
  const double ry = wheelJoint(g.model, right.front()).position.y();
  require(ly > 0 && ry < 0 && std::abs(ly + ry) < 1e-6,
          "differential base must be centered between left/right wheel groups");
  for (const auto &side : {std::make_pair(&left, ly), std::make_pair(&right, ry)})
    for (const auto &name : *side.first) {
      const auto &j = wheelJoint(g.model, name);
      require(std::abs(j.position.y() - side.second) < 1e-6 &&
                  (j.axis - Eigen::Vector3d::UnitY()).norm() < 1e-6,
              "invalid differential URDF wheel layout/axis: " + name);
    }
  derived(p, "wheel_separation_m", ly - ry);
  return p;
}

Json ackermannGeometry(const ModelGeometry &g, const Json &values) {
  Json p = values;
  std::vector<std::string> wheels{p.at("rear_left_joint"), p.at("rear_right_joint")};
  const std::vector<std::string> steering{p.at("left_steering_joint"), p.at("right_steering_joint")};
  double bound = INFINITY;
  for (const auto &name : steering) {
    const auto &j = steeringJoint(g.model, name, 0);
    bound = std::min({bound, -j.lower, j.upper});
    std::vector<std::string> children;
    for (const auto &[child, wheel] : g.model.joints)
      if (wheel.parent_joint == name && wheel.type == "continuous" && wheel.mimic.empty()) children.push_back(child);
    require(children.size() == 1, "steering joint must have exactly one independent wheel child: " + name);
    wheels.push_back(children.front());
  }
  for (size_t i = 0; i < 2; ++i) {
    const auto key = i ? "front_right_joint" : "front_left_joint";
    if (p.contains(key)) require(p.at(key) == wheels[i + 2], std::string(key) + ": does not match URDF steering child");
  }
  derived(p, "wheel_radius_m", commonRadius(g, wheels));
  const double length = wheelJoint(g.model, wheels[2]).position.x();
  const double track = wheelJoint(g.model, wheels[0]).position.y() - wheelJoint(g.model, wheels[1]).position.y();
  require(length > 0 && track > 0, "invalid Ackermann URDF wheelbase/track");
  for (size_t i = 0; i < wheels.size(); ++i) {
    const auto &j = wheelJoint(g.model, wheels[i]);
    const Eigen::Vector2d expected(i < 2 ? 0 : length, (i % 2 ? -1 : 1) * track / 2);
    require((j.position.head<2>() - expected).norm() < 1e-6 &&
                (j.axis - Eigen::Vector3d::UnitY()).norm() < 1e-6,
            "Ackermann base must be rear-axle midpoint; invalid wheel layout/axis: " + wheels[i]);
  }
  derived(p, "wheelbase_m", length); derived(p, "track_width_m", track);
  require(std::isfinite(bound) && bound > 0 && bound < 1.5707963267948966, "invalid URDF steering limits");
  // An explicit lower operational cap remains meaningful; it cannot expand mechanical limits.
  if (p.contains("max_steering_rad")) {
    const double cap = positive(p, "max_steering_rad");
    require(cap <= bound + 1e-9, "max_steering_rad exceeds URDF limits");
    bound = std::min(bound, cap);
  }
  p["max_steering_rad"] = bound;
  return p;
}

Json omniGeometry(const ModelGeometry &g, const Json &values) {
  Json p = values;
  require(p.contains("wheels") && p.at("wheels").is_array(), "wheels: expected joint bindings");
  for (auto &entry : p["wheels"]) {
    require(entry.is_object() && entry.contains("joint"), "wheels[]: missing joint binding");
    const auto name = entry.at("joint").get<std::string>();
    const auto &j = wheelJoint(g.model, name);
    require(std::abs(j.axis.z()) < 1e-6 && std::abs(j.axis.norm()-1) < 1e-6,
            "omni wheel axis must be a unit vector in the base xy plane: " + name);
    derived(entry, "x_m", j.position.x()); derived(entry, "y_m", j.position.y());
    derived(entry, "rolling_angle_rad", std::atan2(-j.axis.x(), j.axis.y()), true);
    derived(entry, "radius_m", radius(g, name));
  }
  wheelConstraints(p.at("wheels"), g.model);
  return p;
}
} // namespace litesim3d::plugins
