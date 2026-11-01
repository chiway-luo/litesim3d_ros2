#include "model.hpp"
#include <ament_index_cpp/get_package_prefix.hpp>
#include <cmath>
#include <set>

namespace litesim3d::bringup {
namespace {
std::vector<std::string> names(const Json &parameters, const std::string &key, std::size_t count = 0) {
  require(parameters.contains(key) && parameters[key].is_array() && !parameters[key].empty(), key + ": invalid joint bindings");
  const auto &values = parameters[key];
  require(!count || values.size() == count, key + ": invalid joint bindings");
  std::set<std::string> unique; std::vector<std::string> result;
  for (const auto &value : values) {
    require(value.is_string() && !value.get<std::string>().empty(), key + ": invalid joint bindings");
    const auto text = value.get<std::string>();
    require(unique.insert(text).second, key + ": invalid joint bindings"); result.push_back(text);
  }
  return result;
}
bool equal(const Json &actual, const Json &expected, bool angle) {
  if (expected.is_array()) {
    if (!actual.is_array() || actual.size() != expected.size()) return false;
    for (std::size_t i = 0; i < expected.size(); ++i) if (!equal(actual[i], expected[i], angle)) return false;
    return true;
  }
  if (!actual.is_number() || !expected.is_number()) return false;
  const auto a = actual.get<double>(), b = expected.get<double>();
  if (!std::isfinite(a) || !std::isfinite(b)) return false;
  const auto delta = a - b;
  return std::abs(angle ? std::atan2(std::sin(delta), std::cos(delta)) : delta) < 1e-6;
}
void derived(Json &parameters, const std::string &key, const Json &value, bool angle = false) {
  require(!parameters.contains(key) || equal(parameters[key], value, angle),
    key + ": configured geometry disagrees with URDF; remove the duplicate value");
  parameters[key] = value;
}
}
Json model_geometry(const std::string &description, const std::string &base, const Json &requests) {
  tinyxml2::XMLDocument document; parse_xml(document, description);
  pin_resources(document.RootElement(), fs::current_path());
  TemporaryDirectory directory("litesim-controller-model-");
  const auto model = directory.path() / "robot.urdf"; write(model, xml_text(document));
  const auto tool = fs::path(ament_index_cpp::get_package_prefix("litesim3d_core")) / "bin" / "model_geometry";
  return Json::parse(checked_run({tool.string(), model.string(), base, "--resolve"}, 60, requests.dump()));
}
Json controller_parameters(Json result, const std::string &description, const std::string &base, const GeometryProvider &provider) {
  require(result.is_object(), "controllers: expected a ROS parameter mapping");
  std::map<std::string, std::string> types;
  for (const auto &node : result) {
    if (!node.is_object() || !node.contains("ros__parameters") || !node["ros__parameters"].is_object()) continue;
    for (auto it = node["ros__parameters"].begin(); it != node["ros__parameters"].end(); ++it) {
      const auto &value = it.value();
      if (!value.is_object() || !value.contains("type") || !value["type"].is_string()) continue;
      const auto implementation = value["type"].get<std::string>();
      require(!types.count(it.key()) || types[it.key()] == implementation, "ambiguous controller type: " + it.key());
      types[it.key()] = implementation;
    }
  }
  const std::map<std::string, std::string> supported = {
    {"diff_drive_controller/DiffDriveController", "differential"},
    {"ackermann_steering_controller/AckermannSteeringController", "ackermann"},
    {"litesim3d_ros2_control/OmniDriveController", "omni"}};
  std::vector<std::pair<std::string, Json *>> targets; auto requests = Json::array();
  for (const auto &[name, implementation] : types) {
    if (!supported.count(implementation)) continue;
    const auto kind = supported.at(implementation); std::vector<Json *> matches;
    for (auto it = result.begin(); it != result.end(); ++it) {
      std::string key = it.key(); key.erase(key.find_last_not_of('/') + 1);
      if (key.substr(key.find_last_of('/') + 1) == name) matches.push_back(&it.value());
    }
    require(matches.size() == 1 && matches[0]->is_object() && matches[0]->contains("ros__parameters")
      && (*matches[0])["ros__parameters"].is_object(), name + ": require one unambiguous controller parameter section");
    auto &p = (*matches[0])["ros__parameters"];
    require(p.value("base_frame_id", base) == base, name + ": base_frame_id differs from the URDF geometry reference " + base);
    Json bindings;
    if (kind == "differential") bindings = {{"left_joints", names(p, "left_wheel_names")}, {"right_joints", names(p, "right_wheel_names")}};
    else if (kind == "ackermann") {
      require(!p.contains("front_steering") || p["front_steering"] == true, "URDF geometry adapter currently requires front steering");
      const auto rear = names(p, "rear_wheels_names", 2), front = names(p, "front_wheels_names", 2);
      bindings = {{"rear_left_joint", rear[1]}, {"rear_right_joint", rear[0]}, {"left_steering_joint", front[1]}, {"right_steering_joint", front[0]}};
    } else {
      bindings = {{"wheels", Json::array()}};
      for (const auto &name : names(p, "wheel_joints")) bindings["wheels"].push_back({{"joint", name}});
    }
    targets.emplace_back(kind, &p); requests.push_back({{"kind", kind}, {"parameters", bindings}});
  }
  if (requests.empty()) return result;
  const auto geometry = provider(description, base, requests);
  require(geometry.contains("resolved") && geometry["resolved"].is_array() && geometry["resolved"].size() == targets.size(),
    "model_geometry returned an incomplete controller result");
  for (std::size_t i = 0; i < targets.size(); ++i) {
    const auto &[kind, target] = targets[i]; auto &p = *target; const auto &values = geometry["resolved"][i];
    if (kind == "differential") {
      derived(p, "wheel_radius", values.at("wheel_radius_m")); derived(p, "wheel_separation", values.at("wheel_separation_m"));
    } else if (kind == "ackermann") {
      for (const auto &[key, source] : std::map<std::string, std::string>{{"wheelbase", "wheelbase_m"},
          {"front_wheel_track", "track_width_m"}, {"rear_wheel_track", "track_width_m"},
          {"front_wheels_radius", "wheel_radius_m"}, {"rear_wheels_radius", "wheel_radius_m"}}) derived(p, key, values.at(source));
    } else {
      for (const auto &[key, source] : std::map<std::string, std::string>{{"wheel_x", "x_m"}, {"wheel_y", "y_m"},
          {"wheel_radius", "radius_m"}, {"wheel_angle", "rolling_angle_rad"}}) {
        auto list = Json::array(); for (const auto &wheel : values.at("wheels")) list.push_back(wheel.at(source));
        derived(p, key, list, key == "wheel_angle");
      }
      auto limits = Json::array();
      for (const auto &name : names(p, "wheel_joints")) {
        const auto limit = geometry.at("joints").at(name).at("max_velocity");
        require(limit.is_number() && std::isfinite(limit.get<double>()) && limit.get<double>() > 0,
          "omni wheels require finite positive URDF velocity limits"); limits.push_back(limit);
      }
      if (p.contains("max_wheel_velocity")) {
        const auto &caps = p["max_wheel_velocity"];
        require(caps.is_array() && caps.size() == limits.size(), "max_wheel_velocity exceeds URDF limits");
        for (std::size_t j = 0; j < caps.size(); ++j)
          require(caps[j].is_number() && std::isfinite(caps[j].get<double>()) && caps[j].get<double>() > 0 && caps[j] <= limits[j],
            "max_wheel_velocity exceeds URDF limits");
      } else p["max_wheel_velocity"] = limits;
    }
  }
  return result;
}
}  // namespace litesim3d::bringup
