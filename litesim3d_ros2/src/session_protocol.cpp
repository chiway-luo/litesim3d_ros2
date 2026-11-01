#include "session_protocol.hpp"
#include <cmath>
#include <regex>

namespace litesim3d::session {
Members validate_members(const Json &definitions, const std::set<std::string> &input_topics) {
  require(definitions.is_array() && definitions.size() <= 32, "members must be a bounded list");
  Members result; result.definitions = definitions;
  const std::regex node_name("[A-Za-z_][A-Za-z0-9_]*");
  for (const auto &member : definitions) {
    require(member.is_object() && member.contains("name") && member.contains("argv") && member["argv"].is_array()
      && !member["argv"].empty(), "each member requires name and nonempty argv");
    for (auto it = member.begin(); it != member.end(); ++it)
      require(it.key() == "name" || it.key() == "argv" || it.key() == "ready_topics", "each member requires name and nonempty argv");
    for (const auto &value : member["argv"]) require(value.is_string() && !value.get<std::string>().empty(), "argv requires nonempty strings");
    require(member["name"].is_string(), "member node names must be unique root-namespace names");
    const auto name = member["name"].get<std::string>();
    require(std::regex_match(name, node_name) && result.owned_names.insert(name).second, "member node names must be unique and cannot use reserved session names");
    const auto topics = member.value("ready_topics", Json::array());
    require(topics.is_array() && topics.size() <= 64, "ready_topics must list configured simulator input topics");
    auto &bindings = result.topics[name];
    for (const auto &topic : topics) {
      require(topic.is_string() && input_topics.count(topic.get<std::string>()), "ready_topics must list configured simulator input topics");
      bindings.insert(topic.get<std::string>());
    }
  }
  return result;
}
Json resolve_resources(Json data, const fs::path &base) {
  data.at("robot").at("urdf") = resolve(data.at("robot").at("urdf"), base).string();
  if (data.contains("lidars")) for (auto &lidar : data["lidars"])
    if (lidar.contains("pattern_file") && !lidar["pattern_file"].get<std::string>().empty()) lidar["pattern_file"] = resolve(lidar["pattern_file"], base).string();
  if (data.contains("entities")) for (auto &entity : data["entities"])
    for (const auto *role : {"geometry", "visual", "collision", "sensor_geometry"})
      if (entity.contains(role) && entity[role].is_object() && entity[role].value("type", "") == "mesh")
        entity[role]["uri"] = resolve(entity[role].at("uri"), base).string();
  return data;
}
std::vector<double> normalized_pose(const geometry_msgs::msg::Pose &pose) {
  std::vector<double> values{pose.position.x, pose.position.y, pose.position.z,
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w};
  for (double value : values) require(std::isfinite(value), "non-finite initial pose");
  // hypot avoids accepting finite components whose squared norm overflows.
  const double norm = std::hypot(std::hypot(values[3], values[4]), std::hypot(values[5], values[6]));
  require(std::isfinite(norm) && norm >= 1e-12, "zero or non-normalizable quaternion");
  for (std::size_t i = 3; i < values.size(); ++i) values[i] /= norm;
  return values;
}
std::vector<double> pose_rpy(const std::vector<double> &values) {
  const auto x = values.at(3), y = values.at(4), z = values.at(5), w = values.at(6);
  return {std::atan2(2 * (w*x+y*z), 1-2*(x*x+y*y)),
    std::asin(std::clamp(2*(w*y-z*x), -1.0, 1.0)), std::atan2(2*(w*z+x*y), 1-2*(y*y+z*z))};
}
Json signature(const std::string &run_id, bool set_pose, const geometry_msgs::msg::Pose &pose) {
  return {run_id, set_pose, {pose.position.x, pose.position.y, pose.position.z,
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w}};
}
void ResetCoordinator::reset(const litesim3d_interfaces::srv::Reset::Request &request,
                            litesim3d_interfaces::srv::Reset::Response &response, const ResetActions &actions) {
  try {
    const auto payload = signature(request.expected_run_id, request.set_initial_pose, request.initial_pose);
    if (cache_.count(request.request_id)) {
      const auto &cached = cache_.at(request.request_id);
      require(cached.payload == payload, "request_id reused with different payload"); response = cached.response; return;
    }
    require(!failed && !stopping && !transitioning, "session not ready");
    // ROS strings are UTF-8; preserve the former 128-character Python limit.
    const auto characters = std::count_if(request.request_id.begin(), request.request_id.end(),
      [](unsigned char byte) { return (byte & 0xc0) != 0x80; });
    require(!request.request_id.empty() && characters <= 128 && cache_.size() < 4096, "invalid request_id or full request cache");
    require(request.expected_run_id == run_id, "stale or missing expected_run_id");
    actions.check_participants();
    const auto pose = request.set_initial_pose ? actions.validate_pose(request) : std::vector<double>{};
    actions.remember_topics(); transitioning = true;
    try { actions.record("reset_begin", {{"request_id", request.request_id}}); actions.restart(pose); }
    catch (...) { failed = true; throw; }
    transitioning = false;
    response.success = true; response.message = "new epoch ready and paused"; response.run_id = run_id;
    cache_.emplace(request.request_id, Cached{payload, response});
  } catch (const std::exception &error) {
    response.success = false; response.message = error.what(); response.run_id = run_id;
    actions.record(failed ? "reset_failed" : "reset_rejected", {{"reason", error.what()}});
  }
}
}  // namespace litesim3d::session
