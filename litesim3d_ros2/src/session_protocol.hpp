#pragma once
#include <litesim3d_runtime/runtime.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <litesim3d_interfaces/srv/reset.hpp>
#include <functional>
#include <map>
#include <set>

namespace litesim3d::session {
using namespace litesim3d::runtime;
struct Members {
  Json definitions;
  std::set<std::string> owned_names{"sim_session", "litesim3d", "robot_state_publisher"};
  std::map<std::string, std::set<std::string>> topics;
};
Members validate_members(const Json &definitions, const std::set<std::string> &input_topics);
Json resolve_resources(Json data, const fs::path &base);
std::vector<double> normalized_pose(const geometry_msgs::msg::Pose &pose);
std::vector<double> pose_rpy(const std::vector<double> &values);
Json signature(const std::string &run_id, bool set_pose, const geometry_msgs::msg::Pose &pose);
struct ResetActions {
  std::function<void()> check_participants, remember_topics;
  std::function<std::vector<double>(const litesim3d_interfaces::srv::Reset::Request &)> validate_pose;
  std::function<void(const std::vector<double> &)> restart;
  std::function<void(const std::string &, const Json &)> record;
};
// The coordinator owns epoch identity and the mutation protocol. ROS transport,
// graph inspection and process operations belong to the session adapter.
class ResetCoordinator {
 public:
  std::string run_id;
  bool transitioning = true, failed = false, stopping = false;
  void reset(const litesim3d_interfaces::srv::Reset::Request &request,
             litesim3d_interfaces::srv::Reset::Response &response, const ResetActions &actions);
 private:
  struct Cached { Json payload; litesim3d_interfaces::srv::Reset::Response response; };
  std::map<std::string, Cached> cache_;
};
}  // namespace litesim3d::session
