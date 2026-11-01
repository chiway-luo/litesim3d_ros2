#include <litesim3d_runtime/runtime.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cmath>
#include <iostream>

using namespace litesim3d::runtime;
namespace {
struct Options {
  std::string ns, frame, imu_topic = "imu/data_raw", imu_frame;
  fs::path output;
  bool stamped = false, drive = false, require_imu = false, require_motion = false;
  double linear = .2, angular = 0, duration = 5;
};
double number(const std::string &text) {
  std::size_t length; const double result = std::stod(text, &length);
  require(length == text.size() && std::isfinite(result), "speeds and duration must be finite"); return result;
}
}
int main(int argc, char **argv) {
  try {
    Options options; std::vector<const char *> ros_args{argv[0]};
    for (int i = 1; i < argc; ++i) {
      const std::string flag = argv[i];
      if (flag == "--ros-args") { for (; i < argc; ++i) ros_args.push_back(argv[i]); break; }
      if (flag == "--help" || flag == "-h") {
        std::cout << "usage: ros_client [--namespace NAME] [--frame FRAME] [--stamped] [--drive]\n"
          "  [--linear MPS] [--angular RADPS] [--duration SECONDS] [--output JSON]\n"
          "  [--imu-topic TOPIC] [--require-imu] [--expect-imu-frame FRAME] [--require-motion]\n"
          "  [--ros-args ROS_ARGUMENTS...]\n"; return 0;
      }
      if (flag == "--stamped") { options.stamped = true; continue; }
      if (flag == "--drive") { options.drive = true; continue; }
      if (flag == "--require-imu") { options.require_imu = true; continue; }
      if (flag == "--require-motion") { options.require_motion = true; continue; }
      require(i + 1 < argc, "missing value for " + flag); const std::string value = argv[++i];
      if (flag == "--namespace") options.ns = value;
      else if (flag == "--frame") options.frame = value;
      else if (flag == "--linear") options.linear = number(value);
      else if (flag == "--angular") options.angular = number(value);
      else if (flag == "--duration") options.duration = number(value);
      else if (flag == "--output") options.output = value;
      else if (flag == "--imu-topic") options.imu_topic = value;
      else if (flag == "--expect-imu-frame") options.imu_frame = value;
      else throw std::runtime_error("unknown argument; put ROS remaps after --ros-args: " + flag);
    }
    require(options.duration > 0, "duration must be positive");
    require(!options.require_motion || (options.drive && (options.linear != 0 || options.angular != 0)), "--require-motion requires --drive and nonzero velocity");
    options.ns.erase(0, options.ns.find_first_not_of('/')); options.ns.erase(options.ns.find_last_not_of('/') + 1);
    if (options.frame.empty()) options.frame = (options.ns.empty() ? "" : options.ns + "/") + "base_link";
    Json counts = {{"clock", 0}, {"points", 0}, {"imu", 0}, {"odom", 0}, {"commands", 0}, {"moving_commands", 0}};
    Json report = {{"namespace", options.ns}, {"cmd_mode", options.stamped ? "stamped" : "twist"}, {"drive", options.drive}, {"frames", Json::object()}};
    rclcpp::init(static_cast<int>(ros_args.size()), ros_args.data());
    rclcpp::NodeOptions node_options; node_options.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
    auto node = std::make_shared<rclcpp::Node>("manual_client", options.ns, node_options);
    auto increment = [&](const char *key) { counts[key] = counts[key].get<uint64_t>() + 1; };
    auto clock = node->create_subscription<rosgraph_msgs::msg::Clock>("/clock", rclcpp::SensorDataQoS(),
      [&](rosgraph_msgs::msg::Clock::ConstSharedPtr message) { increment("clock"); report["clock_ns"] = int64_t(message->clock.sec) * 1000000000LL + message->clock.nanosec; });
    auto points = node->create_subscription<sensor_msgs::msg::PointCloud2>("points_raw", rclcpp::SensorDataQoS(),
      [&](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
        increment("points"); report["frames"]["points"] = message->header.frame_id;
        auto fields = Json::array(); for (const auto &field : message->fields) fields.push_back(field.name);
        report["cloud"] = {{"height", message->height}, {"width", message->width}, {"point_step", message->point_step},
          {"fields", fields}, {"data_bytes", message->data.size()}};
      });
    auto imu = node->create_subscription<sensor_msgs::msg::Imu>(options.imu_topic, rclcpp::SensorDataQoS(),
      [&](sensor_msgs::msg::Imu::ConstSharedPtr message) {
        increment("imu"); report["frames"]["imu"] = message->header.frame_id;
        report["imu_acceleration"] = {message->linear_acceleration.x, message->linear_acceleration.y, message->linear_acceleration.z};
      });
    auto odom = node->create_subscription<nav_msgs::msg::Odometry>("odom", 10,
      [&](nav_msgs::msg::Odometry::ConstSharedPtr message) {
        increment("odom"); report["frames"]["odom"] = message->header.frame_id; report["frames"]["base"] = message->child_frame_id;
        const auto &p = message->pose.pose.position; const Json xyz = {p.x, p.y, p.z};
        if (!report.contains("first_odom_xyz")) report["first_odom_xyz"] = xyz; report["last_odom_xyz"] = xyz;
      });
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr stamped;
    if (options.drive) {
      if (options.stamped) stamped = node->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 1);
      else twist = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
    }
    auto connected = [&] { return counts["clock"] > 0 && counts["points"] > 0 && counts["odom"] > 0; };
    auto publish = [&](double linear, double angular) {
      geometry_msgs::msg::Twist velocity; velocity.linear.x = linear; velocity.angular.z = angular;
      if (stamped) { geometry_msgs::msg::TwistStamped message; message.header.stamp = node->now(); message.header.frame_id = options.frame; message.twist = velocity; stamped->publish(message); }
      else if (twist) twist->publish(velocity);
      increment("commands"); if (linear != 0 || angular != 0) increment("moving_commands");
    };
    const auto start = SteadyClock::now(); auto next_send = start;
    rclcpp::executors::SingleThreadedExecutor executor; executor.add_node(node);
    while (rclcpp::ok() && std::chrono::duration<double>(SteadyClock::now() - start).count() < options.duration) {
      executor.spin_once(std::chrono::milliseconds(50)); const auto now = SteadyClock::now();
      if (options.drive && now >= next_send && node->now().nanoseconds() > 0 && connected() &&
          (stamped ? stamped->get_subscription_count() : twist->get_subscription_count()) > 0) {
        publish(options.linear, options.angular); next_send = now + std::chrono::milliseconds(100);
      }
    }
    if (!rclcpp::ok()) report["interrupted"] = true;
    if (options.drive && rclcpp::ok()) for (int i = 0; i < 3; ++i) { publish(0, 0); executor.spin_once(std::chrono::milliseconds(50)); }
    report["counts"] = counts; report["wall_seconds"] = std::chrono::duration<double>(SteadyClock::now() - start).count();
    report["connected"] = connected(); Json checks = {{"connected", connected()}};
    if (options.require_imu || !options.imu_frame.empty()) checks["imu"] = counts["imu"] > 0;
    if (!options.imu_frame.empty()) checks["imu_frame"] = report["frames"].value("imu", "") == options.imu_frame;
    if (report.contains("first_odom_xyz")) {
      double squared = 0; for (int i = 0; i < 3; ++i) squared += std::pow(report["last_odom_xyz"][i].get<double>() - report["first_odom_xyz"][i].get<double>(), 2);
      report["odom_displacement_m"] = std::sqrt(squared);
    }
    if (options.require_motion) checks["motion"] = counts["moving_commands"] > 0 && report.value("odom_displacement_m", 0.0) >= .01;
    bool success = !report.value("interrupted", false); for (const auto &check : checks) success = success && check.get<bool>();
    report["checks"] = checks; report["success"] = success;
    executor.remove_node(node); if (rclcpp::ok()) rclcpp::shutdown();
    const auto text = report.dump(2) + "\n"; if (!options.output.empty()) atomic_write(options.output, text);
    std::cout << text; return success ? 0 : 1;
  } catch (const std::exception &error) { std::cerr << "ros_client: " << error.what() << '\n'; return 1; }
}
