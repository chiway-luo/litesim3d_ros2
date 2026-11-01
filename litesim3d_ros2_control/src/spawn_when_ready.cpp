#include <ament_index_cpp/get_package_prefix.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <map>
#include <optional>
#include <unistd.h>
#include "readiness.hpp"

namespace {
void wait_for_simulation(double timeout, const std::string &status_topic, const std::string &clock_topic) {
  auto context = std::make_shared<rclcpp::Context>(); context->init(0, nullptr);
  rclcpp::NodeOptions options; options.context(context).use_global_arguments(false);
  auto node = std::make_shared<rclcpp::Node>("litesim_control_startup_" + std::to_string(getpid()), options);
  rclcpp::ExecutorOptions executor_options; executor_options.context = context;
  rclcpp::executors::SingleThreadedExecutor executor(executor_options); executor.add_node(node);
  litesim3d::control::Readiness state;
  auto status = node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(status_topic, 10,
    [&](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr message) {
      std::map<std::string, std::string> values;
      for (const auto &item : message->status) for (const auto &entry : item.values) values[entry.key] = entry.value;
      state.status(values);
    });
  auto clock = node->create_subscription<rosgraph_msgs::msg::Clock>(clock_topic, rclcpp::SensorDataQoS(),
    [&](rosgraph_msgs::msg::Clock::ConstSharedPtr message) {
      const auto stamp = int64_t(message->clock.sec) * 1000000000LL + message->clock.nanosec;
      state.clock(stamp);
    });
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout);
  try {
    while (context->is_valid() && std::chrono::steady_clock::now() < deadline) {
      executor.spin_once(std::chrono::milliseconds(50));
      if (state.fault()) throw std::runtime_error("simulator faulted before controller startup");
      if (state.ready()) { context->shutdown("ready"); return; }
    }
    throw std::runtime_error("simulator readiness and advancing clock not observed within " + std::to_string(timeout) + "s");
  } catch (...) { context->shutdown("startup failed"); throw; }
}
}
int main(int argc, char **argv) {
  try {
    double timeout = 60; std::string status = "/sim/status", clock = "/clock", target = "spawner";
    std::vector<std::string> forwarded;
    for (int i = 1; i < argc; ++i) {
      const std::string flag = argv[i];
      if (flag == "--") { for (++i; i < argc; ++i) forwarded.emplace_back(argv[i]); break; }
      if (!flag.empty() && flag.front() != '-') { for (; i < argc; ++i) forwarded.emplace_back(argv[i]); break; }
      if (flag == "--help" || flag == "-h") {
        std::cout << "usage: spawn_when_ready [--timeout SECONDS] [--status-topic TOPIC] [--clock-topic TOPIC]\n"
          "  [--executable spawner|ros2_control_node] -- TARGET_ARGUMENTS...\n"; return 0;
      }
      if (i + 1 >= argc) throw std::runtime_error("missing value for " + flag);
      const std::string value = argv[++i];
      if (flag == "--timeout") timeout = litesim3d::control::startup_timeout(value);
      else if (flag == "--status-topic") status = value;
      else if (flag == "--clock-topic") clock = value;
      else if (flag == "--executable") target = value;
      else throw std::runtime_error("unknown argument: " + flag);
    }
    if (target != "spawner" && target != "ros2_control_node") throw std::runtime_error("unknown executable: " + target);
    if (forwarded.empty()) throw std::runtime_error("provide controller_manager arguments after --");
    wait_for_simulation(timeout, status, clock);
    const std::string package = target == "spawner" ? "controller_manager" : "litesim3d_ros2_control";
    const auto tool = (std::filesystem::path(ament_index_cpp::get_package_prefix(package)) / "lib" / package / target).string();
    forwarded.insert(forwarded.begin(), tool);
    std::vector<char *> arguments; for (auto &value : forwarded) arguments.push_back(value.data()); arguments.push_back(nullptr);
    execv(tool.c_str(), arguments.data());
    throw std::runtime_error("cannot execute " + tool + ": " + std::strerror(errno));
  } catch (const std::exception &error) { std::cerr << "controller startup: " << error.what() << '\n'; return 1; }
}
