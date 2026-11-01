#include "session_protocol.hpp"
#include <ament_index_cpp/get_package_prefix.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <litesim3d_interfaces/srv/reset.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include <atomic>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <thread>

namespace litesim3d::session {
namespace {
struct Options {
  fs::path config, output, members;
  std::set<std::string> observers;
  double timeout = 15;
  bool sensor_reliable = false;
};
std::string topic_name(std::string name) {
  name.erase(0, name.find_first_not_of('/')); return "/" + name;
}
}

class Session final : public rclcpp::Node {
 public:
  using Reset = litesim3d_interfaces::srv::Reset;
  using GetParameters = rcl_interfaces::srv::GetParameters;
  explicit Session(Options options)
      : Node("sim_session"), options_(std::move(options)), config_(canonical_path(options_.config)),
        directory_(canonical_path(options_.output)), data_(read_yaml(config_)) {
    require(data_.is_object() && data_.contains("robot"), "session requires a single-robot config");
    require(!fs::exists(directory_) || (fs::is_directory(directory_) && fs::is_empty(directory_)), "session output directory must be empty");
    for (const auto &[group, fallback] : std::map<std::string, std::string>{{"lidars", "points_raw"}, {"imus", "imu/data_raw"}})
      for (const auto &item : data_.value(group, Json::array())) input_topics_.insert(topic_name(item.value("topic", fallback)));
    members_ = validate_members(options_.members.empty() ? Json::array() : Json::parse(read(options_.members)), input_topics_);
    fs::create_directories(directory_);
    status_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    service_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    probe_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    // Keep graph entities alive until the executor has stopped using its wait set.
    // The same clients can probe each newly spawned epoch of a registered member.
    for (const auto &member : members_.definitions) {
      const auto name = member.at("name").get<std::string>();
      probes_[name] = create_client<GetParameters>("/" + name + "/get_parameters", rmw_qos_profile_services_default, probe_group_);
    }
    rclcpp::SubscriptionOptions subscription_options; subscription_options.callback_group = status_group_;
    subscription_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>("/sim/status", 10,
      [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr message) {
        if (message->status.empty()) return;
        std::lock_guard<std::mutex> lock(status_mutex_); status_.clear();
        for (const auto &entry : message->status.front().values) status_[entry.key] = entry.value;
        ++status_sequence_;
      }, subscription_options);
    service_ = create_service<Reset>("/sim/reset", [this](const Reset::Request::SharedPtr request, Reset::Response::SharedPtr response) {
      reset(*request, *response);
    }, rmw_qos_profile_services_default, service_group_);
    record("created", {{"members", members_.definitions}, {"observers", options_.observers}});
  }
  void start() { std::lock_guard<std::timed_mutex> lock(mutation_); start_epoch(); }
  bool monitor() {
    std::lock_guard<std::timed_mutex> lock(mutation_);
    if (!coordinator_.transitioning) for (auto &child : children_) if (const auto code = child.process.poll()) {
      record("required_process_exit", {{"name", child.name}, {"returncode", *code}}); coordinator_.failed = true;
    }
    return coordinator_.failed;
  }
  int finish(int code) {
    std::lock_guard<std::timed_mutex> lock(mutation_); coordinator_.stopping = true;
    try { stop_owned(); }
    catch (const std::exception &error) { record("cleanup_failure", {{"reason", error.what()}}); code = 1; }
    code = std::max(code, int(coordinator_.failed)); record("exit", {{"returncode", code}}); return code;
  }
  void failure(const std::string &reason) {
    std::lock_guard<std::timed_mutex> lock(mutation_); record("failure", {{"reason", reason}});
  }

 private:
  struct Child { std::string name; Process process; };
  Options options_;
  fs::path config_, directory_;
  Json data_;
  Members members_;
  std::set<std::string> input_topics_{"/clock", "/odom", "/joint_states"};
  std::vector<Child> children_;
  std::timed_mutex mutation_;
  std::mutex status_mutex_;
  std::map<std::string, std::string> status_;
  uint64_t status_sequence_ = 0;
  unsigned epoch_ = 0;
  ResetCoordinator coordinator_;
  rclcpp::CallbackGroup::SharedPtr status_group_, service_group_, probe_group_;
  std::map<std::string, rclcpp::Client<GetParameters>::SharedPtr> probes_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr subscription_;
  rclcpp::Service<Reset>::SharedPtr service_;

  void record(const std::string &event, Json values = Json::object()) {
    Json row = {{"wall_time_ns", std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count()},
      {"event", event}, {"epoch", epoch_}, {"run_id", coordinator_.run_id}};
    row.update(values); std::ofstream out(directory_ / "session.jsonl", std::ios::app);
    out << row.dump() << '\n'; out.close(); require(bool(out), "cannot write session log");
  }
  std::pair<std::map<std::string, std::string>, uint64_t> status() {
    std::lock_guard<std::mutex> lock(status_mutex_); return {status_, status_sequence_};
  }
  void spawn(const std::string &name, const std::vector<std::string> &command, const fs::path &directory) {
    ProcessOptions options; options.output = directory / (name + ".log"); options.error = options.output;
    options.environment = {{"LWS_RUN_ID", coordinator_.run_id}, {"LWS_RUN_DIR", directory.string()}};
    children_.push_back({name, Process(command, options)});
    record("spawn", {{"name", name}, {"pid", children_.back().process.pid()}, {"argv", command}});
  }
  void wait_startup(const std::function<bool()> &predicate, SteadyClock::time_point deadline, const std::string &detail) {
    while (true) {
      require(!interruption_requested() && rclcpp::ok(), "session startup interrupted");
      for (auto &child : children_) require(!child.process.poll(), "required process exited while waiting for " + detail);
      if (predicate()) return;
      require(SteadyClock::now() < deadline, "consumer readiness timed out: " + detail);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }
  void remember_topics() {
    for (const auto &topic : input_topics_) for (const auto &endpoint : get_subscriptions_info_by_topic(topic))
      if (endpoint.node_namespace() == "/" && members_.topics.count(endpoint.node_name())) members_.topics[endpoint.node_name()].insert(topic);
  }
  bool graph_complete() {
    for (const auto &member : members_.definitions) {
      const auto name = member.at("name").get<std::string>();
      const auto services = get_service_names_and_types_by_node(name, "/");
      if (!services.count("/" + name + "/get_parameters")) return false;
    }
    auto topics = input_topics_; topics.insert("/tf"); topics.insert("/tf_static");
    for (const auto &topic : topics) for (const auto &endpoint : get_subscriptions_info_by_topic(topic))
      if (endpoint.node_name().empty() || endpoint.node_name() == "_NODE_NAME_UNKNOWN_" || endpoint.node_namespace().empty()
          || endpoint.node_namespace().front() != '/') return false;
    return true;
  }
  bool compatible(const rclcpp::TopicEndpointInfo &publisher, const rclcpp::TopicEndpointInfo &subscriber) {
    if (publisher.topic_type() != subscriber.topic_type()) return false;
    rmw_qos_compatibility_type_t compatibility; char reason[1024] = {};
    const auto result = rmw_qos_profile_check_compatible(publisher.qos_profile().get_rmw_qos_profile(),
      subscriber.qos_profile().get_rmw_qos_profile(), &compatibility, reason, sizeof(reason));
    require(result == RMW_RET_OK, "failed to check consumer QoS"); return compatibility != RMW_QOS_COMPATIBILITY_ERROR;
  }
  void wait_members(SteadyClock::time_point deadline) {
    for (const auto &member : members_.definitions) {
      const auto name = member.at("name").get<std::string>();
      const auto &client = probes_.at(name);
      wait_startup([&] { return client->service_is_ready(); }, deadline, name + " parameter service");
      unsigned attempt = 0;
      while (true) {
        ++attempt; auto future = client->async_send_request(std::make_shared<GetParameters::Request>());
        const auto retry_at = std::min(deadline, SteadyClock::now() + std::chrono::seconds(1));
        try {
          wait_startup([&] { return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready || SteadyClock::now() >= retry_at; },
            deadline, name + " executor response");
          if (future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) { future.get(); break; }
        } catch (...) { client->remove_pending_request(future); throw; }
        client->remove_pending_request(future);
        require(SteadyClock::now() < deadline, "consumer readiness timed out: " + name + " executor response");
        record("member_executor_probe_retry", {{"name", name}, {"attempt", attempt}});
      }
      record("member_executor_ready", {{"name", name}});
    }
    if (!members_.definitions.empty()) {
      const auto sequence = status().second;
      wait_startup([&] { auto current = status(); return current.second > sequence && current.first["run_id"] == coordinator_.run_id && graph_complete(); },
        deadline, "member graph ownership and fresh publisher status");
    }
    remember_topics();
    for (const auto &entry : members_.topics) {
      const auto &name = entry.first;
      const auto &topics = entry.second;
      for (const auto &topic : topics) wait_startup([&] {
        std::vector<rclcpp::TopicEndpointInfo> publishers, owned;
        for (const auto &endpoint : get_publishers_info_by_topic(topic))
          if (endpoint.node_name() == "litesim3d" && endpoint.node_namespace() == "/") publishers.push_back(endpoint);
        const auto subscribers = get_subscriptions_info_by_topic(topic);
        for (const auto &endpoint : subscribers) if (endpoint.node_name() == name && endpoint.node_namespace() == "/") owned.push_back(endpoint);
        if (publishers.size() != 1 || owned.empty()) return false;
        for (const auto &subscriber : owned) require(compatible(publishers[0], subscriber), "incompatible consumer subscription: " + name + " " + topic);
        std::size_t expected = 0; for (const auto &subscriber : subscribers) if (compatible(publishers[0], subscriber)) ++expected;
        auto current = status().first;
        const auto counts = Json::parse(current.count("subscription_matches") ? current["subscription_matches"] : "{}");
        return current["run_id"] == coordinator_.run_id && counts.value(topic, std::size_t(0)) >= expected;
      }, deadline, name + " subscription " + topic);
      record("member_ready", {{"name", name}, {"topics", topics}});
    }
  }
  void start_epoch(const std::vector<double> &pose = {}) {
    ++epoch_; std::ostringstream name; name << "epoch-" << std::setw(4) << std::setfill('0') << epoch_;
    const auto directory = directory_ / name.str(); require(fs::create_directory(directory), "epoch directory already exists");
    const auto old_run = coordinator_.run_id;
    { std::lock_guard<std::mutex> lock(status_mutex_); status_.clear(); }
    const auto urdf = resolve(data_.at("robot").at("urdf"), config_.parent_path());
    const auto rsp_params = directory / "rsp.yaml";
    write(rsp_params, dump_yaml({{"/**", {{"ros__parameters", {{"robot_description", read(urdf)}, {"use_sim_time", true}, {"publish_frequency", 200.0}}}}}}));
    spawn("robot_state_publisher", {executable("robot_state_publisher", "robot_state_publisher").string(), "--ros-args", "--params-file", rsp_params.string()}, directory);
    Json parameters = {{"config", config_.string()}, {"use_sim_time", true}, {"start_paused", true},
      {"sensor_reliable", options_.sensor_reliable}, {"manifest_path", (directory / "manifest.json").string()}};
    if (!pose.empty()) parameters["initial_pose_override"] = pose;
    const auto sim_params = directory / "sim.yaml"; write(sim_params, dump_yaml({{"/**", {{"ros__parameters", parameters}}}}));
    spawn("litesim3d", {executable("litesim3d_ros2", "sim_node").string(), "--ros-args", "--params-file", sim_params.string()}, directory);
    const auto deadline = SteadyClock::now() + std::chrono::duration_cast<SteadyClock::duration>(std::chrono::duration<double>(options_.timeout));
    wait_startup([&] {
      auto current = status().first;
      require(current["state"] != "fault", "simulator faulted during epoch startup");
      if (current["ready"] == "true" && !current["run_id"].empty() && current["run_id"] != old_run) {
        coordinator_.run_id = current["run_id"]; return true;
      }
      return false;
    }, deadline, "new epoch readiness");
    for (const auto &member : members_.definitions) spawn(member.at("name"), member.at("argv").get<std::vector<std::string>>(), directory);
    wait_members(deadline); coordinator_.transitioning = false;
    record("ready", {{"time_ns", status().first["time_ns"]}});
  }
  void stop_owned() {
    auto errors = Json::array();
    for (auto it = children_.rbegin(); it != children_.rend(); ++it) {
      bool forced = false; const int pid = it->process.pid();
      try {
        const int code = it->process.stop(5, &forced);
        record("stop", {{"name", it->name}, {"pid", pid}, {"returncode", code}, {"forced", forced}});
        if (forced || (code != 0 && code != -SIGINT)) errors.push_back({it->name, code, forced});
      } catch (const std::exception &error) { errors.push_back({it->name, error.what()}); }
    }
    children_.clear(); require(errors.empty(), "unclean process exit: " + errors.dump());
  }
  void check_participants() {
    auto topics = input_topics_; topics.insert("/tf"); topics.insert("/tf_static");
    std::set<std::string> unknown;
    for (const auto &topic : topics) for (const auto &endpoint : get_subscriptions_info_by_topic(topic))
      if (endpoint.node_namespace() != "/" || (!members_.owned_names.count(endpoint.node_name()) && !options_.observers.count(endpoint.node_name())))
        unknown.insert(endpoint.node_namespace() + "/" + endpoint.node_name());
    for (const auto &endpoint : get_publishers_info_by_topic("/cmd_vel"))
      if (endpoint.node_namespace() != "/" || !members_.owned_names.count(endpoint.node_name())) unknown.insert(endpoint.node_name());
    require(unknown.empty(), "uncoordinated participants: " + Json(unknown).dump());
  }
  std::vector<double> validate_pose(const Reset::Request &request) {
    auto values = normalized_pose(request.initial_pose); auto data = read_yaml(config_);
    data["robot"]["initial_pose"] = {{"xyz", {values[0], values[1], values[2]}}, {"rpy", pose_rpy(values)}};
    const auto candidate = directory_ / "reset-candidate.yaml";
    write(candidate, dump_yaml(resolve_resources(data, config_.parent_path())));
    const auto headless = fs::path(ament_index_cpp::get_package_prefix("litesim3d_core")) / "bin" / "sim_headless";
    const auto check = run({headless.string(), candidate.string(), "1", (directory_ / "preflight.json").string()}, options_.timeout);
    write(directory_ / "preflight.log", check.out + check.err);
    require(check.code == 0, "initial pose failed transactional preflight; previous epoch retained"); return values;
  }
  void reset(const Reset::Request &request, Reset::Response &response) {
    std::unique_lock<std::timed_mutex> lock(mutation_, std::defer_lock);
    if (!lock.try_lock_for(std::chrono::milliseconds(500))) { response.message = "reset already in progress"; return; }
    ResetActions actions;
    actions.check_participants = [this] { check_participants(); };
    actions.validate_pose = [this](const auto &value) { return validate_pose(value); };
    actions.remember_topics = [this] { remember_topics(); };
    actions.restart = [this](const auto &pose) { stop_owned(); start_epoch(pose); };
    actions.record = [this](const auto &event, const auto &values) { record(event, values); };
    coordinator_.reset(request, response, actions);
  }
};
}  // namespace litesim3d::session

int main(int argc, char **argv) {
  using namespace litesim3d::session;
  try {
    Options options;
    for (int i = 1; i < argc; ++i) {
      const std::string flag = argv[i];
      if (flag == "--help" || flag == "-h") {
        std::cout << "usage: sim_session --config YAML --output EMPTY_DIRECTORY [--members JSON]\n"
          "  [--observer NODE]... [--timeout SECONDS] [--sensor-reliable]\n"; return 0;
      }
      if (flag == "--sensor-reliable") { options.sensor_reliable = true; continue; }
      require(i + 1 < argc, "missing value for " + flag); const std::string value = argv[++i];
      if (flag == "--config") options.config = value;
      else if (flag == "--output") options.output = value;
      else if (flag == "--members") options.members = value;
      else if (flag == "--observer") options.observers.insert(value);
      else if (flag == "--timeout") { std::size_t length; options.timeout = std::stod(value, &length); require(length == value.size(), "invalid timeout"); }
      else throw std::runtime_error("unknown argument: " + flag);
    }
    require(!options.config.empty() && !options.output.empty(), "--config and --output are required");
    require(std::isfinite(options.timeout) && options.timeout >= 1 && options.timeout <= 120, "timeout must be in [1,120] seconds");
    InterruptGuard interrupts;
    rclcpp::init(0, nullptr, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    auto node = std::make_shared<Session>(options);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3); executor.add_node(node);
    std::atomic<bool> executor_failed{false};
    std::thread worker([&] { try { executor.spin(); } catch (const std::exception &error) {
      std::cerr << "session executor: " << error.what() << '\n'; executor_failed = true;
    } });
    int code = 0;
    try {
      node->start();
      while (rclcpp::ok() && !interruption_requested() && !executor_failed) {
        if (node->monitor()) { code = 1; break; }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
      if (executor_failed) code = 1;
    } catch (const std::exception &error) {
      std::cerr << "session: " << error.what() << '\n';
      if (!interruption_requested()) code = 1;
      try { node->failure(error.what()); }
      catch (const std::exception &log_error) { std::cerr << "session log: " << log_error.what() << '\n'; code = 1; }
    }
    try { code = node->finish(code); }
    catch (const std::exception &error) { std::cerr << "session cleanup: " << error.what() << '\n'; code = 1; }
    executor.cancel(); worker.join(); executor.remove_node(node); node.reset(); rclcpp::shutdown(); return code;
  } catch (const std::exception &error) { std::cerr << "sim_session: " << error.what() << '\n'; return 1; }
}
