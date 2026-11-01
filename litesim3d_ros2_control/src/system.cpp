#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <litesim3d_interfaces/msg/joint_command.hpp>
#include <litesim3d_interfaces/msg/joint_feedback.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <set>
#include <map>
#include <mutex>
#include <thread>

namespace litesim3d_ros2_control {
using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
class LiteSimSystem : public hardware_interface::SystemInterface {
  friend struct SystemUnitAccess;
  using Command = litesim3d_interfaces::msg::JointCommand;
  using Feedback = litesim3d_interfaces::msg::JointFeedback;
  using Clock = std::chrono::steady_clock;
public:
  ~LiteSimSystem() override {
    stop_transport_.store(true);
    executor_.cancel();
    if (transport_thread_.joinable()) transport_thread_.join();
  }
  CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override {
    if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) return CallbackReturn::ERROR;
    try {
      if (info_.joints.empty() && info_.gpios.empty()) throw std::runtime_error("no joints or body GPIO configured");
      if (!info_.sensors.empty() || info_.gpios.size() > 1)
        throw std::runtime_error("LiteSimSystem supports joints and one body GPIO; sensors use their typed ROS topics");
      if (!info_.gpios.empty()) {
        const auto &body = info_.gpios.front();
        std::set<std::string> states, commands;
        for (const auto &i : body.state_interfaces)
          if (std::find(body_names_.begin(), body_names_.end(), i.name) == body_names_.end() || !states.insert(i.name).second)
            throw std::runtime_error("unknown/duplicate body state interface: " + i.name);
        for (const auto &i : body.command_interfaces)
          if (std::find(body_names_.begin(), body_names_.begin() + 4, i.name) == body_names_.begin() + 4 || !commands.insert(i.name).second)
            throw std::runtime_error("body commands support linear.xyz.velocity and angular.z.velocity only");
        if (body.name.empty() || (states.empty() && commands.empty())) throw std::runtime_error("empty body GPIO");
      }
      auto parameter = [&](const std::string &name, const std::string &fallback) {
        const auto it = info_.hardware_parameters.find(name);
        return it == info_.hardware_parameters.end() ? fallback : it->second;
      };
      timeout_ = std::stod(parameter("feedback_timeout", "2.0"));
      activation_timeout_ = std::stod(parameter("activation_timeout", "10.0"));
      if (!std::isfinite(timeout_) || timeout_ <= 0 || !std::isfinite(activation_timeout_) || activation_timeout_ <= 0)
        throw std::runtime_error("timeouts must be finite and positive");
      static std::atomic<unsigned> next_id{0};
      // Private transport node avoids inheriting the manager's __node/__ns remaps.
      // Topic endpoints and namespace are explicitly configurable in hardware parameters.
      node_ = std::make_shared<rclcpp::Node>("litesim_hardware_" + std::to_string(next_id++),
          parameter("namespace", "/"), rclcpp::NodeOptions().use_global_arguments(false));
      const size_t count = info_.joints.size();
      position_.assign(count, 0); velocity_.assign(count, 0);
      feedback_position_.assign(count, 0); feedback_velocity_.assign(count, 0);
      position_command_.assign(count, 0); velocity_command_.assign(count, 0);
      mode_.assign(count, Command::HOLD);
      reset_on_write_.assign(count, false);
      std::set<std::string> names;
      for (const auto &joint : info_.joints) {
        if (!names.insert(joint.name).second) throw std::runtime_error("duplicate joint: " + joint.name);
        std::set<std::string> states, commands;
        for (const auto &i : joint.state_interfaces) states.insert(i.name);
        if (states != std::set<std::string>{"position", "velocity"} || joint.state_interfaces.size() != 2)
          throw std::runtime_error("each joint requires position and velocity state interfaces: " + joint.name);
        for (const auto &i : joint.command_interfaces)
          if ((i.name != "position" && i.name != "velocity") || !commands.insert(i.name).second)
            throw std::runtime_error("only distinct position/velocity command interfaces are supported: " + joint.name);
      }
      publisher_ = node_->create_publisher<Command>(parameter("command_topic", "sim/joint_commands"), rclcpp::QoS(1).reliable());
      subscription_ = node_->create_subscription<Feedback>(parameter("state_topic", "sim/joint_feedback"),
          rclcpp::QoS(1).reliable(), [this](Feedback::ConstSharedPtr msg) { receive(*msg); });
      watchdog_ = node_->create_wall_timer(std::chrono::milliseconds(20), [this] {
        std::lock_guard<std::mutex> lock(mutex_);
        if (active_ && received_ && elapsed(received_at_) > timeout_) fault_ = true;
      });
      executor_.add_node(node_);
      transport_thread_ = std::thread([this] {
        // cancel() before spin() starts is not sticky: spin resets its flag.
        // Keep our lifetime stop flag, and bound a wait if cancellation races
        // with entry to spin_once. No global ROS shutdown is needed to join.
        while (!stop_transport_.load() && rclcpp::ok())
          executor_.spin_once(std::chrono::milliseconds(20));
      });
    } catch (const std::exception &e) {
      RCLCPP_ERROR(rclcpp::get_logger("litesim_hardware"), "%s", e.what());
      return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
  }
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
    std::vector<hardware_interface::StateInterface> out;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
      out.emplace_back(info_.joints[i].name, "position", &position_[i]);
      out.emplace_back(info_.joints[i].name, "velocity", &velocity_[i]);
    }
    if (!info_.gpios.empty()) for (const auto &interface : info_.gpios.front().state_interfaces) {
      const auto index = std::find(body_names_.begin(), body_names_.end(), interface.name) - body_names_.begin();
      out.emplace_back(info_.gpios.front().name, interface.name, &body_state_[index]);
    }
    return out;
  }
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
    std::vector<hardware_interface::CommandInterface> out;
    for (size_t i = 0; i < info_.joints.size(); ++i)
      for (const auto &interface : info_.joints[i].command_interfaces)
        out.emplace_back(info_.joints[i].name, interface.name,
            interface.name == "position" ? &position_command_[i] : &velocity_command_[i]);
    if (!info_.gpios.empty()) for (const auto &interface : info_.gpios.front().command_interfaces) {
      const auto index = std::find(body_names_.begin(), body_names_.end(), interface.name) - body_names_.begin();
      out.emplace_back(info_.gpios.front().name, interface.name, &body_command_[index]);
    }
    return out;
  }
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
    std::lock_guard<std::mutex> lock(mutex_);
    active_ = received_ = fault_ = false; run_id_.clear();
    body_mode_.fill(false); body_command_.fill(0); body_reset_.fill(false);
    std::fill(mode_.begin(), mode_.end(), Command::HOLD);
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
    const auto start = Clock::now();
    while (rclcpp::ok() && elapsed(start) < activation_timeout_) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (healthy() && commandSubscriberCount() == 1) {
          std::copy(feedback_position_.begin(), feedback_position_.end(), position_.begin());
          std::copy(feedback_velocity_.begin(), feedback_velocity_.end(), velocity_.begin());
          std::copy(position_.begin(), position_.end(), position_command_.begin());
          std::fill(velocity_command_.begin(), velocity_command_.end(), 0);
          body_state_ = feedback_body_; body_command_.fill(0);
          active_ = true;
          return CallbackReturn::SUCCESS;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    RCLCPP_ERROR(node_->get_logger(), "No complete, fresh simulator feedback/command endpoint; check mode, topic parameters and joint names");
    return CallbackReturn::ERROR;
  }
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
    std::lock_guard<std::mutex> lock(mutex_);
    hold(); active_ = false; return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_error(const rclcpp_lifecycle::State &) override {
    std::lock_guard<std::mutex> lock(mutex_);
    hold(); active_ = false; return CallbackReturn::SUCCESS;
  }
  return_type prepare_command_mode_switch(const std::vector<std::string> &start,
                                          const std::vector<std::string> &stop) override {
    std::lock_guard<std::mutex> lock(mutex_);
    auto candidate = mode_;
    auto body_candidate = body_mode_;
    return switchModes(start, stop, candidate) && switchBody(start, stop, body_candidate) ? return_type::OK : return_type::ERROR;
  }
  return_type perform_command_mode_switch(const std::vector<std::string> &start,
                                          const std::vector<std::string> &stop) override {
    std::lock_guard<std::mutex> lock(mutex_);
    auto candidate = mode_;
    auto body_candidate = body_mode_;
    if (!switchModes(start, stop, candidate) || !switchBody(start, stop, body_candidate)) return return_type::ERROR;
    if (!info_.gpios.empty()) for (size_t i = 0; i < 4; ++i) {
      const auto key = info_.gpios.front().name + "/" + body_names_[i];
      if (std::find(start.begin(), start.end(), key) != start.end() || std::find(stop.begin(), stop.end(), key) != stop.end()) {
        body_command_[i] = 0; body_reset_[i] = true;
      }
    }
    body_mode_ = body_candidate;
    for (size_t i = 0; i < mode_.size(); ++i) {
      bool ownership_changed = mode_[i] != candidate[i];
      for (const auto &interface : info_.joints[i].command_interfaces) {
        const auto key = info_.joints[i].name + "/" + interface.name;
        ownership_changed |= std::find(stop.begin(), stop.end(), key) != stop.end();
        ownership_changed |= std::find(start.begin(), start.end(), key) != start.end();
      }
      // Also reset when two controllers exchange the same velocity interface.
      if (ownership_changed) {
        position_command_[i] = position_[i]; velocity_command_[i] = 0;
        reset_on_write_[i] = true;
      }
    }
    mode_ = std::move(candidate);
    return return_type::OK;
  }
  return_type read(const rclcpp::Time &, const rclcpp::Duration &) override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (received_) {
      body_state_ = feedback_body_;
      std::copy(feedback_position_.begin(), feedback_position_.end(), position_.begin());
      std::copy(feedback_velocity_.begin(), feedback_velocity_.end(), velocity_.begin());
    }
    if (!active_ || healthy()) return return_type::OK;
    RCLCPP_ERROR(node_->get_logger(), "Simulator feedback lost, invalid, faulted or session changed; reconfigure hardware before resuming");
    hold(); fault_ = true;
    return return_type::ERROR;
  }
  return_type write(const rclcpp::Time &, const rclcpp::Duration &) override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_) return return_type::OK;
    if (!healthy()) { hold(); return return_type::ERROR; }
    Command msg; msg.run_id = run_id_; msg.header.stamp = stamp_;
    for (size_t i = 0; i < mode_.size(); ++i) {
      // Humble performs the hardware switch before controller lifecycle calls.
      // Ackermann's on_deactivate writes NaN after perform_command_mode_switch.
      // Hold once *after* that handoff, then accept the new owner's next update.
      if (reset_on_write_[i]) {
        position_command_[i] = position_[i]; velocity_command_[i] = 0;
        reset_on_write_[i] = false;
      }
      const double value = mode_[i] == Command::POSITION ? position_command_[i] :
                          mode_[i] == Command::VELOCITY ? velocity_command_[i] : 0;
      if (!std::isfinite(value)) {
        RCLCPP_ERROR(node_->get_logger(), "Non-finite command for joint %s", info_.joints[i].name.c_str());
        hold(); fault_ = true; return return_type::ERROR;
      }
      // Read-only/mimic joints are returned as feedback, never sent as commands.
      if (info_.joints[i].command_interfaces.empty()) continue;
      msg.name.push_back(info_.joints[i].name); msg.mode.push_back(mode_[i]); msg.command.push_back(value);
    }
    std::array<double, 4> body{};
    for (size_t i = 0; i < 4; ++i) {
      if (body_reset_[i]) { body_command_[i] = 0; body_reset_[i] = false; }
      if (!body_mode_[i]) continue;
      if (!std::isfinite(body_command_[i])) { hold(); fault_ = true; return return_type::ERROR; }
      msg.body_velocity_control = true;
      body[i] = body_command_[i];
    }
    msg.body_velocity.linear.x = body[0]; msg.body_velocity.linear.y = body[1];
    msg.body_velocity.linear.z = body[2]; msg.body_velocity.angular.z = body[3];
    publishCommand(msg);
    return return_type::OK;
  }
protected:
  // Keep ROS endpoint discovery and publication at a narrow adapter boundary.
  // Deterministic units can supply endpoint counts and inspect commands without
  // connecting a simulator or depending on asynchronous middleware discovery.
  virtual size_t feedbackPublisherCount() const { return subscription_->get_publisher_count(); }
  virtual size_t commandSubscriberCount() const { return publisher_->get_subscription_count(); }
  virtual void publishCommand(const Command &message) { publisher_->publish(message); }
private:
  static double elapsed(Clock::time_point since) { return std::chrono::duration<double>(Clock::now() - since).count(); }
  bool healthy() const {
    return received_ && !fault_ && elapsed(received_at_) <= timeout_ && feedbackPublisherCount() == 1;
  }
  void receive(const Feedback &msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    // Latch a gap before updating its timestamp: a late queued sample must
    // not erase a timeout just because it is dispatched before the wall timer.
    if (active_ && received_ && elapsed(received_at_) > timeout_) fault_ = true;
    const auto &state = msg.state;
    if (msg.run_id.empty() || !msg.operational || state.header.stamp.sec < 0 ||
        state.header.stamp.nanosec >= 1000000000 || state.name.size() != state.position.size() ||
        state.name.size() != state.velocity.size() ||
        (active_ && (run_id_ != msg.run_id || rclcpp::Time(state.header.stamp) < rclcpp::Time(stamp_)))) {
      fault_ = true; return;
    }
    std::map<std::string, size_t> indexes;
    for (size_t i = 0; i < state.name.size(); ++i)
      if (!indexes.emplace(state.name[i], i).second || !std::isfinite(state.position[i]) || !std::isfinite(state.velocity[i])) {
        fault_ = true; return;
      }
    for (const auto &joint : info_.joints) if (!indexes.count(joint.name)) { fault_ = true; return; }
    for (size_t i = 0; i < info_.joints.size(); ++i) {
      const size_t index = indexes.at(info_.joints[i].name);
      feedback_position_[i] = state.position[index]; feedback_velocity_[i] = state.velocity[index];
    }
    if (!info_.gpios.empty()) {
      const auto &p = msg.body_pose.position; const auto &q = msg.body_pose.orientation;
      const auto &v = msg.body_velocity;
      feedback_body_ = {v.linear.x, v.linear.y, v.linear.z, v.angular.z,
          p.x, p.y, p.z, q.x, q.y, q.z, q.w};
      if ((!info_.gpios.front().command_interfaces.empty() && !msg.body_command_supported) ||
          std::any_of(feedback_body_.begin(), feedback_body_.end(), [](double v) { return !std::isfinite(v); }) ||
          std::abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1) > 1e-5) {
        fault_ = true; return;
      }
    }
    run_id_ = msg.run_id; stamp_ = state.header.stamp;
    received_ = true; received_at_ = Clock::now();
  }
  bool switchModes(const std::vector<std::string> &start, const std::vector<std::string> &stop,
                   std::vector<uint8_t> &candidate) const {
    for (size_t i = 0; i < info_.joints.size(); ++i) {
      const auto &joint = info_.joints[i];
      for (const auto &interface : joint.command_interfaces) {
        const auto key = joint.name + "/" + interface.name;
        const uint8_t mode = interface.name == "position" ? Command::POSITION : Command::VELOCITY;
        if (std::find(stop.begin(), stop.end(), key) != stop.end() && candidate[i] == mode) candidate[i] = Command::HOLD;
      }
      for (const auto &interface : joint.command_interfaces) {
        const auto key = joint.name + "/" + interface.name;
        if (std::find(start.begin(), start.end(), key) == start.end()) continue;
        if (candidate[i] != Command::HOLD) return false;
        candidate[i] = interface.name == "position" ? Command::POSITION : Command::VELOCITY;
      }
    }
    return true;
  }
  bool switchBody(const std::vector<std::string> &start, const std::vector<std::string> &stop,
                  std::array<bool, 4> &candidate) const {
    if (info_.gpios.empty()) return true;
    for (const auto &interface : info_.gpios.front().command_interfaces) {
      const auto i = std::find(body_names_.begin(), body_names_.begin() + 4, interface.name) - body_names_.begin();
      const auto key = info_.gpios.front().name + "/" + interface.name;
      if (std::find(stop.begin(), stop.end(), key) != stop.end()) candidate[i] = false;
      if (std::find(start.begin(), start.end(), key) != start.end()) {
        if (candidate[i]) return false;
        candidate[i] = true;
      }
    }
    return true;
  }
  void hold() {
    body_mode_.fill(false); body_command_.fill(0);
    std::fill(mode_.begin(), mode_.end(), Command::HOLD);
    if (!publisher_ || run_id_.empty()) return;
    Command msg; msg.run_id = run_id_; msg.header.stamp = stamp_;
    publishCommand(msg); // Empty command set stops all independently controlled joints.
  }
  const std::array<std::string, 11> body_names_{
      "linear.x.velocity", "linear.y.velocity", "linear.z.velocity", "angular.z.velocity",
      "position.x", "position.y", "position.z", "orientation.x", "orientation.y", "orientation.z", "orientation.w"};
  std::array<double, 11> body_state_{}, feedback_body_{};
  std::array<double, 4> body_command_{};
  std::array<bool, 4> body_mode_{}, body_reset_{};
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Publisher<Command>::SharedPtr publisher_;
  rclcpp::Subscription<Feedback>::SharedPtr subscription_;
  std::mutex mutex_;
  std::atomic<bool> stop_transport_{false};
  std::thread transport_thread_;
  rclcpp::TimerBase::SharedPtr watchdog_;
  std::vector<double> feedback_position_, feedback_velocity_;
  std::vector<double> position_, velocity_, position_command_, velocity_command_;
  std::vector<uint8_t> mode_;
  std::vector<bool> reset_on_write_;
  std::string run_id_;
  builtin_interfaces::msg::Time stamp_;
  Clock::time_point received_at_{};
  double timeout_ = 2, activation_timeout_ = 10;
  bool active_ = false, received_ = false, fault_ = false;
};
} // namespace litesim3d_ros2_control
// Only direct-class units omit registration; the installed DSO still exports.
#ifndef LITESIM3D_NO_PLUGIN_REGISTRATION
PLUGINLIB_EXPORT_CLASS(litesim3d_ros2_control::LiteSimSystem, hardware_interface::SystemInterface)
#endif
