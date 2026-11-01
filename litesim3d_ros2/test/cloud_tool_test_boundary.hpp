#pragma once
#include "../src/cloud_tool_boundary.hpp"
#include <nlohmann/json.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <functional>
#include <map>
#include <optional>
#include <stdexcept>
#include <type_traits>

namespace cloud_tool_test {
using Cloud = sensor_msgs::msg::PointCloud2;
using Text = std_msgs::msg::String;
struct State;
struct Subscription {
  std::weak_ptr<State> state;
  std::function<void(Cloud::ConstSharedPtr)> callback;
  size_t get_publisher_count() const;
};
struct Timer { std::function<void()> callback; };
struct State {
  nlohmann::json overrides = nlohmann::json::object();
  std::optional<std::string> run_id;
  std::vector<std::string> environment_names, events, parameters, argv;
  std::map<std::string, rmw_qos_profile_t> publishers, subscriptions;
  std::map<std::string, std::vector<Cloud>> clouds;
  std::map<std::string, std::vector<Text>> texts;
  std::map<std::string, unsigned> fail_publish;
  std::shared_ptr<Subscription> input;
  std::shared_ptr<Timer> timer;
  std::chrono::nanoseconds timer_period{};
  size_t input_publishers = 0, output_subscribers = 0;
};
inline size_t Subscription::get_publisher_count() const { return state.lock()->input_publishers; }

template<class Message> struct Publisher {
  std::weak_ptr<State> state;
  std::string topic;
  void publish(const Message &message) {
    auto target = state.lock();
    if (target->fail_publish[topic]) {
      --target->fail_publish[topic];
      throw std::runtime_error("injected publisher failure");
    }
    if constexpr (std::is_same_v<Message, Cloud>) target->clouds[topic].push_back(message);
    else target->texts[topic].push_back(message);
  }
  size_t get_subscription_count() const { return state.lock()->output_subscribers; }
};

struct Boundary {
  inline static std::shared_ptr<State> active;
  struct Node {
    std::shared_ptr<State> state = Boundary::active;
    explicit Node(const std::string &name) { state->events.push_back("node:" + name); }
    template<class T> T declare_parameter(const std::string &name, const T &fallback) {
      state->parameters.push_back(name);
      return state->overrides.contains(name) ? state->overrides.at(name).get<T>() : fallback;
    }
    template<class Message>
    std::shared_ptr<cloud_tool_test::Publisher<Message>> create_publisher(const std::string &topic,
                                                                        const rclcpp::QoS &qos) {
      state->publishers[topic] = qos.get_rmw_qos_profile();
      auto publisher = std::make_shared<cloud_tool_test::Publisher<Message>>();
      publisher->state = state;
      publisher->topic = topic;
      return publisher;
    }
    template<class Message, class Callback>
    std::shared_ptr<cloud_tool_test::Subscription> create_subscription(const std::string &topic,
                                                                     const rclcpp::QoS &qos,
                                                                     Callback callback) {
      static_assert(std::is_same_v<Message, Cloud>);
      state->subscriptions[topic] = qos.get_rmw_qos_profile();
      state->input = std::make_shared<cloud_tool_test::Subscription>();
      state->input->state = state;
      state->input->callback = std::move(callback);
      return state->input;
    }
    template<class Duration, class Callback>
    std::shared_ptr<cloud_tool_test::Timer> create_wall_timer(Duration duration, Callback callback) {
      state->timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(duration);
      state->timer = std::make_shared<cloud_tool_test::Timer>();
      state->timer->callback = std::move(callback);
      return state->timer;
    }
  };
  template<class Message> using Publisher = std::shared_ptr<cloud_tool_test::Publisher<Message>>;
  template<class Message> using Subscription = std::shared_ptr<cloud_tool_test::Subscription>;
  using Timer = std::shared_ptr<cloud_tool_test::Timer>;
  static const char *environment(const char *name) {
    active->environment_names.emplace_back(name);
    return active->run_id ? active->run_id->c_str() : nullptr;
  }
  static void initialize(int argc, char **argv) {
    active->events.push_back("initialize");
    for (int i = 0; i < argc; ++i) active->argv.emplace_back(argv[i]);
  }
  template<class NodeType> static void spin(const std::shared_ptr<NodeType> &) {
    active->events.push_back("spin");
  }
  static void shutdown() { active->events.push_back("shutdown"); }
};

struct Scope {
  std::shared_ptr<State> previous = Boundary::active;
  std::shared_ptr<State> state = std::make_shared<State>();
  Scope() { Boundary::active = state; }
  ~Scope() { Boundary::active = previous; }
  void emit(const Cloud &cloud) const { state->input->callback(std::make_shared<Cloud>(cloud)); }
};
} // namespace cloud_tool_test
