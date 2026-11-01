#include <litesim3d/rate.hpp>
#include "litesim3d/sim.hpp"
#include "service_responses.hpp"
#include "lidar_outputs.hpp"
#include "joint_control.hpp"
#include "world_backend.hpp"
#include <typeinfo>
#include <algorithm>
#include <cmath>
#include <deque>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <fstream>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <iostream>
#include <litesim3d_interfaces/msg/contact.hpp>
#include <litesim3d_interfaces/srv/step.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmw/rmw.h>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace {
uint64_t wallTime() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}
builtin_interfaces::msg::Time stamp(uint64_t ns) {
  builtin_interfaces::msg::Time t;
  t.sec = ns / 1000000000;
  t.nanosec = ns % 1000000000;
  return t;
}
geometry_msgs::msg::Pose pose(const litesim3d::Pose &p) {
  geometry_msgs::msg::Pose v;
  v.position.x = p.translation().x();
  v.position.y = p.translation().y();
  v.position.z = p.translation().z();
  Eigen::Quaterniond q(p.linear());
  v.orientation.x = q.x();
  v.orientation.y = q.y();
  v.orientation.z = q.z();
  v.orientation.w = q.w();
  return v;
}
} // namespace
// Side effects are injected at the bridge boundary. Production defaults remain
// ROS publishers, the node context, steady/system clocks and file streams.
struct WorldNodeHooks {
  std::function<std::unique_ptr<litesim3d_ros::WorldBackend>(const std::filesystem::path &)> load =
      litesim3d_ros::loadWorldBackend<>;
  std::function<std::chrono::steady_clock::time_point()> steady_now;
  std::function<uint64_t()> wall_now;
  std::function<bool()> ok;
  std::function<void()> shutdown;
  std::function<size_t(const std::string &)> publisher_count;
  std::function<void(const std::string &, const std::type_info &, const void *)> publish;
  std::function<std::unique_ptr<std::ostream>(const std::string &)> open_manifest;
  litesim3d_ros::LidarOutputs::PluginFactory lidar_factory;
};
class WorldNode : public rclcpp::Node {
  friend struct WorldNodeUnitAccess;
  struct Channels {
    std::string prefix;
    std::unique_ptr<litesim3d_ros::JointControl> joint_control;
    std::unique_ptr<litesim3d_ros::LidarOutputs> livox;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom, truth;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joints;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr description;
    rclcpp::Publisher<litesim3d_interfaces::msg::Contact>::SharedPtr
        contacts;
    std::map<std::string,
             rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr>
        clouds;
    std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr>
        imus;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
        stamped_command;
    std::set<std::string> odom_sources;
    bool publish_tf = true;
    uint64_t published = 0, contact_index = 0, qos_conflicts = 0;
    std::function<void(rclcpp::QOSOfferedIncompatibleQoSInfo &)> incompatible_qos;
  };

public:
  explicit WorldNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions(),
                     WorldNodeHooks hooks = {})
      : Node("litesim3d_world", options), hooks_(std::move(hooks)) {
    started_wall_ns_ = wallNow();
    stats_ = create_publisher<std_msgs::msg::String>("/sim/stats", rclcpp::QoS(1).best_effort());
    world_ = hooks_.load(declare_parameter<std::string>("config", ""));
    output_ = declare_parameter<std::string>("manifest_path", "");
    paused_ = declare_parameter<bool>("start_paused", false);
    cmd_mode_ = declare_parameter<std::string>("cmd_mode", "twist");
    litesim3d::require(cmd_mode_ == "twist" || cmd_mode_ == "stamped",
                 "cmd_mode must be twist or stamped");
    sensor_reliable_ = declare_parameter<bool>("sensor_reliable", false);
    const bool publish_tf = declare_parameter<bool>("publish_odom_tf", true);
    const auto external = declare_parameter<std::vector<std::string>>(
        "external_odom_tf_robots", std::vector<std::string>{});
    std::set<std::string> ids, external_ids;
    for (size_t i = 0; i < world_->size(); ++i)
      ids.insert(world_->robot(i).config().robot_id);
    for (const auto &id : external) {
      litesim3d::require(ids.count(id), "unknown external odom TF robot: " + id);
      litesim3d::require(external_ids.insert(id).second,
                   "duplicate external odom TF robot: " + id);
    }
    clock_ = create_publisher<rosgraph_msgs::msg::Clock>(
        "/clock", rclcpp::QoS(1).best_effort());
    status_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/sim/status", 10);
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    const auto lidar_interface = declare_parameter<std::string>("lidar_interface", "native");
    const auto xfer_format = declare_parameter<int>("xfer_format", 1);
    const auto lidar_id = declare_parameter<int>("livox_lidar_id", 0);
    channels_.resize(world_->size());
    for (size_t i = 0; i < world_->size(); ++i) {
      auto &c = channels_[i];
      const auto &r = world_->robot(i);
      c.prefix = r.config().robot_id + "/";
      c.publish_tf = publish_tf && !external_ids.count(r.config().robot_id);
      const auto topic = [&](const std::string &t) {
        return "/" + c.prefix + t;
      };
      c.odom = create_publisher<nav_msgs::msg::Odometry>(topic(r.config().control_mode == "ros2_control" ? "sim/odometry" : "odom"), 10);
      c.truth = create_publisher<nav_msgs::msg::Odometry>(
          "/sim/ground_truth/" + r.config().robot_id + "/odom", 10);
      c.contacts = create_publisher<litesim3d_interfaces::msg::Contact>(
          "/sim/ground_truth/" + r.config().robot_id + "/contacts", 10);
      if (r.config().control_mode == "ros2_control") {
        litesim3d_ros::JointControl::FeedbackSink feedback;
        if (hooks_.publish) feedback = [this, i](const auto &message) {
          hooks_.publish("/" + channels_[i].prefix + "sim/joint_feedback", typeid(message), &message);
        };
        c.joint_control = std::make_unique<litesim3d_ros::JointControl>(*this, r,
            [this, i](const auto &cmd) { return world_->jointCommand(i, cmd); }, "/" + c.prefix, feedback);
      } else c.joints = create_publisher<sensor_msgs::msg::JointState>(topic("joint_states"), 10);
      c.description = create_publisher<std_msgs::msg::String>(
          topic("robot_description"), rclcpp::QoS(1).transient_local());
      std_msgs::msg::String description;
      description.data = r.model().xml;
      emit(c.description, description);
      auto sensor_qos = rclcpp::QoS(5);
      if (sensor_reliable_)
        sensor_qos.reliable();
      else
        sensor_qos.best_effort();
      rclcpp::PublisherOptions options;
      c.incompatible_qos = [this, i](rclcpp::QOSOfferedIncompatibleQoSInfo &) {
        ++channels_[i].qos_conflicts;
      };
      options.event_callbacks.incompatible_qos_callback = c.incompatible_qos;
      c.livox = std::make_unique<litesim3d_ros::LidarOutputs>(*this, r.config(), "/" + c.prefix,
          c.prefix, lidar_interface, xfer_format, lidar_id, options, hooks_.lidar_factory);
      for (const auto &l : r.config().lidars) {
        if (c.livox->handles(l.topic)) continue;
        c.clouds[l.topic] = create_publisher<sensor_msgs::msg::PointCloud2>(
            topic(l.topic), sensor_qos, options);
        if (l.publish_range_boundary)
          c.clouds[l.topic + "/range_boundary"] = create_publisher<sensor_msgs::msg::PointCloud2>(
              topic(l.topic + "/range_boundary"), sensor_qos, options);
      }
      for (const auto &imu : r.config().imus)
        c.imus[imu.topic] = create_publisher<sensor_msgs::msg::Imu>(
            topic(imu.topic), sensor_qos, options);
      auto accept = [this, i](const geometry_msgs::msg::Twist &m,
                              std::optional<uint64_t> stamp,
                              const std::string &frame) {
        litesim3d::Command command;
        command.v = m.linear.x;
        command.w = m.angular.z;
        command.lateral = m.linear.y;
        command.vertical = m.linear.z;
        command.roll_rate = m.angular.x;
        command.pitch_rate = m.angular.y;
        command.stamp = stamp;
        command.frame = frame;
        world_->command(i, command);
      };
      if (!c.joint_control && cmd_mode_ == "twist")
        c.command = create_subscription<geometry_msgs::msg::Twist>(
            topic("cmd_vel"), rclcpp::QoS(1).reliable(),
            [accept](geometry_msgs::msg::Twist::ConstSharedPtr m) {
              accept(*m, {}, "");
            });
      else if (!c.joint_control)
        c.stamped_command =
            create_subscription<geometry_msgs::msg::TwistStamped>(
                topic("cmd_vel"), rclcpp::QoS(1).reliable(),
                [this, i,
                 accept](geometry_msgs::msg::TwistStamped::ConstSharedPtr m) {
                  const auto &base = world_->robot(i).config().base_link;
                  const auto &t = m->header.stamp;
                  const uint64_t ns =
                      t.sec >= 0 && t.nanosec < 1000000000
                          ? uint64_t(t.sec) * 1000000000 + t.nanosec
                          : UINT64_MAX;
                  // Validate the robot prefix before passing the model-local
                  // frame to the core.
                  const auto frame =
                      m->header.frame_id == channels_[i].prefix + base
                          ? base
                          : "invalid prefixed command frame: " +
                                m->header.frame_id;
                  accept(m->twist, ns, frame);
                });
    }
    tf_watch_ = create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf", rclcpp::QoS(100),
        [this](tf2_msgs::msg::TFMessage::ConstSharedPtr msg,
               const rclcpp::MessageInfo &info) {
          if (failed_ || !running())
            return;
          for (const auto &t : msg->transforms)
            for (size_t i = 0; i < channels_.size(); ++i) {
              auto &c = channels_[i];
              if (t.child_frame_id != c.prefix + world_->robot(i).model().root)
                continue;
              if (t.header.frame_id != c.prefix + "odom") {
                fail("unexpected odom TF parent for " + t.child_frame_id);
                return;
              }
              const auto &gid = info.get_rmw_message_info().publisher_gid;
              c.odom_sources.insert(
                  std::string(reinterpret_cast<const char *>(gid.data),
                              RMW_GID_STORAGE_SIZE));
              if (c.odom_sources.size() > 1) {
                fail("duplicate odom TF publishers for " + t.child_frame_id);
                return;
              }
            }
        });
    pause_handler_ = [this](std_srvs::srv::SetBool::Request::SharedPtr q,
                             std_srvs::srv::SetBool::Response::SharedPtr a) {
          try {
            world_->pause(q->data);
            paused_ = q->data;
            anchor_ = steadyNow();
            anchor_stamp_ = time();
            a->success = true;
          } catch (const std::exception &e) {
            a->message = e.what();
          }
        };
    pause_ = litesim3d_ros::createResponseService<std_srvs::srv::SetBool>(
        *this, "/sim/pause", pause_handler_);
    step_handler_ = [this](litesim3d_interfaces::srv::Step::Request::SharedPtr q,
               litesim3d_interfaces::srv::Step::Response::SharedPtr a) {
          try {
            std::vector<std::vector<litesim3d::Event>> events;
            auto result =
                ready_ ? world_->singleStep(q->request_id, q->steps, events)
                       : litesim3d::StepResult{false, 1, "not ready", time()};
            a->success = result.success;
            a->code = result.code;
            a->message = result.message;
            a->time_ns = result.time_ns;
            if (result.success)
              publish(events);
            publishStatus();
          } catch (const std::exception &e) {
            a->success = false;
            a->message = e.what();
            fail(e.what());
          }
        };
    step_ = litesim3d_ros::createResponseService<litesim3d_interfaces::srv::Step>(
        *this, "/sim/step", step_handler_);
    anchor_ = steadyNow();
    anchor_stamp_ = time();
    const auto &cfg = world_->robot(0).config();
    timer_ = create_wall_timer(
        std::chrono::nanoseconds(
            cfg.real_time_factor
                ? std::max<uint64_t>(
                      1, uint64_t(cfg.step_ns / cfg.real_time_factor))
                : 1),
        [this] { update(); });
    status_timer_ = create_wall_timer(std::chrono::milliseconds(200), [this] {
      try {
        publishStatus();
      } catch (const std::exception &e) {
        if (running())
          fail(e.what());
      }
    });
    publish(std::vector<std::vector<litesim3d::Event>>(world_->size()));
    save("ready");
  }
  bool failed() const { return failed_; }
  void finish() {
    if (!failed_)
      world_->exit();
    save(failed_ ? "fault" : "shutdown");
  }

private:
  uint64_t time() const { return world_->robot(0).snapshot().stamp; }
  void fail(const std::string &reason) {
    failed_ = true;
    error_ = reason;
    world_->fault(reason);
    RCLCPP_ERROR(get_logger(), "%s", reason.c_str());
    save("fault");
    if (hooks_.shutdown) hooks_.shutdown();
    else rclcpp::shutdown(get_node_base_interface()->get_context());
  }
  void save(const std::string &reason) {
    if (output_.empty())
      return;
    auto m = world_->manifest();
    m["exit_reason"] = reason;
    m["error"] = error_;
    m["ros_distro"] = "humble";
    m["rmw"] = rmw_get_implementation_identifier();
    m["bridge_source_sha256"] = LWS_BRIDGE_SHA256;
    m["started_wall_ns"] = started_wall_ns_;
    m["recorded_wall_ns"] = wallNow();
    m["ros_interface"] = {{"cmd_mode", cmd_mode_},
                          {"sensor_reliable", sensor_reliable_}};
    for (size_t i = 0; i < channels_.size(); ++i) {
      m["robots"][i]["exit_reason"] = reason;
      m["robots"][i]["livox_output"] = channels_[i].livox->description();
      m["ros_interface"]["publish_odom_tf"]
       [world_->robot(i).config().robot_id] = channels_[i].publish_tf;
    }
    auto stream = hooks_.open_manifest ? hooks_.open_manifest(output_)
                                      : std::make_unique<std::ofstream>(output_);
    auto &f = *stream;
    litesim3d::require(bool(f), "cannot open world manifest");
    f << m.dump(2);
    litesim3d::require(bool(f), "cannot write world manifest");
  }
  void state(size_t i, const litesim3d::Snapshot &s) {
    auto &c = channels_[i];
    const auto &r = world_->robot(i);
    sensor_msgs::msg::JointState js;
    js.header.stamp = stamp(s.stamp);
    for (auto &[name, value] : s.joints)
      if (r.model().joints.at(name).type != "fixed") {
        js.name.push_back(name);
        js.position.push_back(value);
        js.velocity.push_back(
            s.joint_velocities.count(name) ? s.joint_velocities.at(name) : 0);
      }
    if (c.joints) emit(c.joints, js);
    if (c.joint_control) c.joint_control->publish(js, s);
    if (!c.publish_tf)
      return;
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = js.header.stamp;
    t.header.frame_id = c.prefix + "odom";
    t.child_frame_id = c.prefix + r.model().root;
    auto p = pose(s.odom_root);
    t.transform.translation.x = p.position.x;
    t.transform.translation.y = p.position.y;
    t.transform.translation.z = p.position.z;
    t.transform.rotation = p.orientation;
    if (hooks_.publish) hooks_.publish("/tf", typeid(t), &t);
    else tf_->sendTransform(t);
  }
  void publishClock() {
    rosgraph_msgs::msg::Clock clock;
    clock.clock = stamp(time());
    emit(clock_, clock);
  }
  void publish(const std::vector<std::vector<litesim3d::Event>> &events) {
    publishClock();
    const std::vector<litesim3d::Event> empty;
    for (size_t i = 0; i < world_->size(); ++i)
      publishRobot(i, i < events.size() ? events[i] : empty);
  }
  void publishRobot(size_t i, const std::vector<litesim3d::Event> &events) {
    auto &c = channels_[i];
    const auto &r = world_->robot(i);
    for (const auto &e : events) {
      state(i, e.snapshot);
      if (e.odom) {
        nav_msgs::msg::Odometry o;
        o.header.stamp = stamp(e.snapshot.stamp);
        o.header.frame_id = c.prefix + "odom";
        o.child_frame_id = c.prefix + r.config().base_link;
        o.pose.pose = pose(e.snapshot.odom_base);
        o.twist.twist.linear.x = e.snapshot.odom_v;
        o.twist.twist.linear.y = e.snapshot.odom_lateral;
        o.twist.twist.linear.z = e.snapshot.odom_vertical;
        o.twist.twist.angular.z = e.snapshot.odom_w;
        o.pose.covariance = r.config().pose_covariance;
        o.twist.covariance = r.config().twist_covariance;
        emit(c.odom, o);
        o.header.frame_id = "world";
        o.pose.pose = pose(e.snapshot.world_base);
        o.twist.twist.linear.x = e.snapshot.v;
        o.twist.twist.linear.y = e.snapshot.lateral;
        o.twist.twist.linear.z = e.snapshot.vertical;
        o.twist.twist.angular.z = e.snapshot.w;
        o.pose.covariance.fill(0);
        o.twist.covariance.fill(0);
        emit(c.truth, o);
      }
      for (const auto &p : e.clouds) {
        if (!c.livox->publish(p)) {
          sensor_msgs::msg::PointCloud2 m;
          m.header.stamp = stamp(p.stamp);
          m.header.frame_id = c.prefix + p.frame;
          m.width = p.width;
          m.height = p.height;
          m.point_step = 24;
          m.row_step = p.row_step;
          m.is_dense = p.dense;
          m.is_bigendian = false;
          m.data = p.data;
          const char *names[] = {"x", "y", "z", "intensity", "ring", "time"};
          uint32_t offsets[] = {0, 4, 8, 12, 16, 20};
          for (int f = 0; f < 6; ++f) {
            sensor_msgs::msg::PointField field;
            field.name = names[f];
            field.offset = offsets[f];
            field.count = 1;
            field.datatype = f == 4 ? 4 : 7;
            m.fields.push_back(field);
          }
          emit(c.clouds.at(p.topic), m);
          if (!p.range_boundary.empty()) {
            m.data = p.range_boundary;
            m.is_dense = false;
            emit(c.clouds.at(p.topic + "/range_boundary"), m);
          }
        }
        ++c.published;
        compute_.push_back(p.compute_ms);
        auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
                       steadyNow().time_since_epoch())
                       .count();
        bridge_.push_back((now - p.compute_end_ns) * 1e-6);
        if (compute_.size() > 4096) {
          compute_.pop_front();
          bridge_.pop_front();
        }
      }
      for (const auto &p : e.imus) {
        sensor_msgs::msg::Imu m;
        m.header.stamp = stamp(p.stamp);
        m.header.frame_id = c.prefix + p.frame;
        m.orientation.w = 1;
        m.orientation_covariance = p.orientation_covariance;
        m.linear_acceleration_covariance = p.acceleration_covariance;
        m.angular_velocity_covariance = p.angular_velocity_covariance;
        m.linear_acceleration.x = p.acceleration.x();
        m.linear_acceleration.y = p.acceleration.y();
        m.linear_acceleration.z = p.acceleration.z();
        m.angular_velocity.x = p.angular_velocity.x();
        m.angular_velocity.y = p.angular_velocity.y();
        m.angular_velocity.z = p.angular_velocity.z();
        emit(c.imus.at(p.topic), m);
      }
    }
    state(i, r.snapshot());
    c.contact_index =
        std::max(c.contact_index, r.contactCount() - r.contacts().size());
    while (c.contact_index < r.contactCount()) {
      const auto &p = r.contacts()[c.contact_index++ -
                                   (r.contactCount() - r.contacts().size())];
      litesim3d_interfaces::msg::Contact m;
      m.header.stamp = stamp(p.stamp);
      m.header.frame_id = "world";
      m.run_id = r.runId();
      m.robot_link = c.prefix + p.robot_link;
      m.entity = p.entity;
      m.clearance = p.clearance;
      emit(c.contacts, m);
    }
  }
  void update() {
    if (!running())
      return;
    try {
      if (!ready_) {
        ready_ = true;
        world_->pause(paused_);
        anchor_ = steadyNow();
        anchor_stamp_ = time();
      }
      if ((hooks_.publisher_count ? hooks_.publisher_count("/clock") : count_publishers("/clock")) > 1) {
        fail("multiple /clock publishers");
        return;
      }
      // A continuously ready unpaced timer must also service wall-clock
      // diagnostics.
      if (steadyNow() >= next_status_)
        publishStatus();
      if (paused_)
        return;
      const auto &cfg = world_->robot(0).config();
      unsigned steps = 1;
      if (cfg.real_time_factor) {
        auto elapsed = std::chrono::duration<double>(
                           steadyNow() - anchor_)
                           .count();
        uint64_t target =
            anchor_stamp_ + uint64_t(elapsed * cfg.real_time_factor * 1e9);
        steps = target > time()
                    ? std::min<uint64_t>(8, (target - time()) / cfg.step_ns)
                    : 0;
      }
      for (unsigned i = 0; i < steps; ++i)
        world_->tick(
            [this](size_t robot, const std::vector<litesim3d::Event> &events) {
              if (robot == 0)
                publishClock();
              publishRobot(robot, events);
            });
    } catch (const std::exception &e) {
      if (running())
        fail(e.what());
    }
  }
  void publishStatus() {
    if (!running())
      return;
    next_status_ =
        steadyNow() + std::chrono::milliseconds(200);
    const auto &first = world_->robot(0);
    const auto actual = rate_.sample(world_->runId(), time(), double(wallNow()) * 1e-9);
    const bool paused = first.state() == litesim3d::State::Paused;
    const auto rate = paused ? litesim3d::Json(0.0) : (actual ? litesim3d::Json(*actual) : litesim3d::Json(nullptr));
    std_msgs::msg::String stats;
    stats.data = litesim3d::Json({{"schema_version", 1}, {"run_id", world_->runId()},
      {"stamp_ns", time()}, {"state", first.diagnostics()["state"]},
      {"actual_real_time_factor", rate}, {"target_real_time_factor", first.config().real_time_factor},
      {"collision_budget_enabled", first.config().collision_budget_enabled}, {"robot_count", world_->size()}}).dump();
    emit(stats_, stats);
    diagnostic_msgs::msg::DiagnosticArray msg;
    msg.header.stamp = stamp(time());
    auto p99 = [](const std::deque<double> &v) {
      if (v.empty())
        return 0.;
      auto a = std::vector<double>(v.begin(), v.end());
      std::sort(a.begin(), a.end());
      return a[size_t(std::ceil(.99 * a.size())) - 1];
    };
    for (size_t i = 0; i < world_->size(); ++i) {
      auto d = world_->robot(i).diagnostics();
      d["ready"] = ready_;
      d["world_run_id"] = world_->runId();
      d["published_clouds"] = channels_[i].published;
      auto matches = nlohmann::json::object();
      auto count_matches = [&matches](const auto &publisher) {
        matches[publisher->get_topic_name()] = publisher->get_subscription_count();
      };
      count_matches(clock_);
      count_matches(channels_[i].odom);
      if (channels_[i].joints) count_matches(channels_[i].joints);
      channels_[i].livox->matches(matches);
      d["livox_output"] = channels_[i].livox->description();
      for (const auto &[topic, publisher] : channels_[i].clouds) count_matches(publisher);
      for (const auto &[topic, publisher] : channels_[i].imus) count_matches(publisher);
      d["subscription_matches"] = std::move(matches);
      d["internal_missed_scans"] = 0;
      d["compute_p99_ms"] = p99(compute_);
      d["bridge_p99_ms"] = p99(bridge_);
      d["timing_window_frames"] = compute_.size();
      d["ray_threads"] = world_->robot(i).config().threads;
      d["robot_count"] = world_->size();
      d["cmd_mode"] = cmd_mode_;
      d["publish_odom_tf"] = channels_[i].publish_tf;
      d["qos_conflicts"] = channels_[i].qos_conflicts;
      d["sensor_queue_depth"] = 5;
      d["sensor_queue_policy"] =
          sensor_reliable_ ? "keep_last/reliable" : "keep_last/best_effort";
      d["dds_delivery_drops"] = "unknown (measure at subscriber)";
      diagnostic_msgs::msg::DiagnosticStatus s;
      s.name = channels_[i].prefix + "sim";
      s.hardware_id = world_->robot(i).runId();
      s.level = failed_ ? 2
                        : ((channels_[i].qos_conflicts ||
                            d["command_timeout"].get<bool>() ||
                            d["rejected_commands"].get<uint64_t>())
                               ? 1
                               : 0);
      s.message = failed_ ? error_ : d["state"].get<std::string>();
      for (auto it = d.begin(); it != d.end(); ++it) {
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = it.key();
        kv.value = it.value().is_string() ? it.value().get<std::string>()
                                          : it.value().dump();
        s.values.push_back(kv);
      }
      msg.status.push_back(s);
    }
    emit(status_, msg);
  }
  template<class Message>
  void emit(const std::shared_ptr<rclcpp::Publisher<Message>> &publisher, const Message &message) {
    if (hooks_.publish) hooks_.publish(publisher->get_topic_name(), typeid(Message), &message);
    else publisher->publish(message);
  }
  std::chrono::steady_clock::time_point steadyNow() const {
    return hooks_.steady_now ? hooks_.steady_now() : std::chrono::steady_clock::now();
  }
  uint64_t wallNow() const { return hooks_.wall_now ? hooks_.wall_now() : wallTime(); }
  bool running() const {
    return hooks_.ok ? hooks_.ok() : rclcpp::ok(get_node_options().context());
  }
  WorldNodeHooks hooks_;
  litesim3d::RealTimeRate rate_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stats_;
  std::unique_ptr<litesim3d_ros::WorldBackend> world_;
  std::vector<Channels> channels_;
  std::string output_, error_, cmd_mode_;
  bool sensor_reliable_ = false;
  uint64_t started_wall_ns_ = 0;
  bool ready_ = false, paused_ = false, failed_ = false;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_watch_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_;
  std::function<void(std_srvs::srv::SetBool::Request::SharedPtr,
                     std_srvs::srv::SetBool::Response::SharedPtr)> pause_handler_;
  std::function<void(litesim3d_interfaces::srv::Step::Request::SharedPtr,
                     litesim3d_interfaces::srv::Step::Response::SharedPtr)> step_handler_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr pause_;
  rclcpp::Service<litesim3d_interfaces::srv::Step>::SharedPtr step_;
  rclcpp::TimerBase::SharedPtr timer_, status_timer_;
  std::chrono::steady_clock::time_point anchor_, next_status_;
  uint64_t anchor_stamp_ = 1;
  std::deque<double> compute_, bridge_;
};
// Lifecycle policy is independently callable without an executor or a real
// world. Empty operations select exactly the normal ROS process operations.
struct WorldMainHooks {
  std::function<void(int, char **)> initialize;
  std::function<std::shared_ptr<WorldNode>()> make_node;
  std::function<void(const std::shared_ptr<WorldNode> &)> spin;
  std::function<bool()> ok;
  std::function<void()> shutdown;
  std::function<void(const std::string &)> report;
};
int runWorldNode(int argc, char **argv, const WorldMainHooks &hooks = {}) {
  if (hooks.initialize) hooks.initialize(argc, argv);
  else rclcpp::init(argc, argv);
  auto running = [&] { return hooks.ok ? hooks.ok() : rclcpp::ok(); };
  std::shared_ptr<WorldNode> node;
  int code = 0;
  try {
    node = hooks.make_node ? hooks.make_node() : std::make_shared<WorldNode>();
    if (hooks.spin) hooks.spin(node);
    else rclcpp::spin(node);
    code = node->failed() ? 1 : 0;
    node->finish();
  } catch (const std::exception &e) {
    if (node && !running() && !node->failed())
      node->finish();
    else {
      if (hooks.report) hooks.report(e.what());
      else std::cerr << e.what() << '\n';
      code = 1;
    }
  }
  if (running()) {
    if (hooks.shutdown) hooks.shutdown();
    else rclcpp::shutdown();
  }
  return code;
}
int main(int argc, char **argv) { return runWorldNode(argc, argv); }
