#include <litesim3d/rate.hpp>
#include "litesim3d/sim.hpp"
#include "service_responses.hpp"
#include "livox_outputs.hpp"
#include <chrono>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <fstream>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <iostream>
#include <litesim3d_interfaces/msg/contact.hpp>
#include <litesim3d_interfaces/srv/set_pose.hpp>
#include <litesim3d_interfaces/srv/step.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmw/rmw.h>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include "litesim3d/sensor_plugin.hpp"
#include <litesim3d_interfaces/srv/render_cameras.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <thread>
#include <visualization_msgs/msg/marker_array.hpp>

namespace {
builtin_interfaces::msg::Time stamp(uint64_t ns) {
  builtin_interfaces::msg::Time s;
  s.sec = static_cast<int32_t>(ns / 1000000000);
  s.nanosec = ns % 1000000000;
  return s;
}
geometry_msgs::msg::Pose poseMsg(const litesim3d::Pose &p) {
  geometry_msgs::msg::Pose m;
  const auto v = p.translation();
  Eigen::Quaterniond q(p.linear());
  m.position.x = v.x();
  m.position.y = v.y();
  m.position.z = v.z();
  m.orientation.x = q.x();
  m.orientation.y = q.y();
  m.orientation.z = q.z();
  m.orientation.w = q.w();
  return m;
}
} // namespace
class SimNode : public rclcpp::Node {
public:
  SimNode() : Node("litesim3d") {
    status_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/sim/status",
                                                                      rclcpp::QoS(10).reliable());
    const auto file = declare_parameter<std::string>("config", "");
    prefix_ = declare_parameter<std::string>("frame_prefix", "");
    litesim3d::require(prefix_.empty() || (prefix_.back() == '/' && prefix_.front() != '/'),
                 "frame_prefix must be empty or relative and end in '/'");
    publish_tf_ = declare_parameter<bool>("publish_odom_tf", true);
    const bool external_odom_tf = declare_parameter<bool>("external_odom_tf", false);
    litesim3d::require(!(publish_tf_ && external_odom_tf), "conflicting odom TF publishers configured");
    wait_rsp_ = declare_parameter<bool>("wait_for_robot_state_publisher", true);
    rsp_name_ =
        declare_parameter<std::string>("robot_state_publisher_name", "robot_state_publisher");
    viewer_ready_file_ = declare_parameter<std::string>("viewer_ready_file", "");
    startup_timeout_ = declare_parameter<double>("startup_timeout", 10.0);
    litesim3d::require(std::isfinite(startup_timeout_) && startup_timeout_ > 0,
                 "invalid startup_timeout");
    paused_ = declare_parameter<bool>("start_paused", false);
    manifest_path_ = declare_parameter<std::string>("manifest_path", "");
    litesim3d::Scene scene;
    auto cfg = litesim3d::Config::load(file, scene);
    auto initial = declare_parameter<std::vector<double>>("initial_pose_override", std::vector<double>{});
    if (!initial.empty()) {
      litesim3d::require(initial.size() == 7, "initial_pose_override requires xyz and xyzw");
      cfg.initial_pose = litesim3d::quaternionPose({initial[0], initial[1], initial[2]},
          {initial[6], initial[3], initial[4], initial[5]});
    }
    auto model = litesim3d::Model::load(cfg.urdf, cfg.model_options);
    sim_ = std::make_unique<litesim3d::Simulator>(cfg, model, scene);
    cfg = sim_->config();
    stats_ = create_publisher<std_msgs::msg::String>("/sim/stats", rclcpp::QoS(1).best_effort());
    const auto scene_file = declare_parameter<std::string>("scene_description", "");
    if (!scene_file.empty()) {
      scene_description_ = litesim3d::Json::parse(litesim3d::readText(scene_file));
      scene_description_["cameras"] = litesim3d::Json::array();
      for (const auto &camera : cfg.cameras) scene_description_["cameras"].push_back(camera.json());
      scene_description_["run_id"] = sim_->run_id;
      scene_description_pub_ = create_publisher<std_msgs::msg::String>(
          "/sim/scene_description", rclcpp::QoS(1).reliable().transient_local());
      scene_state_ = create_publisher<std_msgs::msg::String>("/sim/scene_state", rclcpp::QoS(1).best_effort());
      std_msgs::msg::String description; description.data = scene_description_.dump();
      scene_description_pub_->publish(description);
      scene_timer_ = create_wall_timer(std::chrono::milliseconds(33), [this] { publishSceneState(); });
    }
    const auto mode = declare_parameter<std::string>("cmd_mode", "twist");
    litesim3d::require(mode == "twist" || mode == "stamped", "cmd_mode must be twist or stamped");
    sensor_reliable_ = declare_parameter<bool>("sensor_reliable", false);
    drop_frames_ = declare_parameter<int>("test_drop_frames", 0);
    litesim3d::require(drop_frames_ >= 0 && drop_frames_ <= 100, "test_drop_frames must be in [0,100]");
    rclcpp::QoS sensor_qos(5);
    if (sensor_reliable_)
      sensor_qos.reliable();
    else
      sensor_qos.best_effort();
    rclcpp::PublisherOptions options;
    options.event_callbacks.incompatible_qos_callback =
        [this](rclcpp::QOSOfferedIncompatibleQoSInfo &) { ++qos_conflicts_; };
    livox_ = std::make_unique<litesim3d_ros::LivoxOutputs>(*this, cfg, "", prefix_,
        declare_parameter<std::string>("lidar_interface", "native"),
        declare_parameter<int>("xfer_format", 1), declare_parameter<int>("livox_lidar_id", 0), options);
    for (auto &lidar : cfg.lidars) {
      if (livox_->handles(lidar.topic)) continue;
      clouds_[lidar.topic] =
          create_publisher<sensor_msgs::msg::PointCloud2>(lidar.topic, sensor_qos, options);
      if (lidar.publish_range_boundary)
        clouds_[lidar.topic + "/range_boundary"] = create_publisher<sensor_msgs::msg::PointCloud2>(
            lidar.topic + "/range_boundary", sensor_qos, options);
    }
    for (const auto &imu : cfg.imus)
      imus_[imu.topic] = create_publisher<sensor_msgs::msg::Imu>(imu.topic, sensor_qos, options);
    for (const auto &c : cfg.cameras) if (c.enabled) {
      litesim3d::require(!scene_description_.is_null(), "enabled cameras require an imported SDF scene_description");
      for (const auto &channel : {std::string("color"), std::string("depth")})
        if ((channel == "color" && c.color()) || (channel == "depth" && c.depth())) {
          images_[c.topic + "/" + channel + "/image_raw"] = create_publisher<sensor_msgs::msg::Image>(
              c.topic + "/" + channel + "/image_raw", sensor_qos, options);
          camera_info_[c.topic + "/" + channel + "/camera_info"] = create_publisher<sensor_msgs::msg::CameraInfo>(
              c.topic + "/" + channel + "/camera_info", sensor_qos, options);
        }
    }
    if (!images_.empty()) {
      camera_client_node_ = std::make_shared<rclcpp::Node>("litesim_camera_client");
      camera_client_ = camera_client_node_->create_client<litesim3d_interfaces::srv::RenderCameras>("/sim/render_cameras");
      camera_executor_.add_node(camera_client_node_);
      startup_timeout_ = std::max(startup_timeout_, 30.0);
    }
    odom_ = create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(10).reliable());
    truth_ = create_publisher<nav_msgs::msg::Odometry>(
        "/sim/ground_truth/" + cfg.robot_id + "/odom", rclcpp::QoS(10).reliable());
    contacts_ = create_publisher<litesim3d_interfaces::msg::Contact>(
        "/sim/ground_truth/" + cfg.robot_id + "/contacts", rclcpp::QoS(10).reliable());
    joints_ =
        create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::QoS(10).reliable());
    clock_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::QoS(1).best_effort());
    description_ = create_publisher<std_msgs::msg::String>(
        "robot_description", rclcpp::QoS(1).reliable().transient_local());
    std_msgs::msg::String description;
    description.data = model.xml;
    description_->publish(description);
    markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "scene", rclcpp::QoS(1).reliable().transient_local());
    path_ = create_publisher<nav_msgs::msg::Path>("trajectory", rclcpp::QoS(1).reliable());
    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    auto accept = [this](const geometry_msgs::msg::Twist &m, std::optional<uint64_t> time,
                         std::string frame) {
      litesim3d::Command c;
      c.v = m.linear.x;
      c.w = m.angular.z;
      c.lateral = m.linear.y;
      c.vertical = m.linear.z;
      c.roll_rate = m.angular.x;
      c.pitch_rate = m.angular.y;
      c.stamp = time;
      c.frame =
          frame.empty()
              ? frame
              : (frame == prefix_ + sim_->config().base_link ? sim_->config().base_link : frame);
      sim_->command(c);
    };
    if (mode == "twist")
      twist_ = create_subscription<geometry_msgs::msg::Twist>(
          "cmd_vel", rclcpp::QoS(1).reliable(),
          [accept](geometry_msgs::msg::Twist::ConstSharedPtr m) { accept(*m, std::nullopt, ""); });
    else
      stamped_ = create_subscription<geometry_msgs::msg::TwistStamped>(
          "cmd_vel", rclcpp::QoS(1).reliable(),
          [accept](geometry_msgs::msg::TwistStamped::ConstSharedPtr m) {
            accept(m->twist, uint64_t(m->header.stamp.sec) * 1000000000 + m->header.stamp.nanosec,
                   m->header.frame_id);
          });
    pause_service_ = litesim3d_ros::createResponseService<std_srvs::srv::SetBool>(
        *this, "/sim/pause", [this](std_srvs::srv::SetBool::Request::SharedPtr request,
                             std_srvs::srv::SetBool::Response::SharedPtr response) {
          try {
            sim_->pause(request->data);
            paused_ = request->data;
            pace_time_ = std::chrono::steady_clock::now();
            pace_stamp_ = sim_->snapshot().stamp;
            response->success = true;
            response->message = "state updated";
          } catch (const std::exception &e) {
            response->success = false;
            response->message = e.what();
          }
          publishStatus();
        });
    step_service_ = litesim3d_ros::createResponseService<litesim3d_interfaces::srv::Step>(
        *this, "/sim/step", [this](litesim3d_interfaces::srv::Step::Request::SharedPtr request,
                            litesim3d_interfaces::srv::Step::Response::SharedPtr response) {
          std::vector<litesim3d::Event> events;
          litesim3d::StepResult result;
          try {
            result = ready_ ? sim_->singleStep(request->request_id, request->steps, events)
                            : litesim3d::StepResult{false, 1, "dependencies not ready",
                                              sim_->snapshot().stamp};
            publish(events);
          } catch (const std::exception &e) {
            sim_->fault(e.what());
            result = {false, 1, e.what(), sim_->snapshot().stamp};
          }
          response->success = result.success;
          response->code = result.code;
          response->message = result.message;
          response->time_ns = result.time_ns;
          publishStatus();
        });
    pose_service_ = litesim3d_ros::createResponseService<litesim3d_interfaces::srv::SetPose>(
        *this, "/sim/set_pose",
        [this](litesim3d_interfaces::srv::SetPose::Request::SharedPtr request,
               litesim3d_interfaces::srv::SetPose::Response::SharedPtr response) {
          try {
            const auto &p = request->pose;
            sim_->setPose(litesim3d::quaternionPose(
                {p.position.x, p.position.y, p.position.z},
                {p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z}));
            publishScene();
            saveManifest("initialized");
            response->success = true;
            response->code = 0;
          } catch (const std::exception &e) {
            response->success = false;
            response->code = 1;
            response->message = e.what();
          }
          response->time_ns = sim_->snapshot().stamp;
        });
    tf_watch_ = create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf", rclcpp::QoS(100),
        [this](tf2_msgs::msg::TFMessage::ConstSharedPtr msg, const rclcpp::MessageInfo &info) {
          for (auto &t : msg->transforms)
            if (t.header.frame_id == prefix_ + "odom" &&
                t.child_frame_id == prefix_ + sim_->model().root) {
              const auto &gid = info.get_rmw_message_info().publisher_gid;
              const std::string key(reinterpret_cast<const char *>(gid.data), RMW_GID_STORAGE_SIZE);
              odom_sources_.insert(key);
              if (odom_sources_.size() > 1)
                sim_->fault("duplicate odom TF publishers");
            }
        });
    auto interval = std::chrono::nanoseconds(
        cfg.real_time_factor > 0
            ? std::max<uint64_t>(1, uint64_t(cfg.step_ns / cfg.real_time_factor))
            : 1);
    started_ = std::chrono::steady_clock::now();
    timer_ = create_wall_timer(interval, [this] { update(); });
    status_timer_ = create_wall_timer(std::chrono::milliseconds(200), [this] {
      publishPeriodicStatus();
    });
    publishScene();
    publishClock();
    publishState(sim_->snapshot());
    saveManifest("ready");
  }
  bool failed() const { return sim_ && sim_->state() == litesim3d::State::Fault; }
  void finish() {
    if (sim_) {
      const bool error = failed();
      if (!error)
        sim_->exit();
      saveManifest(error ? "fault" : "shutdown");
    }
  }

private:
  void publishCameras(const litesim3d::Event &event) {
    if (event.cameras.empty()) return;
    using Service = litesim3d_interfaces::srv::RenderCameras;
    auto request = std::make_shared<Service::Request>();
    request->header.stamp = stamp(event.snapshot.stamp); request->header.frame_id = "world";
    request->scene_id = scene_description_.at("scene_id"); request->run_id = sim_->run_id;
    const auto frames = sim_->model().fk(event.snapshot.joints);
    const litesim3d::Pose root = event.snapshot.world_base * frames.at(sim_->config().base_link).inverse();
    for (const auto &[name, local] : frames) {
      request->link_names.push_back(name); request->link_poses.push_back(poseMsg(root * local));
    }
    for (const auto index : event.cameras) request->camera_names.push_back(sim_->config().cameras.at(index).name);
    litesim3d::require(camera_client_ && camera_client_->service_is_ready(), "camera renderer disconnected");
    auto future = camera_client_->async_send_request(request);
    // A separate executor receives the reply without reentering physics callbacks.
    // There is exactly one in-flight frame batch: slow rendering slows simulation.
    const auto status = camera_executor_.spin_until_future_complete(future, std::chrono::seconds(30));
    if (status != rclcpp::FutureReturnCode::SUCCESS) {
      camera_client_->remove_pending_request(future);
      throw std::runtime_error("camera render timeout or shutdown");
    }
    auto response = future.get();
    litesim3d::require(response->success, "camera rendering failed: " + response->message);
    litesim3d::require(response->frames.size() == event.cameras.size(), "incomplete camera response");
    for (size_t i = 0; i < event.cameras.size(); ++i) {
      auto &frame = response->frames[i]; const auto &c = sim_->config().cameras.at(event.cameras[i]);
      litesim3d::require(frame.camera_name == c.name && frame.info.header.stamp == request->header.stamp &&
          frame.info.width == c.width && frame.info.height == c.height && frame.info.header.frame_id == c.link,
          "camera response metadata mismatch");
      // The renderer supplies ideal truth-based pixels; the selected plugin owns
      // measurement effects. Preserve response metadata and byte ordering.
      litesim3d::plugins::ImageFrame pixels;
      pixels.stamp_ns = event.snapshot.stamp; pixels.width = c.width; pixels.height = c.height;
      pixels.depth_big_endian = frame.depth.is_bigendian;
      pixels.color.swap(frame.color.data); pixels.depth.swap(frame.depth.data);
      sim_->processCamera(event.cameras[i],pixels);
      pixels.color.swap(frame.color.data); pixels.depth.swap(frame.depth.data);
      auto send = [&](sensor_msgs::msg::Image &image, const std::string &channel, const char *encoding, unsigned bytes) {
        litesim3d::require(image.header == frame.info.header && image.encoding == encoding &&
            image.width == c.width && image.height == c.height && image.step == c.width * bytes &&
            image.data.size() == size_t(image.step) * image.height, "invalid camera image buffer");
        image.header.frame_id = prefix_ + c.link;
        auto info = frame.info; info.header = image.header;
        images_.at(c.topic + "/" + channel + "/image_raw")->publish(image);
        camera_info_.at(c.topic + "/" + channel + "/camera_info")->publish(info);
      };
      if (c.color()) send(frame.color, "color", "rgb8", 3);
      if (c.depth()) send(frame.depth, "depth", "32FC1", 4);
      ++published_camera_frames_;
    }
  }
  void publishSceneState() {
    const auto now = std::chrono::steady_clock::now();
    if (!scene_state_ || now < next_scene_state_) return;
    next_scene_state_ = now + std::chrono::milliseconds(33);
    const auto &s = sim_->snapshot();
    const auto frames = sim_->model().fk(s.joints);
    const auto world_root = s.world_base * frames.at(sim_->config().base_link).inverse();
    auto poses = litesim3d::Json::object();
    for (const auto &[name, local] : frames) {
      const litesim3d::Pose p = world_root * local; const Eigen::Quaterniond q(p.linear());
      poses[name] = {p.translation().x(), p.translation().y(), p.translation().z(), q.x(), q.y(), q.z(), q.w()};
    }
    std_msgs::msg::String message;
    message.data = litesim3d::Json({{"schema_version", 1}, {"run_id", sim_->run_id},
      {"scene_id", scene_description_.at("scene_id")}, {"stamp_ns", s.stamp},
      {"link_poses", poses}}).dump();
    scene_state_->publish(message);
  }
  void publishClock() {
    rosgraph_msgs::msg::Clock c;
    c.clock = stamp(sim_->snapshot().stamp);
    clock_->publish(c);
  }
  void publishState(const litesim3d::Snapshot &s) {
    sensor_msgs::msg::JointState js;
    js.header.stamp = stamp(s.stamp);
    for (auto &[name, value] : s.joints)
      if (sim_->model().joints.at(name).type != "fixed") {
        js.name.push_back(name);
        js.position.push_back(value);
        js.velocity.push_back(s.joint_velocities.count(name) ? s.joint_velocities.at(name) : 0);
      }
    joints_->publish(js);
    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = stamp(s.stamp);
      t.header.frame_id = prefix_ + "odom";
      t.child_frame_id = prefix_ + sim_->model().root;
      const auto p = poseMsg(s.odom_root);
      t.transform.translation.x = p.position.x;
      t.transform.translation.y = p.position.y;
      t.transform.translation.z = p.position.z;
      t.transform.rotation = p.orientation;
      broadcaster_->sendTransform(t);
    }
  }
  void publish(const std::vector<litesim3d::Event> &events) {
    publishClock();
    for (auto &e : events) {
      publishState(e.snapshot);
      publishCameras(e);
      if (e.odom) {
        nav_msgs::msg::Odometry o;
        o.header.stamp = stamp(e.snapshot.stamp);
        o.header.frame_id = prefix_ + "odom";
        o.child_frame_id = prefix_ + sim_->config().base_link;
        o.pose.pose = poseMsg(e.snapshot.odom_base);
        o.twist.twist.linear.x = e.snapshot.odom_v;
        o.twist.twist.linear.y = e.snapshot.odom_lateral;
        o.twist.twist.linear.z = e.snapshot.odom_vertical;
        o.twist.twist.angular.z = e.snapshot.odom_w;
        o.pose.covariance = sim_->config().pose_covariance;
        o.twist.covariance = sim_->config().twist_covariance;
        odom_->publish(o);
        auto truth = o;
        truth.header.frame_id = "world";
        truth.pose.pose = poseMsg(e.snapshot.world_base);
        truth.pose.covariance.fill(0);
        truth.twist.covariance.fill(0);
        truth.twist.twist.linear.x = e.snapshot.v;
        truth.twist.twist.linear.y = e.snapshot.lateral;
        truth.twist.twist.linear.z = e.snapshot.vertical;
        truth.twist.twist.angular.z = e.snapshot.w;
        truth_->publish(truth);
        geometry_msgs::msg::PoseStamped p;
        p.header = o.header;
        p.pose = o.pose.pose;
        trajectory_.header = o.header;
        trajectory_.poses.push_back(p);
        if (trajectory_.poses.size() > 2000)
          trajectory_.poses.erase(trajectory_.poses.begin(), trajectory_.poses.begin() + 1000);
        path_->publish(trajectory_);
      }
      for (const auto &imu : e.imus) {
        sensor_msgs::msg::Imu m;
        m.header.stamp = stamp(imu.stamp);
        m.header.frame_id = prefix_ + imu.frame;
        m.orientation.w = 1;
        m.orientation_covariance = imu.orientation_covariance;
        m.linear_acceleration_covariance = imu.acceleration_covariance;
        m.angular_velocity_covariance = imu.angular_velocity_covariance;
        m.linear_acceleration.x = imu.acceleration.x();
        m.linear_acceleration.y = imu.acceleration.y();
        m.linear_acceleration.z = imu.acceleration.z();
        m.angular_velocity.x = imu.angular_velocity.x();
        m.angular_velocity.y = imu.angular_velocity.y();
        m.angular_velocity.z = imu.angular_velocity.z();
        imus_.at(imu.topic)->publish(m);
        ++published_imus_;
      }
      for (auto &cloud : e.clouds) {
        if (drop_frames_ > 0) {
          --drop_frames_;
          ++dropped_frames_;
          continue;
        }
        if (!livox_->publish(cloud)) {
          sensor_msgs::msg::PointCloud2 m;
          m.header.stamp = stamp(cloud.stamp);
          m.header.frame_id = prefix_ + cloud.frame;
          m.height = cloud.height;
          m.width = cloud.width;
          m.point_step = 24;
          m.row_step = cloud.row_step;
          m.is_bigendian = false;
          m.is_dense = cloud.dense;
          m.data = cloud.data;
          const char *names[] = {"x", "y", "z", "intensity", "ring", "time"};
          const uint32_t offsets[] = {0, 4, 8, 12, 16, 20};
          for (size_t i = 0; i < 6; ++i) {
            sensor_msgs::msg::PointField f;
            f.name = names[i];
            f.offset = offsets[i];
            f.count = 1;
            f.datatype = i == 4 ? 4 : 7;
            m.fields.push_back(f);
          }
          clouds_.at(cloud.topic)->publish(m);
          if (!cloud.range_boundary.empty()) {
            m.data = cloud.range_boundary;
            m.is_dense = false;
            clouds_.at(cloud.topic + "/range_boundary")->publish(m);
          }
        }
        ++published_clouds_;
        const auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        compute_ms_.push_back(cloud.compute_ms);
        bridge_ms_.push_back((now - cloud.compute_end_ns) * 1e-6);
        if (compute_ms_.size() > 4096) { compute_ms_.pop_front(); bridge_ms_.pop_front(); }
      }
    }
    // Also publish the final integration timestamp; non-divisor sensor rates retain their own exact
    // stamps.
    publishState(sim_->snapshot());
    const auto first_contact = sim_->contactCount() - sim_->contacts().size();
    contact_index_ = std::max<uint64_t>(contact_index_, first_contact);
    while (contact_index_ < sim_->contactCount()) {
      const auto &c = sim_->contacts()[contact_index_++ - first_contact];
      litesim3d_interfaces::msg::Contact m;
      m.header.stamp = stamp(c.stamp);
      m.header.frame_id = "world";
      m.run_id = sim_->run_id;
      m.robot_link = prefix_ + c.robot_link;
      m.entity = c.entity;
      m.clearance = c.clearance;
      contacts_->publish(m);
    }
  }
  void update() {
    if (!rclcpp::ok()) return;
    if (!ready_) {
      bool rsp_ready = !wait_rsp_;
      if (wait_rsp_) {
        for (const auto &endpoint : get_subscriptions_info_by_topic(joints_->get_topic_name())) {
          if (endpoint.node_name() == rsp_name_ && endpoint.node_namespace() == get_namespace())
            rsp_ready = true;
        }
      }
      const bool viewer_ready = viewer_ready_file_.empty() || std::filesystem::is_regular_file(viewer_ready_file_);
      const bool camera_ready = !camera_client_ || camera_client_->service_is_ready();
      if (!rsp_ready || !viewer_ready || !camera_ready) {
        if (std::chrono::duration<double>(std::chrono::steady_clock::now() - started_).count() >
            startup_timeout_) {
          sim_->fault(!camera_ready ? "camera renderer readiness timeout" :
              (viewer_ready ? "robot_state_publisher readiness timeout" : "viewer readiness timeout"));
          timer_->cancel();
          publishStatus();
          saveManifest("startup_timeout");
        }
        return;
      }
      if (count_publishers("/clock") > 1) {
        sim_->fault("multiple /clock publishers");
        timer_->cancel();
        publishStatus();
        return;
      }
      ready_ = true;
      sim_->pause(paused_);
      pace_time_ = std::chrono::steady_clock::now();
      pace_stamp_ = sim_->snapshot().stamp;
    }
    try {
      // Humble can repeatedly select an always-ready simulation timer before
      // the slower timers. Keep wall-clock output alive even with RTF=0 or an
      // overloaded paced simulation; do not cap or skip physics steps.
      publishPeriodicStatus();
      publishSceneState();
      unsigned steps = 1;
      const auto &cfg = sim_->config();
      if (cfg.real_time_factor > 0 && sim_->state() == litesim3d::State::Running) {
        const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - pace_time_).count();
        const uint64_t target = pace_stamp_ + uint64_t(elapsed * cfg.real_time_factor * 1e9);
        steps = target > sim_->snapshot().stamp
            ? unsigned(std::min<uint64_t>(8, (target - sim_->snapshot().stamp) / cfg.step_ns)) : 0;
      }
      for (unsigned i = 0; i < steps; ++i) publish(sim_->tick());
    } catch (const std::exception &e) {
      if (!rclcpp::ok()) return;
      RCLCPP_ERROR(get_logger(), "simulation fault: %s", e.what());
      sim_->fault(e.what());
      timer_->cancel();
      saveManifest("fault");
    }
  }
  void publishPeriodicStatus() {
    if (!rclcpp::ok() || std::chrono::steady_clock::now() < next_status_) return;
    publishStatus();
    if (std::any_of(sim_->scene().visual.begin(), sim_->scene().visual.end(),
                    [](const litesim3d::Geometry &g) { return bool(g.motion); }))
      publishScene(false);
  }
  void publishStatus() {
    if (!sim_ || !rclcpp::ok())
      return;
    next_status_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
    if (count_publishers("/clock") > 1)
      sim_->fault("multiple /clock publishers");
    auto data = sim_->diagnostics();
    const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    const auto actual = rate_.sample(sim_->run_id, sim_->snapshot().stamp, wall);
    const bool paused = sim_->state() == litesim3d::State::Paused;
    data["actual_real_time_factor"] = paused ? litesim3d::Json(0.0) :
        (actual ? litesim3d::Json(*actual) : litesim3d::Json(nullptr));
    data["target_real_time_factor"] = sim_->config().real_time_factor;
    data["collision_budget_enabled"] = sim_->config().collision_budget_enabled;
    std_msgs::msg::String statistics;
    statistics.data = litesim3d::Json({{"schema_version", 1}, {"run_id", sim_->run_id},
      {"stamp_ns", sim_->snapshot().stamp}, {"state", data["state"]},
      {"actual_real_time_factor", data["actual_real_time_factor"]},
      {"target_real_time_factor", sim_->config().real_time_factor},
      {"collision_budget_enabled", sim_->config().collision_budget_enabled}}).dump();
    stats_->publish(statistics);
    data["ready"] = ready_;
    data["qos_conflicts"] = qos_conflicts_;
    data["dropped_frames"] = dropped_frames_;
    data["published_clouds"] = published_clouds_;
    data["published_imus"] = published_imus_;
    data["published_camera_frames"] = published_camera_frames_;
    // Session reset waits for real publisher-side DDS matches before releasing clients.
    auto matches = nlohmann::json::object();
    auto count_matches = [&matches](const auto &publisher) {
      matches[publisher->get_topic_name()] = publisher->get_subscription_count();
    };
    count_matches(clock_);
    count_matches(odom_);
    count_matches(joints_);
    livox_->matches(matches);
    data["livox_output"] = livox_->description();
    for (const auto &[topic, publisher] : clouds_) count_matches(publisher);
    for (const auto &[topic, publisher] : imus_) count_matches(publisher);
    for (const auto &[topic, publisher] : images_) count_matches(publisher);
    data["subscription_matches"] = std::move(matches);
    data["internal_missed_scans"] = 0; // scheduler executes every model event; wall lateness slows simulation
    data["sensor_queue_depth"] = 5;
    data["sensor_queue_policy"] = sensor_reliable_ ? "keep_last/reliable" : "keep_last/best_effort";
    data["dds_delivery_drops"] = "unknown; measure at consumer by scan timestamps";
    auto percentile = [](const std::deque<double> &samples, double p) {
      if (samples.empty()) return 0.0;
      std::vector<double> sorted(samples.begin(), samples.end());
      std::sort(sorted.begin(), sorted.end());
      return sorted[std::min(sorted.size() - 1, size_t(std::ceil(p * sorted.size()) - 1))];
    };
    data["compute_p99_ms"] = percentile(compute_ms_, .99);
    data["bridge_p99_ms"] = percentile(bridge_ms_, .99);
    data["timing_window_frames"] = compute_ms_.size();
    diagnostic_msgs::msg::DiagnosticArray msg;
    msg.header.stamp = stamp(sim_->snapshot().stamp);
    diagnostic_msgs::msg::DiagnosticStatus s;
    s.name = "litesim3d/" + sim_->config().robot_id;
    s.hardware_id = sim_->run_id;
    s.level = sim_->state() == litesim3d::State::Fault
                  ? 2
                  : ((qos_conflicts_ || dropped_frames_ || data["command_timeout"].get<bool>() ||
                      data["rejected_commands"].get<uint64_t>())
                         ? 1
                         : 0);
    s.message = data["last_error"].get<std::string>();
    if (s.message.empty())
      s.message = data["state"].get<std::string>();
    for (auto it = data.begin(); it != data.end(); ++it) {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = it.key();
      kv.value = it.value().is_string() ? it.value().get<std::string>() : it.value().dump();
      s.values.push_back(kv);
    }
    msg.status.push_back(s);
    status_->publish(msg);
    if (failed()) {
      if (!fault_time_)
        fault_time_ = std::chrono::steady_clock::now();
      if (std::chrono::steady_clock::now() - *fault_time_ > std::chrono::milliseconds(400)) {
        saveManifest("fault");
        rclcpp::shutdown();
      }
    }
  }
  void publishScene(bool initialize = true) {
    visualization_msgs::msg::MarkerArray all;
    int id = 0;
    if (initialize) scene_inverse_ = sim_->snapshot().world_base.inverse();
    for (auto &g : sim_->scene().visual) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = prefix_ + "odom";
      m.header.stamp = stamp(sim_->snapshot().stamp);
      m.ns = "scene";
      m.id = id++;
      m.action = m.ADD;
      m.pose = poseMsg(scene_inverse_ * g.at(sim_->snapshot().stamp));
      m.color.r = .55;
      m.color.g = .65;
      m.color.b = .75;
      m.color.a = .8;
      const auto &s = *g.shape;
      if (s.kind == litesim3d::Shape::Kind::Box) {
        m.type = m.CUBE;
        m.scale.x = s.size.x();
        m.scale.y = s.size.y();
        m.scale.z = s.size.z();
      } else if (s.kind == litesim3d::Shape::Kind::Sphere) {
        m.type = m.SPHERE;
        m.scale.x = m.scale.y = m.scale.z = 2 * s.size.x();
      } else if (s.kind == litesim3d::Shape::Kind::Cylinder) {
        m.type = m.CYLINDER;
        m.scale.x = m.scale.y = 2 * s.size.x();
        m.scale.z = s.size.z();
      } else {
        m.type = m.TRIANGLE_LIST;
        m.scale.x = m.scale.y = m.scale.z = 1;
        for (auto &t : s.triangles)
          for (auto &v : {t.a, t.b, t.c}) {
            geometry_msgs::msg::Point p;
            p.x = v.x();
            p.y = v.y();
            p.z = v.z();
            m.points.push_back(p);
          }
      }
      all.markers.push_back(m);
    }
    markers_->publish(all);
  }
  void saveManifest(const std::string &reason) {
    if (manifest_path_.empty())
      return;
    auto data = sim_->manifest(reason);
    if (!scene_description_.is_null()) data["scene_assets"] = scene_description_;
    data["ros_distro"] = "humble";
    data["rmw"] = rmw_get_implementation_identifier();
    data["bridge_source_sha256"] = LWS_BRIDGE_SHA256;
    data["frame_prefix"] = prefix_;
    data["livox_output"] = livox_->description();
    std::ofstream out(manifest_path_);
    litesim3d::require(bool(out), "cannot write manifest: " + manifest_path_);
    out << data.dump(2) << '\n';
  }
  litesim3d::RealTimeRate rate_;
  litesim3d::Json scene_description_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stats_, scene_description_pub_, scene_state_;
  rclcpp::TimerBase::SharedPtr scene_timer_;
  std::unique_ptr<litesim3d::Simulator> sim_;
  std::unique_ptr<litesim3d_ros::LivoxOutputs> livox_;
  litesim3d::Pose scene_inverse_ = litesim3d::Pose::Identity();
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  bool publish_tf_ = true, wait_rsp_ = true, paused_ = false, ready_ = false,
       sensor_reliable_ = false;
  double startup_timeout_ = 10;
  std::string prefix_, manifest_path_, rsp_name_, viewer_ready_file_;
  uint64_t qos_conflicts_ = 0, dropped_frames_ = 0;
  uint64_t published_clouds_ = 0, published_imus_ = 0;
  uint64_t published_camera_frames_ = 0;
  rclcpp::Node::SharedPtr camera_client_node_;
  rclcpp::Client<litesim3d_interfaces::srv::RenderCameras>::SharedPtr camera_client_;
  rclcpp::executors::SingleThreadedExecutor camera_executor_;
  std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> images_;
  std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> camera_info_;
  std::deque<double> compute_ms_, bridge_ms_;
  int drop_frames_ = 0;
  size_t contact_index_ = 0;
  std::set<std::string> odom_sources_;
  std::optional<std::chrono::steady_clock::time_point> fault_time_;
  std::chrono::steady_clock::time_point started_;
  std::chrono::steady_clock::time_point pace_time_;
  std::chrono::steady_clock::time_point next_status_, next_scene_state_;
  uint64_t pace_stamp_ = 1;
  nav_msgs::msg::Path trajectory_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_;
  std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> clouds_;
  std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr> imus_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_, truth_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_;
  rclcpp::Publisher<litesim3d_interfaces::msg::Contact>::SharedPtr contacts_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joints_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr description_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr stamped_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_watch_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr pause_service_;
  rclcpp::Service<litesim3d_interfaces::srv::Step>::SharedPtr step_service_;
  rclcpp::Service<litesim3d_interfaces::srv::SetPose>::SharedPtr pose_service_;
  rclcpp::TimerBase::SharedPtr timer_, status_timer_;
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto node = std::make_shared<SimNode>();
    rclcpp::spin(node);
    result = node->failed() ? 1 : 0;
    node->finish();
  } catch (const std::exception &e) {
    std::cerr << "litesim3d: " << e.what() << '\n';
    result = 1;
    // Publish startup failures on the same diagnostics channel before exiting.
    // No simulator world is committed when its constructor fails validation.
    if (rclcpp::ok()) {
      auto reporter = std::make_shared<rclcpp::Node>("litesim3d_startup_error");
      auto publisher = reporter->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
          "/sim/status", rclcpp::QoS(1).reliable().transient_local());
      diagnostic_msgs::msg::DiagnosticArray report;
      report.header.stamp = stamp(1);
      diagnostic_msgs::msg::DiagnosticStatus error;
      error.name = "litesim3d/startup";
      error.hardware_id =
          "startup-" + std::to_string(std::chrono::system_clock::now().time_since_epoch().count());
      error.level = 2;
      error.message = e.what();
      report.status.push_back(error);
      for (int i = 0; i < 10; ++i) {
        publisher->publish(report);
        rclcpp::spin_some(reporter);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
    }
  }
  if (rclcpp::ok())
    rclcpp::shutdown();
  return result;
}
