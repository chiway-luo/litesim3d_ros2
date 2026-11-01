// World bridge units. The World dependency is an inert recording fake; every
// callback is delivered directly and every output is an in-memory value sink.
// No physics, executor loop, launched process, ROS peer or transport round trip runs.
#define main litesim_world_process_entry
#include "../src/sim_world_node.cpp"
#undef main
#include <gtest/gtest.h>
#include <any>
#include <sstream>
#include <typeindex>
#include <unistd.h>

struct WorldNodeUnitAccess {
  template<class Message>
  static void deliver(const std::shared_ptr<rclcpp::Subscription<Message>> &subscription,
                      const Message &value, const rclcpp::MessageInfo &info = {}) {
    std::shared_ptr<void> erased = std::make_shared<Message>(value);
    subscription->handle_message(erased, info);
  }
  static void twist(WorldNode &n, size_t i, const geometry_msgs::msg::Twist &m) { deliver(n.channels_.at(i).command, m); }
  static void stamped(WorldNode &n, size_t i, const geometry_msgs::msg::TwistStamped &m) { deliver(n.channels_.at(i).stamped_command, m); }
  static void tf(WorldNode &n, const tf2_msgs::msg::TFMessage &m, uint8_t source) {
    rmw_message_info_t rmw{}; rmw.publisher_gid.data[0] = source;
    deliver(n.tf_watch_, m, rclcpp::MessageInfo(rmw));
  }
  static auto pause(WorldNode &n, bool value) {
    auto q = std::make_shared<std_srvs::srv::SetBool::Request>(); q->data = value;
    auto a = std::make_shared<std_srvs::srv::SetBool::Response>(); n.pause_handler_(q, a); return *a;
  }
  static auto step(WorldNode &n, const std::string &id, uint32_t steps) {
    auto q = std::make_shared<litesim3d_interfaces::srv::Step::Request>(); q->request_id = id; q->steps = steps;
    auto a = std::make_shared<litesim3d_interfaces::srv::Step::Response>(); n.step_handler_(q, a); return *a;
  }
  static void update(WorldNode &n) { n.timer_->execute_callback(); }
  static void statusTimer(WorldNode &n) { n.status_timer_->execute_callback(); }
  static void status(WorldNode &n) { n.publishStatus(); }
  static void publish(WorldNode &n, const std::vector<std::vector<litesim3d::Event>> &events) { n.publish(events); }
  static void state(WorldNode &n, size_t i, const litesim3d::Snapshot &s) { n.state(i, s); }
  static void fault(WorldNode &n, const std::string &reason) { n.fail(reason); }
  static void save(WorldNode &n, const std::string &reason) { n.save(reason); }
  static auto &hooks(WorldNode &n) { return n.hooks_; }
  static auto &channels(WorldNode &n) { return n.channels_; }
  static auto &backend(WorldNode &n) { return *n.world_; }
  static auto &compute(WorldNode &n) { return n.compute_; }
  static auto &bridge(WorldNode &n) { return n.bridge_; }
  static auto anchor(WorldNode &n) { return n.anchor_; }
  static auto anchorStamp(WorldNode &n) { return n.anchor_stamp_; }
  static void ready(WorldNode &n, bool value) { n.ready_ = value; }
  static void output(WorldNode &n, const std::string &value) { n.output_ = value; }
  static auto lifetimes(WorldNode &n) {
    std::vector<std::pair<std::string, std::weak_ptr<void>>> result;
    const auto observe = [&](const std::string &name, const auto &entity) {
      if (entity) result.emplace_back(name, entity);
    };
    observe("node base", n.get_node_base_interface());
    observe("node graph", n.get_node_graph_interface());
    observe("statistics publisher", n.stats_);
    observe("clock publisher", n.clock_);
    observe("status publisher", n.status_);
    observe("TF subscription", n.tf_watch_);
    observe("pause service", n.pause_);
    observe("step service", n.step_);
    observe("update timer", n.timer_);
    observe("status timer", n.status_timer_);
    for (const auto &c : n.channels_) {
      observe(c.prefix + "odometry publisher", c.odom);
      observe(c.prefix + "ground-truth publisher", c.truth);
      observe(c.prefix + "joint-state publisher", c.joints);
      observe(c.prefix + "description publisher", c.description);
      observe(c.prefix + "contact publisher", c.contacts);
      observe(c.prefix + "twist subscription", c.command);
      observe(c.prefix + "stamped subscription", c.stamped_command);
      for (const auto &[topic, publisher] : c.clouds) observe(c.prefix + topic, publisher);
      for (const auto &[topic, publisher] : c.imus) observe(c.prefix + topic, publisher);
    }
    return result;
  }
};
namespace litesim3d_ros {
struct JointControlUnitAccess {
  static void deliver(JointControl &control, const JointControl::Command &message) {
    std::shared_ptr<void> erased = std::make_shared<JointControl::Command>(message);
    control.command_->handle_message(erased, rclcpp::MessageInfo{});
  }
};
}
namespace {
using Access = WorldNodeUnitAccess;
using Json = litesim3d::Json;
using Clock = std::chrono::steady_clock;
struct RobotFixture {
  litesim3d::Config cfg;
  litesim3d::Model mdl;
  litesim3d::Scene scn;
  litesim3d::Snapshot snap;
  litesim3d::State operational = litesim3d::State::Ready;
  Json diagnostic{{"state", "ready"}, {"command_timeout", false}, {"rejected_commands", 0}, {"last_error", ""}};
  std::vector<litesim3d::Contact> retained;
  uint64_t contact_count = 0;
  std::string run_id;
  std::function<void()> before_diagnostics;
  const auto &config() const { return cfg; }
  const auto &model() const { return mdl; }
  const auto &scene() const { return scn; }
  const auto &snapshot() const { return snap; }
  auto state() const { return operational; }
  Json diagnostics() const { if (before_diagnostics) before_diagnostics(); return diagnostic; }
  const auto &contacts() const { return retained; }
  uint64_t contactCount() const { return contact_count; }
};
struct RecordingWorld {
  static std::unique_ptr<RecordingWorld> next;
  static std::filesystem::path loaded;
  static std::unique_ptr<RecordingWorld> load(const std::filesystem::path &path) {
    loaded = path; return std::move(next);
  }
  std::vector<RobotFixture> robots;
  std::string run_id = "world-unit-session";
  bool accept = true;
  std::vector<std::pair<size_t, litesim3d::Command>> commands;
  std::vector<std::pair<size_t, litesim3d::JointCommand>> joint_commands;
  std::vector<bool> pauses;
  std::vector<std::string> faults;
  size_t ticks = 0, exits = 0;
  std::vector<std::pair<std::string, uint32_t>> steps;
  std::vector<std::vector<litesim3d::Event>> tick_events, step_events;
  litesim3d::StepResult step_result{true, 0, "stepped", 456};
  std::function<void()> on_pause, on_tick, on_step, on_manifest, on_exit;
  std::shared_ptr<void> lifetime = std::make_shared<int>(0);
  explicit RecordingWorld(size_t count = 2) : robots(count) {
    for (size_t i = 0; i < count; ++i) {
      auto &r = robots[i]; r.cfg.robot_id = "robot_" + std::to_string(i);
      r.run_id = "robot-session-" + std::to_string(i);
      r.mdl.root = r.cfg.base_link; r.mdl.xml = "<robot name='unit'/>";
      r.cfg.real_time_factor = 0;
      r.snap.stamp = 1000000002;
      litesim3d::Joint fixed; fixed.type = "fixed"; r.mdl.joints["fixed"] = fixed;
      litesim3d::Joint revolute; revolute.type = "revolute";
      r.mdl.joints["joint_a"] = revolute; r.mdl.joints["joint_b"] = revolute;
      r.snap.joints = {{"fixed", 8}, {"joint_a", 1.25}, {"joint_b", -.5}};
      r.snap.joint_velocities = {{"joint_a", .75}};
    }
  }
  size_t size() const { return robots.size(); }
  const RobotFixture &robot(size_t i) const { return robots.at(i); }
  bool command(size_t i, const litesim3d::Command &value) { commands.emplace_back(i, value); return accept; }
  bool jointCommand(size_t i, const litesim3d::JointCommand &value) { joint_commands.emplace_back(i, value); return accept; }
  void pause(bool value) { if (on_pause) on_pause(); pauses.push_back(value); }
  void fault(const std::string &value) { faults.push_back(value); }
  void exit() { if (on_exit) on_exit(); ++exits; }
  auto tick(const litesim3d::World::EventSink &sink) {
    ++ticks; if (on_tick) on_tick();
    for (size_t i = 0; i < robots.size(); ++i)
      sink(i, i < tick_events.size() ? tick_events[i] : std::vector<litesim3d::Event>{});
    return tick_events;
  }
  auto singleStep(const std::string &id, uint32_t count, std::vector<std::vector<litesim3d::Event>> &events) {
    steps.emplace_back(id, count); if (on_step) on_step(); events = step_events; return step_result;
  }
  Json manifest() const {
    if (on_manifest) on_manifest();
    Json result{{"schema_version", 1}, {"run_id", run_id}, {"robots", Json::array()}};
    for (const auto &r : robots) result["robots"].push_back({{"robot_id", r.cfg.robot_id}});
    return result;
  }
};
std::unique_ptr<RecordingWorld> RecordingWorld::next;
std::filesystem::path RecordingWorld::loaded;
struct Publication { std::string topic; std::type_index type; std::any message; };
class Serializer final : public litesim3d_ros::LidarOutputPlugin {
public:
  size_t published = 0;
  Json description_{{"serializer", "recording"}};
  void initialize(rclcpp::Node &, const litesim3d::Config &, const std::string &,
      const std::string &, const Json &, const rclcpp::PublisherOptions &) override {}
  bool handles(const std::string &topic) const override { return topic == "handled"; }
  bool publish(const litesim3d::Cloud &cloud) override { if (!handles(cloud.topic)) return false; ++published; return true; }
  void matches(Json &matches) const override { matches["/fake_serializer"] = 7; }
  const Json &description() const override { return description_; }
};
class PrivateContext final : public rclcpp::Context {
public:
  void finalize() {
    if (is_valid()) shutdown("unit teardown");
    // Humble's Context owns its GraphListener subcontext, whose interrupt
    // GuardCondition owns the Context. Shutdown invalidates ROS but leaves
    // that cycle intact. Only after every node/entity is destroyed may this
    // test-owned context perform the same final cleanup as its destructor.
    // clean_up() is idempotent, so the eventual destructor remains safe.
    clean_up();
  }
};
class WorldNodeUnits : public ::testing::Test {
protected:
  void startContext() { context = std::make_shared<PrivateContext>(); context->init(0, nullptr); }
  void releaseContext() {
    ASSERT_FALSE(node);
    for (const auto &created : created_nodes)
      ASSERT_TRUE(created.expired()) << "a WorldNode outlived its private context";
    for (const auto &[name, entity] : created_entities)
      ASSERT_TRUE(entity.expired()) << name << " outlived its private context";
    if (!context) return;
    std::weak_ptr<rclcpp::Context> released = context;
    context->finalize();
    context.reset();
    EXPECT_TRUE(released.expired()) << "private Context retained after final cleanup";
  }
  void SetUp() override { startContext(); }
  void TearDown() override { node.reset(); RecordingWorld::next.reset(); releaseContext(); }
  void record(const std::string &topic, const std::type_info &type, const void *value) {
#define RECORD(Type) if (type == typeid(Type)) { published.push_back({topic, type, *static_cast<const Type *>(value)}); return; }
    RECORD(std_msgs::msg::String)
    RECORD(rosgraph_msgs::msg::Clock)
    RECORD(diagnostic_msgs::msg::DiagnosticArray)
    RECORD(sensor_msgs::msg::JointState)
    RECORD(geometry_msgs::msg::TransformStamped)
    RECORD(nav_msgs::msg::Odometry)
    RECORD(sensor_msgs::msg::PointCloud2)
    RECORD(sensor_msgs::msg::Imu)
    RECORD(litesim3d_interfaces::msg::Contact)
    RECORD(litesim3d_interfaces::msg::JointFeedback)
#undef RECORD
    ADD_FAILURE() << "unexpected publication type " << type.name();
  }
  void build(std::vector<rclcpp::Parameter> parameters = {},
             std::function<void(RecordingWorld &)> configure = {}, bool capture = true) {
    auto engine = std::make_unique<RecordingWorld>(); world = engine.get(); if (configure) configure(*engine);
    RecordingWorld::next = std::move(engine);
    WorldNodeHooks hooks;
    hooks.load = litesim3d_ros::loadWorldBackend<RecordingWorld>;
    hooks.steady_now = [&] { return now; }; hooks.wall_now = [&] { return wall; };
    hooks.ok = [&] { return live; }; hooks.shutdown = [&] { ++shutdowns; };
    hooks.publisher_count = [&](const std::string &topic) { EXPECT_EQ(topic, "/clock"); return clock_publishers; };
    if (capture) hooks.publish = [&](const auto &topic, const auto &type, const void *value) { record(topic, type, value); };
    hooks.lidar_factory = [&] { return serializer; };
    rclcpp::NodeOptions options;
    options.context(context).parameter_overrides(parameters).enable_rosout(false)
        .start_parameter_services(false).start_parameter_event_publisher(false);
    node = std::make_shared<WorldNode>(options, std::move(hooks));
    created_nodes.push_back(node);
    const auto entities = Access::lifetimes(*node);
    created_entities.insert(created_entities.end(), entities.begin(), entities.end());
  }
  template<class Message> Message last(const std::string &topic) const {
    for (auto i = published.rbegin(); i != published.rend(); ++i)
      if (i->topic == topic && i->type == typeid(Message)) return std::any_cast<Message>(i->message);
    ADD_FAILURE() << "no recorded " << typeid(Message).name() << " on " << topic; return Message{};
  }
  template<class Message> size_t count(const std::string &topic) const {
    return std::count_if(published.begin(), published.end(), [&](const auto &p) { return p.topic == topic && p.type == typeid(Message); });
  }
  static std::string key(const diagnostic_msgs::msg::DiagnosticStatus &status, const std::string &name) {
    for (const auto &kv : status.values) if (kv.key == name) return kv.value;
    ADD_FAILURE() << "missing diagnostic " << name; return {};
  }
  static litesim3d::Cloud cloud(const std::string &topic = "points") {
    litesim3d::Cloud p; p.topic = topic; p.frame = "lidar"; p.stamp = 3000000004;
    p.width = 2; p.height = 1; p.row_step = 48; p.dense = true; p.data.resize(48);
    for (size_t i = 0; i < p.data.size(); ++i) p.data[i] = uint8_t(i);
    p.compute_ms = 2.5; p.compute_end_ns = 998000000; return p;
  }
  static void sensors(RecordingWorld &world) {
    for (auto &r : world.robots) {
      litesim3d::LidarConfig l; l.topic = "points"; l.publish_range_boundary = true; r.cfg.lidars.push_back(l);
      litesim3d::ImuConfig imu; imu.topic = "imu"; r.cfg.imus.push_back(imu);
    }
  }
  std::shared_ptr<PrivateContext> context;
  std::shared_ptr<WorldNode> node;
  std::vector<std::weak_ptr<WorldNode>> created_nodes;
  std::vector<std::pair<std::string, std::weak_ptr<void>>> created_entities;
  RecordingWorld *world = nullptr;
  Clock::time_point now{std::chrono::seconds(1)};
  uint64_t wall = 5000000000;
  bool live = true;
  size_t shutdowns = 0, clock_publishers = 1;
  std::vector<Publication> published;
  std::shared_ptr<Serializer> serializer = std::make_shared<Serializer>();
};
}

TEST_F(WorldNodeUnits, NodeDestructionReleasesRosEntitiesAndBackendBeforePrivateContextFinalization) {
  build({}, sensors);
  const auto entities = Access::lifetimes(*node);
  ASSERT_GT(entities.size(), 20U);
  std::weak_ptr<void> backend = world->lifetime;
  node.reset();
  EXPECT_TRUE(backend.expired());
  for (const auto &[name, entity] : entities)
    EXPECT_TRUE(entity.expired()) << name << " retained after WorldNode destruction";
  EXPECT_TRUE(context->is_valid());
  std::weak_ptr<rcl_context_t> rcl_context = context->get_rcl_context();
  context->finalize();
  EXPECT_FALSE(context->is_valid());
  EXPECT_TRUE(rcl_context.expired());
  EXPECT_NO_THROW(context->finalize());
  releaseContext();
}

TEST_F(WorldNodeUnits, ConstructorUnwindReleasesBackendAndPrivateContextBeforeAFreshSession) {
  std::weak_ptr<void> backend;
  EXPECT_THROW(build({rclcpp::Parameter("cmd_mode", "invalid")},
      [&](auto &engine) { backend = engine.lifetime; }), std::runtime_error);
  EXPECT_FALSE(node);
  EXPECT_TRUE(backend.expired());
  releaseContext();
  startContext();
  build();
  EXPECT_EQ(world->ticks, 0U);
  EXPECT_TRUE(context->is_valid());
}

TEST_F(WorldNodeUnits, ConstructorRoutesDescriptionsInitialJointsClockAndTransformsWithoutAdvancingCore) {
  build({rclcpp::Parameter("config", "fixture-world.yml")});
  EXPECT_EQ(RecordingWorld::loaded, "fixture-world.yml"); EXPECT_EQ(world->ticks, 0U); EXPECT_TRUE(world->pauses.empty());
  EXPECT_EQ(last<std_msgs::msg::String>("/robot_1/robot_description").data, "<robot name='unit'/>");
  const auto clock = last<rosgraph_msgs::msg::Clock>("/clock"); EXPECT_EQ(clock.clock.sec, 1); EXPECT_EQ(clock.clock.nanosec, 2U);
  const auto joints = last<sensor_msgs::msg::JointState>("/robot_0/joint_states");
  EXPECT_EQ(joints.name, (std::vector<std::string>{"joint_a", "joint_b"}));
  EXPECT_EQ(joints.position, (std::vector<double>{1.25, -.5})); EXPECT_EQ(joints.velocity, (std::vector<double>{.75, 0}));
  const auto tf = last<geometry_msgs::msg::TransformStamped>("/tf");
  EXPECT_EQ(tf.header.frame_id, "robot_1/odom"); EXPECT_EQ(tf.child_frame_id, "robot_1/base_link"); EXPECT_EQ(tf.transform.rotation.w, 1);
  auto &backend = Access::backend(*node); EXPECT_EQ(backend.runId(), "world-unit-session"); EXPECT_EQ(backend.size(), 2U);
  EXPECT_EQ(&backend.robot(0).scene(), &world->robots[0].scn); EXPECT_EQ(backend.robot(0).runId(), world->robots[0].run_id);
  EXPECT_THROW(backend.robot(2), std::out_of_range);
}

TEST_F(WorldNodeUnits, ConstructorRejectsInvalidModeUnknownAndDuplicateExternalTfRobots) {
  EXPECT_THROW(build({rclcpp::Parameter("cmd_mode", "neither")}), std::runtime_error);
  EXPECT_THROW(build({rclcpp::Parameter("external_odom_tf_robots", std::vector<std::string>{"missing"})}), std::runtime_error);
  EXPECT_THROW(build({rclcpp::Parameter("external_odom_tf_robots", std::vector<std::string>{"robot_0", "robot_0"})}), std::runtime_error);
}

TEST_F(WorldNodeUnits, ExternalTfSelectionAndDisabledTfDoNotSuppressJointStates) {
  build({rclcpp::Parameter("external_odom_tf_robots", std::vector<std::string>{"robot_1"})});
  EXPECT_TRUE(Access::channels(*node)[0].publish_tf); EXPECT_FALSE(Access::channels(*node)[1].publish_tf);
  EXPECT_EQ(count<geometry_msgs::msg::TransformStamped>("/tf"), 1U);
  EXPECT_EQ(count<sensor_msgs::msg::JointState>("/robot_1/joint_states"), 1U);
  node.reset(); published.clear(); build({rclcpp::Parameter("publish_odom_tf", false)});
  EXPECT_EQ(count<geometry_msgs::msg::TransformStamped>("/tf"), 0U);
}

TEST_F(WorldNodeUnits, TwistCallbacksRouteEveryAxisToTheSelectedRobotWithNoStampOrFrame) {
  build(); geometry_msgs::msg::Twist twist;
  twist.linear.x = 1; twist.linear.y = 2; twist.linear.z = 3;
  twist.angular.x = 4; twist.angular.y = 5; twist.angular.z = 6;
  Access::twist(*node, 1, twist); ASSERT_EQ(world->commands.size(), 1U);
  const auto &[i, c] = world->commands.front(); EXPECT_EQ(i, 1U); EXPECT_EQ(c.v, 1); EXPECT_EQ(c.lateral, 2); EXPECT_EQ(c.vertical, 3);
  EXPECT_EQ(c.roll_rate, 4); EXPECT_EQ(c.pitch_rate, 5); EXPECT_EQ(c.w, 6); EXPECT_FALSE(c.stamp); EXPECT_TRUE(c.frame.empty());
}

TEST_F(WorldNodeUnits, StampedCommandsValidateNanosecondsAndRobotPrefixBeforeCoreAcceptance) {
  build({rclcpp::Parameter("cmd_mode", "stamped")});
  geometry_msgs::msg::TwistStamped m; m.header.stamp.sec = 7; m.header.stamp.nanosec = 8;
  m.header.frame_id = "robot_1/base_link"; m.twist.linear.x = .25;
  Access::stamped(*node, 1, m); ASSERT_EQ(world->commands.size(), 1U);
  EXPECT_EQ(world->commands.back().second.frame, "base_link"); EXPECT_EQ(*world->commands.back().second.stamp, 7000000008U);
  for (const auto &frame : {std::string("base_link"), std::string("robot_0/base_link"), std::string("")}) {
    m.header.frame_id = frame; Access::stamped(*node, 1, m);
    EXPECT_EQ(world->commands.back().second.frame, "invalid prefixed command frame: " + frame);
  }
  m.header.stamp.sec = -1; Access::stamped(*node, 0, m); EXPECT_EQ(*world->commands.back().second.stamp, UINT64_MAX);
  m.header.stamp.sec = 1; m.header.stamp.nanosec = 1000000000; Access::stamped(*node, 0, m);
  EXPECT_EQ(*world->commands.back().second.stamp, UINT64_MAX);
}

TEST_F(WorldNodeUnits, JointControlledRobotUsesJointCommandBoundaryAndSeparateOdometryTopic) {
  build({}, [](auto &w) { w.robots[1].cfg.control_mode = "ros2_control"; w.robots[1].cfg.drive.model = "multicopter"; });
  const auto &c = Access::channels(*node)[1]; ASSERT_TRUE(c.joint_control); EXPECT_FALSE(c.joints); EXPECT_FALSE(c.command); EXPECT_FALSE(c.stamped_command);
  EXPECT_EQ(std::string(c.odom->get_topic_name()), "/robot_1/sim/odometry");
  const auto feedback = last<litesim3d_interfaces::msg::JointFeedback>("/robot_1/sim/joint_feedback");
  EXPECT_EQ(feedback.run_id, world->robots[1].run_id); EXPECT_TRUE(feedback.operational);
  EXPECT_TRUE(feedback.body_command_supported); EXPECT_EQ(feedback.state.name, (std::vector<std::string>{"joint_a", "joint_b"}));
  litesim3d_ros::JointControl::Command msg; msg.run_id = world->robots[1].run_id;
  msg.name = {"joint_a"}; msg.mode = {msg.POSITION}; msg.command = {.75};
  litesim3d_ros::JointControlUnitAccess::deliver(*c.joint_control, msg);
  ASSERT_EQ(world->joint_commands.size(), 1U); EXPECT_EQ(world->joint_commands[0].first, 1U);
  EXPECT_EQ(world->joint_commands[0].second.joints.at("joint_a").value, .75);
}

TEST_F(WorldNodeUnits, PauseSuccessResetsPacingAnchorAndFailureReturnsCoreErrorWithoutMutation) {
  build(); now += std::chrono::seconds(2); world->robots[0].snap.stamp = 42;
  auto response = Access::pause(*node, true); EXPECT_TRUE(response.success); EXPECT_EQ(world->pauses, std::vector<bool>{true});
  EXPECT_EQ(Access::anchor(*node), now); EXPECT_EQ(Access::anchorStamp(*node), 42U);
  world->on_pause = [] { throw std::runtime_error("pause rejected"); };
  response = Access::pause(*node, false); EXPECT_FALSE(response.success); EXPECT_EQ(response.message, "pause rejected");
  EXPECT_EQ(world->pauses.size(), 1U); EXPECT_FALSE(node->failed());
  Access::update(*node); EXPECT_EQ(world->ticks, 0U);
}

TEST_F(WorldNodeUnits, StepBeforeReadyRefusesAndSuccessPreservesResultAndRoutesCoreEvents) {
  build(); published.clear(); auto response = Access::step(*node, "before", 3);
  EXPECT_FALSE(response.success); EXPECT_EQ(response.code, 1); EXPECT_EQ(response.message, "not ready");
  EXPECT_EQ(response.time_ns, world->robots[0].snap.stamp); EXPECT_TRUE(world->steps.empty()); EXPECT_EQ(count<rosgraph_msgs::msg::Clock>("/clock"), 0U);
  Access::ready(*node, true); world->step_events.resize(1); litesim3d::Event e; e.odom = true; world->step_events[0].push_back(e);
  response = Access::step(*node, "once", 4); EXPECT_TRUE(response.success); EXPECT_EQ(response.code, 0); EXPECT_EQ(response.message, "stepped"); EXPECT_EQ(response.time_ns, 456U);
  EXPECT_EQ(world->steps, (std::vector<std::pair<std::string, uint32_t>>{{"once", 4}}));
  EXPECT_EQ(count<nav_msgs::msg::Odometry>("/robot_0/odom"), 1U); EXPECT_EQ(count<rosgraph_msgs::msg::Clock>("/clock"), 1U);
  EXPECT_GE(count<sensor_msgs::msg::JointState>("/robot_1/joint_states"), 1U);
  published.clear(); world->step_result = {false, 9, "duplicate mismatch", 123}; response = Access::step(*node, "once", 5);
  EXPECT_FALSE(response.success); EXPECT_EQ(response.code, 9); EXPECT_EQ(response.message, "duplicate mismatch"); EXPECT_EQ(response.time_ns, 123U);
  EXPECT_EQ(count<rosgraph_msgs::msg::Clock>("/clock"), 0U); EXPECT_EQ(count<diagnostic_msgs::msg::DiagnosticArray>("/sim/status"), 1U);
}

TEST_F(WorldNodeUnits, StepExceptionFaultsAndShutsDownWithoutClaimingSuccess) {
  build(); Access::ready(*node, true); world->on_step = [] { throw std::runtime_error("step failure"); };
  const auto response = Access::step(*node, "failed", 1); EXPECT_FALSE(response.success); EXPECT_EQ(response.message, "step failure");
  EXPECT_TRUE(node->failed()); EXPECT_EQ(world->faults, std::vector<std::string>{"step failure"}); EXPECT_EQ(shutdowns, 1U);
  node->finish(); EXPECT_EQ(world->exits, 0U);
}

TEST_F(WorldNodeUnits, OdometryGroundTruthAndJointTransformPreserveSeparateFramesAndCovariances) {
  build({}, [](auto &w) { w.robots[0].cfg.pose_covariance.fill(3); w.robots[0].cfg.twist_covariance.fill(4); }); published.clear();
  litesim3d::Event e; e.odom = true; e.snapshot.stamp = 2000000003;
  e.snapshot.odom_base = litesim3d::pose({1, 2, 3}, {0, 0, .2});
  e.snapshot.world_base = litesim3d::pose({4, 5, 6}, {0, 0, -.3});
  e.snapshot.odom_root = litesim3d::pose({7, 8, 9}, {.1, .2, .3});
  e.snapshot.odom_v = 1; e.snapshot.odom_lateral = 2; e.snapshot.odom_vertical = 3; e.snapshot.odom_w = 4;
  e.snapshot.v = 5; e.snapshot.lateral = 6; e.snapshot.vertical = 7; e.snapshot.w = 8;
  Access::publish(*node, {{e}});
  const auto odom = last<nav_msgs::msg::Odometry>("/robot_0/odom"); const auto truth = last<nav_msgs::msg::Odometry>("/sim/ground_truth/robot_0/odom");
  EXPECT_EQ(odom.header.frame_id, "robot_0/odom"); EXPECT_EQ(odom.child_frame_id, "robot_0/base_link"); EXPECT_EQ(odom.header.stamp.sec, 2); EXPECT_EQ(odom.header.stamp.nanosec, 3U);
  EXPECT_EQ(odom.pose.pose.position.x, 1); EXPECT_EQ(odom.pose.pose.position.y, 2); EXPECT_EQ(odom.pose.pose.position.z, 3);
  EXPECT_NEAR(odom.pose.pose.orientation.z, std::sin(.1), 1e-12); EXPECT_NEAR(odom.pose.pose.orientation.w, std::cos(.1), 1e-12);
  EXPECT_EQ(odom.twist.twist.linear.x, 1); EXPECT_EQ(odom.twist.twist.linear.y, 2); EXPECT_EQ(odom.twist.twist.linear.z, 3); EXPECT_EQ(odom.twist.twist.angular.z, 4);
  EXPECT_EQ(odom.pose.covariance[13], 3); EXPECT_EQ(odom.twist.covariance[21], 4);
  EXPECT_EQ(truth.header.frame_id, "world"); EXPECT_EQ(truth.pose.pose.position.x, 4); EXPECT_EQ(truth.pose.pose.position.y, 5); EXPECT_EQ(truth.pose.pose.position.z, 6);
  EXPECT_EQ(truth.twist.twist.linear.x, 5); EXPECT_EQ(truth.twist.twist.linear.y, 6); EXPECT_EQ(truth.twist.twist.linear.z, 7); EXPECT_EQ(truth.twist.twist.angular.z, 8);
  EXPECT_TRUE(std::all_of(truth.pose.covariance.begin(), truth.pose.covariance.end(), [](double x) { return x == 0; }));
  EXPECT_TRUE(std::all_of(truth.twist.covariance.begin(), truth.twist.covariance.end(), [](double x) { return x == 0; }));
  bool saw_event_transform = false;
  for (const auto &p : published) if (p.type == typeid(geometry_msgs::msg::TransformStamped)) {
    const auto t = std::any_cast<geometry_msgs::msg::TransformStamped>(p.message);
    if (t.header.stamp.sec == 2) { saw_event_transform = true; EXPECT_EQ(t.transform.translation.x, 7); EXPECT_EQ(t.transform.translation.y, 8); EXPECT_EQ(t.transform.translation.z, 9); }
  }
  EXPECT_TRUE(saw_event_transform);
}

TEST_F(WorldNodeUnits, NativeCloudLayoutRangeBoundaryAndImuAreValueSerializedWithRobotPrefix) {
  build({}, sensors); published.clear(); litesim3d::Event e; auto p = cloud(); p.range_boundary = std::vector<uint8_t>(48, 9); e.clouds.push_back(p);
  litesim3d::ImuSample imu; imu.topic = "imu"; imu.frame = "imu_link"; imu.stamp = 4000000005;
  imu.acceleration = {1, 2, 3}; imu.angular_velocity = {4, 5, 6}; imu.orientation_covariance.fill(7); imu.acceleration_covariance.fill(8); imu.angular_velocity_covariance.fill(9); e.imus.push_back(imu);
  Access::publish(*node, {{e}});
  const auto measured = last<sensor_msgs::msg::PointCloud2>("/robot_0/points");
  EXPECT_EQ(measured.header.frame_id, "robot_0/lidar"); EXPECT_EQ(measured.header.stamp.sec, 3); EXPECT_EQ(measured.header.stamp.nanosec, 4U);
  EXPECT_EQ(measured.width, 2U); EXPECT_EQ(measured.height, 1U); EXPECT_EQ(measured.point_step, 24U); EXPECT_EQ(measured.row_step, 48U);
  EXPECT_TRUE(measured.is_dense); EXPECT_FALSE(measured.is_bigendian); EXPECT_EQ(measured.data, p.data);
  ASSERT_EQ(measured.fields.size(), 6U);
  const std::vector<std::string> names{"x", "y", "z", "intensity", "ring", "time"};
  for (size_t i = 0; i < names.size(); ++i) { EXPECT_EQ(measured.fields[i].name, names[i]); EXPECT_EQ(measured.fields[i].offset, i * 4); EXPECT_EQ(measured.fields[i].count, 1U); EXPECT_EQ(measured.fields[i].datatype, i == 4 ? 4 : 7); }
  const auto boundary = last<sensor_msgs::msg::PointCloud2>("/robot_0/points/range_boundary"); EXPECT_EQ(boundary.data, p.range_boundary); EXPECT_FALSE(boundary.is_dense);
  const auto message = last<sensor_msgs::msg::Imu>("/robot_0/imu"); EXPECT_EQ(message.header.frame_id, "robot_0/imu_link"); EXPECT_EQ(message.header.stamp.sec, 4); EXPECT_EQ(message.header.stamp.nanosec, 5U);
  EXPECT_EQ(message.orientation.w, 1); EXPECT_EQ(message.orientation_covariance, imu.orientation_covariance); EXPECT_EQ(message.linear_acceleration_covariance, imu.acceleration_covariance); EXPECT_EQ(message.angular_velocity_covariance, imu.angular_velocity_covariance);
  EXPECT_EQ(message.linear_acceleration.x, 1); EXPECT_EQ(message.linear_acceleration.y, 2); EXPECT_EQ(message.linear_acceleration.z, 3);
  EXPECT_EQ(message.angular_velocity.x, 4); EXPECT_EQ(message.angular_velocity.y, 5); EXPECT_EQ(message.angular_velocity.z, 6);
  EXPECT_EQ(Access::channels(*node)[0].published, 1U); EXPECT_EQ(Access::compute(*node).back(), 2.5); EXPECT_NEAR(Access::bridge(*node).back(), 2, 1e-12);
}

TEST_F(WorldNodeUnits, NativeCloudWithoutBoundaryDoesNotCreateOrPublishAnAuxiliaryTopic) {
  build({}, [](auto &w) { litesim3d::LidarConfig l; l.topic = "points"; w.robots[0].cfg.lidars.push_back(l); });
  EXPECT_EQ(Access::channels(*node)[0].clouds.count("points/range_boundary"), 0U);
  litesim3d::Event e; e.clouds.push_back(cloud()); Access::publish(*node, {{e}});
  EXPECT_EQ(count<sensor_msgs::msg::PointCloud2>("/robot_0/points"), 1U);
  EXPECT_EQ(count<sensor_msgs::msg::PointCloud2>("/robot_0/points/range_boundary"), 0U);
}

TEST_F(WorldNodeUnits, PluginHandledCloudsBypassNativePublishersButStillCountAndBoundTimingWindow) {
  build({rclcpp::Parameter("lidar_interface", "livox"), rclcpp::Parameter("sensor_reliable", true)}, [](auto &w) {
    sensors(w); for (auto &r : w.robots) { auto l = r.cfg.lidars[0]; l.topic = "handled"; l.scan_pattern = "livox_mid360"; r.cfg.lidars.push_back(l); }
  });
  EXPECT_EQ(Access::channels(*node)[0].clouds.count("handled"), 0U);
  litesim3d::Event e; e.clouds.push_back(cloud("handled"));
  for (size_t i = 0; i < 4098; ++i) Access::publish(*node, {{e}});
  EXPECT_EQ(serializer->published, 4098U); EXPECT_EQ(Access::channels(*node)[0].published, 4098U);
  EXPECT_EQ(Access::compute(*node).size(), 4096U); EXPECT_EQ(Access::bridge(*node).size(), 4096U);
  EXPECT_EQ(count<sensor_msgs::msg::PointCloud2>("/robot_0/handled"), 0U);
  Access::status(*node); const auto status = last<diagnostic_msgs::msg::DiagnosticArray>("/sim/status").status[0];
  EXPECT_EQ(Json::parse(key(status, "subscription_matches")).at("/fake_serializer"), 7);
  EXPECT_EQ(key(status, "sensor_queue_policy"), "keep_last/reliable"); EXPECT_EQ(key(status, "timing_window_frames"), "4096");
  EXPECT_EQ(Json::parse(key(status, "livox_output")).at("serializer"), "recording");
}

TEST_F(WorldNodeUnits, ContactRingSkipsEvictedEntriesAndPublishesEachRetainedContactExactlyOnce) {
  build(); published.clear(); auto &r = world->robots[1]; r.contact_count = 5;
  r.retained = {{2000000003, "wheel", "wall", -.02}, {3000000004, "base", "crate", .01}};
  Access::publish(*node, {}); EXPECT_EQ(count<litesim3d_interfaces::msg::Contact>("/sim/ground_truth/robot_1/contacts"), 2U);
  const auto contact = last<litesim3d_interfaces::msg::Contact>("/sim/ground_truth/robot_1/contacts");
  EXPECT_EQ(contact.header.frame_id, "world"); EXPECT_EQ(contact.header.stamp.sec, 3); EXPECT_EQ(contact.header.stamp.nanosec, 4U);
  EXPECT_EQ(contact.run_id, r.run_id); EXPECT_EQ(contact.robot_link, "robot_1/base"); EXPECT_EQ(contact.entity, "crate"); EXPECT_EQ(contact.clearance, .01);
  Access::publish(*node, {}); EXPECT_EQ(count<litesim3d_interfaces::msg::Contact>("/sim/ground_truth/robot_1/contacts"), 2U);
  r.contact_count = 6; r.retained = {{3000000004, "base", "crate", .01}, {4000000005, "wheel", "second", -.1}};
  Access::publish(*node, {}); EXPECT_EQ(count<litesim3d_interfaces::msg::Contact>("/sim/ground_truth/robot_1/contacts"), 3U);
  EXPECT_EQ(Access::channels(*node)[1].contact_index, 6U);
}

TEST_F(WorldNodeUnits, UpdateInitializesReadinessOnceAndUnpacedTickRoutesEveryRobot) {
  build(); world->tick_events.resize(2); for (auto &events : world->tick_events) { litesim3d::Event e; e.odom = true; events.push_back(e); }
  published.clear(); Access::update(*node); EXPECT_EQ(world->pauses, std::vector<bool>{false}); EXPECT_EQ(world->ticks, 1U);
  EXPECT_EQ(count<rosgraph_msgs::msg::Clock>("/clock"), 1U); EXPECT_EQ(count<nav_msgs::msg::Odometry>("/robot_0/odom"), 1U); EXPECT_EQ(count<nav_msgs::msg::Odometry>("/robot_1/odom"), 1U);
  Access::update(*node); EXPECT_EQ(world->pauses.size(), 1U); EXPECT_EQ(world->ticks, 2U);
  live = false; const auto outputs = published.size(); Access::update(*node); Access::status(*node); EXPECT_EQ(world->ticks, 2U); EXPECT_EQ(published.size(), outputs);
}

TEST_F(WorldNodeUnits, PacedUpdateWaitsForTargetAndCapsCatchupAtEightSteps) {
  build({}, [](auto &w) { w.robots[0].cfg.real_time_factor = 1; w.robots[0].cfg.step_ns = 100000000; });
  Access::update(*node); EXPECT_EQ(world->ticks, 0U);
  now += std::chrono::milliseconds(99); Access::update(*node); EXPECT_EQ(world->ticks, 0U);
  now += std::chrono::milliseconds(1); Access::update(*node); EXPECT_EQ(world->ticks, 1U);
  now += std::chrono::seconds(5); Access::update(*node); EXPECT_EQ(world->ticks, 9U);
  world->robots[0].snap.stamp = UINT64_C(9000000000); Access::update(*node); EXPECT_EQ(world->ticks, 9U);
}

TEST_F(WorldNodeUnits, RenderingStallDoesNotAccumulateUnboundedPacingDebt) {
  build({}, [](auto &engine) {
    for (auto &robot : engine.robots) {
      robot.cfg.real_time_factor = .5;
      robot.cfg.step_ns = 5000000;
    }
  });
  const auto initial_stamp = world->robots[0].snap.stamp;
  std::vector<uint64_t> commits;
  world->on_tick = [&] {
    for (auto &robot : world->robots) robot.snap.stamp += robot.cfg.step_ns;
    commits.push_back(world->robots[0].snap.stamp);
  };
  Access::update(*node);
  const auto advance = [&](unsigned callbacks) {
    for (unsigned i = 0; i < callbacks; ++i) {
      now += std::chrono::milliseconds(10);
      wall += 10000000;
      Access::update(*node);
    }
  };
  advance(200);
  const auto before_stall = world->ticks;
  now += std::chrono::seconds(1);
  wall += 1000000000;
  Access::update(*node);
  EXPECT_EQ(world->ticks - before_stall, 8U);
  const auto after_stall = world->ticks;
  Access::update(*node);
  EXPECT_EQ(world->ticks, after_stall);
  advance(200);
  double maximum_rate = 0;
  unsigned rate_samples = 0;
  for (const auto &publication : published) {
    if (publication.topic != "/sim/stats") continue;
    const auto data = Json::parse(std::any_cast<const std_msgs::msg::String &>(publication.message).data);
    EXPECT_EQ(data["target_real_time_factor"], .5);
    if (data["actual_real_time_factor"].is_null()) continue;
    const double actual = data["actual_real_time_factor"];
    EXPECT_GT(actual, 0);
    maximum_rate = std::max(maximum_rate, actual);
    ++rate_samples;
  }
  ASSERT_GE(rate_samples, 20U);
  RecordProperty("maximum_actual_rtf", std::to_string(maximum_rate));
  EXPECT_LE(maximum_rate, .6);
  // One wall-rate and one fixed timestamp sequence, not a sum of robot clocks.
  ASSERT_EQ(commits.size(), world->ticks);
  for (size_t i = 0; i < commits.size(); ++i)
    EXPECT_EQ(commits[i], initial_stamp + (i + 1) * 5000000);
  EXPECT_EQ(world->robots[0].snap.stamp, world->robots[1].snap.stamp);
}

TEST_F(WorldNodeUnits, BoundedPacingKeepsSmallJitterAndFractionalStepPhase) {
  build({}, [](auto &engine) {
    for (auto &robot : engine.robots) {
      robot.cfg.real_time_factor = .5;
      robot.cfg.step_ns = 5000000;
    }
  });
  world->on_tick = [&] {
    for (auto &robot : world->robots) robot.snap.stamp += robot.cfg.step_ns;
  };
  Access::update(*node);
  const auto advance = [&](unsigned milliseconds) {
    now += std::chrono::milliseconds(milliseconds);
    wall += uint64_t(milliseconds) * 1000000;
    Access::update(*node);
  };
  advance(85);
  EXPECT_EQ(world->ticks, 8U);
  advance(6);
  EXPECT_EQ(world->ticks, 9U); // Preserve normal sub-batch jitter.
  advance(1004);
  EXPECT_EQ(world->ticks, 17U);
  advance(6);
  EXPECT_EQ(world->ticks, 18U); // Preserve fractional phase when discarding debt.
}

TEST_F(WorldNodeUnits, StartPausedStillReportsWallStatusAndDoesNotTick) {
  build({rclcpp::Parameter("start_paused", true)}); published.clear(); Access::update(*node);
  EXPECT_EQ(world->pauses, std::vector<bool>{true}); EXPECT_EQ(world->ticks, 0U); EXPECT_EQ(count<diagnostic_msgs::msg::DiagnosticArray>("/sim/status"), 1U);
  Access::update(*node); EXPECT_EQ(count<diagnostic_msgs::msg::DiagnosticArray>("/sim/status"), 1U);
  now += std::chrono::milliseconds(200); Access::update(*node); EXPECT_EQ(count<diagnostic_msgs::msg::DiagnosticArray>("/sim/status"), 2U);
  EXPECT_TRUE(Access::pause(*node, false).success); Access::update(*node); EXPECT_EQ(world->ticks, 1U);
}

TEST_F(WorldNodeUnits, DuplicateClockPublisherFaultsBeforeAnyTick) {
  build(); clock_publishers = 2; Access::update(*node); EXPECT_TRUE(node->failed()); EXPECT_EQ(world->ticks, 0U);
  EXPECT_EQ(world->faults, std::vector<std::string>{"multiple /clock publishers"}); EXPECT_EQ(shutdowns, 1U);
}

TEST_F(WorldNodeUnits, UpdateFaultsForCoreExceptionsOnlyWhileContextIsLive) {
  build(); world->on_pause = [] { throw std::runtime_error("startup error"); }; Access::update(*node);
  EXPECT_TRUE(node->failed()); EXPECT_EQ(world->faults.back(), "startup error"); EXPECT_EQ(world->ticks, 0U);
  node.reset(); build(); world->on_tick = [] { throw std::runtime_error("tick error"); }; Access::update(*node);
  EXPECT_TRUE(node->failed()); EXPECT_EQ(world->faults.back(), "tick error");
  node.reset(); build(); world->on_tick = [&] { live = false; throw std::runtime_error("shutdown race"); }; Access::update(*node);
  EXPECT_FALSE(node->failed()); EXPECT_TRUE(world->faults.empty());
}

TEST_F(WorldNodeUnits, TfWatchIgnoresUnrelatedFramesAndIdenticalSourcesButRejectsBadParentsAndDuplicates) {
  build(); tf2_msgs::msg::TFMessage message; geometry_msgs::msg::TransformStamped t;
  t.child_frame_id = "unrelated"; t.header.frame_id = "outside"; message.transforms.push_back(t); Access::tf(*node, message, 1); EXPECT_FALSE(node->failed());
  message.transforms[0].child_frame_id = "robot_1/base_link"; message.transforms[0].header.frame_id = "robot_1/odom";
  Access::tf(*node, message, 1); Access::tf(*node, message, 1); EXPECT_FALSE(node->failed()); EXPECT_EQ(Access::channels(*node)[1].odom_sources.size(), 1U);
  Access::tf(*node, message, 2); EXPECT_TRUE(node->failed()); EXPECT_EQ(world->faults.back(), "duplicate odom TF publishers for robot_1/base_link");
  Access::tf(*node, message, 3); EXPECT_EQ(world->faults.size(), 1U);
  node.reset(); build(); message.transforms[0].header.frame_id = "world"; Access::tf(*node, message, 1);
  EXPECT_EQ(world->faults.back(), "unexpected odom TF parent for robot_1/base_link");
  node.reset(); build(); live = false; Access::tf(*node, message, 1); EXPECT_FALSE(node->failed()); EXPECT_TRUE(world->faults.empty());
}

TEST_F(WorldNodeUnits, StatusContainsReadyCountersMatchesQoSAndStatisticalPercentiles) {
  build({}, sensors); Access::statusTimer(*node);
  auto msg = last<diagnostic_msgs::msg::DiagnosticArray>("/sim/status"); ASSERT_EQ(msg.status.size(), 2U);
  EXPECT_EQ(msg.status[0].name, "robot_0/sim"); EXPECT_EQ(msg.status[0].hardware_id, world->robots[0].run_id); EXPECT_EQ(msg.status[0].level, 0);
  EXPECT_EQ(key(msg.status[0], "ready"), "false"); EXPECT_EQ(key(msg.status[0], "compute_p99_ms"), "0.0"); EXPECT_EQ(key(msg.status[0], "bridge_p99_ms"), "0.0");
  EXPECT_EQ(key(msg.status[0], "sensor_queue_policy"), "keep_last/best_effort"); EXPECT_EQ(key(msg.status[0], "dds_delivery_drops"), "unknown (measure at subscriber)");
  EXPECT_EQ(key(msg.status[0], "robot_count"), "2"); EXPECT_EQ(key(msg.status[0], "cmd_mode"), "twist");
  const auto matches = Json::parse(key(msg.status[0], "subscription_matches"));
  for (const auto &topic : {"/clock", "/robot_0/odom", "/robot_0/joint_states", "/robot_0/points", "/robot_0/points/range_boundary", "/robot_0/imu"}) EXPECT_EQ(matches.at(topic), 0);
  auto stats = Json::parse(last<std_msgs::msg::String>("/sim/stats").data); EXPECT_TRUE(stats["actual_real_time_factor"].is_null()); EXPECT_EQ(stats["schema_version"], 1); EXPECT_EQ(stats["robot_count"], 2); EXPECT_EQ(stats["run_id"], world->run_id);
  for (size_t i = 1; i <= 100; ++i) { Access::compute(*node).push_back(i); Access::bridge(*node).push_back(i * .5); }
  world->robots[0].snap.stamp += 250000000; wall += 1000000000; Access::ready(*node, true); Access::status(*node);
  msg = last<diagnostic_msgs::msg::DiagnosticArray>("/sim/status"); EXPECT_EQ(key(msg.status[0], "compute_p99_ms"), "99.0"); EXPECT_EQ(key(msg.status[0], "bridge_p99_ms"), "49.5"); EXPECT_EQ(key(msg.status[0], "ready"), "true");
  stats = Json::parse(last<std_msgs::msg::String>("/sim/stats").data); ASSERT_TRUE(stats["actual_real_time_factor"].is_number()); EXPECT_DOUBLE_EQ(stats["actual_real_time_factor"].get<double>(), .25);
  world->robots[0].operational = litesim3d::State::Paused; Access::status(*node); stats = Json::parse(last<std_msgs::msg::String>("/sim/stats").data); EXPECT_EQ(stats["actual_real_time_factor"], 0);
}

TEST_F(WorldNodeUnits, StatusWarnsForEachIndependentSignalAndFaultStatusUsesFailureReason) {
  build(); auto check = [&](int expected) { Access::status(*node); const auto s = last<diagnostic_msgs::msg::DiagnosticArray>("/sim/status").status[0]; EXPECT_EQ(s.level, expected); return s; };
  check(0); rclcpp::QOSOfferedIncompatibleQoSInfo conflict{}; Access::channels(*node)[0].incompatible_qos(conflict);
  auto s = check(1); EXPECT_EQ(key(s, "qos_conflicts"), "1"); Access::channels(*node)[0].qos_conflicts = 0;
  world->robots[0].diagnostic["command_timeout"] = true; check(1); world->robots[0].diagnostic["command_timeout"] = false;
  world->robots[0].diagnostic["rejected_commands"] = 2; check(1); world->robots[0].diagnostic["rejected_commands"] = 0;
  Access::fault(*node, "world fault"); s = check(2); EXPECT_EQ(s.message, "world fault"); EXPECT_EQ(s.hardware_id, world->robots[0].run_id);
}

TEST_F(WorldNodeUnits, StatusTimerHandlesFaultAndShutdownRaceWithoutEscaping) {
  build(); world->robots[0].before_diagnostics = [] { throw std::runtime_error("diagnostic failure"); };
  EXPECT_NO_THROW(Access::statusTimer(*node)); EXPECT_TRUE(node->failed()); EXPECT_EQ(world->faults.back(), "diagnostic failure");
  node.reset(); build(); world->robots[0].before_diagnostics = [&] { live = false; throw std::runtime_error("context ended"); };
  EXPECT_NO_THROW(Access::statusTimer(*node)); EXPECT_FALSE(node->failed()); EXPECT_TRUE(world->faults.empty());
}

TEST_F(WorldNodeUnits, ManifestIncludesPerRobotPolicyAndFinishDistinguishesShutdownFromFault) {
  const auto file = std::filesystem::temp_directory_path() / ("world-node-unit-" + std::to_string(::getpid()) + ".json");
  build({rclcpp::Parameter("manifest_path", file.string()), rclcpp::Parameter("external_odom_tf_robots", std::vector<std::string>{"robot_1"})});
  auto manifest = Json::parse(litesim3d::readText(file)); EXPECT_EQ(manifest["exit_reason"], "ready"); EXPECT_EQ(manifest["error"], "");
  EXPECT_EQ(manifest["ros_distro"], "humble"); EXPECT_EQ(manifest["rmw"], rmw_get_implementation_identifier()); EXPECT_EQ(manifest["bridge_source_sha256"], LWS_BRIDGE_SHA256);
  EXPECT_EQ(manifest["started_wall_ns"], wall); EXPECT_EQ(manifest["recorded_wall_ns"], wall); EXPECT_EQ(manifest["ros_interface"]["cmd_mode"], "twist");
  EXPECT_EQ(manifest["ros_interface"]["sensor_reliable"], false); EXPECT_EQ(manifest["ros_interface"]["publish_odom_tf"]["robot_0"], true); EXPECT_EQ(manifest["ros_interface"]["publish_odom_tf"]["robot_1"], false);
  EXPECT_EQ(manifest["robots"][0]["exit_reason"], "ready"); EXPECT_EQ(manifest["robots"][0]["livox_output"]["lidar_interface"], "native");
  node->finish(); EXPECT_EQ(world->exits, 1U); manifest = Json::parse(litesim3d::readText(file)); EXPECT_EQ(manifest["exit_reason"], "shutdown");
  Access::fault(*node, "manifest fault"); node->finish(); EXPECT_EQ(world->exits, 1U); manifest = Json::parse(litesim3d::readText(file));
  EXPECT_EQ(manifest["exit_reason"], "fault"); EXPECT_EQ(manifest["error"], "manifest fault"); std::filesystem::remove(file);
}

TEST_F(WorldNodeUnits, ManifestOpenAndWriteFailuresPropagateWithoutFabricatingSuccess) {
  build(); Access::output(*node, std::filesystem::temp_directory_path().string());
  EXPECT_THROW(Access::save(*node, "test"), std::runtime_error);
  struct RejectingBuffer : std::streambuf { int_type overflow(int_type) override { return traits_type::eof(); } } buffer;
  Access::hooks(*node).open_manifest = [&](const auto &) { return std::make_unique<std::ostream>(&buffer); };
  try { Access::save(*node, "test"); FAIL() << "write failure ignored"; }
  catch (const std::runtime_error &e) { EXPECT_STREQ(e.what(), "cannot write world manifest"); }
  Access::hooks(*node).open_manifest = [](const auto &) { auto s = std::make_unique<std::ostringstream>(); s->setstate(std::ios::badbit); return s; };
  try { Access::save(*node, "test"); FAIL() << "open failure ignored"; }
  catch (const std::runtime_error &e) { EXPECT_STREQ(e.what(), "cannot open world manifest"); }
}

TEST_F(WorldNodeUnits, DefaultClockContextPublishAndShutdownPathsNeedNoPeerOrPhysics) {
  build({}, [](auto &w) { w.robots[0].cfg.real_time_factor = 1e30; w.robots[1].cfg.control_mode = "ros2_control"; }, false);
  auto &hooks = Access::hooks(*node); hooks.steady_now = {}; hooks.wall_now = {}; hooks.ok = {}; hooks.publisher_count = {}; hooks.publish = {};
  EXPECT_NO_THROW(Access::publish(*node, {}));
  EXPECT_NO_THROW(Access::status(*node)); EXPECT_TRUE(context->is_valid());
  world->robots[0].cfg.real_time_factor = 0;
  EXPECT_NO_THROW(Access::update(*node)); EXPECT_EQ(world->ticks, 1U);
  hooks.shutdown = {};
  EXPECT_NO_THROW(Access::fault(*node, "private context ended")); EXPECT_FALSE(context->is_valid()); EXPECT_TRUE(node->failed());
  EXPECT_NO_THROW(Access::update(*node));
  EXPECT_NO_THROW(Access::status(*node));
  EXPECT_GT(wallTime(), 0U);
}

TEST_F(WorldNodeUnits, ProcessLifecyclePreservesArgumentsAndFinishesSuccessfulNodeBeforeShutdown) {
  build(); WorldMainHooks hooks; size_t initialized = 0, spun = 0, stopped = 0; std::vector<std::string> errors;
  char executable[] = "world-unit", option[] = "--unit"; char *argv[] = {executable, option, nullptr};
  hooks.initialize = [&](int argc, char **actual) { ++initialized; EXPECT_EQ(argc, 2); EXPECT_EQ(actual, argv); };
  hooks.make_node = [&] { return node; };
  hooks.spin = [&](const auto &actual) { ++spun; EXPECT_EQ(actual, node); EXPECT_EQ(world->exits, 0U); };
  hooks.ok = [&] { return live; };
  hooks.shutdown = [&] { ++stopped; EXPECT_EQ(world->exits, 1U); };
  hooks.report = [&](const auto &error) { errors.push_back(error); };
  EXPECT_EQ(runWorldNode(2, argv, hooks), 0); EXPECT_EQ(initialized, 1U); EXPECT_EQ(spun, 1U); EXPECT_EQ(stopped, 1U);
  EXPECT_EQ(world->exits, 1U); EXPECT_TRUE(errors.empty());
}

TEST_F(WorldNodeUnits, ProcessLifecycleFailureAndInterruptedSpinHaveDistinctExitCodesAndFinishPolicy) {
  build(); WorldMainHooks hooks; size_t stopped = 0; std::vector<std::string> errors;
  hooks.initialize = [](int, char **) {}; hooks.make_node = [&] { return node; };
  hooks.ok = [&] { return live; }; hooks.shutdown = [&] { ++stopped; }; hooks.report = [&](const auto &error) { errors.push_back(error); };
  hooks.spin = [&](const auto &) { throw std::runtime_error("spin live failure"); };
  EXPECT_EQ(runWorldNode(0, nullptr, hooks), 1); EXPECT_EQ(world->exits, 0U); EXPECT_EQ(stopped, 1U); EXPECT_EQ(errors, std::vector<std::string>{"spin live failure"});
  errors.clear(); stopped = 0;
  hooks.spin = [&](const auto &) { live = false; throw std::runtime_error("context interrupted"); };
  EXPECT_EQ(runWorldNode(0, nullptr, hooks), 0); EXPECT_EQ(world->exits, 1U); EXPECT_EQ(stopped, 0U); EXPECT_TRUE(errors.empty());
  hooks.spin = [&](const auto &) { Access::fault(*node, "core failed"); throw std::runtime_error("fault spin failure"); };
  EXPECT_EQ(runWorldNode(0, nullptr, hooks), 1); EXPECT_EQ(world->exits, 1U); EXPECT_EQ(stopped, 0U); EXPECT_EQ(errors, std::vector<std::string>{"fault spin failure"});
}

TEST_F(WorldNodeUnits, ProcessLifecycleNormalFaultIsNonzeroWithoutExitingTheFaultedBackend) {
  build(); WorldMainHooks hooks; size_t stopped = 0;
  hooks.initialize = [](int, char **) {}; hooks.make_node = [&] { return node; };
  hooks.spin = [&](const auto &) { Access::fault(*node, "tick fault"); live = false; };
  hooks.ok = [&] { return live; }; hooks.shutdown = [&] { ++stopped; };
  EXPECT_EQ(runWorldNode(0, nullptr, hooks), 1); EXPECT_EQ(world->exits, 0U); EXPECT_EQ(shutdowns, 1U); EXPECT_EQ(stopped, 0U);
}

TEST_F(WorldNodeUnits, ProcessLifecycleReportsFactoryFailureAndRetainsInitializationFailureBehavior) {
  build(); WorldMainHooks hooks; std::vector<std::string> errors; size_t stopped = 0;
  hooks.initialize = [](int, char **) {}; hooks.make_node = []() -> std::shared_ptr<WorldNode> { throw std::runtime_error("factory failure"); };
  hooks.spin = [](const auto &) { ADD_FAILURE() << "failed factory was spun"; };
  hooks.ok = [] { return true; }; hooks.shutdown = [&] { ++stopped; }; hooks.report = [&](const auto &error) { errors.push_back(error); };
  EXPECT_EQ(runWorldNode(0, nullptr, hooks), 1); EXPECT_EQ(stopped, 1U); EXPECT_EQ(errors, std::vector<std::string>{"factory failure"}); EXPECT_EQ(world->exits, 0U);
  hooks.initialize = [](int, char **) { throw std::runtime_error("init failure"); };
  EXPECT_THROW(runWorldNode(0, nullptr, hooks), std::runtime_error); EXPECT_EQ(stopped, 1U); EXPECT_EQ(errors.size(), 1U);
}

TEST_F(WorldNodeUnits, ProcessLifecycleInterruptedFinishRetainsOriginalRecoveryPath) {
  build(); WorldMainHooks hooks; size_t exit_attempts = 0;
  world->on_exit = [&] { if (++exit_attempts == 1) throw std::runtime_error("finish interrupted"); };
  hooks.initialize = [](int, char **) {}; hooks.make_node = [&] { return node; }; hooks.spin = [](const auto &) {};
  hooks.ok = [] { return false; };
  EXPECT_EQ(runWorldNode(0, nullptr, hooks), 0); EXPECT_EQ(exit_attempts, 2U); EXPECT_EQ(world->exits, 1U);
}

TEST_F(WorldNodeUnits, ProcessLifecycleDefaultRosInitializationShutdownAndStderrNeedNoExecutor) {
  build(); WorldMainHooks hooks;
  hooks.make_node = [&] { return node; }; hooks.spin = [](const auto &) {};
  EXPECT_EQ(runWorldNode(0, nullptr, hooks), 0); EXPECT_EQ(world->exits, 1U); EXPECT_FALSE(rclcpp::ok());
  hooks.initialize = [](int, char **) {};
  hooks.make_node = []() -> std::shared_ptr<WorldNode> { throw std::runtime_error("stderr failure"); };
  std::ostringstream errors; auto *old = std::cerr.rdbuf(errors.rdbuf());
  const auto code = runWorldNode(0, nullptr, hooks); std::cerr.rdbuf(old);
  EXPECT_EQ(code, 1); EXPECT_EQ(errors.str(), "stderr failure\n");
}

TEST_F(WorldNodeUnits, DefaultFactoryAndSpinRejectAnInvalidRosContextBeforeAnyExecutorLoopOrPhysics) {
  build(); ASSERT_FALSE(rclcpp::ok());
  WorldMainHooks hooks; std::vector<std::string> errors;
  hooks.initialize = [](int, char **) {}; hooks.ok = [] { return false; };
  hooks.report = [&](const auto &message) { errors.push_back(message); };
  // A default Node cannot be constructed in an invalid global context. This
  // tests the actual factory fallback/error boundary; no World is loaded.
  EXPECT_EQ(runWorldNode(0, nullptr, hooks), 1); ASSERT_EQ(errors.size(), 1U);
  EXPECT_NE(errors[0].find("context"), std::string::npos); EXPECT_EQ(world->exits, 0U); EXPECT_EQ(world->ticks, 0U);
  errors.clear(); hooks.make_node = [&] { return node; };
  context->shutdown("negative executor construction boundary"); ASSERT_FALSE(context->is_valid());
  // spin(node) derives the executor context from this node, not the global
  // context. Its guard-condition constructor must reject our invalid private
  // context before adding the inert node or entering a loop. Shutdown is handled
  // as an interruption, retaining the original successful finish policy.
  EXPECT_EQ(runWorldNode(0, nullptr, hooks), 0); EXPECT_TRUE(errors.empty());
  EXPECT_EQ(world->exits, 1U); EXPECT_EQ(world->ticks, 0U); EXPECT_TRUE(world->commands.empty());
}
