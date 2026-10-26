#include <litesim3d_plugin_api/motion.hpp>
#include <litesim3d_plugin_api/model_geometry.hpp>
#include <litesim3d_plugin_api/declarations.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <fstream>
#include <chrono>

namespace lp = litesim3d::plugins;
using lp::Json;
namespace {
constexpr double pi = 3.14159265358979323846;
const std::string diff = "litesim3d_diff_drive/DifferentialDrive";
const std::string ack = "litesim3d_ackermann_drive/AckermannDrive";
const std::string diff_odom = "litesim3d_diff_odometry/DifferentialOdometry";
const std::string ack_odom = "litesim3d_ackermann_odometry/AckermannOdometry";
const std::string omni_odom = "litesim3d_omni_odometry/OmniOdometry";
std::string omni(int n) { return "litesim3d_omni" + std::to_string(n) + "_drive/Omni" + std::to_string(n) + "Drive"; }
Json json(const YAML::Node &node) {
  if (node.IsMap()) {
    Json out = Json::object(); for (auto entry : node) out[entry.first.as<std::string>()] = json(entry.second); return out;
  }
  if (node.IsSequence()) { Json out = Json::array(); for (auto entry : node) out.push_back(json(entry)); return out; }
  if (node.IsNull()) return nullptr;
  const auto text = node.as<std::string>();
  if (text == "true") return true; if (text == "false") return false;
  try { size_t end; double value = std::stod(text, &end); if (end == text.size()) return value; } catch (...) {}
  return text;
}
lp::Initialization differential(bool four);
lp::Initialization ackermann();
lp::Initialization omnidirectional(int n);
Json config(const std::string &cls, bool resolve = true) {
  const auto package = cls.substr(0, cls.find('/'));
  auto p = json(YAML::LoadFile(ament_index_cpp::get_package_share_directory(package) + "/config/parameters.yaml"));
  if (!resolve) return p;
  // Direct plugin unit tests supply the geometry of their synthetic fixture.
  const bool is_diff = cls == diff || cls == diff_odom, is_ack = cls == ack || cls == ack_odom;
  lp::ModelGeometry g;
  g.model = (is_diff ? differential(false) : is_ack ? ackermann() : omnidirectional(cls == omni(4) ? 4 : 3)).model;
  for (const auto &[name, joint] : g.model.joints) if (joint.type == "continuous") g.wheel_radii[name] = .1;
  if (is_ack) p["max_steering_rad"] = .6;  // Operational cap used by the existing motion tests.
  return is_diff ? lp::differentialGeometry(g, p) : is_ack ? lp::ackermannGeometry(g, p) : lp::omniGeometry(g, p);
}
lp::JointDescription wheel(double x, double y, double angle = 0) {
  lp::JointDescription j; j.type = "continuous"; j.position = {x, y, .1};
  j.axis = {-std::sin(angle), std::cos(angle), 0}; j.max_velocity = 100; return j;
}
lp::Initialization differential(bool four = false) {
  lp::Initialization init; init.instance = "wheel_odom"; init.model.base_link = "base_link";
  init.model.links.insert("base_link");
  init.model.joints["left_wheel_joint"] = wheel(0, .25);
  init.model.joints["right_wheel_joint"] = wheel(0, -.25);
  if (four) {
    init.model.joints["front_left_joint"] = wheel(.4, .25);
    init.model.joints["front_right_joint"] = wheel(.4, -.25);
  }
  return init;
}
lp::Initialization ackermann() {
  lp::Initialization init; init.instance = "ack_odom"; init.model.base_link = "rear_axle"; init.model.links.insert("rear_axle");
  for (const auto &side : {std::string("left"), std::string("right")}) {
    const double y = side == "left" ? .2 : -.2;
    init.model.joints["rear_" + side + "_wheel_joint"] = wheel(0, y);
    auto front = wheel(.6, y); front.parent_joint = side + "_steering_joint";
    init.model.joints["front_" + side + "_wheel_joint"] = front;
    lp::JointDescription steer; steer.type = "revolute"; steer.axis = Eigen::Vector3d::UnitZ();
    steer.position = {.6, y, .1}; steer.lower = -.7; steer.upper = .7; steer.max_velocity = 10;
    init.model.joints[side + "_steering_joint"] = steer;
  }
  return init;
}
lp::Initialization omnidirectional(int n) {
  lp::Initialization init; init.instance = "omni_odom"; init.model.base_link = "base_link"; init.model.links.insert("base_link");
  for (int i = 0; i < n; ++i) {
    const double theta = 2 * pi * i / n + (n == 4 ? pi / 4 : 0);
    const double radius = n == 4 ? std::sqrt(2) * .3 : .3;
    init.model.joints["wheel_" + std::to_string(i) + "_joint"] = wheel(radius * std::cos(theta), radius * std::sin(theta), theta + pi / 2);
  }
  return init;
}
lp::JointState zero(const lp::Initialization &init) {
  lp::JointState result;
  for (const auto &[name, joint] : init.model.joints) { result.position[name] = 0; result.velocity[name] = 0; }
  return result;
}
lp::JointState commit(const lp::JointState &old, const lp::DriveStep &step) {
  auto next = old;
  for (const auto &[name, delta] : step.wheel_delta) next.position[name] += delta;
  for (const auto &[name, angle] : step.steering_position) next.position[name] = angle;
  next.velocity = step.joint_velocity; return next;
}
void contract(const Json &schema, const Json &file, const std::string &path = "") {
  ASSERT_TRUE(file.is_object()) << path;
  EXPECT_EQ(schema.size(), file.size()) << path;
  for (auto it = schema.begin(); it != schema.end(); ++it) {
    ASSERT_TRUE(file.contains(it.key())) << path + it.key();
    if (it.value().is_object()) contract(it.value(), file.at(it.key()), path + it.key() + ".");
    else if (!it.value().is_null()) EXPECT_EQ(it.value(), file.at(it.key())) << path + it.key();
  }
}
}

TEST(PluginLoading, AllInstalledImplementationsAndResolvedDefaultFiles) {
  lp::Loader loader;
  for (const auto &name : {diff, ack, omni(3), omni(4)}) {
    SCOPED_TRACE(name); auto plugin = loader.chassis(name); contract(plugin->defaults(), config(name));
  }
  for (const auto &name : {diff_odom, ack_odom, omni_odom}) {
    SCOPED_TRACE(name); auto plugin = loader.odometry(name); contract(plugin->defaults(), config(name));
  }
}
TEST(PluginParameters, ModelCapabilityResolvesBothChassisAndOdometryFromBindingsOnly) {
  lp::Loader loader;
  for (const auto &name : {diff, diff_odom, ack, ack_odom, omni(3), omni(4), omni_odom}) {
    SCOPED_TRACE(name);
    const bool is_diff = name == diff || name == diff_odom, is_ack = name == ack || name == ack_odom;
    lp::ModelGeometry g;
    g.model = (is_diff ? differential() : is_ack ? ackermann() : omnidirectional(name == omni(4) ? 4 : 3)).model;
    for (const auto &[joint, value] : g.model.joints) if (value.type == "continuous") g.wheel_radii[joint] = .17;
    const auto p = config(name, false);
    std::shared_ptr<lp::ChassisPlugin> chassis;
    std::shared_ptr<lp::OdometryPlugin> odometry;
    lp::ModelParameters *capability;
    if (name == diff_odom || name == ack_odom || name == omni_odom) {
      odometry = loader.odometry(name); capability = dynamic_cast<lp::ModelParameters *>(odometry.get());
    } else {
      chassis = loader.chassis(name); capability = dynamic_cast<lp::ModelParameters *>(chassis.get());
    }
    ASSERT_NE(capability, nullptr);
    const auto resolved = capability->resolveModelParameters(g, p);
    if (is_diff || is_ack) {
      EXPECT_FALSE(p.contains("wheel_radius_m")); EXPECT_EQ(resolved.at("wheel_radius_m"), .17);
    } else {
      EXPECT_FALSE(p.at("wheels")[0].contains("radius_m")); EXPECT_EQ(resolved.at("wheels")[0].at("radius_m"), .17);
    }
    if (is_ack) EXPECT_EQ(resolved.at("max_steering_rad"), .7);
  }
}
TEST(PluginLoading, MissingClassAndWrongCategoryFailClearly) {
  lp::Loader loader;
  EXPECT_THROW(loader.chassis("missing_package/MissingClass"), std::runtime_error);
  EXPECT_THROW(loader.odometry(diff), std::runtime_error);
}
TEST(PluginLoading, InstanceOwnsLoaderLifetime) {
  std::shared_ptr<lp::ChassisPlugin> plugin;
  { lp::Loader loader; plugin = loader.chassis(diff); }
  auto init = differential(); plugin->initialize(init, config(diff)); plugin->reset(zero(init));
  EXPECT_NO_THROW(plugin->step({10000000, {.5, 0, 0, 0}, {}, zero(init)}));
  plugin.reset();
}
TEST(PluginParameters, MergePriorityAndUnknownMissingKeys) {
  auto base = Json{{"geometry", {{"radius", .1}, {"track", .5}}}, {"wheels", {1, 2}}};
  auto merged = lp::merge(base, {{"geometry", {{"radius", .2}}}, {"wheels", {3}}});
  EXPECT_EQ(merged["geometry"]["radius"], .2); EXPECT_EQ(merged["geometry"]["track"], .5);
  EXPECT_EQ(merged["wheels"], Json::array({3}));
  lp::Loader loader; auto plugin = loader.chassis(diff); auto p = config(diff);
  p["typo"] = 1; EXPECT_THROW(plugin->initialize(differential(), p), std::runtime_error);
  p = config(diff); p.erase("wheel_radius_m"); EXPECT_THROW(plugin->initialize(differential(), p), std::runtime_error);
  p = config(diff); p["wheel_radius_m"] = -1; EXPECT_THROW(plugin->initialize(differential(), p), std::runtime_error);
  p = config(diff); p["limits"]["unlimited_acceleration"] = "false"; EXPECT_THROW(plugin->initialize(differential(), p), std::runtime_error);
}
TEST(DifferentialPlugin, StraightTurnAndPureRotationHaveCorrectEncoderRates) {
  lp::Loader loader; auto drive = loader.chassis(diff); auto odom = loader.odometry(diff_odom);
  auto init = differential(); auto p = config(diff); p["limits"]["unlimited_acceleration"] = true;
  drive->initialize(init, p); odom->initialize(init, config(diff_odom)); const auto q = zero(init);
  auto straight = drive->step({100000000, {1, 0, 0, 0}, {}, q});
  EXPECT_NEAR(straight.wheel_delta.at("left_wheel_joint"), 1, 1e-12);
  EXPECT_NEAR(straight.joint_velocity.at("right_wheel_joint"), 10, 1e-12);
  auto turn = drive->step({100000000, {1, 0, 0, 1}, {}, q});
  EXPECT_NEAR(turn.joint_velocity.at("left_wheel_joint"), 7.5, 1e-12);
  EXPECT_NEAR(turn.joint_velocity.at("right_wheel_joint"), 12.5, 1e-12);
  auto out = odom->step({100000000, q, commit(q, turn)});
  EXPECT_NEAR(out.pose.translation().x(), std::sin(.1), 1e-12);
  EXPECT_NEAR(out.pose.translation().y(), 1 - std::cos(.1), 1e-12);
  odom->reset(q);
  auto spin = drive->step({100000000, {0, 0, 0, 1}, {}, q});
  EXPECT_NEAR(spin.joint_velocity.at("left_wheel_joint"), -2.5, 1e-12);
  out = odom->step({100000000, q, commit(q, spin)});
  EXPECT_LT(out.pose.translation().norm(), 1e-12);
  EXPECT_NEAR(Eigen::AngleAxisd(out.pose.linear()).angle(), .1, 1e-12);
}
TEST(DifferentialPlugin, FourIndependentWheelsRunSameSpeedOnEachSide) {
  lp::Loader loader; auto drive = loader.chassis(diff); auto init = differential(true); auto p = config(diff);
  p["limits"]["unlimited_acceleration"] = true;
  p["left_joints"].push_back("front_left_joint"); p["right_joints"].push_back("front_right_joint");
  drive->initialize(init, p);
  auto result = drive->step({100000000, {.5, 0, 0, 1}, {}, zero(init)});
  EXPECT_EQ(result.joint_velocity.at("left_wheel_joint"), result.joint_velocity.at("front_left_joint"));
  EXPECT_EQ(result.joint_velocity.at("right_wheel_joint"), result.joint_velocity.at("front_right_joint"));
  EXPECT_NE(result.joint_velocity.at("left_wheel_joint"), result.joint_velocity.at("right_wheel_joint"));
}
TEST(DifferentialPlugin, InvalidJointBindingsAndUnsupportedMotionFail) {
  lp::Loader loader; auto drive = loader.chassis(diff); auto init = differential(); auto p = config(diff);
  p["right_joints"] = p["left_joints"]; EXPECT_THROW(drive->initialize(init, p), std::runtime_error);
  p = config(diff); p["left_joints"] = {"missing"}; EXPECT_THROW(drive->initialize(init, p), std::runtime_error);
  p = config(diff); init.model.joints.at("left_wheel_joint").axis *= -1; EXPECT_THROW(drive->initialize(init, p), std::runtime_error);
  init = differential(); drive->initialize(init, p);
  EXPECT_THROW(drive->step({100000000, {0, 1, 0, 0}, {}, zero(init)}), std::runtime_error);
  EXPECT_THROW(drive->step({0, {}, {}, zero(init)}), std::runtime_error);
}
TEST(AckermannPlugin, InnerOuterAnglesAndAllWheelRates) {
  lp::Loader loader; auto drive = loader.chassis(ack); auto p = config(ack); p["limits"]["unlimited_acceleration"] = true;
  const auto init = ackermann(); drive->initialize(init, p); const auto q = zero(init);
  auto result = drive->step({100000000, {1, 0, 0, .5}, {}, q});
  EXPECT_NEAR(result.steering_position.at("left_steering_joint"), std::atan(.6 / 1.8), 1e-12);
  EXPECT_NEAR(result.steering_position.at("right_steering_joint"), std::atan(.6 / 2.2), 1e-12);
  EXPECT_NEAR(result.joint_velocity.at("rear_left_wheel_joint"), 9, 1e-12);
  EXPECT_NEAR(result.joint_velocity.at("rear_right_wheel_joint"), 11, 1e-12);
  EXPECT_NEAR(result.joint_velocity.at("front_left_wheel_joint"), std::hypot(.9, .3) / .1, 1e-12);
  auto odom = loader.odometry(ack_odom); odom->initialize(init, config(ack_odom));
  auto out = odom->step({100000000, q, commit(q, result)});
  EXPECT_NEAR(out.pose.translation().x(), 2 * std::sin(.05), 1e-12);
  EXPECT_NEAR(out.pose.translation().y(), 2 * (1 - std::cos(.05)), 1e-12);
  EXPECT_NEAR(out.velocity.wz, .5, 1e-12);
}
TEST(AckermannPlugin, ReverseSteeringLimitsAndNoZeroSpeedPivot) {
  lp::Loader loader; auto drive = loader.chassis(ack); auto p = config(ack); p["limits"]["unlimited_acceleration"] = true;
  const auto init = ackermann(); drive->initialize(init, p); const auto q = zero(init);
  auto reverse = drive->step({100000000, {-1, 0, 0, .5}, {}, q});
  EXPECT_LT(reverse.steering_position.at("left_steering_joint"), 0);
  EXPECT_LT(reverse.joint_velocity.at("front_left_wheel_joint"), 0);
  auto clipped = drive->step({100000000, {.1, 0, 0, 2}, {}, q});
  EXPECT_NEAR(clipped.steering_position.at("left_steering_joint"), .6, 1e-12);
  EXPECT_LT(clipped.steering_position.at("right_steering_joint"), .6);
  auto pivot = drive->step({100000000, {0, 0, 0, 1}, {}, q});
  EXPECT_EQ(pivot.velocity.wz, 0); EXPECT_EQ(pivot.average.wz, 0);
  for (const auto &[name, value] : pivot.wheel_delta) EXPECT_EQ(value, 0) << name;
}
TEST(AckermannPlugin, IncorrectSteeringHierarchyAndLimitsAreRejected) {
  lp::Loader loader; auto drive = loader.chassis(ack); auto init = ackermann();
  init.model.joints.at("front_left_wheel_joint").parent_joint.clear();
  EXPECT_THROW(drive->initialize(init, config(ack)), std::runtime_error);
  init = ackermann(); init.model.joints.at("left_steering_joint").upper = .3;
  EXPECT_THROW(drive->initialize(init, config(ack)), std::runtime_error);
}
TEST(AckermannPlugin, ResetClearsPoseWithNonzeroEncoderAndSteeringAngles) {
  lp::Loader loader; auto drive = loader.chassis(ack); auto odom = loader.odometry(ack_odom);
  auto p = config(ack); p["limits"]["unlimited_acceleration"] = true;
  const auto init = ackermann(); drive->initialize(init, p); odom->initialize(init, config(ack_odom));
  auto before = zero(init);
  auto motion = drive->step({1000000000, {.5, 0, 0, .2}, {}, before});
  auto after = commit(before, motion);
  const auto moved = odom->step({1000000000, before, after});
  EXPECT_GT(moved.pose.translation().norm(), .4);
  for (auto &[name, value] : after.velocity) value = 0;
  drive->reset(after); odom->reset(after);
  const auto stopped = odom->step({100000000, after, after});
  EXPECT_TRUE(stopped.pose.matrix().isApprox(lp::Pose::Identity().matrix(), 1e-12));
  EXPECT_EQ(stopped.velocity.vx, 0); EXPECT_EQ(stopped.velocity.wz, 0);
}
TEST(OmniPlugin, ThreeWheelKnownRatesAndLateralOdometry) {
  lp::Loader loader; auto drive = loader.chassis(omni(3)); auto odom = loader.odometry(omni_odom);
  auto p = config(omni(3)); p["limits"]["unlimited_acceleration"] = true;
  const auto init = omnidirectional(3); drive->initialize(init, p); odom->initialize(init, config(omni_odom));
  const auto q = zero(init);
  auto forward = drive->step({100000000, {1, 0, 0, 0}, {}, q});
  EXPECT_NEAR(forward.joint_velocity.at("wheel_0_joint"), 0, 1e-12);
  EXPECT_NEAR(forward.joint_velocity.at("wheel_1_joint"), -5 * std::sqrt(3), 1e-12);
  EXPECT_NEAR(forward.joint_velocity.at("wheel_2_joint"), 5 * std::sqrt(3), 1e-12);
  auto lateral = drive->step({100000000, {0, 1, 0, 0}, {}, q});
  auto out = odom->step({100000000, q, commit(q, lateral)});
  EXPECT_NEAR(out.pose.translation().x(), 0, 1e-12); EXPECT_NEAR(out.pose.translation().y(), .1, 1e-12);
  EXPECT_NEAR(out.velocity.vy, 1, 1e-12); EXPECT_NEAR(out.velocity.wz, 0, 1e-12);
}
TEST(OmniPlugin, FourOrdinaryOmniWheelsCanTranslateAndRotate) {
  lp::Loader loader; auto drive = loader.chassis(omni(4)); auto odom = loader.odometry(omni_odom);
  auto p = config(omni(4)); p["limits"]["unlimited_acceleration"] = true;
  auto o = config(omni_odom); o["wheels"] = p["wheels"];
  const auto init = omnidirectional(4); drive->initialize(init, p); odom->initialize(init, o); const auto q = zero(init);
  auto lateral = drive->step({100000000, {0, 1, 0, 0}, {}, q});
  EXPECT_NEAR(lateral.joint_velocity.at("wheel_0_joint"), 5 * std::sqrt(2), 1e-12);
  EXPECT_NEAR(lateral.joint_velocity.at("wheel_1_joint"), -5 * std::sqrt(2), 1e-12);
  auto combined = drive->step({100000000, {.4, .3, 0, 1}, {}, q});
  auto out = odom->step({100000000, q, commit(q, combined)});
  EXPECT_NEAR(out.pose.translation().x(), .4 * std::sin(.1) - .3 * (1 - std::cos(.1)), 1e-12);
  EXPECT_NEAR(out.pose.translation().y(), .4 * (1 - std::cos(.1)) + .3 * std::sin(.1), 1e-12);
  EXPECT_NEAR(out.velocity.vx, .4, 1e-12); EXPECT_NEAR(out.velocity.vy, .3, 1e-12); EXPECT_NEAR(out.velocity.wz, 1, 1e-12);
  odom->reset(q);
  auto spin = drive->step({100000000, {0, 0, 0, 1}, {}, q});
  out = odom->step({100000000, q, commit(q, spin)});
  EXPECT_LT(out.pose.translation().norm(), 1e-12);
}
TEST(OmniPlugin, SingularLayoutWrongCountPositionAndDirectionAreRejected) {
  lp::Loader loader; auto drive = loader.chassis(omni(3)); auto p = config(omni(3)); auto init = omnidirectional(3);
  p["wheels"].erase(0); EXPECT_THROW(drive->initialize(init, p), std::runtime_error);
  p = config(omni(3)); p["wheels"][0]["radius_m"] = 0; EXPECT_THROW(drive->initialize(init, p), std::runtime_error);
  p = config(omni(3)); p["wheels"][0]["x_m"] = 1; EXPECT_THROW(drive->initialize(init, p), std::runtime_error);
  p = config(omni(3)); p["wheels"][0]["rolling_angle_rad"] = 0; EXPECT_THROW(drive->initialize(init, p), std::runtime_error);
  p = config(omni(3));
  for (int i = 0; i < 3; ++i) {
    p["wheels"][i]["rolling_angle_rad"] = 0;
    init.model.joints.at("wheel_" + std::to_string(i) + "_joint").axis = Eigen::Vector3d::UnitY();
  }
  EXPECT_THROW(drive->initialize(init, p), std::runtime_error);
}
TEST(MotionContract, AccelerationAndJointVelocityLimits) {
  lp::Loader loader; auto drive = loader.chassis(diff); auto init = differential();
  drive->initialize(init, config(diff)); auto q = zero(init);
  auto result = drive->step({100000000, {1, 0, 0, 0}, {}, q});
  EXPECT_NEAR(result.velocity.vx, .05, 1e-12); EXPECT_NEAR(result.average.vx, .025, 1e-12);
  init.model.joints.at("left_wheel_joint").max_velocity = 2;
  auto p = config(diff); p["limits"]["unlimited_acceleration"] = true; drive->initialize(init, p);
  result = drive->step({100000000, {1, 0, 0, 0}, {}, q});
  EXPECT_NEAR(result.velocity.vx, .2, 1e-12); EXPECT_NEAR(result.joint_velocity.at("left_wheel_joint"), 2, 1e-12);
}
TEST(OdometryContract, UsesEncoderDisplacementNotTruthOrCommandAndResets) {
  lp::Loader loader; auto odom = loader.odometry(diff_odom); const auto init = differential();
  odom->initialize(init, config(diff_odom)); auto q = zero(init), q1 = q;
  q1.position["left_wheel_joint"] = 10; q1.position["right_wheel_joint"] = 10;
  q1.velocity["left_wheel_joint"] = 0; q1.velocity["right_wheel_joint"] = 0;
  auto out = odom->step({1000000000, q, q1});
  EXPECT_NEAR(out.pose.translation().x(), 1, 1e-12); EXPECT_EQ(out.velocity.vx, 0);
  odom->reset(q1); out = odom->step({100000000, q1, q1});
  EXPECT_LT(out.pose.translation().norm(), 1e-12);
  auto invalid = q1; invalid.position.erase("left_wheel_joint");
  EXPECT_THROW(odom->step({100000000, invalid, q1}), std::runtime_error);
}
TEST(OdometryContract, SplitTimeStepsAndNoiseResetAreDeterministic) {
  lp::Loader loader; auto one = loader.odometry(omni_odom), split = loader.odometry(omni_odom);
  auto drive = loader.chassis(omni(3)); const auto init = omnidirectional(3); auto p = config(omni(3));
  p["limits"]["unlimited_acceleration"] = true; drive->initialize(init, p);
  auto cfg = config(omni_odom); one->initialize(init, cfg); split->initialize(init, cfg);
  auto q = zero(init); auto whole = drive->step({1000000000, {.4, .3, 0, .5}, {}, q});
  auto expected = one->step({1000000000, q, commit(q, whole)}); lp::OdometryOutput actual;
  for (int i = 0; i < 10; ++i) {
    auto piece = drive->step({100000000, {.4, .3, 0, .5}, {}, q}); auto next = commit(q, piece);
    actual = split->step({100000000, q, next}); q = next;
  }
  EXPECT_LT((actual.pose.matrix() - expected.pose.matrix()).norm(), 1e-12);
  cfg["noise"]["linear_sigma_mps"] = {.1, .1}; one->initialize(init, cfg); q = zero(init);
  auto first = one->step({1000000000, q, commit(q, whole)}); one->reset(q);
  auto again = one->step({1000000000, q, commit(q, whole)});
  EXPECT_EQ(first.pose.matrix(), again.pose.matrix());
}

TEST(OdometryContract, ZeroCenteredErrorsBiasAndLegacyScaleCompatibility) {
  lp::EncoderIntegrator integrator;
  auto init = differential();
  auto p = lp::EncoderIntegrator::defaults();
  const lp::Twist velocity{.4, .2, 0, .5};
  integrator.initialize(init, p);
  auto ideal = integrator.update(velocity, velocity, 1000000000);
  EXPECT_TRUE(ideal.pose.matrix().isApprox(lp::integrate(lp::Pose::Identity(), velocity, 1).matrix(), 1e-12));
  EXPECT_EQ(ideal.velocity.vx, velocity.vx);
  EXPECT_EQ(ideal.velocity.vy, velocity.vy);
  EXPECT_EQ(ideal.velocity.wz, velocity.wz);

  p["linear_scale_error"] = {.1, -.2}; p["angular_scale_error"] = -.1;
  p["linear_bias_mps"] = {.01, -.02}; p["angular_bias_radps"] = .03;
  integrator.initialize(init, p);
  auto measured = integrator.update(velocity, velocity, 1000000000);
  const lp::Twist expected{.45, .14, 0, .48};
  EXPECT_NEAR(measured.velocity.vx, expected.vx, 1e-12);
  EXPECT_NEAR(measured.velocity.vy, expected.vy, 1e-12);
  EXPECT_NEAR(measured.velocity.wz, expected.wz, 1e-12);
  EXPECT_TRUE(measured.pose.matrix().isApprox(lp::integrate(lp::Pose::Identity(), expected, 1).matrix(), 1e-12));
  integrator.reset();
  const auto stationary = integrator.update({}, {}, 1000000000);
  EXPECT_EQ(stationary.velocity.vx, .01);
  EXPECT_EQ(stationary.velocity.wz, .03);

  p["linear_scale"] = {1.1, 1.0};
  EXPECT_THROW(integrator.initialize(init, p), std::runtime_error);
  p = lp::EncoderIntegrator::defaults(); p["angular_scale_error"] = -1;
  EXPECT_THROW(integrator.initialize(init, p), std::runtime_error);
  p["angular_scale_error"] = 0; p["linear_scale_error"] = {0, -1};
  EXPECT_THROW(integrator.initialize(init, p), std::runtime_error);
  p = lp::EncoderIntegrator::defaults(); p["linear_scale"] = {1.1, 1.0};
  integrator.initialize(init, p);
  EXPECT_NEAR(integrator.update(velocity, velocity, 1000000000).velocity.vx, .44, 1e-12);
}

namespace {
struct ParameterDirectory {
  std::filesystem::path path;
  ParameterDirectory() {
    path = std::filesystem::temp_directory_path() / ("litesim3d-plugin-parameters-" +
        std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    lp::require(std::filesystem::create_directory(path), "failed to create parameter test directory");
  }
  ~ParameterDirectory() { std::filesystem::remove_all(path); }
};
auto resolver = [](const std::string &uri, const std::filesystem::path &directory) { return directory / uri; };
}
TEST(PluginDeclarations, ParameterFilesSectionsAndInstanceOverridePrecedence) {
  ParameterDirectory temp;
  std::ofstream(temp.path / "defaults.yaml") << "radius: 0.1\nlimits: {speed: 1.0, acceleration: 0.5}\n";
  std::ofstream(temp.path / "robot.yaml") << "geometry: {radius: 0.2}\n";
  const auto parsed = lp::declarations(R"(<robot name="test"><litesim3d version="1">
    <chassis name="drive" class="external/Drive">
      <parameters file="defaults.yaml"/><parameters file="robot.yaml" section="geometry"/>
      <overrides>limits: {speed: 2.0}</overrides>
    </chassis></litesim3d></robot>)", temp.path, resolver);
  ASSERT_EQ(parsed.size(), 1u); EXPECT_EQ(parsed[0].class_name, "external/Drive");
  EXPECT_EQ(parsed[0].parameters["radius"], .2);
  EXPECT_EQ(parsed[0].parameters["limits"]["speed"], 2);
  EXPECT_EQ(parsed[0].parameters["limits"]["acceleration"], .5);
  EXPECT_EQ(parsed[0].parameter_files.size(), 2u);
}
TEST(PluginDeclarations, MultipleSensorBlocksAndNoSensorsAreSupported) {
  auto parsed = lp::declarations(R"(<robot name="test">
    <litesim3d version="1"><sensor name="a" class="external/Lidar"/></litesim3d>
    <litesim3d version="1"><sensor name="b" class="external/Lidar"/></litesim3d>
  </robot>)", {}, resolver);
  EXPECT_EQ(parsed.size(), 2u);
  EXPECT_TRUE(lp::declarations("<robot name='mechanical'/>", {}, resolver).empty());
}
TEST(PluginDeclarations, DuplicateNamesCategoriesUnknownFieldsAndMissingFileFail) {
  for (const std::string &children : {
      "<sensor name='a' class='X'/><sensor name='a' class='Y'/>",
      "<chassis name='a' class='X'/><chassis name='b' class='Y'/>",
      "<sensor name='a' clas='X'/>",
      "<sensor name='a' class='X' rate='1'/>",
      "<unknown name='a' class='X'/>",
      "<sensor name='a' class='X'><overrides>{}</overrides><parameters file='missing'/></sensor>",
      "<sensor name='a' class='X'><parameters file='missing'/></sensor>",
      "<sensor name='a' class='X'><overrides>[1,2]</overrides></sensor>",
      "<sensor name='a' class='X'><overrides>radius: 1\nradius: 2</overrides></sensor>"}) {
    SCOPED_TRACE(children);
    EXPECT_THROW(lp::declarations("<robot name='r'><litesim3d version='1'>" + children + "</litesim3d></robot>", {}, resolver), std::runtime_error);
  }
  EXPECT_THROW(lp::declarations("<robot><litesim3d version='2'/></robot>", {}, resolver), std::runtime_error);
}
