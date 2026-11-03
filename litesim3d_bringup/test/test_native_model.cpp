#include <gtest/gtest.h>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <map>
#include <optional>
#include <utility>
#include <vector>
#include "model.hpp"

using namespace litesim3d::bringup;
namespace {
// Native model units own the command boundary in this executable. Production
// model_preparation remains unchanged; no test in this target can launch a child
// through checked_run, including when a validation regression reaches that call.
thread_local const CommandRunner *command_provider = nullptr;
thread_local std::size_t command_calls = 0;
class ScopedCommandProvider {
 public:
  explicit ScopedCommandProvider(CommandRunner provider)
    : provider_(std::move(provider)), previous_(command_provider) { command_provider = &provider_; }
  ~ScopedCommandProvider() { command_provider = previous_; }
  ScopedCommandProvider(const ScopedCommandProvider &) = delete;
  ScopedCommandProvider &operator=(const ScopedCommandProvider &) = delete;
 private:
  CommandRunner provider_;
  const CommandRunner *previous_;
};

class ModelGeometry : public ::testing::Test {
 protected:
  void SetUp() override {
    if (const auto *value = std::getenv("AMENT_PREFIX_PATH")) previous_prefix_ = value;
    write(prefix_.path() / "share/ament_index/resource_index/packages/litesim3d_core", "");
    ASSERT_EQ(::setenv("AMENT_PREFIX_PATH", prefix_.path().c_str(), 1), 0);
    // A real ament lookup must succeed, so package absence cannot hide an
    // unintended call to checked_run in a validation-only test.
    ASSERT_EQ(ament_index_cpp::get_package_prefix("litesim3d_core"), prefix_.path().string());
    ASSERT_EQ(command_provider, nullptr);
    command_calls = 0;
  }
  void TearDown() override {
    EXPECT_EQ(command_provider, nullptr);
    EXPECT_EQ(previous_prefix_ ? ::setenv("AMENT_PREFIX_PATH", previous_prefix_->c_str(), 1) : ::unsetenv("AMENT_PREFIX_PATH"), 0);
  }
  TemporaryDirectory prefix_;
 private:
  std::optional<std::string> previous_prefix_;
};
}  // namespace

namespace litesim3d::runtime {
std::string checked_run(const std::vector<std::string> &command, double timeout, const std::string &input,
                        const std::map<std::string, std::string> &environment) {
  ++command_calls;
  if (command_provider) return (*command_provider)(command, timeout, input, environment);
  ADD_FAILURE() << "unexpected checked_run invocation in native model unit";
  throw std::runtime_error("native model unit forbids child processes");
}
}  // namespace litesim3d::runtime

namespace {
Json configuration(const std::string &kind) {
  const std::map<std::string, std::string> types{{"diff", "diff_drive_controller/DiffDriveController"},
    {"ack", "ackermann_steering_controller/AckermannSteeringController"}, {"omni", "litesim3d_ros2_control/OmniDriveController"}};
  Json bindings;
  if (kind == "diff") bindings = {{"left_wheel_names", {"left"}}, {"right_wheel_names", {"right"}}};
  else if (kind == "ack") bindings = {{"rear_wheels_names", {"right", "left"}}, {"front_wheels_names", {"steer_right", "steer_left"}}};
  else bindings = {{"wheel_joints", {"w0", "w1", "w2"}}};
  return {{"/manager", {{"ros__parameters", {{"drive", {{"type", types.at(kind)}}}}}}}, {"/ns/drive", {{"ros__parameters", bindings}}}};
}
Json geometry(const std::string &kind) {
  Json resolved;
  if (kind == "diff") resolved = {{"wheel_radius_m", .2}, {"wheel_separation_m", .8}};
  else if (kind == "ack") resolved = {{"wheelbase_m", .9}, {"track_width_m", .7}, {"wheel_radius_m", .2}};
  else resolved = {{"wheels", Json::array({{{"x_m", 1.}, {"y_m", 0.}, {"radius_m", .2}, {"rolling_angle_rad", 0.}},
    {{"x_m", -.5}, {"y_m", .8}, {"radius_m", .2}, {"rolling_angle_rad", 2.}},
    {{"x_m", -.5}, {"y_m", -.8}, {"radius_m", .2}, {"rolling_angle_rad", -2.}}})}};
  return {{"resolved", Json::array({resolved})}, {"joints", {{"w0", {{"max_velocity", 10.}}}, {"w1", {{"max_velocity", 20.}}}, {"w2", {{"max_velocity", 30.}}}}}};
}
}
TEST(ControllerParameters, DerivesGeometryAndPreservesInput) {
  for (const auto &kind : {"diff", "ack", "omni"}) {
    const auto source = configuration(kind); auto expected = source;
    int calls = 0;
    const auto result = controller_parameters(source, "description", "base_link", [&](auto description, auto base, auto requests) {
      ++calls; EXPECT_EQ(description, "description"); EXPECT_EQ(base, "base_link"); EXPECT_EQ(requests.size(), 1u);
      if (std::string(kind) == "ack") EXPECT_EQ(requests[0]["parameters"]["rear_left_joint"], "left");
      return geometry(kind);
    });
    EXPECT_EQ(source, expected); EXPECT_EQ(calls, 1);
    const auto &p = result["/ns/drive"]["ros__parameters"];
    if (std::string(kind) == "diff") { EXPECT_EQ(p["wheel_radius"], .2); EXPECT_EQ(p["wheel_separation"], .8); }
    else if (std::string(kind) == "ack") { EXPECT_EQ(p["wheelbase"], .9); EXPECT_EQ(p["rear_wheel_track"], .7); }
    else { EXPECT_EQ(p["max_wheel_velocity"], Json({10., 20., 30.})); EXPECT_EQ(p["wheel_angle"], Json({0., 2., -2.})); }
  }
}
TEST(ControllerParameters, RejectsMalformedBindingsBeforeNativeInspection) {
  const GeometryProvider never = [](auto, auto, auto) -> Json { ADD_FAILURE() << "unexpected geometry request"; return {}; };
  for (const auto &bad : {Json(), Json::array(), Json("bad")}) EXPECT_THROW(controller_parameters(bad, "", "base_link", never), std::runtime_error);
  for (const auto &bad : {Json(), Json("left"), Json::array(), Json({""}), Json({1}), Json({"left", "left"})}) {
    auto source = configuration("diff"); source["/ns/drive"]["ros__parameters"]["left_wheel_names"] = bad;
    EXPECT_THROW(controller_parameters(source, "", "base_link", never), std::runtime_error);
  }
  for (int failure = 0; failure < 7; ++failure) {
    auto source = configuration("ack");
    if (failure == 0) source["/other"] = {{"ros__parameters", {{"drive", {{"type", "vendor/Other"}}}}}};
    if (failure == 1) source["/drive"] = source["/ns/drive"];
    if (failure == 2) source.erase("/ns/drive");
    if (failure == 3) source["/ns/drive"] = 1;
    if (failure == 4) source["/ns/drive"]["ros__parameters"] = Json::array();
    if (failure == 5) source["/ns/drive"]["ros__parameters"]["base_frame_id"] = "wrong";
    if (failure == 6) source["/ns/drive"]["ros__parameters"]["front_steering"] = false;
    EXPECT_THROW(controller_parameters(source, "", "base_link", never), std::runtime_error);
  }
  Json custom = {{"plain", 1}, {"bad_parameters", {{"ros__parameters", Json::array()}}},
    {"manager", {{"ros__parameters", {{"unknown", {{"type", "vendor/Unknown"}}}, {"gain", 3}}}}}};
  EXPECT_EQ(controller_parameters(custom, "", "base_link", never), custom);
}
TEST(ControllerParameters, GeometryConflictLimitsAnglesAndProviderCardinality) {
  for (const auto &bad : {Json(true), Json(".2"), Json(nullptr), Json(.1)}) {
    auto source = configuration("diff"); source["/ns/drive"]["ros__parameters"]["wheel_radius"] = bad;
    EXPECT_THROW(controller_parameters(source, "", "base_link", [](auto, auto, auto) { return geometry("diff"); }), std::runtime_error);
  }
  auto omni = configuration("omni"); auto &parameters = omni["/ns/drive"]["ros__parameters"];
  parameters["wheel_angle"] = {2 * std::acos(-1), 2., -2.}; parameters["max_wheel_velocity"] = {5., 10., 15.};
  const auto result = controller_parameters(omni, "", "base_link", [](auto, auto, auto) { return geometry("omni"); });
  EXPECT_EQ(result["/ns/drive"]["ros__parameters"]["wheel_angle"], Json({0., 2., -2.}));
  for (const auto &bad : {Json("bad"), Json::array(), Json({1., 2.}), Json({true, 2., 3.}), Json({0., 2., 3.}), Json({11., 2., 3.})}) {
    parameters["max_wheel_velocity"] = bad;
    EXPECT_THROW(controller_parameters(omni, "", "base_link", [](auto, auto, auto) { return geometry("omni"); }), std::runtime_error);
  }
  parameters.erase("max_wheel_velocity");
  for (const auto &bad : {Json(nullptr), Json("fast"), Json(0), Json(-1)}) {
    auto provider = geometry("omni"); provider["joints"]["w1"]["max_velocity"] = bad;
    EXPECT_THROW(controller_parameters(omni, "", "base_link", [&](auto, auto, auto) { return provider; }), std::runtime_error);
  }
  EXPECT_THROW(controller_parameters(omni, "", "base_link", [](auto, auto, auto) { return Json{{"resolved", Json::array()}}; }), std::runtime_error);
}
TEST(Model, ExpandsWithExplicitBoundaryAndPinsResources) {
  TemporaryDirectory temporary; const auto root = temporary.path();
  write(root / "mesh.dae", "mesh"); write(root / "texture.png", "texture"); write(root / "pattern.csv", "pattern");
  write(root / "lidar.yaml", "nested: {lidar: {pattern_file: pattern.csv, topic: scan}}\n");
  write(root / "source.xacro", "source"); write(root / "runtime.yaml", "robot: {urdf: source.xacro}\n");
  const std::string xml = "<robot><link name='base_link'><visual><geometry><mesh filename='mesh.dae'/></geometry>"
    "<material><texture filename='texture.png'/></material></visual></link><litesim3d>"
    "<sensor name='scan' class='litesim3d_lidar/Lidar'><parameters file='lidar.yaml' section='nested.lidar'/>"
    "<overrides>pattern_file: pattern.csv</overrides></sensor></litesim3d></robot>";
  int calls = 0;
  const auto config = prepare_config(root / "runtime.yaml", root / "out", {{"mappings", {{"enabled", false}, {"count", 3}}}},
    [&](const auto &command, auto timeout, auto input, auto environment) {
      ++calls; EXPECT_EQ(command[0], "xacro"); EXPECT_EQ(command[2], "enabled:=false"); EXPECT_EQ(command[3], "count:=3");
      EXPECT_EQ(timeout, 60); EXPECT_TRUE(input.empty()); EXPECT_TRUE(environment.empty()); return xml;
    });
  EXPECT_EQ(calls, 1); EXPECT_EQ(read(root / "source.xacro"), "source");
  tinyxml2::XMLDocument document; parse_xml(document, read(read_yaml(config)["robot"]["urdf"].get<std::string>()));
  auto *sensor = document.RootElement()->FirstChildElement("litesim3d")->FirstChildElement("sensor");
  const auto pinned = read_yaml(sensor->FirstChildElement("parameters")->Attribute("file"));
  EXPECT_EQ(pinned["nested"]["lidar"]["pattern_file"], (root / "pattern.csv").string());
  EXPECT_EQ(load_yaml(sensor->FirstChildElement("overrides")->GetText())["pattern_file"], (root / "pattern.csv").string());
  EXPECT_NE(ros_description(read_yaml(config)["robot"]["urdf"].get<std::string>()).find("file://"), std::string::npos);
}
TEST(Model, NoiseAndProfileBindingsCannotBeReplaced) {
  TemporaryDirectory temporary; const auto root = temporary.path();
  tinyxml2::XMLDocument document;
  parse_xml(document, "<robot><litesim3d><sensor name='scan' class='litesim3d_mid360/Mid360'/>"
    "<odometry name='encoders' class='litesim3d_omni_odometry/OmniOdometry'/></litesim3d></robot>");
  write(root / "noise.yaml", "sensors: {scan: {range_sigma_m: 0.02}}\nodometry: {angular_bias_radps: 0.01}\n");
  auto data = Json::object(); apply_noise(data, document.RootElement(), root / "noise.yaml", root);
  auto *sensor = document.RootElement()->FirstChildElement("litesim3d")->FirstChildElement("sensor");
  EXPECT_EQ(load_yaml(sensor->FirstChildElement("overrides")->GetText())["range_sigma_m"], .02);
  for (const auto &noise : {"[]", "simulation: {}", "sensors: []", "sensors: {missing: {}}", "sensors: {scan: {topic: changed}}"}) {
    write(root / "noise.yaml", noise); EXPECT_THROW(apply_noise(data, document.RootElement(), root / "noise.yaml", root), std::runtime_error);
  }
  write(root / "profile.yaml", "lidar: {name: overwritten}"); EXPECT_THROW(profile((root / "profile.yaml").string(), ""), std::runtime_error);
  write(root / "profile.yaml", "lidar: {columns: 16}"); EXPECT_EQ(profile((root / "profile.yaml").string(), "")["columns"], 16);
  EXPECT_THROW(profile("mid360", ""), std::runtime_error);
  EXPECT_THROW(profile("missing-file", ""), std::runtime_error);
  EXPECT_THROW(parse_xml(document, "<broken"), std::runtime_error);
}
TEST(Model, CameraAndLidarSelectionReachPreparedModelAndPreview) {
  TemporaryDirectory temporary; const auto root = temporary.path();
  write(root / "model.urdf", "<robot><litesim3d><sensor name='scan' class='litesim3d_lidar/Lidar'><overrides>topic: scan</overrides></sensor>"
    "<sensor name='camera' class='litesim3d_rgbd_camera/RgbdCamera'/></litesim3d></robot>");
  write(root / "runtime.yaml", "robot: {urdf: model.urdf}\n");
  write(root / "profile.yaml", "lidar: {scan_pattern: livox_mid360, rings: 1, columns: 20000}\n");
  const auto prepared = prepare_config(root / "runtime.yaml", root / "out", {{"lidar_profile", (root / "profile.yaml").string()}, {"cameras", "false"}});
  EXPECT_EQ(preview_topics(prepared, "auto", 1)["/scan"], "/livox/lidar/pointcloud");
  EXPECT_EQ(preview_topics(prepared, "livox", 0)["/scan"], "/livox/lidar");
  EXPECT_TRUE(preview_topics(prepared, "native", 1).empty());
  EXPECT_THROW(prepare_config(root / "runtime.yaml", root / "bad", {{"mappings", {1}}}), std::runtime_error);
  EXPECT_THROW(prepare_config(root / "runtime.yaml", root / "bad", {{"mappings", {{"arg", true}}}}), std::runtime_error);
}
TEST(Model, EmptyOptionalPathsArePreserved) {
  TemporaryDirectory temporary; const auto root = temporary.path();
  write(root / "runtime.yaml", "robot: {urdf: ''}\nlidars: [{name: scan, pattern_file: ''}]\n");
  const auto output = prepare_config(root / "runtime.yaml", root / "out", Json::object());
  const auto data = read_yaml(output);
  EXPECT_EQ(data["robot"]["urdf"], ""); EXPECT_EQ(data["lidars"][0]["pattern_file"], "");
  EXPECT_THROW(prepare_config(root / "runtime.yaml", root / "bad", {{"mappings", {{"a", 1}}}}), std::runtime_error);
}

TEST(ControllerParameters, CollectsMultipleControllersInStableOrderAndRetainsUnrelatedSettings) {
  Json source = {{"manager", {{"ros__parameters", {
    {"ack", {{"type", "ackermann_steering_controller/AckermannSteeringController"}}},
    {"diff", {{"type", "diff_drive_controller/DiffDriveController"}}},
    {"omni", {{"type", "litesim3d_ros2_control/OmniDriveController"}}},
    {"invalid_type", {{"type", 5}}}, {"no_type", {{"gain", 2}}}, {"rate", 100}}}}}};
  source["duplicate_manager"] = source["manager"];
  for (const auto &kind : {"ack", "diff", "omni"}) {
    source[std::string("/robot/") + kind + "///"] = configuration(kind)["/ns/drive"];
    source[std::string("/robot/") + kind + "///"]["ros__parameters"]["base_frame_id"] = "chassis";
  }
  source["/robot/ack///"]["ros__parameters"]["front_steering"] = true;
  source["/robot/diff///"]["ros__parameters"]["left_wheel_names"] = {"left_front", "left_rear"};
  source["/robot/diff///"]["ros__parameters"]["unrelated"] = "keep";
  const auto original = source;
  int calls = 0;
  const auto result = controller_parameters(source, "actual description", "chassis", [&](auto description, auto base, auto requests) {
    ++calls;
    EXPECT_EQ(description, "actual description"); EXPECT_EQ(base, "chassis");
    EXPECT_EQ(requests, Json::array({
      {{"kind", "ackermann"}, {"parameters", {{"rear_left_joint", "left"}, {"rear_right_joint", "right"},
        {"left_steering_joint", "steer_left"}, {"right_steering_joint", "steer_right"}}}},
      {{"kind", "differential"}, {"parameters", {{"left_joints", {"left_front", "left_rear"}}, {"right_joints", {"right"}}}}},
      {{"kind", "omni"}, {"parameters", {{"wheels", Json::array({{{"joint", "w0"}}, {{"joint", "w1"}}, {{"joint", "w2"}}})}}}}
    }));
    auto combined = geometry("omni");
    combined["resolved"] = Json::array({geometry("ack")["resolved"][0], geometry("diff")["resolved"][0], geometry("omni")["resolved"][0]});
    return combined;
  });
  EXPECT_EQ(calls, 1); EXPECT_EQ(source, original);
  EXPECT_EQ(result["manager"], original["manager"]);
  EXPECT_EQ(result["/robot/diff///"]["ros__parameters"]["unrelated"], "keep");
  const auto &ack = result["/robot/ack///"]["ros__parameters"];
  EXPECT_EQ(ack["wheelbase"], .9); EXPECT_EQ(ack["front_wheel_track"], .7);
  EXPECT_EQ(ack["rear_wheel_track"], .7); EXPECT_EQ(ack["front_wheels_radius"], .2); EXPECT_EQ(ack["rear_wheels_radius"], .2);
  EXPECT_EQ(result["/robot/omni///"]["ros__parameters"]["wheel_y"], Json({0., .8, -.8}));
}

TEST(ControllerParameters, RequiresEveryBindingAndExactAckermannWheelCounts) {
  const GeometryProvider never = [](auto, auto, auto) -> Json { ADD_FAILURE() << "invalid bindings reached provider"; return {}; };
  for (const auto &entry : std::vector<std::pair<std::string, std::string>>{
      {"diff", "left_wheel_names"}, {"diff", "right_wheel_names"}, {"ack", "rear_wheels_names"},
      {"ack", "front_wheels_names"}, {"omni", "wheel_joints"}}) {
    SCOPED_TRACE(entry.first + ":" + entry.second);
    auto source = configuration(entry.first); source["/ns/drive"]["ros__parameters"].erase(entry.second);
    EXPECT_THROW(controller_parameters(source, "", "base", never), std::runtime_error);
  }
  for (const auto &key : {"rear_wheels_names", "front_wheels_names"}) {
    for (const auto &bad : {Json({"one"}), Json({"one", "two", "three"})}) {
      SCOPED_TRACE(std::string(key) + bad.dump());
      auto source = configuration("ack"); source["/ns/drive"]["ros__parameters"][key] = bad;
      EXPECT_THROW(controller_parameters(source, "", "base", never), std::runtime_error);
    }
  }
  auto missing_parameters = configuration("diff"); missing_parameters["/ns/drive"].erase("ros__parameters");
  EXPECT_THROW(controller_parameters(missing_parameters, "", "base", never), std::runtime_error);
}

TEST(ControllerParameters, RejectsProviderEnvelopesAndPropagatesFailureWithoutMutatingInput) {
  const auto source = configuration("diff"); const auto original = source;
  for (const auto &bad : {Json(), Json::array(), Json::object(), Json{{"resolved", nullptr}},
      Json{{"resolved", Json::object()}}, Json{{"resolved", Json::array({{}, {}})}}}) {
    SCOPED_TRACE(bad.dump());
    EXPECT_THROW(controller_parameters(source, "", "base", [&](auto, auto, auto) { return bad; }), std::runtime_error);
  }
  EXPECT_THROW(controller_parameters(source, "", "base", [](auto, auto, auto) -> Json {
    throw std::runtime_error("geometry unavailable");
  }), std::runtime_error);
  EXPECT_THROW(controller_parameters(source, "", "base", [](auto, auto, auto) {
    return Json{{"resolved", Json::array({Json::object()})}};
  }), Json::out_of_range);
  EXPECT_EQ(source, original);
}

TEST(ControllerParameters, ComparesFiniteGeometryArraysAndNormalizesWrappedAngles) {
  auto source = configuration("omni"); auto &p = source["/ns/drive"]["ros__parameters"];
  p["wheel_x"] = {1. + 1e-8, -.5, -.5}; p["wheel_y"] = {0., .8, -.8};
  p["wheel_radius"] = {.2, .2, .2}; p["wheel_angle"] = {0., 2. + 2 * std::acos(-1), -2.};
  p["max_wheel_velocity"] = {10., 20., 30.};
  const auto result = controller_parameters(source, "", "base", [](auto, auto, auto) { return geometry("omni"); });
  EXPECT_EQ(result["/ns/drive"]["ros__parameters"]["wheel_x"], Json({1., -.5, -.5}));
  EXPECT_EQ(result["/ns/drive"]["ros__parameters"]["wheel_angle"], Json({0., 2., -2.}));
  EXPECT_EQ(result["/ns/drive"]["ros__parameters"]["max_wheel_velocity"], p["max_wheel_velocity"]);
  for (const auto &bad : {Json(1), Json({1., -.5}), Json({1., -.5, "bad"}), Json({1., -.5, -.3}),
      Json({1., -.5, std::numeric_limits<double>::infinity()}), Json({1., -.5, std::numeric_limits<double>::quiet_NaN()})}) {
    SCOPED_TRACE(bad.dump()); p["wheel_x"] = bad;
    EXPECT_THROW(controller_parameters(source, "", "base", [](auto, auto, auto) { return geometry("omni"); }), std::runtime_error);
  }
  auto differential = configuration("diff");
  differential["/ns/drive"]["ros__parameters"]["wheel_radius"] = .2;
  for (double bad : {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
    auto supplied = geometry("diff"); supplied["resolved"][0]["wheel_radius_m"] = bad;
    EXPECT_THROW(controller_parameters(differential, "", "base", [&](auto, auto, auto) { return supplied; }), std::runtime_error);
    supplied["resolved"][0]["wheel_radius_m"] = "bad";
    EXPECT_THROW(controller_parameters(differential, "", "base", [&](auto, auto, auto) { return supplied; }), std::runtime_error);
  }
}

TEST(ControllerParameters, RejectsNonfiniteVelocityLimitsAndCaps) {
  for (double bad : {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN(), -1.}) {
    auto source = configuration("omni"); auto supplied = geometry("omni");
    supplied["joints"]["w1"]["max_velocity"] = bad;
    EXPECT_THROW(controller_parameters(source, "", "base", [&](auto, auto, auto) { return supplied; }), std::runtime_error);
    source["/ns/drive"]["ros__parameters"]["max_wheel_velocity"] = {1., bad, 1.};
    EXPECT_THROW(controller_parameters(source, "", "base", [](auto, auto, auto) { return geometry("omni"); }), std::runtime_error);
  }
}

TEST_F(ModelGeometry, RejectsInvalidXmlAndMissingResourcesBeforeExecutingExternalTool) {
  tinyxml2::XMLDocument reference;
  ASSERT_NE(reference.Parse("<broken"), tinyxml2::XML_SUCCESS);
  const auto missing = (prefix_.path() / "missing.stl").string();
  const std::vector<std::pair<std::string, std::string>> invalid = {
    {"<broken", std::string("invalid XML: ") + reference.ErrorStr()},
    {"<!-- no root -->", "empty XML document"},
    {"<robot><mesh filename='" + missing + "'/></robot>", "resource not found: " + missing}};
  for (const auto &[description, diagnostic] : invalid) {
    SCOPED_TRACE(description);
    try {
      model_geometry(description, "base", Json::array());
      FAIL() << "invalid model unexpectedly accepted";
    } catch (const std::runtime_error &error) {
      EXPECT_EQ(error.what(), diagnostic);
    }
    EXPECT_EQ(command_calls, 0u);
  }
}

TEST_F(ModelGeometry, InvokesCommandBoundaryWithPinnedTemporaryModelAndExactRequest) {
  const auto mesh = prefix_.path() / "mesh with space.stl";
  write(mesh, "original mesh");
  const auto description = "<robot><link name='base'><mesh filename='" + file_uri(mesh) + "'/></link></robot>";
  const auto requests = Json::array({{{"kind", "differential"}, {"parameters", {{"left_joints", {"left"}}, {"right_joints", {"right"}}}}}});
  const auto expected = geometry("diff");
  fs::path staged_model;
  ScopedCommandProvider provider([&](const auto &command, auto timeout, auto input, auto environment) {
    EXPECT_EQ(command.size(), 4u);
    if (command.size() != 4) throw std::runtime_error("invalid model_geometry command");
    EXPECT_EQ(command[0], (prefix_.path() / "bin/model_geometry").string());
    EXPECT_EQ(command[2], "reference_frame"); EXPECT_EQ(command[3], "--resolve");
    EXPECT_EQ(timeout, 60); EXPECT_EQ(input, requests.dump()); EXPECT_TRUE(environment.empty());
    staged_model = command[1];
    EXPECT_EQ(staged_model.filename(), "robot.urdf"); EXPECT_TRUE(fs::is_regular_file(staged_model));
    tinyxml2::XMLDocument actual; parse_xml(actual, read(staged_model));
    auto *root = actual.RootElement();
    EXPECT_NE(root, nullptr); if (!root) throw std::runtime_error("staged model has no root");
    auto *link = root->FirstChildElement("link");
    EXPECT_NE(link, nullptr); if (!link) throw std::runtime_error("staged model has no link");
    auto *resource = link->FirstChildElement("mesh");
    EXPECT_NE(resource, nullptr); if (!resource) throw std::runtime_error("staged model has no mesh");
    const auto *filename = resource->Attribute("filename");
    EXPECT_NE(filename, nullptr); if (!filename) throw std::runtime_error("staged mesh has no filename");
    EXPECT_EQ(filename, mesh.string());
    return expected.dump();
  });
  EXPECT_EQ(model_geometry(description, "reference_frame", requests), expected);
  EXPECT_EQ(command_calls, 1u); EXPECT_FALSE(staged_model.empty());
  EXPECT_FALSE(fs::exists(staged_model.parent_path())); EXPECT_EQ(read(mesh), "original mesh");
  EXPECT_TRUE(fs::is_directory(prefix_.path()));
}

TEST_F(ModelGeometry, CleansTemporaryModelWhenCommandBoundaryFailsOrReturnsInvalidJson) {
  for (bool throw_from_provider : {false, true}) {
    SCOPED_TRACE(throw_from_provider);
    fs::path staged_model;
    ScopedCommandProvider provider([&](const auto &command, auto, auto, auto) -> std::string {
      EXPECT_EQ(command.size(), 4u);
      if (command.size() != 4) throw std::runtime_error("invalid model_geometry command");
      staged_model = command[1]; EXPECT_TRUE(fs::is_regular_file(staged_model));
      if (throw_from_provider) throw std::runtime_error("geometry provider failed");
      return "not JSON";
    });
    if (throw_from_provider) {
      try { model_geometry("<robot/>", "base", Json::array()); FAIL() << "provider failure was swallowed"; }
      catch (const std::runtime_error &error) { EXPECT_STREQ(error.what(), "geometry provider failed"); }
    } else EXPECT_THROW(model_geometry("<robot/>", "base", Json::array()), Json::parse_error);
    EXPECT_FALSE(staged_model.empty()); EXPECT_FALSE(fs::exists(staged_model.parent_path()));
  }
  EXPECT_EQ(command_calls, 2u);
}

TEST(ModelXml, RejectsRootlessDocumentsAndSerializesOnlyRequestedNode) {
  tinyxml2::XMLDocument document;
  EXPECT_THROW(parse_xml(document, "<!-- no root -->"), std::runtime_error);
  parse_xml(document, "<?xml version='1.0'?><robot><link name='base'/></robot>");
  EXPECT_EQ(xml_text(*document.RootElement()->FirstChildElement()), "<link name=\"base\"/>");
  EXPECT_NO_THROW(pin_resources(nullptr, {}));
}

TEST(ModelXml, PinsNestedResourcesAndAllPluginParameterFilesWithoutTouchingOtherAttributes) {
  TemporaryDirectory directory; const auto root = directory.path();
  for (const auto &name : {"mesh.stl", "texture.png", "first.yaml", "second.yaml", "third.yaml"}) write(root / name, "data");
  tinyxml2::XMLDocument document;
  parse_xml(document, "<robot><link filename='untouched'><visual><mesh filename='mesh.stl'/><texture filename='texture.png'/><mesh/></visual></link>"
    "<litesim3d><sensor><parameters file='first.yaml'/><parameters/><parameters file='second.yaml'/></sensor></litesim3d>"
    "<litesim3d><odometry><parameters file='third.yaml'/></odometry></litesim3d></robot>");
  pin_resources(document.RootElement(), root);
  auto *link = document.RootElement()->FirstChildElement("link");
  EXPECT_STREQ(link->Attribute("filename"), "untouched");
  auto *visual = link->FirstChildElement();
  EXPECT_EQ(visual->FirstChildElement("mesh")->Attribute("filename"), (root / "mesh.stl").string());
  EXPECT_EQ(visual->FirstChildElement("texture")->Attribute("filename"), (root / "texture.png").string());
  auto *sensor = document.RootElement()->FirstChildElement("litesim3d")->FirstChildElement();
  EXPECT_EQ(sensor->FirstChildElement()->Attribute("file"), (root / "first.yaml").string());
  EXPECT_EQ(sensor->LastChildElement()->Attribute("file"), (root / "second.yaml").string());
  EXPECT_EQ(document.RootElement()->LastChildElement()->FirstChildElement()->FirstChildElement()->Attribute("file"), (root / "third.yaml").string());
}

TEST(ModelXml, PluginOverridesDeepMergeAndPreserveBindings) {
  tinyxml2::XMLDocument document;
  parse_xml(document, "<sensor name='scan' class='driver'><overrides>topic: scan\nnoise: {a: 1, b: 2}</overrides></sensor>");
  auto *sensor = document.RootElement();
  override_plugin(sensor, {{"noise", {{"a", 5}, {"c", 3}}}, {"enabled", false}});
  EXPECT_EQ(load_yaml(sensor->FirstChildElement("overrides")->GetText()),
    (Json{{"topic", "scan"}, {"noise", {{"a", 5}, {"b", 2}, {"c", 3}}}, {"enabled", false}}));
  EXPECT_STREQ(sensor->Attribute("name"), "scan"); EXPECT_STREQ(sensor->Attribute("class"), "driver");
  parse_xml(document, "<sensor><overrides/></sensor>");
  override_plugin(document.RootElement(), {{"enabled", true}});
  EXPECT_EQ(load_yaml(document.RootElement()->FirstChildElement()->GetText()), (Json{{"enabled", true}}));
}

TEST(ModelXml, InvalidOverridesFailWithoutReplacingExistingXml) {
  for (const auto &xml : {"<sensor><overrides/><overrides/></sensor>", "<sensor><overrides>[]</overrides></sensor>",
      "<sensor><overrides>null</overrides></sensor>", "<sensor><overrides>5</overrides></sensor>"}) {
    SCOPED_TRACE(xml); tinyxml2::XMLDocument document; parse_xml(document, xml); const auto before = xml_text(document);
    EXPECT_THROW(override_plugin(document.RootElement(), Json::object()), std::runtime_error);
    EXPECT_EQ(xml_text(document), before);
  }
  for (const auto &bad : {Json(), Json::array(), Json(1)}) {
    tinyxml2::XMLDocument document; parse_xml(document, "<sensor/>");
    EXPECT_THROW(override_plugin(document.RootElement(), bad), std::runtime_error);
    EXPECT_EQ(document.RootElement()->FirstChildElement(), nullptr);
  }
}

TEST(ModelProfile, ResolvesNamedProfilesAndRelativePatternsWithoutMutatingFiles) {
  TemporaryDirectory directory; const auto root = directory.path(); write(root / "scan.csv", "pattern");
  for (const auto &name : {"vlp16", "mid360", "mid40"}) {
    SCOPED_TRACE(name); const std::string text = "lidar: {pattern_file: scan.csv, columns: 16}\n";
    write(root / (std::string(name) + ".yaml"), text);
    EXPECT_EQ(profile(name, root), (Json{{"pattern_file", (root / "scan.csv").string()}, {"columns", 16}}));
    EXPECT_EQ(read(root / (std::string(name) + ".yaml")), text);
  }
  for (const auto &value : {"null", "''"}) {
    write(root / "empty.yaml", std::string("lidar: {pattern_file: ") + value + "}");
    EXPECT_EQ(profile((root / "empty.yaml").string(), {}), read_yaml(root / "empty.yaml")["lidar"]);
  }
}

TEST(ModelProfile, RejectsMalformedProfilesAndAllBindingOverrides) {
  TemporaryDirectory directory; const auto filename = directory.path() / "profile.yaml";
  for (const auto &text : {"[]", "{}", "lidar: []", "lidar: {}\nextra: {}", "other: {}", "lidar: {link: base}",
      "lidar: {topic: scan}", "lidar: {name: scan}", "lidar: {pattern_file: missing.csv}"}) {
    SCOPED_TRACE(text); write(filename, text); EXPECT_THROW(profile(filename.string(), {}), std::runtime_error);
  }
  EXPECT_THROW(profile("vlp16", directory.path() / "missing"), std::runtime_error);
}

TEST(ModelNoise, AppliesSupportedLegacySensorFieldsWithoutReplacingBindings) {
  TemporaryDirectory directory; const auto root = directory.path();
  auto data = Json{{"lidars", Json::array({{{"name", "scan"}, {"topic", "scan"}}})},
    {"imus", Json::array({{{"name", "imu"}, {"link", "imu_link"}}})},
    {"cameras", Json::array({{{"name", "rgb"}}, {{"name", "depth"}, {"type", "depth"}}, {{"name", "rgbd"}, {"type", "rgbd"}}})}};
  const Json settings = {{"scan", {{"range_bias_m", .1}, {"range_sigma_m", .2}, {"dropout", .3}}},
    {"imu", {{"accel_bias", {1., 2., 3.}}, {"gyro_bias", {0., 0., 0.}}, {"accel_sigma", .1}, {"gyro_sigma", .2}}},
    {"rgb", {{"color_bias_dn", 1}, {"color_sigma_dn", 2}}},
    {"depth", {{"depth_bias_m", .1}, {"depth_sigma_m", .2}}},
    {"rgbd", {{"color_bias_dn", 3}, {"color_sigma_dn", 4}, {"depth_bias_m", .3}, {"depth_sigma_m", .4}}}};
  write(root / "noise.yaml", dump_yaml({{"sensors", settings}}));
  apply_noise(data, nullptr, "noise.yaml", root);
  EXPECT_EQ(data["lidars"][0]["topic"], "scan"); EXPECT_EQ(data["imus"][0]["link"], "imu_link");
  for (const auto &group : {"lidars", "imus", "cameras"}) for (const auto &sensor : data[group])
    for (auto it = settings.at(sensor.at("name").get<std::string>()).begin(); it != settings.at(sensor.at("name").get<std::string>()).end(); ++it)
      EXPECT_EQ(sensor.at(it.key()), it.value());
}

TEST(ModelNoise, AppliesEveryStandardOdometryClassAndPluginSensorKind) {
  TemporaryDirectory directory; const auto filename = directory.path() / "noise.yaml";
  const Json noise = {{"linear_scale_error", .01}, {"angular_scale_error", .02}, {"linear_bias_mps", .03},
    {"linear_sigma_mps", .04}, {"angular_bias_radps", .05}, {"angular_sigma_radps", .06}};
  for (const auto &implementation : {"litesim3d_diff_odometry/DifferentialOdometry", "litesim3d_ackermann_odometry/AckermannOdometry",
      "litesim3d_omni_odometry/OmniOdometry"}) {
    tinyxml2::XMLDocument document;
    parse_xml(document, std::string("<robot><litesim3d><odometry class='") + implementation + "'><overrides>topic: odom</overrides></odometry></litesim3d></robot>");
    write(filename, dump_yaml({{"odometry", noise}})); auto data = Json::object();
    apply_noise(data, document.RootElement(), filename, {});
    EXPECT_EQ(load_yaml(document.RootElement()->FirstChildElement()->FirstChildElement()->FirstChildElement()->GetText()),
      (Json{{"topic", "odom"}, {"noise", noise}}));
  }
  for (const auto &entry : std::vector<std::pair<std::string, Json>>{
      {"litesim3d_lidar/Lidar", {{"dropout", .1}}}, {"litesim3d_mid360/Mid360", {{"range_bias_m", .1}}},
      {"litesim3d_imu/Imu", {{"gyro_sigma", .1}}}, {"litesim3d_rgb_camera/RgbCamera", {{"color_sigma_dn", 1}}},
      {"litesim3d_depth_camera/DepthCamera", {{"depth_sigma_m", .1}}},
      {"litesim3d_rgbd_camera/RgbdCamera", {{"depth_bias_m", .1}, {"color_bias_dn", 1}}}}) {
    SCOPED_TRACE(entry.first); tinyxml2::XMLDocument document;
    parse_xml(document, "<robot><litesim3d><sensor name='sensor' class='" + entry.first + "'/></litesim3d></robot>");
    write(filename, dump_yaml({{"sensors", {{"sensor", entry.second}}}})); auto data = Json::object();
    apply_noise(data, document.RootElement(), filename, {});
    EXPECT_EQ(load_yaml(document.RootElement()->FirstChildElement()->FirstChildElement()->FirstChildElement()->GetText()), entry.second);
  }
}

TEST(ModelNoise, RejectsAmbiguousNamesUnsupportedTypesAndNonMappingSettings) {
  TemporaryDirectory directory; const auto filename = directory.path() / "noise.yaml";
  for (const auto &xml : {"<robot><litesim3d><sensor name='scan'/><sensor name='scan'/></litesim3d></robot>",
      "<robot><litesim3d><sensor name='scan' class='custom/Sensor'/></litesim3d></robot>"}) {
    SCOPED_TRACE(xml); tinyxml2::XMLDocument document; parse_xml(document, xml);
    write(filename, "sensors: {scan: {}}\n"); auto data = Json::object();
    EXPECT_THROW(apply_noise(data, document.RootElement(), filename, {}), std::runtime_error);
  }
  tinyxml2::XMLDocument document;
  parse_xml(document, "<robot><litesim3d><sensor name='scan' class='litesim3d_lidar/Lidar'/></litesim3d></robot>");
  auto duplicate = Json{{"lidars", Json::array({{{"name", "scan"}}})}};
  EXPECT_THROW(apply_noise(duplicate, document.RootElement(), filename, {}), std::runtime_error);
  duplicate = {{"imus", Json::array({{{"name", "imu"}}, {{"name", "imu"}}})}};
  EXPECT_THROW(apply_noise(duplicate, nullptr, filename, {}), std::runtime_error);
  for (const auto &text : {"sensors: {scan: []}", "sensors: {scan: null}", "sensors: {scan: {enabled: false}}"}) {
    SCOPED_TRACE(text); write(filename, text); auto data = Json::object();
    EXPECT_THROW(apply_noise(data, document.RootElement(), filename, {}), std::runtime_error);
    EXPECT_EQ(document.RootElement()->FirstChildElement()->FirstChildElement()->FirstChildElement(), nullptr);
  }
}

TEST(ModelNoise, RequiresExactlyOneSupportedOdometryAndValidatedFields) {
  TemporaryDirectory directory; const auto filename = directory.path() / "noise.yaml";
  write(filename, "odometry: {}\n"); auto data = Json::object();
  EXPECT_THROW(apply_noise(data, nullptr, filename, {}), std::runtime_error);
  for (const auto &xml : {"<robot/>", "<robot><litesim3d><odometry class='custom/Odom'/></litesim3d></robot>",
      "<robot><litesim3d><odometry/><odometry/></litesim3d></robot>"}) {
    tinyxml2::XMLDocument document; parse_xml(document, xml);
    EXPECT_THROW(apply_noise(data, document.RootElement(), filename, {}), std::runtime_error);
  }
  tinyxml2::XMLDocument document;
  parse_xml(document, "<robot><litesim3d><odometry class='litesim3d_diff_odometry/DifferentialOdometry'/></litesim3d></robot>");
  for (const auto &text : {"odometry: []", "odometry: {topic: wrong}"}) {
    write(filename, text); EXPECT_THROW(apply_noise(data, document.RootElement(), filename, {}), std::runtime_error);
  }
  write(filename, "{}\n"); EXPECT_NO_THROW(apply_noise(data, nullptr, filename, {})); EXPECT_TRUE(data.empty());
}

TEST(ModelPreparation, RejectsInvalidConfigurationAndOverwritingSourceOrModelDirectory) {
  TemporaryDirectory directory; const auto root = directory.path();
  for (const auto &text : {"[]", "{}", "robot: []", "robot: null"}) {
    SCOPED_TRACE(text); write(root / "runtime.yaml", text);
    EXPECT_THROW(prepare_config(root / "runtime.yaml", root / "output", Json::object()), std::runtime_error);
  }
  write(root / "runtime.yaml", "robot: {}\n");
  EXPECT_THROW(prepare_config(root / "runtime.yaml", root, Json::object()), std::runtime_error);
  write(root / "models/model.urdf", "<robot/>");
  write(root / "runtime.yaml", "robot: {urdf: models/model.urdf}\n");
  EXPECT_THROW(prepare_config(root / "runtime.yaml", root / "models", Json::object()), std::runtime_error);
  EXPECT_EQ(read(root / "models/model.urdf"), "<robot/>");
  write(root / "models/model.urdf", "<world/>");
  EXPECT_THROW(prepare_config(root / "runtime.yaml", root / "output", Json::object()), std::runtime_error);
}

TEST(ModelPreparation, XacroScalarArgumentsUseYamlPrecedenceAndExplicitModelOverride) {
  TemporaryDirectory directory; const auto root = directory.path();
  write(root / "source.xacro", "original source"); write(root / "runtime.yaml", "robot: {urdf: missing.urdf}\n");
  const Json options = {{"robot_urdf", "source.xacro"}, {"mappings", Json::array()},
    {"mappings_yaml", "text: 'a b'\nnumber: 2.5\nflag: false\nnothing: null\n"}};
  int calls = 0;
  const auto output = prepare_config(root / "runtime.yaml", root / "out", options,
    [&](const auto &command, auto timeout, auto input, auto environment) {
      ++calls; EXPECT_EQ(command, (std::vector<std::string>{"xacro", (root / "source.xacro").string(),
        "text:=a b", "number:=2.5", "flag:=false", "nothing:=None"}));
      EXPECT_EQ(timeout, 60); EXPECT_TRUE(input.empty()); EXPECT_TRUE(environment.empty()); return "<robot/>";
    });
  EXPECT_EQ(calls, 1); EXPECT_EQ(read(root / "source.xacro"), "original source");
  EXPECT_EQ(read_yaml(output)["robot"]["urdf"], (root / "out/robot.urdf").string());
  EXPECT_EQ(read(root / "runtime.yaml"), "robot: {urdf: missing.urdf}\n");
  write(root / "runtime.yaml", "robot: {urdf: null}\n");
  EXPECT_NO_THROW(prepare_config(root / "runtime.yaml", root / "null", {{"mappings_yaml", "null"}}));
  EXPECT_NO_THROW(prepare_config(root / "runtime.yaml", root / "missing", Json::object()));
  for (const auto &arguments : {"[one]", "arg: [one]", "arg: {nested: value}"}) {
    SCOPED_TRACE(arguments);
    EXPECT_THROW(prepare_config(root / "runtime.yaml", root / "invalid", {{"mappings_yaml", arguments}}), std::runtime_error);
  }
}

TEST(ModelPreparation, ResolvesLegacyMeshRolesPatternsAndPreservesNonMeshEntities) {
  TemporaryDirectory directory; const auto root = directory.path();
  write(root / "scene.stl", "mesh"); write(root / "pattern.csv", "pattern");
  Json entity = {{"name", "obstacle"}};
  for (const auto *role : {"geometry", "visual", "collision", "sensor_geometry"}) entity[role] = {{"type", "mesh"}, {"uri", "scene.stl"}};
  const Json unmodified = {{"geometry", {{"type", "box"}, {"size", {1, 2, 3}}}}, {"visual", nullptr}, {"collision", "none"}};
  const Json source = {{"robot", Json::object()}, {"lidars", Json::array({{{"pattern_file", "pattern.csv"}}, {{"pattern_file", nullptr}}, Json::object()})},
    {"entities", Json::array({entity, unmodified})}};
  write(root / "runtime.yaml", dump_yaml(source));
  const auto result = read_yaml(prepare_config(root / "runtime.yaml", root / "out", Json::object()));
  EXPECT_EQ(result["lidars"][0]["pattern_file"], (root / "pattern.csv").string());
  EXPECT_TRUE(result["lidars"][1]["pattern_file"].is_null()); EXPECT_FALSE(result["lidars"][2].contains("pattern_file"));
  for (const auto *role : {"geometry", "visual", "collision", "sensor_geometry"}) EXPECT_EQ(result["entities"][0][role]["uri"], (root / "scene.stl").string());
  EXPECT_EQ(result["entities"][1], unmodified); EXPECT_EQ(read_yaml(root / "runtime.yaml"), source);
}

TEST(ModelPreparation, EnablesEveryCameraPluginAndLegacyCameraWithoutAffectingOtherSensors) {
  TemporaryDirectory directory; const auto root = directory.path();
  write(root / "model.urdf", "<robot><litesim3d><sensor class='litesim3d_rgb_camera/RgbCamera'/>"
    "<sensor class='litesim3d_depth_camera/DepthCamera'/><sensor class='litesim3d_rgbd_camera/RgbdCamera'/>"
    "<sensor class='litesim3d_imu/Imu'/><sensor class='custom/Sensor'/></litesim3d></robot>");
  write(root / "runtime.yaml", "robot: {urdf: model.urdf}\ncameras: [{name: legacy, enabled: false}]\n");
  for (const auto &enabled : {"true", "false"}) {
    SCOPED_TRACE(enabled);
    const auto result = read_yaml(prepare_config(root / "runtime.yaml", root / enabled, {{"cameras", enabled}}));
    EXPECT_EQ(result["cameras"][0]["enabled"], std::string(enabled) == "true");
    tinyxml2::XMLDocument document; parse_xml(document, read(result["robot"]["urdf"].get<std::string>()));
    auto *sensor = document.RootElement()->FirstChildElement()->FirstChildElement();
    for (int i = 0; i != 3; ++i, sensor = sensor->NextSiblingElement())
      EXPECT_EQ(load_yaml(sensor->FirstChildElement()->GetText())["enabled"], std::string(enabled) == "true");
    EXPECT_EQ(sensor->FirstChildElement(), nullptr); EXPECT_EQ(sensor->NextSiblingElement()->FirstChildElement(), nullptr);
  }
  write(root / "runtime.yaml", "robot: {}\ncameras: [{name: legacy}]\n");
  EXPECT_EQ(read_yaml(prepare_config(root / "runtime.yaml", root / "legacy", {{"cameras", "true"}}))["cameras"][0]["enabled"], true);
  write(root / "runtime.yaml", "robot: {}\n");
  EXPECT_THROW(prepare_config(root / "runtime.yaml", root / "absent", {{"cameras", "true"}}), std::runtime_error);
  EXPECT_THROW(prepare_config(root / "runtime.yaml", root / "invalid", {{"cameras", "enabled"}}), std::runtime_error);
  EXPECT_NO_THROW(prepare_config(root / "runtime.yaml", root / "disabled", {{"cameras", "false"}}));
}

TEST(ModelPreparation, LidarParameterSectionsPinOnlyPopulatedPatterns) {
  TemporaryDirectory directory; const auto root = directory.path(); write(root / "pattern.csv", "pattern");
  write(root / "parameters.yaml", "nested: {settings: {pattern_file: pattern.csv, topic: scan}}\nscalar: 2\nempty: {pattern_file: ''}\n");
  write(root / "model.urdf", "<robot><litesim3d><sensor name='scan' class='litesim3d_lidar/Lidar'>"
    "<parameters file='parameters.yaml' section='.nested..settings.'/><parameters file='parameters.yaml' section='scalar'/>"
    "<parameters file='parameters.yaml' section='empty'/><overrides/></sensor></litesim3d></robot>");
  write(root / "runtime.yaml", "robot: {urdf: model.urdf}\n");
  const auto output = prepare_config(root / "runtime.yaml", root / "out", Json::object());
  tinyxml2::XMLDocument document; parse_xml(document, read(read_yaml(output)["robot"]["urdf"].get<std::string>()));
  auto *params = document.RootElement()->FirstChildElement()->FirstChildElement()->FirstChildElement("parameters");
  EXPECT_EQ(read_yaml(params->Attribute("file"))["nested"]["settings"]["pattern_file"], (root / "pattern.csv").string());
  EXPECT_EQ(params->NextSiblingElement("parameters")->Attribute("file"), (root / "parameters.yaml").string());
  EXPECT_EQ(params->NextSiblingElement("parameters")->NextSiblingElement("parameters")->Attribute("file"), (root / "parameters.yaml").string());
  EXPECT_FALSE(fs::exists(root / "out/sensor-0-parameters-1.yaml"));
  EXPECT_EQ(read_yaml(root / "parameters.yaml")["nested"]["settings"]["pattern_file"], "pattern.csv");
  write(root / "model.urdf", "<robot><litesim3d><sensor class='litesim3d_lidar/Lidar'><overrides/><overrides/></sensor></litesim3d></robot>");
  EXPECT_THROW(prepare_config(root / "runtime.yaml", root / "duplicate", Json::object()), std::runtime_error);
}

TEST(ModelPreparation, AppliesPluginProfilesWithDefaultsExplicitArraysAndClassSelection) {
  TemporaryDirectory directory; const auto root = directory.path();
  write(root / "model.urdf", "<robot><litesim3d><sensor name='scan' class='litesim3d_mid360/Mid360'><overrides>topic: scan</overrides></sensor></litesim3d></robot>");
  write(root / "runtime.yaml", "robot: {urdf: model.urdf}\n");
  const auto filename = root / "profile.yaml";
  for (const auto &settings : {Json{{"columns", 16}}, Json{{"point_layout", "xyzirt_v1"}, {"vertical_angles_deg", {-1., 1.}},
      {"rings", 2}, {"ring_delays_s", {0., .01}}, {"scan_pattern", "uniform"}}, Json{{"rings", nullptr}, {"scan_pattern", "livox_mid360"}}}) {
    SCOPED_TRACE(settings.dump()); write(filename, dump_yaml({{"lidar", settings}}));
    const auto prepared = prepare_config(root / "runtime.yaml", root / "out", {{"lidar_profile", filename.string()}});
    tinyxml2::XMLDocument document; parse_xml(document, read(read_yaml(prepared)["robot"]["urdf"].get<std::string>()));
    auto *sensor = document.RootElement()->FirstChildElement()->FirstChildElement();
    const auto actual = load_yaml(sensor->FirstChildElement()->GetText());
    EXPECT_EQ(actual["topic"], "scan"); EXPECT_FALSE(actual.contains("rings")); EXPECT_FALSE(actual.contains("point_layout"));
    EXPECT_EQ(actual["vertical_angles_deg"], settings.value("vertical_angles_deg", Json::array({0})));
    EXPECT_EQ(actual["ring_delays_s"], settings.value("ring_delays_s", Json::array()));
    const auto expected_class = settings.value("scan_pattern", "livox_mid360") == "livox_mid360" ? "litesim3d_mid360/Mid360" : "litesim3d_lidar/Lidar";
    EXPECT_STREQ(sensor->Attribute("class"), expected_class);
  }
}

TEST(ModelPreparation, RejectsIncompatiblePluginProfileGeometryAndMixedLidarSources) {
  TemporaryDirectory directory; const auto root = directory.path();
  write(root / "model.urdf", "<robot><litesim3d><sensor class='litesim3d_lidar/Lidar'/></litesim3d></robot>");
  write(root / "runtime.yaml", "robot: {urdf: model.urdf}\n");
  const auto filename = root / "profile.yaml";
  for (const auto &settings : {Json{{"point_layout", "unsupported"}}, Json{{"vertical_angles_deg", 0}},
      Json{{"vertical_angles_deg", {0, 1}}, {"rings", 1}}}) {
    SCOPED_TRACE(settings.dump()); write(filename, dump_yaml({{"lidar", settings}}));
    EXPECT_THROW(prepare_config(root / "runtime.yaml", root / "out", {{"lidar_profile", filename.string()}}), std::runtime_error);
  }
  write(filename, "lidar: {}\n");
  for (const auto &legacy : {Json::array({{{"name", "legacy"}}}), Json::array({{{"name", "one"}}, {{"name", "two"}}})}) {
    write(root / "runtime.yaml", dump_yaml({{"robot", {{"urdf", "model.urdf"}}}, {"lidars", legacy}}));
    EXPECT_THROW(prepare_config(root / "runtime.yaml", root / "mixed", {{"lidar_profile", filename.string()}}), std::runtime_error);
  }
  for (const auto &text : {"robot: {}\n", "robot: {}\nlidars: []\n", "robot: {}\nlidars: {}\n",
      "robot: {}\nlidars: [{name: one}, {name: two}]\n"}) {
    write(root / "runtime.yaml", text);
    EXPECT_THROW(prepare_config(root / "runtime.yaml", root / "empty", {{"lidar_profile", filename.string()}}), std::runtime_error);
  }
  write(root / "model.urdf", "<robot><litesim3d><sensor class='litesim3d_lidar/Lidar'/><sensor class='litesim3d_mid360/Mid360'/></litesim3d></robot>");
  write(root / "runtime.yaml", "robot: {urdf: model.urdf}\n");
  EXPECT_THROW(prepare_config(root / "runtime.yaml", root / "multiple", {{"lidar_profile", filename.string()}}), std::runtime_error);
}

TEST(ModelPreparation, LegacyProfileKeepsAvailableBindingsAndReceivesNoiseAndPatternPinning) {
  TemporaryDirectory directory; const auto root = directory.path(); write(root / "patterns/scan.csv", "pattern");
  write(root / "profiles/profile.yaml", "lidar: {pattern_file: ../patterns/scan.csv, columns: 32}\n");
  write(root / "noise.yaml", "sensors: {scan: {range_sigma_m: 0.1}}\n");
  write(root / "runtime.yaml", "robot: {}\nlidars: [{name: scan, link: laser, topic: points, columns: 1}]\n");
  const auto output = prepare_config(root / "runtime.yaml", root / "out", {{"lidar_profile", (root / "profiles/profile.yaml").string()}, {"noise_config", "noise.yaml"}});
  EXPECT_EQ(read_yaml(output)["lidars"], Json::array({{{"pattern_file", (root / "patterns/scan.csv").string()}, {"columns", 32},
    {"name", "scan"}, {"link", "laser"}, {"topic", "points"}, {"range_sigma_m", .1}}}));
  write(root / "runtime.yaml", "robot: {}\nlidars: [{}]\n");
  const auto unbound = read_yaml(prepare_config(root / "runtime.yaml", root / "unbound", {{"lidar_profile", (root / "profiles/profile.yaml").string()}}));
  EXPECT_FALSE(unbound["lidars"][0].contains("name")); EXPECT_FALSE(unbound["lidars"][0].contains("link")); EXPECT_FALSE(unbound["lidars"][0].contains("topic"));
}

TEST(ModelDescription, ConvertsOnlyAbsoluteMeshAndTexturePathsToEscapedFileUris) {
  TemporaryDirectory directory; const auto root = directory.path(); const auto absolute = root / "mesh with space.stl";
  tinyxml2::XMLDocument document; parse_xml(document, "<robot><mesh/><mesh filename='relative.stl'/><texture filename='package://assets/texture.png'/><link filename='unchanged'/></robot>");
  auto *mesh = document.NewElement("mesh"); mesh->SetAttribute("filename", absolute.c_str()); document.RootElement()->InsertEndChild(mesh);
  auto *texture = document.NewElement("texture"); texture->SetAttribute("filename", (root / "tex.png").c_str()); document.RootElement()->InsertEndChild(texture);
  write(root / "model.urdf", xml_text(document));
  tinyxml2::XMLDocument converted; parse_xml(converted, ros_description(root / "model.urdf"));
  auto *first = converted.RootElement()->FirstChildElement();
  EXPECT_EQ(first->Attribute("filename"), nullptr); EXPECT_STREQ(first->NextSiblingElement()->Attribute("filename"), "relative.stl");
  EXPECT_STREQ(converted.RootElement()->FirstChildElement("texture")->Attribute("filename"), "package://assets/texture.png");
  EXPECT_EQ(converted.RootElement()->LastChildElement("mesh")->Attribute("filename"), file_uri(absolute));
  EXPECT_EQ(converted.RootElement()->LastChildElement("texture")->Attribute("filename"), file_uri(root / "tex.png"));
  EXPECT_NE(ros_description(root / "model.urdf").find("%20"), std::string::npos);
}

TEST(ModelPreview, MergesParameterSectionsOverridesAndSelectsEnabledLivoxLidarsOnly) {
  TemporaryDirectory directory; const auto root = directory.path();
  write(root / "params.yaml", "nested: {lidar: {topic: old, scan_pattern: livox_mid360, enabled: false}}\n");
  write(root / "model.urdf", "<robot><litesim3d><sensor name='left' class='litesim3d_lidar/Lidar'>"
    "<parameters file='params.yaml' section='nested.lidar'/><overrides/><overrides>enabled: true\ntopic: ///left/points</overrides></sensor>"
    "<sensor name='disabled' class='litesim3d_mid360/Mid360'><overrides>enabled: false</overrides></sensor>"
    "<sensor name='regular' class='litesim3d_lidar/Lidar'/><sensor class='litesim3d_imu/Imu'/></litesim3d></robot>");
  write(root / "runtime.yaml", "robot: {urdf: model.urdf}\nlidars: [{name: right, scan_pattern: livox_mid360, topic: right/points}, {name: ordinary}]\n");
  const Json expected = {{"/left/points", "/livox/left/lidar/pointcloud"}, {"/left/points/range_boundary", "/livox/left/lidar/range_boundary"},
    {"/right/points", "/livox/right/lidar/pointcloud"}, {"/right/points/range_boundary", "/livox/right/lidar/range_boundary"}};
  const auto actual = preview_topics(root / "runtime.yaml", "auto", 1);
  ASSERT_EQ(actual.size(), expected.size());
  for (auto it = expected.begin(); it != expected.end(); ++it) EXPECT_EQ(actual.at(it.key()), it.value());
  EXPECT_EQ(preview_topics(root / "runtime.yaml", "livox", 0)["/left/points"], "/livox/left/lidar");
  EXPECT_TRUE(preview_topics(root / "nonexistent.yaml", "native", 1).empty());
  write(root / "model.urdf", "<robot/>"); write(root / "runtime.yaml", "robot: {urdf: model.urdf}\n");
  EXPECT_TRUE(preview_topics(root / "runtime.yaml", "auto", 1).empty());
}
