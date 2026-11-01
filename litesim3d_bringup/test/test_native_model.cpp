#include <gtest/gtest.h>
#include "model.hpp"

using namespace litesim3d::bringup;
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
