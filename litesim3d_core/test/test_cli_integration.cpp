#include "litesim3d/sim.hpp"
#include <gtest/gtest.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>

// Component/integration regressions of the production parser/model/simulator
// defaults. These intentionally run the real pipeline and never supply unit hits.
int litesim_headless_integration_entry(int argc, char **argv);
int litesim_model_geometry_integration_entry(int argc, char **argv);

namespace {
struct Directory {
  std::filesystem::path path;
  Directory() {
    static size_t serial = 0;
    path = std::filesystem::temp_directory_path() / ("litesim-cli-unit-" +
        std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + "-" +
        std::to_string(serial++));
    std::filesystem::create_directory(path);
  }
  ~Directory() { std::filesystem::remove_all(path); }
};

struct Result { int status; std::string output, error; };
Result call(int (*entry)(int, char **), std::vector<std::string> arguments,
            const std::string &input = "") {
  std::vector<char *> argv;
  for (auto &argument : arguments) argv.push_back(argument.data());
  argv.push_back(nullptr);
  std::istringstream in(input);
  std::ostringstream out, err;
  const auto input_state = std::cin.rdstate();
  struct Restore {
    std::streambuf *in, *out, *err;
    std::ios::iostate state;
    ~Restore() {
      std::cin.rdbuf(in); std::cin.clear(state);
      std::cout.rdbuf(out); std::cerr.rdbuf(err);
    }
  } restore{std::cin.rdbuf(in.rdbuf()), std::cout.rdbuf(out.rdbuf()), std::cerr.rdbuf(err.rdbuf()),
            input_state};
  std::cin.clear();
  const int status = entry(int(arguments.size()), argv.data());
  return {status, out.str(), err.str()};
}

std::string configuration(Directory &directory, bool camera = false, bool enabled = false) {
  const auto file = directory.path / "configuration.yaml";
  std::ofstream out(file);
  out << "schema_version: 1\nunits: m_rad_s\nrobot: {urdf: '"
      << (std::filesystem::path(SIM_RESOURCES) / "urdf/r01.urdf").string()
      << "'}\nentities: []\n";
  if (camera)
    out << "cameras: [{name: camera, link: base_link, topic: camera/front, width: 16, height: 16, enabled: "
        << (enabled ? "true" : "false") << "}]\n";
  else
    out << "lidars: [{name: lidar, link: lidar_link, columns: 4, vertical_angles_deg: [0]}]\n";
  return file.string();
}
} // namespace

TEST(CoreCliIntegration, HeadlessValidatesUsageStepsAndRealtimeFactor) {
  Directory directory;
  const auto cfg = configuration(directory);
  const auto output = (directory.path / "manifest.json").string();
  const std::vector<std::vector<std::string>> invalid = {
      {"sim_headless"}, {"sim_headless", cfg, "0", output},
      {"sim_headless", cfg, "10000001", output}, {"sim_headless", cfg, "not-a-count", output},
      {"sim_headless", cfg, "1", output, "0", "0", "-1"},
      {"sim_headless", cfg, "1", output, "0", "0", "nan"},
      {"sim_headless", cfg, "1", output, "0", "0", "0", "extra"},
      {"sim_headless", (directory.path / "missing.yaml").string(), "1", output}};
  for (const auto &args : invalid) {
    SCOPED_TRACE(args.back());
    const auto result = call(litesim_headless_integration_entry, args);
    EXPECT_EQ(result.status, 1);
    EXPECT_FALSE(result.error.empty());
  }
}

TEST(CoreCliIntegration, HeadlessRunsRealSensorsAndWritesCompletedManifest) {
  Directory directory;
  const auto cfg = configuration(directory);
  const auto output = (directory.path / "manifest.json").string();
  const auto result = call(litesim_headless_integration_entry, {"sim_headless", cfg, "20", output});
  ASSERT_EQ(result.status, 0) << result.error;
  const auto manifest = litesim3d::Json::parse(litesim3d::readText(output));
  EXPECT_EQ(manifest["frames"], 1);
  EXPECT_EQ(manifest["point_slots"], 4);
  EXPECT_NE(result.output.find("frames=1 points=4"), std::string::npos);
  EXPECT_EQ(call(litesim_headless_integration_entry, {"sim_headless", cfg, "1", output, ".1"}).status, 0);
  EXPECT_EQ(call(litesim_headless_integration_entry, {"sim_headless", cfg, "1", output, ".1", ".2"}).status, 0);
  EXPECT_EQ(call(litesim_headless_integration_entry, {"sim_headless", cfg, "1", output, ".1", ".2", "100"}).status, 0);
  const auto missing_output = (directory.path / "missing" / "manifest.json").string();
  const auto rejected = call(litesim_headless_integration_entry, {"sim_headless", cfg, "1", missing_output});
  EXPECT_EQ(rejected.status, 1);
  EXPECT_NE(rejected.error.find("cannot open output manifest"), std::string::npos);
}

TEST(CoreCliIntegration, HeadlessRejectsEnabledCameraButAllowsDisabledDeclaration) {
  Directory directory;
  const auto output = (directory.path / "manifest.json").string();
  const auto enabled = call(litesim_headless_integration_entry,
      {"sim_headless", configuration(directory, true, true), "1", output});
  EXPECT_EQ(enabled.status, 1);
  EXPECT_NE(enabled.error.find("has no renderer"), std::string::npos);
  const auto disabled = call(litesim_headless_integration_entry,
      {"sim_headless", configuration(directory, true, false), "1", output});
  EXPECT_EQ(disabled.status, 0) << disabled.error;
}

TEST(CoreCliIntegration, ModelGeometryReportsModelAndResolvesRequestsAtInclusiveBudget) {
  const auto model = (std::filesystem::path(SIM_RESOURCES) / "urdf/r01.urdf").string();
  const auto report = call(litesim_model_geometry_integration_entry, {"model_geometry", model, "base_link"});
  ASSERT_EQ(report.status, 0) << report.error;
  const auto parsed = litesim3d::Json::parse(report.output);
  EXPECT_EQ(parsed["base_link"], "base_link");
  EXPECT_DOUBLE_EQ(parsed["joints"]["left_wheel_joint"]["radius"].get<double>(), .1);
  EXPECT_FALSE(parsed["joints"]["lidar_fixed"].contains("radius"));
  const auto request = litesim3d::Json::parse(R"({"kind":"differential","parameters":{
    "left_joints":["left_wheel_joint"],"right_joints":["right_wheel_joint"]}})");
  litesim3d::Json requests = litesim3d::Json::array();
  for (size_t i = 0; i < 64; ++i) requests.push_back(request);
  const auto resolved = call(litesim_model_geometry_integration_entry,
      {"model_geometry", model, "base_link", "--resolve"}, requests.dump());
  ASSERT_EQ(resolved.status, 0) << resolved.error;
  const auto result = litesim3d::Json::parse(resolved.output);
  EXPECT_EQ(result["resolved"].size(), 64u);
  EXPECT_DOUBLE_EQ(result["resolved"][0]["wheel_radius_m"].get<double>(), .1);
  EXPECT_DOUBLE_EQ(result["resolved"][0]["wheel_separation_m"].get<double>(), .5);
}

TEST(CoreCliIntegration, ModelGeometryValidatesUsageJsonKindsAndRequestLimits) {
  const auto model = (std::filesystem::path(SIM_RESOURCES) / "urdf/r01.urdf").string();
  EXPECT_EQ(call(litesim_model_geometry_integration_entry, {"model_geometry"}).status, 1);
  EXPECT_EQ(call(litesim_model_geometry_integration_entry, {"model_geometry", model, "base_link", "--wrong"}).status, 1);
  const std::vector<std::string> argv = {"model_geometry", model, "base_link", "--resolve"};
  for (const auto &input : {"not-json", "{}", "[{}]",
      "[{\"kind\":\"unknown\",\"parameters\":{}}]",
      "[{\"kind\":\"omni\",\"parameters\":{\"wheels\":[]}}]",
      "[{\"kind\":\"ackermann\",\"parameters\":{\"rear_left_joint\":\"left_wheel_joint\","
      "\"rear_right_joint\":\"right_wheel_joint\",\"left_steering_joint\":\"missing\",\"right_steering_joint\":\"missing\"}}]"}) {
    SCOPED_TRACE(input);
    const auto result = call(litesim_model_geometry_integration_entry, argv, input);
    EXPECT_EQ(result.status, 1);
    EXPECT_NE(result.error.find("model geometry:"), std::string::npos);
  }
  const auto oversized = litesim3d::Json(std::vector<int>(65, 0));
  EXPECT_EQ(call(litesim_model_geometry_integration_entry, argv, oversized.dump()).status, 1);
}
