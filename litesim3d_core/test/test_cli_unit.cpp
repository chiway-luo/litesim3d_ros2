#include "cli_test_runtime.hpp"
#include <gtest/gtest.h>
#include <sstream>

// These are orchestration units: no Config::load, Model::load, Simulator,
// tick pipeline, sensors, filesystem, wall clock or real sleep is invoked.
#define LITESIM_CORE_CLI_RUNTIME cli_test::Runtime
#define main headless_cli_unit_entry
#include "../src/headless.cpp"
#undef main
#define main geometry_cli_unit_entry
#include "../src/model_geometry.cpp"
#undef main
#undef LITESIM_CORE_CLI_RUNTIME

namespace {
using cli_test::Scope;
using cli_test::Json;
struct Result { int status; std::string output, error; };
Result call(int (*entry)(int, char **), std::vector<std::string> args, const std::string &input = "") {
  std::vector<char *> argv;
  for (auto &argument : args) argv.push_back(argument.data());
  argv.push_back(nullptr);
  std::istringstream in(input);
  std::ostringstream out, err;
  const auto old_state = std::cin.rdstate();
  struct Restore {
    std::streambuf *input, *output, *error;
    std::ios::iostate state;
    ~Restore() {
      std::cin.rdbuf(input); std::cin.clear(state);
      std::cout.rdbuf(output); std::cerr.rdbuf(error);
    }
  } restore{std::cin.rdbuf(in.rdbuf()), std::cout.rdbuf(out.rdbuf()), std::cerr.rdbuf(err.rdbuf()), old_state};
  std::cin.clear();
  const auto status = entry(int(args.size()), argv.data());
  return {status, out.str(), err.str()};
}
const std::string config_file = "not-a-real-file.unit-config";
const std::string output_file = "not-a-real-file.unit-output";
std::vector<std::string> headless(const std::string &count = "1") {
  return {"sim_headless", config_file, count, output_file};
}
} // namespace

TEST(CoreCliUnit, UsageAndStepValidationNeverReachPipelineDependencies) {
  for (const auto &args : {std::vector<std::string>{"sim_headless"}, headless("0"), headless("10000001"),
                           headless("not-a-number"),
                           std::vector<std::string>{"sim_headless", config_file, "1", output_file, "0", "0", "0", "extra"}}) {
    Scope scope;
    const auto result = call(headless_cli_unit_entry, args);
    EXPECT_EQ(result.status, 1); EXPECT_FALSE(result.error.empty());
    EXPECT_TRUE(scope.state->calls.empty());
  }
}

TEST(CoreCliUnit, HeadlessCountsFixtureEventsAndPreservesManifestWithoutRealSimulation) {
  Scope scope;
  scope.state->ticks = {{{{{2, 3}, {4, 1}}}}, {}, {{{{1, 5}}}, {{{3, 2}}}}};
  const auto result = call(headless_cli_unit_entry, headless("3"));
  ASSERT_EQ(result.status, 0) << result.error;
  const auto manifest = Json::parse(scope.state->output_text);
  EXPECT_EQ(manifest["marker"], "inert fixture");
  EXPECT_EQ(manifest["frames"], 4); EXPECT_EQ(manifest["point_slots"], 21);
  EXPECT_EQ(scope.state->config_file, config_file);
  EXPECT_EQ(scope.state->model_file, "model.from.configuration");
  EXPECT_EQ(scope.state->scene_token, 123);
  EXPECT_EQ(scope.state->output_file, output_file);
  EXPECT_EQ(scope.state->manifest_reason, "completed");
  EXPECT_TRUE(scope.state->exited); EXPECT_EQ(scope.state->tick_count, 3u);
  ASSERT_EQ(scope.state->commands.size(), 3u);
  for (const auto &command : scope.state->commands) { EXPECT_EQ(command.v, 0); EXPECT_EQ(command.w, 0); }
  EXPECT_TRUE(scope.state->sleeps.empty());
  EXPECT_NE(result.output.find("time_ns=307 frames=4 points=21 contacts=2"), std::string::npos);
  const auto exit = std::find(scope.state->calls.begin(), scope.state->calls.end(), "exit");
  const auto manifest_call = std::find(scope.state->calls.begin(), scope.state->calls.end(), "manifest");
  EXPECT_LT(exit, manifest_call);
}

TEST(CoreCliUnit, OptionalArgumentsAndPacingUseRecordedCommandsAndFakeClock) {
  for (size_t options = 1; options <= 3; ++options) {
    Scope scope;
    auto args = headless("2");
    args.push_back(".25");
    if (options >= 2) args.push_back("-.5");
    if (options >= 3) args.push_back("4");
    const auto result = call(headless_cli_unit_entry, args);
    ASSERT_EQ(result.status, 0) << result.error;
    ASSERT_EQ(scope.state->commands.size(), 2u);
    EXPECT_EQ(scope.state->commands[0].v, .25);
    EXPECT_EQ(scope.state->commands[0].w, options >= 2 ? -.5 : 0);
    if (options == 3) {
      ASSERT_EQ(scope.state->sleeps.size(), 2u);
      EXPECT_EQ(scope.state->sleeps[0], scope.state->start + std::chrono::nanoseconds(25));
      EXPECT_EQ(scope.state->sleeps[1], scope.state->start + std::chrono::nanoseconds(50));
    } else EXPECT_TRUE(scope.state->sleeps.empty());
  }
  for (const auto &rtf : {"nan", "-1"}) {
    Scope scope;
    auto args = headless(); args.insert(args.end(), {"0", "0", rtf});
    EXPECT_EQ(call(headless_cli_unit_entry, args).status, 1);
    EXPECT_EQ(scope.state->tick_count, 0u); EXPECT_TRUE(scope.state->sleeps.empty());
  }
}

TEST(CoreCliUnit, ResolvedCameraGuardRejectsEnabledButNotDisabledDescriptors) {
  for (bool enabled : {false, true}) {
    Scope scope;
    scope.state->config.cameras = {{enabled}};
    const auto result = call(headless_cli_unit_entry, headless());
    EXPECT_EQ(result.status, enabled ? 1 : 0);
    if (enabled) {
      EXPECT_NE(result.error.find("has no renderer"), std::string::npos);
      EXPECT_TRUE(scope.state->commands.empty());
      EXPECT_FALSE(scope.state->exited);
    }
  }
}

TEST(CoreCliUnit, InjectedDependencyAndOutputFailuresReturnErrorsWithoutRunningARealPipeline) {
  for (const std::string &failure : {"config", "model", "simulation", "now", "command", "tick", "sleep", "exit", "manifest", "output", "write"}) {
    Scope scope;
    scope.state->fail = failure;
    auto args = headless(); args.insert(args.end(), {"0", "0", "1"});
    const auto result = call(headless_cli_unit_entry, args);
    EXPECT_EQ(result.status, 1) << failure;
    EXPECT_NE(result.error.find("injected " + failure), std::string::npos) << failure;
  }
  for (bool open_failure : {false, true}) {
    Scope scope;
    scope.state->output_open = !open_failure;
    scope.state->output_good = open_failure;
    const auto result = call(headless_cli_unit_entry, headless());
    EXPECT_EQ(result.status, 1);
    EXPECT_NE(result.error.find(open_failure ? "cannot open output manifest" : "cannot write manifest"), std::string::npos);
  }
}

TEST(CoreCliUnit, GeometrySerializationUsesOnlyInertLoaderAndMechanicalGeometryFixtures) {
  Scope scope;
  litesim3d::plugins::JointDescription joint;
  joint.type = "continuous"; joint.position = {1, 2, 3}; joint.axis = {0, 1, 0};
  joint.lower = -2; joint.upper = 4; joint.max_velocity = 7; joint.parent_joint = "parent";
  scope.state->geometry.model.joints["wheel"] = joint;
  joint.type = "fixed"; scope.state->geometry.model.joints["fixed"] = joint;
  scope.state->geometry.wheel_radii["wheel"] = .3;
  const auto result = call(geometry_cli_unit_entry, {"model_geometry", "never-on-disk.urdf", "requested-base"});
  ASSERT_EQ(result.status, 0) << result.error;
  const auto json = Json::parse(result.output);
  EXPECT_EQ(scope.state->model_file, "never-on-disk.urdf");
  EXPECT_TRUE(scope.state->model_options.allow_visual_only);
  EXPECT_EQ(scope.state->model_options.sensor_geometry, "collision");
  EXPECT_EQ(scope.state->base_link, "requested-base");
  EXPECT_EQ(json["base_link"], "fixture-base");
  EXPECT_EQ(json["joints"]["wheel"]["position"], Json::array({1, 2, 3}));
  EXPECT_EQ(json["joints"]["wheel"]["axis"], Json::array({0, 1, 0}));
  EXPECT_EQ(json["joints"]["wheel"]["radius"], .3);
  EXPECT_EQ(json["joints"]["wheel"]["max_velocity"], 7);
  EXPECT_FALSE(json["joints"]["fixed"].contains("radius"));
  EXPECT_TRUE(scope.state->config_file.empty());
  EXPECT_TRUE(scope.state->commands.empty());
}

TEST(CoreCliUnit, GeometryRoutesAllRequestKindsAndPreservesParametersAtTheInclusiveLimit) {
  Scope scope;
  const std::vector<std::string> args = {"model_geometry", "fake.urdf", "base", "--resolve"};
  Json requests = Json::array();
  for (const auto &kind : {"differential", "ackermann", "omni"})
    requests.push_back({{"kind", kind}, {"parameters", {{"token", kind}}}});
  const auto result = call(geometry_cli_unit_entry, args, requests.dump());
  ASSERT_EQ(result.status, 0) << result.error;
  const auto json = Json::parse(result.output);
  ASSERT_EQ(scope.state->resolutions.size(), 3u);
  for (size_t i = 0; i < 3; ++i) {
    EXPECT_EQ(json["resolved"][i]["route"], requests[i]["kind"]);
    EXPECT_EQ(json["resolved"][i]["parameters"], requests[i]["parameters"]);
    EXPECT_EQ(scope.state->resolutions[i].second, requests[i]["parameters"]);
  }
  requests = Json::array();
  for (size_t i = 0; i < 64; ++i) requests.push_back({{"kind", "omni"}, {"parameters", {{"i", i}}}});
  EXPECT_EQ(call(geometry_cli_unit_entry, args, requests.dump()).status, 0);
  EXPECT_EQ(scope.state->resolutions.size(), 67u);
}

TEST(CoreCliUnit, GeometryUsageJsonDispatchAndInjectedFailuresReturnTheDocumentedErrorPrefix) {
  for (const auto &args : {std::vector<std::string>{"model_geometry"},
                           std::vector<std::string>{"model_geometry", "fake", "base", "--wrong"}}) {
    Scope scope;
    EXPECT_EQ(call(geometry_cli_unit_entry, args).status, 1);
    EXPECT_TRUE(scope.state->calls.empty());
  }
  const std::vector<std::string> args = {"model_geometry", "fake.urdf", "base", "--resolve"};
  for (const std::string &input : {"not-json", "{}", "[{}]", "[{\"kind\":\"unknown\",\"parameters\":{}}]"}) {
    Scope scope;
    const auto result = call(geometry_cli_unit_entry, args, input);
    EXPECT_EQ(result.status, 1); EXPECT_NE(result.error.find("model geometry:"), std::string::npos);
    EXPECT_TRUE(scope.state->resolutions.empty());
  }
  Scope scope;
  EXPECT_EQ(call(geometry_cli_unit_entry, args, Json(std::vector<int>(65)).dump()).status, 1);
  for (const std::string &failure : {"model", "geometry", "differential", "ackermann", "omni"}) {
    Scope failed;
    failed.state->fail = failure;
    const auto request = Json::array({{{"kind", failure}, {"parameters", Json::object()}}});
    const auto result = call(geometry_cli_unit_entry, args, request.dump());
    EXPECT_EQ(result.status, 1);
    EXPECT_NE(result.error.find("injected " + failure), std::string::npos);
  }
}
