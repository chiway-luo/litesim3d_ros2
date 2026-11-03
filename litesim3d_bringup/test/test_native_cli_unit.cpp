#include "model.hpp"
#include <gtest/gtest.h>
#include <cstdlib>
#include <iostream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

// Compile the real CLI orchestration below, but replace its provider boundaries.
// No model preparation, resource lookup, disk access, signals or child process is
// performed. model.hpp and standard headers are included BEFORE substitutions,
// so the production API is not rewritten and the macros cannot leak into it.
namespace native_cli_fake {
using Json = litesim3d::bringup::Json;
using Path = std::filesystem::path;
using Element = tinyxml2::XMLElement;
struct Call { std::string name; Json arguments; };
struct State {
  std::vector<Call> calls;
  std::map<std::string, int> occurrences;
  std::string failure;
  int failure_occurrence = 1;
  bool exists = false, directory = true, empty = true, boolean_result = true;
  int guards_started = 0, guards_finished = 0;
  Json json_result = {{"provider_result", "inert fixture"}};
  Json configuration = {{"robot", {{"urdf", "package://fixture/robot.urdf"}}}};
  Path prepared = "/provider/model/runtime.yaml", resolved = "/provider/robot.urdf";
  Path executable = "/provider/bin/sdf_import";
  std::string description = "<robot name='provider-fixture'/>", serialized = "serialized fixture\n";
  std::string process_output = "import diagnostics\n";
  void record(const std::string &name, Json arguments = Json::array()) {
    calls.push_back({name, std::move(arguments)});
    if (++occurrences[name] == failure_occurrence && failure == name)
      throw std::runtime_error("injected " + name);
  }
};
State *active = nullptr;
struct Scope {
  State state;
  State *previous = active;
  Scope() { active = &state; }
  ~Scope() { active = previous; }
};
void require(bool condition, const std::string &message) {
  if (!condition) throw std::runtime_error(message);
}
struct InterruptGuard {
  InterruptGuard() {
    if (active->failure == "interrupt") throw std::runtime_error("injected interrupt");
    ++active->guards_started;
  }
  ~InterruptGuard() { ++active->guards_finished; }
};
namespace fs {
using path = Path;
bool exists(const Path &path) {
  active->record("exists", Json::array({path.string()})); return active->exists;
}
bool is_directory(const Path &path) {
  active->record("is_directory", Json::array({path.string()})); return active->directory;
}
bool is_empty(const Path &path) {
  active->record("is_empty", Json::array({path.string()})); return active->empty;
}
}
Path canonical_path(const Path &path) {
  active->record("canonical_path", Json::array({path.string()}));
  return path.is_absolute() ? path : Path("/canonical") / path;
}
Path prepare_config(const Path &source, const Path &directory, const Json &options) {
  active->record("prepare_config", Json::array({source.string(), directory.string(), options}));
  return active->prepared;
}
Path resolve(const std::string &uri, const Path &base) {
  active->record("resolve", Json::array({uri, base.string()})); return active->resolved;
}
Json load_yaml(const std::string &text) {
  active->record("load_yaml", Json::array({text})); return active->json_result;
}
bool boolean(std::string value, const std::string &name) {
  active->record("boolean", Json::array({value, name})); return active->boolean_result;
}
Json merge(Json base, const Json &overlay) {
  active->record("merge", Json::array({base, overlay})); return active->json_result;
}
Json profile(const std::string &selection, const Path &directory) {
  active->record("profile", Json::array({selection, directory.string()})); return active->json_result;
}
std::string ros_description(const Path &urdf) {
  active->record("ros_description", Json::array({urdf.string()})); return active->description;
}
Json preview_topics(const Path &config, const std::string &interface, int format) {
  active->record("preview_topics", Json::array({config.string(), interface, format})); return active->json_result;
}
Json controller_parameters(Json configuration, const std::string &description, const std::string &base) {
  active->record("controller_parameters", Json::array({configuration, description, base})); return active->json_result;
}
Json model_geometry(const std::string &description, const std::string &base, const Json &requests) {
  active->record("model_geometry", Json::array({description, base, requests})); return active->json_result;
}
void parse_xml(tinyxml2::XMLDocument &document, const std::string &text) {
  active->record("parse_xml", Json::array({text}));
  // Only construct the in-memory provider handle required by dispatch. XML
  // validation and transformation contracts belong to native_model_unit.
  if (document.Parse(text.c_str()) != tinyxml2::XML_SUCCESS)
    throw std::runtime_error("fixture XML parse failure");
}
void override_plugin(Element *element, const Json &values) {
  active->record("override_plugin", Json::array({element->Name(), values}));
  element->SetAttribute("overridden", "yes");
}
std::string xml_text(const tinyxml2::XMLNode &node) {
  const auto *element = node.ToElement();
  active->record("xml_text", Json::array({node.Value(), element->Attribute("overridden") != nullptr,
                                        element->Attribute("noised") != nullptr}));
  return "<provider-result/>";
}
void apply_noise(Json &data, Element *root, const Path &filename, const Path &base) {
  active->record("apply_noise", Json::array({data, root ? Json(root->Name()) : Json(nullptr),
                                           filename.string(), base.string()}));
  data["provider_noise"] = true;
  if (root) root->SetAttribute("noised", "yes");
}
Path executable(const std::string &package, const std::string &name) {
  active->record("executable", Json::array({package, name})); return active->executable;
}
std::string checked_run(const std::vector<std::string> &command, double timeout, const std::string &input,
                        const std::map<std::string, std::string> &environment) {
  active->record("checked_run", Json::array({command, timeout, input, environment})); return active->process_output;
}
Json prepare_xacro(const Path &source, const Path &output, const std::vector<std::string> &mappings,
                   const Path &resource_base) {
  active->record("prepare_xacro", Json::array({source.string(), output.string(), mappings, resource_base.string()}));
  return active->json_result;
}
Json read_yaml(const Path &source) {
  active->record("read_yaml", Json::array({source.string()})); return active->configuration;
}
std::string dump_yaml(const Json &value) {
  active->record("dump_yaml", Json::array({value})); return active->serialized;
}
void atomic_write(const Path &path, const std::string &text) {
  active->record("atomic_write", Json::array({path.string(), text}));
}
}  // namespace native_cli_fake

#define require native_cli_fake::require
#define InterruptGuard native_cli_fake::InterruptGuard
#define fs native_cli_fake::fs
#define canonical_path native_cli_fake::canonical_path
#define prepare_config native_cli_fake::prepare_config
#define resolve native_cli_fake::resolve
#define load_yaml native_cli_fake::load_yaml
#define boolean native_cli_fake::boolean
#define merge native_cli_fake::merge
#define profile native_cli_fake::profile
#define ros_description native_cli_fake::ros_description
#define preview_topics native_cli_fake::preview_topics
#define controller_parameters native_cli_fake::controller_parameters
#define model_geometry native_cli_fake::model_geometry
#define parse_xml native_cli_fake::parse_xml
#define override_plugin native_cli_fake::override_plugin
#define xml_text native_cli_fake::xml_text
#define apply_noise native_cli_fake::apply_noise
#define executable native_cli_fake::executable
#define checked_run native_cli_fake::checked_run
#define prepare_xacro native_cli_fake::prepare_xacro
#define read_yaml native_cli_fake::read_yaml
#define dump_yaml native_cli_fake::dump_yaml
#define atomic_write native_cli_fake::atomic_write
#define main prepare_config_unit_entry
#include "../src/prepare_config.cpp"
#undef main
#define main prepare_xacro_unit_entry
#include "../src/prepare_xacro.cpp"
#undef main
#define main make_rsp_params_unit_entry
#include "../src/make_rsp_params.cpp"
#undef main
#undef atomic_write
#undef dump_yaml
#undef read_yaml
#undef prepare_xacro
#undef checked_run
#undef executable
#undef apply_noise
#undef xml_text
#undef override_plugin
#undef parse_xml
#undef model_geometry
#undef controller_parameters
#undef preview_topics
#undef ros_description
#undef profile
#undef merge
#undef boolean
#undef load_yaml
#undef resolve
#undef prepare_config
#undef canonical_path
#undef fs
#undef InterruptGuard
#undef require

namespace {
using native_cli_fake::Json;
using native_cli_fake::Scope;
using native_cli_fake::State;
struct Result { int status; std::string output, error; };
struct Streams {
  std::streambuf *input = std::cin.rdbuf(), *output = std::cout.rdbuf(), *error = std::cerr.rdbuf();
  std::ios::iostate input_state = std::cin.rdstate(), output_state = std::cout.rdstate(), error_state = std::cerr.rdstate();
  ~Streams() {
    std::cin.rdbuf(input); std::cin.clear(input_state);
    std::cout.rdbuf(output); std::cout.clear(output_state);
    std::cerr.rdbuf(error); std::cerr.clear(error_state);
  }
};
Result call(int (*entry)(int, char **), std::vector<std::string> arguments, const std::string &input = "") {
  std::vector<char *> argv;
  for (auto &argument : arguments) argv.push_back(argument.data());
  argv.push_back(nullptr);
  std::istringstream in(input);
  std::ostringstream out, err;
  Streams restore;
  std::cin.rdbuf(in.rdbuf()); std::cout.rdbuf(out.rdbuf()); std::cerr.rdbuf(err.rdbuf());
  return {entry(static_cast<int>(arguments.size()), argv.data()), out.str(), err.str()};
}
Result request(const Json &value) {
  return call(prepare_config_unit_entry, {"prepare_config", "--request"}, value.dump());
}
std::vector<std::string> names(const State &state) {
  std::vector<std::string> result;
  for (const auto &item : state.calls) result.push_back(item.name);
  return result;
}
const Json &arguments(const State &state, const std::string &name, size_t occurrence = 0) {
  for (const auto &item : state.calls) {
    if (item.name == name) {
      if (occurrence == 0) return item.arguments;
      --occurrence;
    }
  }
  throw std::runtime_error("missing spy call: " + name);
}
void error(const Result &result, const std::string &prefix, const std::string &detail = "") {
  EXPECT_EQ(result.status, 1);
  EXPECT_TRUE(result.output.empty()) << result.output;
  EXPECT_EQ(result.error.find(prefix + ": "), 0u) << result.error;
  EXPECT_NE(result.error.find(detail), std::string::npos) << result.error;
  ASSERT_FALSE(result.error.empty());
  EXPECT_EQ(result.error.back(), '\n');
}
void guards_closed(const State &state) {
  EXPECT_EQ(state.guards_started, 1); EXPECT_EQ(state.guards_finished, 1);
}
struct Environment {
  std::string name;
  std::optional<std::string> original;
  explicit Environment(std::string key, const std::optional<std::string> &value) : name(std::move(key)) {
    if (const char *current = std::getenv(name.c_str())) original = current;
    assign(value);
  }
  void assign(const std::optional<std::string> &value) const {
    if (value) ::setenv(name.c_str(), value->c_str(), 1); else ::unsetenv(name.c_str());
  }
  ~Environment() { assign(original); }
};
struct DispatchCase { Json request, expected_arguments; std::string provider; Json result; };
std::vector<DispatchCase> dispatch_cases() {
  const Json marker = {{"provider_result", "inert fixture"}};
  const Json configuration = {{"drive", {{"type", "differential"}}}};
  const Json geometry = Json::array({{{"kind", "omni"}, {"parameters", {{"wheel", "left"}}}}});
  const Json prepare = {{"operation", "prepare"}, {"source", "input.yaml"}, {"directory", "output"}, {"token", 17}};
  return {
    {prepare, Json::array({"input.yaml", "output", prepare}), "prepare_config", "/provider/model/runtime.yaml"},
    {{{"operation", "resolve"}, {"uri", "package://fixture/model"}, {"base", "resource-base"}},
      Json::array({"package://fixture/model", "resource-base"}), "resolve", "/provider/robot.urdf"},
    {{{"operation", "load_yaml"}, {"text", "x: 12\n"}}, Json::array({"x: 12\n"}), "load_yaml", marker},
    {{{"operation", "boolean"}, {"value", "TRUE"}, {"name", "cameras"}}, Json::array({"TRUE", "cameras"}), "boolean", true},
    {{{"operation", "merge"}, {"base", {{"base", 1}}}, {"overlay", {{"overlay", 2}}}},
      Json::array({Json{{"base", 1}}, Json{{"overlay", 2}}}), "merge", marker},
    {{{"operation", "profile"}, {"selection", "mid360"}, {"directory", "profiles"}},
      Json::array({"mid360", "profiles"}), "profile", marker},
    {{{"operation", "ros_description"}, {"urdf", "robot.urdf"}}, Json::array({"robot.urdf"}),
      "ros_description", "<robot name='provider-fixture'/>"},
    {{{"operation", "preview_topics"}, {"config", "preview.yaml"}, {"interface", "livox"}, {"format", 3}},
      Json::array({"preview.yaml", "livox", 3}), "preview_topics", marker},
    {{{"operation", "controller_parameters"}, {"configuration", configuration}, {"description", "<robot/>"}},
      Json::array({configuration, "<robot/>", "base_link"}), "controller_parameters", marker},
    {{{"operation", "controller_parameters"}, {"configuration", configuration}, {"description", "<robot/>"}, {"base_link", "custom-base"}},
      Json::array({configuration, "<robot/>", "custom-base"}), "controller_parameters", marker},
    {{{"operation", "model_geometry"}, {"description", "<geometry/>"}, {"base_link", "chassis"}, {"requests", geometry}},
      Json::array({"<geometry/>", "chassis", geometry}), "model_geometry", marker},
  };
}
}  // namespace

TEST(NativeCliUnit, HelpAndUsageDoNotReachProviders) {
  struct Entry { int (*entry)(int, char **); std::string name, prefix; bool short_help, guard; };
  const std::vector<Entry> entries = {
    {prepare_config_unit_entry, "prepare_config", "model preparation", true, true},
    {prepare_xacro_unit_entry, "prepare_xacro", "prepare_xacro", false, true},
    {make_rsp_params_unit_entry, "make_rsp_params", "make_rsp_params", false, false}};
  for (const auto &item : entries) {
    SCOPED_TRACE(item.name);
    for (const auto &flag : {"--help", "-h"}) {
      Scope scope;
      const auto result = call(item.entry, {item.name, flag});
      if (std::string(flag) == "--help" || item.short_help) {
        EXPECT_EQ(result.status, 0); EXPECT_TRUE(result.error.empty());
        EXPECT_EQ(result.output.find("usage: " + item.name + " "), 0u);
      } else error(result, item.prefix, "usage:");
      EXPECT_TRUE(scope.state.calls.empty());
      EXPECT_EQ(scope.state.guards_started, item.guard ? 1 : 0);
      EXPECT_EQ(scope.state.guards_finished, scope.state.guards_started);
    }
    for (const auto &args : {std::vector<std::string>{item.name}, std::vector<std::string>{item.name, "source"}}) {
      Scope scope;
      error(call(item.entry, args), item.prefix, "usage:");
      EXPECT_TRUE(scope.state.calls.empty());
      EXPECT_EQ(scope.state.guards_finished, scope.state.guards_started);
    }
  }
}

TEST(NativeCliUnit, EverySimpleRequestDispatchesExactArgumentsAndSerializesProviderResult) {
  for (const auto &item : dispatch_cases()) {
    SCOPED_TRACE(item.request.dump());
    Scope scope;
    const auto result = request(item.request);
    ASSERT_EQ(result.status, 0) << result.error;
    EXPECT_TRUE(result.error.empty()); EXPECT_EQ(result.output, item.result.dump() + "\n");
    ASSERT_EQ(scope.state.calls.size(), 1u);
    EXPECT_EQ(scope.state.calls[0].name, item.provider);
    EXPECT_EQ(scope.state.calls[0].arguments, item.expected_arguments);
    guards_closed(scope.state);
  }
  Scope scope;
  scope.state.boolean_result = false;
  EXPECT_EQ(request({{"operation", "boolean"}, {"value", "false"}, {"name", "enabled"}}).output, "false\n");
}

TEST(NativeCliUnit, EverySimpleRequestProviderFailureHasNoSuccessOutputOrLaterCalls) {
  for (const auto &item : dispatch_cases()) {
    SCOPED_TRACE(item.provider);
    Scope scope;
    scope.state.failure = item.provider;
    error(request(item.request), "model preparation", "injected " + item.provider);
    EXPECT_EQ(names(scope.state), std::vector<std::string>({item.provider}));
    guards_closed(scope.state);
  }
}

TEST(NativeCliUnit, MissingRequiredRequestFieldsCannotInvokeSimpleProviders) {
  for (const auto &item : dispatch_cases()) {
    for (auto field = item.request.begin(); field != item.request.end(); ++field) {
      // The whole prepare request is intentionally forwarded as options; token
      // is not required. controller_parameters has a base_link default.
      if (field.key() == "token" || (item.provider == "controller_parameters" && field.key() == "base_link")) continue;
      SCOPED_TRACE(item.request.dump() + " missing " + field.key());
      Scope scope;
      auto incomplete = item.request; incomplete.erase(field.key());
      error(request(incomplete), "model preparation");
      EXPECT_TRUE(scope.state.calls.empty()); guards_closed(scope.state);
    }
  }
}

TEST(NativeCliUnit, InvalidJsonAndUnknownRequestDoNotRunProviders) {
  for (const std::string input : {"", "not-json", "{", "[]", "null", "1", "{}", "{\"operation\":7}",
                                  "{\"operation\":\"not-supported\"}"}) {
    Scope scope;
    error(call(prepare_config_unit_entry, {"prepare_config", "--request"}, input), "model preparation");
    EXPECT_TRUE(scope.state.calls.empty()); guards_closed(scope.state);
  }
  for (const auto &value : {Json{{"operation", "resolve"}, {"uri", 1}, {"base", "base"}},
                           Json{{"operation", "preview_topics"}, {"config", "x"}, {"interface", "y"}, {"format", "bad"}},
                           Json{{"operation", "controller_parameters"}, {"configuration", Json::object()}, {"description", "x"}, {"base_link", false}}}) {
    Scope scope; error(request(value), "model preparation"); EXPECT_TRUE(scope.state.calls.empty());
  }
}

TEST(NativeCliUnit, OverrideUsesParsedRootAndSerializesTheMutatedProviderHandle) {
  Scope scope;
  const Json values = {{"enabled", false}, {"rate", 19}};
  const auto result = request({{"operation", "override"}, {"xml", "<plugin/>"}, {"values", values}});
  ASSERT_EQ(result.status, 0) << result.error;
  EXPECT_EQ(Json::parse(result.output), "<provider-result/>");
  EXPECT_EQ(names(scope.state), std::vector<std::string>({"parse_xml", "override_plugin", "xml_text"}));
  EXPECT_EQ(arguments(scope.state, "parse_xml"), Json::array({"<plugin/>"}));
  EXPECT_EQ(arguments(scope.state, "override_plugin"), Json::array({"plugin", values}));
  EXPECT_EQ(arguments(scope.state, "xml_text"), Json::array({"plugin", true, false}));
}

TEST(NativeCliUnit, NoiseDispatchPreservesDataAndBothNullableAndPresentXmlContracts) {
  for (const bool with_xml : {false, true}) {
    Scope scope;
    const Json data = {{"keep", {{"nested", 12}}}};
    const auto result = request({{"operation", "noise"}, {"xml", with_xml ? Json("<robot/>") : Json(nullptr)},
                                {"data", data}, {"filename", "noise.yaml"}, {"base", "assets"}});
    ASSERT_EQ(result.status, 0) << result.error;
    auto expected = data; expected["provider_noise"] = true;
    EXPECT_EQ(Json::parse(result.output), (Json{{"data", expected}, {"xml", with_xml ? Json("<provider-result/>") : Json(nullptr)}}));
    EXPECT_EQ(arguments(scope.state, "apply_noise"), Json::array({data, with_xml ? Json("robot") : Json(nullptr), "noise.yaml", "assets"}));
    EXPECT_EQ(names(scope.state), with_xml ? std::vector<std::string>({"parse_xml", "apply_noise", "xml_text"}) :
                                          std::vector<std::string>({"apply_noise"}));
    if (with_xml) { EXPECT_EQ(arguments(scope.state, "xml_text"), Json::array({"robot", false, true})); }
    guards_closed(scope.state);
  }
}

TEST(NativeCliUnit, XmlRequestFailuresStopAtTheFailingProvider) {
  for (const auto &operation : {"override", "noise"}) {
    const auto mutator = std::string(operation) == "override" ? "override_plugin" : "apply_noise";
    const std::vector<std::string> ordered = {"parse_xml", mutator, "xml_text"};
    for (size_t index = 0; index < ordered.size(); ++index) {
      Scope scope;
      scope.state.failure = ordered[index];
      error(request({{"operation", operation}, {"xml", "<plugin/>"}, {"values", Json::object()},
                     {"data", Json::object()}, {"filename", "noise"}, {"base", "base"}}), "model preparation", "injected " + ordered[index]);
      EXPECT_EQ(names(scope.state), std::vector<std::string>(ordered.begin(), ordered.begin() + index + 1));
      guards_closed(scope.state);
    }
  }
}

TEST(NativeCliUnit, MissingXmlRequestFieldsDoNotInvokeMutatorsOrSerializePartialResults) {
  const std::vector<Json> requests = {
    {{"operation", "override"}, {"xml", "<plugin/>"}, {"values", Json::object()}},
    {{"operation", "noise"}, {"xml", "<robot/>"}, {"data", Json::object()}, {"filename", "noise"}, {"base", "assets"}},
    {{"operation", "noise"}, {"xml", nullptr}, {"data", Json::object()}, {"filename", "noise"}, {"base", "assets"}}};
  for (const auto &original : requests) {
    for (auto field = original.begin(); field != original.end(); ++field) {
      SCOPED_TRACE(original.dump() + " missing " + field.key());
      Scope scope;
      auto incomplete = original; incomplete.erase(field.key());
      error(request(incomplete), "model preparation");
      const bool parsed = field.key() != "operation" && field.key() != "xml" && !original["xml"].is_null();
      EXPECT_EQ(names(scope.state), parsed ? std::vector<std::string>({"parse_xml"}) : std::vector<std::string>{});
      guards_closed(scope.state);
    }
  }
  Scope scope;
  scope.state.failure = "apply_noise";
  error(request({{"operation", "noise"}, {"xml", nullptr}, {"data", Json::object()}, {"filename", "noise"}, {"base", "base"}}),
        "model preparation", "injected apply_noise");
  EXPECT_EQ(names(scope.state), std::vector<std::string>({"apply_noise"}));
}

TEST(NativeCliUnit, ConfigurationOptionsAreTranslatedWithoutConvertingOrDroppingValues) {
  Scope scope;
  const auto result = call(prepare_config_unit_entry, {"prepare_config", "source.yaml", "output", "--robot-urdf", "robot.urdf",
    "--cameras", "FALSE", "--lidar-profile", "mid360", "--lidar-profiles-dir", "profiles", "--noise-config", "noise.yaml",
    "--model-arguments", "color: blue"});
  ASSERT_EQ(result.status, 0) << result.error;
  const Json options = {{"robot_urdf", "robot.urdf"}, {"cameras", "FALSE"}, {"lidar_profile", "mid360"},
                        {"profiles_dir", "profiles"}, {"noise_config", "noise.yaml"}, {"mappings", scope.state.json_result}};
  EXPECT_EQ(arguments(scope.state, "load_yaml"), Json::array({"color: blue"}));
  EXPECT_EQ(arguments(scope.state, "prepare_config"), Json::array({"source.yaml", "/canonical/output/model", options}));
  EXPECT_EQ(names(scope.state), std::vector<std::string>({"load_yaml", "canonical_path", "exists", "prepare_config"}));
  EXPECT_EQ(Json::parse(result.output), (Json{{"config", scope.state.prepared.string()}}));
  EXPECT_TRUE(result.error.empty()); guards_closed(scope.state);
}

TEST(NativeCliUnit, EmptyAndDuplicateOptionsFollowLastValueWinsContract) {
  Scope scope;
  const auto result = call(prepare_config_unit_entry, {"prepare_config", "source", "output", "--cameras", "true", "--cameras", "",
    "--model-arguments", "first", "--model-arguments", "second", "--world", "old.sdf", "--world", "final.sdf"});
  ASSERT_EQ(result.status, 0) << result.error;
  EXPECT_EQ(arguments(scope.state, "load_yaml", 0), Json::array({"first"}));
  EXPECT_EQ(arguments(scope.state, "load_yaml", 1), Json::array({"second"}));
  EXPECT_EQ(arguments(scope.state, "prepare_config")[2], (Json{{"cameras", ""}, {"mappings", scope.state.json_result}}));
  EXPECT_EQ(arguments(scope.state, "checked_run")[0][1], "/canonical/final.sdf");
}

TEST(NativeCliUnit, InvalidConfigurationFlagsAndYamlPreventAllPreparationSideEffects) {
  for (const auto &tail : {std::vector<std::string>{"--cameras"}, std::vector<std::string>{"--unknown", "value"},
                           std::vector<std::string>{"--model-arguments"}, std::vector<std::string>{"--world"},
                           std::vector<std::string>{"--help"}}) {
    Scope scope;
    std::vector<std::string> args = {"prepare_config", "source", "output"}; args.insert(args.end(), tail.begin(), tail.end());
    error(call(prepare_config_unit_entry, args), "model preparation",
          tail.size() == 1 ? "missing value for" : "unknown argument:");
    EXPECT_TRUE(scope.state.calls.empty()); guards_closed(scope.state);
  }
  Scope scope;
  scope.state.failure = "load_yaml";
  error(call(prepare_config_unit_entry, {"prepare_config", "source", "output", "--model-arguments", "bad yaml"}),
        "model preparation", "injected load_yaml");
  EXPECT_EQ(names(scope.state), std::vector<std::string>({"load_yaml"}));
}

TEST(NativeCliUnit, OutputDirectoryGuardChecksOnlyNecessaryPredicatesBeforePreparation) {
  for (const bool exists : {false, true}) for (const bool directory : {false, true}) for (const bool empty : {false, true}) {
    Scope scope;
    scope.state.exists = exists; scope.state.directory = directory; scope.state.empty = empty;
    const auto result = call(prepare_config_unit_entry, {"prepare_config", "source", "output"});
    const bool accepted = !exists || (directory && empty);
    EXPECT_EQ(result.status, accepted ? 0 : 1);
    std::vector<std::string> expected = {"canonical_path", "exists"};
    if (exists) expected.push_back("is_directory");
    if (exists && directory) expected.push_back("is_empty");
    if (accepted) {
      expected.push_back("prepare_config");
      EXPECT_EQ(arguments(scope.state, "prepare_config")[2], Json::object());
    } else error(result, "model preparation", "output directory must be empty");
    EXPECT_EQ(names(scope.state), expected);
    for (const auto &predicate : {"exists", "is_directory", "is_empty"})
      if (scope.state.occurrences.count(predicate)) { EXPECT_EQ(arguments(scope.state, predicate), Json::array({"/canonical/output"})); }
    guards_closed(scope.state);
  }
}

TEST(NativeCliUnit, WorldImportDefaultsAndResultPathsAreExact) {
  Scope scope;
  const auto result = call(prepare_config_unit_entry, {"prepare_config", "source", "output", "--world", "world.sdf"});
  ASSERT_EQ(result.status, 0) << result.error;
  EXPECT_EQ(arguments(scope.state, "executable"), Json::array({"litesim3d_assets", "sdf_import"}));
  EXPECT_EQ(arguments(scope.state, "checked_run"), Json::array({
    Json::array({"/provider/bin/sdf_import", "/canonical/world.sdf", "/provider/model/runtime.yaml", "/canonical/output/scene", "false"}),
    120, "", Json::object()}));
  EXPECT_EQ(result.error, scope.state.process_output);
  EXPECT_EQ(Json::parse(result.output), (Json{{"config", "/canonical/output/scene/runtime.yaml"},
                                            {"scene_description", "/canonical/output/scene/scene.json"}}));
  EXPECT_EQ(names(scope.state), std::vector<std::string>({"canonical_path", "exists", "prepare_config", "executable", "canonical_path", "checked_run"}));
}

TEST(NativeCliUnit, WorldImportPropagatesAssetsEnvironmentBudgetAndUnparsedRealTimeFactor) {
  for (const auto &existing : {std::optional<std::string>{}, std::optional<std::string>{""}, std::optional<std::string>{"old:resources"}})
    for (const bool budget : {false, true}) {
      Scope scope;
      Environment environment("GZ_SIM_RESOURCE_PATH", existing);
      scope.state.boolean_result = budget;
      const auto result = call(prepare_config_unit_entry, {"prepare_config", "source", "output", "--world", "world.sdf",
        "--assets-root", "assets", "--collision-budget-enabled", budget ? "YES" : "NO", "--real-time-factor", "provider-validates-rtf"});
      ASSERT_EQ(result.status, 0) << result.error;
      const auto run = arguments(scope.state, "checked_run");
      EXPECT_EQ(run[0], Json::array({"/provider/bin/sdf_import", "/canonical/world.sdf", "/provider/model/runtime.yaml",
                                    "/canonical/output/scene", budget ? "true" : "false", "provider-validates-rtf"}));
      EXPECT_EQ(arguments(scope.state, "boolean"), Json::array({budget ? "YES" : "NO", "collision_budget_enabled"}));
      EXPECT_EQ(run[1], 120); EXPECT_EQ(run[2], "");
      EXPECT_EQ(run[3], (Json{{"GZ_SIM_RESOURCE_PATH", "/canonical/assets/models:/canonical/assets/worlds" +
                                                    (existing ? ":" + *existing : "")}}));
      EXPECT_EQ(arguments(scope.state, "canonical_path", 1), Json::array({"assets"}));
      // The command receives an overlay; the parent's environment is unmodified.
      const char *current = std::getenv("GZ_SIM_RESOURCE_PATH");
      EXPECT_EQ(current != nullptr, existing.has_value());
      if (existing) { ASSERT_NE(current, nullptr); EXPECT_EQ(std::string(current), *existing); }
    }
}

TEST(NativeCliUnit, WorldOnlyOptionsDoNotTriggerImportsOrValidationWithoutWorld) {
  Scope scope;
  const auto result = call(prepare_config_unit_entry, {"prepare_config", "source", "output", "--assets-root", "assets",
    "--collision-budget-enabled", "not-a-boolean", "--real-time-factor", "not-a-number"});
  ASSERT_EQ(result.status, 0) << result.error;
  EXPECT_EQ(names(scope.state), std::vector<std::string>({"canonical_path", "exists", "prepare_config"}));
  EXPECT_EQ(arguments(scope.state, "prepare_config")[2], Json::object());
}

TEST(NativeCliUnit, ConfigurationProviderFailuresNeverRunLaterStagesOrPrintSuccessJson) {
  const std::vector<std::string> args = {"prepare_config", "source", "output", "--world", "world.sdf",
    "--assets-root", "assets", "--collision-budget-enabled", "true"};
  const std::vector<std::string> ordered = {"canonical_path", "exists", "is_directory", "is_empty", "prepare_config",
    "canonical_path", "boolean", "executable", "canonical_path", "checked_run"};
  std::map<std::string, int> occurrences;
  for (size_t index = 0; index < ordered.size(); ++index) {
    SCOPED_TRACE(ordered[index] + " at " + std::to_string(index));
    Scope scope;
    scope.state.exists = true;
    scope.state.failure = ordered[index]; scope.state.failure_occurrence = ++occurrences[ordered[index]];
    error(call(prepare_config_unit_entry, args), "model preparation", "injected " + ordered[index]);
    EXPECT_EQ(names(scope.state), std::vector<std::string>(ordered.begin(), ordered.begin() + index + 1));
    guards_closed(scope.state);
  }
}

TEST(NativeCliUnit, XacroDefaultsAndMixedMappingsArePassedThroughInOrder) {
  for (const bool with_options : {false, true}) {
    Scope scope;
    std::vector<std::string> args = {"prepare_xacro", "input.xacro", "output.urdf"};
    if (with_options) args.insert(args.end(), {"name:=robot", "--resource-base", "old", "empty:=", "--resource-base", "final",
                                             "name:=second", "nested:=a:=b"});
    const auto result = call(prepare_xacro_unit_entry, args);
    ASSERT_EQ(result.status, 0) << result.error;
    EXPECT_TRUE(result.error.empty()); EXPECT_EQ(Json::parse(result.output), scope.state.json_result);
    EXPECT_EQ(names(scope.state), std::vector<std::string>({"prepare_xacro"}));
    EXPECT_EQ(arguments(scope.state, "prepare_xacro"), Json::array({"input.xacro", "output.urdf",
      with_options ? Json::array({"name:=robot", "empty:=", "name:=second", "nested:=a:=b"}) : Json::array(), with_options ? "final" : ""}));
    guards_closed(scope.state);
  }
}

TEST(NativeCliUnit, InvalidXacroArgumentsDoNotStartPreparation) {
  for (const auto &tail : {std::vector<std::string>{"--resource-base"}, std::vector<std::string>{"missing-separator"},
                           std::vector<std::string>{"--unknown", "x"}, std::vector<std::string>{"ok:=1", "bad"},
                           std::vector<std::string>{"--resource-base", "base", "--resource-base"}}) {
    Scope scope;
    std::vector<std::string> args = {"prepare_xacro", "input", "output"}; args.insert(args.end(), tail.begin(), tail.end());
    error(call(prepare_xacro_unit_entry, args), "prepare_xacro");
    EXPECT_TRUE(scope.state.calls.empty()); guards_closed(scope.state);
  }
  Scope scope; scope.state.failure = "prepare_xacro";
  error(call(prepare_xacro_unit_entry, {"prepare_xacro", "input", "output"}), "prepare_xacro", "injected prepare_xacro");
  EXPECT_EQ(names(scope.state), std::vector<std::string>({"prepare_xacro"})); guards_closed(scope.state);
}

TEST(NativeCliUnit, InterruptGuardInstallationFailureCannotReachEitherPreparationProvider) {
  for (const auto &entry : {prepare_config_unit_entry, prepare_xacro_unit_entry}) {
    Scope scope; scope.state.failure = "interrupt";
    const auto result = call(entry, {"command", "input", "output"});
    error(result, entry == prepare_config_unit_entry ? "model preparation" : "prepare_xacro", "injected interrupt");
    EXPECT_TRUE(scope.state.calls.empty()); EXPECT_EQ(scope.state.guards_started, 0); EXPECT_EQ(scope.state.guards_finished, 0);
  }
}

TEST(NativeCliUnit, RspParametersResolveFromCanonicalSourceAndWriteExactRosContract) {
  for (const auto &prefix : {std::optional<std::string>{}, std::optional<std::string>{""},
                             std::optional<std::string>{"robot/"}, std::optional<std::string>{"fleet/robot/"}}) {
    Scope scope;
    std::vector<std::string> args = {"make_rsp_params", "configs/source.yaml", "out.yaml"};
    if (prefix) args.insert(args.end(), {"--frame-prefix", *prefix});
    const auto result = call(make_rsp_params_unit_entry, args);
    ASSERT_EQ(result.status, 0) << result.error;
    EXPECT_EQ(result.output, "/canonical/out.yaml\n"); EXPECT_TRUE(result.error.empty());
    EXPECT_EQ(names(scope.state), std::vector<std::string>({"canonical_path", "read_yaml", "resolve", "ros_description", "dump_yaml", "atomic_write", "canonical_path"}));
    EXPECT_EQ(arguments(scope.state, "read_yaml"), Json::array({"/canonical/configs/source.yaml"}));
    EXPECT_EQ(arguments(scope.state, "resolve"), Json::array({"package://fixture/robot.urdf", "/canonical/configs"}));
    EXPECT_EQ(arguments(scope.state, "ros_description"), Json::array({"/provider/robot.urdf"}));
    const Json expected = {{"/**", {{"ros__parameters", {{"robot_description", scope.state.description},
      {"use_sim_time", true}, {"frame_prefix", prefix.value_or("")}, {"publish_frequency", 200.0}}}}}};
    EXPECT_EQ(arguments(scope.state, "dump_yaml"), Json::array({expected}));
    EXPECT_TRUE(arguments(scope.state, "dump_yaml")[0]["/**"]["ros__parameters"]["publish_frequency"].is_number_float());
    EXPECT_EQ(arguments(scope.state, "atomic_write"), Json::array({"out.yaml", scope.state.serialized}));
    EXPECT_EQ(arguments(scope.state, "canonical_path", 1), Json::array({"out.yaml"}));
    EXPECT_EQ(scope.state.guards_started, 0);
  }
}

TEST(NativeCliUnit, RspRejectsWrongArityFlagsAndInvalidPrefixesBeforeAnyReadOrWrite) {
  for (const auto &tail : {std::vector<std::string>{"extra"}, std::vector<std::string>{"--frame-prefix"},
                           std::vector<std::string>{"--wrong", "valid/"}, std::vector<std::string>{"--frame-prefix", "a/", "extra"},
                           std::vector<std::string>{"--frame-prefix", "/"}, std::vector<std::string>{"--frame-prefix", "/robot/"},
                           std::vector<std::string>{"--frame-prefix", "robot"}}) {
    Scope scope;
    std::vector<std::string> args = {"make_rsp_params", "source", "output"}; args.insert(args.end(), tail.begin(), tail.end());
    error(call(make_rsp_params_unit_entry, args), "make_rsp_params"); EXPECT_TRUE(scope.state.calls.empty());
  }
}

TEST(NativeCliUnit, RspInvalidConfigurationCannotResolveDescriptionOrWrite) {
  for (const auto &configuration : {Json::object(), Json{{"robot", Json::object()}}, Json{{"robot", {{"urdf", 12}}}}, Json::array()}) {
    Scope scope; scope.state.configuration = configuration;
    error(call(make_rsp_params_unit_entry, {"make_rsp_params", "source", "output"}), "make_rsp_params");
    EXPECT_EQ(names(scope.state), std::vector<std::string>({"canonical_path", "read_yaml"}));
  }
}

TEST(NativeCliUnit, RspDependencyFailuresStopBeforeLaterCallsAndDoNotPrintSuccessPaths) {
  const std::vector<std::string> ordered = {"canonical_path", "read_yaml", "resolve", "ros_description", "dump_yaml", "atomic_write", "canonical_path"};
  std::map<std::string, int> occurrences;
  for (size_t index = 0; index < ordered.size(); ++index) {
    Scope scope; scope.state.failure = ordered[index]; scope.state.failure_occurrence = ++occurrences[ordered[index]];
    error(call(make_rsp_params_unit_entry, {"make_rsp_params", "source", "output"}), "make_rsp_params", "injected " + ordered[index]);
    EXPECT_EQ(names(scope.state), std::vector<std::string>(ordered.begin(), ordered.begin() + index + 1));
    // The last canonicalization is deliberately after atomic_write: this test
    // records that ordering rather than pretending the command is transactional.
  }
}

TEST(NativeCliUnit, StreamCaptureRestoresBuffersAndStatesAfterSuccessFailureAndException) {
  Streams restore_original;
  auto *input = std::cin.rdbuf(); auto *output = std::cout.rdbuf(); auto *error_buffer = std::cerr.rdbuf();
  std::cin.setstate(std::ios::eofbit); std::cout.setstate(std::ios::failbit); std::cerr.setstate(std::ios::eofbit);
  const auto input_state = std::cin.rdstate(), output_state = std::cout.rdstate(), error_state = std::cerr.rdstate();
  Scope scope;
  const auto good = request({{"operation", "boolean"}, {"value", "true"}, {"name", "flag"}});
  const auto bad = call(prepare_config_unit_entry, {"prepare_config", "--request"}, "malformed JSON");
  bool exception_observed = false;
  try { call(+[](int, char **) -> int { throw std::runtime_error("uncaught test sentinel"); }, {"sentinel"}); }
  catch (const std::runtime_error &) { exception_observed = true; }
  const bool restored = std::cin.rdbuf() == input && std::cout.rdbuf() == output && std::cerr.rdbuf() == error_buffer &&
    std::cin.rdstate() == input_state && std::cout.rdstate() == output_state && std::cerr.rdstate() == error_state;
  std::cin.clear(); std::cout.clear(); std::cerr.clear();
  EXPECT_EQ(good.status, 0); EXPECT_EQ(bad.status, 1); EXPECT_TRUE(exception_observed); EXPECT_TRUE(restored);
}
