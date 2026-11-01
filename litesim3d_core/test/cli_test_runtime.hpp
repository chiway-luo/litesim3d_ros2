#pragma once
#include "../src/cli_runtime.hpp"
#include <memory>

namespace cli_test {
using Json = litesim3d::Json;
using Geometry = litesim3d::plugins::ModelGeometry;
struct Options { bool allow_visual_only = false; std::string sensor_geometry = "visual"; };
struct Camera { bool enabled = false; };
struct Config {
  uint64_t step_ns = 100;
  std::string urdf = "model.from.configuration";
  Options model_options;
  std::vector<Camera> cameras;
};
struct Cloud { uint32_t width = 0, height = 0; };
struct Event { std::vector<Cloud> clouds; };
struct State {
  Config config;
  Geometry geometry;
  Json manifest = {{"marker", "inert fixture"}};
  std::vector<std::vector<Event>> ticks;
  std::vector<litesim3d::Command> commands;
  std::vector<std::string> calls;
  std::vector<std::pair<std::string, Json>> resolutions;
  std::vector<std::chrono::steady_clock::time_point> sleeps;
  std::chrono::steady_clock::time_point start{std::chrono::nanoseconds(1000)};
  uint64_t stamp = 7;
  size_t tick_count = 0;
  std::vector<int> contacts{1, 2};
  std::string fail, config_file, model_file, base_link, manifest_reason, output_file, output_text;
  Options model_options;
  int scene_token = 0;
  bool exited = false, output_open = true, output_good = true;
  void record(const std::string &operation) {
    calls.push_back(operation);
    if (fail == operation) throw std::runtime_error("injected " + operation);
  }
};
inline std::shared_ptr<State> active;
struct Scene { int token = 0; };
struct Model {
  std::shared_ptr<State> state;
  Geometry mechanicalGeometry(const std::string &base) const {
    state->record("geometry"); state->base_link = base; return state->geometry;
  }
};
struct Simulation {
  std::shared_ptr<State> state;
  Config settings;
  const Config &config() const { state->record("resolved-config"); return settings; }
  void pause(bool paused) { state->record(paused ? "pause" : "resume"); }
  bool command(const litesim3d::Command &command) { state->record("command"); state->commands.push_back(command); return true; }
  std::vector<Event> tick() {
    state->record("tick"); state->stamp += settings.step_ns;
    const auto index = state->tick_count++;
    return index < state->ticks.size() ? state->ticks[index] : std::vector<Event>{};
  }
  void exit() { state->record("exit"); state->exited = true; }
  Json manifest(const std::string &reason) {
    state->record("manifest"); state->manifest_reason = reason; return state->manifest;
  }
  struct Snapshot { uint64_t stamp; };
  Snapshot snapshot() const { return {state->stamp}; }
  const std::vector<int> &contacts() const { return state->contacts; }
};
struct Output {
  std::shared_ptr<State> state;
  bool written = false;
  explicit operator bool() const { return state->output_open && (!written || state->output_good); }
  Output &operator<<(const std::string &text) {
    state->record("write"); written = true;
    if (state->output_good) state->output_text += text;
    return *this;
  }
  Output &operator<<(char value) { return *this << std::string(1, value); }
};
struct Runtime {
  using Scene = cli_test::Scene;
  using ModelOptions = Options;
  static Config loadConfig(const std::string &file, Scene &scene) {
    active->record("config"); active->config_file = file; scene.token = 123; return active->config;
  }
  static Model loadModel(const std::string &file, const ModelOptions &options) {
    active->record("model"); active->model_file = file; active->model_options = options; return {active};
  }
  static Simulation simulation(const Config &config, const Model &, const Scene &scene) {
    active->record("simulation"); active->scene_token = scene.token; return {active, config};
  }
  static auto now() { active->record("now"); return active->start; }
  static void sleepUntil(std::chrono::steady_clock::time_point deadline) {
    active->record("sleep"); active->sleeps.push_back(deadline);
  }
  static Output output(const std::string &file) {
    active->record("output"); active->output_file = file; return {active};
  }
  static Json resolve(const std::string &kind, const Geometry &geometry, const Json &parameters) {
    active->record(kind);
    active->resolutions.emplace_back(kind, parameters);
    return {{"route", kind}, {"parameters", parameters}, {"base", geometry.model.base_link}};
  }
  static Json differentialGeometry(const Geometry &g, const Json &p) { return resolve("differential", g, p); }
  static Json ackermannGeometry(const Geometry &g, const Json &p) { return resolve("ackermann", g, p); }
  static Json omniGeometry(const Geometry &g, const Json &p) { return resolve("omni", g, p); }
};
struct Scope {
  std::shared_ptr<State> previous = active;
  std::shared_ptr<State> state = std::make_shared<State>();
  Scope() { active = state; state->geometry.model.base_link = "fixture-base"; }
  ~Scope() { active = previous; }
};
} // namespace cli_test
