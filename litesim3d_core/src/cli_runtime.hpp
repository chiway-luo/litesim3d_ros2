#pragma once
#include "litesim3d/sim.hpp"
#include <chrono>
#include <fstream>
#include <thread>

namespace litesim3d {
// Production dependencies for CLI orchestration. Unit tests supply inert data
// and spies at this boundary, not a real parser/model/simulator/sensor pipeline.
// Production defaults are separately exercised by component/integration tests.
struct CliRuntime {
  using Scene = litesim3d::Scene;
  using ModelOptions = litesim3d::ModelOptions;
  static Config loadConfig(const std::filesystem::path &file, Scene &scene) {
    return Config::load(file, scene);
  }
  static Model loadModel(const std::filesystem::path &file, const ModelOptions &options) {
    return Model::load(file, options);
  }
  static Simulator simulation(const Config &config, const Model &model, const Scene &scene) {
    return Simulator(config, model, scene);
  }
  static auto now() { return std::chrono::steady_clock::now(); }
  static void sleepUntil(std::chrono::steady_clock::time_point deadline) { std::this_thread::sleep_until(deadline); }
  static std::ofstream output(const std::string &file) { return std::ofstream(file); }
  static Json differentialGeometry(const plugins::ModelGeometry &geometry, const Json &parameters) {
    return plugins::differentialGeometry(geometry, parameters);
  }
  static Json ackermannGeometry(const plugins::ModelGeometry &geometry, const Json &parameters) {
    return plugins::ackermannGeometry(geometry, parameters);
  }
  static Json omniGeometry(const plugins::ModelGeometry &geometry, const Json &parameters) {
    return plugins::omniGeometry(geometry, parameters);
  }
};
} // namespace litesim3d
