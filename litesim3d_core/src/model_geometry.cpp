#include "cli_runtime.hpp"
#include <iostream>

// Read-only model inspection for adapters such as ros2_control parameter preparation.
// Uses exactly the same parser, FK and collision geometry as the simulator.
#ifndef LITESIM_CORE_CLI_RUNTIME
#define LITESIM_CORE_CLI_RUNTIME litesim3d::CliRuntime
#endif
int main(int argc, char **argv) {
  using Runtime = LITESIM_CORE_CLI_RUNTIME;
  try {
    litesim3d::require(argc == 3 || (argc == 4 && std::string(argv[3]) == "--resolve"),
                      "usage: model_geometry expanded.urdf base_link [--resolve < requests.json]");
    Runtime::ModelOptions options; options.allow_visual_only = true; options.sensor_geometry = "collision";
    const auto geometry = Runtime::loadModel(argv[1], options).mechanicalGeometry(argv[2]);
    litesim3d::Json joints = litesim3d::Json::object();
    for (const auto &[name, j] : geometry.model.joints) {
      joints[name] = {{"type", j.type}, {"mimic", j.mimic}, {"parent_joint", j.parent_joint},
        {"position", {j.position.x(), j.position.y(), j.position.z()}},
        {"axis", {j.axis.x(), j.axis.y(), j.axis.z()}},
        {"lower", j.lower}, {"upper", j.upper}, {"max_velocity", j.max_velocity}};
      if (geometry.wheel_radii.count(name)) joints[name]["radius"] = geometry.wheel_radii.at(name);
    }
    litesim3d::Json result = {{"base_link", geometry.model.base_link}, {"joints", joints}};
    if (argc == 4) {
      litesim3d::Json requests; std::cin >> requests;
      litesim3d::require(requests.is_array() && requests.size() <= 64, "expected at most 64 geometry requests");
      result["resolved"] = litesim3d::Json::array();
      for (const auto &request : requests) {
        const auto kind = request.at("kind").get<std::string>();
        const auto &p = request.at("parameters");
        if (kind == "differential") result["resolved"].push_back(Runtime::differentialGeometry(geometry, p));
        else if (kind == "ackermann") result["resolved"].push_back(Runtime::ackermannGeometry(geometry, p));
        else if (kind == "omni") result["resolved"].push_back(Runtime::omniGeometry(geometry, p));
        else throw std::runtime_error("unknown geometry request: " + kind);
      }
    }
    std::cout << result.dump() << '\n';
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "model geometry: " << e.what() << '\n';
    return 1;
  }
}
