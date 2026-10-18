#include "litesim3d/sim.hpp"
#include <chrono>
#include <fstream>
#include <iostream>
int main(int argc, char **argv) {
  try {
    litesim3d::require(argc == 3, "usage: sim_load_benchmark CONFIG OUTPUT.json");
    auto control = std::make_shared<litesim3d::LoadControl>();
    litesim3d::Json rows = litesim3d::Json::array();
    std::vector<std::unique_ptr<litesim3d::Simulator>> retained;
    for (int i = 0; i < 2; ++i) {
      size_t resident = 0;
      for (const auto &entry : control->meshes) resident += !entry.second.expired();
      auto start = std::chrono::steady_clock::now();
      litesim3d::Scene scene;
      auto cfg = litesim3d::Config::load(argv[1], scene, control);
      auto model = litesim3d::Model::load(cfg.urdf, cfg.model_options);
      retained.push_back(std::make_unique<litesim3d::Simulator>(cfg, model, scene));
      rows.push_back({{"application_cache", i == 0 ? "empty" : "resident_geometry"},
          {"resident_meshes_before", resident},
          {"load_and_initialization_seconds", std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count()},
          {"unique_loaded_triangles", control->triangles}});
    }
    litesim3d::Json report = {{"status", "MEASURED"}, {"runs", rows},
        {"config_sha256", litesim3d::sha256(argv[1])}, {"source_sha256", retained[0]->manifest()["core_source_sha256"]},
        {"os_page_cache", "not flushed; no claim of cold physical disk latency"},
        {"definition", "cold = new process with empty application geometry cache; hot = same process retaining loaded geometry"}};
    std::ofstream output(argv[2]); litesim3d::require(bool(output), "cannot open output");
    output << report.dump(2) << '\n'; litesim3d::require(bool(output), "cannot write output");
    std::cout << report.dump() << '\n';
    return 0;
  } catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
}
