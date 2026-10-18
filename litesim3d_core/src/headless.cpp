#include "litesim3d/sim.hpp"
#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>

int main(int argc, char **argv) {
  try {
    litesim3d::require(argc >= 4 && argc <= 7,
                 "usage: sim_headless CONFIG STEPS OUTPUT.json [v] [w] [rtf; 0=unpaced]");
    const auto count = std::stoull(argv[2]);
    litesim3d::require(count > 0 && count <= 10000000, "steps must be in 1..10000000");
    litesim3d::Scene scene;
    auto config = litesim3d::Config::load(argv[1], scene);
    auto model = litesim3d::Model::load(config.urdf, config.model_options);
    litesim3d::Simulator sim(config, model, scene);
    sim.pause(false);
    uint64_t frames = 0, points = 0;
    const double v = argc > 4 ? std::stod(argv[4]) : 0, w = argc > 5 ? std::stod(argv[5]) : 0,
                 rtf = argc > 6 ? std::stod(argv[6]) : 0;
    litesim3d::require(std::isfinite(rtf) && rtf >= 0, "invalid real time factor");
    auto start = std::chrono::steady_clock::now();
    for (uint64_t i = 0; i < count; ++i) {
      sim.command({v, w});
      for (auto &event : sim.tick())
        for (auto &cloud : event.clouds) {
          ++frames;
          points += uint64_t(cloud.width) * cloud.height;
        }
      if (rtf > 0)
        std::this_thread::sleep_until(
            start + std::chrono::nanoseconds(uint64_t((i + 1) * config.step_ns / rtf)));
    }
    sim.exit();
    auto manifest = sim.manifest("completed");
    manifest["frames"] = frames;
    manifest["point_slots"] = points;
    std::ofstream out(argv[3]);
    litesim3d::require(bool(out), "cannot open output manifest");
    out << manifest.dump(2) << '\n';
    litesim3d::require(bool(out), "cannot write manifest");
    std::cout << "time_ns=" << sim.snapshot().stamp << " frames=" << frames << " points=" << points
              << " contacts=" << sim.contacts().size() << '\n';
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
