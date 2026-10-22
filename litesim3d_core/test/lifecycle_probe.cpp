#include "litesim3d/sim.hpp"
#include <fstream>
#include <iostream>
#include <unistd.h>

using namespace litesim3d;
namespace {
size_t entries(const char *path) {
  return std::distance(std::filesystem::directory_iterator(path), std::filesystem::directory_iterator{});
}
Json resources(int cycle) {
  size_t pages = 0, resident = 0;
  std::ifstream("/proc/self/statm") >> pages >> resident;
  require(resident > 0, "cannot read process RSS");
  return {{"cycle", cycle}, {"pid", getpid()}, {"rss_bytes", resident * sysconf(_SC_PAGESIZE)},
          {"fds", entries("/proc/self/fd")}, {"threads", entries("/proc/self/task")}};
}
}

int main(int argc, char **argv) {
  try {
    require(argc == 2, "usage: lifecycle_probe CONFIG");
    uint64_t reference_stamp = 0;
    // Observe the simulator process itself, after all per-cycle owners are gone.
    // Warmup is included in raw evidence but excluded from the resource delta.
    for (int cycle = 0; cycle < 140; ++cycle) {
      {
        Scene scene;
        auto config = Config::load(argv[1], scene);
        config.lidars.at(0).columns = 16;
        config.lidars.at(0).vertical_angles = {0};
        config.step_ns = 100000000;
        config.threads = 2;
        auto model = Model::load(config.urdf);
        {
          Simulator simulator(config, model, scene);
          require(simulator.snapshot().stamp == 1, "fresh simulator did not reset clock");
          for (int reset = 0; reset < 12; ++reset) simulator.setPose(pose());
          simulator.pause(false);
          require(simulator.command({.1, .2}), "command rejected");
          size_t clouds = 0;
          for (int step = 0; step < 12; ++step) {
            for (const auto &event : simulator.tick()) clouds += event.clouds.size();
          }
          require(clouds > 0, "lifecycle probe never exercised sensor workers");
          if (!reference_stamp) reference_stamp = simulator.snapshot().stamp;
          require(reference_stamp == simulator.snapshot().stamp, "fresh run changed simulated duration");
          simulator.pause(true);
          std::vector<Event> events;
          require(simulator.singleStep("step", 2, events).success, "single step failed");
          simulator.fault("lifecycle test");
          require(simulator.tick().empty(), "fault still produced events");
          simulator.exit();
          require(simulator.state() == State::Exited, "exit failed");
        }
        // Failure after the plugin loader has been constructed must also unwind.
        config.lidars.at(0).link = "missing_sensor_link";
        bool rejected = false;
        try { Simulator invalid(config, model, scene); }
        catch (const std::exception &) { rejected = true; }
        require(rejected, "invalid sensor binding accepted");
      }
      std::cout << resources(cycle).dump() << std::endl;
    }
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
