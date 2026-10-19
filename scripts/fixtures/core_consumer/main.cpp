#include <litesim3d/sim.hpp>
#include <iostream>

int main(int argc, char **argv) {
  if (argc != 3) return 2;
  litesim3d::Scene scene;
  auto config = litesim3d::Config::load(argv[1], scene);
  auto model = litesim3d::Model::load(config.urdf, config.model_options);
  litesim3d::Simulator sim(config, model, scene);
  sim.pause(false);
  for (int i = 0; i < 20; ++i) sim.tick();
  auto report = sim.manifest("installed-consumer");
  std::cout << report.dump(2) << '\n';
  return report.at("version") == argv[2] && sim.stepIndex() == 20 ? 0 : 1;
}
