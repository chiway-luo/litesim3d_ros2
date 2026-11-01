#include "model.hpp"
#include <iostream>

int main(int argc, char **argv) {
  using namespace litesim3d::bringup;
  try {
    if (argc == 2 && std::string(argv[1]) == "--help") {
      std::cout << "usage: make_rsp_params CONFIG OUTPUT [--frame-prefix PREFIX]\n"; return 0;
    }
    require(argc == 3 || (argc == 5 && std::string(argv[3]) == "--frame-prefix"), "usage: make_rsp_params CONFIG OUTPUT [--frame-prefix PREFIX]");
    const std::string prefix = argc == 5 ? argv[4] : "";
    require(prefix.empty() || (prefix.front() != '/' && prefix.back() == '/'), "frame-prefix must be relative and end in '/'");
    const auto source = canonical_path(argv[1]); const auto config = read_yaml(source);
    const auto model = resolve(config.at("robot").at("urdf"), source.parent_path());
    const Json parameters = {{"/**", {{"ros__parameters", {{"robot_description", ros_description(model)},
      {"use_sim_time", true}, {"frame_prefix", prefix}, {"publish_frequency", 200.0}}}}}};
    atomic_write(argv[2], dump_yaml(parameters)); std::cout << canonical_path(argv[2]).string() << '\n'; return 0;
  } catch (const std::exception &error) { std::cerr << "make_rsp_params: " << error.what() << '\n'; return 1; }
}
