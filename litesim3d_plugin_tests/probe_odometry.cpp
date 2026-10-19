#include <litesim3d_plugin_api/declarations.hpp>
#include <iostream>

int main(int argc, char **argv) {
  using namespace litesim3d::plugins;
  try {
    require(argc == 3, "usage: plugin_odometry_probe CLASS PARAMETER_FILE");
    Loader loader;
    auto plugin = loader.odometry(argv[1]);
    Initialization init; init.instance = "external_test"; init.model.base_link = "base_link";
    init.model.links.insert("base_link");
    JointState before;
    for (const auto &side : {std::string("left"), std::string("right")}) {
      const std::string name = side + "_wheel_joint";
      JointDescription wheel; wheel.type = "continuous";
      wheel.position = {0, side == "left" ? .25 : -.25, .1};
      init.model.joints[name] = wheel; before.position[name] = 0; before.velocity[name] = 0;
    }
    auto params = readParameterFile(argv[2]);
    const auto schema = plugin->defaults();
    require(schema.size() == params.size(), "external parameter file must cover every parameter");
    for (const auto &[key, value] : schema.items()) {
      require(params.contains(key), "external parameter file missing: " + key);
      if (!value.is_null()) require(params.at(key) == value, "external optional parameter differs from default: " + key);
    }
    plugin->initialize(init, params); plugin->reset(before);
    auto after = before;
    for (auto &[name, value] : after.position) value = 10;
    for (auto &[name, value] : after.velocity) value = 10;
    auto result = plugin->step({1000000000, before, after});
    std::cout << Json{{"class", argv[1]}, {"parameter_file_complete", true}, {"x_m", result.pose.translation().x()},
                     {"y_m", result.pose.translation().y()}, {"vx_mps", result.velocity.vx}}.dump(2) << '\n';
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n'; return 1;
  }
}
