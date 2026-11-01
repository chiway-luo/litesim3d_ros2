#include "model.hpp"
#include <cstdlib>
#include <iostream>
#include <set>

using namespace litesim3d::bringup;
namespace {
Json dispatch(const Json &request) {
  const auto operation = request.at("operation").get<std::string>();
  if (operation == "prepare") return prepare_config(request.at("source").get<std::string>(), request.at("directory").get<std::string>(), request).string();
  if (operation == "resolve") return resolve(request.at("uri"), request.at("base").get<std::string>()).string();
  if (operation == "load_yaml") return load_yaml(request.at("text"));
  if (operation == "boolean") return boolean(request.at("value"), request.at("name"));
  if (operation == "merge") return merge(request.at("base"), request.at("overlay"));
  if (operation == "profile") return profile(request.at("selection"), request.at("directory").get<std::string>());
  if (operation == "ros_description") return ros_description(request.at("urdf").get<std::string>());
  if (operation == "preview_topics") return preview_topics(request.at("config").get<std::string>(), request.at("interface"), request.at("format"));
  if (operation == "controller_parameters") return controller_parameters(request.at("configuration"), request.at("description"), request.value("base_link", "base_link"));
  if (operation == "model_geometry") return model_geometry(request.at("description"), request.at("base_link"), request.at("requests"));
  if (operation == "override") {
    tinyxml2::XMLDocument document; parse_xml(document, request.at("xml"));
    override_plugin(document.RootElement(), request.at("values")); return xml_text(*document.RootElement());
  }
  if (operation == "noise") {
    tinyxml2::XMLDocument document; Element *root = nullptr;
    if (!request.at("xml").is_null()) { parse_xml(document, request.at("xml")); root = document.RootElement(); }
    auto data = request.at("data");
    apply_noise(data, root, request.at("filename").get<std::string>(), request.at("base").get<std::string>());
    return {{"data", data}, {"xml", root ? Json(xml_text(*root)) : Json(nullptr)}};
  }
  throw std::runtime_error("unknown model operation: " + operation);
}
}
int main(int argc, char **argv) {
  try {
    InterruptGuard interrupts;
    if (argc == 2 && std::string(argv[1]) == "--request") {
      Json request; std::cin >> request; std::cout << dispatch(request).dump() << '\n'; return 0;
    }
    if (argc == 2 && (std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h")) {
      std::cout << "usage: prepare_config CONFIG OUTPUT [--robot-urdf PATH] [--model-arguments YAML]\n"
        "  [--cameras true|false] [--lidar-profile NAME|PATH] [--lidar-profiles-dir DIR]\n"
        "  [--noise-config PATH] [--world SDF] [--assets-root DIR]\n"
        "  [--collision-budget-enabled true|false] [--real-time-factor NUMBER]\n"; return 0;
    }
    require(argc >= 3, "usage: prepare_config CONFIG OUTPUT [options]");
    Json options = Json::object(); std::map<std::string, std::string> extra;
    const std::map<std::string, std::string> flags = {{"--robot-urdf", "robot_urdf"}, {"--cameras", "cameras"},
      {"--lidar-profile", "lidar_profile"}, {"--lidar-profiles-dir", "profiles_dir"}, {"--noise-config", "noise_config"}};
    const std::set<std::string> extra_flags = {"--world", "--assets-root", "--collision-budget-enabled", "--real-time-factor"};
    for (int i = 3; i < argc; ++i) {
      const std::string flag = argv[i]; require(i + 1 < argc, "missing value for " + flag); const std::string value = argv[++i];
      if (flag == "--model-arguments") options["mappings"] = load_yaml(value);
      else if (flags.count(flag)) options[flags.at(flag)] = value;
      else { require(extra_flags.count(flag), "unknown argument: " + flag); extra[flag] = value; }
    }
    const auto output = canonical_path(argv[2]);
    require(!fs::exists(output) || (fs::is_directory(output) && fs::is_empty(output)), "output directory must be empty");
    auto prepared = prepare_config(argv[1], output / "model", options);
    Json result = {{"config", prepared.string()}};
    if (extra.count("--world")) {
      std::map<std::string, std::string> environment;
      if (extra.count("--assets-root")) {
        const auto root = canonical_path(extra["--assets-root"]); const char *existing = std::getenv("GZ_SIM_RESOURCE_PATH");
        environment["GZ_SIM_RESOURCE_PATH"] = (root / "models").string() + ":" + (root / "worlds").string() + (existing ? ":" + std::string(existing) : "");
      }
      const bool budget = extra.count("--collision-budget-enabled") && boolean(extra["--collision-budget-enabled"], "collision_budget_enabled");
      std::vector<std::string> command{executable("litesim3d_assets", "sdf_import").string(), canonical_path(extra["--world"]).string(),
        prepared.string(), (output / "scene").string(), budget ? "true" : "false"};
      if (extra.count("--real-time-factor")) command.push_back(extra["--real-time-factor"]);
      const auto imported = checked_run(command, 120, "", environment); std::cerr << imported;
      result = {{"config", (output / "scene/runtime.yaml").string()}, {"scene_description", (output / "scene/scene.json").string()}};
    }
    std::cout << result.dump() << '\n'; return 0;
  } catch (const std::exception &error) { std::cerr << "model preparation: " << error.what() << '\n'; return 1; }
}
