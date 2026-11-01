#include "model.hpp"
#include <functional>
#include <set>

namespace litesim3d::bringup {
namespace {
std::string attribute(Element *element, const char *name) {
  const char *value = element->Attribute(name); return value ? value : "";
}
void walk(Element *element, const std::function<void(Element *)> &visit) {
  if (!element) return;
  visit(element);
  for (auto *child = element->FirstChildElement(); child; child = child->NextSiblingElement()) walk(child, visit);
}
std::vector<Element *> declarations(Element *root, const char *kind) {
  std::vector<Element *> result;
  if (root) for (auto *block = root->FirstChildElement("litesim3d"); block; block = block->NextSiblingElement("litesim3d"))
    for (auto *item = block->FirstChildElement(kind); item; item = item->NextSiblingElement(kind)) result.push_back(item);
  return result;
}
const std::map<std::string, std::string> sensor_kinds = {
  {"litesim3d_lidar/Lidar", "lidar"}, {"litesim3d_mid360/Mid360", "lidar"}, {"litesim3d_imu/Imu", "imu"},
  {"litesim3d_rgb_camera/RgbCamera", "rgb"}, {"litesim3d_depth_camera/DepthCamera", "depth"},
  {"litesim3d_rgbd_camera/RgbdCamera", "rgbd"}};
std::string sensor_kind(Element *sensor) {
  auto found = sensor_kinds.find(attribute(sensor, "class")); return found == sensor_kinds.end() ? "" : found->second;
}
Json &section(Json &document, const std::string &path) {
  auto *node = &document; std::istringstream input(path); std::string key;
  while (std::getline(input, key, '.')) if (!key.empty()) node = &node->at(key);
  return *node;
}
bool populated(const Json &document, const std::string &key) {
  if (!document.contains(key) || document[key].is_null()) return false;
  const auto &value = document[key];
  return value.is_string() ? !value.get_ref<const std::string &>().empty() : !value.empty();
}
void check_fields(const Json &values, const std::set<std::string> &fields, const std::string &name) {
  require(values.is_object(), name + ": unsupported noise fields; expected a mapping");
  for (auto it = values.begin(); it != values.end(); ++it)
    require(fields.count(it.key()), name + ": unsupported noise fields: " + it.key());
}
}
std::string xml_text(const tinyxml2::XMLNode &node) {
  tinyxml2::XMLPrinter printer(nullptr, true); node.Accept(&printer); return printer.CStr();
}
void parse_xml(tinyxml2::XMLDocument &document, const std::string &text) {
  const auto result = document.Parse(text.c_str(), text.size());
  if (result != tinyxml2::XML_SUCCESS) throw std::runtime_error(std::string("invalid XML: ") + document.ErrorStr());
  require(document.RootElement(), "empty XML document");
}
void pin_resources(Element *root, const fs::path &base) {
  walk(root, [&](Element *element) {
    const std::string tag = element->Name();
    if ((tag == "mesh" || tag == "texture") && element->Attribute("filename"))
      element->SetAttribute("filename", resolve(attribute(element, "filename"), base).c_str());
  });
  if (root) for (auto *block = root->FirstChildElement("litesim3d"); block; block = block->NextSiblingElement("litesim3d"))
    for (auto *plugin = block->FirstChildElement(); plugin; plugin = plugin->NextSiblingElement())
      for (auto *params = plugin->FirstChildElement("parameters"); params; params = params->NextSiblingElement("parameters"))
        if (params->Attribute("file")) params->SetAttribute("file", resolve(attribute(params, "file"), base).c_str());
}
void override_plugin(Element *sensor, const Json &values) {
  require(values.is_object(), "plugin overrides must be a mapping");
  auto *element = sensor->FirstChildElement("overrides");
  require(!element || !element->NextSiblingElement("overrides"), "duplicate overrides: " + attribute(sensor, "name"));
  if (!element) { element = sensor->GetDocument()->NewElement("overrides"); sensor->InsertEndChild(element); }
  auto old = load_yaml(element->GetText() ? element->GetText() : "{}");
  require(old.is_object(), "plugin overrides must be a mapping");
  element->SetText(("\n" + dump_yaml(merge(old, values))).c_str());
}
Json profile(const std::string &selection, const fs::path &directory) {
  fs::path source;
  if (selection == "vlp16" || selection == "mid360" || selection == "mid40") {
    require(!directory.empty(), "named lidar_profile requires lidar_profiles_dir");
    source = directory / (selection + ".yaml");
  } else source = canonical_path(selection);
  require(fs::is_regular_file(source), "lidar_profile must be config, vlp16, mid360, mid40, or an existing YAML path");
  const auto data = read_yaml(source);
  require(data.is_object() && data.size() == 1 && data.contains("lidar") && data["lidar"].is_object(),
          "lidar profile must contain a single lidar mapping");
  auto settings = data["lidar"];
  for (const auto *key : {"name", "link", "topic"})
    require(!settings.contains(key), "lidar profile cannot replace name, link or topic bindings");
  if (populated(settings, "pattern_file")) settings["pattern_file"] = resolve(settings["pattern_file"], source.parent_path()).string();
  return settings;
}
void apply_noise(Json &data, Element *root, const fs::path &filename, const fs::path &base) {
  const auto document = read_yaml(resolve(filename.string(), base));
  require(document.is_object(), "noise_config accepts only odometry and sensors mappings");
  for (auto it = document.begin(); it != document.end(); ++it)
    require(it.key() == "odometry" || it.key() == "sensors", "noise_config accepts only odometry and sensors mappings");
  if (document.contains("odometry")) {
    const auto odometry = declarations(root, "odometry");
    const std::set<std::string> supported = {"litesim3d_diff_odometry/DifferentialOdometry",
      "litesim3d_ackermann_odometry/AckermannOdometry", "litesim3d_omni_odometry/OmniOdometry"};
    require(odometry.size() == 1 && supported.count(attribute(odometry.front(), "class")),
      "noise_config.odometry requires one standard wheel odometry plugin; legacy YAML uses odom");
    check_fields(document["odometry"], {"linear_scale_error", "angular_scale_error", "linear_bias_mps",
      "linear_sigma_mps", "angular_bias_radps", "angular_sigma_radps"}, "odometry");
    override_plugin(odometry.front(), {{"noise", document["odometry"]}});
  }
  const auto requested = document.value("sensors", Json::object());
  require(requested.is_object(), "noise_config.sensors must be a mapping keyed by sensor name");
  struct Binding { std::string kind; Element *plugin; Json *legacy; };
  std::map<std::string, Binding> bindings;
  auto add = [&](const std::string &name, Binding binding) {
    require(bindings.emplace(name, binding).second, "ambiguous sensor name in noise_config: " + name);
  };
  for (auto *sensor : declarations(root, "sensor")) add(attribute(sensor, "name"), {sensor_kind(sensor), sensor, nullptr});
  for (const auto &[group, kind] : std::vector<std::pair<std::string, std::string>>{{"lidars", "lidar"}, {"imus", "imu"}, {"cameras", ""}})
    if (data.contains(group)) for (auto &sensor : data[group])
      add(sensor.value("name", ""), {kind.empty() ? sensor.value("type", "rgb") : kind, nullptr, &sensor});
  const std::map<std::string, std::set<std::string>> fields = {
    {"lidar", {"range_bias_m", "range_sigma_m", "dropout"}},
    {"imu", {"accel_bias", "gyro_bias", "accel_sigma", "gyro_sigma"}},
    {"rgb", {"color_bias_dn", "color_sigma_dn"}}, {"depth", {"depth_bias_m", "depth_sigma_m"}},
    {"rgbd", {"color_bias_dn", "color_sigma_dn", "depth_bias_m", "depth_sigma_m"}}};
  for (auto it = requested.begin(); it != requested.end(); ++it) {
    require(bindings.count(it.key()), "noise_config: unknown sensor name: " + it.key());
    auto binding = bindings.at(it.key());
    require(fields.count(binding.kind), "noise_config: unsupported sensor type: " + it.key());
    check_fields(it.value(), fields.at(binding.kind), "sensors." + it.key());
    if (binding.plugin) override_plugin(binding.plugin, it.value()); else binding.legacy->update(it.value());
  }
}
fs::path prepare_config(const fs::path &input, const fs::path &output, const Json &options, const CommandRunner &runner) {
  const auto source = canonical_path(input), directory = canonical_path(output);
  require(directory != source.parent_path(), "prepared output must not overwrite the source directory");
  fs::create_directories(directory);
  auto data = read_yaml(source);
  require(data.is_object() && data.contains("robot") && data["robot"].is_object(), "configuration must contain a robot mapping");
  const auto robot_urdf = options.value("robot_urdf", "");
  if (!robot_urdf.empty()) data["robot"]["urdf"] = robot_urdf;
  auto mappings = options.contains("mappings_yaml") ? load_yaml(options.at("mappings_yaml")) : options.value("mappings", Json::object());
  if (mappings.is_null()) mappings = Json::object();
  require(mappings.is_object(), "model_arguments must be a YAML mapping of scalar Xacro arguments");
  for (const auto &value : mappings) require(!value.is_structured(), "model_arguments must be a YAML mapping of scalar Xacro arguments");
  tinyxml2::XMLDocument model_xml; Element *root = nullptr;
  std::vector<Element *> sensors;
  if (populated(data["robot"], "urdf")) {
    const auto model = resolve(data["robot"]["urdf"], source.parent_path());
    require(directory != model.parent_path(), "prepared output must not overwrite the model directory");
    std::string expanded;
    if (model.extension() == ".xacro") {
      std::vector<std::string> command{"xacro", model.string()};
      for (auto it = mappings.begin(); it != mappings.end(); ++it)
        command.push_back(it.key() + ":=" + (it.value().is_string() ? it.value().get<std::string>() : it.value().is_null() ? "None" : it.value().dump()));
      expanded = runner(command, 60, "", {});
    } else {
      require(mappings.empty(), "model_arguments requires a .xacro model"); expanded = read(model);
    }
    parse_xml(model_xml, expanded); root = model_xml.RootElement();
    require(std::string(root->Name()) == "robot", "model must contain a robot root");
    pin_resources(root, model.parent_path());
    sensors = declarations(root, "sensor");
    for (std::size_t index = 0; index < sensors.size(); ++index) {
      auto *sensor = sensors[index]; if (sensor_kind(sensor) != "lidar") continue;
      std::size_t file_index = 0;
      for (auto *params = sensor->FirstChildElement("parameters"); params; params = params->NextSiblingElement("parameters"), ++file_index) {
        auto document = read_yaml(attribute(params, "file"));
        auto &selected = section(document, attribute(params, "section"));
        if (selected.is_object() && populated(selected, "pattern_file")) {
          selected["pattern_file"] = resolve(selected["pattern_file"], model.parent_path()).string();
          const auto pinned = directory / ("sensor-" + std::to_string(index) + "-parameters-" + std::to_string(file_index) + ".yaml");
          write(pinned, dump_yaml(document)); params->SetAttribute("file", pinned.c_str());
        }
      }
      auto *element = sensor->FirstChildElement("overrides");
      if (element) {
        require(!element->NextSiblingElement("overrides"), "duplicate overrides: " + attribute(sensor, "name"));
        const auto selected = load_yaml(element->GetText() ? element->GetText() : "{}");
        if (selected.is_object() && populated(selected, "pattern_file"))
          override_plugin(sensor, {{"pattern_file", resolve(selected["pattern_file"], model.parent_path()).string()}});
      }
    }
  } else require(mappings.empty(), "model_arguments requires robot.urdf");
  const auto cameras = options.value("cameras", "");
  if (!cameras.empty()) {
    const bool enabled = boolean(cameras, "cameras"); std::vector<Element *> plugins;
    for (auto *sensor : sensors) { const auto kind = sensor_kind(sensor);
      if (kind == "rgb" || kind == "depth" || kind == "rgbd") plugins.push_back(sensor); }
    require(!enabled || !plugins.empty() || populated(data, "cameras"), "cameras:=true requires a camera plugin or legacy camera definitions");
    for (auto *sensor : plugins) override_plugin(sensor, {{"enabled", enabled}});
    if (data.contains("cameras")) for (auto &sensor : data["cameras"]) sensor["enabled"] = enabled;
  }
  const auto selection = options.value("lidar_profile", "config");
  if (selection != "config") {
    auto settings = profile(selection, options.value("profiles_dir", ""));
    std::vector<Element *> lidars; for (auto *sensor : sensors) if (sensor_kind(sensor) == "lidar") lidars.push_back(sensor);
    if (lidars.size() == 1 && !populated(data, "lidars")) {
      require(settings.value("point_layout", "xyzirt_v1") == "xyzirt_v1", "only xyzirt_v1 point layout is supported"); settings.erase("point_layout");
      if (!settings.contains("vertical_angles_deg")) settings["vertical_angles_deg"] = {0};
      require(settings["vertical_angles_deg"].is_array(), "vertical_angles_deg must be a list");
      if (settings.contains("rings") && !settings["rings"].is_null())
        require(settings["rings"] == settings["vertical_angles_deg"].size(), "profile rings must match vertical_angles_deg");
      settings.erase("rings"); if (!settings.contains("ring_delays_s")) settings["ring_delays_s"] = Json::array();
      if (settings.contains("scan_pattern")) lidars.front()->SetAttribute("class", settings["scan_pattern"] == "livox_mid360" ? "litesim3d_mid360/Mid360" : "litesim3d_lidar/Lidar");
      override_plugin(lidars.front(), settings);
    } else if (lidars.empty() && data.contains("lidars") && data["lidars"].is_array() && data["lidars"].size() == 1) {
      const auto binding = data["lidars"][0];
      for (const auto *key : {"name", "link", "topic"}) if (binding.contains(key)) settings[key] = binding[key];
      data["lidars"] = Json::array({settings});
    } else throw std::runtime_error("lidar_profile requires exactly one lidar and no mixed plugin/YAML sensor definitions");
  }
  if (!options.value("noise_config", "").empty()) apply_noise(data, root, options["noise_config"].get<std::string>(), source.parent_path());
  if (root) {
    const auto combined = directory / "robot.urdf"; write(combined, xml_text(model_xml)); data["robot"]["urdf"] = combined.string();
  }
  if (data.contains("lidars")) for (auto &sensor : data["lidars"])
    if (populated(sensor, "pattern_file")) sensor["pattern_file"] = resolve(sensor["pattern_file"], source.parent_path()).string();
  if (data.contains("entities")) for (auto &entity : data["entities"])
    for (const auto *role : {"geometry", "visual", "collision", "sensor_geometry"})
      if (entity.contains(role) && entity[role].is_object() && entity[role].value("type", "") == "mesh")
        entity[role]["uri"] = resolve(entity[role].at("uri"), source.parent_path()).string();
  const auto destination = directory / "config.yaml"; write(destination, dump_yaml(data)); return destination;
}
std::string ros_description(const fs::path &urdf) {
  tinyxml2::XMLDocument document; parse_xml(document, read(urdf));
  walk(document.RootElement(), [](Element *element) {
    const std::string tag = element->Name(), filename = attribute(element, "filename");
    if ((tag == "mesh" || tag == "texture") && fs::path(filename).is_absolute())
      element->SetAttribute("filename", file_uri(filename).c_str());
  });
  return xml_text(*document.RootElement());
}
Json preview_topics(const fs::path &config, const std::string &interface, int format) {
  auto result = Json::object(); if (interface == "native") return result;
  const auto data = read_yaml(config); auto lidars = data.value("lidars", Json::array());
  const auto model = resolve(data.at("robot").at("urdf"), config.parent_path());
  tinyxml2::XMLDocument document; parse_xml(document, read(model));
  for (auto *sensor : declarations(document.RootElement(), "sensor")) {
    if (sensor_kind(sensor) != "lidar") continue;
    auto values = Json::object(); if (attribute(sensor, "class") == "litesim3d_mid360/Mid360") values["scan_pattern"] = "livox_mid360";
    for (auto *params = sensor->FirstChildElement("parameters"); params; params = params->NextSiblingElement("parameters")) {
      auto settings = read_yaml(resolve(attribute(params, "file"), model.parent_path()));
      values = merge(values, section(settings, attribute(params, "section")));
    }
    for (auto *element = sensor->FirstChildElement("overrides"); element; element = element->NextSiblingElement("overrides"))
      values = merge(values, load_yaml(element->GetText() ? element->GetText() : "{}"));
    values["name"] = attribute(sensor, "name"); lidars.push_back(values);
  }
  std::vector<Json> selected;
  for (const auto &lidar : lidars) if (lidar.value("scan_pattern", "") == "livox_mid360" && lidar.value("enabled", true)) selected.push_back(lidar);
  for (const auto &lidar : selected) {
    auto topic = lidar.at("topic").get<std::string>(); topic.erase(0, topic.find_first_not_of('/')); topic = "/" + topic;
    const auto stem = selected.size() == 1 ? "/livox/lidar" : "/livox/" + lidar.at("name").get<std::string>() + "/lidar";
    result[topic] = stem + (format == 1 ? "/pointcloud" : ""); result[topic + "/range_boundary"] = stem + "/range_boundary";
  }
  return result;
}
}  // namespace litesim3d::bringup
