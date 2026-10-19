#include "litesim3d_plugin_api/declarations.hpp"
#include <fstream>
#include <sstream>
#include <tinyxml2.h>
#include <yaml-cpp/yaml.h>

namespace litesim3d::plugins {
namespace {
Json convert(const YAML::Node &node, size_t depth = 0) {
  require(depth <= 32, "parameter YAML nesting exceeds 32");
  if (node.IsMap()) {
    Json result = Json::object();
    for (const auto &entry : node) {
      const auto key = entry.first.as<std::string>();
      require(!result.contains(key), "duplicate parameter key: " + key);
      result[key] = convert(entry.second, depth + 1);
    }
    return result;
  }
  if (node.IsSequence()) {
    require(node.size() <= 65536, "parameter sequence exceeds 65536 items");
    Json result = Json::array(); for (const auto &item : node) result.push_back(convert(item, depth + 1)); return result;
  }
  if (node.IsNull()) return nullptr;
  const auto value = node.as<std::string>();
  if (node.Tag() == "!" || node.Tag() == "tag:yaml.org,2002:str") return value;
  if (value == "true") return true;
  if (value == "false") return false;
  try {
    size_t end = 0;
    const double number = std::stod(value, &end);
    if (end == value.size()) { require(std::isfinite(number), "non-finite YAML number"); return number; }
  } catch (const std::invalid_argument &) {} catch (const std::out_of_range &) {
    throw std::runtime_error("YAML numeric value out of range: " + value);
  }
  return value;
}
void attributes(const tinyxml2::XMLElement *element, const std::set<std::string> &allowed) {
  for (auto *attribute = element->FirstAttribute(); attribute; attribute = attribute->Next())
    require(allowed.count(attribute->Name()), std::string(element->Name()) + ": unknown attribute " + attribute->Name());
}
std::string required(const tinyxml2::XMLElement *element, const char *name) {
  const char *value = element->Attribute(name);
  require(value && *value, std::string(element->Name()) + ": missing attribute " + name);
  return value;
}
Json section(Json document, const char *selection) {
  if (!selection) return document;
  std::stringstream parts(selection); std::string key;
  require(*selection, "parameters.section cannot be empty");
  while (std::getline(parts, key, '.')) {
    require(document.is_object() && document.contains(key), "parameter section not found: " + std::string(selection));
    Json selected = document.at(key); document = std::move(selected);
  }
  require(document.is_object(), "selected parameter section must be a mapping");
  return document;
}
}
Json readParameterFile(const std::filesystem::path &path) {
  require(std::filesystem::is_regular_file(path), "plugin parameter file not found: " + path.string());
  require(std::filesystem::file_size(path) <= 4 * 1024 * 1024, "plugin parameter file exceeds 4 MiB: " + path.string());
  try {
    auto result = convert(YAML::LoadFile(path.string()));
    require(result.is_object(), "plugin parameter file must contain a mapping");
    return result;
  } catch (const std::exception &error) { throw std::runtime_error(path.string() + ": " + error.what()); }
}
std::vector<Declaration> declarations(const std::string &xml, const std::filesystem::path &directory,
                                      const ResourceResolver &resolve) {
  require(xml.size() <= 64 * 1024 * 1024, "model XML exceeds 64 MiB");
  tinyxml2::XMLDocument document;
  require(document.Parse(xml.c_str(), xml.size()) == tinyxml2::XML_SUCCESS, "invalid model XML while parsing plugins");
  const auto *robot = document.FirstChildElement("robot");
  require(robot, "plugin declarations require a robot element");
  std::set<std::string> names, singletons;
  std::vector<Declaration> result;
  for (auto *extension = robot->FirstChildElement("litesim3d"); extension; extension = extension->NextSiblingElement("litesim3d")) {
    attributes(extension, {"version"});
    require(required(extension, "version") == "1", "unsupported LiteSim3D extension version (expected 1)");
    for (auto *element = extension->FirstChildElement(); element; element = element->NextSiblingElement()) {
      Declaration plugin;
      plugin.category = element->Name();
      require(plugin.category == "chassis" || plugin.category == "odometry" || plugin.category == "sensor", "unknown LiteSim3D plugin category: " + plugin.category);
      attributes(element, {"name", "class"});
      plugin.name = required(element, "name"); plugin.class_name = required(element, "class");
      require(names.insert(plugin.name).second, "duplicate LiteSim3D plugin name: " + plugin.name);
      if (plugin.category != "sensor") require(singletons.insert(plugin.category).second, "multiple " + plugin.category + " plugins are not supported");
      bool has_overrides = false;
      for (auto *child = element->FirstChildElement(); child; child = child->NextSiblingElement()) {
        const std::string tag = child->Name();
        require(!child->FirstChildElement(), plugin.name + ": nested XML inside parameter declaration is not allowed");
        if (tag == "parameters") {
          require(!has_overrides, plugin.name + ": parameters must appear before overrides");
          attributes(child, {"file", "section"});
          auto path = resolve(required(child, "file"), directory);
          plugin.parameters = merge(plugin.parameters, section(readParameterFile(path), child->Attribute("section")));
          plugin.parameter_files.push_back(path);
        } else if (tag == "overrides") {
          attributes(child, {});
          require(!has_overrides, plugin.name + ": duplicate overrides element");
          has_overrides = true;
          const auto values = convert(YAML::Load(child->GetText() ? child->GetText() : "{}"));
          require(values.is_object(), plugin.name + ": overrides must be a YAML mapping");
          plugin.parameters = merge(plugin.parameters, values);
        } else throw std::runtime_error(plugin.name + ": unsupported plugin declaration element " + tag);
      }
      result.push_back(std::move(plugin));
      require(result.size() <= 64, "model exceeds 64 plugin instances");
    }
  }
  return result;
}
} // namespace litesim3d::plugins
