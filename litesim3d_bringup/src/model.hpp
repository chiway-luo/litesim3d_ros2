#pragma once
#include <litesim3d_runtime/runtime.hpp>
#include <tinyxml2.h>
#include <functional>

namespace litesim3d::bringup {
using namespace litesim3d::runtime;
using Element = tinyxml2::XMLElement;
std::string xml_text(const tinyxml2::XMLNode &node);
void parse_xml(tinyxml2::XMLDocument &document, const std::string &text);
void pin_resources(Element *root, const fs::path &base);
void override_plugin(Element *sensor, const Json &values);
Json profile(const std::string &selection, const fs::path &directory);
void apply_noise(Json &data, Element *root, const fs::path &filename, const fs::path &base);
using CommandRunner = std::function<std::string(const std::vector<std::string> &, double, const std::string &, const std::map<std::string, std::string> &)>;
fs::path prepare_config(const fs::path &source, const fs::path &directory, const Json &options,
                        const CommandRunner &runner = checked_run);
std::string ros_description(const fs::path &urdf);
Json preview_topics(const fs::path &config, const std::string &interface, int format);
Json model_geometry(const std::string &description, const std::string &base, const Json &requests);
using GeometryProvider = std::function<Json(const std::string &, const std::string &, const Json &)>;
Json controller_parameters(Json configuration, const std::string &description, const std::string &base,
                           const GeometryProvider &provider = model_geometry);
Json prepare_xacro(const fs::path &source, const fs::path &output, const std::vector<std::string> &mappings,
                   const fs::path &resource_base, const CommandRunner &runner = checked_run);
}  // namespace litesim3d::bringup
