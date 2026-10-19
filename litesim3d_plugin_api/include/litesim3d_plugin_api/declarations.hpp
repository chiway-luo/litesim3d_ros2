#pragma once
#include "litesim3d_plugin_api/api.hpp"
#include <filesystem>
#include <functional>

namespace litesim3d::plugins {
struct Declaration {
  std::string category, name, class_name;
  Json parameters = Json::object();
  std::vector<std::filesystem::path> parameter_files;
};
using ResourceResolver = std::function<std::filesystem::path(const std::string &, const std::filesystem::path &)>;
// Parse only LiteSim3D extension elements. The host separately validates URDF.
// Instance values are merged here; implementation defaults/validation are applied at initialize().
std::vector<Declaration> declarations(const std::string &xml, const std::filesystem::path &model_directory,
                                      const ResourceResolver &resolve);
Json readParameterFile(const std::filesystem::path &);
} // namespace litesim3d::plugins
