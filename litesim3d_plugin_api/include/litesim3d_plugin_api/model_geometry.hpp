#pragma once
#include "litesim3d_plugin_api/api.hpp"

namespace litesim3d::plugins {
// Immutable mechanical evidence supplied by the host, never a second geometry configuration.
// A radius is present only for an unambiguous, centered, coaxial collision cylinder.
struct ModelGeometry {
  ModelDescription model;
  std::map<std::string, double> wheel_radii;
};

// Optional capability: existing plugin vtables and Initialization stay unchanged.
// Called before defaults()/parameters() validation; returned values are recorded in the manifest.
class ModelParameters {
public:
  virtual ~ModelParameters();
  virtual Json resolveModelParameters(const ModelGeometry &, const Json &) const = 0;
};

Json differentialGeometry(const ModelGeometry &, const Json &);
Json ackermannGeometry(const ModelGeometry &, const Json &);
Json omniGeometry(const ModelGeometry &, const Json &);
} // namespace litesim3d::plugins
