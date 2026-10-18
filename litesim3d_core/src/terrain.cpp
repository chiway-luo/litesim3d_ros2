#include "litesim3d/sim.hpp"

namespace litesim3d {
void TerrainConfig::validate() const {
  require(std::isfinite(max_slope) && max_slope > 0 && max_slope < pi / 2 &&
              std::isfinite(clearance) && clearance >= 0 && std::isfinite(continuity) &&
              continuity > 0 && continuity <= .1, "terrain: invalid limits");
}
void Scene::updateEntity(const std::string &id, const Pose &transform) {
  require(transform.matrix().allFinite() &&
              (transform.linear().transpose() * transform.linear()).isApprox(Eigen::Matrix3d::Identity(), 1e-9) &&
              transform.linear().determinant() > 0, "entity update: invalid transform");
  Scene candidate = *this;
  bool found = false;
  for (auto *role : {&candidate.visual, &candidate.collision, &candidate.sensor})
    for (auto &geometry : *role) if (geometry.entity == id) {
      geometry.transform = transform;
      found = true;
    }
  require(found, "entity update: unknown id " + id);
  candidate.rebuild();
  *this = std::move(candidate);
}
std::optional<Support> Scene::supportAt(double x, double y, double reference_z, double max_delta,
                                       const std::string &entity) const {
  require(std::isfinite(x) && std::isfinite(y) && std::isfinite(reference_z) &&
              std::isfinite(max_delta) && max_delta >= 0, "support query: invalid bounds");
  std::optional<Support> best;
  auto accept = [&](const Vec3 &point, const Vec3 &normal, const std::string &id) {
    if (normal.z() <= 1e-9 || std::abs(point.z() - reference_z) > max_delta + 1e-9) return;
    if (!best || std::abs(point.z() - reference_z) < std::abs(best->point.z() - reference_z))
      best = Support{point, normal, id};
  };
  for (const auto &g : collision) if (g.support && (entity.empty() || entity == g.entity)) {
    const Pose inverse = g.transform.inverse();
    const Vec3 origin = inverse * Vec3(x, y, reference_z + max_delta + 1),
               down = inverse.linear() * -Vec3::UnitZ();
    if (g.shape->kind == Shape::Kind::Box) {
      // Only the designated upper local face supports a robot; an underside remains a ray/collision face.
      const Vec3 normal = g.transform.linear().col(2);
      if (normal.z() <= 1e-9) continue;
      const double t = (g.shape->size.z() / 2 - origin.z()) / down.z();
      const Vec3 p = origin + t * down;
      if (std::abs(p.x()) <= g.shape->size.x() / 2 + 1e-9 &&
          std::abs(p.y()) <= g.shape->size.y() / 2 + 1e-9)
        accept(g.transform * p, normal, g.entity);
    } else if (g.shape->kind == Shape::Kind::Mesh) {
      g.shape->bvh.query(origin, down, 2 * max_delta + 2, [&](size_t i) {
        const auto &t = g.shape->triangles[i];
        const Vec3 e1 = t.b - t.a, e2 = t.c - t.a, normal = e1.cross(e2).normalized();
        const double denominator = normal.dot(down);
        if (std::abs(denominator) < 1e-12) return;
        const Vec3 p = origin + down * (normal.dot(t.a - origin) / denominator);
        if ((t.b - t.a).cross(p - t.a).dot(normal) >= -1e-10 &&
            (t.c - t.b).cross(p - t.b).dot(normal) >= -1e-10 &&
            (t.a - t.c).cross(p - t.c).dot(normal) >= -1e-10)
          accept(g.transform * p, g.transform.linear() * normal, g.entity);
      });
    }
  }
  return best;
}
std::vector<Vec3> Simulator::supportPoints() const {
  std::vector<Vec3> points;
  std::set<std::string> found;
  for (const auto &g : model_.placed(model_.collision, Pose::Identity(),
                                    config_.base_link, current_.joints)) {
    if (!config_.drive.support_links.count(g.link)) continue;
    found.insert(g.link);
    auto add = [&](const Vec3 &point) { points.emplace_back(point.x(), point.y(), 0); };
    if (g.shape->kind == Shape::Kind::Cylinder) {
      // The contact strip spans the wheel width. Axle coordinates are invariant
      // under wheel spin, unlike a rotated bounding box's corners.
      const Vec3 half_axle = g.transform.linear().col(2) * (g.shape->size.z() / 2);
      add(g.transform.translation());
      add(g.transform.translation() - half_axle);
      add(g.transform.translation() + half_axle);
    } else if (g.shape->kind == Shape::Kind::Sphere) {
      add(g.transform.translation());
    } else {
      // Non-wheel support shapes use a conservative projected footprint.
      const auto b = g.worldBounds();
      for (double x : {b.lo.x(), b.hi.x()})
        for (double y : {b.lo.y(), b.hi.y()}) add({x, y, 0});
    }
  }
  require(!points.empty(), "terrain: no support collision geometry");
  require(found == config_.drive.support_links,
          "terrain: every support link requires collision geometry");
  return points;
}
Pose supportedPose(const Scene &scene, const Pose &candidate, double reference_z,
                   const TerrainConfig &cfg, const std::vector<Vec3> &support_points) {
  cfg.validate();
  require(!support_points.empty(), "terrain: empty support footprint");
  const Vec3 center = candidate.translation();
  auto support = scene.supportAt(center.x(), center.y(), reference_z - cfg.clearance, cfg.continuity);
  require(bool(support), "terrain stop: void or discontinuous support layer");
  require(support->normal.z() >= std::cos(cfg.max_slope) - 1e-12, "terrain stop: maximum slope exceeded");
  const double heading = yaw(candidate);
  const Vec3 forward(std::cos(heading), std::sin(heading), 0);
  Pose result = candidate;
  result.translation().z() = support->point.z() + cfg.clearance;
  const Vec3 z = support->normal;
  const Vec3 f = (forward - z * forward.dot(z)).normalized();
  result.linear().col(0) = f;
  result.linear().col(1) = z.cross(f);
  result.linear().col(2) = z;
  // Check the actual model footprint on the selected tangent plane, including
  // its rotation on slopes; never choose a different bridge/floor layer.
  for (const auto &point : support_points) {
    const Vec3 q = result * point;
    const double expected = support->point.z() -
        z.head<2>().dot((q - center).head<2>()) / z.z();
    auto probe = scene.supportAt(q.x(), q.y(), expected, cfg.continuity);
    require(bool(probe), "terrain stop: unsupported footprint or step");
    require(probe->normal.z() >= std::cos(cfg.max_slope) - 1e-12,
            "terrain stop: maximum slope exceeded");
  }
  return result;
}
} // namespace litesim3d
