#include "litesim3d/sim.hpp"

namespace litesim3d {
void validateCollisionBudget(const Config &cfg, const Model &model, const Scene &scene, size_t robots) {
  if (!cfg.collision_budget_enabled) return;
  // Fixed scene topology lets us reject the conservative pair bound before a
  // step ever starts. No collision tests or triangles are dropped at runtime.
  const size_t robot_shapes = model.collision.size();
  require(robot_shapes <= 64, "collision budget: robot exceeds 64 shapes");
  require(scene.collision.size() + robots * robot_shapes <= 1024,
          "collision budget: world exceeds 1024 shapes");
  const size_t pairs = robots * robot_shapes * scene.collision.size() +
      robots * (robots - 1) / 2 * robot_shapes * robot_shapes;
  require(pairs <= 20000, "collision budget: potential pairs exceed 20000 per collision query");
  size_t triangles = 0, dynamic = 0, memory = 0;
  std::set<const Shape *> unique;
  std::map<std::string, size_t> moving_shapes;
  auto add = [&](const Geometry &g, size_t instances, bool moving) {
    const auto n = g.shape->triangles.size();
    require(n <= 10000, "collision budget: mesh exceeds 10000 triangles: " + g.entity);
    triangles += n * instances;
    if (moving) dynamic += n * instances;
    if (unique.insert(g.shape.get()).second) {
      // Conservative accounting includes vertices, triangle storage, BVH nodes
      // and FCL allocation overhead. This is not a bound on total process RSS.
      memory += n * 1024 + sizeof(Shape);
    }
  };
  for (const auto &g : model.collision) add(g, robots, true);
  for (const auto &g : scene.collision) {
    add(g, 1, bool(g.motion));
    if (g.motion) require(++moving_shapes[g.entity] <= 64,
                          "collision budget: dynamic model exceeds 64 shapes");
  }
  require(triangles <= 200000, "collision budget: total instance triangles exceed 200000");
  require(dynamic <= 20000, "collision budget: dynamic triangles exceed 20000");
  require(memory <= 256 * 1024 * 1024, "collision budget: estimated collision memory exceeds 256 MiB");
}
} // namespace litesim3d
