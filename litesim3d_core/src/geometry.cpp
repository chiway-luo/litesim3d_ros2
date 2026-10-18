#include "litesim3d/sim.hpp"
#include <algorithm>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <numeric>
#include <stdexcept>

namespace litesim3d {
void require(bool ok, const std::string &message) {
  if (!ok)
    throw std::runtime_error(message);
}
Pose pose(const Vec3 &xyz, const Vec3 &rpy) {
  require(xyz.allFinite() && rpy.allFinite(), "pose: non-finite xyz/rpy");
  Pose p = Pose::Identity();
  p.linear() =
      (Eigen::AngleAxisd(rpy.z(), Vec3::UnitZ()) * Eigen::AngleAxisd(rpy.y(), Vec3::UnitY()) *
       Eigen::AngleAxisd(rpy.x(), Vec3::UnitX()))
          .toRotationMatrix();
  p.translation() = xyz;
  return p;
}
Pose quaternionPose(const Vec3 &xyz, Eigen::Quaterniond q) {
  require(xyz.allFinite() && q.coeffs().allFinite() && q.norm() > 1e-12,
          "pose: invalid quaternion");
  Pose p = Pose::Identity();
  p.linear() = q.normalized().toRotationMatrix();
  p.translation() = xyz;
  return p;
}
double yaw(const Pose &p) { return std::atan2(p.linear()(1, 0), p.linear()(0, 0)); }
void Bounds::add(const Vec3 &p) {
  lo = lo.cwiseMin(p);
  hi = hi.cwiseMax(p);
}
void Bounds::add(const Bounds &b) {
  add(b.lo);
  add(b.hi);
}
bool Bounds::ray(const Vec3 &o, const Vec3 &d, double limit) const {
  double near = 0, far = limit;
  for (int i = 0; i < 3; ++i) {
    if (std::abs(d[i]) < 1e-15) {
      if (o[i] < lo[i] || o[i] > hi[i])
        return false;
    } else {
      double a = (lo[i] - o[i]) / d[i], b = (hi[i] - o[i]) / d[i];
      if (a > b)
        std::swap(a, b);
      near = std::max(a, near);
      far = std::min(b, far);
      if (near > far)
        return false;
    }
  }
  return far >= 0;
}
void Bvh::build(const std::vector<Bounds> &boxes) {
  nodes_.clear();
  order_.resize(boxes.size());
  std::iota(order_.begin(), order_.end(), 0);
  if (!boxes.empty())
    partition(0, boxes.size(), boxes);
}
int Bvh::partition(size_t begin, size_t end, const std::vector<Bounds> &boxes) {
  const int index = static_cast<int>(nodes_.size());
  nodes_.emplace_back();
  Bounds b;
  for (size_t i = begin; i < end; ++i)
    b.add(boxes[order_[i]]);
  nodes_[index].bounds = b;
  nodes_[index].begin = begin;
  nodes_[index].end = end;
  if (end - begin > 4) {
    Eigen::Index axis;
    (b.hi - b.lo).maxCoeff(&axis);
    const size_t mid = begin + (end - begin) / 2;
    std::stable_sort(order_.begin() + begin, order_.begin() + end, [&](size_t a, size_t c) {
      return boxes[a].lo[axis] + boxes[a].hi[axis] < boxes[c].lo[axis] + boxes[c].hi[axis];
    });
    const int left = partition(begin, mid, boxes), right = partition(mid, end, boxes);
    nodes_[index].left = left;
    nodes_[index].right = right;
  }
  return index;
}
void Bvh::query(const Vec3 &o, const Vec3 &d, double limit,
                const std::function<void(size_t)> &fn) const {
  if (nodes_.empty())
    return;
  std::vector<int> stack{0};
  while (!stack.empty()) {
    const auto &n = nodes_[stack.back()];
    stack.pop_back();
    if (!n.bounds.ray(o, d, limit))
      continue;
    if (n.left < 0)
      for (size_t j = n.begin; j < n.end; ++j)
        fn(order_[j]);
    else {
      stack.push_back(n.right);
      stack.push_back(n.left);
    }
  }
}
std::shared_ptr<Shape> Shape::box(const Vec3 &size) {
  require(size.allFinite() && size.minCoeff() > 0, "box: size must be positive and finite");
  auto s = std::make_shared<Shape>();
  s->kind = Kind::Box;
  s->size = size;
  s->bounds.add(-size / 2);
  s->bounds.add(size / 2);
  s->collision = std::make_shared<fcl::Boxd>(size);
  return s;
}
std::shared_ptr<Shape> Shape::sphere(double radius) {
  require(std::isfinite(radius) && radius > 0, "sphere: invalid radius");
  auto s = std::make_shared<Shape>();
  s->kind = Kind::Sphere;
  s->size = Vec3::Constant(radius);
  s->bounds.add(-s->size);
  s->bounds.add(s->size);
  s->collision = std::make_shared<fcl::Sphered>(radius);
  return s;
}
std::shared_ptr<Shape> Shape::cylinder(double radius, double length) {
  require(std::isfinite(radius) && radius > 0 && std::isfinite(length) && length > 0,
          "cylinder: invalid radius/length");
  auto s = std::make_shared<Shape>();
  s->kind = Kind::Cylinder;
  s->size = Vec3(radius, radius, length);
  s->bounds.add(Vec3(-radius, -radius, -length / 2));
  s->bounds.add(Vec3(radius, radius, length / 2));
  s->collision = std::make_shared<fcl::Cylinderd>(radius, length);
  return s;
}
void LoadControl::checkpoint(size_t additional_bytes, size_t additional_triangles) {
  require(!cancelled || !cancelled(), "resource load cancelled");
  require(bytes <= byte_budget && additional_bytes <= byte_budget - bytes,
          "resource byte budget exceeded before allocation");
  require(triangles <= triangle_budget && additional_triangles <= triangle_budget - triangles,
          "resource triangle budget exceeded before allocation");
  if (fail_allocation_after && checkpoints >= *fail_allocation_after) throw std::bad_alloc();
  ++checkpoints; bytes += additional_bytes; triangles += additional_triangles;
}
std::shared_ptr<Shape> Shape::mesh(const std::filesystem::path &path, const Vec3 &scale,
                                  const std::shared_ptr<LoadControl> &provided) {
  require(scale.allFinite() && scale.minCoeff() > 0,
          path.string() + ": mesh scale must be positive and finite");
  auto ext = path.extension().string();
  std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
  require(ext == ".stl" || ext == ".obj" || ext == ".dae",
          path.string() + ": supported mesh formats: STL OBJ DAE");
  require(std::filesystem::is_regular_file(path) &&
              std::filesystem::file_size(path) <= 64 * 1024 * 1024,
          path.string() + ": missing mesh or resource exceeds 64 MiB");
  auto control = provided ? provided : std::make_shared<LoadControl>();
  control->checkpoint();
  std::ostringstream cache;
  cache << std::filesystem::canonical(path).string() << std::setprecision(17)
        << ':' << scale.x() << ':' << scale.y() << ':' << scale.z();
  const std::string cache_key = cache.str();
  if (auto cached = control->meshes[cache_key].lock()) return cached;
  control->checkpoint(std::filesystem::file_size(path));
  // For binary STL the declared face count can be checked before the importer allocates vertices.
  if (ext == ".stl") {
    std::ifstream input(path, std::ios::binary);
    std::array<unsigned char,84> header{};
    input.read(reinterpret_cast<char *>(header.data()), header.size());
    if (input.gcount() == 84) {
      uint64_t count = 0;
      for (int b = 0; b < 4; ++b) count |= uint64_t(header[80+b]) << (8*b);
      if (84 + count * 50 == std::filesystem::file_size(path))
        require(count <= control->triangle_budget - control->triangles,
                "STL triangle budget exceeded before import");
    }
  }
  Assimp::Importer importer;
  const aiScene *input =
      importer.ReadFile(path.string(), aiProcess_Triangulate | aiProcess_PreTransformVertices |
                                           aiProcess_ValidateDataStructure);
  require(input && input->HasMeshes(),
          path.string() + ": invalid mesh: " + importer.GetErrorString());
  auto s = std::make_shared<Shape>();
  s->kind = Kind::Mesh;
  s->resource = path.string();
  std::vector<Bounds> boxes;
  auto collision = std::make_shared<fcl::BVHModel<fcl::OBBRSSd>>();
  collision->beginModel();
  for (unsigned m = 0; m < input->mNumMeshes; ++m) {
    const auto *mesh = input->mMeshes[m];
    require(mesh->mNumFaces <= 1000000 && s->triangles.size() + mesh->mNumFaces <= 1000000,
            path.string() + ": triangle budget exceeded");
    control->checkpoint(size_t(mesh->mNumFaces) * (sizeof(Triangle) + sizeof(Bounds)), mesh->mNumFaces);
    s->triangles.reserve(s->triangles.size() + mesh->mNumFaces);
    boxes.reserve(boxes.size() + mesh->mNumFaces);
    for (unsigned f = 0; f < mesh->mNumFaces; ++f) {
      if ((f & 1023) == 0) control->checkpoint();
      const auto &face = mesh->mFaces[f];
      require(face.mNumIndices == 3, path.string() + ": non-triangle face");
      Vec3 v[3];
      for (int k = 0; k < 3; ++k) {
        require(face.mIndices[k] < mesh->mNumVertices, path.string() + ": invalid vertex index");
        const auto &a = mesh->mVertices[face.mIndices[k]];
        v[k] = Vec3(a.x, a.y, a.z).cwiseProduct(scale);
        require(v[k].allFinite(), path.string() + ": non-finite vertex");
      }
      require((v[1] - v[0]).cross(v[2] - v[0]).norm() > 1e-12,
              path.string() + ": degenerate triangle");
      s->triangles.push_back({v[0], v[1], v[2]});
      Bounds b;
      for (auto &p : v)
        b.add(p);
      boxes.push_back(b);
      s->bounds.add(b);
      collision->addTriangle(v[0], v[1], v[2]);
    }
  }
  require(!s->triangles.empty(), path.string() + ": empty mesh");
  control->checkpoint();
  collision->endModel();
  s->collision = collision;
  s->bvh.build(boxes);
  control->meshes[cache_key] = s;
  return s;
}
double Shape::ray(const Vec3 &o, const Vec3 &d, double limit) const {
  double hit = INFINITY;
  auto accept = [&](double t) {
    if (t >= -1e-12 && t <= limit + 1e-12 * std::max(1.0, limit))
      hit = std::min(hit, std::max(0.0, t));
  };
  if (kind == Kind::Box) {
    double near = -INFINITY, far = INFINITY;
    for (int i = 0; i < 3; ++i) {
      if (std::abs(d[i]) < 1e-15) {
        if (o[i] < bounds.lo[i] || o[i] > bounds.hi[i])
          return hit;
      } else {
        double a = (bounds.lo[i] - o[i]) / d[i], b = (bounds.hi[i] - o[i]) / d[i];
        if (a > b)
          std::swap(a, b);
        near = std::max(near, a);
        far = std::min(far, b);
      }
    }
    if (near <= far) {
      accept(near);
      accept(far);
    }
  } else if (kind == Kind::Sphere) {
    const double b = o.dot(d), c = o.squaredNorm() - size.x() * size.x(), disc = b * b - c;
    if (disc >= 0) {
      accept(-b - std::sqrt(disc));
      accept(-b + std::sqrt(disc));
    }
  } else if (kind == Kind::Cylinder) {
    const double a = d.x() * d.x() + d.y() * d.y(), b = o.x() * d.x() + o.y() * d.y();
    const double c = o.x() * o.x() + o.y() * o.y() - size.x() * size.x(), disc = b * b - a * c;
    if (a > 1e-20 && disc >= 0)
      for (double t : {(-b - std::sqrt(disc)) / a, (-b + std::sqrt(disc)) / a})
        if (std::abs(o.z() + t * d.z()) <= size.z() / 2 + 1e-12)
          accept(t);
    if (std::abs(d.z()) > 1e-15)
      for (double z : {-size.z() / 2, size.z() / 2}) {
        double t = (z - o.z()) / d.z();
        Vec3 p = o + t * d;
        if (p.x() * p.x() + p.y() * p.y() <= size.x() * size.x() + 1e-12)
          accept(t);
      }
  } else {
    bvh.query(o, d, limit, [&](size_t i) {
      const auto &t = triangles[i];
      const Vec3 e1 = t.b - t.a, e2 = t.c - t.a, p = d.cross(e2);
      const double det = e1.dot(p);
      if (std::abs(det) < 1e-14)
        return;
      const Vec3 s = o - t.a;
      const double u = s.dot(p) / det;
      if (u < -1e-12 || u > 1 + 1e-12)
        return;
      const Vec3 q = s.cross(e1);
      const double v = d.dot(q) / det;
      if (v < -1e-12 || u + v > 1 + 1e-12)
        return;
      accept(e2.dot(q) / det);
    });
  }
  return hit;
}
Bounds Geometry::worldBounds() const {
  if (shape->kind == Shape::Kind::Sphere) {
    Bounds b;
    const Vec3 r = Vec3::Constant(shape->size.x());
    b.add(transform.translation() - r);
    b.add(transform.translation() + r);
    return b;
  }
  if (shape->kind == Shape::Kind::Cylinder) {
    Bounds b;
    Vec3 extent;
    const Vec3 axis = transform.linear().col(2);
    for (int i = 0; i < 3; ++i)
      extent[i] = shape->size.z() / 2 * std::abs(axis[i]) +
                  shape->size.x() * std::sqrt(std::max(0.0, 1 - axis[i] * axis[i]));
    b.add(transform.translation() - extent);
    b.add(transform.translation() + extent);
    return b;
  }
  Bounds b;
  for (int i = 0; i < 8; ++i) {
    b.add(transform * Vec3(i & 1 ? shape->bounds.hi.x() : shape->bounds.lo.x(),
                           i & 2 ? shape->bounds.hi.y() : shape->bounds.lo.y(),
                           i & 4 ? shape->bounds.hi.z() : shape->bounds.lo.z()));
  }
  return b;
}
void Scene::rebuild() {
  tilted_support = false;
  auto tilt = [&](const Vec3 &normal) {
    if (normal.z() > 1e-12 && normal.head<2>().norm() > 1e-12 * normal.norm())
      tilted_support = true;
  };
  for (const auto &g : collision) if (g.support) {
    if (g.shape->kind == Shape::Kind::Box) tilt(g.transform.linear().col(2));
    if (g.shape->kind == Shape::Kind::Mesh)
      for (const auto &t : g.shape->triangles)
        tilt(g.transform.linear() * (t.b - t.a).cross(t.c - t.a));
  }
  for (auto &g : collision) if (g.support && g.shape->kind == Shape::Kind::Mesh) {
    // Keep risers, walls and undersides when permitting a wheel's contact with an upper support face.
    auto barriers = std::make_shared<fcl::BVHModel<fcl::OBBRSSd>>();
    barriers->beginModel();
    size_t count = 0;
    for (const auto &t : g.shape->triangles)
      if ((g.transform.linear() * (t.b - t.a).cross(t.c - t.a)).z() <= 1e-12) {
        barriers->addTriangle(t.a, t.b, t.c); ++count;
      }
    g.support_barriers.reset();
    if (count) { barriers->endModel(); g.support_barriers = barriers; }
  }
  std::vector<Bounds> boxes;
  for (auto &g : sensor)
    boxes.push_back(g.worldBounds());
  index.build(boxes);
  ++build_count;
}
std::optional<Hit> Scene::ray(const Vec3 &o, const Vec3 &direction, double max_range,
                              const std::vector<Geometry> &robot,
                              const std::set<std::string> &excluded, uint64_t stamp) const {
  require(o.allFinite() && direction.allFinite() && direction.norm() > 1e-12 &&
              std::isfinite(max_range) && max_range > 0,
          "ray: invalid origin/direction/max_range");
  const Vec3 d = direction.normalized();
  double nearest = max_range;
  std::optional<Hit> result;
  auto test = [&](const Geometry &g) {
    Pose inverse = g.at(stamp).inverse();
    double distance = g.shape->ray(inverse * o, inverse.linear() * d, nearest);
    if (std::isfinite(distance) && (!result || distance < nearest)) {
      nearest = distance;
      result = Hit{distance, g.entity, g.link, g.reflectivity};
    }
  };
  index.query(o, d, max_range, [&](size_t i) { if (!sensor[i].motion) test(sensor[i]); });
  for (const auto &g : sensor) if (g.motion) test(g);
  for (const auto &g : robot)
    if (!excluded.count(g.link))
      test(g);
  return result;
}
std::shared_ptr<const LidarPatternTable> LidarPatternTable::load(
    const std::filesystem::path &file, const std::shared_ptr<LoadControl> &provided) {
  require(std::filesystem::is_regular_file(file) && std::filesystem::file_size(file) <= 64 * 1024 * 1024,
          "Mid-360 pattern file missing or exceeds 64 MiB: " + file.string());
  auto control = provided ? provided : std::make_shared<LoadControl>();
  control->checkpoint(std::filesystem::file_size(file));
  auto table = std::make_shared<LidarPatternTable>();
  table->source = std::filesystem::canonical(file).string();
  table->source_sha256 = sha256(file);
  std::ifstream input(file);
  std::string line;
  std::getline(input, line);
  if (!line.empty() && line.back() == '\r') line.pop_back();
  require(line == "Time/s,Azimuth/deg,Zenith/deg", "Mid-360 pattern: unexpected CSV header");
  while (std::getline(input, line)) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    require(!line.empty() && line.size() <= 128 && table->samples.size() < 2000000,
            "Mid-360 pattern: invalid row or sample budget exceeded");
    const char *begin = line.c_str(); char *end = nullptr;
    const auto ordinal = std::strtoull(begin, &end, 10);
    require(end != begin && *end == ',' && ordinal == table->samples.size() + 1,
            "Mid-360 pattern: sample ordinals must be consecutive from 1");
    begin = end + 1; const double az_deg = std::strtod(begin, &end);
    require(end != begin && *end == ',' && std::isfinite(az_deg) && az_deg >= 0 && az_deg <= 360,
            "Mid-360 pattern: invalid azimuth");
    begin = end + 1; const double zen_deg = std::strtod(begin, &end);
    require(end != begin && *end == '\0' && std::isfinite(zen_deg) && zen_deg >= 0 && zen_deg <= 180,
            "Mid-360 pattern: invalid zenith");
    if (table->samples.size() == table->samples.capacity()) {
      const size_t capacity = std::min<size_t>(2000000, table->samples.capacity() + 16384);
      control->checkpoint((capacity - table->samples.capacity()) * sizeof(Sample));
      table->samples.reserve(capacity);
    }
    // Upstream applies Rz(azimuth) Ry(zenith - pi/2) to +X: elevation = pi/2 - zenith.
    const double az = az_deg * pi / 180, el = (90 - zen_deg) * pi / 180;
    table->samples.push_back({Vec3(std::cos(el) * std::cos(az), std::cos(el) * std::sin(az), std::sin(el)), az, el});
  }
  require(input.eof() && !table->samples.empty(), "Mid-360 pattern: read failure or empty table");
  return table;
}
void LidarConfig::validate() const {
  require(!name.empty() && !link.empty() && !topic.empty(), "lidar: empty name/link/topic");
  require(!vertical_angles.empty() && vertical_angles.size() <= 65536, "lidar: invalid rings");
  require(columns > 0 && columns <= 65536 && uint64_t(columns) * vertical_angles.size() <= 4194304,
          "lidar: point budget exceeded");
  for (double angle : vertical_angles)
    require(std::isfinite(angle) && std::abs(angle) <= pi / 2, "lidar: invalid vertical_angles");
  require(std::isfinite(rate) && rate > 0 && rate <= 1000, "lidar: invalid rate_hz");
  require(std::isfinite(min_range) && std::isfinite(max_range) && min_range >= 0 &&
              max_range > min_range,
          "lidar: invalid range");
  require(std::isfinite(horizontal_start) && std::isfinite(horizontal_end) &&
              horizontal_end > horizontal_start &&
              horizontal_end - horizontal_start <= 2 * pi + 1e-12,
          "lidar: invalid horizontal_fov");
  require(scan_mode == "snapshot" || scan_mode == "rolling", "lidar: invalid scan_mode");
  require(scan_pattern == "spinning" || scan_pattern == "spherical" || scan_pattern == "rosette" ||
              scan_pattern == "livox_mid360",
          "lidar: invalid scan_pattern");
  if (scan_pattern == "livox_mid360") {
    require(pattern_table && !pattern_table->samples.empty(), "Mid-360 requires a loaded pattern_file");
    require(std::abs(rate * columns - 200000) <= 1e-6,
            "Mid-360 requires columns * rate_hz = 200000; change frame packing, not sample rate");
  } else require(!pattern_table, "pattern_file only applies to livox_mid360");
  require(std::isfinite(vertical_min) && std::isfinite(vertical_max) &&
              vertical_min >= -pi / 2 && vertical_max <= pi / 2 && vertical_max > vertical_min,
          "lidar: invalid vertical_fov");
  require(std::isfinite(cone_half_angle) && cone_half_angle > 0 && cone_half_angle <= pi / 2 &&
              std::isfinite(pattern_hz) && pattern_hz > 0 && pattern_hz <= 10000,
          "lidar: invalid cone/pattern frequency");
  if (scan_pattern != "spinning") {
    require(vertical_angles.size() == 1 && vertical_angles.front() == 0 && ring_delays.empty(),
            "non-spinning lidar requires one synthetic ring at zero and no ring delays");
    require(scan_pattern == "livox_mid360" || rate * columns >= 4 * pattern_hz * std::sqrt(2.0),
            "lidar: point rate too low for non-repeating pattern");
  }
  if (scan_pattern == "rosette")
    require(std::abs(horizontal_start + pi) < 1e-12 && std::abs(horizontal_end - pi) < 1e-12 &&
                vertical_min == -pi / 2 && vertical_max == pi / 2,
            "rosette uses cone_half_angle_deg; rotate its optical axis in URDF");
  require(std::isfinite(range_bias) && std::isfinite(range_sigma) && range_sigma >= 0 &&
              std::isfinite(dropout) && dropout >= 0 && dropout <= 1,
          "lidar: invalid noise/dropout");
  require(ring_delays.empty() || ring_delays.size() == vertical_angles.size(),
          "lidar: ring_delays must match rings");
  for (size_t r = 0; r < vertical_angles.size(); ++r) {
    const double delay = ring_delays.empty() ? 0 : ring_delays[r];
    require(std::isfinite(delay) && delay >= 0 && delay < 1 / (rate * columns),
            "lidar: ring delay must lie within one column");
    require(scan_mode != "rolling" || offsetNs(r, columns - 1) < uint64_t(std::llround(1e9 / rate)),
            "lidar: sample exceeds scan period");
    require(scan_mode != "snapshot" || delay == 0, "snapshot cannot have ring delays");
  }
  require(filter == "sensor_link" || filter == "robot" || filter == "links",
          "lidar: invalid self_filter");
}
Vec3 LidarConfig::direction(size_t ring, size_t column, uint64_t frame) const {
  if (scan_pattern == "livox_mid360") {
    const uint64_t count = pattern_table->samples.size();
    const auto &sample = pattern_table->samples[((frame % count) * columns + column) % count];
    double relative_az = std::fmod(sample.azimuth - horizontal_start, 2 * pi);
    if (relative_az < 0) relative_az += 2 * pi;
    // Crop the physical scan. Never stretch, clamp or repopulate the masked region.
    if (relative_az >= horizontal_end - horizontal_start || sample.elevation < vertical_min ||
        sample.elevation > vertical_max) return Vec3::Constant(NAN);
    return sample.direction;
  }
  double az = horizontal_start +
              (horizontal_end - horizontal_start) * double(column) / columns;
  double el = vertical_angles.at(ring);
  if (scan_pattern != "spinning") {
    // Continuous deterministic pattern across frame boundaries. These generic
    // mirror trajectories approximate coverage, not proprietary firing tables.
    const long double time = (static_cast<long double>(frame) +
                              static_cast<long double>(column) / columns) / rate;
    const double a = double(std::fmod(time * pattern_hz, 1.0L)) * 2 * pi;
    const double b = double(std::fmod(time * pattern_hz * std::sqrt(2.0L), 1.0L)) * 2 * pi;
    if (scan_pattern == "spherical") {
      az = horizontal_start + (horizontal_end - horizontal_start) * a / (2 * pi);
      el = (vertical_min + vertical_max) / 2 +
           (vertical_max - vertical_min) / 2 * std::sin(b);
    } else {
      const double y = (std::cos(a) + std::cos(b)) / 2;
      const double z = (std::sin(a) - std::sin(b)) / 2;
      const double radius = std::hypot(y, z);
      const double theta = cone_half_angle * std::min(1.0, radius);
      const double scale = radius > 1e-15 ? std::sin(theta) / radius : 0;
      return Vec3(std::cos(theta), scale * y, scale * z);
    }
  }
  return Vec3(std::cos(el) * std::cos(az), std::cos(el) * std::sin(az), std::sin(el));
}
Cloud scan(const Scene &scene, const LidarConfig &cfg, const Pose &world, uint64_t stamp,
           const std::vector<Geometry> &robot) {
  auto local = robot;
  for (auto &g : local) g.transform = world.inverse() * g.transform;
  return scanTrajectory(scene, cfg, stamp, [world](uint64_t) { return world; }, local);
}
} // namespace litesim3d
