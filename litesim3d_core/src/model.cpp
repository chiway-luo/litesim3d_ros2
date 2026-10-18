#include "litesim3d/sim.hpp"
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <openssl/evp.h>
#include <sstream>
#include <tinyxml2.h>
#include <urdf_parser/urdf_parser.h>

namespace litesim3d {
std::string readText(const std::filesystem::path &path) {
  require(std::filesystem::is_regular_file(path), path.string() + ": file not found");
  require(std::filesystem::file_size(path) <= 64 * 1024 * 1024,
          path.string() + ": file exceeds 64 MiB budget");
  std::ifstream in(path, std::ios::binary);
  require(bool(in), path.string() + ": cannot read");
  std::ostringstream contents;
  contents << in.rdbuf();
  require(!in.bad(), path.string() + ": read failed");
  return contents.str();
}
std::string sha256Text(const std::string &content) {
  unsigned char digest[EVP_MAX_MD_SIZE];
  unsigned int length = 0;
  require(EVP_Digest(content.data(), content.size(), digest, &length, EVP_sha256(), nullptr) == 1,
          "SHA256 failed");
  std::ostringstream out;
  out << std::hex << std::setfill('0');
  for (unsigned i = 0; i < length; ++i)
    out << std::setw(2) << int(digest[i]);
  return out.str();
}
std::string sha256(const std::filesystem::path &path) { return sha256Text(readText(path)); }
std::filesystem::path resolveResource(const std::string &uri, const std::filesystem::path &base) {
  std::filesystem::path p;
  if (uri.rfind("package://", 0) == 0) {
    const std::string tail = uri.substr(10);
    auto slash = tail.find('/');
    require(slash != std::string::npos && slash > 0, "invalid package URI: " + uri);
    const std::string package = tail.substr(0, slash);
    const std::filesystem::path relative = tail.substr(slash + 1);
    require(!relative.is_absolute(), "invalid package URI: " + uri);
    for (const auto &part : relative)
      require(part != "..", "package traversal rejected: " + uri);
    std::vector<std::string> prefixes;
    for (const char *var : {"AMENT_PREFIX_PATH", "LWS_RESOURCE_PREFIX"})
      if (const char *value = std::getenv(var)) {
        std::stringstream paths(value);
        std::string part;
        while (std::getline(paths, part, ':'))
          if (!part.empty())
            prefixes.push_back(part);
      }
    for (auto &prefix : prefixes) {
      auto candidate = std::filesystem::path(prefix) / "share" / package / relative;
      if (std::filesystem::is_regular_file(candidate)) {
        p = candidate;
        break;
      }
    }
    require(!p.empty(), "package resource not found: " + uri);
  } else {
    p = uri.rfind("file://", 0) == 0 ? uri.substr(7) : uri;
    if (p.is_relative())
      p = base / p;
  }
  require(std::filesystem::is_regular_file(p), "resource not found: " + p.string());
  return std::filesystem::canonical(p);
}
uint64_t streamSeed(uint64_t seed, const std::string &key) {
  uint64_t h = 14695981039346656037ULL;
  for (unsigned char c : key) {
    h ^= c;
    h *= 1099511628211ULL;
  }
  h += seed + 0x9e3779b97f4a7c15ULL;
  h = (h ^ (h >> 30)) * 0xbf58476d1ce4e5b9ULL;
  h = (h ^ (h >> 27)) * 0x94d049bb133111ebULL;
  return h ^ (h >> 31);
}
namespace {
Pose convert(const urdf::Pose &p) {
  return quaternionPose(Vec3(p.position.x, p.position.y, p.position.z),
                        Eigen::Quaterniond(p.rotation.w, p.rotation.x, p.rotation.y, p.rotation.z));
}
std::shared_ptr<Shape> geometry(const urdf::GeometrySharedPtr &g, const std::filesystem::path &base,
                                Json &report, const std::shared_ptr<LoadControl> &control, bool collision_role) {
  require(bool(g), "missing geometry");
  if (g->type == urdf::Geometry::BOX) {
    auto b = std::dynamic_pointer_cast<urdf::Box>(g);
    return Shape::box(Vec3(b->dim.x, b->dim.y, b->dim.z));
  }
  if (g->type == urdf::Geometry::SPHERE)
    return Shape::sphere(std::dynamic_pointer_cast<urdf::Sphere>(g)->radius);
  if (g->type == urdf::Geometry::CYLINDER) {
    auto c = std::dynamic_pointer_cast<urdf::Cylinder>(g);
    return Shape::cylinder(c->radius, c->length);
  }
  require(g->type == urdf::Geometry::MESH, "unsupported URDF geometry");
  auto m = std::dynamic_pointer_cast<urdf::Mesh>(g);
  auto path = resolveResource(m->filename, base);
  Vec3 scale(m->scale.x, m->scale.y, m->scale.z);
  auto s = Shape::mesh(path, scale, control, collision_role);
  report["resources"][path.string()] = {
      {"sha256", sha256(path)},
      {"scale", {scale.x(), scale.y(), scale.z()}},
      {"unit", "meter after file unit conversion and URDF scale"}};
  return s;
}
} // namespace
Model Model::load(const std::filesystem::path &file, const ModelOptions &options) {
  auto control = options.load_control ? options.load_control : std::make_shared<LoadControl>();
  control->checkpoint(std::filesystem::file_size(file));
  Model result;
  result.xml = readText(file);
  tinyxml2::XMLDocument doc;
  const auto xml_status = doc.Parse(result.xml.c_str());
  require(xml_status == tinyxml2::XML_SUCCESS,
          file.string() + ": invalid XML: " + (doc.ErrorStr() ? doc.ErrorStr() : ""));
  auto *robot = doc.FirstChildElement("robot");
  require(robot, file.string() + ": missing robot element");
  if (auto *version = robot->Attribute("version"))
    require(std::string(version) == "1.0", file.string() + ": only URDF 1.0 is supported");
  std::set<std::string> link_names, joint_names;
  Json extensions = Json::array();
  for (auto *el = robot->FirstChildElement(); el; el = el->NextSiblingElement()) {
    const std::string tag = el->Name();
    if (tag == "link" || tag == "joint") {
      auto *name = el->Attribute("name");
      require(name && *name, file.string() + ": " + tag + " without name");
      auto &set = tag == "link" ? link_names : joint_names;
      require(set.insert(name).second, file.string() + ": duplicate " + tag + " " + name);
      if (tag == "joint") {
        auto *type = el->Attribute("type");
        const std::string t = type ? type : "";
        require(t == "fixed" || t == "revolute" || t == "continuous" || t == "prismatic",
                file.string() + ": unsupported joint " + name + " type " + t);
      }
    } else if (tag != "material")
      extensions.push_back(tag);
  }
  // Prevalidate graph before urdfdom traverses malformed cycles.
  std::map<std::string, std::string> parent;
  for (auto *el = robot->FirstChildElement("joint"); el; el = el->NextSiblingElement("joint")) {
    auto *p = el->FirstChildElement("parent");
    auto *c = el->FirstChildElement("child");
    require(p && c && p->Attribute("link") && c->Attribute("link"),
            file.string() + ": joint " + el->Attribute("name") + " missing parent/child");
    const std::string pn = p->Attribute("link"), cn = c->Attribute("link");
    require(link_names.count(pn) && link_names.count(cn),
            file.string() + ": joint " + el->Attribute("name") + " references unknown link");
    require(!parent.count(cn), file.string() + ": multiple parents for " + cn);
    parent[cn] = pn;
  }
  size_t roots = 0;
  require(link_names.size() <= 4096 && joint_names.size() <= 4095,
          file.string() + ": model exceeds link/joint budget");
  for (const auto &link : link_names) {
    if (!parent.count(link))
      ++roots;
    std::set<std::string> seen;
    std::string cur = link;
    while (parent.count(cur)) {
      require(seen.size() < 128, file.string() + ": joint depth exceeds 128");
      require(seen.insert(cur).second, file.string() + ": link cycle at " + cur);
      cur = parent.at(cur);
    }
  }
  require(roots == 1, file.string() + ": model must have exactly one root");
  auto parsed = urdf::parseURDF(result.xml);
  require(bool(parsed), file.string() + ": urdfdom rejected model (see parser diagnostics)");
  result.root = parsed->getRoot()->name;
  result.links = link_names;
  result.report = {{"source", std::filesystem::absolute(file).string()},
                   {"sha256", sha256(file)},
                   {"root_link", result.root},
                   {"joint_count", parsed->joints_.size()},
                   {"ignored_extensions", extensions},
                   {"resources", Json::object()},
                   {"collision_fallback", Json::array()},
                   {"inertial", Json::object()},
                   {"unit", "meter"},
                   {"undriven_joints", Json::array()}};
  for (auto &[name, link] : parsed->links_) {
    try {
      std::vector<Geometry> visual, collision;
      for (auto &v : link->visual_array)
        visual.push_back({"robot", name, geometry(v->geometry, file.parent_path(), result.report, control, false),
                          convert(v->origin)});
      for (auto &c : link->collision_array)
        collision.push_back({"robot", name,
                             geometry(c->geometry, file.parent_path(), result.report, control, true),
                             convert(c->origin)});
      if (collision.empty() && !visual.empty() && !options.allow_visual_only) {
        require(options.visual_collision_fallback,
                "missing collision; explicit visual fallback required");
        collision = visual;
        for (auto &g : collision) g.shape->ensureCollision();
        result.report["collision_fallback"].push_back(
            {{"link", name}, {"note", "visual tessellation can increase collision cost"}});
      }
      auto choice = options.sensor_overrides.count(name) ? options.sensor_overrides.at(name)
                                                         : options.sensor_geometry;
      require(choice == "visual" || choice == "collision",
              "sensor_geometry must be visual or collision");
      require(choice != "visual" || !visual.empty() || collision.empty(),
              "missing visual for sensor; choose collision explicitly");
      const auto &sensor = choice == "visual" ? visual : collision;
      result.visual.insert(result.visual.end(), visual.begin(), visual.end());
      result.collision.insert(result.collision.end(), collision.begin(), collision.end());
      result.sensor.insert(result.sensor.end(), sensor.begin(), sensor.end());
      if (link->inertial) {
        auto &i = *link->inertial;
        require(std::isfinite(i.mass) && i.mass > 0, "invalid inertial mass");
        result.report["inertial"][name] = {{"mass", i.mass},
                                           {"tensor", {i.ixx, i.ixy, i.ixz, i.iyy, i.iyz, i.izz}},
                                           {"dynamics_enabled", false}};
      }
    } catch (const std::exception &e) {
      throw std::runtime_error(file.string() + ": link " + name + ": " + e.what());
    }
  }
  for (auto &[name, j] : parsed->joints_) {
    Joint joint;
    joint.name = name;
    joint.parent = j->parent_link_name;
    joint.child = j->child_link_name;
    joint.origin = convert(j->parent_to_joint_origin_transform);
    joint.type = j->type == urdf::Joint::FIXED        ? "fixed"
                 : j->type == urdf::Joint::REVOLUTE   ? "revolute"
                 : j->type == urdf::Joint::CONTINUOUS ? "continuous"
                                                      : "prismatic";
    joint.axis = Vec3(j->axis.x, j->axis.y, j->axis.z);
    if (joint.type != "fixed") {
      require(joint.axis.allFinite() && joint.axis.norm() > 1e-12,
              file.string() + ": joint " + name + ": invalid axis");
      joint.axis.normalize();
      if (j->limits) {
        require(std::isfinite(j->limits->velocity) && j->limits->velocity > 0,
                file.string() + ": joint " + name + ": invalid velocity limit");
        joint.velocity = j->limits->velocity;
        if (joint.type != "continuous") {
          joint.lower = j->limits->lower;
          joint.upper = j->limits->upper;
          require(std::isfinite(joint.lower) && std::isfinite(joint.upper) &&
                      joint.lower <= joint.upper,
                  file.string() + ": joint " + name + ": invalid position limits");
        }
      }
      result.report["undriven_joints"].push_back(name);
    }
    if (j->mimic) {
      joint.mimic = j->mimic->joint_name;
      joint.multiplier = j->mimic->multiplier;
      joint.offset = j->mimic->offset;
      require(std::isfinite(joint.multiplier) && std::isfinite(joint.offset),
              file.string() + ": joint " + name + ": non-finite mimic");
    }
    result.joints[name] = joint;
  }
  std::set<std::string> visited, active;
  std::function<void(const std::string &)> visit = [&](const std::string &name) {
    require(result.joints.count(name), file.string() + ": mimic references unknown joint " + name);
    if (visited.count(name))
      return;
    require(active.insert(name).second, file.string() + ": mimic cycle at " + name);
    auto &j = result.joints.at(name);
    if (!j.mimic.empty())
      visit(j.mimic);
    active.erase(name);
    visited.insert(name);
    result.order.push_back(name);
  };
  for (auto &[name, j] : result.joints)
    visit(name);
  for (auto &[name, value] : options.sensor_overrides)
    require(result.links.count(name), file.string() + ": unknown sensor override link " + name);
  result.positions({});
  Bounds boundary;
  for (auto &g : result.placed(result.visual, Pose::Identity(), result.root, {}))
    boundary.add(g.worldBounds());
  if (!result.visual.empty())
    result.report["bounds"] = {{boundary.lo.x(), boundary.lo.y(), boundary.lo.z()},
                               {boundary.hi.x(), boundary.hi.y(), boundary.hi.z()}};
  return result;
}
std::map<std::string, double> Model::positions(const std::map<std::string, double> &input) const {
  for (auto &[name, value] : input)
    require(joints.count(name) && std::isfinite(value),
            "unknown joint or non-finite position: " + name);
  std::map<std::string, std::tuple<std::string, double, double>> expressions;
  std::map<std::string, std::pair<double, double>> limits;
  for (const auto &name : order) {
    const auto &j = joints.at(name);
    if (j.mimic.empty()) {
      expressions[name] = {name, 1, 0};
      limits[name] = {j.lower, j.upper};
    } else {
      auto [root, a, b] = expressions.at(j.mimic);
      a *= j.multiplier;
      b = b * j.multiplier + j.offset;
      require(std::isfinite(a) && std::isfinite(b), "mimic chain overflow: " + name);
      expressions[name] = {root, a, b};
      auto &bound = limits.at(root);
      if (std::abs(a) < 1e-15)
        require(b >= j.lower && b <= j.upper, "mimic constant violates limits: " + name);
      else {
        double lo = (j.lower - b) / a, hi = (j.upper - b) / a;
        if (lo > hi)
          std::swap(lo, hi);
        bound.first = std::max(bound.first, lo);
        bound.second = std::min(bound.second, hi);
        require(bound.first <= bound.second, "incompatible mimic limits: " + name);
      }
    }
  }
  std::map<std::string, double> out;
  for (auto &name : order) {
    const auto &j = joints.at(name);
    double q = input.count(name) ? input.at(name) : 0;
    if (!j.mimic.empty())
      q = out.at(j.mimic) * j.multiplier + j.offset;
    else
      q = std::clamp(q, limits.at(name).first, limits.at(name).second);
    out[name] = j.type == "fixed" ? 0 : q;
  }
  return out;
}
std::map<std::string, double> Model::advance(const std::map<std::string, double> &q,
                                             const std::map<std::string, double> &targets,
                                             double dt) const {
  require(std::isfinite(dt) && dt > 0, "joint update: invalid dt");
  auto out = positions(q);
  for (auto &[name, target] : targets) {
    require(joints.count(name) && std::isfinite(target), "invalid joint target: " + name);
    const auto &j = joints.at(name);
    require(j.mimic.empty(), "mimic joint cannot be driven: " + name);
    double speed = j.velocity;
    for (const auto &child : order) {
      std::string source = child;
      double multiplier = 1;
      while (!joints.at(source).mimic.empty()) {
        multiplier *= joints.at(source).multiplier;
        source = joints.at(source).mimic;
      }
      if (source == name && std::abs(multiplier) > 1e-15)
        speed = std::min(speed, joints.at(child).velocity / std::abs(multiplier));
    }
    out[name] += std::clamp(target - out[name], -speed * dt, speed * dt);
  }
  return positions(out);
}
std::map<std::string, Pose> Model::fk(const std::map<std::string, double> &q) const {
  const auto values = positions(q);
  std::map<std::string, Pose> transforms{{root, Pose::Identity()}};
  std::function<void(const std::string &)> descend = [&](const std::string &parent) {
    for (auto &[name, j] : joints)
      if (j.parent == parent) {
        Pose motion = Pose::Identity();
        if (j.type == "prismatic")
          motion.translation() = j.axis * values.at(name);
        else if (j.type != "fixed")
          motion.linear() = Eigen::AngleAxisd(values.at(name), j.axis).toRotationMatrix();
        transforms[j.child] = transforms.at(parent) * j.origin * motion;
        descend(j.child);
      }
  };
  descend(root);
  return transforms;
}
std::vector<Geometry> Model::placed(const std::vector<Geometry> &source, const Pose &world_base,
                                    const std::string &base_link,
                                    const std::map<std::string, double> &q) const {
  const auto transforms = fk(q);
  require(transforms.count(base_link), "unknown base_link: " + base_link);
  const Pose world_root = world_base * transforms.at(base_link).inverse();
  auto result = source;
  for (auto &g : result)
    g.transform = world_root * transforms.at(g.link) * g.transform;
  return result;
}
} // namespace litesim3d
