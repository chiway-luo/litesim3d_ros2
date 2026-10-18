#include "litesim3d/sim.hpp"
#include <algorithm>
#include <regex>
#include <yaml-cpp/yaml.h>

namespace litesim3d {
Pose Motion::at(uint64_t stamp) const {
  const uint64_t time = std::min(stamp, frozen_at);
  if (path)
    return path(time);
  const double t = double(time > 1 ? time - 1 : 0) * 1e-9;
  Pose result = initial;
  result.translation() += velocity * t;
  result.linear() =
      Eigen::AngleAxisd(yaw_rate * t, Vec3::UnitZ()).toRotationMatrix() *
      initial.linear();
  return result;
}
Pose Geometry::at(uint64_t stamp) const {
  return motion ? motion->at(stamp) * transform : transform;
}
namespace {
double terrainEnvelope(const Scene &scene, const TerrainConfig &cfg,
                       double radius) {
  // supportedPose may project height by continuity and change attitude at a
  // seam. Any rotation moves a point at radius r by at most 2r; flat support
  // needs only the height margin. This also covers roll/pitch, not just
  // commanded yaw.
  return cfg.enabled ? cfg.continuity + (scene.tilted_support ? 2 * radius : 0)
                     : 0;
}
double extent(const Geometry &g) {
  return g.transform.translation().norm() +
         g.shape->bounds.lo.cwiseAbs()
             .cwiseMax(g.shape->bounds.hi.cwiseAbs())
             .norm();
}
double gap(const Geometry &a, const Pose &pa, const Geometry &b,
           const Pose &pb) {
  fcl::CollisionObjectd ca(a.shape->collision, pa), cb(b.shape->collision, pb);
  fcl::CollisionRequestd req;
  fcl::CollisionResultd res;
  fcl::collide(&ca, &cb, req, res);
  if (res.isCollision())
    return 0;
  fcl::DistanceRequestd dr;
  fcl::DistanceResultd ds;
  const double d = fcl::distance(&ca, &cb, dr, ds);
  require(std::isfinite(d), "non-finite shared-world clearance");
  return std::max(0.0, d);
}
void keys(const YAML::Node &n, const std::set<std::string> &allowed) {
  require(n.IsMap(), "world: expected mapping");
  for (auto pair : n)
    require(allowed.count(pair.first.as<std::string>()),
            "unknown world field: " + pair.first.as<std::string>());
}
Vec3 vector(const YAML::Node &n) {
  require(n.IsSequence() && n.size() == 3,
          "world pose requires three coordinates");
  Vec3 v(n[0].as<double>(), n[1].as<double>(), n[2].as<double>());
  require(v.allFinite(), "non-finite world pose");
  return v;
}
} // namespace
void Simulator::stopForStep(const std::string &entity, const std::string &link,
                            double distance) {
  if (!forced_stop_) {
    contacts_.push_back({current_.stamp, link, entity, distance});
    ++contact_count_;
  }
  forced_stop_ = true;
  last_error_ = "relative swept-envelope stop";
}
std::vector<Simulator::DynamicContact> Simulator::dynamicContacts(double dt) const {
  std::vector<DynamicContact> hits;
  const auto parts = model_.placed(model_.collision, current_.world_base,
                                   config_.base_link, current_.joints);
  for (const auto &obstacle : scene_.collision) {
    if (!obstacle.motion || obstacle.motion->path ||
        obstacle.motion->frozen_at <= current_.stamp)
      continue;
    const double dynamic_speed =
        obstacle.motion->velocity.norm() +
        std::abs(obstacle.motion->yaw_rate) * extent(obstacle);
    if (dynamic_speed == 0)
      continue;
    for (const auto &part : parts) {
      const double radius =
          (part.transform.translation() - current_.world_base.translation())
              .norm() +
          extent(part) - part.transform.translation().norm();
      const double speed =
          current_.linearVelocity().norm() + std::abs(current_.w) * radius + dynamic_speed;
      const double distance =
          gap(part, part.transform, obstacle, obstacle.at(current_.stamp));
      if (distance <= speed * dt +
                          terrainEnvelope(scene_, config_.terrain, radius) +
                          1e-5) {
        // Query only: the caller commits shared stop times after every robot
        // has examined the same beginning-of-interval state.
        hits.push_back({obstacle.motion,
                        {current_.stamp, part.link, obstacle.entity, distance}});
        break;
      }
    }
  }
  return hits;
}
Cloud Simulator::makeCloud(const Cloud &pending,
                           const Snapshot &snapshot) const {
  auto found = std::find_if(
      config_.lidars.begin(), config_.lidars.end(),
      [&](const LidarConfig &c) { return c.topic == pending.topic; });
  require(found != config_.lidars.end(), "unknown pending sensor");
  const auto frames = model_.fk(snapshot.joints);
  const Pose mount =
      frames.at(config_.base_link).inverse() * frames.at(found->link);
  const auto local = model_.placed(model_.sensor, mount.inverse(),
                                   config_.base_link, snapshot.joints);
  auto cloud = scanTrajectory(
      scene_, *found, pending.stamp,
      [this, mount](uint64_t t) { return trajectory(t) * mount; }, local,
      streamSeed(config_.seed, config_.robot_id), pending.frame_index,
      config_.threads);
  require(cloud.completion_stamp <= snapshot.stamp,
          "scan sampled future state");
  return cloud;
}
void Simulator::completeScans(std::vector<Event> &events) {
  for (auto &event : events)
    for (auto &cloud : event.clouds)
      cloud = makeCloud(cloud, event.snapshot);
}
World::World(Config config, Model model, Scene scene,
             std::vector<RobotSpec> specs) {
  require(std::none_of(config.cameras.begin(),config.cameras.end(),[](const CameraConfig &c){return c.enabled;}),
          "camera rendering currently requires the single-robot SDF launch");
  config.validate();
  require(!specs.empty() && specs.size() <= 4, "world supports 1..4 robots");
  validateCollisionBudget(config, model, scene, specs.size());
  std::set<std::string> ids;
  uint64_t horizon = config.step_ns;
  for (const auto &lidar : config.lidars)
    if (lidar.scan_mode == "rolling")
      horizon = std::max(horizon, uint64_t(std::ceil(1e9 / lidar.rate)) +
                                      config.step_ns);
  // Shared step/timeout settings permit at most one distinct deadline per step.
  double schedule_rate = (config.timeoutPhaseNs() ? 2e9 : 1e9) / config.step_ns + config.odom_rate;
  for (const auto &l : config.lidars)
    schedule_rate += l.rate;
  for (const auto &i : config.imus)
    schedule_rate += i.rate;
  require(std::ceil(horizon * 1e-9 * schedule_rate) + 4 <= config.history_limit,
          "world peer history budget exceeded");
  std::map<std::string, std::shared_ptr<Motion>> world_motions;
  for (const auto &spec : specs) {
    require(std::regex_match(spec.id, std::regex("[A-Za-z][A-Za-z0-9_]*")) &&
                ids.insert(spec.id).second,
            "invalid/duplicate world robot id");
    for (const auto &l : config.lidars)
      require(l.topic.front() != '/', "world sensor topics must be relative");
    for (const auto &i : config.imus)
      require(i.topic.front() != '/', "world sensor topics must be relative");
    config.robot_id = spec.id;
    config.initial_pose = spec.initial;
    robots_.push_back(std::make_unique<Simulator>(config, model, scene));
    robots_.back()->defer_scans_ = true;
    robots_.back()->peer_history_ns_ = horizon;
    auto &owned_scene = robots_.back()->scene_;
    for (auto *role :
         {&owned_scene.visual, &owned_scene.collision, &owned_scene.sensor})
      for (auto &g : *role)
        if (g.motion) {
          auto &shared = world_motions[g.entity];
          if (!shared)
            shared = g.motion;
          g.motion = shared;
        }
  }
  run_id = "m3-" + robots_.front()->run_id;
  for (size_t i = 0; i < size(); ++i) {
    auto &robot = *robots_[i];
    robot.run_id = run_id + "/" + robot.config_.robot_id;
    for (size_t j = 0; j < size(); ++j)
      if (i != j) {
        auto &other = *robots_[j];
        // Other robots expose collision geometry. Driven collision wheels are
        // coaxial cylinders, so spin leaves these surfaces invariant; all
        // remaining joints keep their configured pose.
        auto parts =
            other.model_.placed(other.model_.collision, Pose::Identity(),
                                other.config_.base_link, other.current_.joints);
        auto motion = std::make_shared<Motion>();
        motion->path = [this, j](uint64_t stamp) {
          return robots_.at(j)->trajectory(stamp);
        };
        for (auto &part : parts) {
          part.entity = "robot/" + other.config_.robot_id;
          part.motion = motion;
          robot.scene_.sensor.push_back(part);
        }
        const auto a = robot.model_.placed(
            robot.model_.collision, robot.current_.world_base, config.base_link,
            robot.current_.joints);
        const auto b = other.model_.placed(
            other.model_.collision, other.current_.world_base, config.base_link,
            other.current_.joints);
        for (const auto &pa : a)
          for (const auto &pb : b)
            require(gap(pa, pa.transform, pb, pb.transform) > 1e-6,
                    "world initial robot overlap");
      }
    robot.scene_.rebuild();
    robot.pause(true);
  }
}
std::unique_ptr<World> World::load(const std::filesystem::path &file) {
  auto n = YAML::Load(readText(file));
  keys(n, {"schema_version", "config", "robots"});
  require(n["schema_version"].as<int>() == 1, "world: unsupported schema");
  Scene scene;
  const auto base = std::filesystem::absolute(file).parent_path();
  auto config =
      Config::load(resolveResource(n["config"].as<std::string>(), base), scene);
  config.resources[std::filesystem::absolute(file).string()] = sha256(file);
  std::vector<RobotSpec> specs;
  require(n["robots"].IsSequence() && n["robots"].size() <= 4,
          "world: invalid robot list");
  for (auto r : n["robots"]) {
    keys(r, {"id", "xyz", "rpy"});
    specs.push_back(
        {r["id"].as<std::string>(),
         pose(vector(r["xyz"]), r["rpy"] ? vector(r["rpy"]) : Vec3::Zero())});
  }
  return std::make_unique<World>(
      config, Model::load(config.urdf, config.model_options), scene, specs);
}
bool World::command(size_t index, const Command &command) {
  return robots_.at(index)->command(command);
}
void World::pause(bool paused) {
  for (auto &r : robots_)
    r->pause(paused);
  paused_ = paused;
}
void World::fault(const std::string &reason) {
  for (auto &r : robots_)
    r->fault(reason);
  paused_ = true;
}
void World::exit() {
  for (auto &r : robots_)
    r->exit();
  paused_ = true;
}
std::vector<std::vector<Event>> World::advance(const EventSink &sink) {
  for (const auto &r : robots_)
    require(r->state() == State::Paused || r->state() == State::Running,
            "world is in a terminal state");
  std::vector<std::vector<Event>> result(size());
  try {
    const double dt = robot(0).config().step_ns * 1e-9;
    // All decisions use the same committed beginning-of-step state, independent
    // of robot order.
    for (size_t i = 0; i < size(); ++i)
      for (size_t j = i + 1; j < size(); ++j) {
        auto &a = *robots_[i], &b = *robots_[j];
        const auto ap = a.model_.placed(a.model_.collision, a.current_.world_base,
                                        a.config_.base_link, a.current_.joints);
        const auto bp = b.model_.placed(b.model_.collision, b.current_.world_base,
                                        b.config_.base_link, b.current_.joints);
        for (const auto &pa : ap)
          for (const auto &pb : bp) {
            const double ra =
                (pa.transform.translation() - a.current_.world_base.translation())
                    .norm() +
                extent(pa) - pa.transform.translation().norm();
            const double rb =
                (pb.transform.translation() - b.current_.world_base.translation())
                    .norm() +
                extent(pb) - pb.transform.translation().norm();
            const double bound =
                (a.config_.drive.max_v + a.config_.drive.max_w * ra +
                 b.config_.drive.max_v + b.config_.drive.max_w * rb) *
                dt;
            const double d = gap(pa, pa.transform, pb, pb.transform);
            if (d <= bound + terrainEnvelope(a.scene_, a.config_.terrain, ra) +
                         terrainEnvelope(b.scene_, b.config_.terrain, rb) +
                         1e-5) {
              a.stopForStep("robot/" + b.config_.robot_id, pa.link, d);
              b.stopForStep("robot/" + a.config_.robot_id, pb.link, d);
            }
          }
      }
    for (auto &r : robots_)
      r->beginStep();
    const uint64_t end = 1 + (robots_[0]->step_ + 1) * robot(0).config().step_ns;
    while (robot(0).snapshot().stamp < end) {
      const auto stamp = robot(0).snapshot().stamp;
      uint64_t next = end;
      for (const auto &r : robots_) {
        require(r->current_.stamp == stamp, "world clocks diverged");
        next = std::min(next, r->nextEvent(end));
      }
      std::vector<Simulator::MotionStep> motions;
      std::vector<std::vector<Simulator::DynamicContact>> contacts;
      for (auto &r : robots_) {
        motions.push_back(r->prepareMotion(next));
        contacts.push_back(r->dynamicContacts((next - stamp) * 1e-9));
      }
      // Commit only after collecting every decision. No robot can move the
      // obstacle's stop time backwards after a peer has committed its history.
      for (size_t i = 0; i < size(); ++i)
        for (const auto &hit : contacts[i]) {
          hit.motion->frozen_at = stamp;
          robots_[i]->stopForStep(hit.contact.entity, hit.contact.robot_link,
                                 hit.contact.clearance);
        }
      for (size_t i = 0; i < size(); ++i)
        if (auto event = robots_[i]->advanceMotion(next, motions[i]))
          result[i].push_back(std::move(*event));
    }
    for (auto &r : robots_)
      r->finishStep();
    // Every peer history is now committed through the same end timestamp.
    for (size_t i = 0; i < size(); ++i) {
      robots_[i]->completeScans(result[i]);
      if (sink)
        sink(i, result[i]);
    }
  } catch (const std::exception &e) {
    fault(e.what());
    throw;
  } catch (...) {
    fault("unknown world advance failure");
    throw;
  }
  return result;
}
std::vector<std::vector<Event>> World::tick(const EventSink &sink) {
  if (paused_ || std::any_of(robots_.begin(), robots_.end(), [](const auto &r) {
        return r->state() != State::Running;
      }))
    return std::vector<std::vector<Event>>(size());
  return advance(sink);
}
StepResult World::singleStep(const std::string &id, uint32_t count,
                             std::vector<std::vector<Event>> &events) {
  events.clear();
  const auto stamp = robot(0).snapshot().stamp;
  if (requests_.count(id))
    return requests_.at(id).first == count
               ? requests_.at(id).second
               : StepResult{false, 4, "changed step request", stamp};
  if (!paused_)
    return {false, 1, "world must be paused", stamp};
  for (const auto &r : robots_)
    if (r->state() != State::Paused)
      return {false, 1, "world is in a terminal state", stamp};
  if (id.empty() || id.size() > 128 || count == 0 || count > 10000 ||
      requests_.size() >= 4096)
    return {false, 2, "invalid/budgeted step request", stamp};
  double bytes = 0;
  for (const auto &r : robots_)
    for (const auto &l : r->config().lidars)
      bytes += (1 + count * r->config().step_ns * 1e-9 * l.rate) * l.columns *
               l.vertical_angles.size() * 24 * (l.publish_range_boundary ? 2 : 1);
  if (bytes > 512 * 1024 * 1024)
    return {false, 2, "world step output budget exceeded", stamp};
  events.resize(size());
  for (uint32_t step = 0; step < count; ++step) {
    auto batch = advance();
    for (size_t i = 0; i < size(); ++i)
      for (auto &e : batch[i])
        events[i].push_back(std::move(e));
  }
  StepResult result{true, 0, "world stepped", robot(0).snapshot().stamp};
  requests_[id] = {count, result};
  return result;
}
Json World::manifest() const {
  Json result = {{"run_id", run_id},
                 {"clock_publishers_required", 1},
                 {"robots", Json::array()},
                 {"peer_geometry", "URDF collision"},
                 {"collision_policy",
                  "conservative relative swept envelope; moving "
                  "entities stop on potential contact"}};
  for (const auto &r : robots_)
    result["robots"].push_back(r->manifest());
  return result;
}
} // namespace litesim3d
