#include "litesim3d/sensor_plugin.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>

namespace litesim3d {
Pose integrate(const Pose &initial, double v, double w, double dt) {
  require(std::isfinite(v) && std::isfinite(w) && std::isfinite(dt) && dt >= 0,
          "integrate: invalid velocity/time");
  const double a = w * dt;
  Pose delta = pose(Vec3::Zero(), Vec3(0, 0, a));
  // sinc form is continuous through zero angular velocity.
  const double sinc = std::abs(a) < 1e-8 ? 1 - a * a / 6 : std::sin(a) / a;
  const double cosc = std::abs(a) < 1e-8 ? a / 2 - a * a * a / 24 : (1 - std::cos(a)) / a;
  delta.translation() = Vec3(v * dt * sinc, v * dt * cosc, 0);
  return initial * delta;
}
Pose integrateFlight(const Pose &initial, const Vec3 &velocity, double yaw_rate, double dt) {
  require(velocity.allFinite(), "integrateFlight: invalid velocity");
  Pose delta = integrate(Pose::Identity(), 1, yaw_rate, dt);
  const double x = delta.translation().x(), y = delta.translation().y();
  delta.translation() = Vec3(velocity.x() * x - velocity.y() * y,
                            velocity.x() * y + velocity.y() * x, velocity.z() * dt);
  return initial * delta;
}
namespace {
uint64_t eventTime(uint64_t index, double rate) {
  return 1 + static_cast<uint64_t>(
                 std::llround(static_cast<long double>(index) * 1000000000.0L / rate));
}
Json asJson(const Pose &p) {
  Eigen::Quaterniond q(p.linear());
  return {{"xyz", {p.translation().x(), p.translation().y(), p.translation().z()}},
          {"xyzw", {q.x(), q.y(), q.z(), q.w()}}};
}
const char *stateName(State s) {
  switch (s) {
  case State::Ready:
    return "ready";
  case State::Running:
    return "running";
  case State::Paused:
    return "paused";
  case State::Fault:
    return "fault";
  case State::Exited:
    return "exited";
  }
  return "invalid";
}
} // namespace
Simulator::Simulator(Config config, Model model, Scene scene)
    : config_(std::move(config)), model_(std::move(model)), scene_(std::move(scene)),
      origin_(config_.initial_pose) {
  config_.validate();
  // Shape resources are immutable and shared. Contact-stop state belongs to this run.
  std::map<const Motion *, std::shared_ptr<Motion>> motions;
  for (auto *role : {&scene_.visual, &scene_.collision, &scene_.sensor})
    for (auto &g : *role) if (g.motion) {
      auto &copy = motions[g.motion.get()];
      if (!copy) copy = std::make_shared<Motion>(*g.motion);
      g.motion = copy;
    }
  require(model_.links.count(config_.base_link), "base_link is absent from URDF");
  require(!model_.collision.empty(), "robot must have collision geometry");
  current_.world_base = origin_;
  current_.joints = model_.positions(config_.initial_joints);
  initializeMotionPlugins();
  validateCollisionBudget(config_, model_, scene_);
  for (const auto &imu : config_.imus)
    require(model_.links.count(imu.link), "imu binding: unknown link " + imu.link);
  for (const auto &camera : config_.cameras)
    require(model_.links.count(camera.link), "camera binding: unknown optical link " + camera.link);
  for (auto &l : config_.lidars) {
    require(model_.links.count(l.link), "lidar binding: unknown link " + l.link);
    for (auto &link : l.excluded_links)
      require(model_.links.count(link), "lidar filter: unknown link " + link);
  }
  auto transforms = model_.fk(current_.joints);
  if (!chassis_plugin_ && config_.drive.model != "multicopter") {
    for (auto &name : {config_.drive.left_joint, config_.drive.right_joint}) {
      require(model_.joints.count(name), "wheel joint not found: " + name);
      const auto &j = model_.joints.at(name);
      require(j.type == "continuous" && j.mimic.empty(),
              "wheel joint must be continuous and independently driven: " + name);
      require(j.velocity >=
                  (config_.drive.max_v + config_.drive.max_w * config_.drive.wheel_separation / 2) /
                      config_.drive.wheel_radius,
              "wheel velocity limit incompatible with drive limits: " + name);
      bool cylinder = false;
      for (auto &g : model_.collision)
        if (g.link == j.child && g.shape->kind == Shape::Kind::Cylinder) {
          require(std::abs(g.shape->size.x() - config_.drive.wheel_radius) < 1e-6,
                  "wheel_radius differs from URDF");
          cylinder = true;
        }
      require(cylinder, "drive wheel requires a cylindrical collision shape: " + name);
    }
    const auto &left = model_.joints.at(config_.drive.left_joint);
    const auto &right = model_.joints.at(config_.drive.right_joint);
    const Pose base_inv = transforms.at(config_.base_link).inverse();
    Vec3 l = (base_inv * transforms.at(left.child)).translation(),
         r = (base_inv * transforms.at(right.child)).translation();
    require(std::abs((l - r).norm() - config_.drive.wheel_separation) < 1e-6 && l.y() > r.y(),
            "wheel_separation/order differs from URDF");
    for (const auto *joint : {&left, &right}) {
      const Vec3 axis =
          (base_inv * transforms.at(joint->parent) * joint->origin).linear() * joint->axis;
      require((axis - Vec3::UnitY()).norm() < 1e-6,
              "wheel axis must point along base +y: " + joint->name);
      for (const auto &g : model_.collision)
        if (g.link == joint->child) {
          require(g.shape->kind == Shape::Kind::Cylinder && g.transform.translation().norm() < 1e-9 &&
                      std::abs(g.transform.linear().col(2).dot(joint->axis)) > 1 - 1e-9,
                  "driven wheel collision must be a centered coaxial cylinder: " + joint->name);
        }
      for (const auto &[name, other] : model_.joints)
        require(other.parent != joint->child,
                "M1 driven wheel links must be leaves of the kinematic tree: " + joint->name);
    }
    for (auto &name : config_.drive.support_links)
      require(model_.links.count(name), "unknown support link: " + name);
  }
  if (config_.terrain.enabled) {
    origin_ = supportedPose(scene_, origin_, origin_.translation().z(),
                            config_.terrain, supportPoints());
    current_.world_base = origin_;
  }
  // The motion reference must be rigidly attached to the model root.
  std::string parent = config_.base_link;
  while (parent != model_.root) {
    bool found = false;
    for (auto &[name, j] : model_.joints) {
      if (j.child == parent) {
        require(j.type == "fixed", "base_link must be fixed relative to root");
        parent = j.parent;
        found = true;
        break;
      }
    }
    require(found, "invalid root/base chain");
  }
  current_.odom_root = transforms.at(config_.base_link).inverse();
  lidar_indices_.assign(config_.lidars.size(), 1);
  imu_indices_.assign(config_.imus.size(), 1);
  camera_indices_.assign(config_.cameras.size(), 1);
  history_.push_back({1, current_.world_base});
  scene_.rebuild();
  static std::atomic<uint64_t> serial{0};
  run_id = "m2-" + std::to_string(std::chrono::system_clock::now().time_since_epoch().count()) +
           "-" + std::to_string(serial++);
  double dist;
  auto contact = clearance(origin_, dist);
  require(!contact || dist > 1e-6, "initial robot pose collides with scene");
  Json undriven = Json::array();
  for (auto &name : model_.report["undriven_joints"])
    if (name != config_.drive.left_joint && name != config_.drive.right_joint)
      undriven.push_back(name);
  model_.report["undriven_joints"] = undriven;
  model_.report["sensor_bindings"] = Json::array();
  for (auto &lcfg : config_.lidars)
    model_.report["sensor_bindings"].push_back({{"name", lcfg.name},
                                                {"link", lcfg.link},
                                                {"filter", lcfg.filter},
                                                {"excluded_links", lcfg.excluded_links}});
}
bool Simulator::command(const Command &cmd) {
  auto reject = [&](const std::string &reason) {
    ++rejected_;
    last_error_ = reason;
    return false;
  };
  if (config_.control_mode != "twist") return reject("Twist input is disabled in ros2_control mode");
  if (state_ == State::Fault || state_ == State::Exited)
    return reject("command rejected in terminal state");
  for (double x : {cmd.v, cmd.w, cmd.lateral, cmd.vertical, cmd.roll_rate, cmd.pitch_rate})
    if (!std::isfinite(x))
      return reject("non-finite velocity command");
  const bool lateral = chassis_plugin_ ? chassis_plugin_->supportsLateral() : config_.drive.model == "multicopter";
  const bool vertical = chassis_plugin_ ? chassis_plugin_->supportsVertical() : config_.drive.model == "multicopter";
  if (cmd.roll_rate != 0 || cmd.pitch_rate != 0 || (!lateral && cmd.lateral != 0) || (!vertical && cmd.vertical != 0))
    return reject("unsupported non-planar velocity command");
  if (!cmd.frame.empty() && cmd.frame != config_.base_link)
    return reject("wrong command frame");
  if (cmd.stamp && (*cmd.stamp > current_.stamp ||
                    current_.stamp - *cmd.stamp > uint64_t(config_.timeout * 1e9)))
    return reject("stale or future command timestamp");
  if (pending_)
    ++coalesced_;
  pending_ = cmd;
  return true;
}
bool Simulator::jointCommand(const JointCommand &cmd) {
  auto reject = [&](const std::string &reason) {
    ++rejected_; last_error_ = reason; return false;
  };
  if (config_.control_mode != "ros2_control") return reject("joint commands require ros2_control mode");
  if (cmd.body_velocity) {
    if (!body_control_) return reject("body velocity requires the kinematic multicopter");
    for (double value : {cmd.body_velocity->vx, cmd.body_velocity->vy, cmd.body_velocity->vz, cmd.body_velocity->wz})
      if (!std::isfinite(value)) return reject("non-finite body velocity");
  }
  if (state_ == State::Fault || state_ == State::Exited) return reject("command rejected in terminal state");
  if (cmd.stamp && (*cmd.stamp > current_.stamp ||
      current_.stamp - *cmd.stamp > uint64_t(config_.timeout * 1e9)))
    return reject("stale or future joint command timestamp");
  for (const auto &[name, target] : cmd.joints) {
    const auto it = model_.joints.find(name);
    if (it == model_.joints.end() || it->second.type == "fixed" || !it->second.mimic.empty())
      return reject("joint command requires independent movable joint: " + name);
    if (passive_joints_.count(name)) return reject("passive wheel has no command interface: " + name);
    if (!std::isfinite(target.value)) return reject("non-finite joint command: " + name);
    if (target.mode != JointCommandMode::Hold && target.mode != JointCommandMode::Position &&
        target.mode != JointCommandMode::Velocity) return reject("unknown joint command mode");
    if (target.mode == JointCommandMode::Position &&
        (target.value < it->second.lower || target.value > it->second.upper))
      return reject("position command exceeds URDF limits: " + name);
    const auto bounds = joint_position_limits_.find(name);
    if (target.mode == JointCommandMode::Position && bounds != joint_position_limits_.end() &&
        (target.value < bounds->second.first || target.value > bounds->second.second))
      return reject("position command exceeds chassis limits: " + name);
  }
  if (pending_joints_) ++coalesced_;
  pending_joints_ = cmd;
  return true;
}
plugins::DriveStep Simulator::prepareJointMotion(double dt) const {
  plugins::DriveStep output;
  plugins::JointState predicted{current_.joints, {}};
  for (const auto &[name, joint] : model_.joints) {
    if (joint.type == "fixed" || !joint.mimic.empty() || passive_joints_.count(name)) continue;
    const double position = current_.joints.at(name);
    double velocity = 0;
    const auto target = joint_target_.joints.find(name);
    if (target != joint_target_.joints.end()) {
      if (target->second.mode == JointCommandMode::Position)
        velocity = (target->second.value - position) / dt;
      else if (target->second.mode == JointCommandMode::Velocity) velocity = target->second.value;
    }
    velocity = std::clamp(velocity, -joint.velocity, joint.velocity);
    const auto bounds = joint_position_limits_.find(name);
    const double lower = bounds == joint_position_limits_.end() ? joint.lower : bounds->second.first;
    const double upper = bounds == joint_position_limits_.end() ? joint.upper : bounds->second.second;
    const double next = std::clamp(position + velocity * dt, lower, upper);
    velocity = (next - position) / dt;
    predicted.position[name] = next;
    predicted.velocity[name] = output.joint_velocity[name] = velocity;
    if (driven_joints_.count(name)) output.wheel_delta[name] = next - position;
    else output.steering_position[name] = next;
  }
  if (joint_chassis_) output.average = output.velocity = joint_chassis_->forward(predicted);
  return output;
}
void Simulator::pause(bool paused) {
  require(state_ != State::Fault && state_ != State::Exited,
          "pause/resume not allowed in terminal state");
  if (paused)
    state_ = State::Paused;
  else
    state_ = State::Running;
}
std::optional<Contact> Simulator::clearance(const Pose &p, double &nearest,
    const std::map<std::string, double> *joints) const {
  nearest = INFINITY;
  std::optional<Contact> contact;
  const auto parts = model_.placed(model_.collision, p, config_.base_link, joints ? *joints : current_.joints);
  for (auto &part : parts)
    for (auto &obstacle : scene_.collision) {
      auto collision_shape = obstacle.shape->collision;
      if (obstacle.support && config_.drive.support_links.count(part.link)) {
        if (!config_.terrain.enabled && part.worldBounds().lo.z() >= -1e-7) continue;
        if (config_.terrain.enabled) {
          const auto bounds = obstacle.worldBounds();
          // A wheel straddles adjacent triangles/entities at a ramp seam. Query each potential
          // support surface near its footprint, rather than treating its neighbouring floor as a wall.
          auto support = scene_.supportAt(
              std::clamp(part.transform.translation().x(), bounds.lo.x(), bounds.hi.x()),
              std::clamp(part.transform.translation().y(), bounds.lo.y(), bounds.hi.y()),
              p.translation().z() - config_.terrain.clearance,
              config_.terrain.continuity + config_.drive.wheel_radius, obstacle.entity);
          if (support && support->entity == obstacle.entity) {
            const double axis = part.transform.linear().col(2).dot(support->normal);
            const double extent = part.shape->size.z() / 2 * std::abs(axis) +
                part.shape->size.x() * std::sqrt(std::max(0.0, 1 - axis * axis));
            if ((part.transform.translation() - support->point).dot(support->normal) - extent >= -0.002) {
              if (!obstacle.support_barriers) continue;
              collision_shape = obstacle.support_barriers;
            }
          }
        }
      }
      fcl::CollisionObjectd a(part.shape->collision, part.transform),
          b(collision_shape, obstacle.at(current_.stamp));
      fcl::CollisionRequestd request;
      fcl::CollisionResultd collision;
      fcl::collide(&a, &b, request, collision);
      double d = 0;
      if (!collision.isCollision()) {
        fcl::DistanceRequestd req;
        req.distance_tolerance = 1e-9;
        fcl::DistanceResultd res;
        d = fcl::distance(&a, &b, req, res);
      }
      require(std::isfinite(d), "collision backend returned non-finite distance");
      if (d < nearest) {
        nearest = std::max(0.0, d);
        contact = Contact{current_.stamp, part.link, obstacle.entity, nearest};
      }
    }
  return contact;
}
void Simulator::move(double dt, uint64_t end_stamp) {
  if (forced_stop_ && !config_.drive.continue_on_collision) {
    current_.v = current_.w = current_.lateral = current_.vertical = 0;
    target_ = Command{};
  }
  const Pose initial = current_.world_base;
  const double v = current_.v, w = current_.w;
  const Vec3 velocity = current_.linearVelocity();
  const bool flying = config_.drive.model == "multicopter";
  double radius = 0;
  for (auto &g :
       model_.placed(model_.collision, Pose::Identity(), config_.base_link, current_.joints)) {
    const auto b = g.worldBounds();
    radius = std::max(radius, b.lo.cwiseAbs().cwiseMax(b.hi.cwiseAbs()).norm());
  }
  const double speed = (flying || chassis_plugin_ ? velocity.stableNorm() : std::abs(v)) + std::abs(w) * radius;
  double elapsed = 0;
  bool blocked = forced_stop_;
  Pose accepted = initial;
  std::string terrain_error;
  const auto support_points = config_.terrain.enabled ? supportPoints() : std::vector<Vec3>{};
  auto at = [&](double time) {
    Pose p = flying || chassis_plugin_ ? integrateFlight(initial, velocity, w, time) : integrate(initial, v, w, time);
    if (config_.terrain.enabled)
      p = supportedPose(scene_, p, p.translation().z(), config_.terrain, support_points);
    return p;
  };
  // Every surface point moves at <= |v| + |omega|*radius. Advancing by <=
  // distance/speed cannot cross an obstacle, regardless of wall thickness.
  for (size_t iteration = 0; !blocked && elapsed < dt && speed > 1e-15; ++iteration) {
    double distance;
    Pose proposed;
    try { proposed = at(elapsed); }
    catch (const std::exception &e) { blocked = true; terrain_error = e.what(); break; }
    auto contact = clearance(proposed, distance);
    // Permit separation from the collision margin. The probe moves every
    // surface point less than the current gap, so it cannot cross a thin wall.
    if (distance > 1e-9 && distance <= 2e-5 && iteration < 10000) {
      const double probe_dt = std::min(dt - elapsed, distance * .5 / speed);
      try {
        const Pose probe = at(elapsed + probe_dt);
        const double travel = (probe.translation() - proposed.translation()).norm() +
            radius * Eigen::AngleAxisd(probe.linear() * proposed.linear().transpose()).angle();
        double probe_distance;
        clearance(probe, probe_distance);
        if (travel <= distance * .75 && probe_distance > distance + 1e-10) {
          elapsed += probe_dt;
          accepted = probe;
          continue;
        }
      } catch (const std::exception &) {
        // The normal terrain/collision path below retains the stop reason.
      }
    }
    if (distance <= 1e-5 || iteration >= 10000) {
      blocked = true;
      if (contact) {
        contact->stamp = end_stamp;
        contacts_.push_back(*contact);
        ++contact_count_;
      }
      break;
    }
    accepted = proposed;
    const double remaining = dt - elapsed;
    if (!std::isfinite(distance)) {
      if (!config_.terrain.enabled) { elapsed = dt; break; }
      distance = 1;
    }
    const double safe = config_.terrain.enabled
        ? std::min(.002 / speed, (distance - 1e-5) * .45 / speed)
        : (distance - 1e-5) * 0.9 / speed;
    if (safe >= remaining) {
      elapsed = dt;
      break;
    }
    if (safe < 1e-12) {
      blocked = true;
      if (contact) {
        contact->stamp = end_stamp;
        contacts_.push_back(*contact);
        ++contact_count_;
      }
      break;
    }
    elapsed += safe;
  }
  if (speed <= 1e-15)
    elapsed = dt;
  if (!blocked && speed > 1e-15) {
    try {
      accepted = at(elapsed);
      if (config_.terrain.enabled) {
        double distance;
        auto contact = clearance(accepted, distance);
        if (contact && distance <= 1e-5) {
          accepted = initial; elapsed = 0; blocked = true;
          contact->stamp = end_stamp; contacts_.push_back(*contact); ++contact_count_;
        }
      }
    } catch (const std::exception &e) {
      blocked = true; terrain_error = e.what(); accepted = initial; elapsed = 0;
    }
  } else if (!config_.terrain.enabled) accepted = at(elapsed);
  current_.world_base = accepted;
  accepted_motion_fraction_ = forced_stop_ ? 0 : std::clamp(elapsed / dt, 0.0, 1.0);
  drive_blocked_ = blocked && (!config_.drive.continue_on_collision || !terrain_error.empty());
  const double wheel_dt = drive_blocked_ ? elapsed : dt;
  if (!flying && !chassis_plugin_) {
    const double wl = (v - w * config_.drive.wheel_separation / 2) / config_.drive.wheel_radius;
    const double wr = (v + w * config_.drive.wheel_separation / 2) / config_.drive.wheel_radius;
    current_.joints[config_.drive.left_joint] += wl * wheel_dt;
    current_.joints[config_.drive.right_joint] += wr * wheel_dt;
    current_.joints = model_.positions(current_.joints);
    current_.joint_velocities[config_.drive.left_joint] = drive_blocked_ ? 0 : wl;
    current_.joint_velocities[config_.drive.right_joint] = drive_blocked_ ? 0 : wr;
  }
  if (blocked) {
    current_.v = 0;
    current_.w = 0;
    current_.lateral = current_.vertical = 0;
    if (drive_blocked_) target_ = Command{};
    last_error_ = terrain_error.empty() ? "collision stop" : terrain_error;
    ++discontinuities_;
  }
  current_.stamp = end_stamp;
  motion_blocked_ = blocked;
  current_.odom_v = blocked ? 0 : v;
  current_.odom_w = blocked ? 0 : w;
  if (!odometry_plugin_ && (config_.odom_error.enabled || config_.drive.continue_on_collision)) {
    const auto &e = config_.odom_error;
    current_.odom_v = v * e.linear_scale + e.linear_bias + e.linear_sigma *
        randomNormal(streamSeed(config_.seed, config_.robot_id + "/odom/linear"), motion_samples_);
    current_.odom_w = w * e.angular_scale + e.angular_bias + e.angular_sigma *
        randomNormal(streamSeed(config_.seed, config_.robot_id + "/odom/angular"), motion_samples_);
    current_.odom_base = integrate(current_.odom_base, current_.odom_v, current_.odom_w, wheel_dt);
  } else if (!odometry_plugin_) current_.odom_base = origin_.inverse() * current_.world_base;
  ++motion_samples_;
  if (contacts_.size() > config_.journal_limit)
    contacts_.erase(contacts_.begin(), contacts_.end() - config_.journal_limit);
  current_.odom_root =
      current_.odom_base * model_.fk(current_.joints).at(config_.base_link).inverse();
}
void Simulator::beginStep() {
  started_ = true;
  require(step_ < 1000000000000ULL, "simulation step budget exceeded");
  if (pending_joints_) {
    joint_target_ = std::move(*pending_joints_);
    pending_joints_.reset();
    if (body_control_) {
      const auto body = joint_target_.body_velocity.value_or(plugins::Twist{});
      target_ = Command{};
      target_.v = body.vx; target_.lateral = body.vy; target_.vertical = body.vz; target_.w = body.wz;
    }
    accepted_stamp_ = current_.stamp;
    timed_out_ = false;
    Json targets = Json::object();
    for (const auto &[name, target] : joint_target_.joints)
      targets[name] = {{"mode", uint8_t(target.mode)}, {"value", target.value}};
    inputs_.push_back({{"sequence", sequence_++}, {"step", step_ + 1},
                       {"accepted_time_ns", accepted_stamp_}, {"joints", targets}});
    if (body_control_) inputs_.back()["body_velocity"] = {target_.v, target_.lateral, target_.vertical, target_.w};
    if (inputs_.size() > config_.journal_limit) { inputs_.erase(inputs_.begin()); ++evicted_inputs_; }
  }
  if (pending_) {
    target_ = *pending_;
    pending_.reset();
    accepted_stamp_ = current_.stamp;
    timed_out_ = false;
    inputs_.push_back({{"sequence", sequence_++},
                       {"step", step_ + 1},
                       {"accepted_time_ns", accepted_stamp_},
                       {"v", target_.v},
                       {"w", target_.w}});
    if (config_.drive.model == "multicopter" || chassis_plugin_) {
      inputs_.back()["lateral"] = target_.lateral;
      inputs_.back()["vertical"] = target_.vertical;
    }
    if (inputs_.size() > config_.journal_limit) { inputs_.erase(inputs_.begin()); ++evicted_inputs_; }
  }
  if (forced_stop_ && !config_.drive.continue_on_collision) {
    target_ = Command{};
    current_.v = current_.w = current_.lateral = current_.vertical = 0;
  }
}
uint64_t Simulator::nextEvent(uint64_t end) const {
  uint64_t next = std::min(end, eventTime(odom_index_, config_.odom_rate));
  for (size_t i = 0; i < lidar_indices_.size(); ++i)
    next = std::min(next, eventTime(lidar_indices_[i], config_.lidars[i].rate));
  for (size_t i = 0; i < imu_indices_.size(); ++i)
    next = std::min(next, eventTime(imu_indices_[i], config_.imus[i].rate));
  for (size_t i = 0; i < camera_indices_.size(); ++i)
    if (config_.cameras[i].enabled)
      next = std::min(next, eventTime(camera_indices_[i], config_.cameras[i].rate));
  // Commands are accepted only at physical-step boundaries, so every possible
  // timeout has this phase. Keep it even without a live command: a peer's
  // command must not repartition this robot's integration/noise stream.
  const uint64_t timeout_sample = 1 + step_ * config_.step_ns + config_.timeoutPhaseNs();
  if (timeout_sample > current_.stamp)
    next = std::min(next, timeout_sample);
  return next;
}
Simulator::MotionStep Simulator::prepareMotion(uint64_t next) {
  const uint64_t deadline = accepted_stamp_ + uint64_t(std::llround(config_.timeout * 1e9));
  if (current_.stamp >= deadline) {
    target_ = Command{};
    joint_target_ = JointCommand{};
    timed_out_ = true;
  }
  const double dt = double(next - current_.stamp) * 1e-9;
  if (chassis_plugin_) {
    const Vec3 old_linear = current_.linearVelocity();
    const double old_v = current_.v, old_w = current_.w;
    plugins::ChassisInput input{next - current_.stamp,
        {target_.v, target_.lateral, target_.vertical, target_.w}, plugin_drive_velocity_,
        {current_.joints, current_.joint_velocities}};
    auto output = joint_chassis_ ? prepareJointMotion(dt) : chassis_plugin_->step(input);
    for (const auto *twist : {&output.average, &output.velocity})
      for (double value : {twist->vx, twist->vy, twist->vz, twist->wz})
        require(std::isfinite(value), "chassis plugin returned non-finite velocity");
    for (const auto &[name, delta] : output.wheel_delta) {
      require(model_.joints.count(name) && model_.joints.at(name).type == "continuous" &&
              model_.joints.at(name).mimic.empty(), "chassis plugin returned invalid wheel: " + name);
      require(std::isfinite(delta) && output.joint_velocity.count(name) &&
              std::isfinite(output.joint_velocity.at(name)), "chassis plugin returned invalid encoder motion: " + name);
    }
    current_.v = output.average.vx; current_.lateral = output.average.vy;
    current_.vertical = output.average.vz; current_.w = output.average.wz;
    return {old_v, old_w, output.velocity.vx, output.velocity.wz, current_.world_base,
            old_linear, {output.velocity.vx, output.velocity.vy, output.velocity.vz}, std::move(output)};
  }
  const auto &d = config_.drive;
  const double tv = std::clamp(target_.v, -d.max_v, d.max_v),
               tw = std::clamp(target_.w, -d.max_w, d.max_w);
  const double old_v = current_.v, old_w = current_.w;
  const double old_drive_v = d.continue_on_collision ? drive_v_ : old_v;
  const double old_drive_w = d.continue_on_collision ? drive_w_ : old_w;
  const double limit =
      (old_drive_v * tv < 0 || std::abs(tv) < std::abs(old_drive_v)) ? d.deceleration : d.acceleration;
  const double new_v =
      d.unlimited_acceleration ? tv : old_drive_v + std::clamp(tv - old_drive_v, -limit * dt, limit * dt);
  double new_w = d.unlimited_acceleration
                           ? tw
                           : old_drive_w + std::clamp(tw - old_drive_w, -d.angular_acceleration * dt,
                                                d.angular_acceleration * dt);
  if (d.model == "multicopter") {
    const Vec3 old = current_.linearVelocity();
    Vec3 desired(target_.v, target_.lateral, target_.vertical);
    if (desired.stableNorm() > d.max_v) desired *= d.max_v / desired.stableNorm();
    Vec3 delta = desired - old;
    const double max_delta = (desired.dot(old) < 0 || desired.stableNorm() < old.stableNorm()
                                 ? d.deceleration : d.acceleration) * dt;
    if (!d.unlimited_acceleration && delta.stableNorm() > max_delta)
      delta *= max_delta / delta.stableNorm();
    const Vec3 next_velocity = old + delta;
    const Vec3 mid = d.unlimited_acceleration ? next_velocity : Vec3((old + next_velocity) / 2);
    current_.v = mid.x(); current_.lateral = mid.y(); current_.vertical = mid.z();
    current_.w = d.unlimited_acceleration ? new_w : (old_w + new_w) / 2;
    return {old_v, old_w, next_velocity.x(), new_w, current_.world_base, old, next_velocity,
            body_control_ ? prepareJointMotion(dt) : plugins::DriveStep{}};
  }
  if (d.model == "ackermann") {
    const double turn_limit = std::abs(new_v) * std::tan(d.max_steering) / d.wheelbase;
    new_w = std::clamp(new_w, -turn_limit, turn_limit);
  }
  current_.v = d.unlimited_acceleration ? new_v : (old_drive_v + new_v) / 2;
  current_.w = d.unlimited_acceleration ? new_w : (old_drive_w + new_w) / 2;
  if (d.model == "ackermann") {
    const double mid_limit = std::abs(current_.v) * std::tan(d.max_steering) / d.wheelbase;
    current_.w = std::clamp(current_.w, -mid_limit, mid_limit);
  }
  const Pose old_pose = current_.world_base;
  return {old_v, old_w, new_v, new_w, old_pose, Vec3::Zero(), Vec3::Zero(), {}};
}
std::optional<Event> Simulator::advanceMotion(uint64_t next, const MotionStep &motion) {
  auto [old_v, old_w, new_v, new_w, old_pose, old_linear, new_linear, plugin_drive] = motion;
  const double dt = double(next - current_.stamp) * 1e-9;
  const uint64_t dt_ns = next - current_.stamp;
  const plugins::JointState previous_encoders{current_.joints, current_.joint_velocities};
  if (chassis_plugin_ || body_control_) moveSteering(plugin_drive.steering_position);
  if (joint_chassis_) {
    // Use collision-accepted steering, never the requested (possibly blocked) angle.
    const plugins::JointState accepted{current_.joints, plugin_drive.joint_velocity};
    plugin_drive.average = plugin_drive.velocity = joint_chassis_->forward(accepted);
    current_.v = new_v = plugin_drive.velocity.vx;
    current_.w = new_w = plugin_drive.velocity.wz;
    current_.lateral = plugin_drive.velocity.vy; current_.vertical = plugin_drive.velocity.vz;
    new_linear = {current_.v, current_.lateral, current_.vertical};
  }
  if (body_control_) {
    current_.joints = model_.positions(current_.joints);
    current_.joint_velocities = plugin_drive.joint_velocity;
    for (const auto &[name, value] : plugin_drive.steering_position)
      current_.joint_velocities[name] = (current_.joints.at(name) - previous_encoders.position.at(name)) / dt;
  }
  const auto &d = config_.drive;
  const double integrated_v = current_.v, integrated_w = current_.w;
  move(dt, next);
  drive_v_ = drive_blocked_ ? 0 : new_v;
  drive_w_ = drive_blocked_ ? 0 : new_w;
  if (!motion_blocked_) {
    current_.v = new_v;
    current_.w = new_w;
    if (d.model == "multicopter" || chassis_plugin_) {
      current_.lateral = new_linear.y(); current_.vertical = new_linear.z();
    }
  }
  if (chassis_plugin_) {
    const double fraction = drive_blocked_ ? accepted_motion_fraction_ : 1;
    for (const auto &[name, delta] : plugin_drive.wheel_delta) current_.joints[name] += delta * fraction;
    if (joint_chassis_) {
      const auto rates = joint_chassis_->passiveVelocities(
          {current_.joints, plugin_drive.joint_velocity}, plugin_drive.average);
      require(rates.size() == passive_joints_.size(), "passive wheel feedback size mismatch");
      for (const auto &[name, rate] : rates) {
        require(passive_joints_.count(name) && std::isfinite(rate), "invalid passive wheel feedback: " + name);
        // Passive wheels follow actual travel, even when driven wheels keep slipping.
        current_.joints[name] += rate * dt * accepted_motion_fraction_;
        plugin_drive.joint_velocity[name] = motion_blocked_ ? 0 : rate;
      }
    }
    current_.joints = model_.positions(current_.joints);
    current_.joint_velocities = plugin_drive.joint_velocity;
    // Feedback reflects accepted articulated motion, including blocked targets.
    if (joint_chassis_)
      for (const auto &[name, value] : plugin_drive.steering_position)
        current_.joint_velocities[name] = (current_.joints.at(name) - previous_encoders.position.at(name)) / dt;
    if (drive_blocked_) for (auto &[name, velocity] : current_.joint_velocities) velocity = 0;
    plugin_drive_velocity_ = drive_blocked_ ? plugins::Twist{} : plugin_drive.velocity;
    const auto output = odometry_plugin_->step({dt_ns, previous_encoders, {current_.joints, current_.joint_velocities}});
    require(output.pose.matrix().allFinite(), "odometry plugin returned non-finite pose");
    for (double value : {output.velocity.vx, output.velocity.vy, output.velocity.vz, output.velocity.wz})
      require(std::isfinite(value), "odometry plugin returned non-finite velocity");
    current_.odom_base = output.pose;
    current_.odom_v = output.velocity.vx; current_.odom_w = output.velocity.wz;
    current_.odom_lateral = output.velocity.vy; current_.odom_vertical = output.velocity.vz;
    current_.odom_root = current_.odom_base * model_.fk(current_.joints).at(config_.base_link).inverse();
  } else if (d.model != "multicopter") {
    current_.joint_velocities[d.left_joint] =
        (drive_v_ - drive_w_ * d.wheel_separation / 2) / d.wheel_radius;
    current_.joint_velocities[d.right_joint] =
        (drive_v_ + drive_w_ * d.wheel_separation / 2) / d.wheel_radius;
  }
  // Mimic positions already follow their source in Model::positions(). Keep
  // their reported velocities consistent too, including chains and stops.
  for (const auto &name : model_.order) {
    const auto &joint = model_.joints.at(name);
    if (!joint.mimic.empty()) {
      const auto source = current_.joint_velocities.find(joint.mimic);
      current_.joint_velocities[name] =
          source == current_.joint_velocities.end() ? 0 : source->second * joint.multiplier;
    }
  }
  if (odometry_plugin_) {
    // Encoder odometry has already produced pose and endpoint twist independently.
  } else if (!config_.odom_error.enabled) {
    current_.odom_v = d.continue_on_collision ? drive_v_ : current_.v;
    current_.odom_w = d.continue_on_collision ? drive_w_ : current_.w;
  } else if (drive_blocked_) {
    current_.odom_v = current_.odom_w = 0;
  } else if (d.continue_on_collision) {
    // Pose uses interval-average wheel speed; twist uses the encoder speed at
    // this sample's timestamp, with the same bias/noise sample as the pose.
    current_.odom_v += (drive_v_ - integrated_v) * config_.odom_error.linear_scale;
    current_.odom_w += (drive_w_ - integrated_w) * config_.odom_error.angular_scale;
  }
  if (!odometry_plugin_) {
    current_.odom_lateral = current_.lateral; current_.odom_vertical = current_.vertical;
  }
  kinematics_.world_body = current_.world_base;
  const auto rotation = current_.world_base.linear().eval();
  kinematics_.omega = rotation.col(2) * current_.w;
  kinematics_.alpha = rotation.col(2) * ((current_.w - old_w) / dt);
  kinematics_.acceleration = rotation.col(0) * ((current_.v - old_v) / dt) +
                            rotation.col(1) * (current_.v * current_.w);
  if (config_.terrain.enabled) {
    Eigen::AngleAxisd delta(rotation * old_pose.linear().transpose());
    kinematics_.omega = delta.axis() * (delta.angle() / dt);
    kinematics_.acceleration = (rotation.col(0) * current_.v - old_pose.linear().col(0) * old_v) / dt;
  }
  if (motion_blocked_ || (d.unlimited_acceleration && (old_v != new_v || old_w != new_w))) {
    // A discontinuity is an experiment/control event, not an infinite accelerometer impulse.
    kinematics_.acceleration = rotation.col(1) * (current_.v * current_.w);
    kinematics_.alpha.setZero();
    if (!motion_blocked_) ++discontinuities_;
  }
  if (d.model == "multicopter" || chassis_plugin_) {
    const Vec3 velocity = current_.linearVelocity();
    const Vec3 centripetal = Vec3(0, 0, current_.w).cross(velocity);
    kinematics_.acceleration = rotation * ((velocity - old_linear) / dt + centripetal);
    if (motion_blocked_ || (d.unlimited_acceleration &&
        (old_linear != new_linear || old_w != new_w))) {
      kinematics_.acceleration = rotation * centripetal;
      kinematics_.alpha.setZero();
      if (!motion_blocked_ && old_v == new_v && old_w == new_w) ++discontinuities_;
    }
  }
  history_.push_back({next, current_.world_base});
  require(history_.size() <= config_.history_limit, "trajectory history budget exceeded");
  Event event;
  event.snapshot = current_;
  if (next == eventTime(odom_index_, config_.odom_rate)) {
    event.odom = true;
    ++odom_index_;
  }
  for (size_t i = 0; i < lidar_indices_.size(); ++i)
    if (next == eventTime(lidar_indices_[i], config_.lidars[i].rate)) {
      const auto &cfg = config_.lidars[i];
      Cloud pending;
      pending.topic = cfg.topic; pending.frame_index = lidar_indices_[i] - 1;
      pending.stamp = cfg.scan_mode == "rolling" ? eventTime(lidar_indices_[i] - 1, cfg.rate) : next;
      event.clouds.push_back(defer_scans_ ? pending : makeCloud(pending, current_));
      ++lidar_indices_[i];
    }
  for (size_t i = 0; i < imu_indices_.size(); ++i)
    if (next == eventTime(imu_indices_[i], config_.imus[i].rate)) {
      const auto &cfg = config_.imus[i];
      auto output = sampleSensor(cfg.name,next,imu_indices_[i]-1,current_);
      require(output.imus.size() == 1 && output.clouds.empty() && output.cameras.empty(),
              "IMU plugin must return one IMU sample");
      require(output.imus[0].stamp == next && output.imus[0].frame == cfg.link && output.imus[0].topic == cfg.topic,
              "IMU plugin returned inconsistent metadata");
      event.imus.push_back(std::move(output.imus[0]));
      ++imu_indices_[i];
    }
  for (size_t i = 0; i < camera_indices_.size(); ++i)
    if (config_.cameras[i].enabled && next == eventTime(camera_indices_[i], config_.cameras[i].rate)) {
      auto output = sampleSensor(config_.cameras[i].name,next,camera_indices_[i]-1,current_);
      require(output.clouds.empty() && output.imus.empty() &&
          output.cameras == std::vector<std::string>{config_.cameras[i].name},
          "camera plugin must request one render for its own instance");
      event.cameras.push_back(i);
      ++camera_indices_[i];
    }
  uint64_t oldest = next;
  for (size_t i = 0; i < lidar_indices_.size(); ++i)
    if (config_.lidars[i].scan_mode == "rolling")
      oldest = std::min(oldest, eventTime(lidar_indices_[i] - 1, config_.lidars[i].rate));
  if (peer_history_ns_) oldest = std::min(oldest, next > peer_history_ns_ ? next - peer_history_ns_ : 1);
  while (history_.size() > 2 && history_[1].stamp <= oldest) history_.pop_front();
  if (event.odom || !event.clouds.empty() || !event.imus.empty() || !event.cameras.empty())
    return event;
  return std::nullopt;
}
void Simulator::finishStep() {
  ++step_;
  forced_stop_ = false;
}
std::vector<Event> Simulator::advanceStep() {
  beginStep();
  const uint64_t end = 1 + (step_ + 1) * config_.step_ns;
  std::vector<Event> events;
  while (current_.stamp < end) {
    const uint64_t next = nextEvent(end);
    const auto motion = prepareMotion(next);
    for (const auto &hit : dynamicContacts((next - current_.stamp) * 1e-9)) {
      hit.motion->frozen_at = current_.stamp;
      stopForStep(hit.contact.entity, hit.contact.robot_link, hit.contact.clearance);
    }
    if (auto event = advanceMotion(next, motion))
      events.push_back(std::move(*event));
  }
  finishStep();
  return events;
}
Pose Simulator::trajectory(uint64_t stamp) const {
  require(!history_.empty() && stamp >= history_.front().stamp && stamp <= history_.back().stamp,
          "trajectory timestamp outside retained interval");
  auto hi = std::lower_bound(history_.begin(), history_.end(), stamp,
      [](const History &entry, uint64_t time) { return entry.stamp < time; });
  if (hi->stamp == stamp) return hi->pose;
  const auto lo = std::prev(hi);
  const double a = double(stamp - lo->stamp) / (hi->stamp - lo->stamp);
  return quaternionPose((1 - a) * lo->pose.translation() + a * hi->pose.translation(),
      Eigen::Quaterniond(lo->pose.linear()).slerp(a, Eigen::Quaterniond(hi->pose.linear())));
}
std::vector<Event> Simulator::tick() {
  if (state_ != State::Running)
    return {};
  try {
    return advanceStep();
  } catch (const std::exception &e) {
    fault(e.what());
    throw;
  }
}
StepResult Simulator::singleStep(const std::string &id, uint32_t count,
                                 std::vector<Event> &output) {
  if (requests_.count(id)) {
    const auto &cached = requests_.at(id);
    if (cached.first != count)
      return {false, 4, "request_id reused with different steps", current_.stamp};
    return cached.second;
  }
  if (state_ != State::Paused)
    return {false, 1, "step requires paused state", current_.stamp};
  if (id.empty() || id.size() > 128 || count == 0 || count > 10000)
    return {false, 2, "invalid request_id or steps", current_.stamp};
  if (requests_.size() >= 4096)
    return {false, 3, "request cache full; start a new experiment", current_.stamp};
  double output_bytes = 0;
  for (const auto &c : config_.lidars)
    output_bytes += (1 + count * config_.step_ns * 1e-9 * c.rate) * c.columns *
                    c.vertical_angles.size() * 24 * (c.publish_range_boundary ? 2 : 1);
  if (output_bytes > 512 * 1024 * 1024)
    return {false, 2, "step output budget exceeded; use smaller batches", current_.stamp};
  try {
    for (uint32_t i = 0; i < count; ++i) {
      auto events = advanceStep();
      output.insert(output.end(), std::make_move_iterator(events.begin()),
                    std::make_move_iterator(events.end()));
    }
  } catch (const std::exception &e) {
    fault(e.what());
    throw;
  }
  StepResult result{true, 0, "stepped", current_.stamp};
  requests_[id] = {count, result};
  return result;
}
void Simulator::setPose(const Pose &initial) {
  require(state_ == State::Ready && !started_,
          "set_pose requires ready state before experiment starts");
  auto candidate = config_;
  candidate.initial_pose = initial;
  candidate.validate();
  const Pose supported = config_.terrain.enabled
      ? supportedPose(scene_, initial, initial.translation().z(), config_.terrain, supportPoints())
      : initial;
  double distance;
  auto contact = clearance(supported, distance);
  require(!contact || distance > 1e-6, "set_pose collides with scene");
  origin_ = supported;
  config_.initial_pose = initial;
  current_.world_base = supported;
  current_.odom_base = Pose::Identity();
  current_.odom_root = model_.fk(current_.joints).at(config_.base_link).inverse();
  target_ = Command{};
  drive_v_ = drive_w_ = 0;
  motion_blocked_ = drive_blocked_ = false;
  pending_.reset();
  pending_joints_.reset();
  joint_target_ = JointCommand{};
  history_.clear();
  history_.push_back({current_.stamp, current_.world_base});
  kinematics_ = Kinematics{};
}
void Simulator::fault(const std::string &reason) {
  state_ = State::Fault;
  last_error_ = reason;
  pending_.reset();
  pending_joints_.reset();
  joint_target_ = JointCommand{};
  current_.v = 0;
  current_.w = 0;
  current_.lateral = current_.vertical = 0;
}
void Simulator::exit() {
  state_ = State::Exited;
  pending_.reset();
  pending_joints_.reset();
  joint_target_ = JointCommand{};
}
Json Simulator::diagnostics() const {
  return {{"run_id", run_id},
          {"robot_id", config_.robot_id},
          {"state", stateName(state_)},
          {"step", step_},
          {"time_ns", current_.stamp},
          {"rejected_commands", rejected_},
          {"coalesced_commands", coalesced_},
          {"pending_command", pending_.has_value() || pending_joints_.has_value()},
          {"command_timeout", timed_out_},
          {"contact_count", contact_count_},
          {"retained_contacts", contacts_.size()},
          {"retained_inputs", inputs_.size()},
          {"evicted_inputs", evicted_inputs_},
          {"trajectory_samples", history_.size()},
          {"discontinuities", discontinuities_},
          {"scene_bvh_builds", scene_.build_count},
          {"last_error", last_error_}};
}
Json Simulator::manifest(const std::string &reason) const {
  Json resolved = config_.resolved;
  const auto &d = config_.drive;
  resolved["drive"] = {{"model", d.model}, {"wheelbase", d.wheelbase}, {"max_steering_rad", d.max_steering}, {"wheel_radius", d.wheel_radius},
                       {"wheel_separation", d.wheel_separation},
                       {"max_v", d.max_v},
                       {"max_w", d.max_w},
                       {"acceleration", d.acceleration},
                       {"deceleration", d.deceleration},
                       {"angular_acceleration", d.angular_acceleration},
                       {"unlimited_acceleration", d.unlimited_acceleration},
                       {"continue_on_collision", d.continue_on_collision},
                       {"left_joint", d.left_joint},
                       {"right_joint", d.right_joint},
                       {"support_links", d.support_links}};
  resolved["plugins"] = Json::array();
  for (const auto &plugin : config_.plugins)
    resolved["plugins"].push_back({{"category", plugin.category}, {"name", plugin.name},
        {"class", plugin.class_name}, {"parameters", plugin.parameters}});
  resolved["lidars"] = Json::array();
  for (const auto &c : config_.lidars)
    resolved["lidars"].push_back({{"name", c.name},
                                  {"link", c.link},
                                  {"topic", c.topic},
                                  {"vertical_angles_rad", c.vertical_angles},
                                  {"columns", c.columns},
                                  {"rate_hz", c.rate},
                                  {"min_range_m", c.min_range},
                                  {"max_range_m", c.max_range},
                                  {"publish_range_boundary", c.publish_range_boundary},
                                  {"horizontal_start_rad", c.horizontal_start},
                                  {"horizontal_end_rad", c.horizontal_end},
                                  {"scan_mode", c.scan_mode},
                                  {"scan_pattern", c.scan_pattern},
                                  {"pattern_file", c.pattern_table ? c.pattern_table->source : ""},
                                  {"pattern_sha256", c.pattern_table ? c.pattern_table->source_sha256 : ""},
                                  {"pattern_samples", c.pattern_table ? c.pattern_table->samples.size() : 0},
                                  {"pattern_cycle_s", c.pattern_table ? c.pattern_table->samples.size() / 200000.0 : 0.0},
                                  {"vertical_min_rad", c.vertical_min},
                                  {"vertical_max_rad", c.vertical_max},
                                  {"cone_half_angle_rad", c.cone_half_angle},
                                  {"pattern_hz", c.pattern_hz},
                                  {"ring_delays_s", c.ring_delays},
                                  {"range_bias_m", c.range_bias},
                                  {"range_sigma_m", c.range_sigma},
                                  {"dropout", c.dropout},
                                  {"self_filter", c.filter},
                                  {"excluded_links", c.excluded_links}});
  resolved["pose_covariance"] = config_.pose_covariance;
  resolved["twist_covariance"] = config_.twist_covariance;
  resolved["initial_joints"] = config_.initial_joints;
  resolved["step_ns"] = config_.step_ns;
  resolved["odom_rate"] = config_.odom_rate;
  resolved["command_timeout"] = config_.timeout;
  resolved["control_mode"] = config_.control_mode;
  resolved["real_time_factor"] = config_.real_time_factor;
  resolved["seed"] = config_.seed;
  resolved["threads"] = config_.threads;
  resolved["history_limit"] = config_.history_limit;
  resolved["journal_limit"] = config_.journal_limit;
  resolved["terrain"] = {{"enabled", config_.terrain.enabled}, {"max_slope_rad", config_.terrain.max_slope},
      {"clearance_m", config_.terrain.clearance}, {"continuity_m", config_.terrain.continuity}};
  const auto &error = config_.odom_error;
  resolved["odom_error"] = {{"enabled", error.enabled}, {"linear_scale", error.linear_scale},
      {"angular_scale", error.angular_scale}, {"linear_bias", error.linear_bias}, {"angular_bias", error.angular_bias},
      {"linear_sigma", error.linear_sigma}, {"angular_sigma", error.angular_sigma}};
  resolved["imus"] = Json::array();
  auto vectorJson = [](const Vec3 &v) { return Json::array({v.x(), v.y(), v.z()}); };
  Json moving = Json::array();
  for (const auto &g : scene_.collision) if (g.motion && !g.motion->path)
    moving.push_back({{"entity", g.entity}, {"pose", asJson(g.at(current_.stamp))},
                      {"frozen_at_ns", g.motion->frozen_at == UINT64_MAX ? Json(nullptr) : Json(g.motion->frozen_at)}});
  resolved["obstacles_fixed"] = moving.empty();
  for (const auto &c : config_.imus)
    resolved["imus"].push_back({{"name", c.name}, {"link", c.link}, {"topic", c.topic}, {"rate_hz", c.rate},
      {"gravity_mps2", c.gravity}, {"accel_bias", vectorJson(c.accel_bias)}, {"gyro_bias", vectorJson(c.gyro_bias)},
      {"accel_sigma", vectorJson(c.accel_sigma)}, {"gyro_sigma", vectorJson(c.gyro_sigma)}});
  return {{"run_id", run_id},
          {"version", LWS_VERSION},
          {"compiler", __VERSION__},
          {"core_source_sha256", LWS_CORE_SHA256},
          {"urdfdom_version", LWS_URDFDOM_VERSION},
          {"config", resolved},
          {"resources", config_.resources},
          {"model", model_.report},
          {"seed", config_.seed},
          {"step_ns", config_.step_ns},
          {"initial_pose", asJson(origin_)},
          {"inputs", inputs_},
          {"status", diagnostics()},
          {"dynamic_entities", moving},
          {"exit_reason", reason},
          {"final_pose", asJson(current_.world_base)},
          {"final_odom_pose", asJson(current_.odom_base)},
          {"final_encoders", current_.joints}};
}
} // namespace litesim3d
