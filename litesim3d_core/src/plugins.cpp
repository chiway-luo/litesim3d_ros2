#include "litesim3d/sensor_plugin.hpp"
#include <algorithm>

namespace litesim3d {
void Simulator::initializeMotionPlugins() {
  plugins::Initialization init;
  init.seed = config_.seed;
  init.model_directory = config_.urdf.parent_path();
  init.model.base_link = config_.base_link;
  init.model.links = model_.links;
  const auto neutral = model_.fk({});
  const Pose base_inverse = neutral.at(config_.base_link).inverse();
  for (const auto &[name, joint] : model_.joints) {
    plugins::JointDescription item;
    item.type = joint.type; item.mimic = joint.mimic;
    const Pose transform = base_inverse * neutral.at(joint.parent) * joint.origin;
    item.position = transform.translation(); item.axis = transform.linear() * joint.axis;
    item.lower = joint.lower; item.upper = joint.upper; item.max_velocity = joint.velocity;
    for (const auto &[parent_name, parent] : model_.joints)
      if (parent.child == joint.parent) { item.parent_joint = parent_name; break; }
    init.model.joints[name] = item;
  }
  plugin_loader_ = std::make_unique<plugins::Loader>();
  std::set<std::string> names;
  for (auto &spec : config_.plugins) {
    require(!spec.name.empty() && names.insert(spec.name).second, "duplicate/empty plugin name: " + spec.name);
    init.instance = config_.robot_id + "/" + spec.name;
    try {
      if (spec.category == "chassis") {
        require(!chassis_plugin_, "multiple chassis plugins");
        chassis_plugin_ = plugin_loader_->chassis(spec.class_name);
        spec.parameters = plugins::parameters(chassis_plugin_->defaults(), spec.parameters);
        chassis_plugin_->initialize(init, spec.parameters);
        chassis_plugin_->reset({current_.joints, current_.joint_velocities});
      } else if (spec.category == "odometry") {
        require(!odometry_plugin_, "multiple odometry plugins");
        odometry_plugin_ = plugin_loader_->odometry(spec.class_name);
        spec.parameters = plugins::parameters(odometry_plugin_->defaults(), spec.parameters);
        odometry_plugin_->initialize(init, spec.parameters);
        odometry_plugin_->reset({current_.joints, current_.joint_velocities});
      } else require(spec.category == "sensor", "unknown plugin category: " + spec.category);
    } catch (const std::exception &error) {
      throw std::runtime_error("plugin '" + spec.name + "' (" + spec.class_name + "): " + error.what());
    }
  }
  require(bool(chassis_plugin_) == bool(odometry_plugin_), "chassis and odometry plugins must be declared together");
  if (config_.control_mode == "ros2_control") {
    joint_chassis_ = dynamic_cast<plugins::JointDrivenChassis *>(chassis_plugin_.get());
    body_control_ = !chassis_plugin_ && config_.drive.model == "multicopter";
    require(joint_chassis_ || body_control_, "ros2_control requires JointDrivenChassis or the kinematic multicopter");
    if (joint_chassis_) {
      driven_joints_ = joint_chassis_->drivenJoints();
      require(!driven_joints_.empty(), "joint-driven chassis returned no drive joints");
      for (const auto &name : driven_joints_)
        require(model_.joints.count(name) && model_.joints.at(name).type == "continuous" &&
                model_.joints.at(name).mimic.empty(), "invalid joint-driven wheel: " + name);
      passive_joints_ = joint_chassis_->passiveJoints();
      for (const auto &name : passive_joints_)
        require(model_.joints.count(name) && model_.joints.at(name).type == "continuous" &&
                model_.joints.at(name).mimic.empty() && !driven_joints_.count(name),
                "invalid passive wheel: " + name);
      joint_position_limits_ = joint_chassis_->positionLimits();
      for (auto &[name, bounds] : joint_position_limits_) {
        require(model_.joints.count(name) && model_.joints.at(name).type == "revolute" &&
                model_.joints.at(name).mimic.empty() && std::isfinite(bounds.first) &&
                std::isfinite(bounds.second), "invalid joint position limits: " + name);
        bounds.first = std::max(bounds.first, model_.joints.at(name).lower);
        bounds.second = std::min(bounds.second, model_.joints.at(name).upper);
        require(bounds.first <= current_.joints.at(name) && current_.joints.at(name) <= bounds.second,
                "initial position outside chassis joint limits: " + name);
      }
    }
    for (const auto &[name, joint] : model_.joints)
      if (joint.type != "fixed" && joint.mimic.empty())
        require(std::isfinite(joint.velocity) && joint.velocity > 0,
                "ros2_control requires a finite positive URDF velocity limit: " + name);
  }
  for (const auto &link : config_.drive.support_links)
    require(model_.links.count(link), "unknown support link: " + link);
  initializeSensorPlugins(init);
}

void Simulator::initializeSensorPlugins(const plugins::Initialization &model_init) {
  if (!config_.sensors_resolved) {
    const bool explicit_sensors = std::any_of(config_.plugins.begin(), config_.plugins.end(),
        [](const auto &p) { return p.category == "sensor"; });
    require(!explicit_sensors || (config_.lidars.empty() && config_.imus.empty() && config_.cameras.empty()),
            "URDF sensor plugins conflict with legacy YAML lidars/imus/cameras; migrate the complete sensor set");
    auto add = [&](const auto &c, const std::string &class_name) {
      config_.plugins.push_back({"sensor",c.name,class_name,plugins::sensorParameters(c),{}});
    };
    for (const auto &c : config_.lidars) add(c,"litesim3d_lidar/Lidar");
    for (const auto &c : config_.imus) add(c,"litesim3d_imu/Imu");
    for (const auto &c : config_.cameras) {
      const std::string cls = c.type == "rgb" ? "RgbCamera" : c.type == "depth" ? "DepthCamera" : "RgbdCamera";
      add(c,"litesim3d_" + c.type + "_camera/" + cls);
    }
  }
  config_.lidars.clear(); config_.imus.clear(); config_.cameras.clear();
  std::set<std::string> names;
  for (auto &spec : config_.plugins) {
    require(!spec.name.empty() && names.insert(spec.name).second,"duplicate/empty plugin name: " + spec.name);
    if (spec.category != "sensor") continue;
    try {
      auto plugin = plugin_loader_->sensor(spec.class_name);
      spec.parameters = plugins::parameters(plugin->defaults(),spec.parameters);
      auto init = model_init;
      init.instance = spec.name; init.seed = streamSeed(config_.seed,config_.robot_id);
      plugin->initialize(init,spec.parameters);
      const auto desc = plugin->description();
      require(desc.name == spec.name,"sensor descriptor name differs from instance");
      plugins::validateSensorBinding(init,desc.link);
      require(std::isfinite(desc.rate_hz) && desc.rate_hz > 0 && desc.rate_hz <= 2000,
              "invalid sensor descriptor rate");
      require(desc.settings.at("link") == desc.link && desc.settings.at("topic") == desc.topic &&
          desc.settings.at("rate_hz") == desc.rate_hz && desc.settings.at("enabled") == desc.enabled,
          "inconsistent sensor descriptor settings");
      if (desc.kind == "lidar") {
        auto cfg = plugins::lidarSettings(desc.name,desc.settings,init.model_directory);
        for (const auto &link : cfg.excluded_links) plugins::validateSensorBinding(init,link);
        if (cfg.pattern_table) config_.resources[cfg.pattern_table->source] = cfg.pattern_table->source_sha256;
        if (desc.enabled) config_.lidars.push_back(cfg);
      } else if (desc.kind == "imu") {
        auto cfg = plugins::imuSettings(desc.name,desc.settings);
        if (desc.enabled) config_.imus.push_back(cfg);
      } else {
        config_.cameras.push_back(plugins::cameraSettings(desc.name,desc.kind,desc.settings));
      }
      plugin->reset(1);
      sensor_plugins_[spec.name] = std::move(plugin);
      sensor_descriptions_[spec.name] = desc;
    } catch (const std::exception &error) {
      throw std::runtime_error("sensor plugin '" + spec.name + "' (" + spec.class_name + "): " + error.what());
    }
  }
  config_.sensors_resolved = true;
  config_.validate();
}

plugins::SensorOutput Simulator::sampleSensor(const std::string &name, uint64_t stamp,
    uint64_t sample_index, const Snapshot &snapshot) const {
  const auto &desc = sensor_descriptions_.at(name);
  const auto frames = model_.fk(snapshot.joints);
  const Pose mount = frames.at(config_.base_link).inverse() * frames.at(desc.link);
  auto body = kinematics_; body.world_body = snapshot.world_base;
  plugins::SensorContext context{scene_,body,mount,
      [this,mount](uint64_t t) { return trajectory(t) * mount; },{},
      stamp,snapshot.stamp,sample_index,config_.threads};
  if (desc.kind == "lidar")
    context.robot_in_sensor = model_.placed(model_.sensor,mount.inverse(),config_.base_link,snapshot.joints);
  plugins::SensorOutput output;
  auto &plugin = *sensor_plugins_.at(name);
  if (auto *articulated = dynamic_cast<plugins::LinkKinematicsSensor *>(&plugin))
    articulated->stepWithKinematics(context, sensorKinematics(snapshot, frames, desc.link), output);
  else plugin.step(context,output);
  return output;
}

Kinematics Simulator::sensorKinematics(const Snapshot &snapshot,
    const std::map<std::string, Pose> &frames, const std::string &link) const {
  // base_link is fixed to the URDF root, which may have a different origin.
  const Pose world_root = snapshot.world_base * frames.at(config_.base_link).inverse();
  Kinematics result = kinematics_;
  const Vec3 root_offset = world_root.translation() - snapshot.world_base.translation();
  result.acceleration += result.alpha.cross(root_offset) +
      result.omega.cross(result.omega.cross(root_offset));
  std::vector<const Joint *> chain;
  for (std::string child = link; child != model_.root;) {
    const auto it = std::find_if(model_.joints.begin(), model_.joints.end(),
        [&](const auto &entry) { return entry.second.child == child; });
    require(it != model_.joints.end(), "invalid sensor joint chain: " + child);
    chain.push_back(&it->second); child = it->second.parent;
  }
  auto rate = [](const auto &values, const std::string &name) {
    const auto it = values.find(name); return it == values.end() ? 0. : it->second;
  };
  for (auto it = chain.rbegin(); it != chain.rend(); ++it) {
    const auto &joint = **it;
    const Pose parent = world_root * frames.at(joint.parent);
    const Pose child = world_root * frames.at(joint.child);
    const Vec3 offset = child.translation() - parent.translation();
    result.acceleration += result.alpha.cross(offset) +
        result.omega.cross(result.omega.cross(offset));
    if (joint.type == "fixed") continue;
    const Vec3 axis = parent.linear() * joint.origin.linear() * joint.axis;
    const Vec3 velocity = axis * rate(snapshot.joint_velocities, joint.name);
    const Vec3 acceleration = axis * rate(joint_accelerations_, joint.name);
    if (joint.type == "prismatic") {
      result.acceleration += acceleration + 2 * result.omega.cross(velocity);
    } else {
      result.alpha += acceleration + result.omega.cross(velocity);
      result.omega += velocity;
    }
  }
  result.world_body = world_root * frames.at(link);
  return result;
}

void Simulator::processCamera(size_t index, plugins::ImageFrame &image) {
  const auto &cfg = config_.cameras.at(index);
  require(cfg.enabled,"cannot process disabled camera");
  sensor_plugins_.at(cfg.name)->processImage(image);
}

void Simulator::moveSteering(const std::map<std::string, double> &targets) {
  if (targets.empty()) return;
  const auto initial = current_.joints;
  double angular_distance = 0, linear_distance = 0, reach = 0, geometry_reach = 0;
  for (const auto &[name, joint] : model_.joints) reach += joint.origin.translation().norm();
  for (const auto &g : model_.collision)
    geometry_reach = std::max(geometry_reach, g.transform.translation().norm() +
        g.shape->bounds.lo.cwiseAbs().cwiseMax(g.shape->bounds.hi.cwiseAbs()).norm());
  reach += geometry_reach;
  for (const auto &[name, value] : targets) {
    require(model_.joints.count(name), "plugin returned unknown steering joint: " + name);
    const auto &joint = model_.joints.at(name);
    require((joint.type == "revolute" || (config_.control_mode == "ros2_control" &&
            (joint.type == "continuous" || joint.type == "prismatic"))) && joint.mimic.empty(),
            "invalid independently controlled joint: " + name);
    require(std::isfinite(value) && value >= joint.lower && value <= joint.upper, "plugin steering target exceeds joint limits: " + name);
    if (joint.type == "prismatic") linear_distance += std::abs(value - initial.at(name));
    else angular_distance += std::abs(value - initial.at(name));
  }
  if (angular_distance + linear_distance < 1e-14) return;
  // Include prismatic reach and mimic amplification in the conservative sweep.
  double amplification = 1;
  for (const auto &[name, joint] : model_.joints) {
    if (joint.type == "prismatic") reach += std::max(std::abs(joint.lower), std::abs(joint.upper));
    if (!joint.mimic.empty()) amplification *= 1 + std::abs(joint.multiplier);
  }
  const double speed_bound = (2 * reach * angular_distance + linear_distance) * amplification;
  auto sample = [&](double fraction) {
    auto q = initial;
    for (const auto &[name, value] : targets) q[name] += fraction * (value - initial.at(name));
    return model_.positions(q);
  };
  // Only these subtrees move during this sweep. An unrelated chassis contact
  // must not stop a free joint or prevent the subsequent body escape motion.
  // Resolve mimic joints first, then include all fixed/movable descendants.
  const auto final = sample(1);
  std::set<std::string> moving_links;
  std::function<void(const std::string &, bool)> descend = [&](const std::string &parent, bool moving) {
    for (const auto &[name, joint] : model_.joints) if (joint.parent == parent) {
      const bool child_moves = moving || final.at(name) != initial.at(name);
      if (child_moves) moving_links.insert(joint.child);
      descend(joint.child, child_moves);
    }
  };
  descend(model_.root, false);
  double fraction = 0;
  for (size_t iteration = 0; fraction < 1; ++iteration) {
    auto q = sample(fraction);
    double distance;
    auto contact = clearance(current_.world_base, distance, &q, &moving_links);
    if (distance > 1e-9 && distance <= 2e-5 && speed_bound > 0 && iteration < 10000) {
      const double increment = std::min(1 - fraction, distance * .5 / speed_bound);
      const auto probe = sample(fraction + increment); double next_distance;
      clearance(current_.world_base, next_distance, &probe, &moving_links);
      if (next_distance > distance + 1e-10) { current_.joints = probe; fraction += increment; continue; }
    }
    if (distance <= 1e-5 || iteration >= 10000) {
      if (contact) stopForStep(contact->entity, contact->robot_link, distance);
      else forced_stop_ = true;
      return;
    }
    current_.joints = q;
    const double increment = std::isfinite(distance) && speed_bound > 0
        ? std::min(1 - fraction, (distance - 1e-5) * .9 / speed_bound) : 1 - fraction;
    if (increment < 1e-12) {
      if (contact) stopForStep(contact->entity, contact->robot_link, distance);
      else forced_stop_ = true;
      return;
    }
    fraction += increment;
  }
  current_.joints = sample(1);
}
} // namespace litesim3d
