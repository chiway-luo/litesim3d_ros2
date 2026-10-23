#include "litesim3d/sim.hpp"
#include <Eigen/Eigenvalues>
#include <yaml-cpp/yaml.h>
#include <regex>

namespace litesim3d {
namespace {
void keys(const YAML::Node &n, std::initializer_list<const char *> allowed,
          const std::string &context) {
  require(n.IsMap(), context + ": expected mapping");
  std::set<std::string> seen;
  for (auto pair : n) {
    auto k = pair.first.as<std::string>();
    require(seen.insert(k).second, context + ": duplicate field " + k);
    bool ok = false;
    for (auto a : allowed)
      if (k == a)
        ok = true;
    require(ok, context + ": unknown field " + k);
  }
}
template <class T> T get(const YAML::Node &n, const char *key, T value) {
  return n[key] ? n[key].as<T>() : value;
}
Vec3 vector(const YAML::Node &n, Vec3 fallback = Vec3::Zero()) {
  if (!n)
    return fallback;
  require(n.IsSequence() && n.size() == 3, "expected three-component vector");
  Vec3 v(n[0].as<double>(), n[1].as<double>(), n[2].as<double>());
  require(v.allFinite(), "non-finite vector");
  return v;
}
Pose position(const YAML::Node &n) {
  if (!n)
    return Pose::Identity();
  keys(n, {"xyz", "rpy"}, "pose");
  return pose(vector(n["xyz"]), vector(n["rpy"]));
}
std::shared_ptr<Shape> shape(const YAML::Node &n, const std::filesystem::path &base,
                             Json &resources, const std::shared_ptr<LoadControl> &control, bool collision_role) {
  keys(n, {"type", "size", "radius", "length", "uri", "scale"}, "geometry");
  const auto type = n["type"].as<std::string>();
  if (type == "box")
    return Shape::box(vector(n["size"]));
  if (type == "sphere")
    return Shape::sphere(n["radius"].as<double>());
  if (type == "cylinder")
    return Shape::cylinder(n["radius"].as<double>(), n["length"].as<double>());
  require(type == "mesh", "unknown shape type: " + type);
  auto path = resolveResource(n["uri"].as<std::string>(), base);
  resources[path.string()] = sha256(path);
  return Shape::mesh(path, vector(n["scale"], Vec3::Ones()), control, collision_role);
}
void covariance(const YAML::Node &n, std::array<double, 36> &out) {
  if (!n)
    return;
  require(n.IsSequence() && n.size() == 36, "covariance must contain 36 values");
  for (size_t i = 0; i < 36; ++i)
    out[i] = n[i].as<double>();
}
void validateCovariance(const std::array<double, 36> &values) {
  Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> m(values.data());
  require(m.allFinite() && m.isApprox(m.transpose(), 1e-12),
          "covariance must be finite and symmetric");
  require(Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>>(m).eigenvalues().minCoeff() >=
              -1e-12,
          "covariance must be positive semidefinite");
}
} // namespace
void CameraConfig::validate() const {
  static const std::regex identifier("[A-Za-z][A-Za-z0-9_]*");
  static const std::regex relative_topic("[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*");
  require(std::regex_match(name, identifier) && !link.empty() && std::regex_match(topic, relative_topic),
          "camera: invalid name/link/relative topic");
  require(type == "rgb" || type == "depth" || type == "rgbd", "camera type must be rgb, depth or rgbd");
  require(width >= 16 && height >= 16 && width <= 4096 && height <= 4096 &&
          uint64_t(width) * height <= 8294400, "camera resolution exceeds 16..4096 / 8.29 MP limit");
  require(std::isfinite(rate) && rate > 0 && rate <= 240, "camera rate_hz must be in (0,240]");
  require(std::isfinite(horizontal_fov) && horizontal_fov > 0 && horizontal_fov < pi,
          "camera horizontal_fov_deg must be in (0,180)");
  require(std::isfinite(near_clip) && std::isfinite(far_clip) && near_clip > 0 && far_clip > near_clip,
          "camera requires 0 < near_clip_m < far_clip_m");
  require(color_bias.allFinite() && color_sigma.allFinite() && color_sigma.minCoeff() >= 0 &&
          std::isfinite(depth_bias) && std::isfinite(depth_sigma) && depth_sigma >= 0,
          "camera: invalid noise/bias");
  require(color() || (color_bias.isZero() && color_sigma.isZero()), "depth camera cannot have RGB noise");
  require(depth() || (depth_bias == 0 && depth_sigma == 0), "RGB camera cannot have depth noise");
}
Json CameraConfig::json() const {
  return {{"name",name},{"link",link},{"topic",topic},{"type",type},{"enabled",enabled},
          {"width",width},{"height",height},{"rate_hz",rate},
          {"horizontal_fov_deg",horizontal_fov*180/pi},{"near_clip_m",near_clip},{"far_clip_m",far_clip},
          {"color_bias_dn",{color_bias.x(),color_bias.y(),color_bias.z()}},
          {"color_sigma_dn",{color_sigma.x(),color_sigma.y(),color_sigma.z()}},
          {"depth_bias_m",depth_bias},{"depth_sigma_m",depth_sigma}};
}
void Drive::validate() const {
  require(model == "differential_drive" || model == "ackermann" || model == "multicopter" || model == "plugin",
          "unsupported motion_model");
  require(std::isfinite(wheelbase) && wheelbase > 0 && std::isfinite(max_steering) &&
          max_steering > 0 && max_steering < pi / 2, "invalid Ackermann wheelbase/steering");
  for (double v : {wheel_radius, wheel_separation, max_v, max_w, acceleration, deceleration,
                   angular_acceleration})
    require(std::isfinite(v) && v > 0,
            "drive: limits and wheel dimensions must be positive finite values");
  require(!left_joint.empty() && !right_joint.empty() && left_joint != right_joint,
          "drive: invalid wheel joints");
}
uint64_t Config::timeoutPhaseNs() const {
  return uint64_t(std::llround(timeout * 1e9)) % step_ns;
}
void Config::validate() const {
  require(step_ns > 0 && step_ns <= 100000000, "step_size: must be in (0, 0.1] seconds");
  require(std::isfinite(odom_rate) && odom_rate > 0 && odom_rate <= 1000, "odom_rate: invalid");
  require(std::isfinite(timeout) && timeout > 0, "command_timeout: invalid");
  require(std::isfinite(real_time_factor) && real_time_factor >= 0 && real_time_factor <= 100,
          "real_time_factor: invalid");
  require(initial_pose.matrix().allFinite() &&
              std::abs(initial_pose.linear().determinant() - 1) < 1e-9,
          "initial_pose: invalid");
  require(terrain.enabled || (std::abs(initial_pose.linear()(2, 0)) < 1e-9 &&
              std::abs(initial_pose.linear()(2, 1)) < 1e-9 && initial_pose.linear()(2, 2) > 0),
          "M1 requires a level initial_pose");
  require(!robot_id.empty() && !base_link.empty(), "robot id/base_link is empty");
  drive.validate();
  terrain.validate();
  odom_error.validate();
  if (drive.model == "multicopter") {
    require(!drive.continue_on_collision, "continue_on_collision requires a wheeled motion model");
    require(!terrain.enabled, "multicopter requires terrain.enabled=false");
    require(!odom_error.enabled, "multicopter currently requires ideal odometry");
    require(drive.support_links.empty(), "multicopter cannot have ground support links");
  }
  require(threads >= 1 && threads <= 4, "threads must be in [1,4]");
  require(history_limit >= 4 && history_limit <= 65536 && journal_limit >= 1 && journal_limit <= 65536,
          "invalid history/journal budget");
  require(imus.size() <= 16, "imu sensor budget exceeded");
  require(lidars.size() <= 16, "lidars: at most 16 sensors");
  std::set<std::string> names, topics;
  uint64_t point_budget = 0;
  for (auto &l : lidars) {
    l.validate();
    point_budget += uint64_t(l.columns) * l.vertical_angles.size() * (l.publish_range_boundary ? 2 : 1);
    require(point_budget <= 4194304, "aggregate sensor point budget exceeded");
    require(names.insert(l.name).second && topics.insert(l.topic).second,
            "duplicate lidar name/topic");
    if (l.publish_range_boundary)
      require(topics.insert(l.topic + "/range_boundary").second, "duplicate range boundary topic");
  }
  double schedule_rate = (timeoutPhaseNs() ? 2e9 : 1e9) / step_ns + odom_rate;
  for (const auto &i : imus) {
    i.validate();
    require(names.insert(i.name).second && topics.insert(i.topic).second, "duplicate sensor name/topic");
    schedule_rate += i.rate;
  }
  for (const auto &l : lidars) schedule_rate += l.rate;
  require(cameras.size() <= 8, "at most 8 cameras are supported");
  uint64_t camera_pixels = 0;
  for (const auto &c : cameras) {
    c.validate();
    require(names.insert(c.name).second, "duplicate sensor name");
    for (const auto &channel : {std::string("color"), std::string("depth")})
      if ((channel == "color" && c.color()) || (channel == "depth" && c.depth()))
        for (const auto &suffix : {"/image_raw", "/camera_info"})
          require(topics.insert(c.topic + "/" + channel + suffix).second, "duplicate camera output topic");
    if (c.enabled) { schedule_rate += c.rate; camera_pixels += uint64_t(c.width) * c.height; }
  }
  require(camera_pixels <= 16588800, "aggregate camera pixel allocation exceeds 16.59 MP");
  for (const auto &l : lidars) if (l.scan_mode == "rolling")
    require(std::ceil(schedule_rate / l.rate) + 4 <= history_limit,
            "rolling scan exceeds trajectory history budget");
  validateCovariance(pose_covariance);
  validateCovariance(twist_covariance);
}
Config Config::load(const std::filesystem::path &file, Scene &output, const std::shared_ptr<LoadControl> &provided) {
  try {
    auto control = provided ? provided : std::make_shared<LoadControl>();
    control->checkpoint(std::filesystem::file_size(file));
    const auto text = readText(file);
    const auto n = YAML::Load(text);
    keys(n, {"schema_version", "units", "simulation", "robot", "lidars", "entities", "odom", "terrain", "imus", "cameras", "scene_format"},
         "config");
    require(n["schema_version"].as<int>() == 1, "unsupported schema_version");
    require(n["units"].as<std::string>() == "m_rad_s", "units must be m_rad_s");
    Config cfg;
    cfg.model_options.load_control = control;
    Scene scene;
    const auto base = std::filesystem::absolute(file).parent_path();
    cfg.resources = Json::object();
    cfg.resources[std::filesystem::absolute(file).string()] = sha256(file);
    if (auto s = n["simulation"]) {
      keys(s, {"step_size", "seed", "odom_rate", "command_timeout", "real_time_factor", "threads", "history_limit", "journal_limit", "collision_budget_enabled"},
           "simulation");
      cfg.collision_budget_enabled = get(s, "collision_budget_enabled", false);
      control->collision_budget_enabled = cfg.collision_budget_enabled;
      double dt = get(s, "step_size", 0.005);
      require(std::isfinite(dt) && dt >= 1e-9 && dt <= 0.1, "invalid step_size");
      cfg.step_ns = std::llround(dt * 1e9);
      cfg.seed = get<uint64_t>(s, "seed", 42);
      cfg.odom_rate = get(s, "odom_rate", 50.0);
      cfg.timeout = get(s, "command_timeout", 0.5);
      cfg.real_time_factor = get(s, "real_time_factor", 1.0);
      cfg.threads = get<unsigned>(s, "threads", 1);
      cfg.history_limit = get<size_t>(s, "history_limit", 8192);
      cfg.journal_limit = get<size_t>(s, "journal_limit", 4096);
    }
    if (auto t = n["terrain"]) {
      keys(t, {"enabled", "max_slope_deg", "clearance_m", "continuity_m"}, "terrain");
      cfg.terrain.enabled = get(t, "enabled", true);
      cfg.terrain.max_slope = get(t, "max_slope_deg", 20.0) * pi / 180;
      cfg.terrain.clearance = get(t, "clearance_m", 0.0);
      cfg.terrain.continuity = get(t, "continuity_m", .02);
    }
    if (auto imus = n["imus"]) {
      require(imus.IsSequence() && imus.size() <= 16, "imus: require bounded sequence");
      for (auto i : imus) {
        keys(i, {"name", "link", "topic", "rate_hz", "accel_bias", "gyro_bias", "accel_sigma", "gyro_sigma",
                 "accel_range_mps2", "gyro_range_radps", "gravity_mps2"}, "imu");
        ImuConfig c;
        c.name = get<std::string>(i, "name", "imu"); c.link = get<std::string>(i, "link", "base_link");
        c.topic = get<std::string>(i, "topic", "imu/data_raw"); c.rate = get(i, "rate_hz", 200.0);
        c.gravity = get(i, "gravity_mps2", c.gravity);
        c.accel_bias = vector(i["accel_bias"]); c.gyro_bias = vector(i["gyro_bias"]);
        c.accel_sigma = vector(i["accel_sigma"]); c.gyro_sigma = vector(i["gyro_sigma"]);
        c.accel_range = get(i,"accel_range_mps2",0.0); c.gyro_range = get(i,"gyro_range_radps",0.0);
        cfg.imus.push_back(c);
      }
    }
    if (auto cameras = n["cameras"]) {
      require(cameras.IsSequence() && cameras.size() <= 8, "cameras: require sequence of at most 8 cameras");
      for (const auto &entry : cameras) {
        keys(entry, {"name","link","topic","type","enabled","width","height","rate_hz",
                     "horizontal_fov_deg","near_clip_m","far_clip_m",
                     "color_bias_dn","color_sigma_dn","depth_bias_m","depth_sigma_m"}, "camera");
        CameraConfig c;
        c.name = entry["name"].as<std::string>(); c.link = entry["link"].as<std::string>();
        c.topic = entry["topic"].as<std::string>(); c.type = get<std::string>(entry,"type",c.type);
        c.enabled = get(entry,"enabled",true); c.width = get<unsigned>(entry,"width",640);
        c.height = get<unsigned>(entry,"height",480); c.rate = get(entry,"rate_hz",15.0);
        c.horizontal_fov = get(entry,"horizontal_fov_deg",70.0) * pi / 180;
        c.near_clip = get(entry,"near_clip_m",.1); c.far_clip = get(entry,"far_clip_m",20.0);
        c.color_bias = vector(entry["color_bias_dn"]); c.color_sigma = vector(entry["color_sigma_dn"]);
        c.depth_bias = get(entry,"depth_bias_m",0.0); c.depth_sigma = get(entry,"depth_sigma_m",0.0);
        cfg.cameras.push_back(c);
      }
    }
    const auto r = n["robot"];
    keys(r,
         {"id", "urdf", "base_link", "initial_pose", "initial_joints", "drive",
          "visual_collision_fallback", "sensor_geometry", "sensor_overrides", "motion_model", "allow_visual_only",
          "continue_on_collision", "support_links"},
         "robot");
    cfg.robot_id = get<std::string>(r, "id", "robot_0");
    cfg.base_link = get<std::string>(r, "base_link", "base_link");
    cfg.drive.model = get<std::string>(r, "motion_model", "differential_drive");
    if (cfg.drive.model == "multicopter") cfg.drive.support_links.clear();
    cfg.urdf = resolveResource(r["urdf"].as<std::string>(), base);
    cfg.initial_pose = position(r["initial_pose"]);
    cfg.model_options.allow_visual_only = get(r, "allow_visual_only", false);
    cfg.model_options.visual_collision_fallback = get(r, "visual_collision_fallback", false);
    cfg.model_options.sensor_geometry = get<std::string>(r, "sensor_geometry", "visual");
    if (auto o = r["sensor_overrides"])
      for (auto p : o)
        cfg.model_options.sensor_overrides[p.first.as<std::string>()] = p.second.as<std::string>();
    if (auto q = r["initial_joints"])
      for (auto p : q)
        cfg.initial_joints[p.first.as<std::string>()] = p.second.as<double>();
    if (auto d = r["drive"]) {
      keys(d,
           {"wheel_radius", "wheel_separation", "max_v", "max_w", "acceleration", "deceleration",
            "angular_acceleration", "unlimited_acceleration", "left_joint", "right_joint",
            "support_links", "wheelbase", "max_steering_rad", "continue_on_collision"},
           "drive");
      cfg.drive.wheelbase = get(d, "wheelbase", 1.0);
      cfg.drive.max_steering = get(d, "max_steering_rad", 0.6);
      cfg.drive.wheel_radius = get(d, "wheel_radius", 0.1);
      cfg.drive.wheel_separation = get(d, "wheel_separation", 0.5);
      cfg.drive.max_v = get(d, "max_v", 1.0);
      cfg.drive.max_w = get(d, "max_w", 2.0);
      cfg.drive.acceleration = get(d, "acceleration", 0.5);
      cfg.drive.deceleration = get(d, "deceleration", 1.0);
      cfg.drive.angular_acceleration = get(d, "angular_acceleration", 2.0);
      cfg.drive.unlimited_acceleration = get(d, "unlimited_acceleration", false);
      cfg.drive.continue_on_collision = get(d, "continue_on_collision", false);
      cfg.drive.left_joint = get<std::string>(d, "left_joint", "left_wheel_joint");
      cfg.drive.right_joint = get<std::string>(d, "right_joint", "right_wheel_joint");
      if (d["support_links"]) {
        cfg.drive.support_links.clear();
        for (auto v : d["support_links"])
          cfg.drive.support_links.insert(v.as<std::string>());
      }
    }
    require(!n["lidars"] || n["lidars"].IsSequence(), "lidars must be a sequence");
    std::map<std::string, std::shared_ptr<const LidarPatternTable>> patterns;
    for (auto l : n["lidars"]) {
      keys(l,
           {"name", "link", "topic", "rate_hz", "columns", "vertical_angles_deg", "rings",
            "min_range_m", "max_range_m", "horizontal_start_deg", "horizontal_end_deg", "scan_mode",
            "point_layout", "self_filter", "excluded_links", "ring_delays_s", "range_bias_m", "range_sigma_m", "dropout",
            "scan_pattern", "vertical_min_deg", "vertical_max_deg", "cone_half_angle_deg", "pattern_hz",
            "publish_range_boundary", "pattern_file"},
           "lidar");
      LidarConfig c;
      c.name = l["name"].as<std::string>();
      c.link = l["link"].as<std::string>();
      c.topic = get<std::string>(l, "topic", "points_raw");
      c.rate = get(l, "rate_hz", 10.0);
      c.columns = get<uint32_t>(l, "columns", 1024);
      c.min_range = get(l, "min_range_m", 0.1);
      c.max_range = get(l, "max_range_m", 100.0);
      c.horizontal_start = get(l, "horizontal_start_deg", -180.0) * pi / 180;
      c.horizontal_end = get(l, "horizontal_end_deg", 180.0) * pi / 180;
      c.scan_pattern = get<std::string>(l, "scan_pattern", "spinning");
      if (l["vertical_angles_deg"]) {
        require(l["vertical_angles_deg"].IsSequence(), "vertical_angles_deg must be a sequence");
        for (auto a : l["vertical_angles_deg"])
          c.vertical_angles.push_back(a.as<double>() * pi / 180);
      } else {
        require(c.scan_pattern != "spinning", "lidar vertical_angles_deg required");
        c.vertical_angles = {0};
      }
      c.vertical_min = get(l, "vertical_min_deg", c.scan_pattern == "livox_mid360" ? -7.0 : -90.0) * pi / 180;
      c.vertical_max = get(l, "vertical_max_deg", c.scan_pattern == "livox_mid360" ? 52.0 : 90.0) * pi / 180;
      c.cone_half_angle = get(l, "cone_half_angle_deg", 19.2) * pi / 180;
      c.pattern_hz = get(l, "pattern_hz", 100.0);
      require(c.scan_pattern != "spinning" ||
                  (!l["vertical_min_deg"] && !l["vertical_max_deg"] &&
                   !l["cone_half_angle_deg"] && !l["pattern_hz"]),
              "spinning uses vertical_angles_deg; non-repeating fields are not applicable");
      require(c.scan_pattern != "spherical" || !l["cone_half_angle_deg"],
              "spherical uses horizontal and vertical FOV, not cone_half_angle_deg");
      if (c.scan_pattern == "livox_mid360") {
        require(l["pattern_file"] && !l["pattern_hz"] && !l["cone_half_angle_deg"],
                "Mid-360 requires pattern_file; pattern_hz/cone_half_angle_deg do not apply");
        const auto path = resolveResource(l["pattern_file"].as<std::string>(), base);
        const auto key = std::filesystem::canonical(path).string();
        if (!patterns.count(key)) patterns[key] = LidarPatternTable::load(path, control);
        c.pattern_table = patterns.at(key);
        cfg.resources[key] = c.pattern_table->source_sha256;
      } else require(!l["pattern_file"], "pattern_file only applies to livox_mid360");
      require(get<size_t>(l, "rings", c.vertical_angles.size()) == c.vertical_angles.size(),
              "lidar rings/angles mismatch");
      c.scan_mode = get<std::string>(l, "scan_mode", "snapshot");
      c.range_bias = get(l, "range_bias_m", 0.0); c.range_sigma = get(l, "range_sigma_m", 0.0);
      c.dropout = get(l, "dropout", 0.0);
      c.publish_range_boundary = get(l, "publish_range_boundary", false);
      if (l["ring_delays_s"]) {
        require(l["ring_delays_s"].IsSequence(), "ring_delays_s must be a sequence");
        for (auto delay : l["ring_delays_s"]) c.ring_delays.push_back(delay.as<double>());
      }
      require(get<std::string>(l, "point_layout", "xyzirt_v1") == "xyzirt_v1",
              "unsupported point_layout");
      c.filter = get<std::string>(l, "self_filter", "sensor_link");
      if (l["excluded_links"])
        for (auto v : l["excluded_links"])
          c.excluded_links.insert(v.as<std::string>());
      cfg.lidars.push_back(c);
    }
    const auto format = get<std::string>(n, "scene_format", "legacy");
    require(format == "legacy" || format == "role_entries", "invalid scene_format");
    const auto entities = n["entities"];
    require(entities.IsSequence(), "entities must be a sequence");
    std::set<std::string> ids;
    Json ignored_motion = Json::array();
    for (auto e : entities) {
      control->checkpoint();
      keys(e,
           {"id", "pose", "geometry", "visual", "collision", "sensor_geometry", "reflectivity",
            "support", "motion"},
           "entity");
      const auto id = e["id"].as<std::string>();
      require(!id.empty() && ids.insert(id).second, "duplicate/empty entity id: " + id);
      const Pose p = position(e["pose"]);
      double reflectivity = get(e, "reflectivity", 0.5);
      require(std::isfinite(reflectivity) && reflectivity >= 0 && reflectivity <= 1,
              id + ": invalid reflectivity");
      const bool support = get(e, "support", false);
      if (auto m = e["motion"]) {
        keys(m, {"linear_velocity", "yaw_rate"}, "entity.motion");
        const auto velocity = vector(m["linear_velocity"]);
        const auto yaw_rate = get(m, "yaw_rate", 0.0);
        require(velocity.norm() <= 100 && std::isfinite(yaw_rate) && std::abs(yaw_rate) <= 100,
                "invalid dynamic entity velocity");
        // Accept legacy input, but all configured scenery stays at its initial
        // pose. Record the ignored binding rather than creating a trajectory.
        ignored_motion.push_back(id);
      }
      auto add = [&](const YAML::Node &item, std::vector<Geometry> &dest, bool collision_role) {
        if (!item && format == "role_entries") return;
        require(bool(item), id + ": required geometry missing");
        dest.push_back({id, "", shape(item, base, cfg.resources, control, collision_role), p, reflectivity, support});
      };
      if (e["geometry"]) {
        require(!e["visual"] && !e["collision"] && !e["sensor_geometry"],
                id + ": geometry shorthand cannot mix with separate roles");
        add(e["geometry"], scene.visual, true);
        scene.collision.push_back(scene.visual.back());
        scene.sensor.push_back(scene.visual.back());
      } else {
        require(e["visual"] || e["collision"] || e["sensor_geometry"], id + ": empty geometry entry");
        add(e["visual"], scene.visual, false);
        add(format == "legacy" && !e["collision"] ? e["visual"] : e["collision"], scene.collision, true);
        add(e["sensor_geometry"], scene.sensor, false);
      }
      require(!support || e["collision"] || e["geometry"] || (format == "legacy" && e["visual"]),
              id + ": support requires collision geometry");
      if (support && !cfg.terrain.enabled) {
        const auto &g = scene.collision.back();
        require(g.shape->kind == Shape::Kind::Box &&
                    g.transform.linear().isApprox(Eigen::Matrix3d::Identity(), 1e-12) &&
                    std::abs(g.worldBounds().hi.z()) < 1e-9,
                id + ": M1 support must be an axis-aligned box with top z=0");
      }
    }
    if (auto o = n["odom"]) {
      keys(o, {"pose_covariance", "twist_covariance", "mode", "linear_scale", "angular_scale",
               "linear_scale_error", "angular_scale_error", "linear_bias", "angular_bias", "linear_sigma", "angular_sigma"}, "odom");
      const auto mode = get<std::string>(o, "mode", "ideal");
      require(mode == "ideal" || mode == "noisy", "invalid odom mode");
      cfg.odom_error.enabled = mode == "noisy";
      auto &e = cfg.odom_error;
      e.linear_scale = get(o, "linear_scale", 1.0); e.angular_scale = get(o, "angular_scale", 1.0);
      const double linear_error = get(o, "linear_scale_error", 0.0), angular_error = get(o, "angular_scale_error", 0.0);
      require(std::isfinite(linear_error) && linear_error > -1 && std::isfinite(angular_error) && angular_error > -1,
              "odom scale errors must be finite and greater than -1");
      require((linear_error == 0 || e.linear_scale == 1) && (angular_error == 0 || e.angular_scale == 1),
              "use odom scale_error or legacy scale, not both");
      e.linear_scale *= 1 + linear_error; e.angular_scale *= 1 + angular_error;
      e.linear_bias = get(o, "linear_bias", 0.0); e.angular_bias = get(o, "angular_bias", 0.0);
      e.linear_sigma = get(o, "linear_sigma", 0.0); e.angular_sigma = get(o, "angular_sigma", 0.0);
      require(e.enabled || (e.linear_scale == 1 && e.angular_scale == 1 && e.linear_bias == 0 && e.angular_bias == 0 && e.linear_sigma == 0 && e.angular_sigma == 0), "ideal odom cannot specify errors");
      covariance(o["pose_covariance"], cfg.pose_covariance);
      covariance(o["twist_covariance"], cfg.twist_covariance);
    }
    cfg.plugins = litesim3d::plugins::declarations(readText(cfg.urdf), cfg.urdf.parent_path(), resolveResource);
    bool plugin_chassis = false, plugin_odometry = false;
    for (const auto &plugin : cfg.plugins) {
      plugin_chassis |= plugin.category == "chassis";
      plugin_odometry |= plugin.category == "odometry";
      for (const auto &path : plugin.parameter_files) cfg.resources[path.string()] = sha256(path);
    }
    if (plugin_chassis) {
      require(!r["drive"] && !r["motion_model"], "URDF chassis plugin conflicts with legacy robot.drive/motion_model");
      cfg.drive.model = "plugin";
      cfg.drive.support_links.clear();
    }
    if (plugin_odometry) require(!n["odom"], "URDF odometry plugin conflicts with legacy YAML odom");
    require(plugin_chassis == plugin_odometry, "declare both chassis and odometry plugins in the composed model");
    if (r["continue_on_collision"]) {
      require(!r["drive"] || !r["drive"]["continue_on_collision"], "continue_on_collision is declared twice");
      cfg.drive.continue_on_collision = r["continue_on_collision"].as<bool>();
    }
    if (r["support_links"]) {
      require(!r["drive"] || !r["drive"]["support_links"], "support_links is declared twice");
      require(r["support_links"].IsSequence(), "robot.support_links must be a sequence");
      cfg.drive.support_links.clear();
      for (const auto &link : r["support_links"]) cfg.drive.support_links.insert(link.as<std::string>());
    }
    cfg.validate();
    cfg.resolved = {{"yaml", text},
                    {"step_ns", cfg.step_ns},
                    {"seed", cfg.seed},
                    {"odom_rate", cfg.odom_rate},
                    {"command_timeout", cfg.timeout},
                    {"real_time_factor", cfg.real_time_factor},
                    {"collision_budget_enabled", cfg.collision_budget_enabled},
                    {"obstacles_fixed", true},
                    {"ignored_entity_motion", ignored_motion},
                    {"point_layout", "xyzirt_v1"},
                    {"scan_mode", "per_sensor"},
                    {"odom_mode", cfg.odom_error.enabled ? "noisy" : "ideal"}};
    scene.rebuild();
    control->checkpoint();
    output = std::move(scene);
    return cfg;
  } catch (const std::exception &e) {
    throw std::runtime_error(file.string() + ": " + e.what());
  }
}
} // namespace litesim3d
