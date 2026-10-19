#include "litesim3d/sensor_plugin.hpp"
#include <algorithm>
#include <cstring>
#include <mutex>
#include <regex>

namespace litesim3d::plugins {
namespace {
Vec3 vector(const Json &j, const std::string &key) {
  const auto &a = j.at(key);
  require(a.is_array() && a.size() == 3, key + ": expected three finite numbers");
  Vec3 v(a.at(0).get<double>(), a.at(1).get<double>(), a.at(2).get<double>());
  require(v.allFinite(), key + ": non-finite vector");
  return v;
}
Json array(const Vec3 &v) { return {v.x(), v.y(), v.z()}; }
unsigned count(const Json &j, const std::string &key, unsigned maximum) {
  const double value = j.at(key).get<double>();
  require(std::isfinite(value) && value >= 1 && value <= maximum && std::floor(value) == value,
          key + ": expected bounded positive integer");
  return unsigned(value);
}
std::shared_ptr<const LidarPatternTable> pattern(const std::filesystem::path &file) {
  static std::mutex mutex;
  static std::map<std::string, std::weak_ptr<const LidarPatternTable>> cache;
  const auto key = std::filesystem::canonical(file).string();
  const auto hash = sha256(file);
  std::lock_guard<std::mutex> lock(mutex);
  if (auto value = cache[key].lock(); value && value->source_sha256 == hash) return value;
  auto value = LidarPatternTable::load(file);
  cache[key] = value;
  return value;
}
}
Json sensorDefaults(const std::string &kind) {
  Json p = {{"link",nullptr},{"topic",nullptr},{"enabled",true}};
  if (kind == "lidar") return merge(p, {
    {"rate_hz",10.0},{"columns",1024},{"vertical_angles_deg",{0.0}},
    {"min_range_m",.1},{"max_range_m",100.0},{"horizontal_start_deg",-180.0},{"horizontal_end_deg",180.0},
    {"scan_mode","snapshot"},{"scan_pattern","spinning"},{"vertical_min_deg",-90.0},{"vertical_max_deg",90.0},
    {"cone_half_angle_deg",19.2},{"pattern_hz",100.0},{"pattern_file",""},{"ring_delays_s",Json::array()},
    {"self_filter","sensor_link"},{"excluded_links",Json::array()},
    {"range_bias_m",0.0},{"range_sigma_m",0.0},{"dropout",0.0},{"publish_range_boundary",false}});
  if (kind == "imu") return merge(p, {
    {"rate_hz",200.0},{"accel_bias",{0.,0.,0.}},{"gyro_bias",{0.,0.,0.}},
    {"accel_sigma",{0.,0.,0.}},{"gyro_sigma",{0.,0.,0.}},
    {"accel_range_mps2",0.0},{"gyro_range_radps",0.0}});
  require(kind == "rgb" || kind == "depth" || kind == "rgbd", "unknown sensor output kind: " + kind);
  p = merge(p, {{"width",640},{"height",480},{"rate_hz",15.0},{"horizontal_fov_deg",70.0},
                {"near_clip_m",.1},{"far_clip_m",20.0}});
  if (kind != "depth") p = merge(p, {{"color_bias_dn",{0.,0.,0.}},{"color_sigma_dn",{0.,0.,0.}}});
  if (kind != "rgb") p = merge(p, {{"depth_bias_m",0.0},{"depth_sigma_m",0.0}});
  return p;
}
LidarConfig lidarSettings(const std::string &name, const Json &input, const std::filesystem::path &base) {
  const auto p = parameters(sensorDefaults("lidar"), input);
  LidarConfig c; c.name = name; c.link = p.at("link"); c.topic = p.at("topic");
  c.rate = p.at("rate_hz"); c.columns = count(p,"columns",65536);
  for (const auto &a : p.at("vertical_angles_deg")) c.vertical_angles.push_back(a.get<double>() * pi / 180);
  c.min_range = p.at("min_range_m"); c.max_range = p.at("max_range_m");
  c.horizontal_start = p.at("horizontal_start_deg").get<double>() * pi / 180;
  c.horizontal_end = p.at("horizontal_end_deg").get<double>() * pi / 180;
  c.scan_mode = p.at("scan_mode"); c.scan_pattern = p.at("scan_pattern");
  c.vertical_min = p.at("vertical_min_deg").get<double>() * pi / 180;
  c.vertical_max = p.at("vertical_max_deg").get<double>() * pi / 180;
  c.cone_half_angle = p.at("cone_half_angle_deg").get<double>() * pi / 180;
  c.pattern_hz = p.at("pattern_hz");
  const auto file = p.at("pattern_file").get<std::string>();
  if (!file.empty()) c.pattern_table = pattern(resolveResource(file, base));
  c.ring_delays = p.at("ring_delays_s").get<std::vector<double>>();
  c.filter = p.at("self_filter"); c.excluded_links = p.at("excluded_links").get<std::set<std::string>>();
  c.range_bias = p.at("range_bias_m"); c.range_sigma = p.at("range_sigma_m"); c.dropout = p.at("dropout");
  c.publish_range_boundary = p.at("publish_range_boundary"); c.validate(); return c;
}
ImuConfig imuSettings(const std::string &name, const Json &input) {
  const auto p = parameters(sensorDefaults("imu"), input);
  ImuConfig c; c.name = name; c.link = p.at("link"); c.topic = p.at("topic"); c.rate = p.at("rate_hz");
  c.accel_bias = vector(p,"accel_bias"); c.gyro_bias = vector(p,"gyro_bias");
  c.accel_sigma = vector(p,"accel_sigma"); c.gyro_sigma = vector(p,"gyro_sigma");
  c.accel_range = p.at("accel_range_mps2"); c.gyro_range = p.at("gyro_range_radps");
  c.validate(); return c;
}
CameraConfig cameraSettings(const std::string &name, const std::string &kind, const Json &input) {
  const auto p = parameters(sensorDefaults(kind), input);
  CameraConfig c; c.name = name; c.type = kind; c.link = p.at("link"); c.topic = p.at("topic");
  c.enabled = p.at("enabled"); c.width = count(p,"width",4096); c.height = count(p,"height",4096);
  c.rate = p.at("rate_hz"); c.horizontal_fov = p.at("horizontal_fov_deg").get<double>() * pi / 180;
  c.near_clip = p.at("near_clip_m"); c.far_clip = p.at("far_clip_m");
  if (c.color()) { c.color_bias = vector(p,"color_bias_dn"); c.color_sigma = vector(p,"color_sigma_dn"); }
  if (c.depth()) { c.depth_bias = p.at("depth_bias_m"); c.depth_sigma = p.at("depth_sigma_m"); }
  c.validate(); return c;
}
Json sensorParameters(const LidarConfig &c) {
  Json angles = Json::array(); for (const auto a : c.vertical_angles) angles.push_back(a * 180 / pi);
  return {{"link",c.link},{"topic",c.topic},{"enabled",true},{"rate_hz",c.rate},{"columns",c.columns},
    {"vertical_angles_deg",angles},{"min_range_m",c.min_range},{"max_range_m",c.max_range},
    {"horizontal_start_deg",c.horizontal_start*180/pi},{"horizontal_end_deg",c.horizontal_end*180/pi},
    {"scan_mode",c.scan_mode},{"scan_pattern",c.scan_pattern},{"vertical_min_deg",c.vertical_min*180/pi},
    {"vertical_max_deg",c.vertical_max*180/pi},{"cone_half_angle_deg",c.cone_half_angle*180/pi},
    {"pattern_hz",c.pattern_hz},{"pattern_file",c.pattern_table ? c.pattern_table->source : ""},
    {"ring_delays_s",c.ring_delays},{"self_filter",c.filter},{"excluded_links",c.excluded_links},
    {"range_bias_m",c.range_bias},{"range_sigma_m",c.range_sigma},{"dropout",c.dropout},
    {"publish_range_boundary",c.publish_range_boundary}};
}
Json sensorParameters(const ImuConfig &c) {
  return {{"link",c.link},{"topic",c.topic},{"enabled",true},{"rate_hz",c.rate},
    {"accel_bias",array(c.accel_bias)},{"gyro_bias",array(c.gyro_bias)},
    {"accel_sigma",array(c.accel_sigma)},{"gyro_sigma",array(c.gyro_sigma)},
    {"accel_range_mps2",c.accel_range},{"gyro_range_radps",c.gyro_range}};
}
Json sensorParameters(const CameraConfig &c) {
  Json p = {{"link",c.link},{"topic",c.topic},{"enabled",c.enabled},{"rate_hz",c.rate},
    {"width",c.width},{"height",c.height},{"horizontal_fov_deg",c.horizontal_fov*180/pi},
    {"near_clip_m",c.near_clip},{"far_clip_m",c.far_clip}};
  if (c.color()) p = merge(p, {{"color_bias_dn",array(c.color_bias)},{"color_sigma_dn",array(c.color_sigma)}});
  if (c.depth()) p = merge(p, {{"depth_bias_m",c.depth_bias},{"depth_sigma_m",c.depth_sigma}});
  return p;
}
void validateSensorBinding(const Initialization &init, const std::string &link) {
  require(!init.instance.empty(), "empty sensor instance");
  require(init.model.links.count(link), "sensor binding: unknown link " + link);
}
void applyCameraNoise(const CameraConfig &c, uint64_t seed, ImageFrame &f) {
  require(f.width == c.width && f.height == c.height, "camera image resolution mismatch");
  const size_t pixels = size_t(f.width) * f.height;
  require(f.color.size() == (c.color() ? pixels * 3 : 0) &&
          f.depth.size() == (c.depth() ? pixels * 4 : 0), "invalid rendered image buffers");
  const auto key = streamSeed(streamSeed(seed,c.name), std::to_string(f.stamp_ns));
  if (c.color()) for (size_t i = 0; i < f.color.size(); ++i) {
    const auto channel = i % 3;
    const double value = f.color[i] + c.color_bias[channel] + c.color_sigma[channel] * randomNormal(key,i,0);
    f.color[i] = uint8_t(std::lround(std::clamp(value,0.0,255.0)));
  }
  if (c.depth()) for (size_t i = 0; i < pixels; ++i) {
    auto *p = f.depth.data() + i * 4;
    uint32_t bits = 0;
    for (int b = 0; b < 4; ++b) bits |= uint32_t(p[b]) << (8 * (f.depth_big_endian ? 3-b : b));
    float value; std::memcpy(&value,&bits,4);
    if (std::isfinite(value)) {
      value += c.depth_bias + c.depth_sigma * randomNormal(key,i,1);
      if (value < c.near_clip || value > c.far_clip) value = std::numeric_limits<float>::quiet_NaN();
      std::memcpy(&bits,&value,4);
      for (int b = 0; b < 4; ++b) p[b] = uint8_t(bits >> (8 * (f.depth_big_endian ? 3-b : b)));
    }
  }
}
} // namespace litesim3d::plugins
