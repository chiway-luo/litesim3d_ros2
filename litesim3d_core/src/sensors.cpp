#include "litesim3d/sim.hpp"
#include <chrono>
#include <cstring>
#include <thread>

namespace litesim3d {
namespace {
uint64_t mix(uint64_t x) {
  x += 0x9e3779b97f4a7c15ULL;
  x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ULL;
  x = (x ^ (x >> 27)) * 0x94d049bb133111ebULL;
  return x ^ (x >> 31);
}
void putFloat(std::vector<uint8_t> &data, size_t offset, float value) {
  uint32_t word;
  std::memcpy(&word, &value, 4);
  for (int b = 0; b < 4; ++b) data[offset + b] = (word >> (8 * b)) & 255;
}
} // namespace
double randomUniform(uint64_t key, uint64_t sample, uint64_t lane) {
  // Open interval (0,1), including on all-zero/all-one integer output.
  return (double(mix(key ^ mix(sample) ^ mix(lane)) >> 12) + .5) / 4503599627370496.0;
}
double randomNormal(uint64_t key, uint64_t sample, uint64_t lane) {
  return std::sqrt(-2 * std::log(randomUniform(key, sample, lane * 2))) *
         std::cos(2 * pi * randomUniform(key, sample, lane * 2 + 1));
}
uint64_t LidarConfig::offsetNs(size_t ring, size_t column) const {
  if (scan_mode == "snapshot") return 0;
  return uint64_t(std::llround((static_cast<long double>(column) / (rate * columns) +
                                (ring_delays.empty() ? 0 : ring_delays.at(ring))) * 1e9L));
}
Cloud scanTrajectory(const Scene &scene, const LidarConfig &cfg, uint64_t start,
                     const std::function<Pose(uint64_t)> &world_sensor,
                     const std::vector<Geometry> &robot, uint64_t seed, uint64_t frame,
                     unsigned threads) {
  const auto begin = std::chrono::steady_clock::now();
  cfg.validate();
  require(threads >= 1 && threads <= 4, "ray worker budget must be in [1,4]");
  Cloud c;
  c.stamp = start;
  c.completion_stamp = start;
  c.frame = cfg.link;
  c.topic = cfg.topic;
  c.height = uint32_t(cfg.vertical_angles.size());
  c.width = cfg.columns;
  c.row_step = c.width * 24;
  c.data.resize(size_t(c.height) * c.row_step, 0);
  if (cfg.publish_range_boundary) c.range_boundary.resize(c.data.size(), 0);
  for (size_t r = 0; r < c.height; ++r)
    c.completion_stamp = std::max(c.completion_stamp, start + cfg.offsetNs(r, c.width - 1));
  auto excluded = cfg.excluded_links;
  if (cfg.filter == "sensor_link") excluded.insert(cfg.link);
  const uint64_t range_key = streamSeed(seed, cfg.name + "/range"),
                 drop_key = streamSeed(seed, cfg.name + "/dropout");
  std::array<uint8_t, 4> dense{1, 1, 1, 1};
  std::array<std::exception_ptr, 4> errors{};
  auto work = [&](unsigned worker) {
    try {
      for (size_t slot = worker; slot < size_t(c.height) * c.width; slot += threads) {
        const size_t r = slot / c.width, col = slot % c.width, off = slot * 24;
        const Vec3 direction = cfg.direction(r, col, frame);
        const uint64_t offset = cfg.offsetNs(r, col);
        const bool emitted = direction.allFinite();
        std::optional<Hit> hit;
        if (emitted) {
          const Pose world = world_sensor(start + offset);
          hit = scene.ray(world.translation(), world.linear() * direction, cfg.max_range, {}, {}, start + offset);
        }
        // Robot geometry is queried in sensor coordinates; no per-ray vector allocation.
        if (emitted && cfg.filter != "robot")
          for (const auto &g : robot) if (!excluded.count(g.link)) {
            const Pose inverse = g.transform.inverse();
            const double distance = g.shape->ray(inverse.translation(), inverse.linear() * direction,
                                                 hit ? hit->distance : cfg.max_range);
            if (std::isfinite(distance) && (!hit || distance < hit->distance))
              hit = Hit{distance, g.entity, g.link, g.reflectivity};
          }
        Vec3 point = Vec3::Constant(NAN);
        float intensity = 0;
        const uint64_t counter = mix(frame) ^ uint64_t(slot);
        if (hit && hit->distance + 1e-12 * std::max(1.0, cfg.min_range) >= cfg.min_range &&
            (cfg.dropout == 0 || randomUniform(drop_key, counter) >= cfg.dropout)) {
          const double distance = hit->distance + cfg.range_bias +
                                  (cfg.range_sigma == 0 ? 0 : cfg.range_sigma * randomNormal(range_key, counter));
          if (distance + 1e-12 * std::max(1.0, cfg.min_range) >= cfg.min_range &&
              distance <= cfg.max_range + 1e-12 * std::max(1.0, cfg.max_range)) {
            point = direction * distance;
            intensity = float(hit->reflectivity); // declared constant-return model
          }
        }
        if (!point.allFinite()) dense[worker] = 0;
        for (int j = 0; j < 3; ++j) putFloat(c.data, off + j * 4, float(point[j]));
        putFloat(c.data, off + 12, intensity);
        c.data[off + 16] = r & 255;
        c.data[off + 17] = (r >> 8) & 255;
        putFloat(c.data, off + 20, float(offset * 1e-9));
        if (cfg.publish_range_boundary) {
          // Missing geometry within range is different from a blocked/dropped return.
          // This visualization-only cloud must not fabricate free space behind an occluder.
          const Vec3 boundary = emitted && !hit ? Vec3(direction * cfg.max_range) : Vec3::Constant(NAN);
          for (int j = 0; j < 3; ++j)
            putFloat(c.range_boundary, off + j * 4, float(boundary[j]));
          c.range_boundary[off + 16] = r & 255;
          c.range_boundary[off + 17] = (r >> 8) & 255;
          putFloat(c.range_boundary, off + 20, float(offset * 1e-9));
        }
      }
    } catch (...) { errors[worker] = std::current_exception(); }
  };
  std::vector<std::thread> workers;
  try {
    for (unsigned i = 1; i < threads; ++i) workers.emplace_back(work, i);
  } catch (...) {
    for (auto &worker : workers) worker.join();
    throw;
  }
  work(0);
  for (auto &worker : workers) worker.join();
  for (unsigned i = 0; i < threads; ++i) {
    if (errors[i]) std::rethrow_exception(errors[i]);
    c.dense = c.dense && dense[i];
  }
  c.compute_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count();
  c.compute_end_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  return c;
}
void ImuConfig::validate() const {
  require(!name.empty() && !link.empty() && !topic.empty(), "imu: empty name/link/topic");
  require(std::isfinite(rate) && rate > 0 && rate <= 2000, "imu: invalid rate");
  require(accel_bias.allFinite() && gyro_bias.allFinite() && accel_sigma.allFinite() &&
              gyro_sigma.allFinite() && accel_sigma.minCoeff() >= 0 && gyro_sigma.minCoeff() >= 0,
          "imu: invalid noise/bias");
}
ImuSample sampleImu(const ImuConfig &cfg, const Kinematics &body, const Pose &mount,
                    uint64_t stamp, uint64_t seed, uint64_t sample) {
  cfg.validate();
  require(body.world_body.matrix().allFinite() && body.acceleration.allFinite() &&
              body.omega.allFinite() && body.alpha.allFinite() && mount.matrix().allFinite(),
          "imu: non-finite kinematics");
  ImuSample result;
  result.stamp = stamp; result.frame = cfg.link; result.topic = cfg.topic;
  const Vec3 offset = body.world_body.linear() * mount.translation();
  const auto rotation = (body.world_body.linear() * mount.linear()).transpose().eval();
  result.acceleration = rotation * (body.acceleration + body.alpha.cross(offset) +
      body.omega.cross(body.omega.cross(offset)) - Vec3(0, 0, -9.80665));
  result.angular_velocity = rotation * body.omega;
  const auto ak = streamSeed(seed, cfg.name + "/acceleration"),
             gk = streamSeed(seed, cfg.name + "/angular_velocity");
  for (int i = 0; i < 3; ++i) {
    result.acceleration[i] += cfg.accel_bias[i] + cfg.accel_sigma[i] * randomNormal(ak, sample, i);
    result.angular_velocity[i] += cfg.gyro_bias[i] + cfg.gyro_sigma[i] * randomNormal(gk, sample, i);
    result.acceleration_covariance[i * 4] = cfg.accel_sigma[i] * cfg.accel_sigma[i];
    result.angular_velocity_covariance[i * 4] = cfg.gyro_sigma[i] * cfg.gyro_sigma[i];
  }
  return result;
}
void OdomError::validate() const {
  for (double v : {linear_scale, angular_scale, linear_bias, angular_bias, linear_sigma, angular_sigma})
    require(std::isfinite(v), "odom: non-finite error model");
  require(linear_scale > 0 && angular_scale > 0 && linear_sigma >= 0 && angular_sigma >= 0,
          "odom: invalid scale/sigma");
}
} // namespace litesim3d
