#include "livox_outputs.hpp"
#include <algorithm>
#include <cstring>
#include <limits>
#include <pluginlib/class_list_macros.hpp>
#include <set>
#include <type_traits>

namespace litesim3d_ros {
namespace {
builtin_interfaces::msg::Time stamp(uint64_t ns) {
  builtin_interfaces::msg::Time result;
  result.sec = static_cast<int32_t>(ns / 1000000000);
  result.nanosec = ns % 1000000000;
  return result;
}
float readFloat(const std::vector<uint8_t> &data, size_t offset) {
  uint32_t bits = 0;
  for (unsigned i = 0; i < 4; ++i) bits |= uint32_t(data.at(offset + i)) << (8 * i);
  float value;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}
template<class T> void write(std::vector<uint8_t> &data, size_t offset, T value) {
  static_assert(sizeof(T) == 4 || sizeof(T) == 8);
  using Bits = std::conditional_t<sizeof(T) == 4, uint32_t, uint64_t>;
  Bits bits;
  std::memcpy(&bits, &value, sizeof(value));
  for (size_t i = 0; i < sizeof(T); ++i) data.at(offset + i) = (bits >> (8 * i)) & 255;
}
sensor_msgs::msg::PointCloud2 cloudMessage(const litesim3d::Cloud &c,
                                         const std::string &prefix) {
  sensor_msgs::msg::PointCloud2 m;
  m.header.stamp = stamp(c.stamp);
  m.header.frame_id = prefix + c.frame;
  m.height = 1;
  m.width = 0;
  m.point_step = 26;
  m.row_step = 0;
  m.is_bigendian = false;
  m.is_dense = true;
  const std::vector<std::string> names = {"x", "y", "z", "intensity", "tag", "line", "timestamp"};
  const std::vector<uint32_t> offsets = {0,4,8,12,16,17,18};
  const std::vector<uint8_t> types = {7,7,7,7,2,2,8};
  for (size_t i = 0; i < names.size(); ++i) {
    sensor_msgs::msg::PointField f;
    f.name = names[i]; f.offset = offsets[i]; f.count = 1; f.datatype = types[i];
    m.fields.push_back(f);
  }
  return m;
}
} // namespace

void LivoxOutputs::initialize(rclcpp::Node &node, const litesim3d::Config &config,
    const std::string &topic_prefix, const std::string &frame_prefix,
    const litesim3d::Json &settings, const rclcpp::PublisherOptions &options) {
  lidars_.clear();
  frame_prefix_ = frame_prefix;
  const auto interface = settings.at("lidar_interface").get<std::string>();
  const int xfer_format = settings.at("xfer_format"), lidar_id = settings.at("livox_lidar_id");
  using litesim3d::require;
  require(interface == "native" || interface == "auto" || interface == "livox",
          "lidar_interface must be native, auto or livox");
  require(xfer_format == 0 || xfer_format == 1, "xfer_format must be 0 (PointCloud2) or 1 (CustomMsg)");
  require(lidar_id >= 0 && lidar_id <= 255, "livox_lidar_id must be in [0,255]");
  const auto selected = [&](const auto &l) {
    return interface != "native" && l.scan_pattern == "livox_mid360";
  };
  const auto count = std::count_if(config.lidars.begin(), config.lidars.end(), selected);
  require(interface != "livox" || count > 0, "livox output requires a Mid-360 lidar");
  require(count <= 256 - lidar_id, "Livox lidar IDs exceed uint8 range");
  // Official ROS 2 driver uses reliable/volatile KeepLast publishers.
  const auto livox_qos = rclcpp::QoS(256).reliable();
  description_ = {{"lidar_interface", interface}, {"xfer_format", xfer_format},
                  {"lidars", litesim3d::Json::array()}};
  std::set<std::string> topics;
  const auto topic = [&](const std::string &name) {
    const auto result = node.get_node_topics_interface()->resolve_topic_name(topic_prefix + name);
    require(topics.insert(result).second, "sensor output topic collision: " + result);
    return result;
  };
  unsigned sensor_index = 0;
  for (const auto &cfg : config.lidars) {
    if (!selected(cfg)) continue;
    Lidar l; l.config = cfg;
    l.lidar_id = static_cast<uint8_t>(lidar_id + sensor_index++);
    // Keep the single-sensor driver contract; separate multi-sensor streams.
    const std::string stem = count == 1 ? "livox/lidar" : "livox/" + cfg.name + "/lidar";
    const auto main = topic(stem);
    {
      require(cfg.offsetNs(0, cfg.columns - 1) <= std::numeric_limits<uint32_t>::max(),
              "Livox frame point offset exceeds uint32 nanoseconds");
      if (xfer_format == 1) {
        l.custom = node.create_publisher<livox_ros_driver2::msg::CustomMsg>(main, livox_qos, options);
        l.cloud = node.create_publisher<sensor_msgs::msg::PointCloud2>(
            topic(stem + "/pointcloud"), livox_qos, options);
      } else {
        l.cloud = node.create_publisher<sensor_msgs::msg::PointCloud2>(main, livox_qos, options);
      }
    }
    if (cfg.publish_range_boundary)
      l.boundary = node.create_publisher<sensor_msgs::msg::PointCloud2>(
          topic(stem + "/range_boundary"), livox_qos, options);
    description_["lidars"].push_back({{"name",cfg.name}, {"topic",main}, {"lidar_id",l.lidar_id},
        {"message_type",l.custom ? "livox_ros_driver2/msg/CustomMsg" : "sensor_msgs/msg/PointCloud2"},
        {"pointcloud_topic",l.cloud->get_topic_name()},
        {"frame_id",frame_prefix_ + cfg.link}, {"qos","reliable/256"}});
    lidars_.emplace(cfg.topic, std::move(l));
  }
}

bool LivoxOutputs::publish(const litesim3d::Cloud &c) {
  if (!handles(c.topic)) return false;
  auto &l = lidars_.at(c.topic);
  auto m = cloudMessage(c, frame_prefix_);
  livox_ros_driver2::msg::CustomMsg custom;
  custom.header = m.header; custom.timebase = c.stamp; custom.lidar_id = l.lidar_id;
  if (l.custom) custom.points.reserve(size_t(c.width) * c.height);
  auto boundary = cloudMessage(c, frame_prefix_);
  const auto convert = [&](const std::vector<uint8_t> &data, bool helper,
                           sensor_msgs::msg::PointCloud2 &dest) {
    dest.data.reserve(size_t(c.width) * c.height * 26);
    for (size_t r = 0; r < c.height; ++r) for (size_t col = 0; col < c.width; ++col) {
      const size_t slot = r * c.width + col, offset = r * c.row_step + col * 24;
      const float x = readFloat(data,offset), y = readFloat(data,offset+4), z = readFloat(data,offset+8);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
      const auto dt = l.config.offsetNs(r,col);
      litesim3d::require(c.stamp <= std::numeric_limits<uint64_t>::max() - dt, "Livox timestamp overflow");
      const auto reflectivity = uint8_t(std::lround(std::clamp(readFloat(data,offset+12),0.0f,1.0f)*255));
      // Official Mid-360 driver assigns i % 4. Keep original slots after filtering.
      const uint8_t line = uint8_t((c.frame_index * uint64_t(c.width) * c.height + slot) % 4);
      const uint8_t tag = 0; // Synthetic normal-confidence return; no device confidence simulation.
      const size_t at = dest.data.size(); dest.data.resize(at + 26);
      write(dest.data,at,x); write(dest.data,at+4,y); write(dest.data,at+8,z);
      write(dest.data,at+12,float(reflectivity));
      dest.data[at+16] = tag; dest.data[at+17] = line;
      // Upstream PointXYZRTLT stores absolute nanoseconds as FLOAT64, not seconds.
      write(dest.data,at+18,double(c.stamp + dt));
      if (!helper && l.custom) {
        livox_ros_driver2::msg::CustomPoint p;
        p.x=x; p.y=y; p.z=z; p.reflectivity=reflectivity; p.tag=tag; p.line=line;
        p.offset_time=static_cast<uint32_t>(dt);
        custom.points.push_back(p);
      }
    }
    dest.width = static_cast<uint32_t>(dest.data.size()/26);
    dest.row_step = dest.width * 26;
  };
  convert(c.data, false, m);
  if (l.custom) { custom.point_num = custom.points.size(); l.custom->publish(custom); }
  l.cloud->publish(m);
  if (l.boundary) { convert(c.range_boundary, true, boundary); l.boundary->publish(boundary); }
  return true;
}

void LivoxOutputs::matches(litesim3d::Json &result) const {
  const auto count = [&](const auto &pub) {
    if (pub) result[pub->get_topic_name()] = pub->get_subscription_count();
  };
  for (const auto &[topic,l] : lidars_) { count(l.cloud); count(l.custom); count(l.boundary); }
}
} // namespace litesim3d_ros
PLUGINLIB_EXPORT_CLASS(litesim3d_ros::LivoxOutputs, litesim3d_ros::LidarOutputPlugin)
