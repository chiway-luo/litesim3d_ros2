#include <nlohmann/json.hpp>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
// Own registered type with the frozen original field ABI; no dependency on an
// Ouster driver.
struct EIGEN_ALIGN16 ObservedPoint {
  PCL_ADD_POINT4D;
  float intensity;
  uint32_t t;
  uint16_t reflectivity, ring, ambient;
  uint32_t range;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
POINT_CLOUD_REGISTER_POINT_STRUCT(
    ObservedPoint,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
        uint32_t, t, t)(uint16_t, reflectivity,
                        reflectivity)(uint16_t, ring,
                                      ring)(uint16_t, ambient,
                                            ambient)(uint32_t, range, range))
static_assert(sizeof(ObservedPoint) == 48 && offsetof(ObservedPoint, t) == 20 &&
                  offsetof(ObservedPoint, range) == 32,
              "frozen adapter ABI");
class Probe : public rclcpp::Node {
public:
  Probe() : Node("sim_cloud_probe") {
    output_ = create_publisher<std_msgs::msg::String>("probe_status", 10);
    input_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "points_ouster", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) {
          try {
            const char *names[] = {"x",         "y",       "z",
                                   "intensity", "t",       "reflectivity",
                                   "ring",      "ambient", "range"};
            const uint32_t offsets[] = {0, 4, 8, 16, 20, 24, 26, 28, 32};
            const uint8_t types[] = {7, 7, 7, 7, 6, 4, 4, 4, 6};
            if (m->is_bigendian || m->point_step != 48 ||
                uint64_t(m->row_step) != uint64_t(m->width) * 48 || !m->width ||
                !m->height || uint64_t(m->width) * m->height > 4194304 ||
                m->data.size() != uint64_t(m->height) * m->row_step ||
                m->fields.size() != 9)
              throw std::runtime_error("invalid original cloud layout/size");
            for (int i = 0; i < 9; ++i) {
              auto f = std::find_if(
                  m->fields.begin(), m->fields.end(),
                  [&](const auto &field) { return field.name == names[i]; });
              if (f == m->fields.end() || f->offset != offsets[i] ||
                  f->datatype != types[i] || f->count != 1)
                throw std::runtime_error("invalid original field");
            }
            pcl::PCLPointCloud2 blob;
            blob.width = m->width;
            blob.height = m->height;
            blob.point_step = m->point_step;
            blob.row_step = m->row_step;
            blob.is_bigendian = m->is_bigendian;
            blob.is_dense = m->is_dense;
            blob.data = m->data;
            for (const auto &f : m->fields) {
              pcl::PCLPointField p;
              p.name = f.name;
              p.offset = f.offset;
              p.datatype = f.datatype;
              p.count = f.count;
              blob.fields.push_back(p);
            }
            pcl::PointCloud<ObservedPoint> vendor;
            pcl::fromPCLPointCloud2(blob, vendor);
            auto points = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
            uint32_t max_t = 0, max_ring = 0;
            bool fields_ok = true;
            for (const auto &p : vendor) {
              max_t = std::max(max_t, p.t);
              max_ring = std::max(max_ring, uint32_t(p.ring));
              if (std::isfinite(p.x) && std::isfinite(p.y) &&
                  std::isfinite(p.z)) {
                pcl::PointXYZI q;
                q.x = p.x;
                q.y = p.y;
                q.z = p.z;
                q.intensity = p.intensity;
                points->push_back(q);
                double range = std::sqrt(double(p.x) * p.x + double(p.y) * p.y +
                                         double(p.z) * p.z) *
                               1000;
                fields_ok &=
                    std::abs(range - p.range) <= .51 && p.ambient == 0 &&
                    std::abs(double(p.reflectivity) - p.intensity * 65535) <=
                        .51;
              }
            }
            pcl::VoxelGrid<pcl::PointXYZI> filter;
            filter.setLeafSize(.1f, .1f, .1f);
            filter.setInputCloud(points);
            pcl::PointCloud<pcl::PointXYZI> result;
            filter.filter(result);
            auto centroids = nlohmann::json::array();
            for (size_t i = 0; i < std::min<size_t>(8, result.size()); ++i)
              centroids.push_back({result[i].x, result[i].y, result[i].z});
            std_msgs::msg::String out;
            out.data = nlohmann::json({{"algorithm", "PCL VoxelGrid"},
                                       {"version", PCL_VERSION_PRETTY},
                                       {"frames", ++frames_},
                                       {"input", vendor.size()},
                                       {"finite", points->size()},
                                       {"output", result.size()},
                                       {"first_centroids", centroids},
                                       {"fields_ok", fields_ok},
                                       {"max_t_ns", max_t},
                                       {"max_ring", max_ring},
                                       {"frame", m->header.frame_id}})
                           .dump();
            output_->publish(out);
          } catch (const std::exception &e) {
            std_msgs::msg::String out;
            out.data = nlohmann::json({{"algorithm", "PCL VoxelGrid"},
                                       {"error", e.what()},
                                       {"fields_ok", false}})
                           .dump();
            output_->publish(out);
          }
        });
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr output_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr input_;
  uint64_t frames_ = 0;
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Probe>());
  rclcpp::shutdown();
}
