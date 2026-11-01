#include "point_adapter.hpp"
#include <chrono>
#include <cstdlib>
#include <nlohmann/json.hpp>
#include "cloud_tool_boundary.hpp"
#include <std_msgs/msg/string.hpp>
template<class Boundary = litesim3d_ros2::CloudToolBoundary>
class Adapter : public Boundary::Node {
  using BaseNode = typename Boundary::Node;
public:
  Adapter() : BaseNode("sim_ouster_adapter") {
    if (const auto *run = Boundary::environment("LWS_RUN_ID"))
      run_id_ = run;
    auto sensor_qos = rclcpp::SensorDataQoS();
    if (this->template declare_parameter<bool>("sensor_reliable", false))
      sensor_qos.reliable();
    output_ = this->template create_publisher<sensor_msgs::msg::PointCloud2>(
        this->template declare_parameter<std::string>("output_topic", "points_ouster"),
        sensor_qos);
    status_ = this->template create_publisher<std_msgs::msg::String>("adapter_status", 10);
    input_ = this->template create_subscription<sensor_msgs::msg::PointCloud2>(
        this->template declare_parameter<std::string>("input_topic", "points_raw"),
        sensor_qos,
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) { processCloud(*m); });
    status_timer_ = this->create_wall_timer(std::chrono::milliseconds(200),
                                     [this] { publishStatus(); });
  }

private:
  void processCloud(const sensor_msgs::msg::PointCloud2 &m) {
    try {
      output_->publish(ousterCloud(m));
      ++accepted_;
      error_.clear();
    } catch (const std::exception &e) {
      ++rejected_;
      error_ = e.what();
    }
    publishStatus();
  }
  void publishStatus() {
    std_msgs::msg::String s;
    s.data = nlohmann::json({{"accepted", accepted_}, {"rejected", rejected_},
                            {"error", error_}, {"run_id", run_id_},
                            {"input_publishers", input_->get_publisher_count()},
                            {"output_subscribers", output_->get_subscription_count()}})
                 .dump();
    status_->publish(s);
  }
  typename Boundary::template Publisher<sensor_msgs::msg::PointCloud2> output_;
  typename Boundary::template Publisher<std_msgs::msg::String> status_;
  typename Boundary::template Subscription<sensor_msgs::msg::PointCloud2> input_;
  typename Boundary::Timer status_timer_;
  uint64_t accepted_ = 0, rejected_ = 0;
  std::string error_, run_id_;
};
#ifndef LITESIM_CLOUD_TOOL_BOUNDARY
#define LITESIM_CLOUD_TOOL_BOUNDARY litesim3d_ros2::CloudToolBoundary
#endif
int main(int argc, char **argv) {
  using Boundary = LITESIM_CLOUD_TOOL_BOUNDARY;
  Boundary::initialize(argc, argv);
  Boundary::spin(std::make_shared<Adapter<Boundary>>());
  Boundary::shutdown();
  return 0;
}
