#include "point_adapter.hpp"
#include <chrono>
#include <cstdlib>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
class Adapter : public rclcpp::Node {
public:
  Adapter() : Node("sim_ouster_adapter") {
    if (const auto *run = std::getenv("LWS_RUN_ID"))
      run_id_ = run;
    auto sensor_qos = rclcpp::SensorDataQoS();
    if (declare_parameter<bool>("sensor_reliable", false))
      sensor_qos.reliable();
    output_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        declare_parameter<std::string>("output_topic", "points_ouster"),
        sensor_qos);
    status_ = create_publisher<std_msgs::msg::String>("adapter_status", 10);
    input_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        declare_parameter<std::string>("input_topic", "points_raw"),
        sensor_qos,
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) {
          try {
            output_->publish(ousterCloud(*m));
            ++accepted_;
            error_.clear();
          } catch (const std::exception &e) {
            ++rejected_;
            error_ = e.what();
          }
          publishStatus();
        });
    status_timer_ = create_wall_timer(std::chrono::milliseconds(200),
                                     [this] { publishStatus(); });
  }

private:
  void publishStatus() {
    std_msgs::msg::String s;
    s.data = nlohmann::json({{"accepted", accepted_}, {"rejected", rejected_},
                            {"error", error_}, {"run_id", run_id_},
                            {"input_publishers", input_->get_publisher_count()},
                            {"output_subscribers", output_->get_subscription_count()}})
                 .dump();
    status_->publish(s);
  }
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr output_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr input_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  uint64_t accepted_ = 0, rejected_ = 0;
  std::string error_, run_id_;
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Adapter>());
  rclcpp::shutdown();
}
