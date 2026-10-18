#include "point_adapter.hpp"
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
class Adapter : public rclcpp::Node {
public:
  Adapter() : Node("sim_ouster_adapter") {
    output_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        declare_parameter<std::string>("output_topic", "points_ouster"),
        rclcpp::SensorDataQoS());
    status_ = create_publisher<std_msgs::msg::String>("adapter_status", 10);
    input_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        declare_parameter<std::string>("input_topic", "points_raw"),
        rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) {
          try {
            output_->publish(ousterCloud(*m));
            ++accepted_;
            error_.clear();
          } catch (const std::exception &e) {
            ++rejected_;
            error_ = e.what();
          }
          std_msgs::msg::String s;
          s.data = nlohmann::json({{"accepted", accepted_},
                                   {"rejected", rejected_},
                                   {"error", error_}})
                       .dump();
          status_->publish(s);
        });
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr output_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr input_;
  uint64_t accepted_ = 0, rejected_ = 0;
  std::string error_;
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Adapter>());
  rclcpp::shutdown();
}
