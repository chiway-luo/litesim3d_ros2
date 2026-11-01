// Selector/serializer-host units: the factory supplies a recording serializer,
// never a vendor plugin, publisher peer, rendering process or hardware driver.
#include "../src/lidar_outputs.hpp"
#include <gtest/gtest.h>

namespace {
using litesim3d_ros::LidarOutputs;
class Serializer final : public litesim3d_ros::LidarOutputPlugin {
public:
  int initialized = 0, published = 0;
  std::string topic, frame;
  litesim3d::Json settings, record{{"serializer", "unit"}};
  const litesim3d::Config *config = nullptr;
  rclcpp::CallbackGroup::SharedPtr group;
  void initialize(rclcpp::Node &, const litesim3d::Config &value,
      const std::string &topic_prefix, const std::string &frame_prefix,
      const litesim3d::Json &options, const rclcpp::PublisherOptions &publisher) override {
    ++initialized; config = &value; topic = topic_prefix; frame = frame_prefix;
    settings = options; group = publisher.callback_group;
  }
  bool handles(const std::string &topic) const override { return topic == "handled"; }
  bool publish(const litesim3d::Cloud &cloud) override { ++published; return cloud.topic == "handled"; }
  void matches(litesim3d::Json &value) const override { value["matched"] = true; }
  const litesim3d::Json &description() const override { return record; }
};
class LidarOutputUnits : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  rclcpp::Node node{"lidar_output_unit"};
};
}

TEST_F(LidarOutputUnits, NativeAndAutoWithoutMid360NeverLoadAnOptionalPlugin) {
  litesim3d::Config config;
  for (const char *interface : {"native", "auto"}) {
    int loaded = 0;
    LidarOutputs output(node, config, "", "", interface, 0, 0, {}, [&] {
      ++loaded; return std::make_shared<Serializer>();
    });
    EXPECT_EQ(loaded, 0); EXPECT_FALSE(output.handles("handled"));
    EXPECT_FALSE(output.publish(litesim3d::Cloud{}));
    auto match = litesim3d::Json::object(); output.matches(match); EXPECT_TRUE(match.empty());
    EXPECT_EQ(output.description().at("lidar_interface"), interface);
    EXPECT_EQ(output.description().at("xfer_format"), 0);
    EXPECT_TRUE(output.description().at("lidars").empty());
  }
  litesim3d::LidarConfig lidar; lidar.scan_pattern = "livox_mid360"; config.lidars.push_back(lidar);
  int loaded = 0;
  LidarOutputs native(node, config, "", "", "native", 1, 255, {}, [&] {
    ++loaded; return std::make_shared<Serializer>();
  });
  EXPECT_EQ(loaded, 0);
}

TEST_F(LidarOutputUnits, InvalidSelectorsAndMissingMid360AreRejectedBeforeLoading) {
  litesim3d::Config config;
  EXPECT_THROW(LidarOutputs(node, config, "", "", "invalid", 1, 0, {}), std::runtime_error);
  for (int format : {-1, 2})
    EXPECT_THROW(LidarOutputs(node, config, "", "", "native", format, 0, {}), std::runtime_error);
  for (int id : {-1, 256})
    EXPECT_THROW(LidarOutputs(node, config, "", "", "native", 1, id, {}), std::runtime_error);
  EXPECT_THROW(LidarOutputs(node, config, "", "", "livox", 1, 0, {}), std::runtime_error);
  config.lidars.push_back(litesim3d::LidarConfig{});
  EXPECT_THROW(LidarOutputs(node, config, "", "", "livox", 1, 0, {}), std::runtime_error);
}

TEST_F(LidarOutputUnits, MatchingSensorInitializesOneSerializerAndForwardsCallsAndSettings) {
  for (const char *interface : {"auto", "livox"}) for (int format : {0, 1}) {
    litesim3d::Config config; config.lidars.resize(2);
    config.lidars[1].scan_pattern = "livox_mid360";
    auto serializer = std::make_shared<Serializer>();
    LidarOutputs output(node, config, "robot/", "frame/", interface, format, 255, {}, [&] { return serializer; });
    EXPECT_EQ(serializer->initialized, 1); EXPECT_EQ(serializer->config, &config);
    EXPECT_EQ(serializer->topic, "robot/"); EXPECT_EQ(serializer->frame, "frame/");
    EXPECT_NE(serializer->group, nullptr);
    EXPECT_EQ(serializer->settings, (litesim3d::Json{{"lidar_interface", interface}, {"xfer_format", format}, {"livox_lidar_id", 255}}));
    EXPECT_TRUE(output.handles("handled")); EXPECT_FALSE(output.handles("unhandled"));
    litesim3d::Cloud cloud; cloud.topic = "handled"; EXPECT_TRUE(output.publish(cloud));
    cloud.topic = "unhandled"; EXPECT_FALSE(output.publish(cloud)); EXPECT_EQ(serializer->published, 2);
    auto match = litesim3d::Json::object(); output.matches(match); EXPECT_EQ(match.at("matched"), true);
    EXPECT_EQ(&output.description(), &serializer->record);
  }
}

TEST_F(LidarOutputUnits, FactoryFailureIncludesActionableOptionalPluginGuidance) {
  litesim3d::Config config; config.lidars.resize(1); config.lidars[0].scan_pattern = "livox_mid360";
  try {
    LidarOutputs output(node, config, "", "", "livox", 1, 0, {}, []() -> std::shared_ptr<litesim3d_ros::LidarOutputPlugin> {
      throw std::runtime_error("unit loader failure");
    });
    FAIL() << "factory failure was swallowed";
  } catch (const std::runtime_error &error) {
    const std::string message = error.what();
    EXPECT_NE(message.find("litesim3d_mid360 plugin"), std::string::npos);
    EXPECT_NE(message.find("lidar_interface:=native"), std::string::npos);
    EXPECT_NE(message.find("unit loader failure"), std::string::npos);
  }
  EXPECT_THROW(LidarOutputs(node, config, "", "", "livox", 1, 0, {}, [] {
    return std::shared_ptr<litesim3d_ros::LidarOutputPlugin>{};
  }), std::runtime_error);
}
