#include <litesim3d/sensor_plugin.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gtest/gtest.h>

namespace lp = litesim3d::plugins;
using namespace litesim3d;
namespace {
lp::Initialization init() {
  lp::Initialization i;
  i.instance = "mid360"; i.seed = 17;
  i.model.base_link = "base_link"; i.model.links = {"base_link", "lidar_link"};
  return i;
}
Json config() {
  return lp::readParameterFile(std::filesystem::path(
      ament_index_cpp::get_package_share_directory("litesim3d_mid360")) / "config/parameters.yaml");
}
}

TEST(Mid360Plugin, InstalledDefaultsAndRequiredBindings) {
  lp::Loader loader;
  auto plugin = loader.sensor("litesim3d_mid360/Mid360");
  const auto defaults = plugin->defaults(), file = config();
  ASSERT_EQ(defaults.size(), file.size());
  for (auto it = defaults.begin(); it != defaults.end(); ++it) {
    ASSERT_TRUE(file.contains(it.key()));
    if (!it.value().is_null()) EXPECT_EQ(it.value(), file.at(it.key())) << it.key();
  }
  plugin->initialize(init(), {{"link", "lidar_link"}, {"topic", "points_raw"}});
  const auto desc = plugin->description();
  EXPECT_EQ(desc.kind, "lidar");
  EXPECT_EQ(desc.settings["scan_pattern"], "livox_mid360");
  EXPECT_EQ(desc.settings["columns"], 20000);
  EXPECT_EQ(desc.settings["scan_mode"], "rolling");
  EXPECT_THROW(plugin->initialize(init(), {{"topic", "points"}}), std::runtime_error);
  auto bad = file; bad["link"] = "missing";
  EXPECT_THROW(plugin->initialize(init(), bad), std::runtime_error);
  bad = file; bad["scan_pattern"] = "spinning";
  EXPECT_THROW(plugin->initialize(init(), bad), std::runtime_error);
  bad = file; bad["columns"] = 1000;
  EXPECT_THROW(plugin->initialize(init(), bad), std::runtime_error);
}

TEST(Mid360Plugin, MatchesGenericKernelWithRollingTrajectoryAndReset) {
  lp::Loader loader;
  auto mid = loader.sensor("litesim3d_mid360/Mid360");
  auto parameters = config();
  parameters["range_sigma_m"] = .02;
  mid->initialize(init(), parameters);
  Scene scene;
  scene.sensor = {{"wall", "", Shape::box({.2, 20, 20}), pose({3, 0, 0})}};
  scene.rebuild();
  lp::SensorContext context{scene, {}, Pose::Identity(),
      [](uint64_t stamp) { return pose({double(stamp - 1) * 1e-9, 0, 0}); }, {}, 1, 100000001, 2, 1};
  lp::SensorOutput actual, replay;
  mid->step(context, actual);
  const auto cfg = lp::lidarSettings("mid360", parameters);
  const auto expected = scanTrajectory(scene, cfg, context.stamp_ns, context.world_sensor,
      context.robot_in_sensor, init().seed, context.sample_index, context.threads);
  ASSERT_EQ(actual.clouds.size(), 1u);
  EXPECT_EQ(actual.clouds[0].data, expected.data);
  EXPECT_EQ(actual.clouds[0].stamp, expected.stamp);
  EXPECT_EQ(actual.clouds[0].completion_stamp, expected.completion_stamp);
  mid->reset(1);
  mid->step(context, replay);
  EXPECT_EQ(actual.clouds[0].data, replay.clouds[0].data);
  parameters["enabled"] = false;
  mid->initialize(init(), parameters);
  lp::SensorOutput disabled;
  mid->step(context, disabled);
  EXPECT_TRUE(disabled.clouds.empty());
}
