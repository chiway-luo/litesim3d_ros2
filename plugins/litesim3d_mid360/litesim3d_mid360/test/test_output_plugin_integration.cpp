// Production DSO integration only. Direct-class serializer units deliberately
// do not load this library or participate in class_loader registration.
#include <litesim3d_ros2/lidar_output_plugin.hpp>
#include <pluginlib/class_loader.hpp>
#include <gtest/gtest.h>
#include <algorithm>

TEST(LivoxOutputPluginIntegration, InstalledPluginLoadsCreatesDestroysAndUnloads) {
  using litesim3d_ros::LidarOutputPlugin;
  constexpr auto name = "litesim3d_mid360/LivoxOutput";
  pluginlib::ClassLoader<LidarOutputPlugin> loader{
      "litesim3d_ros2", "litesim3d_ros::LidarOutputPlugin"};
  const auto declared = loader.getDeclaredClasses();
  ASSERT_NE(std::find(declared.begin(), declared.end(), name), declared.end());
  ASSERT_FALSE(loader.isClassLoaded(name));
  ASSERT_FALSE(class_loader::impl::hasANonPurePluginLibraryBeenOpened());

  for (int cycle = 0; cycle < 2; ++cycle) {
    auto plugin = loader.createSharedInstance(name);
    ASSERT_NE(plugin, nullptr);
    EXPECT_TRUE(loader.isClassLoaded(name));
    // Construction needs no ROS context, publishers, sensor or remote peer.
    EXPECT_FALSE(plugin->handles("unconfigured"));
    EXPECT_FALSE(plugin->publish(litesim3d::Cloud{}));
    EXPECT_TRUE(plugin->description().is_null());
    EXPECT_TRUE(class_loader::impl::getAvailableClasses<LidarOutputPlugin>(nullptr).empty());
    plugin.reset();
    EXPECT_EQ(loader.unloadLibraryForClass(name), 0);
    EXPECT_FALSE(loader.isClassLoaded(name));
    EXPECT_FALSE(class_loader::impl::hasANonPurePluginLibraryBeenOpened());
  }
}
