#include "../src/ros_event_loop.hpp"
#include <rclcpp/rclcpp.hpp>
#include <gtest/gtest.h>

namespace {
struct TestContext : rclcpp::Context {
  // Humble's GraphListener owns a GuardCondition which retains its Context.
  // Finalize this private fixture after all nodes/executors are destroyed to
  // break that sub-context cycle, without suppressing any sanitizer reports.
  void finalize() { clean_up(); }
};

struct RosEventLoop : testing::Test {
  std::shared_ptr<TestContext> context = std::make_shared<TestContext>();
  void SetUp() override { context->init(0, nullptr); }
  void TearDown() override {
    context->shutdown("test complete");
    context->finalize();
    std::weak_ptr<TestContext> weak = context;
    context.reset();
    EXPECT_TRUE(weak.expired());
  }
};

TEST_F(RosEventLoop, ShutdownBetweenGuardAndSpinStopsNormally) {
  auto node = std::make_shared<rclcpp::Node>(
      "shutdown_race", rclcpp::NodeOptions().context(context));
  rclcpp::executors::SingleThreadedExecutor executor(
      litesim3d_render::ros_executor_options(context));
  executor.add_node(node);
  EXPECT_TRUE(litesim3d_render::process_ros_events(context, [&] { executor.spin_some(); }));
  bool entered = false;
  EXPECT_FALSE(litesim3d_render::process_ros_events(context, [&] {
    entered = true;
    context->shutdown("signal arrived after the GUI guard");
    // Exercise a real, already-created Humble executor across shutdown.
    executor.spin_some();
  }));
  EXPECT_TRUE(entered);
}

TEST_F(RosEventLoop, AlreadyStoppedContextDoesNotDispatch) {
  context->shutdown("already stopped");
  bool entered = false;
  EXPECT_FALSE(litesim3d_render::process_ros_events(context, [&] { entered = true; }));
  EXPECT_FALSE(entered);
}

TEST_F(RosEventLoop, ActiveContextDoesNotHideUnexpectedErrors) {
  EXPECT_THROW(litesim3d_render::process_ros_events(context, [] {
    throw std::runtime_error("unexpected callback failure");
  }), std::runtime_error);
  EXPECT_TRUE(context->is_valid());
}
}  // namespace
