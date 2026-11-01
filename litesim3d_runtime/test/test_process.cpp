#include <gtest/gtest.h>
#include <litesim3d_runtime/runtime.hpp>
#include <signal.h>
#include <thread>
using namespace litesim3d::runtime;
TEST(Process, CapturesArgumentsStdinAndEnvironmentWithoutShellExpansion) {
  const auto result = run({"/bin/sh", "-c", "cat; printf '%s' \"$LITESIM_TEST_VALUE\"; printf error >&2; exit 7"},
    2, "input\n", {{"LITESIM_TEST_VALUE", "$(do-not-execute)"}});
  EXPECT_EQ(result.code, 7); EXPECT_EQ(result.out, "input\n$(do-not-execute)"); EXPECT_EQ(result.err, "error");
  EXPECT_THROW(checked_run({"/bin/false"}), std::runtime_error);
  EXPECT_THROW(run({"/definitely/missing"}), std::runtime_error);
}
TEST(Process, TimeoutReapsAndMovedOwnerRetainsChild) {
  EXPECT_THROW(run({"/bin/sleep", "30"}, .03), std::runtime_error);
  Process child({"/bin/sleep", "30"}); const auto pid = child.pid();
  Process owner(std::move(child)); EXPECT_EQ(owner.pid(), pid);
  EXPECT_EQ(owner.stop(.1), -SIGINT); EXPECT_EQ(child.pid(), -1);
  EXPECT_EQ(kill(pid, 0), -1);
}
TEST(Process, InterruptionUnwindsTheOwnedCommand) {
  InterruptGuard guard;
  std::thread signal([] { std::this_thread::sleep_for(std::chrono::milliseconds(100)); raise(SIGINT); });
  EXPECT_THROW(checked_run({"/bin/sleep", "30"}, 10), std::runtime_error);
  signal.join();
  EXPECT_TRUE(interruption_requested());
}
