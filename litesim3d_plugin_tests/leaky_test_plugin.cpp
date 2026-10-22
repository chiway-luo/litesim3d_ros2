// Deliberately faulty, ASan-only positive control. Never a production plugin.
#include <cstdlib>
#include <litesim3d_plugin_api/api.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace litesim3d_test {
class LeakyChassis final : public litesim3d::plugins::ChassisPlugin {
public:
  __attribute__((noinline)) LeakyChassis() {
    volatile int *lost = static_cast<int *>(std::malloc(64));
    *lost = 42;
  }
  litesim3d::plugins::Json defaults() const override { return {}; }
  void initialize(const litesim3d::plugins::Initialization &,
                  const litesim3d::plugins::Json &) override {}
  litesim3d::plugins::DriveStep step(const litesim3d::plugins::ChassisInput &) const override { return {}; }
  void reset(const litesim3d::plugins::JointState &) override {}
};
}
PLUGINLIB_EXPORT_CLASS(litesim3d_test::LeakyChassis, litesim3d::plugins::ChassisPlugin)
