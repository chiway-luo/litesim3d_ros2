#include <litesim3d_plugin_api/api.hpp>

int main() {
  litesim3d::plugins::Loader loader;
  loader.chassis("litesim3d_test/LeakyChassis").reset();
}
