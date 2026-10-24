#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/system_interface.hpp>
#include <pluginlib/class_loader.hpp>
#include <string>

int main(int argc, char **argv) {
  if (argc != 2) return 2;
  if (std::string(argv[1]) == "hardware") {
    pluginlib::ClassLoader<hardware_interface::SystemInterface> loader{
      "hardware_interface", "hardware_interface::SystemInterface"};
    loader.createSharedInstance("probe/Hardware").reset();
  } else {
    pluginlib::ClassLoader<controller_interface::ControllerInterface> loader{
      "controller_interface", "controller_interface::ControllerInterface"};
    loader.createSharedInstance("probe/Controller").reset();
  }
}
