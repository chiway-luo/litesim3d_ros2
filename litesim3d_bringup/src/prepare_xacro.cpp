#include "model.hpp"
#include <iostream>

int main(int argc, char **argv) {
  using namespace litesim3d::bringup;
  try {
    InterruptGuard interrupts;
    if (argc == 2 && std::string(argv[1]) == "--help") {
      std::cout << "usage: prepare_xacro SOURCE OUTPUT [name:=value ...] [--resource-base DIR]\n"; return 0;
    }
    require(argc >= 3, "usage: prepare_xacro SOURCE OUTPUT [name:=value ...] [--resource-base DIR]");
    fs::path base; std::vector<std::string> mappings;
    for (int i = 3; i < argc; ++i) {
      const std::string value = argv[i];
      if (value == "--resource-base") { require(i + 1 < argc, "--resource-base needs a directory"); base = argv[++i]; }
      else { require(value.find(":=") != std::string::npos, "expected Xacro name:=value argument"); mappings.push_back(value); }
    }
    std::cout << prepare_xacro(argv[1], argv[2], mappings, base).dump() << '\n'; return 0;
  } catch (const std::exception &error) { std::cerr << "prepare_xacro: " << error.what() << '\n'; return 1; }
}
