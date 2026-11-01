#include <gtest/gtest.h>
#include "model.hpp"
using namespace litesim3d::bringup;

TEST(XacroPreparation, ProvenanceHashesEveryPinnedResourceIncludingTextures) {
  TemporaryDirectory directory; const auto root = directory.path();
  write(root / "source.xacro", "source"); write(root / "include space.xacro", "include");
  write(root / "resources/mesh.stl", "abc"); write(root / "resources/texture.png", "texture");
  const auto output = root / "out/robot.urdf"; std::vector<std::vector<std::string>> commands;
  auto runner = [&](const auto &command, auto timeout, auto input, auto environment) {
    commands.push_back(command); EXPECT_EQ(timeout, 30); EXPECT_TRUE(input.empty()); EXPECT_TRUE(environment.empty());
    return command[1] == "--deps" ? (root / "include space.xacro").string() :
      "<robot><mesh filename='mesh.stl'/><texture filename='texture.png'/></robot>";
  };
  const auto result = prepare_xacro(root / "source.xacro", output, {"name:=unit"}, root / "resources", runner);
  EXPECT_EQ(commands.size(), 2u); EXPECT_EQ(commands[0].back(), "name:=unit");
  EXPECT_EQ(result["resources"][(root / "resources/mesh.stl").string()], "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  EXPECT_EQ(result["resources"].size(), 2u); EXPECT_EQ(result["includes"].size(), 1u);
  EXPECT_EQ(Json::parse(read(output.string() + ".json")), result);
  EXPECT_EQ(result["expanded_sha256"].get<std::string>().size(), 64u);
  EXPECT_NE(read(output).find((root / "resources/mesh.stl").string()), std::string::npos);
  EXPECT_EQ(std::distance(fs::directory_iterator(output.parent_path()), fs::directory_iterator()), 2);
}
TEST(XacroPreparation, FailuresPreserveBothExistingOutputs) {
  TemporaryDirectory directory; const auto root = directory.path();
  const auto output = root / "model.urdf"; write(root / "source.xacro", "source");
  for (int failure = 0; failure < 5; ++failure) {
    write(output, "previous model"); write(output.string() + ".json", "previous metadata");
    auto runner = [&](const auto &command, auto, auto, auto) -> std::string {
      if (failure == 0 || (failure == 1 && command[1] == "--deps")) throw std::runtime_error("provider failed");
      if (command[1] == "--deps") return failure == 4 ? "/missing/include" : "";
      if (failure == 2) return "<broken";
      return "<robot><mesh filename='missing.stl'/></robot>";
    };
    EXPECT_THROW(prepare_xacro(root / "source.xacro", output, {}, {}, runner), std::runtime_error);
    EXPECT_EQ(read(output), "previous model"); EXPECT_EQ(read(output.string() + ".json"), "previous metadata");
  }
  const auto result = prepare_xacro(root / "source.xacro", output, {}, {}, [](const auto &command, auto, auto, auto) {
    return command[1] == "--deps" ? "" : "<robot/>";
  });
  EXPECT_EQ(result["resource_base"], root.string()); EXPECT_TRUE(result["includes"].empty()); EXPECT_TRUE(result["resources"].empty());
  auto runner = [](const auto &command, auto, auto, auto) { return command[1] == "--deps" ? "" : "<robot/>"; };
  EXPECT_THROW(prepare_xacro(root / "source.xacro", root / "source.xacro", {}, {}, runner), std::runtime_error);
  fs::remove(output.string() + ".json"); fs::create_directory(output.string() + ".json");
  const auto previous = read(output);
  EXPECT_THROW(prepare_xacro(root / "source.xacro", output, {}, {}, runner), std::runtime_error);
  EXPECT_EQ(read(output), previous);
}
