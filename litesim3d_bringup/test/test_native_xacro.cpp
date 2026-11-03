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

TEST(XacroPreparation, ResolvesWhitespaceSeparatedIncludesDeduplicatesResourcesAndReplacesArtifacts) {
  TemporaryDirectory directory; const auto root = directory.path();
  const auto source = root / "source.xacro", output = root / "out/robot.urdf";
  write(source, "abc"); write(root / "one.xacro", "one"); write(root / "with space/two.xacro", "two");
  write(root / "assets/mesh.stl", "abc"); write(root / "assets/texture.png", "texture");
  write(output, "old model"); write(output.string() + ".json", "old metadata");
  std::vector<std::vector<std::string>> calls;
  const std::vector<std::string> mappings = {"label:=a b", "flag:=false"};
  const auto provenance = prepare_xacro(source, output, mappings, root / "assets/../assets",
    [&](const auto &command, auto timeout, auto input, auto environment) {
      calls.push_back(command); EXPECT_EQ(timeout, 30); EXPECT_TRUE(input.empty()); EXPECT_TRUE(environment.empty());
      if (command[1] == "--deps") return std::string(" \t") + (root / "one.xacro").string() + "\n\r" +
        (root / "with space/two.xacro").string() + "\t" + (root / "one.xacro").string() + " \n";
      return std::string("<?xml version='1.0'?><robot><link><mesh filename='mesh.stl'/><visual><mesh filename='") +
        file_uri(root / "assets/mesh.stl") + "'/><texture filename='texture.png'/></visual></link></robot>";
    });
  ASSERT_EQ(calls.size(), 2u);
  EXPECT_EQ(calls[0], (std::vector<std::string>{"xacro", source.string(), "label:=a b", "flag:=false"}));
  EXPECT_EQ(calls[1], (std::vector<std::string>{"xacro", "--deps", source.string(), "label:=a b", "flag:=false"}));
  EXPECT_EQ(provenance["source"], source.string()); EXPECT_EQ(provenance["resource_base"], (root / "assets").string());
  EXPECT_EQ(provenance["source_sha256"], "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  EXPECT_EQ(provenance["mappings"], mappings); EXPECT_EQ(provenance["resources"].size(), 2u);
  EXPECT_EQ(provenance["includes"].size(), 2u); EXPECT_TRUE(provenance["includes"].contains((root / "one.xacro").string()));
  EXPECT_TRUE(provenance["includes"].contains((root / "with space/two.xacro").string()));
  EXPECT_EQ(Json::parse(read(output.string() + ".json")), provenance);
  EXPECT_EQ(read(source), "abc"); EXPECT_EQ(read(root / "assets/mesh.stl"), "abc");
  EXPECT_EQ(read(output).front(), '<'); EXPECT_EQ(read(output).back(), '\n'); EXPECT_EQ(read(output).find("<?xml"), std::string::npos);
  EXPECT_EQ(std::distance(fs::directory_iterator(output.parent_path()), fs::directory_iterator()), 2);
}

TEST(XacroPreparation, RejectsAmbiguousDependencyPrefixesWithoutChangingOutputs) {
  TemporaryDirectory directory; const auto root = directory.path();
  write(root / "source.xacro", "source"); write(root / "include", "short"); write(root / "include with space", "long");
  const auto output = root / "out/robot.urdf"; write(output, "old model"); write(output.string() + ".json", "old metadata");
  EXPECT_THROW(prepare_xacro(root / "source.xacro", output, {}, {}, [&](const auto &command, auto, auto, auto) {
    return command[1] == "--deps" ? (root / "include with space").string() : std::string("<robot/>");
  }), std::runtime_error);
  EXPECT_EQ(read(output), "old model"); EXPECT_EQ(read(output.string() + ".json"), "old metadata");
  EXPECT_EQ(std::distance(fs::directory_iterator(output.parent_path()), fs::directory_iterator()), 2);
}

TEST(XacroPreparation, ResourceValidationNeverPublishesPartialNewArtifacts) {
  TemporaryDirectory directory; const auto root = directory.path(); write(root / "source.xacro", "source");
  const auto output = root / "uncreated/robot.urdf";
  for (const auto &xml : {"<robot><mesh/></robot>", "<robot><texture/></robot>",
      "<robot><mesh filename='missing.stl'/></robot>", "<!-- missing root -->"}) {
    SCOPED_TRACE(xml);
    EXPECT_THROW(prepare_xacro(root / "source.xacro", output, {}, {}, [&](const auto &command, auto, auto, auto) {
      return command[1] == "--deps" ? "" : xml;
    }), std::runtime_error);
    EXPECT_FALSE(fs::exists(output)); EXPECT_FALSE(fs::exists(output.string() + ".json"));
    EXPECT_FALSE(fs::exists(output.parent_path()));
  }
  EXPECT_THROW(prepare_xacro(root / "missing.xacro", output, {}, {}, [](const auto &command, auto, auto, auto) {
    return command[1] == "--deps" ? "" : "<robot/>";
  }), std::runtime_error);
  EXPECT_FALSE(fs::exists(output)); EXPECT_FALSE(fs::exists(output.string() + ".json"));
}

TEST(XacroPreparation, OutputPreconditionsPreventRunnerExecution) {
  TemporaryDirectory directory; const auto root = directory.path(); write(root / "source.xacro", "source");
  const CommandRunner never = [](auto, auto, auto, auto) -> std::string { ADD_FAILURE() << "runner called before output validation"; return {}; };
  fs::create_directory(root / "output-directory");
  EXPECT_THROW(prepare_xacro(root / "source.xacro", root / "output-directory", {}, {}, never), std::runtime_error);
  fs::create_directory(root / "metadata-blocked.urdf.json");
  EXPECT_THROW(prepare_xacro(root / "source.xacro", root / "metadata-blocked.urdf", {}, {}, never), std::runtime_error);
  EXPECT_THROW(prepare_xacro(root / "source.xacro", root / "./source.xacro", {}, {}, never), std::runtime_error);
  EXPECT_EQ(read(root / "source.xacro"), "source"); EXPECT_FALSE(fs::exists(root / "metadata-blocked.urdf"));
}

TEST(XacroPreparation, MetadataCommitFailureRollsBackExistingModelAndCleansStagingDirectory) {
  TemporaryDirectory directory; const auto root = directory.path(); const auto output = root / "out/robot.urdf";
  write(root / "source.xacro", "source"); write(output, "previous model");
  // The runner is the existing boundary: mutate only this fixture after the
  // precondition check to simulate another writer blocking the metadata commit.
  EXPECT_THROW(prepare_xacro(root / "source.xacro", output, {}, {}, [&](const auto &command, auto, auto, auto) {
    if (command[1] == "--deps") { fs::create_directory(output.string() + ".json"); return ""; }
    return "<robot/>";
  }), fs::filesystem_error);
  EXPECT_EQ(read(output), "previous model"); EXPECT_TRUE(fs::is_directory(output.string() + ".json"));
  EXPECT_EQ(std::distance(fs::directory_iterator(output.parent_path()), fs::directory_iterator()), 2);
  EXPECT_EQ(read(root / "source.xacro"), "source");
}

TEST(XacroPreparation, MetadataCommitFailureRemovesNewModelAndCleansStagingDirectory) {
  TemporaryDirectory directory; const auto root = directory.path(); const auto output = root / "out/robot.urdf";
  write(root / "source.xacro", "source");
  EXPECT_THROW(prepare_xacro(root / "source.xacro", output, {}, {}, [&](const auto &command, auto, auto, auto) {
    if (command[1] == "--deps") { fs::create_directories(output.string() + ".json"); return ""; }
    return "<robot/>";
  }), fs::filesystem_error);
  EXPECT_FALSE(fs::exists(output)); EXPECT_TRUE(fs::is_directory(output.string() + ".json"));
  EXPECT_EQ(std::distance(fs::directory_iterator(output.parent_path()), fs::directory_iterator()), 1);
}
