#include <gtest/gtest.h>
#include <litesim3d_runtime/runtime.hpp>
#include <signal.h>
#include <thread>

using namespace litesim3d::runtime;
TEST(Resources, YamlTypesDuplicatesAndRecursiveAliases) {
  const auto data = load_yaml("text: 'false'\nenabled: true\nnumber: 1.25\nitems: [1, null, two]\n");
  EXPECT_EQ(data["text"], "false"); EXPECT_EQ(data["enabled"], true);
  EXPECT_EQ(data["number"], 1.25); EXPECT_EQ(load_yaml(dump_yaml(data)), data);
  const auto floats = load_yaml(dump_yaml({{"frequency", 200.0}, {"count", 200}}));
  EXPECT_TRUE(floats["frequency"].is_number_float()); EXPECT_TRUE(floats["count"].is_number_integer());
  EXPECT_THROW(load_yaml("x: 1\nx: 2"), std::runtime_error);
  EXPECT_THROW(load_yaml("a: &a [*a]"), std::runtime_error);
  EXPECT_THROW(load_yaml("? [a,b]\n: value"), std::runtime_error);
}
TEST(Resources, ResourceAndOutputOwnership) {
  TemporaryDirectory temporary;
  auto path = temporary.path() / "中文 space/model.urdf";
  write(path, "original"); EXPECT_EQ(resolve("中文 space/model.urdf", temporary.path()), path);
  EXPECT_NE(file_uri(path).find("%20"), std::string::npos);
  EXPECT_EQ(resolve(file_uri(path), temporary.path()), path);
  for (const auto *uri : {"file://%", "file://%GG", "file://%00", "package://unknown/%2e%2e/escape"})
    EXPECT_THROW(resolve(uri, temporary.path()), std::runtime_error);
  EXPECT_THROW(resolve("missing", temporary.path()), std::runtime_error);
  EXPECT_THROW(resolve("package://unknown/../escape", temporary.path()), std::runtime_error);
  atomic_write(path, "replacement"); EXPECT_EQ(read(path), "replacement");
  EXPECT_TRUE(boolean(" TRUE ", "value")); EXPECT_THROW(boolean("1", "value"), std::runtime_error);
  EXPECT_EQ(merge({{"nested", {{"a", 1}}}}, {{"nested", {{"b", 2}}}})["nested"].size(), 2u);
}
