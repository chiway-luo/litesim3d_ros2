#include <gtest/gtest.h>
#include <litesim3d_runtime/runtime.hpp>

#include <cstdlib>
#include <fstream>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <type_traits>

using namespace litesim3d::runtime;

namespace {

// Environment changes are scoped to a test and restored even after a fatal
// assertion. These tests use the real ament index against temporary prefixes.
class ScopedEnvironment {
 public:
  explicit ScopedEnvironment(const std::string &name) : name_(name) {
    if (const auto *value = std::getenv(name.c_str())) previous_ = value;
  }
  ~ScopedEnvironment() {
    if (previous_) setenv(name_.c_str(), previous_->c_str(), 1);
    else unsetenv(name_.c_str());
  }
  void set(const std::string &value) { ASSERT_EQ(setenv(name_.c_str(), value.c_str(), 1), 0); }
  void unset() { ASSERT_EQ(unsetenv(name_.c_str()), 0); }
 private:
  std::string name_;
  std::optional<std::string> previous_;
};

class ScopedWorkingDirectory {
 public:
  explicit ScopedWorkingDirectory(const fs::path &path) : previous_(fs::current_path()) {
    fs::current_path(path);
  }
  ~ScopedWorkingDirectory() {
    std::error_code error;
    fs::current_path(previous_, error);
  }
 private:
  fs::path previous_;
};

std::set<fs::path> entries(const fs::path &directory) {
  std::set<fs::path> result;
  for (const auto &entry : fs::directory_iterator(directory)) result.insert(entry.path().filename());
  return result;
}

class ResourceFiles : public testing::Test {
 protected:
  TemporaryDirectory temporary;
};

class PackageResources : public ResourceFiles {
 protected:
  PackageResources() : prefix_("AMENT_PREFIX_PATH") {}
  void SetUp() override {
    root_ = temporary.path() / "install prefix";
    install(root_, "resource_test");
    prefix_.set(root_.string());
  }
  void install(const fs::path &prefix, const std::string &package) {
    write(prefix / "share/ament_index/resource_index/packages" / package, "");
    fs::create_directories(prefix / "share" / package);
  }
  ScopedEnvironment prefix_;
  fs::path root_;
};

}  // namespace

TEST(Resources, RequirePreservesTheDiagnostic) {
  EXPECT_NO_THROW(require(true, "unused"));
  try {
    require(false, "configuration must name a model");
    FAIL() << "require accepted a failed precondition";
  } catch (const std::runtime_error &error) {
    EXPECT_STREQ(error.what(), "configuration must name a model");
  }
}

TEST_F(ResourceFiles, BinaryReadsAndWritesPreserveEveryByte) {
  std::string bytes;
  for (unsigned value = 0; value < 256; ++value) bytes.push_back(static_cast<char>(value));
  const auto path = temporary.path() / "nested/中文 space/data.bin";
  write(path, bytes);
  EXPECT_EQ(read(path), bytes);
  EXPECT_EQ(fs::file_size(path), bytes.size());
  write(path, "short");
  EXPECT_EQ(read(path), "short");
  EXPECT_EQ(fs::file_size(path), 5u);
  write(path, "");
  EXPECT_TRUE(read(path).empty());
  EXPECT_EQ(fs::file_size(path), 0u);
}

TEST_F(ResourceFiles, WritesANameWithNoParentPath) {
  ScopedWorkingDirectory working(temporary.path());
  write("settings.yaml", "enabled: true\n");
  EXPECT_EQ(read(temporary.path() / "settings.yaml"), "enabled: true\n");
}

TEST_F(ResourceFiles, MissingReadsAndUnwritablePathsFailWithoutChangingOtherFiles) {
  const auto missing = temporary.path() / "absent";
  try {
    read(missing);
    FAIL() << "read accepted a nonexistent file";
  } catch (const std::runtime_error &error) {
    EXPECT_EQ(error.what(), "cannot read: " + missing.string());
  }
  write(temporary.path() / "parent-file", "keep");
  EXPECT_THROW(write(temporary.path() / "parent-file/child", "new"), fs::filesystem_error);
  EXPECT_THROW(write(temporary.path(), "new"), std::runtime_error);
  EXPECT_EQ(read(temporary.path() / "parent-file"), "keep");
  EXPECT_FALSE(fs::exists(missing));
}

TEST_F(ResourceFiles, ReadingADirectoryMustReportAnIoError) {
  const auto empty = temporary.path() / "empty";
  const auto populated = temporary.path() / "populated";
  const auto link = temporary.path() / "directory-link";
  fs::create_directories(empty);
  write(populated / "keep", "unchanged");
  fs::create_directory_symlink(empty, link);
  for (const auto &path : {empty, populated, link}) {
    SCOPED_TRACE(path.string());
    try {
      read(path);
      FAIL() << "read accepted a directory";
    } catch (const std::runtime_error &error) {
      EXPECT_EQ(error.what(), "cannot read: " + path.string());
    }
  }
  EXPECT_TRUE(fs::is_empty(empty));
  EXPECT_EQ(read(populated / "keep"), "unchanged");
}

TEST_F(ResourceFiles, TemporaryDirectoriesAreUniqueAndRemoveOnlyOwnedTrees) {
  static_assert(!std::is_copy_constructible<TemporaryDirectory>::value, "one cleanup owner");
  static_assert(!std::is_copy_assignable<TemporaryDirectory>::value, "one cleanup owner");
  const auto parent = temporary.path() / "parent";
  write(parent / "caller-owned", "survives");
  fs::path first_path;
  fs::path second_path;
  {
    TemporaryDirectory first("resource-unit-", parent);
    first_path = first.path();
    EXPECT_EQ(first_path.parent_path(), parent);
    EXPECT_EQ(first_path.filename().string().rfind("resource-unit-", 0), 0u);
    write(first_path / "nested/file", "owned");
    {
      TemporaryDirectory second("resource-unit-", parent);
      second_path = second.path();
      EXPECT_NE(first_path, second_path);
      EXPECT_TRUE(fs::is_directory(second_path));
    }
    EXPECT_FALSE(fs::exists(second_path));
    EXPECT_EQ(read(first_path / "nested/file"), "owned");
  }
  EXPECT_FALSE(fs::exists(first_path));
  EXPECT_EQ(entries(parent), std::set<fs::path>{"caller-owned"});
  EXPECT_EQ(read(parent / "caller-owned"), "survives");
}

TEST_F(ResourceFiles, TemporaryDirectoryCleanupDoesNotFollowSymlinks) {
  const auto outside = temporary.path() / "outside";
  write(outside / "keep", "external");
  fs::path owned;
  {
    TemporaryDirectory directory("owned-", temporary.path());
    owned = directory.path();
    fs::create_directory_symlink(outside, owned / "link");
  }
  EXPECT_FALSE(fs::exists(owned));
  EXPECT_EQ(read(outside / "keep"), "external");
}

TEST_F(ResourceFiles, TemporaryDirectoryCreatesParentsAndToleratesAlreadyRemovedTree) {
  const auto parent = temporary.path() / "new/nested/parent";
  {
    TemporaryDirectory directory("", parent);
    EXPECT_EQ(directory.path().filename().string().size(), 6u);
    EXPECT_TRUE(fs::is_directory(directory.path()));
    fs::remove_all(directory.path());
  }
  EXPECT_TRUE(fs::is_directory(parent));
  EXPECT_TRUE(fs::is_empty(parent));
}

TEST_F(ResourceFiles, TemporaryDirectoryReportsAllocationAndParentErrors) {
  EXPECT_THROW(TemporaryDirectory("missing/child-", temporary.path()), std::runtime_error);
  write(temporary.path() / "file", "keep");
  EXPECT_THROW(TemporaryDirectory("unit-", temporary.path() / "file"), fs::filesystem_error);
  EXPECT_EQ(entries(temporary.path()), std::set<fs::path>{"file"});
  EXPECT_EQ(read(temporary.path() / "file"), "keep");
}

TEST_F(ResourceFiles, AtomicWriteCreatesAndReplacesWithoutLeavingScratchDirectories) {
  const auto target = temporary.path() / "new/nested/config.json";
  atomic_write(target, "original long document");
  EXPECT_EQ(read(target), "original long document");
  EXPECT_EQ(entries(target.parent_path()), std::set<fs::path>{"config.json"});
  const std::string replacement("a\0b", 3);
  atomic_write(target, replacement);
  EXPECT_EQ(read(target), replacement);
  atomic_write(target, "");
  EXPECT_TRUE(read(target).empty());
  EXPECT_EQ(entries(target.parent_path()), std::set<fs::path>{"config.json"});
}

TEST_F(ResourceFiles, AtomicWriteResolvesRelativePathsAndExistingSymlinkTargets) {
  const auto target = temporary.path() / "target";
  const auto link = temporary.path() / "link";
  write(target, "old");
  fs::create_symlink("target", link);
  ScopedWorkingDirectory working(temporary.path());
  atomic_write("./link", "new");
  EXPECT_TRUE(fs::is_symlink(link));
  EXPECT_EQ(read(target), "new");
  EXPECT_EQ(read(link), "new");
  EXPECT_EQ(entries(temporary.path()), (std::set<fs::path>{"link", "target"}));
}

TEST_F(ResourceFiles, AtomicWriteReplacesTheFileInsteadOfTruncatingExistingReaders) {
  const auto target = temporary.path() / "target";
  const auto previous_link = temporary.path() / "previous";
  write(target, "original snapshot");
  fs::create_hard_link(target, previous_link);
  std::ifstream existing_reader(target, std::ios::binary);
  ASSERT_TRUE(existing_reader);
  atomic_write(target, "replacement");
  std::ostringstream snapshot;
  snapshot << existing_reader.rdbuf();
  EXPECT_EQ(snapshot.str(), "original snapshot");
  EXPECT_EQ(read(previous_link), "original snapshot");
  EXPECT_EQ(read(target), "replacement");
  EXPECT_FALSE(fs::equivalent(previous_link, target));
  EXPECT_EQ(entries(temporary.path()), (std::set<fs::path>{"previous", "target"}));
}

TEST_F(ResourceFiles, AtomicWriteFailurePreservesTargetAndCleansItsCandidate) {
  const auto target = temporary.path() / "directory";
  write(target / "keep", "original");
  EXPECT_THROW(atomic_write(target, "replacement"), fs::filesystem_error);
  EXPECT_EQ(read(target / "keep"), "original");
  EXPECT_EQ(entries(temporary.path()), std::set<fs::path>{"directory"});
  write(temporary.path() / "parent-file", "parent");
  EXPECT_THROW(atomic_write(temporary.path() / "parent-file/child", "new"), fs::filesystem_error);
  EXPECT_EQ(read(temporary.path() / "parent-file"), "parent");
}

TEST_F(ResourceFiles, CanonicalPathNormalizesRelativeAndNotYetExistingComponents) {
  ScopedWorkingDirectory working(temporary.path());
  fs::create_directory("present");
  EXPECT_EQ(canonical_path("present/../new/./file"), temporary.path() / "new/file");
  EXPECT_EQ(canonical_path("."), temporary.path());
  EXPECT_EQ(canonical_path(temporary.path() / "present/../leaf"), temporary.path() / "leaf");
  fs::create_directory_symlink("present", "alias");
  EXPECT_EQ(canonical_path("alias/new"), temporary.path() / "present/new");
}

TEST_F(ResourceFiles, CanonicalPathExpandsOnlyTheCurrentUsersHomeSyntax) {
  ScopedEnvironment home("HOME");
  home.set((temporary.path() / "home space").string());
  EXPECT_EQ(canonical_path("~"), temporary.path() / "home space");
  EXPECT_EQ(canonical_path("~/nested/../model"), temporary.path() / "home space/model");
  ScopedWorkingDirectory working(temporary.path());
  EXPECT_EQ(canonical_path("~someone/model"), temporary.path() / "~someone/model");
  EXPECT_EQ(canonical_path("embedded~name"), temporary.path() / "embedded~name");
  home.unset();
  EXPECT_THROW(canonical_path("~"), std::runtime_error);
  EXPECT_THROW(canonical_path("~/file"), std::runtime_error);
  EXPECT_EQ(canonical_path("ordinary"), temporary.path() / "ordinary");
}

TEST_F(ResourceFiles, ResolveAcceptsAbsoluteRelativeSymlinkAndHomeResources) {
  const auto model = temporary.path() / "models/中文 space/model.urdf";
  write(model, "robot");
  EXPECT_EQ(resolve("models/中文 space/model.urdf", temporary.path()), model);
  EXPECT_EQ(resolve(model.string(), temporary.path() / "unrelated"), model);
  EXPECT_EQ(resolve("../model.urdf", model.parent_path() / "not-created"), model);
  fs::create_symlink(model, temporary.path() / "alias");
  EXPECT_EQ(resolve("alias", temporary.path()), model);
  ScopedEnvironment home("HOME");
  home.set(temporary.path().string());
  EXPECT_EQ(resolve("~/models/中文 space/model.urdf", "/unused"), model);
  home.set(model.string());
  EXPECT_EQ(resolve("~", "/unused"), model);
}

TEST_F(ResourceFiles, ResolveRejectsMissingFilesDirectoriesAndBrokenSymlinks) {
  EXPECT_THROW(resolve("missing", temporary.path()), std::runtime_error);
  EXPECT_THROW(resolve("", temporary.path()), std::runtime_error);
  EXPECT_THROW(resolve(".", temporary.path()), std::runtime_error);
  fs::create_symlink("missing", temporary.path() / "broken");
  EXPECT_THROW(resolve("broken", temporary.path()), std::runtime_error);
}

TEST_F(ResourceFiles, HomeBasedResourcesAndFileUrisRequireHomeToBeSet) {
  ScopedEnvironment home("HOME");
  home.unset();
  EXPECT_THROW(resolve("~/model.urdf", temporary.path()), std::runtime_error);
  EXPECT_THROW(resolve("file://~/model.urdf", temporary.path()), std::runtime_error);
  EXPECT_THROW(file_uri("~/model.urdf"), std::runtime_error);
}

TEST_F(ResourceFiles, CanonicalResourcesAndFileUrisPropagateSymlinkLoopErrors) {
  fs::create_symlink("second", temporary.path() / "first");
  fs::create_symlink("first", temporary.path() / "second");
  EXPECT_THROW(canonical_path(temporary.path() / "first"), fs::filesystem_error);
  EXPECT_THROW(resolve("first", temporary.path()), fs::filesystem_error);
  EXPECT_THROW(file_uri(temporary.path() / "first"), fs::filesystem_error);
}

TEST_F(ResourceFiles, FileUrisEncodeReservedUtf8AndControlBytesExactly) {
  const std::string basename = "Az09-_.~ space+#?%[]@!$&'()=,;中文" + std::string("\x01\x7f", 2);
  const auto path = temporary.path() / basename;
  write(path, "bytes");
  const auto expected = "file://" + temporary.path().string() +
    "/Az09-_.~%20space%2B%23%3F%25%5B%5D%40%21%24%26%27%28%29%3D%2C%3B%E4%B8%AD%E6%96%87%01%7F";
  EXPECT_EQ(file_uri(path), expected);
  EXPECT_EQ(resolve(expected, temporary.path() / "wrong-base"), path);
  EXPECT_EQ(file_uri(temporary.path() / "not yet created"),
            "file://" + temporary.path().string() + "/not%20yet%20created");
}

TEST_F(ResourceFiles, FileUriDecodingAcceptsHexCaseAndDoesNotDecodeBarePaths) {
  write(temporary.path() / "part+name", "plus");
  write(temporary.path() / "encoded%20name", "literal percent");
  write(temporary.path() / "percent%2B", "decode once");
  EXPECT_EQ(resolve("file://part%2Bname", temporary.path()), temporary.path() / "part+name");
  EXPECT_EQ(resolve("file://part%2bname", temporary.path()), temporary.path() / "part+name");
  EXPECT_EQ(resolve("file://part+name", temporary.path()), temporary.path() / "part+name");
  EXPECT_EQ(resolve("encoded%20name", temporary.path()), temporary.path() / "encoded%20name");
  EXPECT_EQ(resolve("file://percent%252B", temporary.path()), temporary.path() / "percent%2B");
}

class InvalidFileUri : public ResourceFiles, public testing::WithParamInterface<const char *> {};
TEST_P(InvalidFileUri, RejectsMalformedPercentEscapesBeforeOpeningAResource) {
  try {
    resolve(GetParam(), temporary.path());
    FAIL() << "accepted " << GetParam();
  } catch (const std::runtime_error &error) {
    const std::string uri = GetParam();
    EXPECT_STREQ(error.what(), uri.find("%00") != std::string::npos
      ? "NUL in resource URI" : "invalid escaped resource URI");
  }
}
INSTANTIATE_TEST_SUITE_P(Escapes, InvalidFileUri, testing::Values(
  "file://%", "file://%0", "file://%GG", "file://%G0", "file://%0G", "file://%g0",
  "file://%/0", "file://%:0", "file://%00", "file://prefix%00suffix",
  "file://ok%20bad%", "file://%2F%"));

TEST_F(PackageResources, PackageShareAndExecutableUseTheRealIndexPrefix) {
  EXPECT_EQ(package_share("resource_test"), root_ / "share/resource_test");
  EXPECT_EQ(executable("resource_test", "worker"), root_ / "lib/resource_test/worker");
  EXPECT_FALSE(fs::exists(executable("resource_test", "worker")));
  EXPECT_THROW(package_share("missing_resource_package"), std::out_of_range);
  EXPECT_THROW(executable("missing_resource_package", "worker"), std::out_of_range);
}

TEST_F(PackageResources, FirstRegisteredPackagePrefixWins) {
  const auto overlay = temporary.path() / "overlay";
  install(overlay, "resource_test");
  prefix_.set(overlay.string() + ":" + root_.string());
  write(overlay / "share/resource_test/model.urdf", "overlay");
  write(root_ / "share/resource_test/model.urdf", "underlay");
  EXPECT_EQ(package_share("resource_test"), overlay / "share/resource_test");
  EXPECT_EQ(executable("resource_test", "worker"), overlay / "lib/resource_test/worker");
  EXPECT_EQ(read(resolve("package://resource_test/model.urdf", temporary.path())), "overlay");
}

TEST_F(PackageResources, PackageResolutionDecodesAndCanonicalizesRelativeResources) {
  const auto model = root_ / "share/resource_test/models/中文 space/model+.urdf";
  write(model, "robot");
  EXPECT_EQ(resolve("package://resource_test/models/中文%20space/model%2B.urdf", "/unused"), model);
  EXPECT_EQ(resolve("package://resource_test/./models/中文 space/model+.urdf", "/unused"), model);
  EXPECT_EQ(resolve("package://resource_test/models%2f中文%20space%2fmodel%2b.urdf", "/unused"), model);
  EXPECT_THROW(resolve("package://resource_test/missing", temporary.path()), std::runtime_error);
  EXPECT_THROW(resolve("package://resource_test/models", temporary.path()), std::runtime_error);
  EXPECT_THROW(resolve("package://missing_resource_package/model", temporary.path()), std::out_of_range);
}

TEST_F(PackageResources, PackageNameDotDotMustNotEscapeAPackageShare) {
  const auto outside = root_ / "private.txt";
  write(outside, "outside every package share");
  try {
    resolve("package://../private.txt", temporary.path());
    FAIL() << "resolved a resource outside a package share";
  } catch (const std::runtime_error &error) {
    EXPECT_STREQ(error.what(), "invalid package resource: package://../private.txt");
  }
  EXPECT_EQ(read(outside), "outside every package share");
}

TEST_F(PackageResources, CompatiblePackageNamesAndSymlinkInstallStillResolve) {
  for (const auto *name : {"a", "0", "Legacy-Package_2", "resource_test"}) {
    SCOPED_TRACE(name);
    install(root_, name);
    const auto model = root_ / "share" / name / "model.urdf";
    write(model, name);
    EXPECT_EQ(resolve(std::string("package://") + name + "/model.urdf", "/unused"), model);
  }
  // A symlink install may intentionally refer to a source tree outside share.
  const auto source = temporary.path() / "source" / "model.urdf";
  write(source, "symlink-installed resource");
  fs::create_symlink(source, root_ / "share/resource_test/linked.urdf");
  EXPECT_EQ(resolve("package://resource_test/linked.urdf", "/unused"), source);
  EXPECT_EQ(read(resolve("package://resource_test/linked.urdf", "/unused")),
            "symlink-installed resource");
}

TEST_F(PackageResources, EmbeddedNulInPackageNameIsRejectedBeforeIndexLookup) {
  const auto uri = std::string("package://resource_test") + '\0' + "/model.urdf";
  prefix_.unset();
  try {
    resolve(uri, temporary.path());
    FAIL() << "accepted a NUL in a package name";
  } catch (const std::runtime_error &error) {
    EXPECT_STREQ(error.what(), "invalid package resource: package://resource_test");
  }
}

TEST_F(PackageResources, MissingAmentEnvironmentReportsLookupFailure) {
  prefix_.unset();
  EXPECT_THROW(package_share("resource_test"), std::runtime_error);
  EXPECT_THROW(executable("resource_test", "worker"), std::runtime_error);
  prefix_.set("");
  EXPECT_THROW(package_share("resource_test"), std::runtime_error);
}

class InvalidPackageUri : public PackageResources, public testing::WithParamInterface<const char *> {};
TEST_P(InvalidPackageUri, RejectsMalformedOrTraversingPackagePaths) {
  try {
    resolve(GetParam(), temporary.path());
    FAIL() << "accepted " << GetParam();
  } catch (const std::runtime_error &error) {
    const std::string uri = GetParam();
    if (uri == "package://resource_test/%00") EXPECT_STREQ(error.what(), "NUL in resource URI");
    else if (uri == "package://resource_test/%" || uri == "package://resource_test/%0X")
      EXPECT_STREQ(error.what(), "invalid escaped resource URI");
    else EXPECT_EQ(error.what(), "invalid package resource: " + uri);
  }
}
INSTANTIATE_TEST_SUITE_P(PackagePaths, InvalidPackageUri, testing::Values(
  "package://", "package:///model", "package://resource_test", "package://resource_test/",
  "package://resource_test//absolute", "package://resource_test/%2fabsolute",
  "package://resource_test/../outside", "package://resource_test/a/../model",
  "package://resource_test/%2e%2e/outside", "package://resource_test/%2E%2E/outside",
  "package://resource_test/a/%2e%2e/model", "package://resource_test/%00",
  "package://resource_test/%", "package://resource_test/%0X",
  "package://./model", "package://%2e%2e/model", "package://resource.test/model",
  "package://_bad/model", "package://-bad/model", "package://bad name/model",
  "package://bad\\\\name/model", "package://bad%00name/model"));

TEST(Resources, BooleanAcceptsBothValuesWithWhitespaceAndMixedCase) {
  for (const auto *value : {"true", "TRUE", "TrUe", " \ttrue\r\n"}) {
    SCOPED_TRACE(value);
    EXPECT_TRUE(boolean(value, "enabled"));
  }
  for (const auto *value : {"false", "FALSE", "FaLsE", "\r\n false \t"}) {
    SCOPED_TRACE(value);
    EXPECT_FALSE(boolean(value, "enabled"));
  }
}

TEST(Resources, BooleanRejectsEmbeddedNulAndNonAsciiBytes) {
  EXPECT_THROW(boolean(std::string("true\0", 5), "enabled"), std::runtime_error);
  EXPECT_THROW(boolean(std::string(1, static_cast<char>(0xff)), "enabled"), std::runtime_error);
}

class InvalidBoolean : public testing::TestWithParam<const char *> {};
TEST_P(InvalidBoolean, ReportsTheFieldNameForNonBooleanValues) {
  try {
    boolean(GetParam(), "use_sim_time");
    FAIL() << "accepted " << GetParam();
  } catch (const std::runtime_error &error) {
    EXPECT_STREQ(error.what(), "use_sim_time must be true or false");
  }
}
INSTANTIATE_TEST_SUITE_P(Values, InvalidBoolean, testing::Values(
  "", " \t\r\n", "1", "0", "yes", "no", "on", "off", "true false", "tr ue", "falsex", "\vtrue"));

TEST(Resources, YamlNullEmptyContainersAndNestedSequencesRetainTheirTypes) {
  for (const auto *text : {"", "# comment only\n", "null", "~", "---\n"}) {
    SCOPED_TRACE(text);
    EXPECT_TRUE(load_yaml(text).is_null());
  }
  EXPECT_EQ(load_yaml("{}"), Json::object());
  EXPECT_EQ(load_yaml("[]"), Json::array());
  const auto value = load_yaml("items: [1, null, two, [true, {}]]\nempty:\n");
  EXPECT_EQ(value["items"], Json::array({1, nullptr, "two", Json::array({true, Json::object()})}));
  EXPECT_TRUE(value["empty"].is_null());
}

TEST(Resources, YamlQuotedAndExplicitStringTagsPreventImplicitConversion) {
  const auto data = load_yaml(
    "boolean: 'false'\ninteger: \"200\"\nnull_text: 'null'\n"
    "explicit: !!str 42\nlocal: ! true\nempty: ''\n"
    "unicode: 中文\nblock: |\n  first\n  second\n");
  EXPECT_EQ(data["boolean"], "false");
  EXPECT_EQ(data["integer"], "200");
  EXPECT_EQ(data["null_text"], "null");
  EXPECT_EQ(data["explicit"], "42");
  EXPECT_EQ(data["local"], "true");
  EXPECT_EQ(data["empty"], "");
  EXPECT_EQ(data["unicode"], "中文");
  EXPECT_EQ(data["block"], "first\nsecond\n");
  for (const auto &entry : data.items()) EXPECT_TRUE(entry.value().is_string());
}

TEST(Resources, YamlBooleansAndIntegerBoundariesAreConvertedWithoutLosingType) {
  const auto values = load_yaml("[true, FALSE, yes, Off, 0, -17, +42, 0007, -9223372036854775808, 9223372036854775807]");
  EXPECT_EQ(values[0], true);
  EXPECT_EQ(values[1], false);
  EXPECT_EQ(values[2], true);
  EXPECT_EQ(values[3], false);
  EXPECT_EQ(values[4], 0);
  EXPECT_EQ(values[5], -17);
  EXPECT_EQ(values[6], 42);
  EXPECT_EQ(values[7], 7);
  EXPECT_EQ(values[8], std::numeric_limits<long long>::min());
  EXPECT_EQ(values[9], std::numeric_limits<long long>::max());
  for (std::size_t i = 0; i < 4; ++i) EXPECT_TRUE(values[i].is_boolean());
  for (std::size_t i = 4; i < values.size(); ++i) EXPECT_TRUE(values[i].is_number_integer());
}

TEST(Resources, YamlDecimalExponentAndOutOfIntegerRangeNumbersBecomeFloats) {
  const auto values = load_yaml("[1.25, +.5, -0.25, 2., 1e3, -2E-2, 9223372036854775808, -9223372036854775809]");
  const std::vector<double> expected{1.25, .5, -.25, 2., 1000., -.02, 9223372036854775808.0, -9223372036854775809.0};
  ASSERT_EQ(values.size(), expected.size());
  for (std::size_t i = 0; i < expected.size(); ++i) {
    SCOPED_TRACE(i);
    EXPECT_TRUE(values[i].is_number_float());
    EXPECT_DOUBLE_EQ(values[i].get<double>(), expected[i]);
  }
  EXPECT_THROW(load_yaml("1e9999"), std::out_of_range);
}

TEST(Resources, YamlUnrecognizedNumericSpellingsRemainText) {
  for (const auto *value : {"0xff", "0o12", "1_000", "1.2.3", "1e", ".inf", "-.Inf", ".nan", "2026-11-02", "robot", "+"}) {
    SCOPED_TRACE(value);
    EXPECT_EQ(load_yaml(value), value);
  }
}

TEST(Resources, YamlStringKeysPreserveInsertionOrderAndRejectDuplicatesAtAnyDepth) {
  const auto values = load_yaml("z: 1\na: 2\n'1': numeric text\n'': empty key\n");
  EXPECT_EQ(values.begin().key(), "z");
  EXPECT_EQ(values["1"], "numeric text");
  EXPECT_EQ(values[""], "empty key");
  EXPECT_THROW(load_yaml("x: 1\nx: 2"), std::runtime_error);
  EXPECT_THROW(load_yaml("outer: {x: 1, x: 2}"), std::runtime_error);
  EXPECT_THROW(load_yaml("'x': 1\n\"x\": 2"), std::runtime_error);
}

class InvalidYamlKey : public testing::TestWithParam<const char *> {};
TEST_P(InvalidYamlKey, RejectsNonStringMappingKeys) {
  EXPECT_THROW(load_yaml(GetParam()), std::runtime_error);
}
INSTANTIATE_TEST_SUITE_P(KeyTypes, InvalidYamlKey, testing::Values(
  "1: value", "1.5: value", "true: value", "null: value", "? [a, b]\n: value",
  "? {a: b}\n: value", "outer: {false: value}"));

TEST(Resources, YamlRepeatedAliasesAreValuesButRecursiveAliasesAreRejected) {
  const auto values = load_yaml("first: &base {mass: 2, names: [a, b]}\nsecond: *base\n");
  EXPECT_EQ(values["first"], values["second"]);
  EXPECT_THROW(load_yaml("a: &a [*a]"), std::runtime_error);
  EXPECT_THROW(load_yaml("a: &a {self: *a}"), std::runtime_error);
  EXPECT_THROW(load_yaml("["), YAML::ParserException);
  EXPECT_THROW(load_yaml("value: *undefined"), YAML::ParserException);
}

TEST(Resources, YamlDepthLimitAccepts63LevelsAndRejects64) {
  const auto nested = [](unsigned depth) {
    return std::string(depth, '[') + "0" + std::string(depth, ']');
  };
  auto result = load_yaml(nested(63));
  for (unsigned depth = 0; depth < 63; ++depth) {
    ASSERT_TRUE(result.is_array());
    ASSERT_EQ(result.size(), 1u);
    // Copy through a separate object; assigning an owning JSON object from its
    // own child can invalidate that child before the copy is complete.
    auto next = result[0];
    result = std::move(next);
  }
  EXPECT_EQ(result, 0);
  EXPECT_THROW(load_yaml(nested(64)), std::runtime_error);
}

TEST_F(ResourceFiles, ReadYamlUsesFileContentsAndPropagatesMissingAndParseErrors) {
  const auto path = temporary.path() / "configuration.yaml";
  write(path, "robot: {mass: 4.5, enabled: true}\n");
  EXPECT_EQ(read_yaml(path), load_yaml(read(path)));
  EXPECT_DOUBLE_EQ(read_yaml(path)["robot"]["mass"].get<double>(), 4.5);
  write(path, "invalid: [");
  EXPECT_THROW(read_yaml(path), YAML::ParserException);
  EXPECT_THROW(read_yaml(temporary.path() / "missing.yaml"), std::runtime_error);
}

TEST(Resources, DumpYamlRoundTripsEscapesNullsContainersAndRosNumericTypes) {
  const Json data = {
    {"frequency", 200.0}, {"count", 200}, {"enabled", false},
    {"nothing", nullptr}, {"text", "false\n\"quoted\"\\中文"},
    {"array", Json::array({1, 1.5, nullptr, true})},
    {"map", Json::object()}, {"empty", Json::array()}
  };
  const auto dumped = dump_yaml(data);
  EXPECT_EQ(dumped.back(), '\n');
  const auto restored = load_yaml(dumped);
  EXPECT_EQ(restored, data);
  EXPECT_TRUE(restored["frequency"].is_number_float());
  EXPECT_TRUE(restored["count"].is_number_integer());
  EXPECT_EQ(dump_yaml(nullptr), "null\n");
  EXPECT_EQ(dump_yaml(Json::array()), "[]\n");
  EXPECT_EQ(dump_yaml(Json::object()), "{}\n");
  EXPECT_EQ(load_yaml(dump_yaml("true")), "true");
}

TEST(Resources, MergeRecursivelyOverlaysMappingsWithoutMutatingInputs) {
  const Json base = {
    {"untouched", "keep"},
    {"nested", {{"keep", 1}, {"replace", 2}, {"deep", {{"left", true}}}}},
    {"array", Json::array({1, 2})}, {"nullable", 5}, {"scalar_to_map", 7}
  };
  const Json overlay = {
    {"nested", {{"replace", 20}, {"new", 3}, {"deep", {{"right", false}}}}},
    {"array", Json::array({3})}, {"nullable", nullptr},
    {"scalar_to_map", {{"child", 8}}}, {"added", true}
  };
  const auto original_base = base;
  const auto original_overlay = overlay;
  const auto result = merge(base, overlay);
  EXPECT_EQ(result["untouched"], "keep");
  EXPECT_EQ(result["nested"], (Json{{"keep", 1}, {"replace", 20}, {"deep", {{"left", true}, {"right", false}}}, {"new", 3}}));
  EXPECT_EQ(result["array"], Json::array({3}));
  EXPECT_TRUE(result["nullable"].is_null());
  EXPECT_EQ(result["scalar_to_map"], (Json{{"child", 8}}));
  EXPECT_EQ(result["added"], true);
  EXPECT_EQ(base, original_base);
  EXPECT_EQ(overlay, original_overlay);
  EXPECT_EQ(result.begin().key(), "untouched");
}

TEST(Resources, MergeReplacesNonMappingValuesAndHandlesEmptyMappings) {
  const std::vector<Json> values{nullptr, true, 1, 1.5, "text", Json::array({1}), Json{{"key", 2}}};
  for (const auto &base : values) {
    for (const auto &overlay : values) {
      SCOPED_TRACE(base.dump() + " -> " + overlay.dump());
      if (base.is_object() && overlay.is_object()) continue;
      EXPECT_EQ(merge({{"value", base}}, {{"value", overlay}})["value"], overlay);
    }
  }
  EXPECT_EQ(merge(Json::object(), Json::object()), Json::object());
  EXPECT_EQ(merge({{"keep", 1}}, Json::object()), (Json{{"keep", 1}}));
  EXPECT_EQ(merge(Json::object(), {{"new", 2}}), (Json{{"new", 2}}));
  EXPECT_EQ(merge({{"nested", {{"keep", 1}}}}, {{"nested", Json::object()}}),
            (Json{{"nested", {{"keep", 1}}}}));
}

TEST(Resources, MergeRejectsEachNonMappingOperand) {
  for (const auto &value : std::vector<Json>{nullptr, true, 1, 1.5, "text", Json::array()}) {
    SCOPED_TRACE(value.dump());
    EXPECT_THROW(merge(value, Json::object()), std::runtime_error);
    EXPECT_THROW(merge(Json::object(), value), std::runtime_error);
    EXPECT_THROW(merge(value, value), std::runtime_error);
  }
}

// Retain the original baseline regressions as well as the expanded cases above.
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
