#include "litesim3d_runtime/runtime.hpp"
#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <regex>
#include <sstream>
#include <unistd.h>

namespace litesim3d::runtime {
void require(bool condition, const std::string &message) {
  if (!condition) throw std::runtime_error(message);
}
std::string read(const fs::path &path) {
  std::ifstream stream(path, std::ios::binary);
  require(bool(stream), "cannot read: " + path.string());
  require(!fs::is_directory(path), "cannot read: " + path.string());
  std::ostringstream out; out << stream.rdbuf();
  require(!stream.bad(), "read failed: " + path.string());
  return out.str();
}
void write(const fs::path &path, const std::string &text) {
  if (!path.parent_path().empty()) fs::create_directories(path.parent_path());
  std::ofstream stream(path, std::ios::binary | std::ios::trunc);
  require(bool(stream), "cannot write: " + path.string());
  stream << text; stream.close();
  require(bool(stream), "write failed: " + path.string());
}
TemporaryDirectory::TemporaryDirectory(const std::string &prefix, const fs::path &parent) {
  fs::create_directories(parent);
  auto pattern = (parent / (prefix + "XXXXXX")).string();
  std::vector<char> buffer(pattern.begin(), pattern.end()); buffer.push_back('\0');
  const char *created = mkdtemp(buffer.data());
  require(created != nullptr, "cannot allocate temporary directory in " + parent.string());
  path_ = created;
}
TemporaryDirectory::~TemporaryDirectory() {
  std::error_code error; fs::remove_all(path_, error);
}
void atomic_write(const fs::path &path, const std::string &text) {
  const auto target = canonical_path(path);
  TemporaryDirectory temporary(".litesim-write-", target.parent_path());
  const auto candidate = temporary.path() / "file";
  write(candidate, text);
  fs::rename(candidate, target);
}
fs::path canonical_path(const fs::path &path) {
  auto value = path.string();
  if (value == "~" || value.rfind("~/", 0) == 0) {
    const auto *home = std::getenv("HOME");
    require(home != nullptr, "HOME is required to expand '~'");
    value = std::string(home) + value.substr(1);
  }
  return fs::weakly_canonical(fs::absolute(value));
}
fs::path package_share(const std::string &package) {
  return ament_index_cpp::get_package_share_directory(package);
}
fs::path executable(const std::string &package, const std::string &name) {
  return fs::path(ament_index_cpp::get_package_prefix(package)) / "lib" / package / name;
}
namespace {
std::string decode_uri(const std::string &value) {
  auto hex = [](char c) -> int {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
  };
  std::string result;
  for (std::size_t i = 0; i < value.size(); ++i) {
    if (value[i] != '%') { result += value[i]; continue; }
    require(i + 2 < value.size() && hex(value[i + 1]) >= 0 && hex(value[i + 2]) >= 0, "invalid escaped resource URI");
    const char byte = static_cast<char>(hex(value[i + 1]) * 16 + hex(value[i + 2]));
    require(byte != '\0', "NUL in resource URI"); result += byte; i += 2;
  }
  return result;
}
}
fs::path resolve(const std::string &uri, const fs::path &base) {
  fs::path path;
  if (uri.rfind("package://", 0) == 0) {
    const auto split = uri.find('/', 10);
    require(split != std::string::npos && split > 10, "invalid package resource: " + uri);
    const auto package = uri.substr(10, split - 10);
    // Match catkin_pkg's compatible package names, including legacy uppercase/hyphens.
    static const std::regex package_name("[a-zA-Z0-9][a-zA-Z0-9_-]*");
    require(std::regex_match(package, package_name), "invalid package resource: " + uri);
    const fs::path relative(decode_uri(uri.substr(split + 1)));
    require(!relative.empty() && !relative.is_absolute(), "invalid package resource: " + uri);
    for (const auto &part : relative) require(part != "..", "invalid package resource: " + uri);
    path = package_share(package) / relative;
  } else {
    const auto value = uri.rfind("file://", 0) == 0 ? decode_uri(uri.substr(7)) : uri;
    path = !value.empty() && value.front() == '~' ? canonical_path(value) : base / value;
  }
  path = canonical_path(path);
  require(fs::is_regular_file(path), "resource not found: " + uri);
  return path;
}
std::string file_uri(const fs::path &path) {
  std::ostringstream out; out << "file://" << std::uppercase << std::hex;
  for (const unsigned char c : canonical_path(path).string()) {
    if (std::isalnum(c) && c < 128) out << c;
    else if (c == '/' || c == '-' || c == '_' || c == '.' || c == '~') out << c;
    else out << '%' << std::setw(2) << std::setfill('0') << static_cast<int>(c);
  }
  return out.str();
}
bool boolean(std::string value, const std::string &name) {
  const auto begin = value.find_first_not_of(" \t\r\n"), end = value.find_last_not_of(" \t\r\n");
  value = begin == std::string::npos ? "" : value.substr(begin, end - begin + 1);
  for (auto &c : value) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  require(value == "true" || value == "false", name + " must be true or false");
  return value == "true";
}
namespace {
Json convert_yaml(const YAML::Node &node, unsigned depth) {
  require(depth < 64, "YAML nesting exceeds 64 levels (possibly a recursive alias)");
  if (node.IsNull() || !node.IsDefined()) return nullptr;
  if (node.IsMap()) {
    auto result = Json::object();
    for (const auto &entry : node) {
      require(entry.first.IsScalar(), "YAML mapping keys must be scalar strings");
      require(convert_yaml(entry.first, depth + 1).is_string(), "YAML mapping keys must be strings (model_arguments requires a mapping of scalar Xacro arguments)");
      const auto key = entry.first.Scalar();
      require(!result.contains(key), "duplicate YAML key: " + key);
      result[key] = convert_yaml(entry.second, depth + 1);
    }
    return result;
  }
  if (node.IsSequence()) {
    auto result = Json::array();
    for (const auto &entry : node) result.push_back(convert_yaml(entry, depth + 1));
    return result;
  }
  const auto value = node.Scalar();
  if (node.Tag() == "!" || node.Tag() == "tag:yaml.org,2002:str") return value;
  try { return node.as<bool>(); } catch (const YAML::BadConversion &) {}
  static const std::regex integer("[+-]?[0-9]+"), number("[+-]?([0-9]+(\\.[0-9]*)?|\\.[0-9]+)([eE][+-]?[0-9]+)?");
  if (std::regex_match(value, integer)) {
    try { return std::stoll(value); } catch (const std::exception &) {}
  }
  if (std::regex_match(value, number)) {
    const double result = std::stod(value);
    require(std::isfinite(result), "non-finite YAML number: " + value);
    return result;
  }
  return value;
}
}  // namespace
Json load_yaml(const std::string &text) { return convert_yaml(YAML::Load(text), 0); }
Json read_yaml(const fs::path &path) { return load_yaml(read(path)); }
std::string dump_yaml(const Json &value) {
  // JSON is YAML flow syntax. Its float spelling keeps 200.0 distinct from
  // integer 200, which ROS parameter type checking requires.
  return value.dump(2) + "\n";
}
Json merge(Json base, const Json &overlay) {
  require(base.is_object() && overlay.is_object(), "merge requires mappings");
  for (auto it = overlay.begin(); it != overlay.end(); ++it)
    base[it.key()] = base.contains(it.key()) && base[it.key()].is_object() && it.value().is_object()
      ? merge(base[it.key()], it.value()) : it.value();
  return base;
}
}  // namespace litesim3d::runtime
