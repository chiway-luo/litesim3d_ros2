#pragma once

#include <chrono>
#include <filesystem>
#include <map>
#include <optional>
#include <signal.h>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>

namespace litesim3d::runtime {
namespace fs = std::filesystem;
using Json = nlohmann::ordered_json;
using SteadyClock = std::chrono::steady_clock;

void require(bool condition, const std::string &message);
std::string read(const fs::path &path);
void write(const fs::path &path, const std::string &text);
void atomic_write(const fs::path &path, const std::string &text);
fs::path canonical_path(const fs::path &path);
fs::path resolve(const std::string &uri, const fs::path &base);
fs::path executable(const std::string &package, const std::string &name);
fs::path package_share(const std::string &package);
std::string file_uri(const fs::path &path);
bool boolean(std::string value, const std::string &name);
Json load_yaml(const std::string &text);
Json read_yaml(const fs::path &path);
std::string dump_yaml(const Json &value);
Json merge(Json base, const Json &overlay);

// Install once in a command's main, before starting worker threads. The handler
// only sets a signal-safe flag; ordinary control flow owns child cleanup.
class InterruptGuard {
 public:
  InterruptGuard();
  ~InterruptGuard();
  InterruptGuard(const InterruptGuard &) = delete;
  InterruptGuard &operator=(const InterruptGuard &) = delete;
 private:
  struct sigaction previous_int_{}, previous_term_{};
};
bool interruption_requested();

// Only removes a directory allocated by this instance, never a caller's directory.
class TemporaryDirectory {
 public:
  explicit TemporaryDirectory(const std::string &prefix = "litesim-", const fs::path &parent = fs::temp_directory_path());
  ~TemporaryDirectory();
  TemporaryDirectory(const TemporaryDirectory &) = delete;
  TemporaryDirectory &operator=(const TemporaryDirectory &) = delete;
  const fs::path &path() const { return path_; }
 private:
  fs::path path_;
};

struct ProcessOptions {
  fs::path input, output, error;
  std::map<std::string, std::string> environment;
};
// posix_spawn keeps the child side async-signal-safe when called by ROS executors.
// Each child owns a process group, so cancellation also reaches its descendants.
class Process {
 public:
  Process(const std::vector<std::string> &argv, const ProcessOptions &options = {});
  ~Process();
  Process(Process &&other) noexcept;
  Process &operator=(Process &&other) noexcept;
  Process(const Process &) = delete;
  Process &operator=(const Process &) = delete;
  int pid() const { return pid_; }
  std::optional<int> poll();
  int stop(double grace_seconds = 5.0, bool *forced = nullptr);
 private:
  void cleanup() noexcept;
  int pid_ = -1;
  std::optional<int> result_;
};
struct CommandResult { int code; std::string out, err; };
CommandResult run(const std::vector<std::string> &argv, double timeout = 60,
                  const std::string &input = "", const std::map<std::string, std::string> &environment = {});
std::string checked_run(const std::vector<std::string> &argv, double timeout = 60,
                        const std::string &input = "", const std::map<std::string, std::string> &environment = {});
}  // namespace litesim3d::runtime
