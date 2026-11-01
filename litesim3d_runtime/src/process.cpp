#include "litesim3d_runtime/runtime.hpp"
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

extern char **environ;
namespace litesim3d::runtime {
namespace {
volatile sig_atomic_t interrupted = 0;
void on_signal(int) { interrupted = 1; }
void check(int code, const std::string &operation) {
  require(code == 0, operation + ": " + std::strerror(code));
}
struct SpawnAttributes {
  posix_spawnattr_t attributes;
  posix_spawn_file_actions_t files;
  SpawnAttributes() { check(posix_spawnattr_init(&attributes), "spawn attributes");
    const auto error = posix_spawn_file_actions_init(&files);
    if (error) { posix_spawnattr_destroy(&attributes); check(error, "spawn files"); }
  }
  ~SpawnAttributes() { posix_spawnattr_destroy(&attributes); posix_spawn_file_actions_destroy(&files); }
};
}
InterruptGuard::InterruptGuard() {
  struct sigaction action{};
  action.sa_handler = on_signal;
  sigemptyset(&action.sa_mask);
  interrupted = 0;
  require(sigaction(SIGINT, &action, &previous_int_) == 0, "cannot install interrupt handler");
  if (sigaction(SIGTERM, &action, &previous_term_) != 0) {
    sigaction(SIGINT, &previous_int_, nullptr);
    throw std::runtime_error("cannot install termination handler");
  }
}
InterruptGuard::~InterruptGuard() {
  sigaction(SIGINT, &previous_int_, nullptr);
  sigaction(SIGTERM, &previous_term_, nullptr);
  interrupted = 0;
}
bool interruption_requested() { return interrupted != 0; }
Process::Process(const std::vector<std::string> &command, const ProcessOptions &options) {
  require(!command.empty() && !command.front().empty(), "empty child command");
  auto argv_storage = command;
  std::vector<char *> argv;
  for (auto &s : argv_storage) { require(s.find('\0') == std::string::npos, "NUL in child argument"); argv.push_back(s.data()); }
  argv.push_back(nullptr);
  std::map<std::string, std::string> environment;
  for (char **entry = environ; *entry; ++entry) {
    std::string value(*entry); const auto split = value.find('=');
    if (split != std::string::npos) environment[value.substr(0, split)] = value.substr(split + 1);
  }
  for (const auto &[key, value] : options.environment) environment[key] = value;
  std::vector<std::string> env_storage;
  for (const auto &[key, value] : environment) env_storage.push_back(key + "=" + value);
  std::vector<char *> env; for (auto &s : env_storage) env.push_back(s.data()); env.push_back(nullptr);
  SpawnAttributes spawn;
  sigset_t empty, defaults; sigemptyset(&empty); sigemptyset(&defaults);
  for (int signal : {SIGINT, SIGTERM, SIGPIPE}) sigaddset(&defaults, signal);
  check(posix_spawnattr_setsigmask(&spawn.attributes, &empty), "child signal mask");
  check(posix_spawnattr_setsigdefault(&spawn.attributes, &defaults), "child signals");
  check(posix_spawnattr_setpgroup(&spawn.attributes, 0), "child group");
  short flags = POSIX_SPAWN_SETPGROUP | POSIX_SPAWN_SETSIGMASK | POSIX_SPAWN_SETSIGDEF;
#ifdef __APPLE__
  flags |= POSIX_SPAWN_CLOEXEC_DEFAULT;
  for (const int fd : {STDIN_FILENO, STDOUT_FILENO, STDERR_FILENO})
    check(posix_spawn_file_actions_adddup2(&spawn.files, fd, fd), "inherit standard child stream");
#else
  // Ubuntu 22.04/glibc 2.35 provides closefrom for safe spawning in threaded nodes.
  check(posix_spawn_file_actions_addclosefrom_np(&spawn.files, 3), "close inherited child descriptors");
#endif
  check(posix_spawnattr_setflags(&spawn.attributes, flags), "child flags");
  auto open = [&](int fd, const fs::path &path, int flags) {
    if (!path.empty()) check(posix_spawn_file_actions_addopen(&spawn.files, fd, path.c_str(), flags, 0600), "child stream");
  };
  open(STDIN_FILENO, options.input, O_RDONLY);
  open(STDOUT_FILENO, options.output, O_CREAT | O_WRONLY | O_TRUNC);
  if (!options.error.empty() && options.error == options.output)
    check(posix_spawn_file_actions_adddup2(&spawn.files, STDOUT_FILENO, STDERR_FILENO), "child stderr");
  else open(STDERR_FILENO, options.error, O_CREAT | O_WRONLY | O_TRUNC);
  pid_t pid;
  check(posix_spawnp(&pid, argv.front(), &spawn.files, &spawn.attributes, argv.data(), env.data()), "start " + command.front());
  pid_ = pid;
}
Process::Process(Process &&other) noexcept : pid_(other.pid_), result_(other.result_) { other.pid_ = -1; }
Process &Process::operator=(Process &&other) noexcept {
  if (this != &other) { cleanup(); pid_ = other.pid_; result_ = other.result_; other.pid_ = -1; }
  return *this;
}
Process::~Process() { cleanup(); }
std::optional<int> Process::poll() {
  if (result_ || pid_ < 0) return result_;
  int status = 0; const auto result = waitpid(pid_, &status, WNOHANG);
  if (result < 0 && errno == EINTR) return {};
  require(result >= 0, "waitpid failed: " + std::string(std::strerror(errno)));
  if (result == pid_) result_ = WIFEXITED(status) ? WEXITSTATUS(status) : -WTERMSIG(status);
  return result_;
}
int Process::stop(double grace, bool *forced) {
  if (forced) *forced = false;
  if (pid_ < 0) return result_.value_or(0);
  poll();
  // Signal the owned group even if its leader has exited: grandchildren may remain.
  if (kill(-pid_, SIGINT) != 0 && errno != ESRCH) throw std::runtime_error("cannot interrupt owned process group");
  const auto deadline = SteadyClock::now() + std::chrono::duration<double>(grace);
  while (!poll() && SteadyClock::now() < deadline) std::this_thread::sleep_for(std::chrono::milliseconds(10));
  if (!result_) {
    if (forced) *forced = true;
    kill(-pid_, SIGKILL);
    int status; pid_t waited;
    do { waited = waitpid(pid_, &status, 0); } while (waited < 0 && errno == EINTR);
    require(waited == pid_, "failed to reap owned process");
    result_ = WIFEXITED(status) ? WEXITSTATUS(status) : -WTERMSIG(status);
  }
  // No process in an owned group may outlive its owner.
  kill(-pid_, SIGKILL);
  pid_ = -1;
  return *result_;
}
void Process::cleanup() noexcept {
  if (pid_ < 0) return;
  try { stop(); }
  catch (...) {
    kill(-pid_, SIGKILL);
    if (!result_) { int status; while (waitpid(pid_, &status, 0) < 0 && errno == EINTR) {} }
  }
}
CommandResult run(const std::vector<std::string> &argv, double timeout, const std::string &input,
                  const std::map<std::string, std::string> &environment) {
  require(std::isfinite(timeout) && timeout > 0, "command timeout must be finite and positive");
  TemporaryDirectory temp("litesim-command-");
  ProcessOptions options; options.input = temp.path() / "stdin"; options.output = temp.path() / "stdout";
  options.error = temp.path() / "stderr"; options.environment = environment;
  write(options.input, input);
  Process process(argv, options);
  const auto deadline = SteadyClock::now() + std::chrono::duration<double>(timeout);
  while (!process.poll()) {
    require(!interruption_requested(), "command interrupted: " + argv.front());
    if (SteadyClock::now() >= deadline) {
      process.stop(0); throw std::runtime_error("command timed out: " + argv.front());
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  const int code = *process.poll(); process.stop(0);
  return {code, read(options.output), read(options.error)};
}
std::string checked_run(const std::vector<std::string> &argv, double timeout, const std::string &input,
                        const std::map<std::string, std::string> &environment) {
  const auto result = run(argv, timeout, input, environment);
  require(result.code == 0, result.err.empty() ? "command failed: " + argv.front() : result.err);
  return result.out;
}
}  // namespace litesim3d::runtime
