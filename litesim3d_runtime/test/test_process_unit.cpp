// Compile the actual implementation with only its operating-system boundaries
// redirected. The macros are local to this translation unit and follow all
// real headers: GTest, resources.cpp, and the runtime DSO retain real syscalls.
// No child is launched and no process is signalled by this unit target.
#include <gtest/gtest.h>
#include <litesim3d_runtime/runtime.hpp>
#include <algorithm>
#include <cerrno>
#include <cstring>
#include <deque>
#include <fcntl.h>
#include <functional>
#include <limits>
#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

namespace process_unit_boundary {
using litesim3d::runtime::fs::path;
struct WaitReply { pid_t result; int status = 0; int error = 0; };
struct KillReply { int result; int error; };
struct OpenAction { int fd; path file; int flags; mode_t mode; };
struct SignalCall { int signal; struct sigaction action; bool saving_previous; };
struct State {
  std::map<std::string, std::deque<int>> errors;
  std::vector<std::string> calls;
  std::vector<OpenAction> opens;
  std::vector<std::pair<int, int>> duplicates;
  std::vector<int> closefrom;
  std::vector<std::string> argv, environment;
  std::vector<std::string> inherited_environment;
  std::vector<char *> inherited_pointers;
  std::vector<std::pair<pid_t, int>> waits, signals;
  std::deque<WaitReply> wait_replies;
  std::deque<KillReply> kill_replies;
  std::map<int, struct sigaction> handlers;
  std::vector<SignalCall> handler_calls;
  std::deque<long> clock_milliseconds;
  long last_clock_milliseconds = 0;
  std::size_t clock_calls = 0;
  pid_t next_pid = 42000;
  pid_t group = -1;
  short flags = 0;
  sigset_t mask{}, defaults{};
  std::string executable, input, output = "unit stdout", error_output = "unit stderr";
  std::function<void()> on_spawn;
  int code(const std::string &operation) {
    calls.push_back(operation);
    auto &replies = errors[operation];
    if (replies.empty()) return 0;
    const int result = replies.front(); replies.pop_front(); return result;
  }
} state;
void original_int(int) {}
void original_term(int) {}
int attr_init(posix_spawnattr_t *) { return state.code("attributes init"); }
int attr_destroy(posix_spawnattr_t *) { return state.code("attributes destroy"); }
int files_init(posix_spawn_file_actions_t *) { return state.code("files init"); }
int files_destroy(posix_spawn_file_actions_t *) { return state.code("files destroy"); }
int set_mask(posix_spawnattr_t *, const sigset_t *mask) {
  state.mask = *mask; return state.code("signal mask");
}
int set_defaults(posix_spawnattr_t *, const sigset_t *signals) {
  state.defaults = *signals; return state.code("signal defaults");
}
int set_group(posix_spawnattr_t *, pid_t group) {
  state.group = group; return state.code("group");
}
int set_flags(posix_spawnattr_t *, short flags) {
  state.flags = flags; return state.code("flags");
}
int add_closefrom(posix_spawn_file_actions_t *, int fd) {
  state.closefrom.push_back(fd); return state.code("closefrom");
}
int add_open(posix_spawn_file_actions_t *, int fd, const char *file, int flags, mode_t mode) {
  state.opens.push_back({fd, file, flags, mode}); return state.code("open " + std::to_string(fd));
}
int add_duplicate(posix_spawn_file_actions_t *, int from, int to) {
  state.duplicates.emplace_back(from, to); return state.code("duplicate");
}
int spawn(pid_t *pid, const char *file, const posix_spawn_file_actions_t *,
          const posix_spawnattr_t *, char *const argv[], char *const environment[]) {
  state.executable = file;
  for (std::size_t i = 0; argv[i]; ++i) state.argv.emplace_back(argv[i]);
  for (std::size_t i = 0; environment[i]; ++i) state.environment.emplace_back(environment[i]);
  const int result = state.code("spawn");
  if (result) return result;
  *pid = state.next_pid++;
  // run() uses real owned temporary files. Populate its outputs instead of
  // starting a child; constructor-only tests leave streams unspecified.
  for (const auto &action : state.opens) {
    if (action.fd == STDIN_FILENO) state.input = litesim3d::runtime::read(action.file);
    if (action.fd == STDOUT_FILENO) litesim3d::runtime::write(action.file, state.output);
    if (action.fd == STDERR_FILENO) litesim3d::runtime::write(action.file, state.error_output);
  }
  if (state.on_spawn) state.on_spawn();
  return 0;
}
pid_t wait(pid_t pid, int *status, int options) {
  state.waits.emplace_back(pid, options);
  if (state.wait_replies.empty()) { *status = 0; return pid; }
  const auto reply = state.wait_replies.front(); state.wait_replies.pop_front();
  *status = reply.status; errno = reply.error;
  return reply.result == -2 ? pid : reply.result;
}
int send_signal(pid_t pid, int signal) {
  state.signals.emplace_back(pid, signal);
  if (state.kill_replies.empty()) return 0;
  const auto reply = state.kill_replies.front(); state.kill_replies.pop_front();
  errno = reply.error; return reply.result;
}
int set_handler(int signal, const struct sigaction *action, struct sigaction *previous) {
  state.handler_calls.push_back({signal, *action, previous != nullptr});
  const int error = state.code("sigaction " + std::to_string(signal));
  if (error) { errno = error; return -1; }
  if (previous) *previous = state.handlers[signal];
  state.handlers[signal] = *action;
  return 0;
}
struct Clock {
  using time_point = std::chrono::steady_clock::time_point;
  static time_point now() {
    ++state.clock_calls;
    if (!state.clock_milliseconds.empty()) {
      state.last_clock_milliseconds = state.clock_milliseconds.front();
      state.clock_milliseconds.pop_front();
    }
    return time_point{} + std::chrono::milliseconds(state.last_clock_milliseconds);
  }
};
}  // namespace process_unit_boundary
char **process_unit_environment = nullptr;

#define environ process_unit_environment
#define posix_spawnattr_init(...) ::process_unit_boundary::attr_init(__VA_ARGS__)
#define posix_spawnattr_destroy(...) ::process_unit_boundary::attr_destroy(__VA_ARGS__)
#define posix_spawn_file_actions_init(...) ::process_unit_boundary::files_init(__VA_ARGS__)
#define posix_spawn_file_actions_destroy(...) ::process_unit_boundary::files_destroy(__VA_ARGS__)
#define posix_spawnattr_setsigmask(...) ::process_unit_boundary::set_mask(__VA_ARGS__)
#define posix_spawnattr_setsigdefault(...) ::process_unit_boundary::set_defaults(__VA_ARGS__)
#define posix_spawnattr_setpgroup(...) ::process_unit_boundary::set_group(__VA_ARGS__)
#define posix_spawnattr_setflags(...) ::process_unit_boundary::set_flags(__VA_ARGS__)
#define posix_spawn_file_actions_addclosefrom_np(...) ::process_unit_boundary::add_closefrom(__VA_ARGS__)
#define posix_spawn_file_actions_addopen(...) ::process_unit_boundary::add_open(__VA_ARGS__)
#define posix_spawn_file_actions_adddup2(...) ::process_unit_boundary::add_duplicate(__VA_ARGS__)
#define posix_spawnp(...) ::process_unit_boundary::spawn(__VA_ARGS__)
#define waitpid(...) ::process_unit_boundary::wait(__VA_ARGS__)
#define kill(...) ::process_unit_boundary::send_signal(__VA_ARGS__)
#define sigaction(...) ::process_unit_boundary::set_handler(__VA_ARGS__)
#define SteadyClock ::process_unit_boundary::Clock
#include "../src/process.cpp"
#undef SteadyClock
#undef sigaction
#undef kill
#undef waitpid
#undef posix_spawnp
#undef posix_spawn_file_actions_adddup2
#undef posix_spawn_file_actions_addopen
#undef posix_spawn_file_actions_addclosefrom_np
#undef posix_spawnattr_setflags
#undef posix_spawnattr_setpgroup
#undef posix_spawnattr_setsigdefault
#undef posix_spawnattr_setsigmask
#undef posix_spawn_file_actions_destroy
#undef posix_spawn_file_actions_init
#undef posix_spawnattr_destroy
#undef posix_spawnattr_init
#undef environ

namespace {
using namespace litesim3d::runtime;
using process_unit_boundary::state;
constexpr pid_t owned = -2;  // Return the pid supplied to the fake waitpid.
int exited(int code) { return code << 8; }
class ProcessUnit : public ::testing::Test {
 protected:
  void SetUp() override {
    state = {};
    state.handlers[SIGINT].sa_handler = process_unit_boundary::original_int;
    state.handlers[SIGTERM].sa_handler = process_unit_boundary::original_term;
    { InterruptGuard reset; }
    state.calls.clear(); state.handler_calls.clear();
    environment({"BASE=original", "OVERRIDE=old", "IGNORED_WITHOUT_EQUALS", "DUP=old", "DUP=new"});
  }
  void TearDown() override {
    for (const auto &[name, replies] : state.errors) EXPECT_TRUE(replies.empty()) << name;
    EXPECT_TRUE(state.wait_replies.empty()) << "unused waitpid replies";
    EXPECT_TRUE(state.kill_replies.empty()) << "unused kill replies";
  }
  void environment(std::vector<std::string> values) {
    state.inherited_environment = std::move(values);
    state.inherited_pointers.clear();
    for (auto &value : state.inherited_environment) state.inherited_pointers.push_back(value.data());
    state.inherited_pointers.push_back(nullptr);
    process_unit_environment = state.inherited_pointers.data();
  }
  void fails(const std::string &operation, int error = EIO) { state.errors[operation].push_back(error); }
  std::size_t count(const std::string &operation) const {
    return std::count(state.calls.begin(), state.calls.end(), operation);
  }
  std::vector<std::pair<int, int>> standard_duplicates() const {
#ifdef __APPLE__
    return {{STDIN_FILENO, STDIN_FILENO}, {STDOUT_FILENO, STDOUT_FILENO}, {STDERR_FILENO, STDERR_FILENO}};
#else
    return {};
#endif
  }
  void error_contains(const std::function<void()> &call, const std::string &message) {
    try { call(); FAIL() << "expected runtime_error containing " << message; }
    catch (const std::runtime_error &error) { EXPECT_NE(std::string(error.what()).find(message), std::string::npos); }
  }
};

TEST_F(ProcessUnit, RejectsEmptyCommandsAndEmbeddedNulBeforeAnySpawnSetup) {
  EXPECT_THROW(Process(std::vector<std::string>{}), std::runtime_error);
  EXPECT_THROW(Process({""}), std::runtime_error);
  EXPECT_THROW(Process({"tool", std::string("a\0b", 3)}), std::runtime_error);
  EXPECT_THROW(Process({std::string("t\0ool", 5)}), std::runtime_error);
  EXPECT_TRUE(state.calls.empty()); EXPECT_TRUE(state.waits.empty()); EXPECT_TRUE(state.signals.empty());
}
TEST_F(ProcessUnit, PreservesLiteralArgumentsAndMergesEnvironmentWithoutShellExpansion) {
  ProcessOptions options; options.environment = {{"OVERRIDE", "new=literal"}, {"EMPTY", ""}, {"ADDED", "$(never-run)"}};
  Process child({"tool name", "", "$(touch nope)", "two words"}, options);
  EXPECT_EQ(state.executable, "tool name");
  EXPECT_EQ(state.argv, (std::vector<std::string>{"tool name", "", "$(touch nope)", "two words"}));
  EXPECT_EQ(state.environment, (std::vector<std::string>{"ADDED=$(never-run)", "BASE=original", "DUP=new", "EMPTY=", "OVERRIDE=new=literal"}));
  EXPECT_EQ(child.pid(), 42000);
  EXPECT_EQ(state.group, 0);
  short expected_flags = POSIX_SPAWN_SETPGROUP | POSIX_SPAWN_SETSIGMASK | POSIX_SPAWN_SETSIGDEF;
#ifdef __APPLE__
  expected_flags |= POSIX_SPAWN_CLOEXEC_DEFAULT;
#endif
  EXPECT_EQ(state.flags, expected_flags);
  for (int signal : {SIGINT, SIGTERM, SIGPIPE}) {
    EXPECT_EQ(sigismember(&state.mask, signal), 0); EXPECT_EQ(sigismember(&state.defaults, signal), 1);
  }
  EXPECT_EQ(sigismember(&state.defaults, SIGUSR1), 0);
#ifndef __APPLE__
  EXPECT_EQ(state.closefrom, (std::vector<int>{3}));
#endif
  EXPECT_TRUE(state.opens.empty());
  EXPECT_EQ(count("attributes destroy"), 1u); EXPECT_EQ(count("files destroy"), 1u);
}
TEST_F(ProcessUnit, AcceptsEmptyInheritedEnvironment) {
  environment({});
  Process child({"tool"}); EXPECT_TRUE(state.environment.empty());
}
TEST_F(ProcessUnit, ConfiguresDistinctStreamsAndPrivateCreationModes) {
  TemporaryDirectory temp;
  ProcessOptions options{temp.path() / "in", temp.path() / "out", temp.path() / "err", {}};
  write(options.input, "binary\ninput");
  Process child({"tool"}, options);
  ASSERT_EQ(state.opens.size(), 3u);
  EXPECT_EQ(state.opens[0].fd, STDIN_FILENO); EXPECT_EQ(state.opens[0].file, options.input); EXPECT_EQ(state.opens[0].flags, O_RDONLY);
  EXPECT_EQ(state.opens[1].fd, STDOUT_FILENO); EXPECT_EQ(state.opens[1].file, options.output);
  EXPECT_EQ(state.opens[2].fd, STDERR_FILENO); EXPECT_EQ(state.opens[2].file, options.error);
  for (std::size_t i : {1u, 2u}) EXPECT_EQ(state.opens[i].flags, O_CREAT | O_WRONLY | O_TRUNC);
  for (const auto &action : state.opens) EXPECT_EQ(action.mode, mode_t{0600});
  EXPECT_EQ(state.input, "binary\ninput"); EXPECT_EQ(state.duplicates, standard_duplicates());
}
TEST_F(ProcessUnit, SharedErrorAndOutputUseDuplicationRatherThanTruncatingTwice) {
  TemporaryDirectory temp;
  ProcessOptions options; options.output = options.error = temp.path() / "combined";
  Process child({"tool"}, options);
  ASSERT_EQ(state.opens.size(), 1u); EXPECT_EQ(state.opens.front().fd, STDOUT_FILENO);
  auto expected = standard_duplicates(); expected.emplace_back(STDOUT_FILENO, STDERR_FILENO);
  EXPECT_EQ(state.duplicates, expected);
}
TEST_F(ProcessUnit, AttributeInitializationFailureOwnsNothing) {
  fails("attributes init", ENOMEM);
  error_contains([] { Process child({"tool"}); }, "spawn attributes:");
  EXPECT_EQ(state.calls, (std::vector<std::string>{"attributes init"}));
  EXPECT_TRUE(state.signals.empty());
}
TEST_F(ProcessUnit, FileActionInitializationFailureReleasesOnlyInitializedAttributes) {
  fails("files init", ENOMEM);
  error_contains([] { Process child({"tool"}); }, "spawn files:");
  EXPECT_EQ(state.calls, (std::vector<std::string>{"attributes init", "files init", "attributes destroy"}));
}
class SpawnSetupFailure : public ProcessUnit, public ::testing::WithParamInterface<std::pair<std::string, std::string>> {};
TEST_P(SpawnSetupFailure, ReleasesBothResourcesAndDoesNotLaunchOrOwnAChild) {
  const auto &[operation, message] = GetParam(); fails(operation, EACCES);
  error_contains([] { Process child({"tool"}); }, message);
  EXPECT_EQ(count("attributes destroy"), 1u); EXPECT_EQ(count("files destroy"), 1u);
  EXPECT_EQ(count("spawn"), 0u); EXPECT_TRUE(state.signals.empty()); EXPECT_TRUE(state.waits.empty());
}
std::vector<std::pair<std::string, std::string>> setup_errors() {
  std::vector<std::pair<std::string, std::string>> errors{
    {"signal mask", "child signal mask:"}, {"signal defaults", "child signals:"},
    {"group", "child group:"}, {"flags", "child flags:"}};
#ifndef __APPLE__
  errors.emplace_back("closefrom", "close inherited child descriptors:");
#endif
  return errors;
}
INSTANTIATE_TEST_SUITE_P(BoundaryErrors, SpawnSetupFailure, ::testing::ValuesIn(setup_errors()));
class StreamSetupFailure : public ProcessUnit, public ::testing::WithParamInterface<int> {};
TEST_P(StreamSetupFailure, ReportsStreamErrorAndReleasesBothResourcesBeforeSpawn) {
  ProcessOptions options; options.input = "/unused/in"; options.output = "/unused/out"; options.error = "/unused/err";
  fails("open " + std::to_string(GetParam()), EACCES);
  error_contains([&] { Process child({"tool"}, options); }, "child stream:");
  EXPECT_EQ(count("attributes destroy"), 1u); EXPECT_EQ(count("files destroy"), 1u); EXPECT_EQ(count("spawn"), 0u);
}
INSTANTIATE_TEST_SUITE_P(EachDescriptor, StreamSetupFailure, ::testing::Values(STDIN_FILENO, STDOUT_FILENO, STDERR_FILENO));
TEST_F(ProcessUnit, DuplicateStreamFailureReleasesBothResources) {
  ProcessOptions options; options.output = options.error = "/unused/combined";
#ifdef __APPLE__
  state.errors["duplicate"] = {0, 0, 0};
#endif
  fails("duplicate", EBADF);
  error_contains([&] { Process child({"tool"}, options); }, "child stderr:");
  EXPECT_EQ(count("attributes destroy"), 1u); EXPECT_EQ(count("files destroy"), 1u); EXPECT_EQ(count("spawn"), 0u);
}
TEST_F(ProcessUnit, SpawnFailureIncludesExecutableAndOwnsNoPid) {
  fails("spawn", ENOENT);
  error_contains([] { Process child({"missing-tool", "arg"}); }, "start missing-tool:");
  EXPECT_EQ(count("attributes destroy"), 1u); EXPECT_EQ(count("files destroy"), 1u);
  EXPECT_TRUE(state.signals.empty()); EXPECT_TRUE(state.waits.empty());
}
TEST_F(ProcessUnit, PollRetriesInterruptedWaitAndCachesExitStatus) {
  state.wait_replies = {{0}, {-1, 0, EINTR}, {owned, exited(23)}};
  Process child({"tool"});
  EXPECT_FALSE(child.poll()); EXPECT_FALSE(child.poll()); EXPECT_EQ(child.poll(), 23); EXPECT_EQ(child.poll(), 23);
  EXPECT_EQ(state.waits, (std::vector<std::pair<pid_t, int>>(3, {42000, WNOHANG})));
  child.stop(0); EXPECT_EQ(state.waits.size(), 3u);
}
TEST_F(ProcessUnit, PollDecodesSignalTerminationAndIgnoresUnrelatedWaitResult) {
  state.wait_replies = {{42001, exited(9)}, {owned, SIGTERM}};
  Process child({"tool"}); EXPECT_FALSE(child.poll()); EXPECT_EQ(child.poll(), -SIGTERM);
}
TEST_F(ProcessUnit, PollReportsNonInterruptedErrorsWithoutCachingAResult) {
  state.wait_replies = {{-1, 0, ECHILD}, {owned, exited(4)}};
  Process child({"tool"}); error_contains([&] { child.poll(); }, "waitpid failed:"); EXPECT_EQ(child.poll(), 4);
}
TEST_F(ProcessUnit, MoveConstructionTransfersOwnershipAndInertSourceDoesNotWaitOrSignal) {
  Process source({"tool"}); Process owner(std::move(source));
  EXPECT_EQ(owner.pid(), 42000); EXPECT_EQ(source.pid(), -1); EXPECT_FALSE(source.poll());
  bool forced = true; EXPECT_EQ(source.stop(0, &forced), 0); EXPECT_FALSE(forced);
  EXPECT_TRUE(state.waits.empty()); EXPECT_TRUE(state.signals.empty());
  EXPECT_EQ(owner.stop(0), 0); EXPECT_EQ(owner.pid(), -1);
  const auto signal_count = state.signals.size(); EXPECT_EQ(owner.stop(0), 0); EXPECT_EQ(state.signals.size(), signal_count);
}
TEST_F(ProcessUnit, MoveConstructionPreservesCachedExitStatusWithoutExtraWait) {
  state.wait_replies = {{owned, exited(17)}};
  Process source({"tool"}); ASSERT_EQ(source.poll(), 17); Process owner(std::move(source));
  EXPECT_EQ(owner.poll(), 17); EXPECT_EQ(source.stop(0), 17); EXPECT_EQ(owner.stop(0), 17); EXPECT_EQ(state.waits.size(), 1u);
}
TEST_F(ProcessUnit, MoveAssignmentCleansOldGroupThenAdoptsNewOwnerAndSupportsSelfMove) {
  Process previous({"previous"}); Process replacement({"replacement"});
  previous = std::move(replacement);
  EXPECT_EQ(previous.pid(), 42001); EXPECT_EQ(replacement.pid(), -1);
  EXPECT_EQ(state.signals, (std::vector<std::pair<pid_t, int>>{{-42000, SIGINT}, {-42000, SIGKILL}}));
  Process *alias = &previous; previous = std::move(*alias);
  EXPECT_EQ(previous.pid(), 42001); EXPECT_EQ(state.signals.size(), 2u);
  EXPECT_EQ(previous.stop(0), 0); EXPECT_EQ(state.signals.back(), std::make_pair(pid_t{-42001}, SIGKILL));
}
TEST_F(ProcessUnit, MoveAssignmentPreservesCachedResultOfReplacement) {
  Process target({"target"}); Process source({"source"});
  state.wait_replies = {{owned, exited(31)}}; ASSERT_EQ(source.poll(), 31);
  target = std::move(source); EXPECT_EQ(target.poll(), 31); EXPECT_EQ(target.stop(0), 31);
}
TEST_F(ProcessUnit, StopSignalsGroupEvenWhenLeaderAlreadyExitedAndResetsForcedFlag) {
  state.wait_replies = {{owned, exited(9)}};
  Process child({"tool"}); ASSERT_EQ(child.poll(), 9);
  bool forced = true; EXPECT_EQ(child.stop(0, &forced), 9); EXPECT_FALSE(forced); EXPECT_EQ(child.pid(), -1);
  EXPECT_EQ(state.signals, (std::vector<std::pair<pid_t, int>>{{-42000, SIGINT}, {-42000, SIGKILL}}));
}
TEST_F(ProcessUnit, StopAllowsMissingGroupAndWaitsForGracefulExit) {
  state.wait_replies = {{0}, {0}, {owned, exited(6)}};
  state.kill_replies = {{-1, ESRCH}};
  state.clock_milliseconds = {0, 1};
  Process child({"tool"}); bool forced = true;
  EXPECT_EQ(child.stop(1, &forced), 6); EXPECT_FALSE(forced);
  EXPECT_EQ(state.waits.size(), 3u); EXPECT_EQ(state.clock_calls, 2u);
  EXPECT_EQ(state.signals.back().second, SIGKILL);
}
TEST_F(ProcessUnit, StopEscalatesAtDeadlineRetriesInterruptedReapAndDecodesSignal) {
  state.wait_replies = {{0}, {0}, {-1, 0, EINTR}, {owned, SIGKILL}};
  state.clock_milliseconds = {100, 100};
  Process child({"tool"}); bool forced = false;
  EXPECT_EQ(child.stop(0, &forced), -SIGKILL); EXPECT_TRUE(forced); EXPECT_EQ(child.pid(), -1);
  EXPECT_EQ(state.waits, (std::vector<std::pair<pid_t, int>>{{42000, WNOHANG}, {42000, WNOHANG}, {42000, 0}, {42000, 0}}));
  EXPECT_EQ(state.signals, (std::vector<std::pair<pid_t, int>>{{-42000, SIGINT}, {-42000, SIGKILL}, {-42000, SIGKILL}}));
}
TEST_F(ProcessUnit, ForcedStopAlsoDecodesNormalExitAndDoesNotRequireForcedPointer) {
  state.wait_replies = {{0}, {0}, {owned, exited(29)}};
  Process child({"tool"}); EXPECT_EQ(child.stop(0), 29);
}
TEST_F(ProcessUnit, StopReportsInterruptFailureAndLeavesOwnershipForDestructor) {
  state.wait_replies = {{0}, {owned, exited(0)}}; state.kill_replies = {{-1, EPERM}};
  {
    Process child({"tool"}); bool forced = true;
    error_contains([&] { child.stop(0, &forced); }, "cannot interrupt owned process group");
    EXPECT_FALSE(forced); EXPECT_EQ(child.pid(), 42000);
  }
  EXPECT_EQ(state.signals.size(), 3u); EXPECT_EQ(state.signals.back().second, SIGKILL);
}
TEST_F(ProcessUnit, FailedForcedReapReportsErrorAndDestructorStillOwnsCleanup) {
  state.wait_replies = {{0}, {0}, {-1, 0, ECHILD}, {owned, exited(0)}};
  {
    Process child({"tool"}); bool forced = false;
    error_contains([&] { child.stop(0, &forced); }, "failed to reap owned process"); EXPECT_TRUE(forced);
    EXPECT_EQ(child.pid(), 42000);
  }
  EXPECT_EQ(state.signals.back(), std::make_pair(pid_t{-42000}, SIGKILL));
}
TEST_F(ProcessUnit, DestructorRecoversFromPollFailureAndRetriesInterruptedReap) {
  state.wait_replies = {{-1, 0, ECHILD}, {-1, 0, EINTR}, {owned, SIGKILL}};
  { Process child({"tool"}); }
  EXPECT_EQ(state.signals, (std::vector<std::pair<pid_t, int>>{{-42000, SIGKILL}}));
  EXPECT_EQ(state.waits, (std::vector<std::pair<pid_t, int>>{{42000, WNOHANG}, {42000, 0}, {42000, 0}}));
}
TEST_F(ProcessUnit, DestructorSwallowsReapFailureInsteadOfThrowing) {
  state.wait_replies = {{-1, 0, EIO}, {-1, 0, ECHILD}};
  EXPECT_NO_THROW({ Process child({"tool"}); }); EXPECT_EQ(state.signals.size(), 1u);
}
TEST_F(ProcessUnit, DestructorDoesNotReapAgainAfterCachedExitAndInterruptFailure) {
  state.wait_replies = {{owned, exited(11)}}; state.kill_replies = {{-1, EPERM}};
  { Process child({"tool"}); EXPECT_EQ(child.poll(), 11); }
  EXPECT_EQ(state.waits.size(), 1u);
  EXPECT_EQ(state.signals, (std::vector<std::pair<pid_t, int>>{{-42000, SIGINT}, {-42000, SIGKILL}}));
}
TEST_F(ProcessUnit, InterruptGuardRecordsSignalsWithoutRaisingAnyAndRestoresBothHandlers) {
  EXPECT_FALSE(interruption_requested());
  {
    InterruptGuard guard;
    ASSERT_EQ(state.handler_calls.size(), 2u);
    EXPECT_TRUE(state.handler_calls[0].saving_previous); EXPECT_TRUE(state.handler_calls[1].saving_previous);
    EXPECT_EQ(state.handlers[SIGINT].sa_flags, 0);
    EXPECT_EQ(sigismember(&state.handlers[SIGINT].sa_mask, SIGTERM), 0);
    state.handlers[SIGINT].sa_handler(SIGINT); EXPECT_TRUE(interruption_requested());
  }
  EXPECT_FALSE(interruption_requested());
  EXPECT_EQ(state.handlers[SIGINT].sa_handler, &process_unit_boundary::original_int);
  EXPECT_EQ(state.handlers[SIGTERM].sa_handler, &process_unit_boundary::original_term);
  ASSERT_EQ(state.handler_calls.size(), 4u);
  EXPECT_FALSE(state.handler_calls[2].saving_previous); EXPECT_FALSE(state.handler_calls[3].saving_previous);
  { InterruptGuard guard; state.handlers[SIGTERM].sa_handler(SIGTERM); EXPECT_TRUE(interruption_requested()); }
  EXPECT_FALSE(interruption_requested());
}
TEST_F(ProcessUnit, GuardInstallFailureDoesNotAttemptTheSecondHandler) {
  fails("sigaction " + std::to_string(SIGINT), EPERM);
  error_contains([] { InterruptGuard guard; }, "cannot install interrupt handler");
  ASSERT_EQ(state.handler_calls.size(), 1u);
  EXPECT_EQ(state.handlers[SIGINT].sa_handler, &process_unit_boundary::original_int);
}
TEST_F(ProcessUnit, GuardTerminationInstallFailureRestoresTheFirstHandler) {
  fails("sigaction " + std::to_string(SIGTERM), EPERM);
  error_contains([] { InterruptGuard guard; }, "cannot install termination handler");
  ASSERT_EQ(state.handler_calls.size(), 3u); EXPECT_EQ(state.handler_calls.back().signal, SIGINT);
  EXPECT_FALSE(state.handler_calls.back().saving_previous);
  EXPECT_EQ(state.handlers[SIGINT].sa_handler, &process_unit_boundary::original_int);
  EXPECT_EQ(state.handlers[SIGTERM].sa_handler, &process_unit_boundary::original_term);
}
TEST_F(ProcessUnit, RunRejectsNonPositiveAndNonFiniteTimeoutsBeforeAnySpawn) {
  for (double timeout : {0.0, -1.0, std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
    error_contains([&] { run({"tool"}, timeout); }, "command timeout must be finite and positive");
  }
  EXPECT_TRUE(state.calls.empty()); EXPECT_TRUE(state.waits.empty());
}
TEST_F(ProcessUnit, RunCapturesSeparateOutputsInputEnvironmentAndCleansOwnedDirectory) {
  state.wait_replies = {{0}, {owned, exited(7)}}; state.clock_milliseconds = {0, 1};
  const auto result = run({"tool", "literal"}, 1, "stdin payload", {{"OVERRIDE", "run value"}});
  EXPECT_EQ(result.code, 7); EXPECT_EQ(result.out, "unit stdout"); EXPECT_EQ(result.err, "unit stderr");
  EXPECT_EQ(state.input, "stdin payload");
  EXPECT_NE(std::find(state.environment.begin(), state.environment.end(), "OVERRIDE=run value"), state.environment.end());
  ASSERT_EQ(state.opens.size(), 3u); EXPECT_FALSE(fs::exists(state.opens.front().file.parent_path()));
  EXPECT_EQ(state.signals, (std::vector<std::pair<pid_t, int>>{{-42000, SIGINT}, {-42000, SIGKILL}}));
}
TEST_F(ProcessUnit, RunTimeoutEscalatesReapsAndCleansOwnedDirectory) {
  state.wait_replies = {{0}, {0}, {0}, {owned, SIGKILL}}; state.clock_milliseconds = {0, 1000, 1000, 1000};
  error_contains([] { run({"slow-tool"}, 1); }, "command timed out: slow-tool");
  EXPECT_EQ(state.signals, (std::vector<std::pair<pid_t, int>>{{-42000, SIGINT}, {-42000, SIGKILL}, {-42000, SIGKILL}}));
  ASSERT_FALSE(state.opens.empty());
  EXPECT_FALSE(fs::exists(state.opens.front().file.parent_path()));
}
TEST_F(ProcessUnit, RunInterruptionUnwindsOwnerAndRestoresGuardState) {
  {
    InterruptGuard guard;
    state.on_spawn = [] { state.handlers[SIGTERM].sa_handler(SIGTERM); };
    state.wait_replies = {{0}, {owned, SIGTERM}};
    error_contains([] { run({"interrupted-tool"}, 1); }, "command interrupted: interrupted-tool");
    ASSERT_FALSE(state.opens.empty());
    EXPECT_TRUE(interruption_requested()); EXPECT_FALSE(fs::exists(state.opens.front().file.parent_path()));
    ASSERT_FALSE(state.signals.empty());
    EXPECT_EQ(state.signals.back(), std::make_pair(pid_t{-42000}, SIGKILL));
  }
  EXPECT_FALSE(interruption_requested());
}
TEST_F(ProcessUnit, RunSpawnFailureStillRemovesTemporaryFiles) {
  fails("spawn", ENOENT);
  error_contains([] { run({"missing-tool"}, 1); }, "start missing-tool:");
  ASSERT_FALSE(state.opens.empty()); EXPECT_FALSE(fs::exists(state.opens.front().file.parent_path()));
  EXPECT_TRUE(state.signals.empty());
}
TEST_F(ProcessUnit, CheckedRunReturnsStdoutOnSuccess) {
  EXPECT_EQ(checked_run({"tool"}, 1), "unit stdout");
}
TEST_F(ProcessUnit, CheckedRunPrefersChildStderrForNonzeroExit) {
  state.wait_replies = {{owned, exited(12)}}; state.error_output = "specific diagnostic";
  error_contains([] { checked_run({"tool"}, 1); }, "specific diagnostic");
}
TEST_F(ProcessUnit, CheckedRunProvidesExecutableFallbackWhenStderrIsEmpty) {
  state.wait_replies = {{owned, exited(12)}}; state.error_output.clear();
  error_contains([] { checked_run({"failing-tool"}, 1); }, "command failed: failing-tool");
}
}  // namespace
