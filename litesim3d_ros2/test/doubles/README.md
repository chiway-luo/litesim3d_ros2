# Isolated orchestration boundaries

These headers are added only to the named unit targets, before system include
directories. Installed executables still use the real ROS headers and libraries.
ros_client_unit and sim_session_unit include the unchanged production source;
the former records publications and synchronously delivers real ROS messages.
The latter also interposes the runtime Process, executable lookup and preflight
run functions in its own executable. No simulator, DDS peer or child process is
started. Virtual steady time controls application deadlines; real std::thread
locking is retained for the reset contention check.

spawn_when_ready_unit shares this ROS boundary and records execv arguments.
The real readiness state machine executes; execv returns ENOENT without replacing
the test process. Executable lookup is local to this test binary.

render_orchestration_unit substitutes ignition-rendering entrypoints and objects
with the recording backend under litesim3d_gui/test/doubles. It constructs no ROS
node or executor. viewer_unit additionally substitutes ROS Context, Node,
Subscription and Executor through these shared headers; a compile-time marker
rejects accidentally selecting real ROS transport headers. Each viewer case owns
a separate fake context, including the shutdown case. No DDS executor is reachable.

Real JSON, resource decoding, ignition math, ROS message definitions and Qt
offscreen widgets remain in use. Tests assert API arguments, serialized pixels
and actual widget state; no Ogre/GPU or display server is used.

This describes these new targets only. Existing sim_node_unit/world_node_unit
tests use real in-process rclcpp entities with direct callback delivery and
injected application/backend/publication hooks; they are not claimed to replace
every rclcpp object with a mock.

No production condition, function body, coverage denominator or inventory source
is removed. The full runner builds real executables and retains their zero-hit
mappings, including branches unavailable through a unit boundary. Each unit ELF
and shared runtime library keeps independent raw profiles and build-ID attribution.
