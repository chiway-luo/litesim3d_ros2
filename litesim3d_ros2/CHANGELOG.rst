^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package litesim3d_ros2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Prepared for the first planned 0.4.0 release. Entries consolidate development
history; no earlier tagged releases or publication dates are implied.

Forthcoming
-----------
* Remove single/world startup gating on RSP names and namespaces; keep viewer/camera readiness and duplicate odometry TF detection. Cover absent/late RSP and explicit remaps across namespaces.
* Share paused-command acceptance and target-timestamp waits across ROS tests instead of fixed sleeps; observe per-robot pending/rejected command diagnostics before stepping and retain the original deadlines and motion assertions.
* Wait for the target step timestamp on every robot's odometry before comparing shared-world poses; preserve exact position/cloud assertions and record the observed timestamps.
* Retain and retry timed-out service responses in a shared bounded module for single/world nodes, preserving the original request identity and result without re-executing commands. Add Linux fault-injection coverage for recovery, expiry, capacity, fatal errors and shutdown; keep the original reset stress test and client deadline.
* Declare the directly used RCL dependency and include response handling in the bridge source fingerprint.
* Bound every slow-consumer poll by the configured sensor history depth and scan period; record system-clock steps so a broad timestamp span cannot hide old-frame backlog.
* Retry unanswered read-only member readiness probes within the original startup deadline, cleaning pending requests and still requiring an actual executor response before reset succeeds.
* Wait for an observed clock before reading its timestamp in stamped-command and headless tests; ready status does not guarantee clock callback order.
* Use low ROS domain IDs in default tests and launch checks to avoid Linux ephemeral UDP port overlap.
* Make the single-cloud PCL payload test deterministic during DDS discovery: use reliable input and request the retained probe result.
* Observe initialized waiting status in the delayed-dependency startup test and use the normal startup budget; retain the separate short-deadline fault test.
* Isolate the slow consumer in its own ROS context and drive queue checks by actual produced frames and cloud callbacks; retain memory, replay, timestamp and 30-second pause assertions and save per-poll evidence.
* Declare RMW, JSON and test dependencies directly; register package-local ROS/launch tests and move repository tooling checks to integration tests.
* Introduce ROS 2 nodes, launch files and RViz configuration for simulation clock, point clouds, odometry, TF, joint states, scene markers, diagnostics and pause/step control.
* Add IMU publishing, coordinated process/session reset, shared-world multi-robot topics, TwistStamped input, configurable sensor QoS and per-robot odometry TF ownership checks.
* Add an Ouster original-field adapter and an actual PCL consumer while retaining the native point-cloud layout and no-return NaN slots.
* Fix reset readiness checks to wait for responsive member executors, matching subscription types/QoS, current run status and DDS endpoint matches; preserve relative LiDAR pattern resources when relocating configurations.
* Fix Humble/GCC empty-vector parameter initialization, observer graph readiness and reliable large-cloud single-step checks.
* Add static scene descriptions, live state/statistics snapshots, exact camera render requests and multicopter velocity support.
* Fix wall-clock status publication during unpaced simulation and improve test callback fairness, startup readiness and shutdown observation under sensor load.
* Set the maintainer contact to chiwayfpv@gmail.com and provide the repository Apache-2.0 license; retain existing third-party notices.
