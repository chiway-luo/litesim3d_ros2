^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package litesim3d_core
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Prepared for the first planned 0.4.0 release. Entries consolidate development
history; no earlier tagged releases or publication dates are implied.

Forthcoming
-----------
* Expose the existing pending-command state in diagnostics so paused-step clients can observe command acceptance; test rejection preserving the queued command and exactly-once application without changing simulator layout or stepping semantics.
* Read the compiled version from package-local metadata; declare pkg-config, libssl-dev and GTest for independent release builds.
* Introduce a standalone C++17 kinematic simulation core with strict YAML configuration, URDF loading, STL/OBJ/DAE geometry, ray queries, geometric collision checks, deterministic stepping, pause and initialization controls.
* Add continuous terrain support, per-point rolling LiDAR scans, seeded noise and dropout, raw IMU samples, independent odometry error and bounded resource checks.
* Add moving obstacles, one to four homogeneous robots in a shared world, isolated random streams and ideal Ackermann kinematics.
* Add spinning, spherical and rosette scan patterns and the fixed upstream Livox Mid-360 direction table at 200,000 directions per second. Preserve the four-second table period, per-point timing, provenance and MIT notice.
* Fix shared-world dynamic-contact ordering, terminal fault handling and URDF-derived wheel/support sampling; preserve independent robot trajectories and random streams.
* Add optional scene complexity budgets, measured real-time-factor reporting and exact camera sampling events for the separate renderer.
* Add level-body multicopter translation and yaw kinematics, combined velocity/acceleration limits, command-timeout braking and three-dimensional collision constraints.
* Set the maintainer contact to chiwayfpv@gmail.com and provide the repository Apache-2.0 license; retain existing third-party notices.

References: `Source 4 <test/test_review_fixes.cpp>`_.
