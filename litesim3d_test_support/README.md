# Shared simulation test support

This package installs `litesim3d_test_support`, a Python module for tests of
the LiteSim single-robot topic and service contract. Client packages declare
it as a `test_depend`; running the simulator does not require this module.

```python
from litesim3d_test_support import SimulationHarness

rig = SimulationHarness(tmp_path, package="my_robot_sim", launch_file="robot.launch.py")
try:
    rig.start(paused="true")
    result = rig.step(20, "first-step")
    assert result.success
finally:
    rig.close()
```

The client selects its own installed launch package. The harness observes
status, clock, standard sensors and TF, waits for DDS matching and shuts down
its owned launch process group. Reliable-sensor checks expect the example's
`config/ros.yaml` layout, or an explicit `params` path supplied to `start()`.
The default domain pool is 60..99, separate from the bridge tests' 10..59 pool and below Linux's ephemeral-port overlap. Tests within each pool run sequentially.

`LWS_TEST_DOMAIN_BASE` selects the domain range; the older
`LWS_4W_TEST_DOMAIN_BASE` remains a fallback for existing runners.

Tests must import this installed module rather than adding another package's
test source directory to `sys.path`.

Each launched run records its command, process IDs and ROS domain in
`launch-process.json`. `graph-before-shutdown.json` captures publisher counts,
node names and DDS endpoint GIDs before its process group is stopped. Composed
chassis tests also save `tf-graph.json` at the exact TF ownership assertion.
These records diagnose graph failures without relaxing the publisher count or
automatically retrying a failed test.

Cleanup signals and waits for the entire owned process group even if its launch
leader has already exited. `exit.json` records that state, whether forced
termination was needed, and whether any group members remain. A failed model
import must not leave an RSP writer behind for the next test using the same
domain; `scripts/integration/test_launch_cleanup.py` exercises that sequence.
