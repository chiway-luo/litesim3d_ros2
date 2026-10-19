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
