^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dual_mid360_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

* Wait for simulator readiness and advancing simulation time before starting the standard controller manager and spawner; fail within controller_startup_timeout if startup cannot complete.

* Use ld.add_action to configure the mechanical Xacro and RSP, then include LiteSim's simulation.launch.py with explicit model/parameter files and topic remaps; configure RSP separately with rsp.yaml.

* Resolve the model through the package index in bringup.launch.py and leave robot.urdf empty in source runtime YAML.

* Add a standalone simulation bringup package with bringup.launch.py, sensor configuration, and grounded workshop and minimal world templates.
