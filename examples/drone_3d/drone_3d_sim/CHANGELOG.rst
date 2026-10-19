^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package drone_3d_sim
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Prepared for the first planned 0.4.0 release. Entries consolidate development
history; no earlier tagged releases or publication dates are implied. See the
`repository development history <../../../CHANGELOG.rst>`_ for chronology and evidence.

Forthcoming
-----------
* Remove the dependency on the car example; use common installed bringup/test-support modules with explicit dependencies.
* Introduce a one-command multicopter example with a quadrotor model, a three-dimensional flight course and annotated ROS/runtime configuration.
* Configure level-body three-axis velocity and yaw control, initial hover, command-timeout stopping and ground/obstacle collision constraints.
* Reuse the SDF, viewer and optional RGB/depth camera startup pipeline without adding fictitious wheel joints or flight-controller dynamics.
* Resolve the drone world and model resources through the dedicated drone_3d_models package after the asset split.
* Add ROS and optional desktop checks for three-dimensional motion, TF/IMU/cloud output, pause/timeout behavior, ground clearance and RGB-D rendering.
* Set the maintainer contact to chiwayfpv@gmail.com and provide the repository Apache-2.0 license; retain existing third-party notices.

References: `Source 1 <../README.md>`_, `Source 2 <../../../docs/testing/无人机与办公场景验证.md>`_.
