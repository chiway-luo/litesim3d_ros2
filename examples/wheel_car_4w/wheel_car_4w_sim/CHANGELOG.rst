^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package wheel_car_4w_sim
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Prepared for the first planned 0.4.0 release. Entries consolidate development
history; no earlier tagged releases or publication dates are implied. See the
`repository development history <../../../CHANGELOG.rst>`_ for chronology and evidence.

Forthcoming
-----------

* Wait for the viewer's own subscriptions and a statistics sample before checking its graph or closing it; simulator readiness alone does not imply renderer discovery.
* Drain RGB-D validation samples in bounded step batches so exact Image/CameraInfo checks respect the publisher's KEEP_LAST(5) history. Preserve full timestamp, calibration and idempotent-retry assertions.
* Use the installed common bringup and test-support modules; keep only car defaults and car-specific tests in this package.
* Introduce one-command four-wheel simulation with annotated runtime/ROS parameters, robot_state_publisher and RViz integration.
* Add VLP-16, Mid-360 and Mid-40 presets, rolling-scan settings and an optional synthetic range-boundary visualization kept separate from raw sensor returns.
* Fix RViz camera tools, mimic wheel-state coverage, DDS readiness for one-shot clouds and propagation of required child-process failures.
* Add validated SDF startup, independent viewer/camera switches, configurable RGB/depth modes, target pacing and optional collision budgets.
* Use the furnished office as the default world and retain the warehouse regression scene.
* Resolve car models and worlds through wheel_car_4w_models after splitting the original shared litesim_models package.
* Add installed-resource, control-timeout, TF/QoS, pause/step, camera lifecycle and scene-viewer reconnect regression coverage.
* Set the maintainer contact to chiwayfpv@gmail.com and provide the repository Apache-2.0 license; retain existing third-party notices.

References: `Source 1 <../README.md>`_, `Source 2 <../../../docs/testing/四轮车与雷达测试.md>`_, `Source 3 <../../../docs/testing/SDF实现验证.md>`_, `Source 4 <../../../docs/testing/无人机与办公场景验证.md>`_.
