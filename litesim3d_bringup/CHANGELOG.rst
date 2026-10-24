^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package litesim3d_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

* Remove built-in RSP ownership. Callers configure RSP themselves; add joint-state/TF/model-topic remaps and honor LiDAR output settings from the ROS parameter file unless explicitly overridden.
* Accept launch-time robot_urdf and prepare_config --robot-urdf overrides so model paths can be resolved through the package index while source YAML remains empty.
* Encode prepared mesh and texture paths as file URLs for RViz without changing simulator resource paths or mechanical transforms.
* Map RViz previews to per-sensor Livox topics for multiple active MID-360s; preserve single-sensor topics and ignore disabled plugin instances.
* Initial 0.4.0 release preparation: extract reusable, independently installable modules.
