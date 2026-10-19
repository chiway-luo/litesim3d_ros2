# Common simulation bringup

`simulation.launch.py` owns the SDF import, simulator/RSP lifecycle, optional
renderer, viewer, RViz and temporary-resource cleanup. It has no dependency on
an example package. Example launch files supply their own defaults through
`IncludeLaunchDescription` using the installed ament package index.

| Argument | Contract |
| --- | --- |
| `params` | Required ROS parameter YAML |
| `assets_root` | Required in SDF mode; directory containing `models/` and `worlds/` |
| `world` | Required in SDF mode; world filename or explicit SDF path |
| `runtime` | Required in SDF mode; drive, sensor and entity bindings YAML |
| `config` | Optional legacy simulation YAML; selects legacy mode when nonempty |
| `lidar_profiles_dir` | Directory used by the named `vlp16`, `mid360`, `mid40` profiles |
| `lidar_profile` | `config` retains configuration; otherwise a preset name or YAML path |
| `gui`, `rviz`, `camera_view` | Window toggles; `gui` defaults to true, others false |
| `cameras`, `paused`, `real_time_factor`, `manifest` | Empty preserves the input configuration |
| `collision_budget_enabled` | SDF import budget toggle, default false |
| `rviz_config` | Optional RViz template; defaults to the bridge's installed template |

All resource paths belong to the supplied model/configuration directories or
installed packages. No source checkout is needed at runtime. For a terminal
session set `gui:=false cameras:=false`; camera rendering requires a working
Ogre2 graphics backend even when no viewer window is requested.
