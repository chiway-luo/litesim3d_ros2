# Common simulation bringup

`simulation.launch.py` owns the SDF import, simulator/RSP lifecycle, optional
renderer, viewer, RViz and temporary-resource cleanup. It has no dependency on
an example package. Example launch files supply their own defaults through
`IncludeLaunchDescription` using the installed ament package index.

The complete argument list, defaults and example overrides are maintained in the
[launch and node parameter reference](../docs/api/启动与节点参数.md).
For model setup and operating steps, see the [user manual](../docs/usage/README.md).

All resource paths belong to the supplied model/configuration directories or
installed packages. No source checkout is needed at runtime. For a terminal
session set `gui:=false cameras:=false`; camera rendering requires a working
Ogre2 graphics backend even when no viewer window is requested.
