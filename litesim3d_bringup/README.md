# Common simulation bringup

`simulation.launch.py` owns SDF import, the simulator, optional renderer,
viewer, RViz and temporary-resource cleanup. It does not create or wait for RSP.
The caller owns its RSP name, namespace, mechanical model and parameters.

Resolve the simulation URDF/Xacro with `get_package_share_directory()` and
`os.path.join()`, then pass it as `robot_urdf`, together with explicit `runtime`
and `params` file paths. The source YAML's `robot.urdf` may remain empty.
LiDAR output settings come from `params`; nonempty launch arguments override them.

Use `joint_states_topic`, `tf_topic`, `tf_static_topic` and
`robot_description_topic` to connect the simulator/RViz to the caller's RSP.
Apply the corresponding remaps to RSP itself. Other ROS topics accept normal
ROS remapping, including scoped `SetRemap` actions around the include.

After preparation, `prepared_config` in the launch context contains the generated
runtime path. SDF-only example launch files can read its model after the include
and create their own RSP. No simulation logic depends on the RSP node's identity.

The complete argument list, defaults and example overrides are maintained in the
[launch and node parameter reference](../docs/api/启动与节点参数.md).
For model setup and operating steps, see the [user manual](../docs/usage/README.md).

All resource paths belong to the supplied model/configuration directories or
installed packages. No source checkout is needed at runtime. For a terminal
session set `gui:=false cameras:=false`; camera rendering requires a working
Ogre2 graphics backend even when no viewer window is requested.

For ros2_control, set runtime `simulation.control_mode: ros2_control` and start
`controller_manager` with `litesim3d_ros2_control/LiteSimSystem` in your own bringup.
The simulator receives joint targets and returns actual states; the caller's
`joint_state_broadcaster` publishes public joint states. Use `joint_commands_topic`
and `joint_feedback_topic` for explicit simulator remaps, then configure the
hardware's `command_topic` and `state_topic` to match. The shared launch starts
neither controller_manager nor a joint-state publisher/broadcaster.
See the [hardware interface](../litesim3d_ros2_control/README.md) and
[four-wheel example](../examples/wheel_car_4w/README.md).
