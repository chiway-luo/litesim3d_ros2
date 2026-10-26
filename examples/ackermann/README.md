# 阿克曼 ros2_control 示例

先完成[安装与构建](../../docs/usage/安装与构建.md)，两个终端加载同一安装环境和 ROS domain。

终端 A：

```bash
ros2 launch ackermann_sim ackermann.launch.py gui:=false
```

终端 B：

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.4}, angular: {z: 0.2}}'
```

小车应按输入运动，`/odom` 和 `/joint_states` 持续更新。Ctrl+C 停止发布后，控制器默认在 0.5 仿真秒输入超时后停车；终端 A 按 Ctrl+C 结束仿真。

后轮接收速度，左右转向关节接收位置；前轮旋转仅提供反馈。转角范围 ±0.6 rad，不支持原地转向。

机械尺寸在 `urdf/mechanical.urdf.xacro` 中定义，`config/drive.yaml` 只绑定关节；仿真、里程计和控制器自动从完整 URDF 读取几何。`controllers.yaml` 配置控制设置，`runtime.yaml` 配置支撑和碰撞。出生位姿从 `config/ros.yaml` 的 `initial_pose_override` 加载；自定义文件用 `params:=绝对路径` 传入，格式见[初始位姿](../../docs/usage/接入与使用手册.md#初始位姿和运行中重置)。修改源码配置后重新构建并启动。

本例使用仿真编码器里程计 `/odom` 和 odom TF，joint_state_broadcaster 发布 `/joint_states`，RSP 发布内部 TF。默认碰撞时驱动轮仍可空转；`robot.continue_on_collision: false` 可停止驱动累计。

省略 `gui:=false` 打开三维窗口，`rviz:=true` 开启 RViz；`cameras:=true` 启用相机，需要图形后端。完整配置和第三方算法接管见[阿克曼与全向轮教程](../../docs/tutorials/阿克曼与全向轮ros2_control.md)。
