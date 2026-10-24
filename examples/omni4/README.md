# 四轮普通全向 ros2_control 示例

构建并 source 安装环境后执行：

```bash
ros2 launch omni4_sim omni4.launch.py gui:=false
```

另一个已 source 的终端发送车体速度：

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2, y: 0.3}, angular: {z: 0.2}}'
```

默认由 `litesim3d_ros2_control/OmniDriveController` 接收速度并写入 LiteSimSystem 的关节接口；停止发布后 0.5 仿真秒停车。公共仿真 launch 不创建 RSP 或控制器，案例使用 `ld` 单独配置这些节点并显式传入参数文件。

`geometry.yaml` 是机械 Xacro、仿真底盘和编码器里程计共用的几何来源；`controllers.yaml` 的尺寸/数组须与其一致。`robot.urdf.xacro` 用于 RSP，`control.urdf.xacro` 声明控制硬件，`simulation.urdf.xacro` 声明仿真底盘和传感器插件；runtime 的模型路径留空，由 launch 动态定位。

本例使用仿真编码器里程计 `/odom` 和 odom TF；`joint_state_broadcaster` 是 `/joint_states` 的唯一发布者。碰撞后车体受阻，默认驱动轮和编码器里程计继续累计；`robot.continue_on_collision: false` 可停止驱动累计。这是普通全向轮模型，支持横移与原地旋转。

省略 `gui:=false` 打开三维窗口；`rviz:=true` 打开 RViz；`cameras:=true` 开启前向 RGB，需要可用图形后端。world 提供固定地面、前墙和侧墙，车轮贴地；传感器跟随实际车体和关节。

完整的 world、URDF、控制器、launch 和第三方算法接管流程见 [阿克曼与全向轮接入 ros2_control](../../docs/tutorials/阿克曼与全向轮ros2_control.md)。
