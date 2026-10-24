# 双 MID-360 巡检小车

配套[从克隆仓库到完整仿真教程](../../docs/tutorials/双MID360从零搭建.md)。示例包含独立 description 与 bringup 包，可从本机模板配置 world、URDF 传感器、runtime、ros2_control 和 `bringup.launch.py`。

![双 MID-360 小车实际渲染](../../docs/images/dual-mid360-rover.png)

| 包 | 责任 |
| --- | --- |
| [dual_mid360_description](dual_mid360_description) | 车体、四轮、轮轴、真实 MID-360 CAD、倾斜支架及机械 TF |
| [dual_mid360_bringup](dual_mid360_bringup) | world、仿真插件组合、传感器参数、ROS 参数、RViz 和启动入口 |

这是独立的教学案例。`bringup.launch.py` 使用 `ld = LaunchDescription()`、`ld.add_action(...)` 和 `return ld` 组织启动：单独展开机械 Xacro 并配置 RSP，再通过 `IncludeLaunchDescription` 调用仿真器提供的 `litesim3d_bringup/launch/simulation.launch.py`。`launch_arguments` 显式传入 world、runtime、仿真组合 Xacro、ROS 参数文件和 RViz 配置；RSP 单独使用 `rsp.yaml`。雷达接口与格式放在 `ros.yaml`，话题通过显式 remap 接线。公共仿真入口不创建或等待 RSP，也不绑定它的节点名或命名空间。公共 launch 负责仿真模型展开、场景导入和窗口管理，源 `runtime.yaml` 的 `robot.urdf` 保持空。包路径由 `get_package_share_directory()` 查询，再用 `os.path.join()` 拼接。

在已构建并 source 的 Humble 工作空间：

```bash
export ROS_DOMAIN_ID=71 ROS_LOCALHOST_ONLY=1
ros2 launch dual_mid360_bringup bringup.launch.py
```

默认启动完整工作间和双雷达 RViz。`world:=minimal.sdf` 使用最小场景模板，`gui:=false rviz:=false` 关闭窗口，`lidar_interface:=native` 使用原生 PointCloud2。完整文件内容、修改位置和控制命令见教程。

地面顶面与 base_link 都在 z=0；轮心 z=0.15 m、半径 0.15 m。两台真实 MID-360 CAD 位于车头／车尾中线上，各向外俯倾 15°。立柱、楔块、安装板和 CAD 底面连续贴合；扫描原点约为 (±0.334235, 0, 0.576523) m。倾角在 description 的 `config/geometry.yaml` 配置，楔块与 TF 随之更新。

CAD 来源见[模型来源](dual_mid360_description/meshes/MID360_SOURCE.md)。底盘和支架属于示例设计；运动学仿真不模拟接触力或悬架。

本例默认通过 `ros2_control` 控制：算法向 `/cmd_vel` 发送车体速度，`diff_drive_controller` 通过 `litesim3d_ros2_control/LiteSimSystem` 驱动四轮，`joint_state_broadcaster` 发布实际 `/joint_states`。案例的 `ld` 单独启动管理器并显式传入 `controllers.yaml`；公共仿真 launch 不包含控制管理器。可按教程切换到关节速度或位置控制器。`control.urdf.xacro` 是管理器的机械模型及硬件接口，`simulation.urdf.xacro` 是仿真器的机械模型及传感器插件。

控制管理器和 spawner 通过 `spawn_when_ready.py` 等待仿真就绪且时钟推进。`controller_startup_timeout` 默认 60 墙钟秒；暂停启动时须在预算内恢复或步进仿真，超时会使 launch 失败。
