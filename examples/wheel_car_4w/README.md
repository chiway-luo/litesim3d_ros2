# 四轮小车仿真

在办公场景中运行四轮差速小车，提供雷达、IMU、里程计、ros2_control 和可选三维窗口。先完成[安装与构建](../../docs/usage/安装与构建.md)。

## 启动和控制

终端 A，从源码仓库根目录执行：

```bash
source /opt/ros/humble/setup.bash
source "$(python3 scripts/paths.py install)/setup.bash"
export ROS_DOMAIN_ID=70 ROS_LOCALHOST_ONLY=1
ros2 launch wheel_car_4w_sim wheel_car_4w.launch.py
```

默认打开三维窗口，启动 RSP、controller_manager、diff_drive_controller 和 joint_state_broadcaster。场景为 28 × 22 m 办公区；相机与 RViz 默认关闭。

终端 B 加载相同环境和 domain，发送速度：

```bash
source /opt/ros/humble/setup.bash
source "$(python3 scripts/paths.py install)/setup.bash"
export ROS_DOMAIN_ID=70 ROS_LOCALHOST_ONLY=1
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.3}, angular: {z: 0.2}}'
```

正 x 向前，正 z 角速度为俯视逆时针。Ctrl+C 停止发布后，小车按控制器限值减速；默认输入超时为 0.5 仿真秒。可显式停车并查看反馈：

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0}, angular: {z: 0.0}}'
ros2 topic echo /odom --once
ros2 topic echo /points_raw --once --field header --qos-reliability best_effort
```

完整通信与位移检查见[启动与控制](../../docs/tutorials/启动与控制.md)。终端 A 按 Ctrl+C 结束仿真；仅关闭三维窗口时仿真继续。同一 domain 每次运行一个世界。

## 选择场景、雷达和窗口

每次停止旧 launch，再按需添加参数：

| 参数 | 用途 |
| --- | --- |
| `gui:=false` | 关闭三维窗口 |
| `rviz:=true` | 查看机器人、点云、TF 和轨迹 |
| `world:=warehouse.sdf` | 使用小型仓库；默认 `office.sdf` |
| `lidar_profile:=vlp16` / `mid360` / `mid40` | 切换[雷达预设](雷达配置说明.md)；默认 `config` |
| `lidar_interface:=native` | 使用原生 PointCloud2 |
| `cameras:=true` | 开启默认 640 × 480、15 Hz RGB 相机 |
| `camera_view:=true` | 打开已启用相机的图像预览 |
| `real_time_factor:=0.5` | 目标半速；0 表示不限速 |
| `noise_config:=绝对路径` | 按传感器实例注入[测量误差](../../docs/usage/传感器配置.md) |
| `controllers:=绝对路径` | 使用自己的控制器参数 |

相机即使在 `gui:=false` 时也需要可用的 Ogre2 渲染环境。深度/RGB-D 类型、光学坐标和输出话题见[相机说明](../../docs/usage/SDF与独立窗口.md#车载-rgb-与深度相机)。

全部参数及默认值见[启动参数](../../docs/api/启动与节点参数.md)，也可查询：

```bash
ros2 launch wheel_car_4w_sim wheel_car_4w.launch.py --show-args
```

## 修改模型和配置

| 文件 | 内容 |
| --- | --- |
| [office.sdf](wheel_car_4w_models/worlds/office.sdf) | 场景、灯光、步长与目标速率 |
| [wheel_car_4w.urdf](wheel_car_4w_description/urdf/wheel_car_4w.urdf) | 机械结构、轮子几何和关节限位 |
| [robot.urdf.xacro](wheel_car_4w_description/urdf/robot.urdf.xacro) | RSP 使用的机械模型与传感器安装 |
| [simulation.urdf.xacro](wheel_car_4w_description/urdf/simulation.urdf.xacro) | 仿真插件组合入口 |
| [drive.yaml](wheel_car_4w_description/config/drive.yaml) | 独立后轮的左右绑定，尺寸由 URDF 自动读取 |
| [sensors.yaml](wheel_car_4w_description/config/sensors.yaml) / [sensors.xacro](wheel_car_4w_description/urdf/sensors.xacro) | 传感器参数 / 插件声明 |
| [runtime.yaml](wheel_car_4w_sim/config/runtime.yaml) | 地形、支撑轮、控制模式与碰撞策略 |
| [ros.yaml](wheel_car_4w_sim/config/ros.yaml) / [rsp.yaml](wheel_car_4w_sim/config/rsp.yaml) | 出生位姿与仿真节点参数 / RSP 参数 |
| [controllers.yaml](wheel_car_4w_sim/config/controllers.yaml) / [control.urdf.xacro](wheel_car_4w_sim/urdf/control.urdf.xacro) | 控制器参数 / 硬件接口 |

修改源码资源后重新构建并启动。自定义模型用 `robot_urdf`、`rsp_model` 分别传给仿真和 RSP；两份模型中的关节与安装外参须一致。自定义场景用 `assets_root`、`world`，运行参数用 `runtime`、`params`。出生位姿在 ROS 参数文件的 `initial_pose_override` 中设置，用 `params:=绝对路径` 加载；格式见[初始位姿](../../docs/usage/接入与使用手册.md#初始位姿和运行中重置)。参数仅在启动时读取。

算法默认向 diff_drive_controller 的 `/cmd_vel` 发送 Twist。改为 TwistStamped 时，在 controllers.yaml 设 `use_stamped_vel: true`，并把 launch 中控制器输入 remap 从 `cmd_vel_unstamped` 改为 `cmd_vel`；使用仿真时间。仿真节点的 `cmd_mode` 不改变控制器输入类型。

独立轮速接管见[统一控制教程](../../docs/tutorials/全部模型与第三方算法ros2_control.md)。旧 URDF/YAML 直接 Twist 入口使用 `legacy_twist.launch.py`，其配置见[兼容说明](../../docs/usage/兼容配置.md)。

## 数据与坐标

| 话题 | 默认内容 |
| --- | --- |
| `/points_raw` | 16 线 × 512 列、10 Hz rolling PointCloud2，frame=`lidar_link` |
| `/points_raw/range_boundary` | 最远量程辅助点，不代表障碍物回波 |
| `/imu/data_raw` | 200 Hz raw IMU，frame=`imu_link` |
| `/odom` | 50 Hz 编码器里程计，pose 在 odom，twist 在 base_link |
| `/joint_states` | 实际轮角/轮速，前轮 mimic 同侧后轮 |
| `/clock`、`/tf`、`/tf_static`、`/robot_description` | 仿真时间、坐标与模型 |
| `/sim/ground_truth/wheel_car_4w/odom` | 世界位姿真值，用于独立评测 |

点云 `time` 是相对 header 的秒偏移，rolling 云尚未去畸变；无回波点为 NaN。算法设置 `use_sim_time: true`，点云和 IMU 使用 SensorDataQoS。数据格式见[传感器数据](../../docs/api/传感器数据.md)。

轮半径 0.12 m、左右轮距 0.58 m。后轮独立驱动，前轮按 1:1 mimic 同侧后轮；模型采用差速等效运动学。底盘、里程计和控制器均从完整 URDF 读取几何，`drive.yaml` 不重复填写尺寸。

```text
odom -> base_footprint -> base_link -> 四个 wheel_link
                                 ├-> lidar_mount_link -> lidar_link
                                 ├-> imu_link
                                 └-> camera_link -> camera_optical_frame
```

仿真发布 `odom → base_footprint`，RSP 发布内部 TF。不要再发布 `odom → base_link`。水平静止时 raw IMU 的 z 比力约 +9.81 m/s²，未提供姿态估计。

默认 `robot.continue_on_collision: true`：车体被障碍挡住时，驱动轮和编码器里程计仍可累计；雷达、相机、IMU 和真值使用受阻后的实际状态。设为 false 时只累计碰撞前运动。障碍物固定，不模拟轮胎受力或悬架。

## 暂停与单步

启动时加 `paused:=true`；控制器等待仿真时间推进，需在启动超时前恢复或单步。已运行的仿真可以先暂停，再推进 20 步：

```bash
ros2 service call /sim/pause std_srvs/srv/SetBool '{data: true}'
ros2 service call /sim/step litesim3d_interfaces/srv/Step '{request_id: "car-step-001", steps: 20}'
ros2 service call /sim/pause std_srvs/srv/SetBool '{data: false}'
```

默认步长 5 ms，20 步推进 0.1 仿真秒。新操作换 request_id；相同 ID 和步数只返回原结果。回到起点时停止完整 launch 后重启；录包和排障见[记录与排障](../../docs/usage/记录与排障.md)。
