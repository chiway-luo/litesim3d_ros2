# ROS 2 接口

适用版本以仓库 [VERSION](../../VERSION) 为准。操作步骤见 [使用手册](../usage/README.md)。

## 单机话题表

下表使用根命名空间和基础单机配置。传感器话题可以由 YAML 改名。

| 话题 | 类型 | 方向/频率 | 发布 QoS |
| --- | --- | --- | --- |
| `/cmd_vel` | geometry_msgs/msg/Twist 或 TwistStamped | 控制器→仿真；建议持续 10 Hz 以上，按超时配置调整 | 仿真订阅 reliable、depth 1 |
| `/points_raw` | sensor_msgs/msg/PointCloud2 | 仿真→消费端；雷达 rate_hz | best effort、depth 5，可切换 reliable |
| `/imu/data_raw` | sensor_msgs/msg/Imu | 配置 IMU 后发布，rate_hz | 同点云 |
| `/odom` | nav_msgs/msg/Odometry | 仿真→估计器/控制器；odom_rate | reliable、depth 10 |
| `/joint_states` | sensor_msgs/msg/JointState | 仿真→RSP；随仿真状态更新 | reliable、depth 10 |
| `/robot_description` | std_msgs/msg/String | URDF 描述 | reliable、transient local、depth 1 |
| `/clock` | rosgraph_msgs/msg/Clock | 唯一仿真时钟 | best effort、depth 1 |
| `/tf`、`/tf_static` | tf2_msgs/msg/TFMessage | 桥接和 RSP 按职责发布 | 使用 TF/RSP 的 QoS，消费者用标准 TF API |
| `/scene` | visualization_msgs/msg/MarkerArray | 单机环境数据；内置 RViz 视图不加载此层 | reliable、transient local、depth 1 |
| `/trajectory` | nav_msgs/msg/Path | 单机轨迹显示 | reliable、depth 1 |
| `/sim/status` | diagnostic_msgs/msg/DiagnosticArray | 诊断，按墙钟周期更新，暂停仍可读 | reliable、depth 10 |
| `/sim/ground_truth/robot_0/odom` | nav_msgs/msg/Odometry | world 真值，仅评测 | reliable、depth 10 |
| `/sim/ground_truth/robot_0/contacts` | litesim3d_interfaces/msg/Contact | 碰撞事件，仅评测 | reliable、depth 10 |

`namespace:=robot` 使相对话题变为 `/robot/cmd_vel`、`/robot/points_raw` 等；`/clock`、`/sim/status`、服务和真值通道仍是绝对名称。单机 `robot.id` 决定真值路径，不自动跟随 namespace。

原生点云表适用于 `lidar_interface=native` 或非 Mid-360 雷达。选择 auto/livox 的 Mid-360 使用下列独立输出，原配置的点云话题不再发布该雷达的数据，其他雷达和 IMU 不受影响。

| 话题 | 类型 / 条件 | QoS |
| --- | --- | --- |
| `livox/lidar` | xfer_format=1：livox_ros_driver2/msg/CustomMsg；0：sensor_msgs/msg/PointCloud2 | reliable、volatile、KEEP_LAST 256 |
| `livox/lidar/pointcloud` | xfer_format=1 时同时发布 Livox 布局 PointCloud2，供 RViz 等消费 | 同上 |
| `livox/lidar/range_boundary` | 显式开启边界显示时发布 Livox 布局 PointCloud2 | 同上 |
| `<lidar.topic>/range_boundary` | 原生雷达显式开启时发布 xyzirt_v1；显示用途 | 同原生点云 |
| `<camera.topic>/color/image_raw` | RGB/RGB-D：sensor_msgs/msg/Image | sensor_reliable 控制，KEEP_LAST 5 |
| `<camera.topic>/color/camera_info` | RGB/RGB-D：sensor_msgs/msg/CameraInfo | 同图像 |
| `<camera.topic>/depth/image_raw` | Depth/RGB-D：sensor_msgs/msg/Image | 同图像 |
| `<camera.topic>/depth/camera_info` | Depth/RGB-D：sensor_msgs/msg/CameraInfo | 同图像 |

上表名称是相对话题；单机根命名空间下如 `/livox/lidar`，多机如 `/r0/livox/lidar`。Livox 的可靠性不由 sensor_reliable 改写。字段和单位见 [传感器数据](传感器数据.md)。

## 速度输入

每个机器人只有一个 cmd_vel 输入类型，由 cmd_mode 选择。Twist/带时间戳 TwistStamped 的 linear 单位为 m/s，angular 为 rad/s，坐标均为配置的机体 base_link。

| 运动模型 | 可用分量 | 约束 |
| --- | --- | --- |
| 差速 | linear.x、angular.z | linear.y/z、angular.x/y 必须为 0 |
| Ackermann | linear.x、angular.z | 曲率受轴距与最大转角限制，不能原地转向 |
| 三轮/四轮全向插件 | linear.x/y、angular.z | 无垂直、roll/pitch 控制 |
| 兼容 multicopter | linear.x/y/z、angular.z | 速度控制，保持水平；无飞行动力学 |

NaN、Inf、不支持的分量、非法时间和 frame 会拒绝，且不刷新停车超时。有效命令按模型限制裁剪并按加减速度推进；暂停时保留待执行命令，时间超时也暂停。停止持续发布后，超过 command_timeout 仿真秒目标速度归零。

## 时钟与 TwistStamped

每次新实验从 1 ns 仿真时间开始。所有数据时间戳来自同一仿真时间轴；所有需要时间的算法节点设置 `use_sim_time: true`。一个 domain 只能有一个 `/clock` 发布源；并行启动两份单机仿真会触发故障，应改用 world 或不同 domain。

Stamped 命令必须使用当前仿真时钟，而不是 `time.time()`、电脑日期或 ROS 默认墙钟。时间戳必须不晚于当前仿真时间，且不早于 `当前时间−command_timeout`。多机要求 frame 精确匹配，如 `r0/base_link`；单机使用配置的 base_link；带前缀名称由桥接移除前缀后交给内核校验，空 frame 也可被单机接受；接入端应固定发送准确的带前缀 base_link。

[示例 ros_client.py](../examples/ros_client.py) 的 `--stamped` 模式已启用 use_sim_time，并等待时钟有效后发送；可以直接参考其写法。暂停时基于 ROS 时间的 timer 不会继续触发，必要的运行监测/超时应明确使用墙钟。

## TF 发布责任

默认 R01 单机 TF：

```text
odom ──仿真桥接──> base_link ──robot_state_publisher──> lidar_link
                            ├──────────────────────> left_wheel
                            └──────────────────────> right_wheel
```

实际桥接发布的是 `odom → URDF root`。如果 URDF root 是 `base_footprint`，则链为 `odom → base_footprint → base_link → 传感器`；root 到 base 的固定偏移由 RSP 发布。里程计消息 child_frame_id 仍为配置的 base_link。

| TF 边 | 谁负责 |
| --- | --- |
| URDF 内部各 link | robot_state_publisher，消费 joint_states |
| odom → URDF root | 默认仿真桥接；外部估计器接管时关闭桥接这一条 |
| map → odom | 外部定位/SLAM；仿真不注入真值 map TF |
| world → 每个 odom | 当前没有提供通用发布；真值留在评测通道 |

外部估计器应尊重 URDF root/base 关系，不能在已有 `root → base_link` 的情况下再给 base_link 添加一个不同父节点。不要同时由多个节点发布同一 odom TF。多机还检查错误父 frame，发现冲突会使世界退出；单机也检查重复 odom TF 发布源。

检查 TF：

```bash
ros2 run tf2_ros tf2_echo odom base_link --ros-args -p use_sim_time:=true
ros2 run tf2_ros tf2_echo base_link lidar_link --ros-args -p use_sim_time:=true
```

检查完按 Ctrl+C 关闭；这些观察节点若跨越协调 reset 存活，需要登记为 observer 或成员。

## QoS 如何匹配

默认原生点云/IMU 发布 best effort。若消费端要求 reliable，它无法与 best effort 发布者匹配。优先使用：

```python
from rclpy.qos import qos_profile_sensor_data
node.create_subscription(PointCloud2, 'points_raw', on_cloud, qos_profile_sensor_data)
```

或 C++ 的 `rclcpp::SensorDataQoS()`。必须使用 reliable 的消费者，可在单机高级入口设置节点参数 `sensor_reliable:=true`，或在 world launch 设置同名参数。reliable 发布者也能匹配 best effort 订阅者；这不意味着每个接收器一定不丢帧。

```bash
ros2 topic info /points_raw --verbose
ros2 topic echo /points_raw --field header --once --qos-reliability best_effort
```

发布端 `qos_conflicts`、`dropped_frames`、`internal_missed_scans` 与 DDS 接收端实际丢帧不是同一个指标。发布端无法完整知道 DDS 交付丢失，需消费端按扫描时间戳检查缺口。

## 多机器人命名

world 入口将机器人 ID 同时作为 ROS namespace 与 frame 前缀。以 r0 为例：输入 `/r0/cmd_vel`，输出 `/r0/points_raw`、`/r0/odom`、`/r0/joint_states`；TF 为 `r0/odom → r0/<URDF root> → r0/base_link → r0/<sensor link>`。`/clock`、`/tf`、`/tf_static`、`/sim/status`、pause/step 是世界共享接口。每台机器人的真值位于 `/sim/ground_truth/<id>/…`。

单机只有 namespace 不会自动改变 frame，必须同时配置 frame_prefix；robot.id 只决定诊断/真值标识。多机自动同时处理两者，不支持 set_pose 或完整 reset。服务可用性见 [消息与服务](消息与服务.md)。

## 诊断与运行清单

```bash
ros2 topic echo /sim/status --once
```

| 字段 | 解释 |
| --- | --- |
| ready / state / run_id | 依赖是否就绪、ready/running/paused/fault 等状态、实验标识 |
| subscription_matches | 单机发布端按话题记录的 DDS 订阅匹配数；用于会话启动/重置就绪检查，不是消息送达计数 |
| step / time_ns | 当前物理步数和仿真时间 |
| rejected_commands / last_error | 被拒绝控制数及最近错误 |
| command_timeout | 当前是否触发控制看门狗；无控制的静止实验也可能显示此项 |
| contact_count | 接触事件数量；结合具体实体和位姿排查 |
| published_clouds / published_imus | 桥接实际发布计数；多机按状态项查看相应计数 |
| qos_conflicts | 不兼容 QoS 事件计数 |
| sensor_queue_policy / sensor_queue_depth | 实际传感器可靠性及 depth 5 |
| compute_p99_ms / bridge_p99_ms | 有界窗口中的射线计算和桥接延迟 p99 |
| internal_missed_scans / dropped_frames | 内部漏扫描、显式丢弃计数；不代表接收端 DDS 丢失已知 |
| retained_inputs / evicted_inputs | 保留与截断的命令日志数量 |
| trajectory_samples / scene_bvh_builds | 轨迹历史量和场景索引重建计数 |

不同入口的诊断项有差异，以实际 values 为准。存在警告时看具体字段，不要仅凭等级推断程序崩溃。

启动时提供 `manifest`（launch）或 `manifest_path`（节点）。记录目录先创建，文件名每次实验尽量不同；同一路径会在本次运行状态更新时覆写快照。

运行清单用于追溯以下内容：

- run_id、状态、仿真时间、退出原因。
- 核心/桥接源码指纹、配置和模型资源 SHA-256、URDF 导入报告。
- 解析后参数、命令生效步号、接触事件、固定障碍策略及已忽略的旧 motion 实体。
- ROS/RMW 环境及接口配置；多机包含每台机器人的状态与 TF 发布责任。

命令/接触内存记录有 `journal_limit` 上限，总量和截断量另记。manifest 不包含完整传感器序列，不能替代 bag，也不能将其中有限的最近输入当成完整回放记录。

`pending_command=true` 表示当前 run_id 下已接收有效速度，等待下一物理步；执行后恢复 false。无效命令不会替换已有待执行命令。它是状态位，不能作为多个发布源的逐条确认。单步响应与测量话题分别传输，消费端仍需等到目标测量时间戳。

## 查看器内部通道

| 话题 | 类型 | 发布条件 / QoS |
| --- | --- | --- |
| `/sim/scene_description` | std_msgs/msg/String，JSON | 单机配置 scene_description；reliable、transient local、depth 1 |
| `/sim/scene_state` | std_msgs/msg/String，JSON | 同上；best effort、depth 1，真实 link 位姿 |
| `/sim/stats` | std_msgs/msg/String，JSON | 单机/多机速率统计；best effort、depth 1 |
| `adapter_status` | std_msgs/msg/String，JSON | Ouster 适配器；reliable、volatile、depth 10 |
| `probe_status` | std_msgs/msg/String，JSON | PCL 消费探针；reliable、transient local、depth 10 |

这些通道服务于展示与诊断，scene_state 含世界真值，不接入估计算法。场景 JSON 与生产它的导入器版本一致，不承诺独立于版本的资源格式兼容。

来源：[单机桥接](../../litesim3d_ros2/src/sim_node.cpp)、[多机桥接](../../litesim3d_ros2/src/sim_world_node.cpp)、[Livox 发布层](../../plugins/litesim3d_mid360/litesim3d_mid360/src/livox_outputs.cpp)。
