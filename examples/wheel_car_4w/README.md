# wheel_car_4w：四轮小车完整仿真示例

本示例在一个 launch 中启动 **四轮小车模型、三维雷达、IMU、里程计、场景、ROS 2 接口和 RViz**。首次使用先按仓库 [README](../../README.md) 安装依赖并构建。

## 1. 目录与职责

```text
examples/wheel_car_4w/
├── README.md
├── wheel_car_4w_description/          # 独立模型包
│   ├── package.xml
│   ├── CMakeLists.txt
│   └── urdf/wheel_car_4w.urdf         # 有注释的完整 URDF，无需先展开 Xacro
└── wheel_car_4w_sim/                  # 独立仿真启动包
    ├── package.xml
    ├── CMakeLists.txt
    ├── config/
    │   ├── simulation.yaml           # 物理、底盘、传感器、地形、场景，完整中文注释
    │   ├── ros.yaml                  # 控制类型、QoS、TF、暂停、RSP 等 ROS 参数
    │   └── lidars/                   # vlp16、mid360、mid40，带厂家来源与建模说明
    ├── launch/wheel_car_4w.launch.py  # 唯一启动入口
    ├── rviz/wheel_car_4w.rviz         # 模型、点云、环境、TF、轨迹
    └── test/test_wheel_car_4w.py      # 安装态核心/ROS/启动故障与可选 GUI 测试
```

仿真包复用 `litesim3d_core`、`litesim3d_ros2` 和 `litesim3d_interfaces`。description 包只提供模型；两个示例包都经 ament 安装，运行时使用 package 资源索引，启动终端不必处于源码目录。

## 2. 构建后，一条 launch 启动

终端 A 激活 Humble 环境、进入 clone 的源码仓库根目录后执行；后文相同：

```bash
evidence_dir="$(python3 scripts/paths.py evidence)"
mkdir -p "$evidence_dir"
source "$(python3 scripts/paths.py install)/setup.bash"
export ROS_DOMAIN_ID=160
export ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR="${evidence_dir}/wheel-car-4w/manual-logs"
mkdir -p "$ROS_LOG_DIR"
ros2 launch wheel_car_4w_sim wheel_car_4w.launch.py
```

默认同时启动 RSP、仿真和 RViz，**不需要再手工启动模型发布器或单独的传感器节点**。默认 16 线×512 列、10 Hz rolling 激光、1 cm测距标准差，200 Hz raw IMU，50 Hz odom，5 ms 积分步长。机械多线、球面带非重复扫描和圆形花瓣扫描的切换及真实规格参考，见 [雷达配置说明](雷达配置说明.md)。

桌面不可用或只测算法时关闭 RViz：

```bash
ros2 launch wheel_car_4w_sim wheel_car_4w.launch.py rviz:=false
```

默认场景有地面、两面墙、箱体、立柱和悬空障碍。小车车头的红色标记朝 +x；轮毂色条便于观察四轮转动。按完整 launch 所在终端的 Ctrl+C 退出；RSP/仿真子进程意外退出会使 launch 清理其他进程并以非零状态结束。

RViz 默认选中 **Move Camera**，采用 Orbit 视图。在三维画面中左键按住拖动可旋转，Shift+左键拖动可平移，滚轮可缩放。使用触控板时，旋转需要按住并拖动，双指滚动用于缩放。切换其他工具后，可点击工具栏的 Move Camera 或按 `m` 返回相机工具；Views 面板可手动调整距离、俯仰和朝向。

同一个 domain 只运行一个仿真时钟；开始本例前关闭此前启动的单机/world 仿真。

## 3. 让车动起来

终端 B 先激活相同 Humble 环境并进入源码仓库根目录（Zsh 将 `setup.bash` 改为 `setup.zsh`）：

```bash
evidence_dir="$(python3 scripts/paths.py evidence)"
mkdir -p "$evidence_dir"
source "$(python3 scripts/paths.py install)/setup.bash"
export ROS_DOMAIN_ID=160
export ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR="${evidence_dir}/wheel-car-4w/manual-logs"
```

持续发前进/左转指令：

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.3}, angular: {z: 0.2}}'
```

正 x 向前，正 z 角速度为俯视逆时针。停止发布后，默认 0.5 秒**仿真时间**超时，按减速度停车。也可显式发送零速度：

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0}, angular: {z: 0.0}}'
```

查看数据：

```bash
ros2 topic echo /sim/status --once
ros2 topic echo /joint_states --once
ros2 topic echo /points_raw --once --field header --qos-reliability best_effort
ros2 topic echo /imu/data_raw --once --qos-reliability best_effort
```

默认接口：

| 话题 | 内容 |
| --- | --- |
| `/cmd_vel` | Twist，使用 linear.x / angular.z |
| `/points_raw` | 16×512 PointCloud2，xyz/intensity/ring/time，24 字节，frame=lidar_link |
| `/points_raw/range_boundary` | 没有命中物体的方向对应最大量程辅助点；RViz淡蓝色，不能当成实际障碍 |
| `/imu/data_raw` | raw IMU，frame=imu_link，无姿态估计 |
| `/odom` | 局部里程计，pose 在 odom，twist 在 base_link |
| `/joint_states` | 四个轮关节的位置/速度，前后同侧一致 |
| `/robot_description` | 完整 URDF，transient local |
| `/clock`、`/tf`、`/tf_static` | 仿真时间和坐标关系 |
| `/scene`、`/trajectory` | 环境和局部轨迹可视化 |
| `/sim/ground_truth/wheel_car_4w/odom`、`contacts` | 独立真值/碰撞评测通道 |

rolling 点云中的 time 是相对 header 的秒偏移，尚未去畸变。无回波为 NaN。算法订阅点云/IMU 默认使用 SensorDataQoS；所有算法设置 `use_sim_time=true`。

## 4. 四轮模型和 TF 约定

| 几何/传感器 | 数值 |
| --- | --- |
| 车体 | 0.74×0.48×0.18 m，中心 z=0.25 m |
| 四个轮子 | 半径 0.12 m、宽 0.07 m |
| 左右轮中心距 | 0.58 m |
| 前后轮中心距 | 0.48 m |
| 轮心 | x=±0.24、y=±0.29、z=0.12 m |
| lidar_link 相对 base_link | `[0.12, 0, 0.49]` m，零安装旋转 |
| imu_link 相对 base_link | `[-0.08, 0, 0.36]` m，零安装旋转 |

后轮关节 `rear_left_wheel_joint`、`rear_right_wheel_joint` 代表两侧独立驱动；前轮用 URDF mimic 按 1:1 跟随同侧后轮。小车整体采用**同侧等速的差速等效运动学**；不模拟四个独立电机、轮胎侧滑力、悬架或真实 Ackermann 转向机构。四个轮子的碰撞体都参与支撑和障碍检查。

模型 root 是没有几何/惯性的 `base_footprint`，base_link 与它固定重合，均位于车体中心的地面投影。TF 链为：

```text
odom -> base_footprint -> base_link -> 四个 wheel_link
                                 ├-> lidar_mount_link -> lidar_link
                                 └-> imu_link
```

`odom -> base_footprint` 由仿真发布；其余边由 RSP 发布。不能同时再发 `odom -> base_link`，否则会让 base_link 出现两个父节点。IMU 静止、水平时 z 比力约为 +9.80665 m/s²，不是零；orientation_covariance[0]=-1。

URDF 四轮关节的轴都是底盘 +y，碰撞为居中共轴圆柱；后轮 link 必须保持叶节点。修改轮径、轮距时同时修改 URDF 和 `simulation.yaml/robot.drive`，否则加载会拒绝。

## 5. 修改参数

两个文件中已经列出本例使用的完整字段、单位、范围和修改说明：

- [simulation.yaml](wheel_car_4w_sim/config/simulation.yaml)：模型路径、初值、底盘、限速、加减速、时间、随机种子、雷达、IMU、odom、地形、场景。
- [ros.yaml](wheel_car_4w_sim/config/ros.yaml)：use_sim_time、命令类型、QoS、TF 责任、RSP 等待、暂停、manifest、RSP 发布频率。

launch 默认读取**安装目录中的副本**。修改源码配置后可以重新构建，也可以在启动时直接指定源码参数文件，省去配置修改后的构建步骤：

```bash
ros2 launch wheel_car_4w_sim wheel_car_4w.launch.py \
  config:="$PWD/examples/wheel_car_4w/wheel_car_4w_sim/config/simulation.yaml" \
  params:="$PWD/examples/wheel_car_4w/wheel_car_4w_sim/config/ros.yaml"
```

URDF 默认仍通过 package URI 读取安装副本；修改 URDF 后重新构建，或在自定义 simulation.yaml 中将 robot.urdf 显式改为所需文件的绝对路径。参数都在启动时读取，修改后重启 launch；当前不支持热加载。

| 想改什么 | 改哪个字段 |
| --- | --- |
| 最大速度/转速 | robot.drive.max_v / max_w |
| 加速和停车快慢 | acceleration / deceleration / angular_acceleration |
| 初始位置/航向 | robot.initial_pose.xyz / rpy，rpy 单位弧度 |
| 雷达频率/点数 | lidars[0].rate_hz / columns / vertical_angles_deg，rings 同步调整 |
| 雷达安装位置 | URDF lidar_mount_joint / lidar_joint |
| 测距误差 | range_bias_m / range_sigma_m / dropout |
| 最大量程边界显示 | publish_range_boundary；默认开启，距离由max_range_m决定；辅助点与实际回波分话题 |
| IMU 误差 | accel_bias / gyro_bias / accel_sigma / gyro_sigma |
| 非理想里程计 | odom.mode=noisy，再设置 scale/bias/sigma；不改变真实运动 |
| 场景障碍 | entities；尺寸和 pose 按米/弧度 |
| 动态障碍 | simulation.yaml 末尾已有可取消注释的完整 motion 示例 |
| 算法要求 reliable | ros.yaml 中 sensor_reliable=true |
| 算法发 TwistStamped | cmd_mode=stamped；时间戳用当前 /clock、frame=base_link |
| 外部估计器接管 odom TF | publish_odom_tf=false、external_odom_tf=true；输出到 URDF root |
| TF 前缀 | frame_prefix，如 car/；launch 自动同步 RSP 和 RViz，话题仍在根命名空间 |

不要直接把 `cmd_mode:=stamped` 写在本 launch 后假定会改变节点参数；修改 `ros.yaml` 后通过 `params:=` 加载。Stamped 模式可以用项目的接入示例控制：

```bash
python3 docs/examples/ros_client.py --stamped --drive --duration 5
```

如果同时设了 `frame_prefix: car/`，上面的命令再加 `--frame car/base_link`。此启动包面向一个小车实例，不是多机器人世界；多机共享时钟的方法见总手册。

### launch 参数

| 参数 | 默认与用途 |
| --- | --- |
| config | 安装的 config/simulation.yaml |
| params | 安装的 config/ros.yaml |
| lidar_profile | config；也可选 vlp16、mid360、mid40 或自定义预设YAML绝对路径，启动时生效 |
| rviz | true；false 时只运行 RSP/仿真 |
| rviz_config | 安装的 rviz/wheel_car_4w.rviz；可提供自己的显示配置 |
| paused | 空表示遵循 ros.yaml；true/false 显式覆盖 start_paused |
| manifest | 空表示遵循 ros.yaml；非空覆盖为该运行清单文件，父目录须存在 |

```bash
ros2 launch wheel_car_4w_sim wheel_car_4w.launch.py --show-args
```

launch 自动填入 config、同一份 URDF 的 robot_description，并将 RSP/RViz frame 前缀与仿真节点同步。RViz 显示配置使用临时副本，正常关闭时删除，安装模板保持不变。

## 6. 暂停、单步和保存实验

暂停启动并保存清单：

```bash
evidence_dir="$(python3 scripts/paths.py evidence)"
mkdir -p "$evidence_dir"
mkdir -p "${evidence_dir}/wheel-car-4w/manual"
ros2 launch wheel_car_4w_sim wheel_car_4w.launch.py \
  paused:=true manifest:="${evidence_dir}/wheel-car-4w/manual/run.json"
```

另一个同环境终端：

```bash
ros2 service call /sim/step litesim3d_interfaces/srv/Step \
  '{request_id: "car-step-001", steps: 20}'
ros2 service call /sim/pause std_srvs/srv/SetBool '{data: false}'
ros2 service call /sim/pause std_srvs/srv/SetBool '{data: true}'
```

20 步为 0.1 秒仿真时间。相同 ID 和相同步数重试只返回原结果，新一次推进换 ID。运行态不能单步。运行后想回初始状态可关闭并重启整个 launch；需要跨算法协调 reset 时改用总手册的 sim_session，以本例 simulation.yaml 为 config，不能和本 launch 同时运行。

独立核心预检：

```bash
evidence_dir="$(python3 scripts/paths.py evidence)"
mkdir -p "$evidence_dir"
lws_prefix="$(ros2 pkg prefix litesim3d_core)"
lws_config="$(ros2 pkg prefix wheel_car_4w_sim)/share/wheel_car_4w_sim/config/simulation.yaml"
mkdir -p "${evidence_dir}/wheel-car-4w/manual"
"$lws_prefix/bin/sim_headless" "$lws_config" 400 \
  "${evidence_dir}/wheel-car-4w/manual/headless.json" 0.2 0.1 0
```

这运行 2 秒仿真，持续输入 0.2 m/s 和 0.1 rad/s；预期产生 20 帧雷达。不启动 ROS、RSP 或窗口。

## 7. 后续接入

- [雷达配置说明](雷达配置说明.md)：雷达预设、范围与精度语义。
- [接入与使用手册](../../docs/usage/接入与使用手册.md)：模型、算法、参数、录包与排障。
- [四轮车与雷达测试](../../docs/testing/四轮车与雷达测试.md)：测试入口、结果与未验证范围。
