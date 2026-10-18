# wheel_car_4w：四轮小车完整仿真示例

本示例在一个 launch 中启动 **四轮小车模型、三维雷达、IMU、里程计、场景、ROS 2 接口和独立三维查看窗口**。首次使用先按仓库 [README](../../README.md) 安装依赖并构建。

## 1. 目录与职责

```text
examples/wheel_car_4w/
├── README.md
├── litesim_models/                    # SDF 世界、模型、网格和贴图资源包
├── wheel_car_4w_description/          # 独立模型包
│   ├── package.xml
│   ├── CMakeLists.txt
│   └── urdf/wheel_car_4w.urdf         # 有注释的完整 URDF，无需先展开 Xacro
└── wheel_car_4w_sim/                  # 独立仿真启动包
    ├── package.xml
    ├── CMakeLists.txt
    ├── config/
    │   ├── runtime.yaml              # SDF 模式底盘、传感器及地形绑定
    │   ├── simulation.yaml           # 保留旧 URDF/YAML 回归入口
    │   ├── ros.yaml                  # 控制类型、QoS、TF、暂停、RSP 等 ROS 参数
    │   └── lidars/                   # vlp16、mid360、mid40，带厂家来源与建模说明
    ├── launch/wheel_car_4w.launch.py  # 唯一启动入口
    ├── rviz/wheel_car_4w.rviz         # 模型、点云、环境、TF、轨迹
    └── test/test_wheel_car_4w.py      # 安装态核心/ROS/启动故障与可选 GUI 测试
```

仿真包复用 `litesim3d_core`、`litesim3d_ros2` 和 `litesim3d_interfaces`。litesim_models 提供默认 SDF 资源，description 保留旧 URDF；三个示例包都经 ament 安装，运行时使用 package 资源索引，启动终端不必处于源码目录。

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

默认同时启动 RSP、仿真和独立三维查看器，**不需要再手工启动模型发布器或单独的传感器节点**。默认 16 线×512 列、10 Hz rolling 激光、1 cm测距标准差，200 Hz raw IMU，50 Hz odom，5 ms 积分步长。机械多线、球面带非重复扫描和圆形花瓣扫描的切换及真实规格参考，见 [雷达配置说明](雷达配置说明.md)。

桌面不可用或只测算法时关闭查看器：

```bash
ros2 launch wheel_car_4w_sim wheel_car_4w.launch.py gui:=false
```

默认场景有地面、两面墙、箱体、立柱和悬空障碍。小车车头的红色标记朝 +x；轮毂色条便于观察四轮转动。按完整 launch 所在终端的 Ctrl+C 退出；RSP/仿真子进程意外退出会使 launch 清理其他进程并以非零状态结束。

独立窗口只允许左键旋转、中键或右键平移、滚轮缩放；无模型选择和仿真控制入口。底部自动显示实际 RTF、目标倍率、FPS 和碰撞预算状态。关闭窗口后仿真继续，可用 `ros2 run litesim3d_gui viewer` 重连。需要点云/TF 图层时显式加 `rviz:=true`。

车载相机已支持 `rgb`、`depth`、`rgbd`，在 runtime 的 `cameras` 中设置分辨率、帧率、视场角和近远裁剪面。示例相机默认关闭，类型默认 `rgb`；用 `cameras:=true` 开启 640×480 / 15 Hz 普通彩色相机，通过 ROS 2 话题输出，不弹出摄像头窗口：

```bash
ros2 launch wheel_car_4w_sim wheel_car_4w.launch.py cameras:=true
```

普通相机只输出 `/camera/front/color/image_raw` 及对应 `camera_info`，不创建深度渲染目标或深度话题。需要彩色加深度时显式设置 `type: rgbd`，额外输出 `/camera/front/depth/image_raw` 及对应 `camera_info`；`type: depth` 则仅输出深度。深度是 `32FC1` 米制光轴距离，RGB-D 两通道同帧对齐。图像预览默认关闭；仅手动加 `camera_view:=true` 时打开。`gui:=false` 关闭三维窗口，不停止相机渲染和话题发布。参数当前启动时生效，修改后重新启动。具体字段、光学坐标与平台限制见 [相机使用说明](../../docs/usage/SDF与独立窗口.md#车载-rgb-与深度相机)。

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

SDF 四轮关节的轴都是底盘 +y，碰撞为居中共轴圆柱；后轮 link 必须保持叶节点。修改轮径、轮距时同时修改 `litesim_models/models/wheel_car_4w/model.sdf` 和 `runtime.yaml/robot.drive`，否则加载会拒绝。

## 5. 修改参数

默认 SDF 模式使用以下文件：

- [warehouse.sdf](litesim_models/worlds/warehouse.sdf)：场景模型、初始位姿、灯光、步长和默认目标速率。
- [model.sdf](litesim_models/models/wheel_car_4w/model.sdf)：四轮车 link/joint、visual/collision 和材质。
- [runtime.yaml](wheel_car_4w_sim/config/runtime.yaml)：底盘、传感器、噪声和地形支持绑定。
- [ros.yaml](wheel_car_4w_sim/config/ros.yaml)：命令类型、QoS、TF、暂停和 RSP 参数。

修改已安装资源的源码后重新构建；或指定自己的资源根及 runtime。资源职责、贴图兼容矩阵和碰撞预算见 [SDF 与独立窗口](../../docs/usage/SDF与独立窗口.md)。

```bash
ros2 launch wheel_car_4w_sim wheel_car_4w.launch.py \
  assets_root:="$PWD/examples/wheel_car_4w/litesim_models" \
  runtime:="$PWD/examples/wheel_car_4w/wheel_car_4w_sim/config/runtime.yaml"
```

旧 `simulation.yaml` 和 URDF 保留用于回归，显式 `config:=路径` 才启用旧入口。参数在启动时读取，无热加载。

| 想改什么 | 改哪个字段 |
| --- | --- |
| 最大速度/转速 | robot.drive.max_v / max_w |
| 加速和停车快慢 | acceleration / deceleration / angular_acceleration |
| 初始位置/航向 | world SDF 中对应 include/model 的 pose，单位米/弧度 |
| 雷达频率/点数 | lidars[0].rate_hz / columns / vertical_angles_deg，rings 同步调整 |
| 雷达安装位置 | SDF lidar link/joint 的 pose |
| 测距误差 | range_bias_m / range_sigma_m / dropout |
| 最大量程边界显示 | publish_range_boundary；默认开启，距离由max_range_m决定；辅助点与实际回波分话题 |
| IMU 误差 | accel_bias / gyro_bias / accel_sigma / gyro_sigma |
| 非理想里程计 | odom.mode=noisy，再设置 scale/bias/sigma；不改变真实运动 |
| 场景障碍 | SDF include/model；尺寸和 pose 按米/弧度 |
| 动态障碍 | 目前使用旧 YAML 入口的 motion；新 SDF 入口仅支持静态环境 |
| 算法要求 reliable | ros.yaml 中 sensor_reliable=true |
| 算法发 TwistStamped | cmd_mode=stamped；时间戳用当前 /clock、frame=base_link |
| 外部估计器接管 odom TF | publish_odom_tf=false、external_odom_tf=true；输出到 URDF root |
| TF 前缀 | frame_prefix，如 car/；launch 自动同步 RSP 和 RViz，话题仍在根命名空间 |

暂停单步校验大点云时，建议在 `ros.yaml` 中设置 `sensor_reliable: true`，通过 `params:=` 加载，并把算法订阅及 RViz 点云显示的 Reliability Policy 设为 Reliable。20,000 点的数据区为 480,000 字节；best effort 即使 DDS 已匹配也可能丢失整帧，暂停后不会自动补发。只修改发布端可靠性、保留 best effort 消费端，仍不能得到可靠传输。默认配置保留 best effort，适合允许丢帧的实时显示。

不要直接把 `cmd_mode:=stamped` 写在本 launch 后假定会改变节点参数；修改 `ros.yaml` 后通过 `params:=` 加载。Stamped 模式可以用项目的接入示例控制：

```bash
python3 docs/examples/ros_client.py --stamped --drive --duration 5
```

如果同时设了 `frame_prefix: car/`，上面的命令再加 `--frame car/base_link`。此启动包面向一个小车实例，不是多机器人世界；多机共享时钟的方法见总手册。

### launch 参数

| 参数 | 默认与用途 |
| --- | --- |
| assets_root / world | litesim_models share / warehouse.sdf |
| runtime | 安装的 config/runtime.yaml |
| gui | true；仅查看三维场景，false 无窗口 |
| collision_budget_enabled | false；仅关闭预算，保留碰撞检测 |
| real_time_factor | 空使用 SDF；0 不限速 |
| config | 空；显式传入旧 simulation.yaml 切换到旧入口 |
| params | 安装的 config/ros.yaml |
| lidar_profile | config；也可选 vlp16、mid360、mid40 或自定义预设YAML绝对路径，启动时生效 |
| rviz | false；true 另开算法图层查看器 |
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
