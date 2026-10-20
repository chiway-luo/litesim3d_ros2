# 添加一个 IMU

目标：在已有四轮车上增加第二个 IMU，保持原 IMU 不变。新增实例为 `tutorial_imu`，安装在 `tutorial_imu_link`，输出 `/imu/tutorial`。

## 前置条件与完整文件

先完成[启动并控制小车](启动与控制.md)，停止上一组 launch，并在每个终端加载同一环境。

完整文件已随仓库提供，可直接运行和修改：

| 文件 | 作用 |
| --- | --- |
| [simulation.urdf.xacro](../examples/tutorial_robot/urdf/simulation.urdf.xacro) | include 原四轮组合模型，新增 link、fixed joint 和 IMU 声明 |
| [runtime.yaml](../examples/tutorial_robot/config/runtime.yaml) | 指向本目录的组合模型，设置仿真与 SDF 绑定 |

本例不创建新 ROS 包，使用仓库中的独立资源目录；依赖的四轮模型和插件已由安装步骤提供。修改此目录不会改动安装空间中的标准模型。自己的正式模型可按[模型接入](../usage/模型接入.md)安装为独立包。

## 1. 核对安装坐标

打开随附 Xacro。`tutorial_imu_joint` 把新 link 固定到 `base_link`，平移为 `[0.1, 0, 0.15]` 米，旋转为零。坐标轴为 x 前、y 左、z 上。该 link 没有碰撞几何，不改变车体碰撞形状。

同一文件的 sensor 声明引用 `litesim3d_imu/Imu` 及其完整参数文件，再覆盖 link、topic、100 Hz 采样频率和零噪声。默认值、单位及限制见 [IMU 参数参考](../api/plugins/litesim3d_imu.md)。

## 2. 启动自定义模型

终端 A，从源码仓库根目录执行：

<!-- tutorial-command: imu-launch -->
```bash
ros2 launch wheel_car_4w_sim wheel_car_4w.launch.py runtime:="$PWD/docs/examples/tutorial_robot/config/runtime.yaml" gui:=false rviz:=false cameras:=false lidar_profile:=vlp16 lidar_interface:=native
```

`runtime` 使用绝对路径；其中 `robot.urdf` 的相对路径按 runtime 文件目录解析。SDF 办公场景仍由四轮车启动器提供。不要同时给这个组合模型填写旧 YAML 的 `imus` 列表。

## 3. 验证新增数据和 TF

终端 B：

<!-- tutorial-command: imu-observe -->
```bash
python3 docs/examples/ros_client.py --duration 10 --imu-topic imu/tutorial --require-imu --expect-imu-frame tutorial_imu_link
```

应输出 `success: true`、`counts.imu > 0` 和 `frames.imu: "tutorial_imu_link"`。水平静止时，加速度 z 分量应接近 +9.81 m/s²，x/y 接近 0；raw IMU 不提供姿态估计。

查看实际消息和固定变换：

```bash
ros2 topic echo /imu/tutorial --once --qos-reliability best_effort
ros2 run tf2_ros tf2_echo base_link tutorial_imu_link
```

TF 应给出约 `[0.1, 0, 0.15]` 的平移与零旋转。第二条持续显示，按 Ctrl+C 结束。自动教程检查也会核对该变换及静止比力。

## 4. 修改一次并复验

在 Xacro 的 overrides 中将 `rate_hz: 100.0` 改为 `rate_hz: 50.0`，停止终端 A 的 launch 后重新启动，再运行第 3 步。参数在启动时读取。可以用 `ros2 topic hz /imu/tutorial` 观察接收频率，但它受实际 RTF 和接收端负载影响，不应把墙钟测量直接当作仿真频率验收。

复验后恢复为 100 Hz，保留一个与教程检查一致的起点。

## 常见失败

| 现象 | 检查与处理 |
| --- | --- |
| 找不到 `tutorial_imu_link` | 确认运行的是本例 runtime，Xacro 中有 link 和 fixed joint |
| 启动时报重复实例名/话题 | 每个 sensor 使用不同 name 与 topic；不要复制原实例名 |
| 新 IMU 无数据、旧 IMU 正常 | 消费端必须订阅 `imu/tutorial`，不能仍检查 `imu/data_raw` |
| frame 正确但轴向不符 | 核对 fixed joint 的 rpy；IMU 使用安装 link 的方向 |
| 修改频率未生效 | 重新启动整组 launch，检查是否误改安装目录或另一份配置 |

下一步：[编写 ROS 2 消费节点](ROS2接入.md)。
