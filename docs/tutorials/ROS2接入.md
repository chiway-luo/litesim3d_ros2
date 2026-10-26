# 编写 ROS 2 消费节点

目标：理解并运行一个完整 Python 节点，订阅仿真数据，在显式允许时发布速度，并把缺失输入判定为失败。代码是 [ros_client.py](../examples/ros_client.py)，不需要额外 Python 包或下载算法。

## 前置条件

已完成[启动并控制小车](启动与控制.md)。停止其他仿真，在终端 A 重新运行该教程第 2 步的四轮车命令；终端 B 加载相同 Humble 和安装环境。所有命令仍从源码仓库根目录执行。

## 1. 阅读节点的四个部分

| 代码位置 | 作用 | 接入自己算法时保留的约定 |
| --- | --- | --- |
| 创建 `Node` | 设置 `use_sim_time=True` | 传感器处理使用消息时间和仿真时钟 |
| `on_clock/on_cloud/on_imu/on_odom` | 保存消息计数、frame、布局和位姿 | 点云/IMU 使用 `qos_profile_sensor_data`，不把接收时刻当采样时刻 |
| `publish` 和主循环 | 显式 `--drive` 才发布；等待时钟、数据和订阅者 | 默认 Twist；Stamped 模式需要服务端类型一致 |
| `finally` 和 `checks` | 退出发零速度，检查通信/必需 IMU/运动 | 返回码表达结果；失败时保留 JSON，而非仅打印“已连接” |

完整字段、时间单位和 TF 发布责任见 [ROS 2 接口](../api/ROS2接口.md)与[传感器数据](../api/传感器数据.md)。

## 2. 只读订阅并保存结果

<!-- tutorial-command: client-observe -->
```bash
python3 docs/examples/ros_client.py --duration 10 --require-imu
```

应返回 0，输出 `success: true`，且 `counts.commands` 为 0。需要保存报告时，在命令后增加 `--output /tmp/litesim-client.json`。这是通信检查，没有运行 SLAM、定位或规划器。

## 3. 显式启用控制

<!-- tutorial-command: client-drive -->
```bash
python3 docs/examples/ros_client.py --drive --linear 0.1 --duration 8 --require-imu --require-motion
```

应返回 0，`counts.moving_commands > 0`，`checks.motion: true`。默认 `--require-motion` 的判据是里程计首末位置相距至少 0.01 m；需要真实位置精度评估时，应另外记录独立真值并明确坐标对齐方法。

## 4. 验证失败确实能被发现

下列命令故意订阅不存在的 IMU：

<!-- tutorial-command: client-missing-imu -->
```bash
python3 docs/examples/ros_client.py --duration 3 --imu-topic imu/not_present --require-imu
```

预期退出码为 **1**，`connected` 仍可为 true，但 `success` 和 `checks.imu` 必须为 false。它不会发送速度。若仍返回成功，应先修正消费节点的必需输入检查，再接实际算法。

## 5. 改成自己的节点

复制该脚本到自己的 ROS 包，先保持只读。在消息回调中接入处理逻辑，将处理结果发布到自己定义的话题；等时钟、frame、QoS 和时间单位都验证后，再显式接上控制输出。

新增 IMU 教程的消费方式为 `--imu-topic imu/tutorial --require-imu --expect-imu-frame tutorial_imu_link`。其他话题可用 ROS remap，例如在命令末尾加 `--ros-args -r points_raw:=/my_points`。程序通过 `rclpy` 接收这些 ROS 参数。

| 常见现象 | 原因与处理 |
| --- | --- |
| 雷达发布正常，订阅无数据 | 检查消息类型、话题和 QoS；best effort 发布者不能满足 reliable 订阅者 |
| Stamped 命令被拒绝 | 服务端需选择 stamped，frame 匹配，时间戳来自有效仿真时钟 |
| 暂停后控制/监测停止 | ROS 时间 timer 会随仿真暂停；运行时限和退出监测使用墙钟 |
| 里程计/TF 跳变 | 检查 odom TF 是否由多个节点发布；外部估计器接管时明确关闭仿真发布者 |

最后停止终端 A 的 launch。接下来可查看[算法接入与日常操作](../usage/接入与使用手册.md#接入自己的算法或控制器)、[多机器人](../usage/多机器人.md)或[插件接口](../api/插件接口.md)。
