# 入门教程

先完成[安装与构建](../usage/安装与构建.md)。以下教程使用 Ubuntu 22.04、ROS 2 Humble 和 Bash；同一 ROS domain 每次运行一个仿真。

| 教程 | 完成后能做什么 |
| --- | --- |
| [启动并控制小车](启动与控制.md) | 读取传感器，发送速度并观察里程计位移 |
| [添加一个 IMU](添加IMU.md) | 配置安装坐标和采样频率，检查消息与 TF |
| [编写 ROS 2 消费节点](ROS2接入.md) | 订阅仿真数据、发布控制指令并识别缺失输入 |
| [双 MID-360 从零搭建](双MID360从零搭建.md) | 用独立 description/bringup 包配置 world、双雷达和控制器 |
| [阿克曼与全向轮 ros2_control](阿克曼与全向轮ros2_control.md) | 配置转向、轮速及外部控制器 |
| [全部模型与第三方算法 ros2_control](全部模型与第三方算法ros2_control.md) | 接入四轮车、无人机和独立控制算法插件 |

操作查询见[使用手册](../usage/README.md)，字段、单位和限制见[接口参考](../api/README.md)。
