# LiteSim 3D 文档

使用环境：Ubuntu 22.04 + ROS 2 Humble。第一次运行按[安装与构建](usage/安装与构建.md) → [启动并控制小车](tutorials/启动与控制.md)操作。

没有 ROS 环境时，直接按[容器运行](usage/容器运行.md)在浏览器中启动。

| 要做什么 | 文档 |
| --- | --- |
| 用自己的机器人搭建仿真 | [双 MID-360 从零搭建](tutorials/双MID360从零搭建.md) |
| 配置阿克曼、三轮或四轮全向底盘 | [阿克曼与全向轮 ros2_control](tutorials/阿克曼与全向轮ros2_control.md) |
| 接入第三方控制算法 | [统一 ros2_control 接入](tutorials/全部模型与第三方算法ros2_control.md) |
| 添加 IMU、编写 ROS 2 消费节点 | [教程目录](tutorials/README.md) |
| 配置模型、场景、传感器、多机和重置 | [使用手册](usage/README.md) |
| 查询话题、字段、默认值、单位和接口 | [接口参考](api/README.md)、[插件参数](api/plugins/README.md) |
| 理解配置流程、TF 和仿真时间 | [运行与坐标](concepts/运行与坐标.md) |
| 查找完整车型和扩展包 | [车型与扩展示例](../examples/README.md) |
| 录包、回放或排查问题 | [记录与排障](usage/记录与排障.md) |

教程配套文件位于 [docs/examples](examples)。已有 YAML 工程见[兼容配置](usage/兼容配置.md)；源码与安装包的版本应与 [VERSION](../VERSION) 一致。
