# LiteSim 3D 文档

从四轮车组合模型和 SDF 场景开始。正式运行环境为 Ubuntu 22.04 + ROS 2 Humble，当前文档面向开发树；先核对[版本与验证状态](版本与验证状态.md)。

## 选择任务

| 现在要做什么 | 从这里开始 |
| --- | --- |
| 第一次使用 | [安装与构建](usage/安装与构建.md) → [启动并控制小车](tutorials/启动与控制.md) |
| 从自己的 description 搭建完整仿真 | [双 MID-360：独立 bringup、world、URDF 传感器与 launch 配置](tutorials/双MID360从零搭建.md) |
| 阿克曼与普通全向轮外部控制 | [ros2_control、控制器参数与案例 launch](tutorials/阿克曼与全向轮ros2_control.md) |
| 给所有现有模型接入第三方控制算法 | [统一 ros2_control 接入、四轮车与无人机、独立控制器案例](tutorials/全部模型与第三方算法ros2_control.md) |
| 学习模型扩展和 ROS 接入 | [完整教程目录](tutorials/README.md) |
| 查询日常操作、算法接入和排障 | [使用手册](usage/README.md) |
| 查字段、默认值、错误码和 SDK | [接口参考](api/README.md)、[逐插件参数表](api/plugins/README.md) |
| 理解配置、TF 和重置 | [运行与坐标](concepts/运行与坐标.md) |
| 编写扩展或维护文档 | [插件开发](development/插件开发.md)、[文档维护](development/文档维护.md) |
| 确认平台、版本和证据 | [版本与验证状态](版本与验证状态.md) |

## 示例与历史资料

车型完整工程见[车型与扩展示例](../examples/README.md)，教程配套文件位于 [docs/examples](examples)。已有 YAML 实验使用[兼容配置](usage/兼容配置.md)。

[测试文档](testing/README.md)保存回归入口和历史证据；[设计资料](design/开发方案.md)保存设计背景。历史阶段编号不作为使用者的前置知识，历史结果也不自动覆盖当前工作树。
