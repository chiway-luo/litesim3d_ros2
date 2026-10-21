# Livox 官方消息的纯接口构建

此接口包归属 [Mid-360 插件](../../README.md)，位于其 `interfaces/` 目录，独立生成官方 ROS 类型。

此目录只编译上游原样的 `CustomMsg.msg`、`CustomPoint.msg`，保留官方包名，生成真正的
`livox_ros_driver2/msg/CustomMsg` 类型，供仿真与使用官方驱动消息的算法互通。
不含设备驱动、SDK、联网发现或硬件控制功能；0.4.0 是本仓库分发版本，不是上游驱动版本。

固定上游版本、文件 SHA-256 见 [UPSTREAM.json](UPSTREAM.json)，许可见 [LICENSE.txt](LICENSE.txt)。
消息定义不得在此修改；升级时应重新核对官方消息、逐点时间语义。

已有官方 `livox_ros_driver2` 的工作空间应只选一个同名包：在此目录放置本地
`COLCON_IGNORE`，或构建时 `--packages-skip livox_ros_driver2` 并先 source 官方驱动安装。
不要在同一安装前缀覆盖两种分发。仿真桥接仅依赖消息，可以直接链接官方包。
