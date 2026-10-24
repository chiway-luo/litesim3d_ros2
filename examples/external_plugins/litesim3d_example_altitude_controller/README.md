# 独立第三方定高控制器

这是标准 ros2_control ControllerInterface 插件，仅依赖标准 ROS 控制包。它读 body/position.z，以 P 控制律生成限幅垂直速度，并申领全部四个 body 速度接口，其他轴置零。目标通过可配置的 target_topic 传入，超时、停用和重新激活都会清空运动指令。

默认 COLCON_IGNORE，复制到独立工作空间再构建，不需要重编译 LiteSim。完整参数文件、构建、launch 和原子控制权切换步骤见 [教程](../../../docs/tutorials/全部模型与第三方算法ros2_control.md#4-独立构建第三方算法插件)。

仅用于现有水平姿态速度运动学无人机；不提供电机推力或姿态飞控。position.z 是理想世界高度，不是带噪声的高度估计。
