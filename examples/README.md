# 车型与扩展示例

先完成[安装与构建](../docs/usage/安装与构建.md)。车型默认使用 ros2_control；同一 ROS domain 每次运行一个示例。

| 示例 | 用途 |
| --- | --- |
| [四轮小车](wheel_car_4w/README.md) | 办公场景、差速运动、雷达与 IMU |
| [双 MID-360 巡检小车](dual_mid360/README.md) | 独立 description/bringup、前后倾斜双雷达 |
| [阿克曼](ackermann/README.md) | 转向关节与后轮速度控制 |
| [三轮全向](omni3/README.md) | 普通全向轮的平移与旋转 |
| [四轮全向](omni4/README.md) | 四轮普通全向底盘，角点切向 X 布局 |
| [无人机](drone_3d/README.md) | 三维速度控制，不含飞行动力学 |
| [外部里程计插件](external_plugins/README.md) | 独立安装和替换编码器里程计 |
| [第三方定高控制器](external_plugins/litesim3d_example_altitude_controller/README.md) | 独立 ros2_control 算法插件 |

控制器配置见[统一接入教程](../docs/tutorials/全部模型与第三方算法ros2_control.md)，自己的机器人按[模型接入](../docs/usage/模型接入.md)组织资源。参数见[插件参考](../docs/api/plugins/README.md)。
