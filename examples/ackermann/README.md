# 阿克曼示例

构建并 source 安装环境后执行：

```bash
ros2 launch ackermann_sim ackermann.launch.py gui:=false
```

省略 gui 参数可打开三维窗口；`cameras:=true` 开启前向 RGB，需要图形后端。示例带固定地面、前墙和侧墙，默认碰撞后车体受阻，编码器与轮式里程计继续累计；在 runtime.yaml 把 robot.continue_on_collision 改为 false 可让驱动累计停止。

`config/geometry.yaml` 同时供机械 Xacro、底盘和里程计引用；`urdf/sensors.xacro` 定义雷达、IMU 和光学坐标系，`urdf/simulation.urdf.xacro` 组合模型并选择独立插件。所有传感器跟随真实车体。参数与替换方法见 [模型组合与插件迁移](../../docs/usage/模型组合与插件迁移.md)。
