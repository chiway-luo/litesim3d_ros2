# 独立安装的里程计插件示例

`litesim3d_example_encoder_odometry` 只依赖公共 `litesim3d_plugin_api` 与 pluginlib，不依赖核心源码或任何内置里程计插件。它从两轮编码器恢复运动，增加一个 `distance_scale` 标定参数，方便检查替换是否实际生效。目录中的 `COLCON_IGNORE` 使其不进入主构建。

先构建主工程，然后在已经 source ROS 2 的终端执行：

```bash
bash scripts/test_external_plugin.sh /tmp/litesim3d-external-test /tmp/litesim3d-plugins/install
```

第二个参数换成已有主工程的安装前缀。脚本将示例源码复制到独立工作空间，仅编译示例包，source 新 overlay，再让原来的 `sim_headless` 根据组合 URDF 加载新类。它检查车体真实位移 1 m、外部插件里程计位移 1.25 m，以及核心可执行文件、核心共享库及 API 共享库 SHA-256 在独立构建前后不变。证据写入第一个参数指定的目录。

该包的完整参数和中文说明位于 `config/parameters.yaml`。实际接入时在模型中替换 odometry 声明：

```xml
<odometry name="wheel_odom" class="litesim3d_example_encoder_odometry/ExampleEncoderOdometry">
  <parameters file="package://litesim3d_example_encoder_odometry/config/parameters.yaml"/>
  <overrides>distance_scale: 1.25</overrides>
</odometry>
```

只改变配置和安装空间；无需给核心添加类名白名单、源文件或链接目标。主工程接口 ABI 变化时，外部插件需要针对同一版 API 重新编译；这不属于运行中热替换。

## 外部 ros2_control 算法

[定高控制器](litesim3d_example_altitude_controller/README.md) 是另一个独立包：只依赖标准 ControllerInterface，读取机体高度反馈、输出 XYZ / 偏航速度接口。它通过 controller_manager 与默认速度控制器交换资源，不替换里程计或传感器插件。
