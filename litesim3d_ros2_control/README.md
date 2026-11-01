# LiteSim3D ros2_control 硬件接口

`litesim3d_ros2_control/LiteSimSystem` 是 Humble 的 `hardware_interface::SystemInterface` 插件。
使用者的 bringup 启动 controller_manager、控制器、joint_state_broadcaster 和 RSP；公共仿真 launch 不启动这些节点。

带相机或其他慢启动依赖的例子可使用 `spawn_when_ready.py --timeout 60 -- <原 spawner 参数>`。
它等待 `/sim/status` 就绪和 `/clock` 实际推进，再执行原生 controller_manager spawner，保留其退出码。
使用 `--executable ros2_control_node` 时以同样顺序启动管理器，`--` 后的 ROS 参数和 remap 原样传递；
这样硬件接口不会在相机尚未就绪时提前激活、随后把初始化期间的反馈空窗误判为失联。
管理器入口由本包提供，继续使用 Humble 的 controller_manager 库、控制器和仿真时钟。
它处理关闭时 ROS 上下文在更新后、休眠前失效的竞态，避免 Ctrl+C 导致 `context cannot be slept with because it's invalid` 和退出码 -6。
此处理仅覆盖关闭时的休眠异常；运行中的异常和进程失败仍原样报告。
等待期间仿真故障或超过墙钟预算会明确失败；不会推进仿真时间或放宽指令看门狗。
五种基础运动模型的例子通过 `controller_startup_timeout` 配置该预算，默认 60 秒。
Humble 的控制器加载会等待管理器更新线程；相机初始化时停滞的时钟不能靠提前启动 spawner 解决。

在 runtime.yaml 中设置 `simulation.control_mode: ros2_control`，在控制模型的 `<ros2_control>` 中配置本插件。
本模式关闭仿真器的直接 `cmd_vel` 订阅及 `joint_states` 发布，由控制器接收算法指令，broadcaster 发布实际状态。
仿真编码器里程计改发 `sim/odometry`。odom TF 可由控制器负责，此时设置仿真 `publish_odom_tf: false`、`external_odom_tf: true`。

每个关节提供 `position`、`velocity` 状态接口；指令接口可选 `position`、`velocity` 中的一项或两项。
同一关节同时只能启用一种指令接口；跨模式切换应在同一次 controller_manager switch 中停用旧控制器并启用新控制器。
位置按 URDF velocity 限制追踪目标，速度经过 URDF velocity 限幅并受关节上下限限制。
本接口是运动学仿真，不提供 effort、惯性、接触力或硬实时保证。mimic 关节可仅声明状态接口，由仿真跟随源关节。

支持以下底盘的关节正运动学，以及底盘上的独立 revolute、continuous、prismatic 关节：

| 仿真底盘 | 案例控制器 | 对外指令 |
| --- | --- | --- |
| DifferentialDrive | 标准 diff_drive_controller | vx、wz |
| AckermannDrive | 标准 ackermann_steering_controller | vx、wz；后轮速度与前轮转向位置接口 |
| Omni3Drive / Omni4Drive | 本包 OmniDriveController | vx、vy、wz；三个或四个轮速接口 |
| multicopter | 本包 BodyVelocityController | vx、vy、vz、wz；机体 GPIO 资源 |

阿克曼前轮旋转关节为 state-only；它们随实际车体行程滚动。转向关节按 URDF velocity 追踪目标，受 URDF 与插件 max_steering_rad 的共同约束。实际转角用于运动计算，不能用目标转角代替受阻后的反馈。标准阿克曼控制器不替使用者限制曲率；算法需保证所需内轮转角在机械范围内，越界位置指令整包拒绝并由仿真 command_timeout 停车。

其他底盘插件若未实现 `JointDrivenChassis`，启用此模式会明确报错；原有 `twist` 模式不受影响。

硬件参数：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| command_topic | `sim/joint_commands` | 向仿真发送指令的内部话题 |
| state_topic | `sim/joint_feedback` | 仿真实际关节状态话题 |
| namespace | `/` | 内部通信节点命名空间；相对话题在此解析 |
| feedback_timeout | `2.0` | 反馈中断判定，墙钟秒 |
| activation_timeout | `10.0` | 激活时等待完整反馈和指令订阅者的上限，墙钟秒 |

仿真话题可使用 ROS remap；硬件端用以上参数接到 remap 后的话题，不依赖仿真或 RSP 的节点名。
每个机器人仅配置一个 SystemInterface/控制器管理器向其指令话题发送数据。
反馈接收和墙钟看门狗独立于管理器的仿真时钟循环。正常 pause 仍有反馈，不触发失联；时钟和反馈一起中断时先锁存故障，在管理器下一次 read/write 时返回 ERROR。Humble 将硬件转为 unconfigured；控制器生命周期状态以管理器服务返回为准，恢复时须重新配置硬件和控制器。
反馈使用 run_id 隔离仿真会话；运行中会话变化、时间倒退、错误反馈或连接超时会触发硬件错误，需重新配置/激活控制器。
`simulation.command_timeout` 是仿真时间下的独立指令看门狗。控制器停用发送 hold，失联后仿真按该超时停止；暂停期间时间不推进。

完整配置见 [双 MID360 教程](../docs/tutorials/双MID360从零搭建.md)。

传感器使用各采样快照的关节变换。IMU 使用安装 link 的完整运动链，包含旋转、滑动、mimic 关节的角速度、加速度、杆臂与科里奥利项；关节加速度由碰撞和限位处理后的速度差分得到。当前滚动扫描仅插值车体轨迹，不插值扫描周期内关节运动。双 MID360 的固定安装不受该限制。

关节扫掠只检查随该关节运动的 link（包括 mimic 与固定后代），底盘平移和转动仍检查整机几何。车体碰墙不会冻结无关的自由关节，也不会因该关节仍有运动指令而阻止倒车脱离。

## 普通全向轮控制器

`litesim3d_ros2_control/OmniDriveController` 是独立的 `controller_interface::ControllerInterface` 插件，由使用者的 controller_manager 加载。它订阅 `~/cmd_vel`（Twist，可 remap），只使用 linear.x、linear.y、angular.z；提供轮关节 velocity 指令，不发布 joint_states、odom 或 TF。案例选择仿真独立编码器里程计作为 /odom 与 odom TF 来源，切换关节控制器时仍可使用。

| 参数 | 含义 |
| --- | --- |
| wheel_joints | 三个或四个独立轮关节，顺序与以下数组一致 |
| wheel_x / wheel_y | 各轮心相对 base_link 的位置，m |
| wheel_radius | 各轮有效半径，m，正数 |
| wheel_angle | 各轮正滚动方向相对 +x 的角度，rad |
| max_wheel_velocity | 每轮速度上限，rad/s，正数且不超过 URDF velocity |
| max_linear_velocity | 平面速度模长上限，m/s，默认 1.0 |
| max_angular_velocity | 角速度绝对值上限，rad/s，默认 1.0 |
| command_timeout | Twist 超时停车，仿真秒，默认 0.5 |

上述几何数组是控制器的运行时输入，案例的 `UrdfControllerParameters` 在启动时从完整 URDF 自动生成，用户 YAML 只保留 `wheel_joints` 和控制设置。机械速度上限默认来自 URDF，可配置更小的运行上限。适配器复用核心的模型解析与几何校验，支持差速、阿克曼和普通全向控制器；不识别的第三方控制器保持原配置。生成的几何数组必须等长、有限，布局必须能独立控制 vx、vy、wz，否则配置失败。逆运动学为 `wheel_rate = (cos(a)*vx + sin(a)*vy + (x*sin(a)-y*cos(a))*wz)/radius`。任一轮超限时等比缩放所有轮速，保持请求的平面运动方向。非有限输入立即停车；停用、重新激活后清空旧指令。控制器不做轨迹跟踪或加速度规划，第三方算法应自行生成平滑速度。

配置、launch 与接管步骤见 [阿克曼和全向轮教程](../docs/tutorials/阿克曼与全向轮ros2_control.md)。这是普通全向轮的模型，麦克纳姆轮需使用对应约束与控制器，不能只改控制器名称。

## 无人机机体资源

现有 multicopter 运动学模型通过同一个 LiteSimSystem 接入，不需要虚构轮子或电机关节。在 `<ros2_control>` 内声明一个 `<gpio name="body">`，名称可自定义，并与控制器 body_name 一致。每个系统最多一个机体资源，可与关节资源共存；关节模式仍支持独立云台等可动关节。

| 接口 | 指令 | 状态 | 单位 / 坐标系 |
| --- | --- | --- | --- |
| linear.x.velocity / linear.y.velocity / linear.z.velocity | 可选 | 可选 | m/s，机体系前、左、上 |
| angular.z.velocity | 可选 | 可选 | rad/s，机体系偏航 |
| position.x / position.y / position.z | 无 | 可选 | m，世界坐标 |
| orientation.x / orientation.y / orientation.z / orientation.w | 无 | 可选 | 世界中机体四元数 |

每个 GPIO 接口可独立由控制器申领，未申领的速度轴为零。只有 multicopter 可申领机体指令；轮式模型可以只读机体状态，控制车轮仍走关节接口。错误名称、重复接口、第二个机体资源、unsupported command 会在初始化或能力握手时报错。速度限制、加减速、三维碰撞、仿真超时和硬件反馈失联处理都继续生效。此模型保持水平，零速度悬停；不模拟推力、姿态飞控、桨叶转动和自由落体。

`BodyVelocityController` 申领四个速度接口，订阅 `~/cmd_vel`（Twist，可 remap）。参数 body_name 默认 body，command_timeout 默认 0.5 仿真秒，max_linear_velocity 默认 2.0 m/s（XYZ 向量模长），max_angular_velocity 默认 1.2 rad/s。非有限输入或非零 angular.x/y 会替换为停车目标；停用和再次激活清空旧指令。无需可动关节的无人机不启动 joint_state_broadcaster；RSP 仍独立发布固定安装 TF。

## 传感器、里程计与第三方算法

全部现有传感器插件继续通过各自 ROS 数据接口供算法消费：Lidar/Mid360 的 PointCloud2 或 Livox，Imu 的 Imu，RgbCamera/DepthCamera/RgbdCamera 的 Image 与 CameraInfo。它们从控制后的实际采样快照取位姿，不能将图像、点云硬塞进标量 command_interface。里程计插件保持独立，可选用其输出或控制器自己的里程计，并明确 odom TF 的唯一发布者。

第三方控制器继承标准 ControllerInterface，声明所需 command/state interfaces，在 update 中读反馈并写指令；不需要修改仿真核心。算法输入超时由控制器实现：仿真 command_timeout 只监测硬件报文中断，不能替代上游算法话题超时，ForwardCommandController 本身会保留最后指令。完整的独立构建、参数文件、加载和控制权切换案例见 [全部模型与第三方算法教程](../docs/tutorials/全部模型与第三方算法ros2_control.md)。
