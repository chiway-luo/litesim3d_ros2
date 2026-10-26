# 阿克曼与全向轮接入 ros2_control

阿克曼、三轮和四轮普通全向案例都使用外部 ros2_control。第三方算法可以发布车体速度，也可以通过控制器管理器切换到关节接口。公共仿真 launch 只启动仿真及可选显示，RSP、controller_manager、控制器和状态广播器由各自案例配置。

| 案例包 | 速度控制器 | 主动关节 | 被动关节 |
| --- | --- | --- | --- |
| ackermann_sim | ackermann_steering_controller/AckermannSteeringController | 后轮 velocity + 前轮转向 position | 前轮旋转，只读反馈 |
| omni3_sim | litesim3d_ros2_control/OmniDriveController | 三个轮 velocity | 无 |
| omni4_sim | litesim3d_ros2_control/OmniDriveController | 四个轮 velocity | 无 |

差速底盘见 [双 MID360 案例](双MID360从零搭建.md)。这里的全向轮是带被动滚子的普通全向轮；四轮采用角点切向 X 布局。

## 1. 构建并运行

先按[安装依赖](../usage/安装与构建.md#安装依赖)准备 Ubuntu 22.04、ROS 2 Humble 和 Bash。已有源码时直接进入仓库：

```bash
source /opt/ros/humble/setup.bash
git clone --branch humble https://github.com/chiway-luo/litesim3d_ros2.git
cd litesim3d_ros2
rosdep install --from-paths . --ignore-src --rosdistro humble -y
bash scripts/build.sh
source "$(python3 scripts/paths.py install)/setup.bash"
export ROS_DOMAIN_ID=70 ROS_LOCALHOST_ONLY=1
ros2 launch ackermann_sim ackermann.launch.py gui:=false
```

三轮或四轮全向分别改用 `ros2 launch omni3_sim omni3.launch.py gui:=false`、`ros2 launch omni4_sim omni4.launch.py gui:=false`，每次只运行一个案例。

终端 B 进入同一源码目录并加载相同环境：

```bash
source /opt/ros/humble/setup.bash
source "$(python3 scripts/paths.py install)/setup.bash"
export ROS_DOMAIN_ID=70 ROS_LOCALHOST_ONLY=1
ros2 control list_controllers
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.4}, angular: {z: 0.2}}'
```

状态广播器和对应底盘控制器应为 active，小车开始前进转弯。全向案例可将输入改为 `'{linear: {y: 0.3}}'` 观察横移；阿克曼不接受横移。

Ctrl+C 停止发布后，控制器在 0.5 仿真秒输入超时后停车。终端 A 按 Ctrl+C 结束仿真。省略 `gui:=false` 打开三维窗口；`rviz:=true` 打开 RViz；`cameras:=true` 开启相机，需要可用图形后端。

## 2. 找到配置文件

每个案例独立安装。例如 `examples/ackermann/ackermann_sim/`：

| 文件 | 配置内容 |
| --- | --- |
| worlds/room.sdf | 地面、障碍物、机器人占位和仿真步长 |
| config/drive.yaml | 只绑定驱动/转向关节角色；所有几何来自机械 URDF |
| urdf/mechanical.urdf.xacro | 车体、车轮、转向关节和 URDF 限位 |
| urdf/sensor_mounts.xacro | 雷达、IMU、相机的安装 link 和固定关节 |
| urdf/robot.urdf.xacro | 机械结构与传感器安装，单独展开给 RSP |
| urdf/sensors.xacro | 引用安装结构并配置传感器插件 |
| urdf/simulation.urdf.xacro | 组合完整模型，选择仿真底盘和编码器里程计 |
| urdf/control.urdf.xacro | 机械模型加 ros2_control 硬件与关节接口 |
| config/runtime.yaml | control_mode、碰撞策略、地形和机器人身份 |
| config/ros.yaml、rsp.yaml | 出生位姿、仿真节点参数 / RSP 参数 |
| config/controllers.yaml | 管理器频率、控制器和 broadcaster 参数 |
| launch/ackermann.launch.py | 使用 ld 组装上述文件与节点 |

自己的工程可将机械与安装结构移入独立 description 包，将 world、runtime、控制器与 launch 放在自己的 bringup 包，再修改 `get_package_share_directory()` 使用的包名。

## 3. 配置场景和传感器

`room.sdf` 的地面中心为 z=-0.05、厚度 0.1 m，因此顶面为 z=0。默认出生位姿从 `config/ros.yaml` 的 `initial_pose_override` 加载，值为 `[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]`，顺序是 `[x,y,z,qx,qy,qz,qw]`。基准 link 在地面投影，轮心高度等于轮半径。机器人 SDF model 保留身份和兼容默认位姿，完整几何交给组合 URDF；环境实体要有 visual 和 collision。

完整文件：[worlds/room.sdf](../../examples/ackermann/ackermann_sim/worlds/room.sdf)。

`runtime.robot.id` 必须等于 world 中机器人 model 的 name。三轮/四轮例子分别是 `omni3`、`omni4`。地面通过 entity_bindings.ground.support 参加轮式地形支撑。改变出生位置时修改 ROS 参数文件，用 `params:=绝对路径` 加载；坐标与参数格式见[初始位姿](../usage/接入与使用手册.md#初始位姿和运行中重置)。

安装位置写入 `sensor_mounts.xacro`，RSP 和仿真共用同一份 link/joint；传感器插件放在 `sensors.xacro`。雷达以 lidar_link 为测量坐标，IMU 为 imu_link，相机使用光学坐标。阿克曼的传感器配置见下面的完整文件。

完整文件：[urdf/sensors.xacro](../../examples/ackermann/ackermann_sim/urdf/sensors.xacro)。

## 4. 声明硬件和关节接口

阿克曼的后轮接 velocity；左右转向关节接 position。两个前轮旋转关节只声明状态接口，它们按实际行程滚动。位置与速度接口同时声明时仍只能由一个控制器占用其中一种。

完整文件：[urdf/control.urdf.xacro](../../examples/ackermann/ackermann_sim/urdf/control.urdf.xacro)。

三轮全向声明三个独立轮；四轮全向按相同格式再加入 wheel_3_joint。每个主动轮提供 velocity 和可选 position，joint_state_broadcaster 读取真实 position/velocity。

完整文件：[urdf/control.urdf.xacro](../../examples/omni3/omni3_sim/urdf/control.urdf.xacro)。

所有主动关节在机械 URDF 中都必须有有限正的 velocity 限制。转向还需 lower/upper；本例为 ±0.6 rad。position 目标按关节速度上限逐步追踪，关节及车体碰撞会约束实际运动，反馈来自接受后的状态。

硬件 command_topic/state_topic 与仿真 joint_commands_topic/joint_feedback_topic 指向同一对话题即可；重命名或放到机器人命名空间时同时改两端，RSP 节点名不参与硬件连接。

## 5. 配置控制器

阿克曼使用 [Humble 标准控制器](https://control.ros.org/humble/doc/ros2_controllers/ackermann_steering_controller/doc/userdoc.html)。其数组顺序为右、左，front_wheels_names 填转向关节名称。轴距、轮距和轮径由 launch 的 `UrdfControllerParameters` 从完整控制 URDF 自动生成，源文件只保留关节绑定和控制设置。

完整文件：[config/controllers.yaml](../../examples/ackermann/ackermann_sim/config/controllers.yaml)。

标准控制器接收 vx/wz，不支持原地旋转。算法应满足 `abs(wz/vx) <= tan(max_steering)/(wheelbase + track_width/2*tan(max_steering))`；本例约为 0.92849 m⁻¹（实际限幅应使用公式值）。零 vx 时也发送零 wz。越界的转向 position 指令会整包拒绝，并由仿真 command_timeout 停车；控制器不会代替上层算法做曲率限幅。

三轮全向控制器将 vx、vy、wz 换算成轮速；`wheel_joints` 指定轮顺序，轮位、滚动方向、轮径及机械速度限位由完整 URDF 自动生成。轮速超限时等比降低所有轮速，保持运动方向。

完整文件：[config/controllers.yaml](../../examples/omni3/omni3_sim/config/controllers.yaml)。
四轮全向使用同一个控制器，绑定四个轮关节：

完整文件：[config/controllers.yaml](../../examples/omni4/omni4_sim/config/controllers.yaml)。

全向控制器做逆运动学和速度限幅，不做路径跟踪或加速度规划。算法需要连续发布速度并自行规划平滑轨迹。两个控制器的算法超时均为 0.5 仿真秒；这与硬件失联的仿真 command_timeout 是不同层级。

## 6. 配置 runtime 和 ROS 参数

启用 ros2_control 后，仿真不订阅 cmd_vel，也不发布公共 joint_states。模型路径留空，由 launch 动态查找包资源并传入。

完整文件：[config/runtime.yaml](../../examples/ackermann/ackermann_sim/config/runtime.yaml)。

三种案例均选择仿真编码器里程计作为 /odom 和 odom→base_link 的唯一来源；RSP 负责内部 link TF。这样第三方关节控制器接管时里程计仍可使用。阿克曼控制器自己的 odometry 保留在其私有话题，enable_odom_tf=false。

完整文件：[config/ros.yaml](../../examples/ackermann/ackermann_sim/config/ros.yaml)。

完整文件：[config/rsp.yaml](../../examples/ackermann/ackermann_sim/config/rsp.yaml)。

## 7. 编写自己的 launch

分别展开 robot.urdf.xacro 给 RSP、control.urdf.xacro 给 controller_manager；simulation.urdf.xacro 通过 robot_urdf 传给公共 launch 展开。下面的 launch 显式列出 runtime、仿真 ROS、RSP 与控制器参数文件，普通仿真选项使用公共 launch 默认值。

完整文件：[launch/ackermann.launch.py](../../examples/ackermann/ackermann_sim/launch/ackermann.launch.py)。

[omni3.launch.py](../../examples/omni3/omni3_sim/launch/omni3.launch.py) 和 [omni4.launch.py](../../examples/omni4/omni4_sim/launch/omni4.launch.py) 使用同一结构：包名分别改为 omni3_sim/omni4_sim，spawner 激活 omni_controller，将 /omni_controller/cmd_vel remap 到 /cmd_vel。源码案例已完整配置。

RSP 的 joint_states、tf、tf_static、robot_description 都有显式 remap。修改 RSP 名称、命名空间时，保留对应话题连接即可。controller_manager 不使用全局 name remap，避免连带改变其动态加载的控制器节点名。

## 8. 第三方算法直接接管关节


若算法输出车体 Twist，直接发布 /cmd_vel。若算法输出每轮速度或转向角，先加载本案例预留的 ForwardCommandController，再在一次切换中停用旧控制器并启用新控制器。保持 joint_state_broadcaster 和 RSP 运行，不增加 joint_state_publisher。

阿克曼：

```bash
ros2 run controller_manager spawner wheel_velocity_controller steering_position_controller --inactive
ros2 control switch_controllers --deactivate ackermann_controller --activate wheel_velocity_controller steering_position_controller --strict
ros2 topic pub --once /steering_position_controller/commands std_msgs/msg/Float64MultiArray '{data: [0.3, 0.25]}'
ros2 topic pub -r 20 /wheel_velocity_controller/commands std_msgs/msg/Float64MultiArray '{data: [2.0, 2.0]}'
```

转角数组顺序为左、右，单位 rad；轮速数组为后左、后右，单位 rad/s。前轮旋转为只读反馈。恢复速度控制前停止上一条发布，再切换：

```bash
ros2 control switch_controllers --deactivate wheel_velocity_controller steering_position_controller --activate ackermann_controller --strict
```

全向案例：

```bash
ros2 run controller_manager spawner wheel_velocity_controller --inactive
ros2 control switch_controllers --deactivate omni_controller --activate wheel_velocity_controller --strict
# 三轮：按 wheel_0、wheel_1、wheel_2；这一组约为沿 +y 以 0.3 m/s 横移。
ros2 topic pub -r 20 /wheel_velocity_controller/commands std_msgs/msg/Float64MultiArray '{data: [3.0, -1.5, -1.5]}'
# 四轮改发四个数：[2.12132, -2.12132, -2.12132, 2.12132]。
```

恢复时停用 wheel_velocity_controller、启用 omni_controller。自定义 ros2_control 控制器同样通过 controller_manager 声明并占用这些接口，无需改仿真器。

ForwardCommandController 会保持最近值，没有算法消息超时；停止算法前发送零轮速或停用它。需要自动超时停车的第三方关节控制器应自行实现消息看门狗。硬件通信继续时，仿真 command_timeout 不能替代算法超时。接口为运动学位置/速度控制，不提供 effort 或轮胎接触力动力学。
