# Mid-360 扫描参考表

`mid360.csv` 原样取自 [Livox-SDK/livox_laser_simulation](https://github.com/Livox-SDK/livox_laser_simulation)，MIT 授权保留在 `LICENSE.livox`。

- 固定版本：`1cce1073633a062b92e30243a4c2920e45551bb5`。
- [原始文件](https://raw.githubusercontent.com/Livox-SDK/livox_laser_simulation/1cce1073633a062b92e30243a4c2920e45551bb5/scan_mode/mid360.csv)：800000 行，16809997 字节。
- SHA-256：`aa1fc08b6a4400608dbd6ee832b7ea3a9c3c37197e734f60f58fe5abf762269a`。
- 机器可读来源记录：`mid360.source.json`。

原 CSV 的 `Time/s` 列实际是从 1 开始的连续序号，并非秒数。仿真按原始行序，以 200000 个方向/秒发射；逐点间隔 5 μs。第二列是方位角，第三列是天顶角，`elevation = 90° - zenith`；坐标为 x 前、y 左、z 上。方向与上游插件的旋转约定一致。

配置 `scan_pattern: livox_mid360` 和 `pattern_file: package://litesim3d_core/lidar_patterns/mid360.csv`。10 Hz 使用 20000 槽/帧；20 Hz 使用 10000 槽/帧。强制 `columns × rate_hz = 200000`，改变封帧频率不会改变发射顺序。800000 个方向用完后循环，周期为 **4 秒**。

视场参数仅裁剪原有方向，保留采样槽位和时间，不拉伸轨迹或补点。官方表仰角约 −7.2123°~52.164°，默认按标称 −7°~52°裁剪全表的 0.21775%；裁掉的槽位在原始点云和辅助边界中都为 NaN。实际返回仍由当前仿真场景、遮挡、距离和测距噪声决定。

这是官方公开的**仿真扫描表**，不是逐台设备标定或无限非重复的光学模型。不包含发散角、材料/天气衰减、多回波或 Livox UDP/CustomMsg；输出仍是项目的 `xyzirt_v1` PointCloud2，`ring=0` 为兼容占位。

加载时校验 CSV 格式、连续序号、有限角度和资源预算（文件≤64 MiB、方向≤2000000），并记录实际文件 SHA-256 到 manifest。同一配置的多雷达及共享 world 复用只读方向表。自定义表内容会改变轨迹，不能沿用官方一致性结论。
