# MID-360 网格来源与坐标

`livox_mid360.dae` 由真实的 `MID-360_4_ASM` STEP 装配体转换，保留散热鳍片、光学圆顶、壳体、底部孔位和侧面 M12 接插件几何。没有用基本体重画雷达。

- 厂商资料：[Livox MID-360 下载页](https://www.livoxtech.com/mid-360/downloads)，其中的 3D Model 发布于 2023-01-10。
- 厂商 STEP 地址：[mid-360-asm.stp](https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Mid360/mid-360-asm.stp)。本次本机无法从该 CDN 下载，实际读取的是 HKU-MARS 收录的[同名 STEP 文件](https://github.com/hku-mars/LIV_handhold_2/blob/5b87693d52f3d1ce4d58f2dec872e6af4d842d5c/liv_eye_cad/mid-360-asm.stp)。未声称已核对两个下载端的字节一致性。
- 文件头：`MID-360_4_ASM`，Creo Parametric，2023-01-06。原文件压缩保存在 `source/mid-360-asm.stp.gz`，解压后 SHA-256 为 `c28576589f443c77cec7a8574d9afb9632ff83e3e876c68b5eb5b8f88aaf9d45`。
- 转换脚本：[convert_mid360_step.py](../tools/convert_mid360_step.py)。使用 Open CASCADE / `cadquery-ocp==8.0.1.1.0`，遍历全部 1715 个 CAD 面进行三角化，滤除 54 个数值退化三角片，输出 117483 个有效三角形；线性偏差 0.15 mm，角度偏差 0.35 rad。转换环境独立于 ROS，用户构建或启动仿真不需要安装 CAD 软件。
- [mid360_mesh.json](mid360_mesh.json) 记录源文件与 DAE 的哈希、面数、三角形数和米制包围盒。壳体银灰、光学窗深色、接口深灰为显示材质；没有把 CAD 工程配色冒充实物表面材质测量。

## 坐标与装配

CAD 使用毫米、Y 朝上、底面 Y = −25.9171 mm，接插件在 +X。输出使用米制、Z 朝上、+X 背离接插件；按[用户手册尺寸图与坐标图](https://terra-1-g.djicdn.com/851d20f7b9f64838a34cd02351370894/Livox/Livox_Mid-360_User_Manual_EN.pdf)将扫描原点 O 放在底面上方 47 mm：

```text
(x, y, z)_scan = (-X, Z, Y - 21.0829)_CAD / 1000
```

DAE 的 `unit` 为米，`up_axis` 为 `Z_UP`；URDF 不再缩放或旋转这个网格。底面在扫描坐标 z = −0.047 m，装到厚 8 mm 的倾斜安装板时，扫描 link 在安装板中心上方 0.051 m。两者沿安装板法向一起倾斜，实际 CAD 底面与安装板上表面贴合。此坐标来自尺寸图，不等于逐台实机的外参标定。

`unit_wedge.stl` 由 [generate_wedge.py](../tools/generate_wedge.py) 生成二进制 STL，供查看器和 RViz 共同读取；它是本示例自制的封闭三棱柱，和雷达 CAD 无关：x/y ∈ [−0.5, 0.5]，下表面 z = 0，上表面 z = 0.5 − x。Xacro 将它缩放成 100 × 80 mm 的楔块，高度为 `100 mm × tan(倾角)`；楔块、安装板和雷达的方向同步计算。0° 时省略楔块实体，安装板直接贴转接底板。

## 权利说明

MID-360 原始 CAD 及其派生网格的权利归原权利人；下载资源未附独立开源许可证，不纳入本项目的 Apache-2.0 授权。原始文件、来源和转换方式随模型保留。自制支架、楔块和转换脚本采用本包的 Apache-2.0 许可。
