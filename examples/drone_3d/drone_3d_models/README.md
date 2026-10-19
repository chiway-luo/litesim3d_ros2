# drone_3d_models：无人机模型与场景

本包仅安装无人机示例的资源，不包含运行节点。通过 ament 查询 `share/drone_3d_models`，`model://` 搜索根为其中的 `models/`。

```text
drone_3d_models/
├── models/
│   ├── quadrotor/          # 四旋翼机体、传感器安装位置与碰撞几何
│   ├── aerial_course/      # 门框、立柱、平台、货架与起降区
│   └── textured_crate/     # 本包自带的网格、材质与贴图
└── worlds/drone_arena.sdf  # 飞行场地与无人机初始位姿
```

模型、场景及其引用的贴图均保存在本包内。`textured_crate` 是项目自制测试资产的独立副本，无人机资源不引用小车模型目录。上述资源使用包清单声明的 Apache-2.0 许可证。

四旋翼、飞行场地及世界文件由仓库 `scripts/fixtures/generate_example_worlds.py` 生成，文件已随源码提供。增加模型时保留 `model.config`、`model.sdf` 与相对 `meshes/`、`materials/` 路径；SDF 的 visual 和 collision 分别定义外观与碰撞几何。
