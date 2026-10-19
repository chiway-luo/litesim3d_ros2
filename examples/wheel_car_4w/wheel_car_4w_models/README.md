# wheel_car_4w_models：小车模型与场景

本包仅安装模型资源，不包含运行节点。通过 ament 查询 `share/wheel_car_4w_models`，`model://` 搜索根为其中的 `models/`。

四轮车及基本场景从本仓库原有示例几何迁移；`textured_crate` 的 OBJ、MTL、棋盘图片为本项目制作，DAE 由同一 OBJ 导出并明确声明 Z_UP，用作多材质和贴图回归资产。上述资源遵循仓库 Apache-2.0 许可证，无外部商业模型或下载贴图。

```text
wheel_car_4w_models/
├── models/
│   ├── wheel_car_4w/       # 四轮车与传感器安装位置
│   ├── office_layout/      # 办公区家具与墙体
│   ├── textured_crate/     # 本包自带的网格、材质与贴图
│   └── ...                # 仓库场景的地板、墙体与障碍物
└── worlds/
    ├── office.sdf          # 默认办公区
    └── warehouse.sdf       # 小型回归场景
```

本包仅保存小车模型和场景。无人机资源由独立的 `drone_3d_models` 包安装，两者分别携带各自场景所需的贴图。

办公区模型及世界文件由仓库 `scripts/fixtures/generate_example_worlds.py` 生成，SDF 文件已提交，无需用户先执行生成器。增加模型时保留 `model.config`、`model.sdf` 与相对 meshes/materials 路径；SDF 的 visual 和 collision 分别提供外观及碰撞几何。兼容范围见仓库 `docs/usage/SDF与独立窗口.md`。
