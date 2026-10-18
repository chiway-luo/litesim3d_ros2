# LiteSim models

本包仅安装模型资源，不包含运行节点。通过 ament 查询 `share/litesim_models`，`model://` 搜索根为其中的 `models/`。

四轮车及基本场景从本仓库原有示例几何迁移；`textured_crate` 的 OBJ、MTL、棋盘图片为本项目制作，DAE 由同一 OBJ 导出并明确声明 Z_UP，用作多材质和贴图回归资产。上述资源遵循仓库 Apache-2.0 许可证，无外部商业模型或下载贴图。

小车默认世界为 `worlds/office.sdf`，无人机为 `worlds/drone_arena.sdf`；`worlds/warehouse.sdf` 保留为小型回归场景。办公区、无人机场地和四旋翼由仓库 `scripts/fixtures/generate_example_worlds.py` 生成，SDF 文件已提交，无需用户先执行生成器。增加模型时保留 `model.config`、`model.sdf` 与相对 meshes/materials 路径；SDF 的 visual 和 collision 分别提供外观及碰撞几何。兼容范围见仓库 `docs/usage/SDF与独立窗口.md`。
