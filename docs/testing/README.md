# 测试文档

2026-10-18 的 SDF 资源、独立窗口、碰撞预算和真实速率验收见 [SDF 实现验证](SDF实现验证.md)。

这里集中保存回归入口、验收门槛与历史记录。普通安装和运行不需要测试证据，见 [使用文档](../usage/README.md)。以下命令从源码仓库根目录、已激活 Humble 的 Bash 终端执行。

## 当前源码的检查与回归

先构建并加载安装环境：

```bash
bash scripts/build.sh
source "$(python3 scripts/paths.py install)/setup.bash"
```

若使用标准 colcon 工作空间，先加载该工作空间 `install/setup.bash`，并设置 `LWS_INSTALL_PREFIX` 为它的绝对路径；构建必须启用 `BUILD_TESTING`，`build/` 与 merged `install/` 应为同级目录。

| 范围 | 命令 | 实际作用 |
| --- | --- | --- |
| 仓库文档 | `python3 scripts/checks/check_repository_docs.py` | 不依赖历史日志，检查仓库内链接、使用流程中的个人路径与 Shell 语法 |
| 完整功能回归 | `bash scripts/test_m3.sh` | 内核、M1/M2/M3 ROS、100 次会话重置、报告门禁负例、四轮示例、单机/多机 launch |
| M1 / M2 子集 | `bash scripts/test_m1.sh` / `bash scripts/test_m2.sh` | 按阶段选择测试，非完整新版本验收 |
| 安装后独立运行 | `python3 scripts/checks/check_m3_install.py` | 新目录构建、隐藏源码副本后验证安装资源和节点 |
| 四轮示例 | 见 [四轮车与雷达测试](四轮车与雷达测试.md) | 真实 ROS 接口、配置、TF 和子进程退出 |

`test_m3.sh` 默认不启动 GUI、正式性能窗口或 8 小时长稳。测试失败返回非零；日志/JUnit 输出到 `evidence/m3/run-*`，每轮单独保存。`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` 用于避开不兼容的自动加载插件，脚本会设置，不修改 Python 环境安装包。

正式性能测量需单独运行，避免同时构建或执行其他测试：

```bash
LWS_M3_PERFORMANCE=1 bash scripts/test_m3.sh
```

该选项增加 B3 的两种节拍、每种 5 轮、每轮 30 秒预热与 120 秒测量。性能阈值见 [M3 基线](M3基线.md)。长稳是单独的验收项目，不用短回归代替。

Linux 独立验收脚本为 `python3 scripts/checks/check_linux.py`，图形一致性检查需 Xvfb 或桌面会话；依赖和命令见 [CI](../../.github/workflows/m1.yml)。检测器构建可用 `SIM_SANITIZE=ON`（ASan/UBSan）或 `SIM_TSAN=ON`（独立 TSan），不可混用；`SIM_COVERAGE=ON` 用于 Clang 覆盖率构建。它们均需单独的构建目录与实际结果。

## 结果与能力边界

初始历史验收平台为 macOS arm64 / RoboStack Humble。历史报告按当时的源码指纹、版本和运行条件读取；同日后续改名、目录调整、雷达及重置修复不能自动继承全部历史结论。

- [近期修复与验证](近期修复与验证.md)：目录整理、四轮车/雷达和重置首帧问题的实测记录。
- [文档发布检查](文档发布检查.md)：本次通用上手流程与独立检出的验证结果。
- [M1 报告](M1测试报告.md)、[M2 报告](M2测试报告.md)、[M3 报告](M3测试报告.md)、[文档补全报告](文档对照与补全报告.md)：历史结果，不能作为当前提交全量复测的证明。
- 外部 SLAM/Nav2 闭环、8 小时长稳与真实设备验收仍未完成；Linux 的历史补充记录也不等于当前提交通过。远端 CI 以对应提交的 GitHub Actions 实际结果为准。

## 规范与基线

| 文档 | 用途 |
| --- | --- |
| [测试与验收规范](测试与验收规范.md) | 数值容差、覆盖率与验收要求 |
| [测试用例与需求追踪](测试用例与需求追踪.md) | 原始 140 条规格及需求映射 |
| [M1](M1基线.md)、[M2](M2基线.md)、[M3](M3基线.md) 基线 | 各阶段冻结范围与平台决定 |
| [测试报告模板](测试报告模板.md) | 新测试报告格式 |
| [文档检查记录](文档检查记录.md) | 早期文档审核历史 |

## 原始证据与历史报告生成

仓库只提交源码、文档与测试程序。大型日志、运行清单、JUnit、点云、覆盖率与旧构建树留在外部产物目录，不随 clone 下载。报告中的 `evidence/...` 是逻辑证据路径，不是仓库内超链接；查询本机位置：

```bash
python3 scripts/paths.py evidence
```

例如 `evidence/reset-first-frame-20261018/verification.json` 对应该输出目录下的 `reset-first-frame-20261018/verification.json`。新检出只能生成自己的运行证据，不能据缺失的历史文件声称旧结果已验证。GitHub Actions 新产生的日志作为对应 workflow run 的 artifact 下载。

`scripts/reports/report_*.py`、`check_docs.py` 和 `check_m2_docs.py` 是历史证据审计/报告工具，依赖完整的对应日志、固定门槛和源码指纹。它们不是首次运行步骤，也不能在空 clone 上代替新测试。生成器若缺少或发现过期证据，应失败而非生成通过结论。通用文档检查使用上表的 `check_repository_docs.py`。
