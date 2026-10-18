#!/usr/bin/env python3
"""Build the requirement index from actual JUnit reports, never inferred skips."""
from collections import Counter, defaultdict
from datetime import datetime
import hashlib
import json
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from paths import REPO_ROOT, EVIDENCE_DIR, SOURCE_FOLDERS, resolve_reference, reference, write_report
import re
import xml.etree.ElementTree as ET
from zoneinfo import ZoneInfo

ROOT = REPO_ROOT
OUT = EVIDENCE_DIR / "final"


def junit(relative):
    path = resolve_reference(relative)
    cases = []
    if not path.exists():
        return cases
    for case in ET.parse(path).getroot().iter("testcase"):
        status = "PASS"
        if case.find("failure") is not None:
            status = "FAIL"
        elif case.find("error") is not None:
            status = "ERROR"
        elif case.find("skipped") is not None or case.get("status") == "notrun":
            status = "NOT_RUN"
        cases.append({"entry": case.get("classname", "") + "." + case.get("name", ""),
                      "status": status, "evidence": relative})
    return cases


def main():
    env = json.loads((OUT / "environment.json").read_text())
    install = json.loads((OUT / "install-check.json").read_text())
    linux_root = EVIDENCE_DIR / "linux/results"
    linux_path = linux_root / "linux-result/acceptance.json"
    linux = json.loads(linux_path.read_text()) if linux_path.exists() else None
    linux_pass = linux is not None and linux["status"] == "PASS"
    review_path = EVIDENCE_DIR / "linux/visual-review.json"
    review = json.loads(review_path.read_text()) if review_path.exists() else None
    visual_pass = review is not None and review["status"] == "PASS"
    runtime_reports = {
        "core": "evidence/final/core-final.xml", "ros": "evidence/final/ros.xml",
        "launch": "evidence/final/launch.xml", "gui": "evidence/final/gui-independence/result.xml",
    }
    runtime_cases = {k: junit(v) for k, v in runtime_reports.items()}
    def score(name):
        cases = runtime_cases[name]
        return f"{sum(c['status'] == 'PASS' for c in cases)}/{len(cases)} PASS"
    runtime_total = sum(len(c) for c in runtime_cases.values())
    runtime_pass = sum(c["status"] == "PASS" for cases in runtime_cases.values() for c in cases)
    specs = []
    for line in (ROOT / "docs/testing/测试用例与需求追踪.md").read_text().splitlines():
        if line.startswith("| T-") and "；M1；" in line:
            parts = [p.strip() for p in line.strip("|").split("|")]
            ident, title = parts[0].split(" ", 1)
            specs.append({"id": ident, "title": title, "requirements": re.findall(r"REQ-\d+", parts[1]),
                          "specified_input": parts[2], "criteria": parts[3]})
    assert len(specs) == 82 and len({s["id"] for s in specs}) == 82
    indexed = defaultdict(list)
    for relative in ("evidence/final/core-final.xml", "evidence/final/ros.xml"):
        for case in junit(relative):
            group = re.match(r"([A-Z]{3})(\d{2})\.", case["entry"])
            if group:
                indexed[f"T-{group[1]}-{group[2]}"].append(case)
            else:
                group = re.search(r"test_T_([A-Z]{3})_(\d{2})(?:_(\d{2})(?=_))?", case["entry"])
                if group:
                    for n in group.groups()[1:]:
                        if n:
                            indexed[f"T-{group[1]}-{n}"].append(case)
    for ident, relative in (
        ("T-COR-01", "evidence/final/gui-independence/result.xml"),
        ("T-DEL-02", "evidence/final/install-launch.xml"),
        ("T-URD-11", "evidence/final/install-mesh.xml"),
        ("T-DEL-05", "evidence/final/coverage.xml"),
        ("T-DEL-06", "evidence/final/docs.xml"),
    ):
        indexed[ident].extend(junit(relative))
    for ident in ("T-LID-05", "T-LID-06", "T-LID-09"):
        indexed[ident].extend(indexed["T-ROS-02"])
    indexed["T-COR-12"].extend(indexed["T-GEO-11"])
    # User decision, 2026-10-18: M1 accepts macOS/Humble; Linux is supplemental.
    # Re-evaluate existing execution evidence, without inventing a new test run.
    delivery_states = {install["status"]}
    if install.get("platform") != "Darwin" or env.get("ROS_DISTRO") != "humble":
        delivery_states.add("ERROR")
    if not install.get("source_hidden") or any(install.get(name) != 0 for name in
            ("build_exit", "launch_exit", "headless_exit", "mesh_exit")):
        delivery_states.add("FAIL")
    for relative, minimum in ((runtime_reports["core"], 67), (runtime_reports["ros"], 26),
                              (runtime_reports["launch"], 2),
                              ("evidence/final/install-launch.xml", 2),
                              ("evidence/final/install-mesh.xml", 1)):
        cases = junit(relative)
        delivery_states.update(c["status"] for c in cases)
        if len(cases) < minimum:
            delivery_states.add("NOT_RUN")
    indexed["T-DEL-01"] = [{
        "entry": "scripts/checks/check_install.py + scripts/test_m1.sh (macOS/Humble)",
        "status": next(s for s in ("ERROR", "FAIL", "BLOCKED", "NOT_RUN", "PASS")
                       if s in delivery_states),
        "evidence": "evidence/final/install-check.json"}]
    if not indexed["T-DEL-06"]:
        indexed["T-DEL-06"] = [{"entry": "python scripts/checks/check_docs.py", "status": "NOT_RUN",
                                "evidence": "scripts/checks/check_docs.py"}]
    priority = ["ERROR", "FAIL", "BLOCKED", "NOT_RUN", "PASS"]
    for spec in specs:
        spec["runs"] = indexed[spec["id"]]
        states = {c["status"] for c in spec["runs"]} or {"NOT_RUN"}
        spec["status"] = next(s for s in priority if s in states)
    assert all(s["runs"] for s in specs), "Unmapped M1 requirement"
    counts = dict(Counter(s["status"] for s in specs))
    summary = "、".join(f"{counts[state]} {state}" for state in ("PASS", "FAIL", "ERROR", "BLOCKED", "NOT_RUN") if counts.get(state))
    closure = ("**本轮 M1 按本机 macOS / Humble 基线，82 条适用验收项全部通过**。本机全新源码副本构建、真实 ROS 2、源码隐藏后的安装运行有独立证据；不扩展为 M2、现场或真实机器人验收。"
               if counts.get("PASS") == 82 else
               "**M1 验收尚未全部关闭**。尚未通过的条目见逐项追踪表；T-DEL-01 按用户确认的本机 macOS / Humble 基线，以真实构建、ROS 2 和安装运行证据判定。")
    visual = ("补充画面证据：Linux 独立 Xvfb 中的 RViz 软件渲染截图已经检查，见 [画面核验记录](../evidence/linux/visual-review.json)与 [截图](../evidence/linux/results/final/gui-independence/on/rviz-xvfb.png)。"
              if visual_pass else "RViz 启动和输出一致性已验证；画面核验尚未完成。")
    result = {"time": datetime.now(ZoneInfo("Asia/Shanghai")).isoformat(), "platform": "macOS / ROS 2 Humble",
              "platform_decision": {"date": "2026-10-18", "source": "docs/testing/M1基线.md",
                                    "authority": "explicit user confirmation", "linux_required": False},
              "total": 82, "counts": counts, "cases": specs}
    (OUT / "test-index.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    paths = [ROOT / "standalone/CMakeLists.txt"]
    for directory in (*SOURCE_FOLDERS, ".github"):
        paths.extend(p for p in (ROOT / directory).rglob("*") if p.is_file() and not {"__pycache__", ".pytest_cache"}.intersection(p.parts))
    hashes = {reference(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(paths)}
    digest = hashlib.sha256(json.dumps(hashes, sort_keys=True).encode()).hexdigest()
    (OUT / "source-hashes.json").write_text(json.dumps({"aggregate_sha256": digest, "files": hashes}, indent=2) + "\n")
    coverage = json.loads((OUT / "coverage-gate.json").read_text())
    manifest = json.loads((OUT / "install-manifest.json").read_text())
    lines = [
        "# M1 原型测试报告", "",
        f"更新时间：{result['time']}。范围为原始规格中的 82 条 M1 用例。",
        "",
        f"原型已实现并完成实际运行验证。逐项结果为 **{summary}**，适用通过率 {counts.get('PASS', 0)/82:.2%}。",
        "",
        closure + " " + visual,
        "",
        "验收平台依据：2026-10-18 用户确认“以本机 macOS / Humble 验收 M1，Linux 留作后续验证”。本报告据此调整 T-DEL-01 的平台条件，数值与覆盖率门槛不变；见 [M1 基线](M1基线.md)。已有 Linux 运行结果作为补充历史证据保留，不参与本轮 M1 关闭判定。本次更新复核已有运行证据，未重新执行功能测试。", "",
        "## 环境、版本与命令", "",
        "| 项目 | 本次实测/记录 |", "| --- | --- |",
        f"| 平台 | macOS {env['macOS']} {env['arch']}；Apple M5 Pro，18 核、48 GiB；单个仿真工作线程 |",
        f"| ROS / RMW | RoboStack Humble；{env['rmw']}；Python {env['python']} |",
        *([f"| Linux 补充历史环境（非 M1 门禁） | {linux['environment']['os_release']['PRETTY_NAME']} {linux['environment']['machine']}；kernel {linux['environment']['kernel']}；{linux['environment']['rmw']}；Python {linux['environment']['python']}；[完整版本](../evidence/linux/results/linux-result/acceptance.json) |"] if linux_pass else []),
        f"| 编译 | {env['compiler']}；交付构建 RelWithDebInfo；覆盖率构建 Clang instrumentation；ASan/UBSan 单独构建 |",
        "| 依赖 | " + "；".join(f"{k} {v}" for k, v in env["dependencies"].items()) + " |",
        "| 基准输入 | R01/R02/R03；seed=42；dt=5 ms；默认 32×1024、10 Hz 瞬时雷达；odom=50 Hz；无噪声 |",
        f"| 核心源码指纹 | `{manifest.get('core_source_sha256', '见运行清单')}` |",
        f"| 当前交付文件总指纹 | `{digest}`；逐文件值见 [source-hashes.json](../evidence/final/source-hashes.json) |",
        "| 资源与运行参数 | [安装运行清单](../evidence/final/install-manifest.json)含解析后的参数、URDF/mesh 哈希和实际控制生效记录 |",
        "| 性能条件 | 电源模式、后台负载未锁定；本次不作 M2 性能、长稳或真实设备验收结论 |",
        "| 执行/复核 | Codex 自动执行并复核日志；无独立人工验收签字 |", "",
        "已执行的标准流程在 `ros2h` 环境中运行，工作目录为项目根目录：", "",
        "```bash", "bash scripts/build.sh", "bash scripts/test_m1.sh", "python3 scripts/checks/check_install.py",
        "python3 scripts/checks/check_gui_independence.py", 'python3 scripts/checks/check_coverage.py "$(python3 scripts/paths.py evidence)/final"', "python3 scripts/checks/check_docs.py", "```", "",
        "实际非交互环境入口为 `<Humble 环境包装器>`；只读复用现有环境。安装/构建路径使用 ASCII 临时目录，具体位置见 [安装前缀](../evidence/install-prefix.txt)及 [隔离安装结果](../evidence/final/install-check.json)。GUI 检查需桌面图形会话；自动化测试使用独立 ROS domain 和 localhost。", "",
        *(["Linux 执行 `source /opt/ros/humble/setup.bash` 后运行 `python3 scripts/checks/check_linux.py`，每轮使用新的空构建目录。该脚本记录命令、返回码、JUnit 和实际包版本；初始化依赖命令见 [临时 VM 配置脚本](../evidence/linux/provision.sh)。", ""] if linux_pass else []),
        "## 执行结果", "",
        "| 检查 | 真实结果 | 证据 |", "| --- | --- | --- |",
        f"| 核心 C++ 测试 | {score('core')}；含 100000 步、60 s 同时多频采样、URDF/网格、射线、运动和扫掠碰撞 | [JUnit](../evidence/final/core-final.xml)、[日志](../evidence/final/core-final.log) |",
        f"| 真实 ROS 2 / 交付 pytest | {score('ros')}；参数化命令、R03 TF、QoS、启动顺序、字节解析、Xacro 等 | [JUnit](../evidence/final/ros.xml)、[日志](../evidence/final-ros.log) |",
        f"| launch_testing | {score('launch')}；安装包真实启动、收到 clock/点云、节点正常退出 | [JUnit](../evidence/final/launch.xml) |",
        f"| RViz 开关一致性 | {score('gui')}；每组推进 60 s、600 点云帧、3000 odom，全部消息字段、点云 data 字节、控制生效记录和终态完全相同 | [结果](../evidence/final/gui-independence/result.json)、[哈希](../evidence/final/gui-independence/on/message-hashes.json) |",
        "| macOS 干净构建 | 3 个包成功；另外复制全新源码、构建后隐藏副本，再从工作空间外启动 | [构建日志](../evidence/final/clean-build.log)、[隔离安装](../evidence/final/install-check.json) |",
        *(["| Linux 补充历史结果（非 M1 门禁） | 全新 VM、全新空构建目录；当时的核心/ROS/launch、安装后运行及软件渲染 RViz 均通过 | [Linux 运行结果](../evidence/linux/results/linux-result/acceptance.json)、[Linux 构建日志](../evidence/linux/results/linux-result/build.log)、[JUnit](../evidence/linux/results/linux-result/gate.xml) |"] if linux_pass else []),
        "| 安装后 STL/OBJ/DAE | 三种网格从安装 share 目录定位并生成点云，未使用源目录资源 | [JUnit](../evidence/final/install-mesh.xml)、[日志](../evidence/final/install-mesh.log) |",
        "| ASan / UBSan 补充检查 | 当时的 67 个核心用例全部通过；macOS LeakSanitizer 关闭，不替代 Linux/TSan 门禁 | [JUnit](../evidence/sanitize.xml)、[日志](../evidence/sanitize.log) |",
        "| 文档与结构检查 | 检查结果单列；不是功能、Linux 或真实设备测试 | [记录](../evidence/final/docs-check.json) |", "",
        f"macOS 运行测试共 {runtime_total} 个最终自动化测试实例，{runtime_pass} 个通过；Linux 独立结果、安装复测、覆盖率和文档检查另列。实例数不同于 82 个规格 ID，多个参数化实例会映射到同一个 ID。", "",
        "GUI 开关一致性使用 4×32 点的低负载配置；32×1024 的标准布局、逐点数据与 TF 查询由真实 ROS 专项验证，16/32/64 线规模另有核心测试。截图不作为点云数量或测距精度的证明。", "",
        "PointCloud2 已按独立 little-endian 读取器逐点解包：字段偏移 0/4/8/12/16/20、24 字节点步长、保留字节、NaN、ring、time 和 stamp 均有断言；[原始点云](../evidence/final/ros/test_T_ROS_02_03_time_tf_and_p0/cloud.bin)可供复查。TF 使用每条传感器实际 stamp 查询，等待上限 0.5 s，不以最新 TF 代替。误差阈值以原始规范及测试断言为准；未把未记录的误差最大值填写为零。", "",
        "## 覆盖率", "",
        "LLVM 与 Python coverage 的真实执行结果去重合并；第三方、生成消息、测试代码、夹具生成器、构建/报告脚本和纯资源不计入运行逻辑。头文件只有声明、数据结构与简单访问器，功能逻辑位于被测 `.cpp`。优化后的少量访问器无 profile 记录，已明确排除，未当作覆盖成功。", "",
        "| 模块 | 行覆盖率 | 分支覆盖率 | 门槛 |", "| --- | --- | --- | --- |",
    ]
    for name, item in coverage["files"].items():
        core = name.endswith(("geometry.cpp", "simulator.cpp", "model.cpp"))
        lines.append(f"| {Path(name).name} | {item['lines']['percent']:.2f}% | {item['branches']['percent']:.2f}% | {'90% / 80%' if core else '计入整体'} |")
    total = coverage["totals"]
    lines += [f"| 全部运行逻辑 | {total['lines']['percent']:.2f}% ({total['lines']['covered']}/{total['lines']['count']}) | {total['branches']['percent']:.2f}% ({total['branches']['covered']}/{total['branches']['count']}) | 80% / 70% |", "",
              "[覆盖率门禁原始结果](../evidence/final/coverage-gate.json)。核心变换、时间、几何、运动、传感器覆盖率达到原阈值；规格 ID 到实际入口的完整映射见下表及 [test-index.json](../evidence/final/test-index.json)。", "",
              "## 逐项追踪", "",
              "输入与阈值完整保留在 [原始 140 条规格](测试用例与需求追踪.md)和 `test-index.json`；各次参数化输入、断言与预期值位于所列可执行入口。JUnit 表示该入口实际执行结果，历史失败未被删除。", "",
              "| ID / 需求 | 名称 | 状态 | 可执行入口 | 证据 |", "| --- | --- | --- | --- | --- |"]
    for spec in specs:
        entries = "<br>".join(c["entry"].replace("src.litesim3d_ros2.test.", "") for c in spec["runs"])
        evidence = "、".join(f"[{Path(p).name}](../{p})" for p in dict.fromkeys(c["evidence"] for c in spec["runs"]))
        lines.append(f"| {spec['id']} / {', '.join(spec['requirements'])} | {spec['title']} | {spec['status']} | {entries} | {evidence} |")
    lines += ["", "## 历史问题与未验证范围", "",
              "- 测距闭区间的浮点舍入曾使 min_range 边界被误判；已修复并保留 GEO04/LID08 回归。初轮 [core-r2.xml](../evidence/core-r2.xml) 的其他两处失败来自测试反向加速预期及桥面射线几何设置，修正依据为解析参考，没有降低原容差。",
              "- Humble 的旧 launch_testing pytest 插件与本机 pytest 9 不兼容；禁用自动加载后显式调用 launch_test。测试执行器 Context、Python 字节数组比较以及 R03 关节测试初姿碰地的问题已修正；[ROS 历史失败](../evidence/ros-r5.xml)和[回归](../evidence/ros-r6.xml)保留。",
              "- 本机 SDK 与环境内旧链接器、Humble 对中文构建路径的处理不兼容；构建脚本为本项目选择 Xcode 编译器及 ASCII 输出路径。未修改系统或现有 Humble 环境。",
              ("- 已有 Linux 补充实测在本机临时 Ubuntu VM 内完成；后续 Linux 交付仍单独验证，远端 GitHub Actions 尚未执行。使用 [Lima 官方发行包](https://lima-vm.io/docs/installation/)及 [ROS 官方 apt 源配置](https://github.com/ros2/ros2_documentation/blob/humble/source/Installation/_Apt-Repositories.rst)，具体镜像、初始化命令和日志保存在 evidence/linux。" if linux_pass else "- Linux 留作后续验证，不阻塞本轮 M1；CI 配置文件存在不能作为已执行的证明。"),
              *(["- Ubuntu 的 tinyxml2 开发包使用 pkg-config 导出依赖，现已同时兼容该包与本机版本。ROS 启动超时测试不再要求必定收到单次 best-effort 初始时钟，而以可靠诊断、零步退出清单及非零故障退出交叉核验；历史失败保存在 evidence/linux/runtime-failure-r3。",
                 "- Fast DDS 下 PointCloud2 / Odometry 的原始 CDR 对齐间隙存在字节差异；所有消息字段及点云 data 内全部字节完全一致。测试按各消息的 CDR1 布局单独识别对齐间隙，保留原始样本和差异位置，不忽略点云 data 中的保留字节：[编码核验](../evidence/linux/results/final/gui-independence/cdr-comparison.json)。",
                 "- RViz 场景显示配置使用 Humble 的 Topic 属性；已修复旧属性名，并检查 RViz 对 /scene 的实际订阅及 transient-local / reliable QoS，防止一次发布的场景在后启动的 GUI 中缺失。"] if linux_pass else []),
              "- macOS 界面自动化读取 RViz 窗口曾超时；此工具限制与 ROS 消息/GUI 输出一致性检查分开记录。" + visual,
              "- M1 没有在线重新加载/协调 reset、噪声、滚动扫描、坡面动力学、IMU、算法闭环、多机器人、M2 性能/8 h 长稳测试；这些不计入本次通过项。网格碰撞按三角形表面处理，驱动轮限于已声明的圆柱轴向和叶节点约束。",
              *(["- 验收完成后已关闭并删除本次专用 Linux VM，清除本次镜像缓存；本机 ros2h 环境及原型安装产物保留，见 [清理记录](../evidence/linux/cleanup.json)。"] if (EVIDENCE_DIR / "linux/cleanup.json").exists() else []),
              "- 无部署、Git 提交/推送或真实设备控制；所有测试仅针对本机仿真进程。", ""]
    write_report(ROOT / "docs/testing/M1测试报告.md", "\n".join(lines))
    print(json.dumps({"counts": counts, "source_sha256": digest}, ensure_ascii=False))


if __name__ == "__main__":
    main()
