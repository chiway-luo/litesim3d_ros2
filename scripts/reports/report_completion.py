#!/usr/bin/env python3
"""Audit the original 140 specifications against versioned, real execution evidence.

Writes a new report only; historical M1/M2/M3 reports and raw results stay intact.
Missing, skipped, failed, stale or shortened evidence cannot pass current gates.
"""
import ast
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from paths import REPO_ROOT, EVIDENCE_DIR, SOURCE_FOLDERS, resolve_reference, reference, python_sources, write_report as save_report
import re
import subprocess
import xml.etree.ElementTree as ET
from reports.report_m3 import source_hash

ROOT = REPO_ROOT
OUT = EVIDENCE_DIR / 'completion-audit'
BRIDGE = ['src/sim_node.cpp', 'src/sim_world_node.cpp', 'src/point_adapter.cpp',
          'src/point_adapter.hpp', 'src/sim_ouster_adapter.cpp', 'src/sim_cloud_probe.cpp',
          'launch/demo.launch.py', 'launch/world.launch.py', 'scripts/sim_session']


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def bridge_hash():
    return hashlib.sha256(''.join(f'{name}:{digest(ROOT / "litesim3d_ros2" / name)}\n'
                                  for name in BRIDGE).encode()).hexdigest()


def read(path):
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return {}


def suite(path):
    try:
        return [{'name': c.get('classname', '') + '.' + c.get('name', ''),
                 'passed': all(c.find(tag) is None for tag in ('failure', 'error', 'skipped'))}
                for c in ET.parse(path).getroot().iter('testcase')]
    except (OSError, ET.ParseError):
        return []


def suite_passed(tests, count):
    return count > 0 and len(tests) == count and all(t['passed'] for t in tests)


def b3_passed(result, manifest, core, bridge, rtf):
    try:
        return (result['status'] == 'PASS' and result['rtf_target'] == rtf and not result['gui']
                and result['warmup_seconds'] == 30 and result['duration_seconds'] == 120
                and result['wall_seconds'] >= 120
                and (result['rtf'] >= 1 if rtf == 0 else .98 <= result['rtf'] <= 1.02)
                and result['internal_missed_scans'] == 0
                and result['peak_reported_compute_p99_ms'] <= 80
                and result['peak_reported_bridge_p99_ms'] <= 20
                and result['resources']['sim']['rss_peak'] <= 1024**3
                and len(result['exits']) == 5 and all(code == 0 for code in result['exits'].values())
                and manifest['exit_reason'] == 'shutdown'
                and manifest['bridge_source_sha256'] == bridge and len(manifest['robots']) == 4
                and all(r['core_source_sha256'] == core and r['exit_reason'] == 'shutdown'
                        for r in manifest['robots']))
    except (KeyError, TypeError):
        return False


def audit():
    checks = []
    def check(name, passed):
        checks.append({'check': name, 'passed': bool(passed)})
    core, bridge = source_hash(), bridge_hash()
    groups = {}
    for build in ('native', 'coverage'):
        for name, count in (('core', 113), ('ros', 66), ('launch-m1', 2), ('launch-m3', 2)):
            path = f'{build}/{name}.xml'
            groups[path] = suite(OUT/path)
            check(path + ' all executed and passed', suite_passed(groups[path], count))
    check('24 targeted ROS regressions', suite_passed(suite(OUT/'verified.xml'), 24))
    check('16 evidence gate negative/positive checks', suite_passed(suite(OUT/'report-gate.xml'), 16))
    historical = []
    for path in ('evidence/final/test-index.json', 'evidence/m2/test-index.json', 'evidence/m3/test-index.json'):
        data = read(resolve_reference(path))
        for case in data.get('cases', []):
            historical.append({**case, 'historical_index': path})
    specs = re.findall(r'^\| (T-[A-Z0-9]{3}-\d{2}) .+；M[123]；',
                       (ROOT/'docs/testing/测试用例与需求追踪.md').read_text(), re.M)
    check('all 140 specification IDs exactly once', len(historical) == len(specs) == 140
          and set(specs) == {c['id'] for c in historical})
    check('all 36 requirements mapped', {r for c in historical for r in c['requirements']} == {f'REQ-{i:03}' for i in range(1, 37)})
    deferred = {f'T-E2E-{i:02}' for i in range(1, 8)} | {'T-ROB-01'}
    check('exactly the eight user-deferred test IDs',
          {c['id'] for c in historical if c['status'] == 'DEFERRED'} == deferred)
    cases = []
    for old in historical:
        current = []
        for run in old['runs']:
            check(old['id'] + ' historical evidence exists: ' + run['evidence'], (resolve_reference(run['evidence'])).exists())
            selectors = run.get('tests', [run.get('selector', run.get('entry', ''))])
            for path in ('native/core.xml', 'native/ros.xml', 'native/launch-m1.xml', 'native/launch-m3.xml'):
                selected = [t for t in groups[path] if any(s and s in t['name'] for s in selectors)]
                if selected:
                    current.append({'evidence': 'evidence/completion-audit/' + path,
                                    'tests': [t['name'] for t in selected]})
        cases.append({'id': old['id'], 'title': old.get('title', old.get('name', '')),
                      'requirements': old['requirements'], 'status': old['status'],
                      'current_runs': current, 'historical_runs': old['runs'],
                      'historical_index': old['historical_index'], 'note': old.get('note', '')})
    check('132 prior passes and 8 deferrals, no silent pending items',
          Counter(c['status'] for c in cases) == {'PASS': 132, 'DEFERRED': 8})
    # Directly changed runtime must be present in both native and instrumented runs.
    for build in ('native', 'coverage'):
        manifests = list((OUT/build/'ros').glob('test_AUD01*/world-manifest.json'))
        check(build + ' final bridge/core fingerprint', bool(manifests) and all(
            read(p).get('bridge_source_sha256') == bridge and len(read(p).get('robots', [])) == 2 and
            all(r.get('core_source_sha256') == core for r in read(p).get('robots', []))
            for p in manifests))
    coverage = read(OUT/'coverage/coverage-gate.json')
    check('current coverage gate', coverage.get('status') == 'PASS')
    provenance = read(OUT/'coverage/profile-provenance.json')
    check('coverage sources match', provenance.get('core_source_sha256') == core
          and provenance.get('bridge_source_sha256') == bridge
          and provenance.get('historical_profiles_merged') is False)
    install = read(OUT/'install/install-check.json')
    check('fresh hidden-source install, M1/M3/PCL', install.get('status') == 'PASS'
          and install.get('source_hidden') and install.get('platform') == 'Darwin'
          and all(install.get(k) == 0 for k in ('build_exit', 'launch_exit', 'headless_exit',
                                               'mesh_exit', 'm3_launch_exit', 'adapter_exit')))
    gui = read(OUT/'gui/result.json')
    check('real RViz output independence', gui.get('status') == 'PASS'
          and gui.get('clouds_per_run') == 600 and gui.get('odom_per_run') == 3000)
    b3 = []
    for rtf in (0, 1):
        for repeat in range(5):
            folder = OUT/f'B3-performance/rtf-{rtf}-gui-0-{repeat}'
            result, manifest = read(folder/'result.json'), read(folder/'manifest.json')
            passed = b3_passed(result, manifest, core, bridge, rtf)
            check(f'B3 complete current window {rtf}/{repeat}', passed)
            b3.append({'directory': reference(folder), 'passed': passed,
                       **{k: v for k, v in result.items() if k != 'hashes'}})
    old_m3 = read(EVIDENCE_DIR / 'm3/test-index.json')
    check('unchanged core permits historical sanitizer evidence',
          old_m3.get('current_core_source_sha256') == core
          and suite_passed(suite(EVIDENCE_DIR / 'm3/asan-final.xml'), 113)
          and suite_passed(suite(EVIDENCE_DIR / 'm3/tsan-final.xml'), 113))
    # Historical B1/B2/load/start-stop are explicitly retained, not relabelled as new execution.
    b1 = read(EVIDENCE_DIR / 'm2/B1-performance/results.json')
    check('historical B1 has 20 complete passing windows', isinstance(b1, list) and len(b1) == 20
          and all(r.get('status') == 'PASS' and r.get('warmup_seconds') == 30
                  and r.get('duration_seconds') == 120 and r.get('wall_seconds', 0) >= 120 for r in b1))
    stress = read(EVIDENCE_DIR / 'm2/B2-stress/result.json')
    check('historical overload gate passed', stress.get('status') == 'PASS' and bool(stress.get('loads')))
    load = read(EVIDENCE_DIR / 'm2/load-times.json')
    check('historical cold/hot loading measured without invented threshold',
          load.get('status') == 'MEASURED' and len(load.get('runs', [])) == 2
          and all(r.get('load_and_initialization_seconds', 0) > 0 for r in load.get('runs', [])))
    starts = read(EVIDENCE_DIR / 'm2/start-stop-100/result.json')
    check('historical 100 start-stop cycles passed', starts.get('status') == 'PASS'
          and len(starts.get('cycles', [])) == 100
          and all(r.get('exits') and all(v == 0 for v in r['exits'].values()) for r in starts.get('cycles', [])))
    for document in [ROOT/'README.md', *sorted((ROOT/'docs').rglob('*.md'))]:
        for link in re.findall(r'\[[^\]]*\]\(([^)]+)\)', document.read_text()):
            if re.match(r'\w+://', link) or link.startswith(('#', 'mailto:')):
                continue
            local = link.split('#')[0].strip('<>')
            if local:
                check(f'local link {document.name}: {local}', (document.parent/local).exists())
    for path in python_sources():
        try:
            ast.parse(path.read_text()); valid = True
        except SyntaxError:
            valid = False
        check('Python syntax ' + reference(path), valid)
    for path in ROOT.glob('scripts/*.sh'):
        check('shell syntax ' + path.name, subprocess.run(['bash', '-n', str(path)], capture_output=True).returncode == 0)
    supplementary = {
        'T-COR-01': ['gui/result.json'], 'T-DEL-01': ['install/install-check.json'],
        'T-DEL-05': ['coverage/coverage-gate.json'], 'T-DEL-06': ['test-index.json'],
        'T-EXT-04': ['B3-performance/results.json'],
    }
    for case in cases:
        case['current_runs'] += [{'evidence': 'evidence/completion-audit/' + p}
                                for p in supplementary.get(case['id'], [])]
    failures = [c['check'] for c in checks if not c['passed']]
    return {'status': 'PASS_WITH_DEFERRED' if not failures else 'INCOMPLETE',
            'updated_at': datetime.now(timezone.utc).isoformat(), 'platform': 'macOS / ROS 2 Humble',
            'core_source_sha256': core, 'bridge_source_sha256': bridge,
            'counts': dict(Counter(c['status'] for c in cases)), 'cases': cases,
            'groups': {k: {'tests': len(v), 'passed': sum(t['passed'] for t in v)} for k, v in groups.items()},
            'coverage': coverage.get('totals', {}), 'B3': b3, 'checks': checks, 'failures': failures,
            'scope': {'linux': 'DEFERRED', 'eight_hour_soak': 'DEFERRED; not started',
                      'external_slam_navigation': 'DEFERRED; user decision from M2',
                      'historical_baselines': ['B1', 'B2', 'load times', '100 start-stop cycles', 'core ASan/UBSan/TSan']}}


def write_report(data):
    paths = {'native/core.xml': '内核', 'native/ros.xml': '完整真实 ROS 2',
             'native/launch-m1.xml': '单机 launch', 'native/launch-m3.xml': '多机 launch'}
    lines = ['# 文档对照与补全报告', '', f'本轮状态：**{data["status"]}**。验收平台为本机 macOS / ROS 2 Humble。', '',
             '原文档 36 项需求、140 条测试规格已统一对照；132 条保留有效通过记录，7 条外部算法闭环和 1 条 8 小时长稳按用户决定延期。Linux 平台另行验证。新增回归和本轮执行结果列在下方；没有将延期项或历史测试改写为本轮通过。', '',
             '## 本轮补全', '',
             '| 原缺口 | 完成内容 | 新增真实 ROS 回归 |', '| --- | --- | --- |',
             '| 多机只有 Twist | 支持带前缀 TwistStamped，拒绝空/跨机 frame、过期/未来/非法时间戳和 NaN；拒绝不刷新超时 | AUD01 |',
             '| 多机 QoS 不可配置、冲突未诊断 | 点云与 IMU 统一可切换 reliable，深度 5；按机器人计数和 WARN 诊断 | AUD02 |',
             '| 多机 TF 责任不可交接 | 逐机接管 odom TF；外部 TF 不改变仿真真值；重复源和错误父级故障退出 | AUD03–04 |',
             '| 多机启动参数与等待行为缺少异常验证 | 参数校验、可配置 RSP 名称和墙钟超时；清单记录 ROS/RMW、桥接指纹及一致退出原因 | AUD01、05–06 |',
             '| 必需子进程退出仍返回成功 | 单机/多机仿真器与 RSP 意外退出时，清理其他子进程并返回非零；完整 launch 正常 Ctrl+C 返回 0 | AUD07、09 |',
             '| 启动资源/参数交付不一致 | file URI、严格布尔值、安装后的多机接口参数透传 | AUD08–10 |',
             '| 总览和 CI 仍停在旧阶段 | 统一 140 条索引，补齐 Contact/Reset 入口；CI 改为 M1–M3 回归和独立安装，未运行远端 CI | 本报告静态门禁 |', '',
             '新增 24 个参数化用例见 [test_completion.py](../litesim3d_ros2/test/test_completion.py)。修复前 8 项失败的[原始记录](../evidence/completion-audit/before.xml)及首次修复中的失败均保留；最终[专项测试](../evidence/completion-audit/verified.xml)单列。', '',
             '## 本轮验证', '', '另有 16 个证据门禁正反例检查，验证空测试、跳过、失败、过期源码指纹及缩短性能窗口不会被判为通过；见 [report-gate.xml](../evidence/completion-audit/report-gate.xml)。', '', '| 检查 | 通过/执行 | 证据 |', '| --- | --- | --- |']
    for path, title in paths.items():
        g = data['groups'][path]
        lines.append(f'| {title} | {g["passed"]}/{g["tests"]} | [{path}](../evidence/completion-audit/{path}) |')
    cov = data['coverage']
    if cov:
        lines += ['', f'覆盖率构建另外执行全套内核、ROS 与两个 launch；整体行覆盖率 {cov["lines"]["percent"]:.2f}%，分支覆盖率 {cov["branches"]["percent"]:.2f}%。[覆盖率门禁](../evidence/completion-audit/coverage/coverage-gate.json)、[来源与源码指纹](../evidence/completion-audit/coverage/profile-provenance.json)。仅使用本次覆盖率构建产生的 profiles。']
    lines += ['', '[独立安装](../evidence/completion-audit/install/install-check.json) 从空目录构建，并在隐藏源码副本后测试单机/多机 launch 与 PCL 管线。',
              '[RViz 开关回归](../evidence/completion-audit/gui/result.json) 各运行 60 仿真秒，比较 600 帧点云、3000 条里程计和最终状态；点云 data 字节及消息字段相同，CDR 对齐间隙单列。未进行人工图形画面验收。', '',
              'RViz 首次在受限执行环境中因 `no screens available` 失败；保留 [失败记录](../evidence/completion-audit/gui-sandbox/result.json)。随后通过可访问 macOS 图形会话的入口重跑，最终结果单独保存。', '',
              '本轮完整 ROS 回归包含 100 次协调重置、60 仿真秒 IMU、30 秒暂停、消费端与进程故障；这些是有界功能测试。核心源码未变，沿用上一轮 113 项 ASan/UBSan、113 项 TSan 的原始证据。B1/B2、资源加载时间与 100 次启停记录保留为历史基线，未重新执行；本轮复测实际修改的多机桥接 B3。T-PER-05 的通过仅表示冷/热应用缓存测量记录齐全；未声明硬盘冷缓存或启动性能达标。', '',
              '## 本轮 B3 性能', '', '保持 [M3 冻结门槛](M3基线.md)：4 台机器人、F09、每台 16×512@10 Hz，总射线线程预算 4；每轮预热 30 秒并采样 120 秒，两种节拍各 5 轮。测试时不并行运行构建或其他测试。', '',
              '| 节拍/轮次 | 状态 | RTF | 最大计算/桥接 p99 ms | 采样峰值 RSS MiB | 接收间断 |',
              '| --- | --- | --- | --- | --- | --- |']
    for row in data['B3']:
        if 'rtf' in row:
            lines.append(f'| {row["rtf_target"]}/{row["directory"].split("-")[-1]} | {"PASS" if row["passed"] else "INCOMPLETE"} | {row["rtf"]:.3f} | {row["peak_reported_compute_p99_ms"]:.2f}/{row["peak_reported_bridge_p99_ms"]:.2f} | {row["resources"]["sim"]["rss_peak"]/1024**2:.1f} | {row["observed_delivery_gaps"]} |')
    lines += ['', '[B0 环境](../evidence/completion-audit/B3-performance/B0.json)；[逐轮结果](../evidence/completion-audit/B3-performance/results.json)。每轮保留采样、运行清单和退出码；RSS 为测量窗口每秒采样峰值。性能窗口不替代 8 小时长稳。', '',
              '## 剩余项', '', '| 项目 | 状态及边界 |', '| --- | --- |',
              '| T-E2E-01–07：真实建图/定位/导航算法闭环 | 按 M2 用户决定延期；PCL 点云消费不等同于这 7 项通过 |',
              '| T-ROB-01：8 小时墙钟稳定性 | 按用户要求暂不执行，本轮未启动；原门槛保留 |',
              '| Linux 干净环境、检测器与集成验收 | 按用户要求留作后续；本轮只声明 macOS / Humble |',
              '| 全向底盘、非重复扫描、多机完整重置、统一全局 RViz | 未启用的后续扩展，不列作已完成能力 |', '',
              '## 全量追踪', '', '[140 条统一索引](../evidence/completion-audit/test-index.json) 为每条规格保留需求编号、本轮测试名和历史证据路径。下表区分有本轮执行证据与仅保留历史基线，不用自动化实例数量替代规格数量。', '',
              '| 规格 | 状态 | 本轮证据 |', '| --- | --- | --- |']
    for c in data['cases']:
        execution = '；'.join(dict.fromkeys(f'[{Path(r["evidence"]).name}](../{r["evidence"]})' for r in c['current_runs']))
        lines.append(f'| {c["id"]} {c["title"]} | {c["status"]} | {execution or ("用户延期" if c["status"] == "DEFERRED" else "历史基线；详见统一索引")} |')
    lines += ['', f'核心指纹：`{data["core_source_sha256"]}`；桥接/launch 指纹：`{data["bridge_source_sha256"]}`。[运行源码和脚本指纹](../evidence/completion-audit/source-sha256.json)及[变更清单](../evidence/completion-audit/change-inventory.json)。', '',
              '复查命令：`bash scripts/test_m3.sh`；报告门禁：`python3 scripts/reports/report_completion.py`。后者只核对已有证据，不代替测试；缺失、跳过、失败、过期指纹或缩短性能窗口均不能关闭门禁。']
    if data['failures']:
        lines += ['', '尚未满足的本轮门禁：' + '；'.join(data['failures']) + '。']
    save_report(ROOT / "docs/testing/文档对照与补全报告.md", '\n'.join(lines) + '\n')


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    data = audit()
    (OUT/'test-index.json').write_text(json.dumps(data, ensure_ascii=False, indent=2))
    files = {reference(p): digest(p) for folder in SOURCE_FOLDERS
             for p in sorted((ROOT/folder).rglob('*')) if p.is_file() and '__pycache__' not in p.parts}
    (OUT/'source-sha256.json').write_text(json.dumps(files, ensure_ascii=False, indent=2))
    write_report(data)
    print(json.dumps({k: data[k] for k in ('status', 'counts', 'groups', 'failures')}, ensure_ascii=False))
    return int(bool(data['failures']))


if __name__ == '__main__':
    raise SystemExit(main())
