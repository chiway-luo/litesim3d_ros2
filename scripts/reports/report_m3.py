#!/usr/bin/env python3
"""M3 evidence gate: never equate an implemented test or a short exploration with acceptance."""
import ast
import hashlib
import json
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from paths import REPO_ROOT, EVIDENCE_DIR, SOURCE_FOLDERS, reference, python_sources, write_report
import re
import subprocess
import xml.etree.ElementTree as ET

ROOT = REPO_ROOT
OUT=EVIDENCE_DIR / 'm3'


def read(path):
    file=OUT/path
    return json.loads(file.read_text()) if file.exists() else {}


def suite(path):
    file=OUT/path
    if not file.exists():return []
    return [{'name':c.get('classname','')+'.'+c.get('name',''),
             'passed':c.find('failure') is None and c.find('error') is None and c.find('skipped') is None}
            for c in ET.parse(file).getroot().iter('testcase')]


def source_hash():
    names=['include/litesim3d/sim.hpp',*[f'src/{name}.cpp' for name in
           ['geometry','model','config','simulator','sensors','terrain','world']]]
    data=''.join(f'{n}:{hashlib.sha256((ROOT/"litesim3d_core"/n).read_bytes()).hexdigest()}\n' for n in names)
    return hashlib.sha256(data.encode()).hexdigest()


def main():
    groups={name:suite(path) for name,path in {
        'core':'core-final.xml','ros_all':'coverage-final/ros.xml','ros_m3_native':'ros-verified.xml',
        'asan_ubsan':'asan-final.xml','tsan':'tsan-final.xml',
        'launch_m1':'coverage-final/launch-m1.xml','launch_m3':'coverage-final/launch-m3.xml'}.items()}
    checks=[]
    def check(name,passed):checks.append({'check':name,'passed':bool(passed)})
    for name,tests in groups.items():check(name+' executed and passed',tests and all(t['passed'] for t in tests))
    check('full M1/M2/M3 core count',len(groups['core'])==113)
    check('full ROS count',len(groups['ros_all'])==42)
    check('M3 native ROS count',len(groups['ros_m3_native'])==9)
    coverage=read('coverage-final/coverage-gate.json');check('coverage gates',coverage.get('status')=='PASS')
    install=read('install-final/install-check.json')
    check('fresh hidden-source install',install.get('status')=='PASS' and install.get('source_hidden') and install.get('m3_launch_exit')==0 and install.get('adapter_exit')==0)
    gui=read('gui-regression/result.json');check('real RViz output independence',gui.get('status')=='PASS')
    expected=source_hash();rows=[]
    for rtf in (0,1):
        for repeat in range(5):
            directory=f'B3-performance/rtf-{rtf}-gui-0-{repeat}'
            result=read(directory+'/result.json');manifest=read(directory+'/manifest.json')
            good=(result.get('status')=='PASS' and result.get('warmup_seconds')==30 and
                  result.get('duration_seconds')==120 and result.get('wall_seconds',0)>=120 and
                  len(manifest.get('robots',[]))==4 and
                  all(r.get('core_source_sha256')==expected for r in manifest.get('robots',[])))
            check('B3 '+directory,good)
            rows.append({'directory':directory,'passed':good,**result})
    titles=['动态实体逐点时刻','机器人与动态实体相对扫掠','2/4 机器人隔离与互相感知',
            'B3 性能、共享资源与随机流隔离','Ackermann 解析与里程计反馈闭环','Ouster 字段与实际 PCL 消费']
    requirements=[['REQ-033'],['REQ-014','REQ-033'],['REQ-024','REQ-034'],['REQ-004','REQ-034'],['REQ-035'],['REQ-019','REQ-026','REQ-035']]
    cases=[]
    for i,title in enumerate(titles,1):
        core=[t for t in groups['core'] if f'EXT{i:02d}M3.' in t['name']]
        ros=[t for t in groups['ros_m3_native'] if f'test_EXT{i:02d}_' in t['name']]
        valid=bool(core or ros) and all(t['passed'] for t in core+ros)
        runs=[]
        if core:runs.append({'evidence':'evidence/m3/core-final.xml','tests':[t['name'] for t in core]})
        if ros:runs.append({'evidence':'evidence/m3/ros-verified.xml','tests':[t['name'] for t in ros]})
        if i==4:
            valid=valid and all(r['passed'] for r in rows)
            runs.append({'evidence':'evidence/m3/B3-performance/results.json','tests':['2 modes × 5 complete wall-clock windows']})
        cases.append({'id':f'T-EXT-{i:02d}','title':title,'requirements':requirements[i-1],
                      'status':'PASS' if valid else 'INCOMPLETE','runs':runs})
    for document in [ROOT/'README.md',*sorted((ROOT/'docs').rglob('*.md'))]:
        for link in re.findall(r'\[[^\]]*\]\(([^)]+)\)',document.read_text()):
            if re.match(r'\w+://',link) or link.startswith(('#','mailto:')):continue
            local=link.split('#')[0].strip('<>')
            if local:check(f'local link {document.name}: {local}',(document.parent/local).exists())
    for source in python_sources():
        try:ast.parse(source.read_text());ok=True
        except SyntaxError:ok=False
        check('Python syntax '+reference(source),ok)
    for script in ROOT.glob('scripts/*.sh'):
        check('shell syntax '+script.name,subprocess.run(['bash','-n',str(script)],capture_output=True).returncode==0)
    failures=[c['check'] for c in checks if not c['passed']]
    status='PASS' if not failures and all(c['status']=='PASS' for c in cases) else 'INCOMPLETE'
    index={'status':status,'platform':'macOS / ROS 2 Humble','current_core_source_sha256':expected,
           'scope':{'linux':'DEFERRED','eight_hour_soak':'DEFERRED by user; not run',
                    'external_slam_nav2':'DEFERRED from M2; PCL VoxelGrid is the tested adapter consumer'},
           'cases':cases,'groups':{k:{'tests':len(v),'passed':sum(t['passed'] for t in v)} for k,v in groups.items()},
           'checks':checks,'failures':failures}
    (OUT/'test-index.json').write_text(json.dumps(index,ensure_ascii=False,indent=2))
    artifacts={reference(p):hashlib.sha256(p.read_bytes()).hexdigest()
               for folder in SOURCE_FOLDERS for p in sorted((ROOT/folder).rglob('*'))
               if p.is_file() and '__pycache__' not in p.parts and p.suffix not in ('.pyc','.profraw')}
    (OUT/'source-sha256.json').write_text(json.dumps(artifacts,ensure_ascii=False,indent=2))
    lines=['# M3 测试报告','',f'本轮状态：**{status}**。验收平台为本机 macOS / Humble；Linux 和 8 小时长稳按用户要求延期。',
           '','范围及 B3 门槛在正式测试前冻结，见 [M3 基线](M3基线.md)；启动和接口见 [多机器人](../usage/多机器人.md) 与 [接口文档](../api/README.md)。',
           '','## 结果','', '| 用例 | 状态 | 可复查入口与证据 |','| --- | --- | --- |']
    for case in cases:
        evidence='；'.join(f'[{Path(r["evidence"]).name}](../{r["evidence"]})' for r in case['runs'])
        lines.append(f'| {case["id"]} {case["title"]} | {case["status"]} | {evidence} |')
    lines+=['','完整需求/可执行测试名映射见 [test-index.json](../evidence/m3/test-index.json)。规格 ID 数量与自动化测试实例数分别统计。',
            '','## 功能回归与检测器','']
    evidence_paths={'core':'core-final.xml','ros_all':'coverage-final/ros.xml','ros_m3_native':'ros-verified.xml',
                    'asan_ubsan':'asan-final.xml','tsan':'tsan-final.xml','launch_m1':'coverage-final/launch-m1.xml','launch_m3':'coverage-final/launch-m3.xml'}
    for name,tests in groups.items():
        lines.append(f'- {name}：{sum(t["passed"] for t in tests)}/{len(tests)}，[{evidence_paths[name]}](../evidence/m3/{evidence_paths[name]})。')
    lines+=['','113 项内核测试包含 M1 的 67 项、M2 的 30 项及 M3 的 16 项。全量 ROS 回归包含 100 次跨进程完整重置、60 仿真秒 IMU、故障退出、QoS、TF、暂停及单步；不将这些短时测试视为长期稳定证明。',
            '','[独立安装](../evidence/m3/install-final/install-check.json) 从全新临时目录构建，隐藏源码副本后运行 M1/M3 启动和 PCL 管线。',
            '[RViz 回归](../evidence/m3/gui-regression/result.json) 各执行 60 仿真秒，比较 600 帧点云、3000 条里程计及最终状态；字段和点云 data 字节一致，CDR 对齐间隙单列。未进行额外截图人工验收。',
            '','## 覆盖率','', '[覆盖率门禁与排除规则](../evidence/m3/coverage-final/coverage-gate.json) 使用真实 LLVM/Python 执行数据，包含新增 world、ROS 节点和适配器；第三方、生成消息、测试及构建报告脚本不计入运行逻辑。全部 profile 均来自最终构建的本轮执行，没有合并历史 profile；[来源与指纹](../evidence/m3/coverage-final/profile-provenance.json)。']
    if coverage:
        lines+=['',f'整体行覆盖率 {coverage["totals"]["lines"]["percent"]:.2f}%，分支覆盖率 {coverage["totals"]["branches"]["percent"]:.2f}%；变换、几何、时间、运动、传感器及共享世界模块分别要求行 ≥90%、分支 ≥80%；配置、桥接等逻辑计入整体 ≥80%/≥70% 门禁。']
    lines+=['','## 正式 B3 性能','', '同一 F09 场景、4×16×512@10Hz、总射线线程预算 4；每轮预热 30 秒，随后测量 120 秒，各节拍 5 轮。下表使用每个窗口报告的最大滑动 p99，桥接计时从点云计算完成到 DDS publish 返回，采用逐台完成即发布。',
            '','| 节拍/轮次 | 状态 | RTF | 最大计算 / 桥接 p99 (ms) | 窗口采样峰值 RSS (MiB) | 接收端间断 |',
            '| --- | --- | --- | --- | --- | --- |']
    for r in rows:
        if 'rtf' in r:
            lines.append(f'| {r["rtf_target"]}/{r["directory"].split("-")[-1]} | {r["status"]} | {r["rtf"]:.3f} | {r["peak_reported_compute_p99_ms"]:.2f} / {r["peak_reported_bridge_p99_ms"]:.2f} | {r["resources"]["sim"]["rss_peak"]/1024**2:.1f} | {r["observed_delivery_gaps"]} |')
    lines+=['','[逐轮结果](../evidence/m3/B3-performance/results.json)；[B0 环境](../evidence/m3/B3-performance/B0.json)；每轮目录保留 samples.jsonl、manifest 和退出码。CPU 按单核 100% 统计，RSP 单独统计；RSS 是测量窗口内每秒采样的峰值，不声明启动瞬间或整个进程寿命的绝对峰值。监控器逐台核对时间、10 Hz 预期生成数及唯一时钟；接收端间断与内部漏帧分开，未将 QoS 丢包当成少生成点云。',
            '','首次启动只报告当时的系统缓存条件，不声明硬盘冷缓存测试。性能窗口不替代 8 小时长稳。',
            '','## 修复与边界','',
            '- 首轮 B3 桥接 p99 超过 20 ms：四机状态先统一提交，再按机器人完成扫描即发布，消除了等待其余三台扫描的额外排队；测量口径与门槛保持不变。原始失败记录保留在 `evidence/m3/before-streaming/B3-performance/`。',
            '- Ackermann 零速反向步不允许残余偏航；新增解析回归。动态停止状态仅在同一个世界中共享，不污染独立实例。',
            '- 无节拍节点收到 SIGINT 时，ROS 上下文失效不再被当成模型故障；保留首次短测失败及后续退出测试。超高速退出观察器只订阅所需点云和诊断，避免 Python TF/时钟/4 路 IMU 处理饱和遮蔽退出行为；正常功能仍验证所有通道。',
            '- Ouster original 适配使用 PCL 1.15.1 真实读取与 VoxelGrid，含独立质心参考、无回波和非法字节布局。强度、reflectivity 与 ambient 是明确声明的合成字段；未声称真实雷达标定或 SLAM/Nav2 兼容。',
            '- 多机限定同构 1–4 台、Twist、整体暂停/单步和 URDF collision 互相感知；全向底盘、非重复扫描、多机协调 reset 及真实动力学不在本轮声明范围。',
            '- Linux、8 小时稳定性及 M2 留待后续的外部建图/定位/导航算法未计 PASS。CI 依赖已补充，但本轮没有远端 CI 执行证据。',
            '',f'当前核心源码指纹：`{expected}`。完整文件哈希见 [source-sha256.json](../evidence/m3/source-sha256.json)。中间失败日志保留，最终门禁只采纳上面列明的完成记录。']
    if failures:lines+=['','尚未通过的门禁：'+ '；'.join(failures)+'。']
    write_report(ROOT / "docs/testing/M3测试报告.md", '\n'.join(lines)+'\n')
    print(json.dumps({'status':status,'failures':failures},ensure_ascii=False))
    return int(bool(failures))


if __name__=='__main__':raise SystemExit(main())
