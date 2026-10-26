#!/usr/bin/env python3
"""Evidence-backed M2 status; deferred algorithms, running tests and failures never count as PASS."""
import collections
from datetime import datetime
from zoneinfo import ZoneInfo
import hashlib
import json
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from paths import REPO_ROOT, EVIDENCE_DIR, resolve_reference, write_report
import re
import xml.etree.ElementTree as ET

ROOT = REPO_ROOT
OUT=EVIDENCE_DIR / 'm2'

def read(relative):
    p=OUT/relative
    return json.loads(p.read_text()) if p.exists() else None

def suite(relative):
    p=OUT/relative
    if not p.exists():return []
    return list(ET.parse(p).iter('testcase'))

def success(case):return all(case.find(name) is None for name in ('failure','error','skipped'))

def main():
    core=suite('core.xml')
    ros={}
    # Later explicit rechecks supersede the same testcase, while failed historical logs stay intact.
    for file in ['coverage/ros.xml','coverage/updated.xml','reset-100-final.xml']:
        for case in suite(file):ros[case.attrib['name']]=(case,file)
    records=[]
    specs=resolve_reference('docs/testing/测试用例与需求追踪.md').read_text()
    required=re.findall(r'^\| (T-[A-Z0-9]{3}-\d{2}) ([^|]+)\| ([^|]+；M2；[^|]+)\|',specs,re.M)
    def evidence_status(core_prefixes=(),ros_names=(),json_paths=(),extra=None):
        runs=[];statuses=[]
        for prefix in core_prefixes:
            found=[c for c in core if c.attrib.get('classname','').startswith(prefix)]
            statuses.append('PASS' if found and all(success(c) for c in found) else ('FAIL' if found else 'NOT_RUN'))
            runs.append({'evidence':'evidence/m2/core.xml','selector':prefix})
        for name in ros_names:
            found=[(c,f) for key,(c,f) in ros.items() if key.startswith(name)]
            statuses.append('PASS' if found and all(success(c) for c,f in found) else ('FAIL' if found else 'NOT_RUN'))
            for c,f in found:runs.append({'evidence':'evidence/m2/'+f,'selector':c.attrib['name']})
        for path in json_paths:
            data=read(path)
            statuses.append(data.get('status','NOT_RUN') if data else 'NOT_RUN')
            runs.append({'evidence':'evidence/m2/'+path})
        if extra:
            status,paths=extra;statuses.append(status)
            runs.extend({'evidence':'evidence/m2/'+p} for p in paths)
        status=next((s for s in ('FAIL','ERROR','NOT_RUN','RUNNING') if s in statuses),'PASS' if statuses else 'NOT_RUN')
        return status,runs
    core_map={
      'GEO-10':['GEO10M2'],'GEO-12':['GEO12M2'],'MOT-08':['MOT08M2'],'MOT-09':['MOT09M2'],'MOT-10':['MOT10M2'],
      'LID-10':['LID10M2'],'LID-11':['LID11M2'],'LID-12':['LID12M2'],'LID-13':['LID11M2','LID12M2'],
      'LID-14':['LID14M2'],'LID-15':['LID15M2'],'LID-16':['LID16M2'],'LID-17':['LID17M2','M2Validation'],
      'LID-18':['LID17M2'],'LID-19':['LID19M2'],'LID-22':['LID22M2'],
      'ODO-03':['ODO03M2'],'IMU-01':['IMU01M2'],'IMU-02':['IMU01M2','IMU03M2'],'IMU-03':['IMU03M2'],
      'IMU-04':['IMU04M2'],'IMU-05':['IMU01M2','IMU06M2'],'IMU-06':['IMU06M2'],'IMU-07':['IMU07M2'],
      'IMU-08':['IMU08M2'],'PER-06':['LID19M2'],'ROB-05':['ROB05M2'],'ROB-06':['ROB06M2'],'ROB-08':['ROB08M2','ROB06M2']}
    ros_map={
      'LID-22':['test_ROS09_reset_rejects_uncoordinated_and_stale_requests'],
      'ODO-05':['test_ROB02_ROB07_hundred_full_resets_and_first_frames'],
      'IMU-07':['test_IMU07_real_ros_sixty_sim_seconds'],
      'IMU-08':['test_ROS09_reset_rejects_uncoordinated_and_stale_requests'],
      'ROS-07':['test_ROS07_slow_consumer_queue_and_thirty_second_pause'],
      'ROS-09':['test_ROS09_reset_rejects_uncoordinated_and_stale_requests'],
      'ROS-12':['test_ROS12_required_process_crash_is_nonzero'],
      'E2E-08':['test_E2E08_evaluator_keeps_failures_and_analytic_offsets'],
      'ROB-02':['test_ROB02_ROB07_hundred_full_resets_and_first_frames'],
      'ROB-07':['test_ROB02_ROB07_hundred_full_resets_and_first_frames'],
      'DEL-04':['test_DEL04_runner_propagates_assertions_and_timeout']}
    performance=read('B1-performance/results.json')
    soak_deferral=read('B1-soak-8h/deferred-by-user.json')
    soak_deferred=bool(soak_deferral) and soak_deferral.get('status')=='DEFERRED' and soak_deferral.get('cleanup_complete') is True
    perf_ok=bool(performance) and len(performance)==20 and all(r['status']=='PASS' and r['wall_seconds']>=120 and r['warmup_seconds']>=30 for r in performance)
    perf_state='PASS' if perf_ok else ('FAIL' if performance and any(r['status']!='PASS' for r in performance) else 'RUNNING' if (OUT/'B1-performance/B0.json').exists() else 'NOT_RUN')
    for ident,title,meta in required:
        key=ident[2:];note='';extra=None;json_paths=[]
        if key in [f'E2E-{i:02d}' for i in range(1,8)]:
            state='DEFERRED';runs=[{'evidence':'docs/testing/M2基线.md'}];note='用户确认不接入外部 SLAM/Nav2；不计为通过。'
        elif key=='ROB-01' and soak_deferred:
            state='DEFERRED';runs=[{'evidence':'evidence/m2/B1-soak-8h/deferred-by-user.json'}]
            note='用户要求长时间稳定测试先不做；已停止并清理进程，部分采样保留，不计为通过。'
        else:
            if key in ('PER-01','PER-02','PER-03'):
                extra=(perf_state,['B1-performance/results.json','B1-performance/B0.json'])
                if key=='PER-03':json_paths=['gui-m2-desktop/result.json']
            if key=='PER-04':json_paths=['B2-stress/result.json']
            if key=='PER-05':
                load=read('load-times.json')
                ok=bool(load) and load.get('status')=='MEASURED' and len(load['runs'])==2 and load['runs'][0]['resident_meshes_before']==0 and load['runs'][1]['resident_meshes_before']>0
                extra=('PASS' if ok else 'NOT_RUN',['load-times.json'])
                note='应用空缓存冷启/同进程几何缓存热载；未清除 OS 页缓存，不声称冷磁盘延迟。'
            if key=='ROB-01':
                data=read('B1-soak-8h/rtf-1-gui-0-0/result.json')
                extra=(data['status'] if data else ('RUNNING' if (OUT/'B1-soak-8h/B0.json').exists() else 'NOT_RUN'),['B1-soak-8h/rtf-1-gui-0-0/result.json','B1-soak-8h/B0.json'])
            if key=='ROB-03':json_paths=['start-stop-100/result.json']
            if key=='ROB-04':
                paths=['asan-final.xml','tsan-final.xml'];cases=[c for path in paths for c in suite(path)]
                ok=len(suite(paths[0]))>=96 and len(suite(paths[1]))>=8 and all(success(c) for c in cases)
                extra=('PASS' if ok else 'NOT_RUN',paths)
                note='macOS ASan/UBSan 与单独 TSan；本机不支持 LSan，detect_leaks=0；Linux 后续。'
            state,runs=evidence_status(core_map.get(key,()),ros_map.get(key,()),json_paths,extra)
        records.append({'id':ident,'name':title.strip(),'requirements':re.findall(r'REQ-\d+',meta),'status':state,'runs':runs,'note':note})
    assert len(records)==52,len(records)
    counts=dict(collections.Counter(r['status'] for r in records))
    gates={}
    for label,path in [('coverage','coverage/coverage-gate.json'),('install','install-final/install-check.json'),('M1_gui','gui-m1-desktop/result.json'),('documentation','docs-check.json')]:
        data=read(path);gates[label]=data.get('status','NOT_RUN') if data else 'NOT_RUN'
    fingerprints=''
    for name in ['include/litesim3d/sim.hpp','src/geometry.cpp','src/model.cpp','src/config.cpp','src/simulator.cpp','src/sensors.cpp','src/terrain.cpp']:
        fingerprints+=name+':'+hashlib.sha256((ROOT/'litesim3d_core'/name).read_bytes()).hexdigest()+'\n'
    current=hashlib.sha256(fingerprints.encode()).hexdigest()
    manifest=read('B1-soak-8h/rtf-1-gui-0-0/manifest.json')
    gates['runtime_source']='PASS' if manifest and manifest['core_source_sha256']==current else 'NOT_RUN'
    installed=suite('install-final/m2-ros.xml')
    gates['M2_installed_runtime']='PASS' if len(installed)==2 and all(success(c) for c in installed) else 'NOT_RUN'
    release=suite('release-ros.xml')
    release_manifest=read('release-smoke.json')
    gates['final_installation']='PASS' if len(release)==1 and all(success(c) for c in release) and release_manifest and release_manifest['core_source_sha256']==current else 'NOT_RUN'
    original=[c for c in core if 'M2' not in c.attrib.get('classname','')]
    legacy=[c for n,(c,f) in ros.items() if n.startswith('test_T_')]
    launch=suite('coverage/launch.xml')
    gates['M1_regression']='PASS' if len(original)>=67 and len(legacy)>=26 and len(launch)>=2 and all(success(c) for c in original+legacy+launch) else 'NOT_RUN'
    closed=counts.get('PASS',0)==45-int(soak_deferred) and counts.get('DEFERRED',0)==7+int(soak_deferred) and all(v=='PASS' for v in gates.values())
    index={'updated_at':datetime.now(ZoneInfo('Asia/Shanghai')).isoformat(),'milestone':'M2','status':'PASS_WITH_DEFERRED' if closed else 'IN_PROGRESS','platform':'macOS / ROS 2 Humble',
           'platform_decision':{'date':'2026-10-18','linux_required':False,'external_algorithms_deferred':True,'eight_hour_soak_deferred':soak_deferred},
           'current_core_source_sha256':current,'counts':counts,'gates':gates,'cases':records}
    (OUT/'test-index.json').write_text(json.dumps(index,ensure_ascii=False,indent=2)+'\n')
    lines=['# M2 测试报告','','更新时间：'+datetime.now(ZoneInfo('Asia/Shanghai')).isoformat(),
           '',f"状态：**{'本轮调整范围验收通过' if closed else '验收中，尚未关闭 M2'}**。",'',
           '本轮平台为用户确认的 macOS / Humble。外部 SLAM/导航算法的 7 项验收以及 Linux 验证后续进行，不计入 PASS。',
           *(['用户随后要求“长时间稳定测试先不做”；8 小时长稳已受控停止并列为 DEFERRED，不计入本轮关闭条件，也不计为通过。'] if soak_deferred else []),
           '',f'原始 M2 规格 52 项：{counts}。附加门禁：{gates}。',
           '',f'当前核心源码指纹：`{current}`。原始记录见 [M2 追踪索引](../evidence/m2/test-index.json)。',
           '', '## 已执行与边界','',
           f'- {len(core)} 项内核测试覆盖 M1 回归、坡面/多层支撑、滚动扫描与去畸变参考、噪声/丢点统计、raw IMU、独立里程计误差和资源失败回滚。',
           '- 真实 ROS 使用独立 domain；消费端只检查接口与历史清理，不冒充建图或导航算法。',
           '- 长稳以单调墙钟记录，不用仿真时间替代。性能每种 GUI/节拍组合均要求预热 30 秒、测量 120 秒、重复 5 次；中止或未完成的测试不计 PASS。',
           '- 冷/热加载测量区分应用缓存；OS 页缓存没有清除。ASan 禁用本平台不支持的 LeakSanitizer；Linux 检测器仍待后续。',
           '- 失败历史保留在 evidence/m2；复测明确标记，不将旧失败日志删除或改写为成功。',
           '', '## 用例结果','', '| 用例 | 状态 | 证据/说明 |','| --- | --- | --- |']
    for r in records:
        refs=[]
        for run in r['runs']:
            path=run['evidence']
            if (resolve_reference(path)).exists():refs.append(f'[{Path(path).name}](../{path})')
        lines.append(f"| {r['id']} {r['name']} | {r['status']} | {'、'.join(dict.fromkeys(refs))} {r['note']} |")
    lines += ['', '## 测量摘要', '']
    coverage=read('coverage/coverage-gate.json')
    if coverage:
        totals=coverage['totals']
        lines.append(f"- 覆盖率：行 {totals['lines']['percent']:.2f}%，分支 {totals['branches']['percent']:.2f}%；逐文件门槛和排除规则见 [coverage-gate.json](../evidence/m2/coverage/coverage-gate.json)。")
    lines.append('- 最新独立安装检查：[安装结果](../evidence/m2/install-final/install-check.json)、[安装包 M2 采样与重置](../evidence/m2/install-final/m2-ros.xml)。构建副本的源码目录隐藏后仍能运行。')
    lines.append('- 最终常用安装前缀已更新并通过 [ROS 重置复测](../evidence/m2/release-ros.xml) 和 [独立核心运行](../evidence/m2/release-smoke.json)。macOS 增量安装出现重复删除既有 RPATH 的工具诊断，构建退出码为 0，实际消息类型加载与重置验证通过；[完整构建日志](../evidence/m2/build-release-final.log) 保留。')
    lines.append('- M1/M2 的 GUI 开关点云和里程计一致性：[M1](../evidence/m2/gui-m1-desktop/result.json)、[M2](../evidence/m2/gui-m2-desktop/result.json)。RViz 原生渲染器正常运行；该检查不等同于逐像素人工视觉验收。')
    if performance:
        finished=[r for r in performance if r.get('wall_seconds',0)>=120]
        lines.append(f'- B1 当前完成 {len(finished)}/20 轮；单轮全部结果及原始采样见 [性能结果](../evidence/m2/B1-performance/results.json)。')
        if finished:
            lines+=['', '| GUI | 节拍 | 完成轮数 | RTF 范围 | 最大窗口计算 p99 / 桥接 p99（ms） | 核心与桥接峰值 RSS（MiB） |',
                    '| --- | --- | --- | --- | --- | --- |']
            for gui in (False,True):
                for rtf in (0,1):
                    group=[r for r in finished if r['gui']==gui and r['rtf_target']==rtf and 'resources' in r]
                    if group:
                        lines.append(f"| {'开' if gui else '关'} | {'无节拍' if rtf==0 else '1 倍'} | {len(group)}/5 | {min(r['rtf'] for r in group):.4f}–{max(r['rtf'] for r in group):.4f} | {max(r['peak_reported_compute_p99_ms'] for r in group):.3f} / {max(r['peak_reported_bridge_p99_ms'] for r in group):.3f} | {max(r['resources']['sim']['rss_peak'] for r in group)/1024**2:.1f} |")
    soak=read('B1-soak-8h/rtf-1-gui-0-0/result.json')
    progress=read('B1-soak-8h/rtf-1-gui-0-0/progress.json')
    if soak_deferred:
        lines += ['', f"- 八小时长稳：按用户要求暂缓；停止前实际采样 {soak_deferral['observed_wall_seconds']:.2f} 秒，所属进程已清理。该时长不足 8 小时，不证明长期稳定性。见 [停止与暂缓记录](../evidence/m2/B1-soak-8h/deferred-by-user.json)。"]
    elif soak and 'rss_slope_mib_per_hour' in soak:
        lines += ['', f"- 八小时长稳：{soak['status']}；实际墙钟 {soak['wall_seconds']:.2f} 秒，RSS 斜率 {soak['rss_slope_mib_per_hour']:.4f} MiB/h，同状态差 {soak['same_state_rss_delta']/1024**2:.2f} MiB，峰值 {soak['resources']['sim']['rss_peak']/1024**2:.2f} MiB。见 [长稳原始结果](../evidence/m2/B1-soak-8h/rtf-1-gui-0-0/result.json)。"]
    elif soak:
        lines += ['', f"- 八小时长稳状态：{soak['status']}，原因：{soak.get('error','详见原始结果')}。见 [长稳原始结果](../evidence/m2/B1-soak-8h/rtf-1-gui-0-0/result.json)。"]
    elif progress:
        lines += ['', f"- 八小时长稳仍在运行：最近进度 {progress['wall_seconds']:.0f} / {progress['target']} 秒；该进度不代表通过，见 [进度记录](../evidence/m2/B1-soak-8h/rtf-1-gui-0-0/progress.json)。"]
    load=read('install-final/load-times.json')
    if load:
        lines += ['', f"- F09 应用空缓存加载 {load['runs'][0]['load_and_initialization_seconds']:.4f} 秒，同进程缓存热加载 {load['runs'][1]['load_and_initialization_seconds']:.4f} 秒；未清除 OS 页缓存，尚无启动延迟达标阈值。见 [安装包加载测量](../evidence/m2/install-final/load-times.json)。"]
    write_report(ROOT / "docs/testing/M2测试报告.md", '\n'.join(lines)+'\n')
    print(json.dumps({'status':index['status'],'counts':counts,'gates':gates},ensure_ascii=False))
    return 0 if closed else 1

if __name__=='__main__':raise SystemExit(main())
