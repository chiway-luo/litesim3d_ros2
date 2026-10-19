#!/usr/bin/env python3
"""B2 overload characterization. Slowdown is an expected outcome, data/time loss is not."""
import argparse
import json
import os
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import xml.etree.ElementTree as ET
import yaml
from benchmarks.benchmark_m2 import measure,environment


def main():
    p=argparse.ArgumentParser();p.add_argument('--config',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();a.config=a.config.resolve();a.output=a.output.resolve();a.output.mkdir(parents=True,exist_ok=True)
    os.environ['ROS_DOMAIN_ID']='6';os.environ['ROS_LOCALHOST_ONLY']='1';os.environ['ROS_LOG_DIR']=str(a.output/'ros_logs')
    (a.output/'B0.json').write_text(json.dumps(environment(),indent=2))
    records=[];error=None
    try:
        for columns in (2048,4096,8192):
            data=yaml.safe_load(a.config.read_text());data['lidars'][0]['columns']=columns
            cfg=a.output/f'columns-{columns}.yaml';cfg.write_text(yaml.safe_dump(data))
            result=measure(cfg,a.output/f'columns-{columns}',0,False,5,20)
            records.append(result)
            assert result['status']!='ERROR',result
            assert result['wall_seconds']>=20 and result['simulated_seconds']>0
            assert result['internal_missed_scans']==0
            # Count comes from the independent event scheduler, never wall-derived fake stamps.
            assert abs(result['generated_frames']-result['simulated_seconds']*20)<=1.1
            assert all(value==0 for value in result['exits'].values())
            assert result['resources']['sim']['rss_peak']<2*1024**3
        assert any(row['rtf']<1 for row in records),'extend workload until actual overload is observed'
    except Exception as exc:error=repr(exc)
    report={'status':'FAIL' if error else 'PASS','error':error,'loads':records,
            'note':'Per-load B1 latency/RTF gate status is diagnostic; B2 requires honest slowdown and bounded queues.'}
    (a.output/'result.json').write_text(json.dumps(report,indent=2))
    suite=ET.Element('testsuite',name='T_PER_04',tests='1',failures=str(int(error is not None)))
    case=ET.SubElement(suite,'testcase',name='B2_overload_curve')
    if error:ET.SubElement(case,'failure',message=error[:4000])
    ET.ElementTree(suite).write(a.output/'result.xml',encoding='utf-8',xml_declaration=True)
    return int(error is not None)

if __name__=='__main__':raise SystemExit(main())
