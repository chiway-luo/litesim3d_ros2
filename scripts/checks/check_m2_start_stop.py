#!/usr/bin/env python3
"""100 installed ROS process start/stop cycles; per-cycle exit and ownership evidence."""
import argparse
import json
import os
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import time
import xml.etree.ElementTree as ET
import psutil
from benchmarks.benchmark_m2 import Run


def main():
    p=argparse.ArgumentParser();p.add_argument('--config',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--cycles',type=int,default=100);a=p.parse_args()
    a.output=a.output.resolve();a.output.mkdir(parents=True,exist_ok=True)
    os.environ['ROS_DOMAIN_ID']='4';os.environ['ROS_LOCALHOST_ONLY']='1';os.environ['ROS_LOG_DIR']=str(a.output/'ros_logs')
    self=psutil.Process();records=[];error=None
    try:
        for cycle in range(a.cycles):
            run=Run.__new__(Run)
            try:
                run.__init__(a.config.resolve(),a.output/f'cycle-{cycle:03d}',1,False)
                run.until(lambda:run.clouds>=1,10)
                pids=[p.pid for _,p in run.processes]
                alive={name:{'pid':p.pid,'fds':psutil.Process(p.pid).num_fds()} for name,p in run.processes}
            finally:exits=run.close()
            assert all(value==0 for value in exits.values()),exits
            assert not any(psutil.pid_exists(pid) for pid in pids),pids
            row={'cycle':cycle,'exits':exits,'processes':alive,'observer_fds':self.num_fds(),'observer_rss':self.memory_info().rss}
            records.append(row)
            (a.output/'progress.json').write_text(json.dumps({'completed':cycle+1,'required':a.cycles,'status':'RUNNING'}))
        assert a.cycles>=100,'shortened execution is not acceptance'
        assert records[-1]['observer_fds']-records[5]['observer_fds']<=4
        assert records[-1]['observer_rss']-records[5]['observer_rss']<=32*1024**2
    except Exception as exc:error=repr(exc)
    (a.output/'result.json').write_text(json.dumps({'status':'FAIL' if error else 'PASS','error':error,'cycles':records},indent=2))
    suite=ET.Element('testsuite',name='T_ROB_03',tests='1',failures=str(int(error is not None)))
    case=ET.SubElement(suite,'testcase',name='100_installed_start_stop')
    if error:ET.SubElement(case,'failure',message=error)
    ET.ElementTree(suite).write(a.output/'result.xml',encoding='utf-8',xml_declaration=True)
    return int(error is not None)

if __name__=='__main__':raise SystemExit(main())
