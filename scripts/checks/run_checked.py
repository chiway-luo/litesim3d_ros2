#!/usr/bin/env python3
"""Run a required check; assertion failures, signals and wall timeouts all produce failing JUnit."""
import argparse
import os
import signal
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import subprocess
import time
import xml.etree.ElementTree as ET


def run(command,timeout,xml,log):
    start=time.monotonic();status=1;message=''
    with log.open('w') as out:
        try:
            process=subprocess.Popen(command,stdout=out,stderr=subprocess.STDOUT,start_new_session=True)
            status=process.wait(timeout=timeout)
            if status:message=f'command exited {status}'
        except subprocess.TimeoutExpired:
            status=124;message=f'wall timeout after {timeout} seconds'
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGKILL);process.wait()
        except OSError as error:
            status=127;message=str(error)
    suite=ET.Element('testsuite',name='required_command',tests='1',failures=str(int(status!=0)))
    case=ET.SubElement(suite,'testcase',name=' '.join(command),time=str(time.monotonic()-start))
    if status:ET.SubElement(case,'failure',message=message).text=log.read_text()[-6000:]
    ET.ElementTree(suite).write(xml,encoding='utf-8',xml_declaration=True)
    return status if status>=0 else 1


def main():
    p=argparse.ArgumentParser();p.add_argument('--timeout',type=float,required=True)
    p.add_argument('--xml',type=Path,required=True);p.add_argument('--log',type=Path,required=True)
    p.add_argument('command',nargs=argparse.REMAINDER);a=p.parse_args()
    command=a.command[1:] if a.command and a.command[0]=='--' else a.command
    return run(command,a.timeout,a.xml,a.log)

if __name__=='__main__':raise SystemExit(main())
