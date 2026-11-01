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


class Interrupted(BaseException):
    def __init__(self, signum):
        self.signum = signum


def xml_text(text):
    """Replace XML 1.0-invalid characters in the report, never the raw log."""
    return ''.join(character if (character in '\t\n\r'
                                 or 0x20 <= ord(character) <= 0xD7FF
                                 or 0xE000 <= ord(character) <= 0xFFFD
                                 or 0x10000 <= ord(character) <= 0x10FFFF)
                   else '\ufffd' for character in text)


def process_tree(pid):
    """Snapshot owned Linux descendants, including children that called setsid."""
    records = {}
    for path in Path('/proc').glob('[0-9]*/stat'):
        try:
            fields = path.read_text().rsplit(')', 1)[1].split()
            records[int(path.parent.name)] = (int(fields[1]), fields[19], fields[0])
        except (OSError, ValueError, IndexError):
            continue
    owned = {pid}
    while True:
        extended = owned | {child for child, (parent, _, _) in records.items() if parent in owned}
        if extended == owned:
            return {child: records[child][1] for child in owned if child in records}
        owned = extended


def stop_tree(process):
    owned = process_tree(process.pid)

    def alive(pid, identity):
        try:
            fields = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
            return fields[19] == identity and fields[0] != 'Z'
        except (OSError, IndexError):
            return False

    if not owned:  # POSIX fallback on platforms without /proc.
        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
    else:
        for pid, identity in owned.items():
            if alive(pid, identity):
                try:
                    os.kill(pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        process.poll()  # reap our child, not a process belonging to another runner
        if process.returncode is not None and not any(alive(pid, identity) for pid, identity in owned.items()):
            return
        time.sleep(.02)
    for pid, identity in owned.items():
        if alive(pid, identity):
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
    if not owned:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    process.wait()


def run(command,timeout,xml,log):
    start=time.monotonic();status=1;message=''
    process = None
    interruption = None
    handlers = {s: signal.getsignal(s) for s in (signal.SIGINT, signal.SIGTERM)}
    def interrupted(signum, frame):
        raise Interrupted(signum)
    for signum in handlers:
        signal.signal(signum, interrupted)
    with log.open('w') as out:
        try:
            process=subprocess.Popen(command,stdout=out,stderr=subprocess.STDOUT,start_new_session=True)
            status=process.wait(timeout=timeout)
            if status:message=f'command exited {status}'
        except subprocess.TimeoutExpired:
            status=124;message=f'wall timeout after {timeout} seconds'
        except Interrupted as error:
            status=128+error.signum;message=f'interrupted by signal {error.signum}'
            interruption = error
        except OSError as error:
            status=127;message=str(error)
        finally:
            # An outer runner can interrupt this wrapper while its own child
            # has a separate session. Forward cancellation and reap it too.
            for signum in handlers:
                signal.signal(signum, signal.SIG_IGN)
            try:
                if process is not None and status in (124, 128+signal.SIGINT, 128+signal.SIGTERM):
                    stop_tree(process)
            finally:
                for signum, handler in handlers.items():
                    signal.signal(signum, handler)
    suite=ET.Element('testsuite',name='required_command',tests='1',failures=str(int(status!=0)))
    case=ET.SubElement(suite,'testcase',name=xml_text(' '.join(command)),time=str(time.monotonic()-start))
    if status:
        detail=log.read_text(errors='replace')[-6000:]
        ET.SubElement(case,'failure',message=xml_text(message)).text=xml_text(detail)
        # Also expose the failure in the CI step log, before artifact upload.
        print(f'{message}; log: {log}\n{detail}',file=sys.stderr,flush=True)
    ET.ElementTree(suite).write(xml,encoding='utf-8',xml_declaration=True)
    if interruption is not None:
        raise interruption  # An embedding CI runner must stop, not start its next stage.
    return status if status>=0 else 1


def main():
    p=argparse.ArgumentParser();p.add_argument('--timeout',type=float,required=True)
    p.add_argument('--xml',type=Path,required=True);p.add_argument('--log',type=Path,required=True)
    p.add_argument('command',nargs=argparse.REMAINDER);a=p.parse_args()
    command=a.command[1:] if a.command and a.command[0]=='--' else a.command
    try:
        return run(command,a.timeout,a.xml,a.log)
    except Interrupted as error:
        return 128 + error.signum

if __name__=='__main__':raise SystemExit(main())
