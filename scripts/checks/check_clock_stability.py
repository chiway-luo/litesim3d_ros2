#!/usr/bin/env python3
"""Detect system-clock steps that can invalidate DDS source-timestamp ordering."""
import argparse
import json
import math
from pathlib import Path
import time


def check_clock(duration, threshold_ms):
    started = time.monotonic()
    previous = time.time_ns() - time.monotonic_ns()
    largest_step = 0
    jumps = []
    while time.monotonic() - started < duration:
        time.sleep(min(.05, max(0, duration - (time.monotonic() - started))))
        offset = time.time_ns() - time.monotonic_ns()
        step = offset - previous
        if abs(step) > abs(largest_step):
            largest_step = step
        if abs(step) > threshold_ms * 1e6:
            jumps.append({'elapsed_seconds': time.monotonic() - started,
                          'step_ms': step / 1e6})
        previous = offset
    return {'status': 'FAIL' if jumps else 'PASS',
            'duration_seconds': time.monotonic() - started,
            'threshold_ms': threshold_ms, 'largest_step_ms': largest_step / 1e6,
            'clock_steps': jumps,
            'scope': 'System clock versus monotonic clock; does not test /clock or set system time.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=15,
                        help='Observation duration in seconds (default: 15).')
    parser.add_argument('--threshold-ms', type=float, default=100,
                        help='Maximum clock step between samples (default: 100 ms).')
    parser.add_argument('--output', type=Path, help='Optional JSON evidence file.')
    args = parser.parse_args()
    if any(not math.isfinite(value) or value <= 0
           for value in (args.duration, args.threshold_ms)):
        parser.error('duration and threshold must be finite and positive')
    result = check_clock(args.duration, args.threshold_ms)
    payload = json.dumps(result, indent=2) + '\n'
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload)
    print(payload, end='')
    return 0 if result['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
