#!/usr/bin/env python3
"""Check same-process lifecycle resource bounds; sanitizers establish heap errors."""
import argparse
import json
from pathlib import Path
import statistics
import subprocess


def evaluate(samples, instrumented=False):
    failures = []
    if len(samples) < 140 or [item["cycle"] for item in samples] != list(range(140)):
        return ["Expected 140 complete lifecycle cycles"], {}
    if len({item["pid"] for item in samples}) != 1:
        failures.append("Lifecycle was not measured in one simulator process")
    stable = samples[40:]
    delta = {key: statistics.median([item[key] for item in stable[-20:]]) -
             statistics.median([item[key] for item in stable[:20]]) for key in ("rss_bytes", "fds", "threads")}
    if not instrumented and delta["rss_bytes"] > 32 * 1024 * 1024:
        failures.append("Post-warmup simulator RSS grew by more than 32 MiB")
    for key, tolerance in (("fds", 2), ("threads", 1)):
        if max(item[key] for item in stable) - min(item[key] for item in stable) > tolerance:
            failures.append(f"Post-warmup {key} did not remain bounded")
    return failures, delta


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--instrumented", action="store_true")
    args = parser.parse_args()
    raw = args.output.with_suffix(".jsonl")
    with raw.open("w") as output:
        process = subprocess.run([str(args.binary), str(args.config)], stdout=output, timeout=480)
    samples = [json.loads(line) for line in raw.read_text().splitlines() if line.startswith("{")]
    failures, delta = evaluate(samples, args.instrumented)
    if process.returncode:
        failures.append(f"Lifecycle child exit code: {process.returncode}")
    result = {"status": "FAIL" if failures else "PASS", "failures": failures,
              "cycles": len(samples), "process_exit_code": process.returncode, "post_warmup_delta": delta,
              "rss_gate": "Recorded only: sanitizer quarantine changes RSS" if args.instrumented else "32 MiB after 40 warmup cycles",
              "scope": "140 create/run/set-pose/fault/destroy and failed-initialization cycles in one process. This bounded test does not certify an 8-hour run or absence of every possible leak."}
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    return int(bool(failures))


if __name__ == "__main__":
    raise SystemExit(main())
