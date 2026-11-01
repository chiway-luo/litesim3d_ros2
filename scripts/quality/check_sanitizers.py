#!/usr/bin/env python3
"""Reject sanitizer findings in child logs even when a launch parent exits zero."""
import argparse
import json
from pathlib import Path
import re

from audit import compilation_errors
from inventory import inventory
from sanitizer_control import FINDING

SUPPRESSION = re.compile(r"^\s*(\d+)\s+(\d+)\s+(\S.*)\s*$")


def findings(evidence):
    result = []
    for path in sorted(evidence.rglob("*")):
        if not path.is_file() or path.suffix in {".xml", ".json", ".png", ".profraw"}:
            continue
        if "positive-control" in path.relative_to(evidence).parts:
            continue # Verified independently; deliberately faulty programs.
        if path.suffix == ".log" or re.match(r"^(asan|lsan|ubsan|tsan)\.[0-9]+$", path.name):
            for number, line in enumerate(path.read_text(errors="replace").splitlines(), 1):
                if FINDING.search(line):
                    result.append({"file": str(path), "line": number, "message": line.strip()})
    return result


def suppression_usage(evidence):
    """Keep dependency exceptions visible instead of silently discarding them."""
    result = []
    rules = {line.removeprefix("leak:") for name in ("lsan.supp", "lsan_controller_host.supp")
             for line in Path(__file__).with_name(name).read_text().splitlines() if line.startswith("leak:")}
    for path in sorted(evidence.rglob("*")):
        if (not path.is_file() or "positive-control" in path.relative_to(evidence).parts or
                not (path.suffix == ".log" or re.match(r"^(asan|lsan|ubsan)\.[0-9]+$", path.name))):
            continue
        for line in path.read_text(errors="replace").splitlines():
            match = SUPPRESSION.match(line)
            if match and match[3] in rules:
                result.append({"file": str(path), "allocations": int(match[1]),
                               "bytes": int(match[2]), "rule": match[3]})
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("root", "build", "evidence"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--mode", choices=("asan", "tsan"), required=True)
    args = parser.parse_args()
    flag = "-fsanitize=address,undefined" if args.mode == "asan" else "-fsanitize=thread"
    failures = compilation_errors(args.root, args.build, inventory(args.root), flag)
    detected = findings(args.evidence)
    if detected:
        failures.append(f"{len(detected)} sanitizer diagnostic(s), including child processes")
    control = args.evidence / "positive-control/result.json"
    if not control.exists() or json.loads(control.read_text()).get("status") != "PASS":
        failures.append("Missing successful sanitizer positive control")
    if args.mode == "asan":
        plugin_control = args.evidence / "positive-control/plugin-result.json"
        if not plugin_control.exists() or json.loads(plugin_control.read_text()).get("status") != "PASS":
            failures.append("Plugin instance leak must remain detectable with dependency suppressions")
        control_probe = args.evidence / "positive-control/control/result.json"
        if not control_probe.exists() or json.loads(control_probe.read_text()).get("status") != "PASS":
            failures.append("Hardware and controller instance leaks must remain detectable")
    result = {"status": "FAIL" if failures else "PASS", "failures": failures,
              "findings": detected, "mode": args.mode, "leak_detection": args.mode == "asan",
              "dependency_suppression_usage": suppression_usage(args.evidence),
              "suppression_policy": "scripts/quality/lsan.supp" if args.mode == "asan" else None,
              "controller_host_suppression_policy": "scripts/quality/lsan_controller_host.supp" if args.mode == "asan" else None}
    (args.evidence / "sanitizers.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({key: value for key, value in result.items() if key != "dependency_suppression_usage"}, indent=2))
    return int(bool(failures))


if __name__ == "__main__":
    raise SystemExit(main())
