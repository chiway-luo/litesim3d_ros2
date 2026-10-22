#!/usr/bin/env python3
"""Collect LLVM/Python coverage of the current source inventory, including misses."""
import argparse
import json
from pathlib import Path
import shutil
import subprocess
import sys

from audit import canonical_source, compilation_errors, has_function_body
from inventory import inventory

CRITICAL = {f"litesim3d_core/src/{name}.cpp" for name in
            ("geometry", "model", "simulator", "sensors", "terrain", "world")}


def counts(covered, total):
    return {"covered": covered, "count": total, "percent": 100.0 * covered / total if total else 100.0}


def native_metrics(lcov, exported, normalize):
    """Union line/condition locations across DSOs, templates and installed headers."""
    lines, branches, current = {}, {}, None
    for record in lcov.splitlines():
        if record.startswith("SF:"):
            current = normalize(record[3:])
            if current:
                lines.setdefault(current, {})
        elif current and record.startswith("DA:"):
            number, value, *_ = record[3:].split(",")
            key = int(number)
            lines[current][key] = max(lines[current].get(key, 0), int(value))
    for datum in exported["data"]:
        for file in datum["files"]:
            name = normalize(file["filename"])
            if not name:
                continue
            target = branches.setdefault(name, {})
            for branch in file.get("branches", []):
                for side in (0, 1):
                    key = (*branch[:4], side)
                    target[key] = max(target.get(key, 0), branch[4 + side])
    return {name: {"lines": counts(sum(value > 0 for value in data.values()), len(data)),
                   "branches": counts(sum(value > 0 for value in branches.get(name, {}).values()),
                                      len(branches.get(name, {})))} for name, data in lines.items()}


def sum_metrics(files):
    return {kind: counts(sum(file[kind]["covered"] for file in files.values()),
                         sum(file[kind]["count"] for file in files.values()))
            for kind in ("lines", "branches")}


def fixed_denominators(observed, baseline):
    """LLVM can drop mismatched same-name functions (e.g. multiple mains).

    Read the complete zero-hit mapping separately so missing profiles cannot
    shrink the denominator. Only measured hits contribute to the numerator.
    """
    if set(observed) - set(baseline):
        raise RuntimeError("Observed source is missing from the static LLVM mapping")
    result = {}
    for name, mapping in baseline.items():
        result[name] = {}
        for kind in ("lines", "branches"):
            hit = observed.get(name, {}).get(kind, {}).get("covered", 0)
            if hit > mapping[kind]["count"]:
                raise RuntimeError(f"Observed {kind} exceed the static mapping: {name}")
            result[name][kind] = counts(hit, mapping[kind]["count"])
    return result


def gate(item, metrics, unit=False):
    failures = []
    for name, source in item["sources"].items():
        if name not in metrics and not source["declarations_only"]:
            failures.append("Missing source coverage: " + name)
        elif name in metrics and Path(name).suffix in {".cpp", ".cc", ".cxx"} and metrics[name]["lines"]["count"] == 0:
            failures.append("No executable coverage mapping: " + name)
    total = sum_metrics(metrics)
    if not unit:
        for kind, minimum in (("lines", 80), ("branches", 70)):
            if total[kind]["count"] == 0 or total[kind]["percent"] < minimum:
                failures.append(f"Total {kind}: {total[kind]['percent']:.2f}% < {minimum}%")
    for name in sorted(CRITICAL):
        if name not in metrics:
            continue
        for kind, minimum in (("lines", 90), ("branches", 80)):
            if metrics[name][kind]["percent"] < minimum:
                failures.append(f"{name} {kind}: {metrics[name][kind]['percent']:.2f}% < {minimum}%")
    return total, failures


def llvm_tools():
    paths = [shutil.which(name + "-14") or shutil.which(name) for name in ("llvm-profdata", "llvm-cov")]
    if not all(paths):
        raise RuntimeError("llvm-14 is required")
    return paths


def objects(build):
    result = []
    # Top-level build outputs avoid installed copies and CMake compiler probes.
    for path in sorted(build.glob("*/*")):
        if path.is_file() and not path.is_symlink():
            with path.open("rb") as stream:
                if stream.read(4) == b"\x7fELF" and b"__llvm_covmap" in stream.read():
                    result.append(path)
    if not result:
        raise RuntimeError("No instrumented ELF objects")
    return result


def native_report(args, kind, binary_paths, report):
    profdata, llvm_cov = llvm_tools()
    directories = [args.evidence / "profiles/unit"]
    if kind == "combined":
        directories.append(args.evidence / "profiles/integration")
    raw = sorted(path for directory in directories for path in directory.glob("*.profraw") if path.stat().st_size)
    if not raw and kind != "baseline":
        raise RuntimeError(f"No {kind} native execution profiles")
    merged = report / f"{kind}.profdata"
    if kind == "baseline":
        # A zero-count text profile produces a valid empty indexed profile.
        # The dummy name matches no project function and never supplies hits.
        empty = report / "empty.proftext"
        empty.write_text("__litesim_empty_baseline\n0\n1\n0\n")
        raw = [empty]
    subprocess.run([profdata, "merge", "-sparse", *map(str, raw), "-o", str(merged)], check=True)
    command = [llvm_cov, "export", str(binary_paths[0]), f"-instr-profile={merged}"]
    command += [f"-object={path}" for path in binary_paths[1:]]
    exported = report / f"{kind}-llvm.json"
    lcov = report / f"{kind}.lcov"
    with exported.open("w") as output:
        subprocess.run(command, stdout=output, check=True)
    with lcov.open("w") as output:
        subprocess.run([*command, "-format=lcov"], stdout=output, check=True)
    item = inventory(args.root)
    return native_metrics(lcov.read_text(), json.loads(exported.read_text()),
                          lambda path: canonical_source(path, args.root, args.prefix, item))


def python_report(args, kind, item, report):
    import coverage
    directories = [args.evidence / "profiles/unit/python"]
    if kind == "combined":
        directories.append(args.evidence / "profiles/integration/python")
    cov = coverage.Coverage(config_file=str(args.evidence / "coveragerc"),
                            data_file=str(report / f"python-{kind}"))
    # Explicit files are essential: the destination basename differs from the
    # subprocess .coverage.* basename, which directory-based combine filters on.
    raw = sorted(path for directory in directories for path in directory.glob(".coverage.*"))
    cov.combine(data_paths=list(map(str, raw)), strict=True, keep=True)
    cov.save()
    target = report / f"{kind}-python.json"
    cov.json_report(morfs=[str(args.root / name) for name, source in item["sources"].items()
                           if source["language"] == "python"], outfile=str(target))
    result = {}
    for path, file in json.loads(target.read_text())["files"].items():
        name = canonical_source(path, args.root, args.prefix, item)
        if name:
            data = file["summary"]
            result[name] = {"lines": counts(data["covered_lines"], data["num_statements"]),
                            "branches": counts(data["covered_branches"], data["num_branches"])}
    return result


def collect(args):
    item = inventory(args.root)
    report = args.evidence / "coverage"
    report.mkdir(exist_ok=True)
    failures = compilation_errors(args.root, args.build, item, "-fcoverage-mapping")
    for name, source in item["sources"].items():
        if source["declarations_only"] and has_function_body((args.root / name).read_text()):
            failures.append("Declaration-only exemption now contains a function: " + name)
    binary_paths = objects(args.build)
    baseline = native_report(args, "baseline", binary_paths, report)
    summaries = {}
    for kind in ("unit", "combined"):
        files = fixed_denominators(native_report(args, kind, binary_paths, report), baseline)
        files.update(python_report(args, kind, item, report))
        total, errors = gate(item, files, unit=kind == "unit")
        packages = {package: sum_metrics({name: value for name, value in files.items()
                                         if item["sources"][name]["package"] == package})
                    for package in sorted({source["package"] for source in item["sources"].values()})}
        summaries[kind] = {"files": files, "total": total, "packages": packages,
                           "threshold_failures": errors}
        failures += [f"{kind}: {error}" for error in errors]
        (report / f"{kind}-coverage.json").write_text(json.dumps(summaries[kind], indent=2) + "\n")
    result = {"status": "FAIL" if failures else "PASS", "failures": failures,
              "thresholds": {"total": {"lines": 80, "branches": 70},
                             "critical_core": {"lines": 90, "branches": 80}},
              "inventory": item, "objects": list(map(str, binary_paths)),
              "total": {kind: summary["total"] for kind, summary in summaries.items()},
              "semantics": "Native: LLVM executable lines and source-condition outcomes, unioned across DSOs/templates/installed headers. Python: coverage.py statements and branch destinations. Never average file percentages. Critical core thresholds must pass with unit execution alone; overall thresholds use combined execution. Uninstantiated C++ inline/template bodies without LLVM mappings are not measurable."}
    (report / "coverage-gate.json").write_text(json.dumps(result, indent=2) + "\n")
    rows = ["# Current-source coverage", "", result["semantics"], "",
            "| Package | Lines | Branches |", "| --- | ---: | ---: |"]
    for package, data in summaries["combined"]["packages"].items():
        rows.append(f"| {package} | {data['lines']['percent']:.2f}% | {data['branches']['percent']:.2f}% |")
    rows += ["", "Result: " + result["status"], "", *["- " + failure for failure in failures]]
    (report / "summary.md").write_text("\n".join(rows) + "\n")
    print(json.dumps({key: result[key] for key in ("status", "failures", "total")}, indent=2))
    return bool(failures)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("root", "prefix", "build", "evidence"):
        parser.add_argument("--" + name, type=Path, required=True)
    args = parser.parse_args()
    return int(collect(args))


if __name__ == "__main__":
    sys.exit(main())
