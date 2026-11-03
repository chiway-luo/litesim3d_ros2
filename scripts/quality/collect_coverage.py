#!/usr/bin/env python3
"""Collect LLVM/Python coverage of the current source inventory, including misses."""
import argparse
import hashlib
import json
import re
from pathlib import Path
import shutil
import subprocess
import sys

from audit import canonical_source, compilation_errors, has_function_body
from inventory import DECLARATION_SHA256, inventory, python_aliases
from python_startup.function_coverage import source_functions

METRICS = ("lines", "branches", "functions")
MINIMUM = 90
TOTAL_MINIMUM = {"lines": 90, "branches": 95, "functions": 90}

CRITICAL = {f"litesim3d_core/src/{name}.cpp" for name in
            ("geometry", "model", "simulator", "sensors", "terrain", "world")}


def counts(covered, total):
    return {"covered": covered, "count": total, "percent": 100.0 * covered / total if total else 100.0}


def native_metrics(lcov, exported, normalize):
    """Union canonical source locations, not DSO or template instantiations.

    A native function is its definition-body source range. Constructor ABI
    variants, template instantiations and installed-header copies share one
    source function; distinct overloads/lambdas have distinct body ranges.
    """
    lines, branches, functions, current = {}, {}, {}, None
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
        for function in datum.get("functions", []):
            # The first code region is the defining function body, not a
            # dependency macro expansion or an inlined callee in another file.
            region = next((value for value in function["regions"] if value[7] == 0), None)
            if region is None:
                continue
            name = normalize(function["filenames"][region[5]])
            if name:
                target = functions.setdefault(name, {})
                key = tuple(region[:4])
                target[key] = max(target.get(key, 0), function["count"])
    locations = {"lines": lines, "branches": branches, "functions": functions}
    names = set().union(*(values.keys() for values in locations.values()))
    return {name: {kind: counts(sum(value > 0 for value in values.get(name, {}).values()),
                                len(values.get(name, {})))
                   for kind, values in locations.items()} for name in names}


def sum_metrics(files):
    return {kind: counts(sum(file[kind]["covered"] for file in files.values()),
                         sum(file[kind]["count"] for file in files.values()))
            for kind in METRICS}


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
        for kind in METRICS:
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
    # Integer arithmetic enforces strict per-metric thresholds, even when the
    # displayed percentage rounds to the limit. Combined hits never rescue unit.
    for kind in METRICS:
        value = total[kind]
        minimum = TOTAL_MINIMUM[kind]
        if value["count"] == 0 or 100 * value["covered"] <= minimum * value["count"]:
            failures.append(f"Total {kind}: {value['percent']:.2f}% <= {minimum}%")
    for name in sorted(CRITICAL):
        if name not in metrics:
            continue
        for kind in METRICS:
            value = metrics[name][kind]
            if value["count"] and 100 * value["covered"] <= MINIMUM * value["count"]:
                failures.append(f"{name} {kind}: {value['percent']:.2f}% <= {MINIMUM}%")
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


def binary_id(binary):
    readelf = shutil.which("llvm-readelf-14") or shutil.which("readelf")
    if not readelf:
        raise RuntimeError("readelf is required for native profile attribution")
    output = subprocess.check_output([readelf, "--notes", str(binary)], text=True)
    values = set(re.findall(r"Build ID:\s*([0-9a-fA-F]+)", output))
    if len(values) != 1:
        raise RuntimeError(f"Native object requires one build ID: {binary}")
    return values.pop().lower()


def profile_binary_id(profdata, path):
    output = subprocess.check_output([profdata, "show", "--binary-ids", str(path)], text=True)
    values = re.findall(r"^\s*([0-9a-fA-F]+)\s*$", output, re.MULTILINE)
    # A profile with merged module identities has already lost attribution of
    # same-name/hash counters. Never transfer those counts into either object.
    if len(values) != 1:
        raise RuntimeError(f"Native raw profile requires one unambiguous binary ID: {path}")
    return values[0].lower()


def module_prefix(binary):
    matches = []
    for path in binary.parent.glob("litesim-coverage-*.json"):
        data = json.loads(path.read_text())
        if Path(data["binary"]).resolve() == binary.resolve():
            matches.append(data["filename_prefix"])
    if len(matches) != 1 or not matches[0] or any(char in matches[0] for char in "/\\%"):
        raise RuntimeError(f"Native object requires one valid coverage-module manifest: {binary}")
    return matches[0]


def native_report(args, kind, binary_paths, report):
    profdata, llvm_cov = llvm_tools()
    directories = [args.evidence / "profiles/unit"]
    if kind == "combined":
        directories.append(args.evidence / "profiles/integration")
    raw = sorted(path for directory in directories for path in directory.rglob("*.profraw") if path.stat().st_size)
    if not raw and kind != "baseline":
        raise RuntimeError(f"No {kind} native execution profiles")
    # Empty mapping profiles never supply hits, including for unexecuted GUI
    # executables. Their complete compiled mappings still enter the denominator.
    empty_text, empty_data = report / "empty.proftext", report / "empty.profdata"
    if not empty_data.exists():
        empty_text.write_text("__litesim_empty_baseline\n0\n1\n0\n")
        subprocess.run([profdata, "merge", "-sparse", str(empty_text), "-o", str(empty_data)], check=True)
    modules, prefixes = {}, set()
    for binary in binary_paths:
        identity = binary_id(binary)
        if identity in modules:
            raise RuntimeError(f"Duplicate instrumented ELF build ID: {binary}")
        prefix = module_prefix(binary)
        if any(prefix.startswith(other) or other.startswith(prefix) for other in prefixes):
            raise RuntimeError(f"Ambiguous native coverage-module filename prefix: {prefix}")
        prefixes.add(prefix)
        modules[identity] = (binary, prefix)
    by_id = {}
    if kind != "baseline":
        for path in raw:
            identity = profile_binary_id(profdata, path)
            if identity not in modules:
                raise RuntimeError(f"Native raw profile is not from this instrumented build: {path}")
            if not path.name.startswith(modules[identity][1]):
                # LLVM14 signature collisions can retain only the *last* build
                # ID while mixing two DSOs' counters. One ID alone is not proof.
                raise RuntimeError(f"Unqualified native profile may contain module-collision hits: {path}")
            by_id.setdefault(identity, []).append(path)
    # Independent object AND profile exports are essential. Globally merged
    # profiles can give an unexecuted main() hits from a different executable
    # when their external name and CFG hash happen to match.
    exported = {"data": []}
    lcov, attribution = [], {}
    object_reports = report / "native" / kind
    object_reports.mkdir(parents=True, exist_ok=True)
    for index, binary in enumerate(binary_paths):
        identity = next(key for key, value in modules.items() if value[0] == binary)
        matching = by_id.get(identity, [])
        merged = empty_data
        if matching:
            merged = object_reports / f"{index:03d}.profdata"
            subprocess.run([profdata, "merge", "-sparse", *map(str, matching), "-o", str(merged)], check=True)
        attribution[str(binary)] = {"binary_id": identity, "filename_prefix": modules[identity][1],
                                    "raw_profiles": list(map(str, matching))}
        command = [llvm_cov, "export", str(binary), f"-instr-profile={merged}"]
        json_path = object_reports / f"{index:03d}.json"
        lcov_path = object_reports / f"{index:03d}.lcov"
        with json_path.open("w") as output:
            subprocess.run(command, stdout=output, check=True)
        with lcov_path.open("w") as output:
            subprocess.run([*command, "-format=lcov"], stdout=output, check=True)
        exported["data"].extend(json.loads(json_path.read_text())["data"])
        lcov.append(lcov_path.read_text())
    (report / f"{kind}-native-attribution.json").write_text(json.dumps(attribution, indent=2) + "\n")
    item = inventory(args.root)
    return native_metrics("\n".join(lcov), exported,
                          lambda path: canonical_source(path, args.root, args.prefix, item))


def python_functions(args, directories, item):
    """Count statically inventoried callables with measured function-entry hits."""
    aliases = {str(Path(path).resolve()): name
               for name, paths in python_aliases(args.root, args.prefix, item).items()
               for path in paths}
    hits = {}
    profiles = [path for directory in directories for path in directory.glob(".functions.*.json")]
    if not profiles:
        raise RuntimeError("No Python function-entry profiles; enable the quality startup hook")
    for profile in profiles:
        data = json.loads(profile.read_text())
        if data["version"] != 1:
            raise RuntimeError(f"Unsupported Python function-entry profile: {profile}")
        for filename, values in data["files"].items():
            name = aliases.get(str(Path(filename).resolve()))
            if name:
                hits.setdefault(name, set()).update(values)
    result = {}
    for name, source in item["sources"].items():
        if source["language"] != "python":
            continue
        definitions = source_functions(args.root / name)
        measured = hits.get(name, set())
        if measured - definitions:
            raise RuntimeError(f"Python function entries do not match the current source: {name}")
        result[name] = counts(len(measured), len(definitions))
    return result


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
    functions = python_functions(args, directories, item)
    result = {}
    for path, file in json.loads(target.read_text())["files"].items():
        name = canonical_source(path, args.root, args.prefix, item)
        if name:
            data = file["summary"]
            result[name] = {"lines": counts(data["covered_lines"], data["num_statements"]),
                            "branches": counts(data["covered_branches"], data["num_branches"]),
                            "functions": functions[name]}
    return result


def collect(args):
    item = inventory(args.root)
    report = args.evidence / "coverage"
    report.mkdir(exist_ok=True)
    failures = compilation_errors(args.root, args.build, item, "-fcoverage-mapping")
    for name, source in item["sources"].items():
        expected = DECLARATION_SHA256.get(name)
        if source["declarations_only"] and expected and hashlib.sha256((args.root / name).read_bytes()).hexdigest() != expected:
            failures.append("Declaration-only source changed; re-review required: " + name)
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
              "thresholds": {"comparison": ">", "unit_total": dict(TOTAL_MINIMUM),
                             "combined_total": dict(TOTAL_MINIMUM),
                             "critical_core": dict.fromkeys(METRICS, MINIMUM)},
              "inventory": item, "objects": list(map(str, binary_paths)),
              "total": {kind: summary["total"] for kind, summary in summaries.items()},
              "semantics": "Native: LLVM executable lines, source-condition outcomes and function-definition body locations, unioned across DSOs/templates/installed headers; independent per-object exports and exact raw-profile ELF build-ID attribution retain same-name functions without transferring hits from other executables. Python: coverage.py statements/branches plus measured function-entry calls for statically inventoried functions, methods and lambdas (not module/class bodies or generated comprehensions). Never average file percentages. Aggregate branches must be strictly >95%, and lines/functions strictly >90%, using unit execution alone; combined execution is reported and gated separately and cannot supply unit hits. Critical core files also require >90% for nonempty metrics. Full runtime source inventory and zero-hit native maps fix denominators. Uninstantiated C++ inline/template bodies without LLVM mappings are not measurable."}
    (report / "coverage-gate.json").write_text(json.dumps(result, indent=2) + "\n")
    rows = ["# Current-source coverage", "", result["semantics"], "",
            "| Package (unit only) | Lines | Branches | Functions |", "| --- | ---: | ---: | ---: |"]
    for package, data in summaries["unit"]["packages"].items():
        rows.append(f"| {package} | {data['lines']['percent']:.2f}% | {data['branches']['percent']:.2f}% | {data['functions']['percent']:.2f}% |")
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
