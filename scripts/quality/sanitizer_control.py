#!/usr/bin/env python3
"""Prove detectors reject a known leak/UB/race before trusting negative results."""
import argparse
import json
import os
from pathlib import Path
import platform
import re
import shutil
import subprocess

CASES = {
    "leak": ("#include <cstdlib>\n__attribute__((noinline)) void leak(){volatile int *p=(int*)malloc(64); *p=7;}\nint main(){leak();}",
             "-fsanitize=address"),
    "undefined": ("#include <climits>\nint main(){volatile int n=INT_MAX; return n+1;}",
                  "-fsanitize=undefined"),
    "race": ("#include <thread>\nint value;\nvoid write(){for(int i=0;i<100000;++i) value++;}\nint main(){std::thread a(write), b(write);a.join();b.join();}",
             "-fsanitize=thread"),
}

# A detector's name is not evidence that it ran: LSan also prints its name
# when ptrace prevents leak checking. Keep runtime failures separate from the
# deliberate findings, and share these diagnostics with the child-log audit.
UNSUPPORTED = re.compile(
    r"LeakSanitizer does not work under ptrace|"
    r"(?:Address|Leak|Thread)Sanitizer (?:is not supported|does not support)|"
    r"FATAL: ThreadSanitizer: unexpected memory mapping")
FATAL = re.compile(
    r"(?:Address|Leak|Thread|UndefinedBehavior)Sanitizer has encountered a fatal error|"
    r"FATAL: (?:Address|Leak|Thread|UndefinedBehavior)Sanitizer|"
    r"(?:Address|Leak|Thread|UndefinedBehavior)?Sanitizer:? CHECK failed|"
    r"ASan runtime does not come first")
CRASH = re.compile(
    r"(?:Address|Leak|Thread|UndefinedBehavior)Sanitizer:\s*(?:DEADLYSIGNAL|SEGV|BUS|ABRT)|"
    r"Segmentation fault|Aborted \(core dumped\)")
FINDING = re.compile(
    r"(?:ERROR|WARNING|FATAL): (?:Address|Leak|Thread|UndefinedBehavior)Sanitizer|"
    r"SUMMARY: (?:Address|Leak|Thread|UndefinedBehavior)Sanitizer|runtime error:|"
    + UNSUPPORTED.pattern + "|" + FATAL.pattern + "|" + CRASH.pattern)
LEAK_HEADER = re.compile(r"^(?:==\d+==)?ERROR: LeakSanitizer: detected memory leaks[ \t]*$", re.M)
LEAK_SUMMARY = re.compile(
    r"^SUMMARY: (?:AddressSanitizer|LeakSanitizer): (\d+) byte\(s\) leaked in (\d+) allocation\(s\)\.[ \t]*$", re.M)
ALLOCATION = re.compile(
    r"^(Direct|Indirect) leak of (\d+) byte\(s\) in (\d+) object\(s\) allocated from:[ \t]*$", re.M)
FRAME = re.compile(r"^[ \t]+#(\d+) 0x[0-9a-fA-F]+[ \t]+\S.*$", re.M)
SUPPRESSION_TABLE = re.compile(
    r"^Suppressions used:\n[ \t]*count[ \t]+bytes[ \t]+template[ \t]*\n(.*?)^-{5,}[ \t]*$", re.M | re.S)
SUPPRESSION_ROW = re.compile(r"^[ \t]*(\d+)[ \t]+(\d+)[ \t]+(\S.*?)[ \t]*$", re.M)
PLUGIN_REGISTRATION_RULE = "pluginlib::ClassLoader<litesim3d::plugins::ChassisPlugin>::loadLibraryForClass"


def leak_report(output):
    """Return only a complete, internally consistent single-process LSan report."""
    headers, summaries = list(LEAK_HEADER.finditer(output)), list(LEAK_SUMMARY.finditer(output))
    allocations = list(ALLOCATION.finditer(output))
    if len(headers) != 1 or len(summaries) != 1 or not allocations:
        return None
    header, summary = headers[0], summaries[0]
    if not header.end() < allocations[0].start() < allocations[-1].end() < summary.start():
        return None
    blocks = []
    for index, allocation in enumerate(allocations):
        end = allocations[index + 1].start() if index + 1 < len(allocations) else summary.start()
        frames = list(FRAME.finditer(output[allocation.end():end]))
        size, count = int(allocation[2]), int(allocation[3])
        if (size <= 0 or count <= 0 or len(frames) < 2 or
                [int(frame[1]) for frame in frames] != list(range(len(frames)))):
            return None
        blocks.append({"kind": allocation[1], "bytes": size, "allocations": count,
                       "stack": "\n".join(frame[0] for frame in frames)})
    if (sum(block["bytes"] for block in blocks) != int(summary[1]) or
            sum(block["allocations"] for block in blocks) != int(summary[2])):
        return None
    return {"bytes": int(summary[1]), "allocations": int(summary[2]), "blocks": blocks}


def suppression_used(output, rules):
    """Require a positive entry for an expected rule, not just a table heading."""
    return any(int(row[1]) > 0 and int(row[2]) > 0 and row[3] in rules
               for table in SUPPRESSION_TABLE.finditer(output)
               for row in SUPPRESSION_ROW.finditer(table[1]))


def control_result(kind, returncode, output, *, expected_bytes=64, required_stack=None,
                   suppression_rules=()):
    """Fail closed on crashes, unavailable runtimes and incomplete diagnostics.

    ``expected_bytes=None`` audits registration leaks of platform-dependent
    size; the intentional instance controls must instead report exactly one
    direct 64-byte allocation. ``detected`` is separate from a successful exit
    policy: even a complete report with exit code zero cannot pass a control.
    """
    result = {"exit_code": returncode, "detected": False, "passed": False}
    if returncode is None:
        classification = "timeout"
    elif returncode < 0 or returncode >= 128:
        classification = "signal"
    elif UNSUPPORTED.search(output):
        classification = "unsupported"
    elif FATAL.search(output):
        classification = "fatal"
    elif CRASH.search(output):
        classification = "crash"
    else:
        if kind == "clean":
            detected = False
            classification = "unexpected_diagnostic" if FINDING.search(output) else "clean"
        elif kind == "leak":
            report = leak_report(output)
            detected = report is not None
            if detected and expected_bytes is not None:
                detected = (report["bytes"] == expected_bytes and report["allocations"] == 1 and
                            len(report["blocks"]) == 1 and report["blocks"][0]["kind"] == "Direct")
            if detected and required_stack:
                detected = any(required_stack in block["stack"] for block in report["blocks"])
            classification = ("detected" if detected else "unexpected_report" if report else
                              "incomplete_report" if LEAK_HEADER.search(output) else "not_detected")
            # A valid leak report cannot excuse an additional UB, ASan or TSan
            # finding in the same process.
            if any(FINDING.search(line) and not (LEAK_HEADER.fullmatch(line) or LEAK_SUMMARY.fullmatch(line))
                   for line in output.splitlines()):
                classification = "unexpected_diagnostic"
        elif kind == "undefined":
            diagnostics = [line for line in output.splitlines() if FINDING.search(line)]
            detected = bool(diagnostics) and all(re.search(
                r"undefined\.cpp:\d+:\d+: runtime error: signed integer overflow: "
                r"2147483647 \+ 1 cannot be represented in type 'int'", line)
                or line.startswith("SUMMARY: UndefinedBehaviorSanitizer: undefined-behavior ")
                for line in diagnostics)
            detected = detected and any("runtime error:" in line for line in diagnostics)
            classification = "detected" if detected else "not_detected"
        elif kind == "race":
            diagnostics = [line for line in output.splitlines() if FINDING.search(line)]
            warning = re.compile(r"^WARNING: ThreadSanitizer: data race(?: \(pid=\d+\))?$")
            summary = re.compile(r"^SUMMARY: ThreadSanitizer: data race .+")
            detected = (any(warning.match(line) for line in diagnostics) and
                        any(summary.match(line) for line in diagnostics) and
                        all(warning.match(line) or summary.match(line) for line in diagnostics))
            classification = "detected" if detected else "incomplete_report"
        else:
            raise ValueError(f"Unknown sanitizer control: {kind}")
        result["detected"] = detected
        if classification in {"detected", "clean"}:
            if kind != "clean" and returncode == 0:
                classification = "zero_exit"
            elif kind == "clean" and returncode != 0:
                classification = "nonzero_exit"
            elif suppression_rules and not suppression_used(output, suppression_rules):
                classification = "missing_suppression"
            else:
                result["passed"] = True
    result["classification"] = classification
    return result


def run_probe(command, env):
    """Capture timeout evidence without mistaking a partial report for success."""
    try:
        process = subprocess.run(command, env=env, stdout=subprocess.PIPE,
                                 stderr=subprocess.STDOUT, timeout=30)
        return process.returncode, process.stdout.decode(errors="replace")
    except subprocess.TimeoutExpired as error:
        output = error.output or b""
        return None, output.decode(errors="replace") if isinstance(output, bytes) else output


def asan_options(symbolizer, log_prefix=None):
    # Ubuntu's x86_64 dependency binaries can omit frame pointers. The fast
    # allocator unwinder then loses the loading frames used by narrow LSan
    # suppressions. Use DWARF unwinding in both controls and real test processes.
    # Mesa allocation paths can exceed the default 30 frames; retain the
    # owning renderer frames even after the driver libraries are unloaded.
    options = ("detect_leaks=1:halt_on_error=1:fast_unwind_on_malloc=0:malloc_context_size=64:"
               f"external_symbolizer_path={symbolizer}")
    return options + (f":log_path={log_prefix}" if log_prefix is not None else "")


def asan_link_options(llvm_unwind=False):
    # Match instrument.cmake, including the unwinder used by the real renderer.
    flags = []
    if llvm_unwind:
        directory = os.environ.get("LITESIM_LLVM_UNWIND_DIR")
        if not directory:
            config = shutil.which("llvm-config-14")
            if not config:
                raise RuntimeError("ASan rendering requires llvm-14 and libunwind-14-dev")
            directory = subprocess.check_output([config, "--libdir"], text=True).strip()
        if not (Path(directory) / "libunwind.so").exists():
            raise RuntimeError("ASan rendering requires libunwind-14-dev")
        flags += ["-rtlib=compiler-rt", "-unwindlib=libunwind", f"-L{directory}"]
    if platform.system() == "Linux" and platform.machine().lower() in ("x86_64", "amd64"):
        flags.append("-no-pie")
    return flags


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler", required=True)
    parser.add_argument("--mode", choices=("asan", "tsan"), required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--plugin-probe", type=Path)
    parser.add_argument("--llvm-unwind", action="store_true", help="Verify the ASan renderer's unwinder")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    # Fail closed if a rebuild, exec, or later case raises before aggregation.
    # Output directories may be reused; an old PASS must not survive a new run.
    result_path = args.output / ("plugin-result.json" if args.plugin_probe else "result.json")
    incomplete = {"status": "FAIL", "classification": "incomplete"}
    incomplete.update({"exit_code": None, "instance_leak_detected_with_registration_suppression": False}
                      if args.plugin_probe else {"cases": {}})
    result_path.write_text(json.dumps(incomplete) + "\n")
    if args.plugin_probe:
        env = dict(os.environ)
        symbolizer = env.get("ASAN_SYMBOLIZER_PATH", "llvm-symbolizer")
        env.update(ASAN_OPTIONS=asan_options(symbolizer),
                   UBSAN_OPTIONS=f"halt_on_error=1:external_symbolizer_path={symbolizer}")
        returncode, output = run_probe([str(args.plugin_probe)], env)
        (args.output / "plugin-leak.log").write_text(output)
        result = control_result("leak", returncode, output,
                                suppression_rules=(PLUGIN_REGISTRATION_RULE,))
        passed = result["passed"]
        result.update({"status": "PASS" if passed else "FAIL",
                       "instance_leak_detected_with_registration_suppression": passed})
        result_path.write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result))
        if not passed:
            print(output)
        return int(not passed)
    results = {}
    for name in (("leak", "undefined") if args.mode == "asan" else ("race",)):
        code, flag = CASES[name]
        source, binary = args.output / f"{name}.cpp", args.output / name
        source.write_text(code)
        subprocess.run([args.compiler, "-O0", "-g", "-pthread", flag, "-fno-sanitize-recover=all",
                        *(asan_link_options(args.llvm_unwind) if args.mode == "asan" else []),
                        str(source), "-o", str(binary)], check=True)
        env = dict(os.environ)
        symbolizer = env.get("ASAN_SYMBOLIZER_PATH", "llvm-symbolizer")
        env.update(ASAN_OPTIONS=asan_options(symbolizer),
                   UBSAN_OPTIONS=f"halt_on_error=1:external_symbolizer_path={symbolizer}",
                   TSAN_OPTIONS=f"halt_on_error=1:external_symbolizer_path={symbolizer}")
        returncode, output = run_probe([str(binary)], env)
        (args.output / f"{name}.log").write_text(output)
        results[name] = control_result(name, returncode, output)
    passed = all(item["passed"] for item in results.values())
    result_path.write_text(json.dumps({"status": "PASS" if passed else "FAIL", "cases": results}, indent=2) + "\n")
    print(json.dumps(results, indent=2))
    return int(not passed)


if __name__ == "__main__":
    raise SystemExit(main())
