#!/usr/bin/env python3
"""Prove detectors reject a known leak/UB/race before trusting negative results."""
import argparse
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess

CASES = {
    "leak": ("#include <cstdlib>\n__attribute__((noinline)) void leak(){volatile int *p=(int*)malloc(64); *p=7;}\nint main(){leak();}",
             "-fsanitize=address", "LeakSanitizer"),
    "undefined": ("#include <climits>\nint main(){volatile int n=INT_MAX; return n+1;}",
                  "-fsanitize=undefined", "runtime error:"),
    "race": ("#include <thread>\nint value;\nvoid write(){for(int i=0;i<100000;++i) value++;}\nint main(){std::thread a(write), b(write);a.join();b.join();}",
             "-fsanitize=thread", "ThreadSanitizer: data race"),
}


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
    if args.plugin_probe:
        env = dict(os.environ)
        symbolizer = env.get("ASAN_SYMBOLIZER_PATH", "llvm-symbolizer")
        env.update(ASAN_OPTIONS=asan_options(symbolizer),
                   UBSAN_OPTIONS=f"halt_on_error=1:external_symbolizer_path={symbolizer}")
        process = subprocess.run([str(args.plugin_probe)], env=env, stdout=subprocess.PIPE,
                                 stderr=subprocess.STDOUT, timeout=30)
        output = process.stdout.decode(errors="replace")
        (args.output / "plugin-leak.log").write_text(output)
        passed = (process.returncode != 0 and "LeakSanitizer" in output and
                  "Direct leak of 64 byte(s)" in output and "Suppressions used:" in output)
        result = {"status": "PASS" if passed else "FAIL", "exit_code": process.returncode,
                  "instance_leak_detected_with_registration_suppression": passed}
        (args.output / "plugin-result.json").write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result))
        if not passed:
            print(output)
        return int(not passed)
    results = {}
    for name in (("leak", "undefined") if args.mode == "asan" else ("race",)):
        code, flag, expected = CASES[name]
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
        process = subprocess.run([str(binary)], env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=30)
        output = process.stdout.decode(errors="replace")
        (args.output / f"{name}.log").write_text(output)
        results[name] = {"exit_code": process.returncode, "detected": expected in output,
                         "passed": process.returncode != 0 and expected in output}
    passed = all(item["passed"] for item in results.values())
    (args.output / "result.json").write_text(json.dumps({"status": "PASS" if passed else "FAIL", "cases": results}, indent=2) + "\n")
    print(json.dumps(results, indent=2))
    return int(not passed)


if __name__ == "__main__":
    raise SystemExit(main())
