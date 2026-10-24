"""Audit the two control registration exceptions against real pluginlib loads."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess

from sanitizer_control import asan_options
from asan_host import runtime_library, write_wrapper


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    root = args.output.resolve()
    root.mkdir(parents=True, exist_ok=True)
    build, prefix = root / "build", root / "install"
    fixture = Path(__file__).resolve().parents[1] / "fixtures/control_plugin_probe"
    for command in (["cmake", "-S", str(fixture), "-B", str(build),
                     f"-DCMAKE_CXX_COMPILER={args.compiler}", f"-DCMAKE_INSTALL_PREFIX={prefix}"],
                    ["cmake", "--build", str(build), "--parallel", "1", "--target", "install"]):
        subprocess.run(command, check=True, timeout=120)
    env = dict(os.environ)
    env["AMENT_PREFIX_PATH"] = str(prefix) + os.pathsep + env.get("AMENT_PREFIX_PATH", "")
    symbolizer = env.get("ASAN_SYMBOLIZER_PATH") or shutil.which("llvm-symbolizer-14")
    if not symbolizer:
        raise RuntimeError("Control plugin audit requires llvm-symbolizer-14")
    env["ASAN_OPTIONS"] = asan_options(symbolizer)
    env["UBSAN_OPTIONS"] = "halt_on_error=1:print_stacktrace=1"
    suppression = Path(__file__).with_name("lsan.supp")
    host_policy = root / "host.supp"
    host_policy.write_text(suppression.read_text() + "\n" +
                           Path(__file__).with_name("lsan_controller_host.supp").read_text())
    runtime = runtime_library(args.compiler)
    raw_host, host = root / "raw-host", root / "host"
    write_wrapper(raw_host, build / "control_plugin_probe_stripped", runtime)
    write_wrapper(host, build / "control_plugin_probe_stripped", runtime, host_policy)
    cases = {}
    for loader in ("native", "stripped-host"):
        for kind in ("hardware", "controller"):
            for mode in ("registration", "clean", "leak"):
                current = dict(env)
                current.pop("LITESIM_PROBE_LEAK", None)
                current["LSAN_OPTIONS"] = "" if mode == "registration" else f"suppressions={suppression}:print_suppressions=1"
                if mode == "leak":
                    current["LITESIM_PROBE_LEAK"] = "1"
                binary = (build / "control_plugin_probe" if loader == "native" else
                          raw_host if mode == "registration" else host)
                process = subprocess.run([str(binary), kind], env=current,
                                         capture_output=True, text=True, timeout=30)
                output = process.stdout + process.stderr
                name = f"{loader}-{kind}-{mode}"
                (root / f"{name}.log").write_text(output)
                if mode == "clean":
                    passed = process.returncode == 0
                elif mode == "registration":
                    passed = process.returncode != 0 and "LeakSanitizer" in output and "class_loader::impl::loadLibrary(" in output
                else:
                    passed = process.returncode != 0 and "Direct leak of 64 byte(s)" in output and "Suppressions used:" in output
                cases[name] = {"exit_code": process.returncode, "passed": passed}
    result = {"status": "PASS" if all(item["passed"] for item in cases.values()) else "FAIL", "cases": cases}
    (root / "result.json").write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))
    return int(result["status"] != "PASS")


if __name__ == "__main__":
    raise SystemExit(main())
