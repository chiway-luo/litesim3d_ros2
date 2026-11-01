"""Apply process-local ASan runtime and LSan policy to ROS control hosts."""
import argparse
import json
import os
from pathlib import Path
import platform
import re
import subprocess


def runtime_library(compiler):
    arch = {"arm64": "aarch64", "amd64": "x86_64"}.get(platform.machine().lower(), platform.machine().lower())
    path = Path(subprocess.check_output(
        [compiler, f"-print-file-name=libclang_rt.asan-{arch}.so"], text=True).strip())
    if not path.is_absolute() or not path.is_file():
        raise RuntimeError(f"Missing shared ASan runtime for {compiler}: {path}")
    return path.resolve()


def preload_command(executable):
    """Keep an x86-64 PIE host outside Clang ASan's fixed allocator range."""
    command = [str(executable)]
    if platform.system() != "Linux" or platform.machine().lower() not in {"x86_64", "amd64"}:
        return command
    headers = subprocess.check_output(
        ["readelf", "--file-header", "--program-headers", "--wide", str(executable)],
        text=True, env={**os.environ, "LC_ALL": "C"})
    if not re.search(r"Type:\s+DYN\b", headers):
        return command
    match = re.search(r"\[Requesting program interpreter: ([^\]]+)\]", headers)
    if not match or not Path(match[1]).is_absolute() or not Path(match[1]).is_file():
        raise RuntimeError(f"Missing ELF interpreter for ASan host: {executable}")
    # With mmap_rnd_bits=32, a kernel-loaded PIE can land in ASan 14's
    # 0x600000000000..0x640000000000 allocator reservation and be overwritten
    # before main. Invoking its own ELF loader maps the PIE as a shared object.
    # This retains ASLR, the original binary, and all sanitizer checks.
    return [match[1], *command]


def write_wrapper(path, executable, runtime=None, suppression=None):
    # Python and ROS launch stay uninstrumented. Only exec of the native host
    # receives LD_PRELOAD; all project executables keep their linked runtime.
    policy = (f"env['LSAN_OPTIONS'] = env.get('LSAN_OPTIONS', '') + {(':suppressions=' + str(suppression))!r}\n"
              if suppression else "")
    preload = (f"env['LD_PRELOAD'] = {str(runtime)!r} + (':' + env['LD_PRELOAD'] if env.get('LD_PRELOAD') else '')\n"
               if runtime else "")
    command = preload_command(executable) if runtime else [str(executable)]
    path.write_text("#!/usr/bin/env python3\nimport os, sys\n"
                    f"command = {command!r}\n"
                    "env = dict(os.environ)\n"
                    + preload + policy +
                    "os.execve(command[0], [*command, *sys.argv[1:]], env)\n")
    path.chmod(0o755)


def prepare(underlay, output, compiler, project_prefix):
    hosts = [(underlay, "controller_manager"), (project_prefix, "litesim3d_ros2_control")]
    for prefix, package in hosts:
        host = prefix / "lib" / package / "ros2_control_node"
        if not host.is_file():
            raise RuntimeError(f"Missing controller manager host: {host}")
    runtime = runtime_library(compiler)
    output.mkdir(parents=True, exist_ok=True)
    policy = output / "lsan-controller-host.supp"
    policy.write_text(Path(__file__).with_name("lsan.supp").read_text() + "\n" +
                      Path(__file__).with_name("lsan_controller_host.supp").read_text())
    for prefix, package in hosts:
        binary_dir = output / "lib" / package
        binary_dir.mkdir(parents=True, exist_ok=True)
        marker = output / "share/ament_index/resource_index/packages" / package
        marker.parent.mkdir(parents=True, exist_ok=True)
        marker.touch()
        share = output / "share" / package
        if not share.exists():
            share.symlink_to(prefix / "share" / package, target_is_directory=True)
        for executable in (prefix / "lib" / package).iterdir():
            target = binary_dir / executable.name
            if executable.name == "ros2_control_node":
                # The project host already links ASan. Preloading the shared
                # runtime there would mix two runtimes in the same process.
                write_wrapper(target, executable,
                              runtime if package == "controller_manager" else None, policy)
            elif not target.exists():
                target.symlink_to(executable)
    # Package-prefix lookups also locate plugin libraries. Preserve that layout
    # when the local package marker points into this quality-only overlay.
    for library in (project_prefix / "lib").glob("*.so*"):
        target = output / "lib" / library.name
        if not target.exists():
            target.symlink_to(library)
    return {"hosts": [str(prefix / "lib" / package / "ros2_control_node") for prefix, package in hosts],
            "runtime": str(runtime), "overlay": str(output), "suppression": str(policy),
            "scope": "both ros2_control_node hosts only; spawner and Python unchanged"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler", required=True)
    parser.add_argument("--underlay", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--project-prefix", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(prepare(args.underlay, args.output, args.compiler, args.project_prefix), indent=2))


if __name__ == "__main__":
    main()
