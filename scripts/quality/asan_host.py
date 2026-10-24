"""Give an uninstrumented ROS plugin host the matching Clang runtime, locally."""
import argparse
import json
from pathlib import Path
import platform
import subprocess


def runtime_library(compiler):
    arch = {"arm64": "aarch64", "amd64": "x86_64"}.get(platform.machine().lower(), platform.machine().lower())
    path = Path(subprocess.check_output(
        [compiler, f"-print-file-name=libclang_rt.asan-{arch}.so"], text=True).strip())
    if not path.is_absolute() or not path.is_file():
        raise RuntimeError(f"Missing shared ASan runtime for {compiler}: {path}")
    return path.resolve()


def write_wrapper(path, executable, runtime, suppression=None):
    # Python and ROS launch stay uninstrumented. Only exec of the native host
    # receives LD_PRELOAD; all project executables keep their linked runtime.
    policy = (f"env['LSAN_OPTIONS'] = env.get('LSAN_OPTIONS', '') + {(':suppressions=' + str(suppression))!r}\n"
              if suppression else "")
    path.write_text("#!/usr/bin/env python3\nimport os, sys\n"
                    f"host = {str(executable)!r}\nruntime = {str(runtime)!r}\n"
                    "env = dict(os.environ)\n"
                    "env['LD_PRELOAD'] = runtime + (':' + env['LD_PRELOAD'] if env.get('LD_PRELOAD') else '')\n"
                    + policy +
                    "os.execve(host, [host, *sys.argv[1:]], env)\n")
    path.chmod(0o755)


def prepare(underlay, output, compiler):
    package = "controller_manager"
    upstream = underlay / "lib" / package
    host = upstream / "ros2_control_node"
    if not host.is_file():
        raise RuntimeError(f"Missing controller manager host: {host}")
    runtime = runtime_library(compiler)
    binary_dir = output / "lib" / package
    binary_dir.mkdir(parents=True, exist_ok=True)
    policy = output / "lsan-controller-host.supp"
    policy.write_text(Path(__file__).with_name("lsan.supp").read_text() + "\n" +
                      Path(__file__).with_name("lsan_controller_host.supp").read_text())
    marker = output / "share/ament_index/resource_index/packages" / package
    marker.parent.mkdir(parents=True, exist_ok=True)
    marker.touch()
    share = output / "share" / package
    if not share.exists():
        share.symlink_to(underlay / "share" / package, target_is_directory=True)
    for executable in upstream.iterdir():
        target = binary_dir / executable.name
        if executable.name == host.name:
            write_wrapper(target, executable, runtime, policy)
        elif not target.exists():
            target.symlink_to(executable)
    return {"host": str(host), "runtime": str(runtime), "overlay": str(output), "suppression": str(policy),
            "scope": "ros2_control_node only; stock spawner and Python unchanged"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler", required=True)
    parser.add_argument("--underlay", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(prepare(args.underlay, args.output, args.compiler), indent=2))


if __name__ == "__main__":
    main()
