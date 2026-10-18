#!/usr/bin/env bash
set -euo pipefail
# Activate your ROS 2 Humble environment before running this script.
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if ! command -v colcon >/dev/null; then
  echo 'Activate ROS 2 Humble first (Ubuntu: source /opt/ros/humble/setup.bash).' >&2
  exit 1
fi
artifact_dir="$(python3 "$project_dir/scripts/paths.py" artifacts)"
evidence_dir="$(python3 "$project_dir/scripts/paths.py" evidence)"
export LITESIM3D_ARTIFACTS_DIR="$artifact_dir" PYTHONDONTWRITEBYTECODE=1
# Humble rosidl reads its IDL list with CMake file(STRINGS); an ASCII build
# directory avoids corruption of Unicode path components without patching ROS.
task_key="$(python3 -c 'import hashlib,sys;print(hashlib.sha256(sys.argv[1].encode()).hexdigest()[:12])' "$project_dir")"
artifact_root="${LWS_BUILD_ROOT:-/private/tmp/litesim3d-$task_key}"
if [[ "$(uname -s)" != Darwin ]]; then
  artifact_root="${LWS_BUILD_ROOT:-/tmp/litesim3d-$task_key}"
fi
mkdir -p "$artifact_root" "$evidence_dir"
touch "$artifact_dir/COLCON_IGNORE"
platform_args=()
if [[ "$(uname -s)" == Darwin ]]; then
  # The bundled conda linker can be too old for the installed Xcode SDK format.
  # Select the installed Xcode compiler for this project only.
  platform_args=(-DCMAKE_C_COMPILER=/usr/bin/clang -DCMAKE_CXX_COMPILER=/usr/bin/clang++
                 "-DCMAKE_OSX_SYSROOT=$(xcrun --show-sdk-path)")
fi
colcon --log-base "$artifact_root/log" build --base-paths "$project_dir" \
  --build-base "$artifact_root/build" --install-base "$artifact_root/install" --merge-install \
  --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON "${platform_args[@]}" "$@"
python3 - "$artifact_dir" "$artifact_root" <<'PY'
from pathlib import Path
import sys
artifacts, runtime = map(lambda value: Path(value).resolve(), sys.argv[1:])
pairs = [(artifacts / name, runtime / name) for name in ("build", "install", "log")]
for link, target in pairs:
    if link != target and link.exists() and not link.is_symlink():
        raise SystemExit(f"Refusing to replace existing artifact directory: {link}")
for link, target in pairs:
    if link == target or (link.is_symlink() and link.resolve() == target):
        continue
    if link.is_symlink():
        link.unlink()
    link.symlink_to(target, target_is_directory=True)
(artifacts / "evidence/install-prefix.txt").write_text(str(runtime / "install") + "\n")
(artifacts / "build-root.txt").write_text(str(runtime) + "\n")
PY
printf 'Build complete. Source: %s/install/setup.bash\n' "$artifact_root"
