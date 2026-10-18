#!/usr/bin/env bash
set -euo pipefail
# Run from a real desktop session with the project's Ogre backend available.
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
prefix="${LWS_INSTALL_PREFIX:-$(python3 "$project_dir/scripts/paths.py" install)}"
set +u
source "$prefix/setup.bash"
set -u
run_dir="$(python3 "$project_dir/scripts/paths.py" evidence)/desktop/run-$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -p "$run_dir"
export LWS_GUI_TESTS=1 LWS_4W_GUI=1 ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
python3 "$project_dir/scripts/checks/run_checked.py" --timeout 300 --xml "$run_dir/runner.xml" --log "$run_dir/desktop.log" -- \
  python3 -m pytest -q -o cache_dir="$run_dir/pytest-cache" \
  "$project_dir/examples/wheel_car_4w/wheel_car_4w_sim/test/test_cameras.py" \
  "$project_dir/examples/wheel_car_4w/wheel_car_4w_sim/test/test_sdf_runtime.py::test_viewer_only_subscribes_and_can_close_without_stopping_simulation" \
  "$project_dir/examples/wheel_car_4w/wheel_car_4w_sim/test/test_wheel_car_4w.py::test_rviz_starts_and_consumes_installed_model" \
  "$project_dir/examples/drone_3d/drone_3d_sim/test/test_drone.py" \
  --basetemp="$run_dir/cases" --junitxml="$run_dir/desktop.xml"
printf 'Desktop evidence: %s\n' "$run_dir"
