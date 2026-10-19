#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
evidence_dir="$(python3 "$project_dir/scripts/paths.py" evidence)"
export PYTHONDONTWRITEBYTECODE=1
prefix="${LWS_INSTALL_PREFIX:-$(python3 "$project_dir/scripts/paths.py" install)}"
# shellcheck disable=SC1090
set +u
source "$prefix/setup.bash"
set -u
run_dir="$evidence_dir/run-$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -p "$run_dir"
export ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR="$run_dir/ros_logs" LWS_INSTALL_PREFIX="$prefix"
# Humble's auto-loaded launch_testing pytest plugin is incompatible with pytest 9.
# These tests manage real subprocesses directly and need only built-in pytest fixtures.
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
status=0
core_build="$(dirname "$prefix")/build/litesim3d_core"
ctest --test-dir "$core_build" -E M2 --output-on-failure --output-junit "$run_dir/core.xml" > "$run_dir/core.log" 2>&1 || status=1
python3 -m pytest -q -o cache_dir="$run_dir/pytest-cache" "$project_dir/litesim3d_ros2/test/test_ros.py" "$project_dir/litesim3d_ros2/test/test_delivery.py" \
  "$project_dir/scripts/integration/test_repository_tools.py::test_T_URD_18_xacro" \
  --basetemp="$run_dir/ros" --junitxml="$run_dir/ros.xml" > "$run_dir/ros.log" 2>&1 || status=1
ROS_DOMAIN_ID=5 launch_test "$project_dir/litesim3d_ros2/test/launch_m1.py" \
  --junit-xml "$run_dir/launch.xml" > "$run_dir/launch.log" 2>&1 || status=1
printf 'Evidence: %s\n' "$run_dir"
exit "$status"
