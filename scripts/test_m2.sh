#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
evidence_dir="$(python3 "$project_dir/scripts/paths.py" evidence)"
export PYTHONDONTWRITEBYTECODE=1
prefix="${LWS_INSTALL_PREFIX:-$(python3 "$project_dir/scripts/paths.py" install)}"
set +u
source "$prefix/setup.bash"
set -u
run_dir="$evidence_dir/m2/run-$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -p "$run_dir"
export ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR="$run_dir/ros_logs" LWS_INSTALL_PREFIX="$prefix"
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
status=0
python3 "$project_dir/scripts/checks/run_checked.py" --timeout 300 --xml "$run_dir/core-runner.xml" --log "$run_dir/core.log" -- \
  "$(dirname "$prefix")/build/litesim3d_core/sim_core_tests" --gtest_output="xml:$run_dir/core.xml" || status=1
python3 "$project_dir/scripts/checks/run_checked.py" --timeout 1200 --xml "$run_dir/ros-runner.xml" --log "$run_dir/ros.log" -- \
  python3 -m pytest -q -o cache_dir="$run_dir/pytest-cache" "$project_dir/litesim3d_ros2/test" "$project_dir/scripts/integration/test_repository_tools.py" --basetemp="$run_dir/ros" --junitxml="$run_dir/ros.xml" || status=1
ROS_DOMAIN_ID=5 launch_test "$project_dir/litesim3d_ros2/test/launch_m1.py" --junit-xml "$run_dir/launch.xml" > "$run_dir/launch.log" 2>&1 || status=1
if [[ "${LWS_M2_LONG_TESTS:-0}" == 1 ]]; then
  python3 "$project_dir/scripts/fixtures/generate_m2_fixtures.py" --resources "$prefix/share/litesim3d_core" --output "$run_dir/fixtures"
  python3 "$project_dir/scripts/checks/check_m2_start_stop.py" --config "$prefix/share/litesim3d_core/config/m2.yaml" --output "$run_dir/start-stop" || status=1
  python3 "$project_dir/scripts/benchmarks/benchmark_m2.py" --config "$run_dir/fixtures/F09.yaml" --output "$run_dir/performance" || status=1
  python3 "$project_dir/scripts/benchmarks/benchmark_m2.py" --config "$run_dir/fixtures/F09.yaml" --output "$run_dir/soak" --mode soak || status=1
else
  printf '%s\n' 'Long tests NOT_RUN. Set LWS_M2_LONG_TESTS=1 for performance and the 8-hour wall-clock soak.' > "$run_dir/long-tests.txt"
fi
printf 'M2 evidence: %s\n' "$run_dir"
exit "$status"
