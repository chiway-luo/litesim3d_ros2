#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
evidence_dir="$(python3 "$project_dir/scripts/paths.py" evidence)"
export PYTHONDONTWRITEBYTECODE=1
prefix="${LWS_INSTALL_PREFIX:-$(python3 "$project_dir/scripts/paths.py" install)}"
set +u
source "$prefix/setup.bash"
set -u
run_dir="$evidence_dir/m3/run-$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -p "$run_dir"
export ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR="$run_dir/ros_logs" LWS_INSTALL_PREFIX="$prefix"
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
status=0
python3 "$project_dir/scripts/checks/run_checked.py" --timeout 300 --xml "$run_dir/core-runner.xml" --log "$run_dir/core.log" -- \
  "$(dirname "$prefix")/build/litesim3d_core/sim_core_tests" --gtest_output="xml:$run_dir/core.xml" || status=1
python3 "$project_dir/scripts/checks/run_checked.py" --timeout 1200 --xml "$run_dir/ros-runner.xml" --log "$run_dir/ros.log" -- \
  python3 -m pytest -q -o cache_dir="$run_dir/pytest-cache" "$project_dir/litesim3d_ros2/test" --basetemp="$run_dir/ros" --junitxml="$run_dir/ros.xml" || status=1
python3 "$project_dir/scripts/checks/run_checked.py" --timeout 60 --xml "$run_dir/report-runner.xml" --log "$run_dir/report-tests.log" -- \
  python3 -m pytest -q -o cache_dir="$run_dir/pytest-cache" "$project_dir/scripts/reports/test_report_completion.py" "$project_dir/scripts/checks/test_check_coverage.py" --junitxml="$run_dir/report-tests.xml" || status=1
python3 "$project_dir/scripts/checks/run_checked.py" --timeout 180 --xml "$run_dir/wheel-car-runner.xml" --log "$run_dir/wheel-car.log" -- \
  python3 -m pytest -q -o cache_dir="$run_dir/pytest-cache" "$project_dir/examples/wheel_car_4w/wheel_car_4w_sim/test" --basetemp="$run_dir/wheel-car" --junitxml="$run_dir/wheel-car.xml" || status=1
for milestone in m1 m3; do
  ROS_DOMAIN_ID=164 launch_test "$project_dir/litesim3d_ros2/test/launch_$milestone.py" \
    --junit-xml "$run_dir/launch-$milestone.xml" > "$run_dir/launch-$milestone.log" 2>&1 || status=1
done
if [[ "${LWS_M3_PERFORMANCE:-0}" == 1 ]]; then
  python3 "$project_dir/scripts/fixtures/generate_m3_fixtures.py" --resources "$prefix/share/litesim3d_core" --output "$run_dir/fixtures"
  python3 "$project_dir/scripts/benchmarks/benchmark_m3.py" --config "$run_dir/fixtures/B3-world.yaml" \
    --output "$run_dir/performance" --gui off || status=1
fi
printf '%s\n' '8-hour stability: DEFERRED by user; never started by this runner.' > "$run_dir/stability.txt"
printf 'M3 evidence: %s\n' "$run_dir"
exit "$status"
