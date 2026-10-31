#!/usr/bin/env bash
set -euo pipefail
# The parent must allow all independently bounded suites to finish. In ASan,
# the old 3600-second aggregate cap was shorter than its child-suite budgets.
if [[ "${1:-}" == --timeout-budget ]]; then
  printf '%s\n' "$(( ${LWS_TEST_CORE_TIMEOUT:-300} + 300 + 60 + ${LWS_TEST_ROS_TIMEOUT:-1200} + 60 + 60 + 6 * ${LWS_TEST_SUITE_TIMEOUT:-180} + 2 * ${LWS_TEST_LAUNCH_SUITE_TIMEOUT:-180} + 60 ))"
  exit 0
fi
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
python3 "$project_dir/scripts/checks/run_checked.py" --timeout "${LWS_TEST_CORE_TIMEOUT:-300}" --xml "$run_dir/core-runner.xml" --log "$run_dir/core.log" -- \
  "$(dirname "$prefix")/build/litesim3d_core/sim_core_tests" --gtest_output="xml:$run_dir/core.xml" || status=1
python3 "$project_dir/scripts/checks/run_checked.py" --timeout 300 --xml "$run_dir/plugins-runner.xml" --log "$run_dir/plugins.log" -- \
  ctest --test-dir "$(dirname "$prefix")/build/litesim3d_plugin_tests" --output-on-failure || status=1
python3 "$project_dir/scripts/checks/run_checked.py" --timeout 60 --xml "$run_dir/control-configuration-runner.xml" --log "$run_dir/control-configuration.log" -- \
  ctest --test-dir "$(dirname "$prefix")/build/litesim3d_ros2_control" -R 'configuration|control_loop_sleep' --output-on-failure || status=1
python3 "$project_dir/scripts/checks/run_checked.py" --timeout "${LWS_TEST_SUITE_TIMEOUT:-180}" --xml "$run_dir/control-hardware-runner.xml" --log "$run_dir/control-hardware.log" -- \
  python3 -m pytest -q -o cache_dir="$run_dir/pytest-cache" "$project_dir/litesim3d_ros2_control/test/test_integration.py" "$project_dir/litesim3d_ros2_control/test/test_spawner.py" "$project_dir/litesim3d_ros2_control/test/test_shutdown.py" \
    --basetemp="$run_dir/control-hardware" --junitxml="$run_dir/control-hardware.xml" || status=1
python3 "$project_dir/scripts/checks/run_checked.py" --timeout "${LWS_TEST_ROS_TIMEOUT:-1200}" --xml "$run_dir/ros-runner.xml" --log "$run_dir/ros.log" -- \
  python3 -m pytest -q -o cache_dir="$run_dir/pytest-cache" "$project_dir/litesim3d_ros2/test" --basetemp="$run_dir/ros" --junitxml="$run_dir/ros.xml" || status=1
python3 "$project_dir/scripts/checks/run_checked.py" --timeout 60 --xml "$run_dir/report-runner.xml" --log "$run_dir/report-tests.log" -- \
  python3 -m pytest -q -o cache_dir="$run_dir/pytest-cache" "$project_dir/scripts/reports/test_report_completion.py" "$project_dir/scripts/checks/test_check_coverage.py" --junitxml="$run_dir/report-tests.xml" || status=1
python3 "$project_dir/scripts/checks/run_checked.py" --timeout "${LWS_TEST_SUITE_TIMEOUT:-180}" --xml "$run_dir/wheel-car-runner.xml" --log "$run_dir/wheel-car.log" -- \
  python3 -m pytest -q -o cache_dir="$run_dir/pytest-cache" "$project_dir/examples/wheel_car_4w/wheel_car_4w_sim/test" --basetemp="$run_dir/wheel-car" --junitxml="$run_dir/wheel-car.xml" || status=1
python3 "$project_dir/scripts/checks/run_checked.py" --timeout "${LWS_TEST_SUITE_TIMEOUT:-180}" --xml "$run_dir/chassis-runner.xml" --log "$run_dir/chassis.log" -- \
  python3 -m pytest -q --import-mode=importlib -o cache_dir="$run_dir/pytest-cache" \
    "$project_dir/examples/ackermann/ackermann_sim/test" "$project_dir/examples/omni3/omni3_sim/test" "$project_dir/examples/omni4/omni4_sim/test" \
    --basetemp="$run_dir/chassis" --junitxml="$run_dir/chassis.xml" || status=1
python3 "$project_dir/scripts/checks/run_checked.py" --timeout 60 --xml "$run_dir/bringup-runner.xml" --log "$run_dir/bringup.log" -- \
  python3 -m pytest -q -o cache_dir="$run_dir/pytest-cache" "$project_dir/litesim3d_bringup/test" \
    --basetemp="$run_dir/bringup" --junitxml="$run_dir/bringup.xml" || status=1
python3 "$project_dir/scripts/checks/run_checked.py" --timeout "${LWS_TEST_SUITE_TIMEOUT:-180}" --xml "$run_dir/sdf-runner.xml" --log "$run_dir/sdf.log" -- \
  python3 -m pytest -q -o cache_dir="$run_dir/pytest-cache" "$project_dir/litesim3d_assets/test" --basetemp="$run_dir/sdf" --junitxml="$run_dir/sdf.xml" || status=1
python3 "$project_dir/scripts/checks/run_checked.py" --timeout "${LWS_TEST_SUITE_TIMEOUT:-180}" --xml "$run_dir/drone-runner.xml" --log "$run_dir/drone.log" -- \
  python3 -m pytest -q -o cache_dir="$run_dir/pytest-cache" "$project_dir/examples/drone_3d/drone_3d_sim/test" --basetemp="$run_dir/drone" --junitxml="$run_dir/drone.xml" || status=1
python3 "$project_dir/scripts/checks/run_checked.py" --timeout "${LWS_TEST_SUITE_TIMEOUT:-180}" --xml "$run_dir/integration-runner.xml" --log "$run_dir/integration.log" -- \
  python3 -m pytest -q -o cache_dir="$run_dir/pytest-cache" "$project_dir/scripts/integration" --basetemp="$run_dir/integration" --junitxml="$run_dir/integration.xml" || status=1
for milestone in m1 m3; do
  ROS_DOMAIN_ID=4 python3 "$project_dir/scripts/checks/run_checked.py" --timeout "${LWS_TEST_LAUNCH_SUITE_TIMEOUT:-180}" \
    --xml "$run_dir/launch-$milestone-runner.xml" --log "$run_dir/launch-$milestone.log" -- \
    launch_test "$project_dir/litesim3d_ros2/test/launch_$milestone.py" \
      --junit-xml "$run_dir/launch-$milestone.xml" || status=1
done
if [[ "${LWS_M3_PERFORMANCE:-0}" == 1 ]]; then
  python3 "$project_dir/scripts/fixtures/generate_m3_fixtures.py" --resources "$prefix/share/litesim3d_core" --output "$run_dir/fixtures"
  python3 "$project_dir/scripts/benchmarks/benchmark_m3.py" --config "$run_dir/fixtures/B3-world.yaml" \
    --output "$run_dir/performance" --gui off || status=1
fi
printf '%s\n' '8-hour stability: DEFERRED by user; never started by this runner.' > "$run_dir/stability.txt"
printf 'M3 evidence: %s\n' "$run_dir"
exit "$status"
