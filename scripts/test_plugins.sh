#!/usr/bin/env bash
# Run after sourcing the ROS 2 environment. No existing install is modified.
set -eo pipefail
plugin_root="$(cd "$(dirname "$0")/.." && pwd)"
plugin_workspace="${1:?usage: test_plugins.sh /absolute/isolated-workspace}"
case "$plugin_workspace" in /*) ;; *) echo 'workspace must be absolute' >&2; exit 2;; esac
plugin_cmake=(-DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo)
if [[ "$(uname -s)" == Darwin && -n "${CONDA_PREFIX:-}" ]]; then
  plugin_cmake+=(-DCMAKE_CXX_COMPILER=/usr/bin/clang++ -DCMAKE_BUILD_WITH_INSTALL_RPATH=ON
    "-DCMAKE_INSTALL_RPATH=$plugin_workspace/install/lib;$CONDA_PREFIX/lib"
    "-DCMAKE_EXE_LINKER_FLAGS=-Wl,-headerpad_max_install_names -Wl,-dead_strip_dylibs -L$CONDA_PREFIX/lib"
    "-DCMAKE_SHARED_LINKER_FLAGS=-Wl,-headerpad_max_install_names -Wl,-dead_strip_dylibs -L$CONDA_PREFIX/lib")
fi
colcon --log-base "$plugin_workspace/log" build \
  --base-paths "$plugin_root/litesim3d_plugin_api" "$plugin_root/litesim3d_core" "$plugin_root/plugins" "$plugin_root/litesim3d_plugin_tests" \
    "$plugin_root/litesim3d_ros2" "$plugin_root/litesim3d_interfaces" \
  --merge-install --build-base "$plugin_workspace/build" --install-base "$plugin_workspace/install" \
  --executor parallel --parallel-workers 2 --cmake-args "${plugin_cmake[@]}"
source "$plugin_workspace/install/setup.bash"
ctest --test-dir "$plugin_workspace/build/litesim3d_plugin_tests" --output-on-failure \
  --output-junit "$plugin_workspace/plugin-contracts.xml"
ctest --test-dir "$plugin_workspace/build/litesim3d_mid360" --output-on-failure \
  --output-junit "$plugin_workspace/mid360-contracts.xml"
