#!/usr/bin/env bash
# Source ROS 2 first. Build only an external package, then run the existing core.
set -eo pipefail
plugin_root="$(cd "$(dirname "$0")/.." && pwd)"
plugin_workspace="${1:?usage: test_external_plugin.sh /absolute/addon-workspace /absolute/base-install}"
plugin_underlay="${2:?provide the existing LiteSim3D install prefix}"
case "$plugin_workspace:$plugin_underlay" in /*:/*) ;; *) echo 'both paths must be absolute' >&2; exit 2;; esac
source "$plugin_underlay/setup.bash"
python3 - "$plugin_root" "$plugin_workspace" "$plugin_underlay" <<'PY'
from pathlib import Path
import hashlib, json, shutil, sys
root, workspace, prefix = map(Path, sys.argv[1:])
workspace.mkdir(parents=True, exist_ok=True)
source = root / 'examples/external_plugins/litesim3d_example_encoder_odometry'
destination = workspace / 'src/litesim3d_example_encoder_odometry'
shutil.copytree(source, destination, dirs_exist_ok=True, ignore=shutil.ignore_patterns('COLCON_IGNORE'))
binary = prefix / 'bin/sim_headless'
runtime = [binary, *sorted(path for pattern in ('liblitesim3d_core.*', 'liblitesim3d_plugin_api.*')
    for path in (prefix / 'lib').glob(pattern) if path.suffix in ('.dylib', '.so') or '.so.' in path.name)]
assert len(runtime) >= 3, 'installed shared core/API libraries are missing'
hashes = {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in runtime}
(workspace / 'core-before.json').write_text(json.dumps({'binary': str(binary), 'sha256': hashes[str(binary)],
    'runtime_sha256': hashes}, indent=2))
PY
plugin_cmake=(-DCMAKE_BUILD_TYPE=RelWithDebInfo)
if [[ "$(uname -s)" == Darwin && -n "${CONDA_PREFIX:-}" ]]; then
  plugin_cmake+=(-DCMAKE_CXX_COMPILER=/usr/bin/clang++ -DCMAKE_BUILD_WITH_INSTALL_RPATH=ON
    "-DCMAKE_INSTALL_RPATH=$plugin_workspace/install/lib;$plugin_underlay/lib;$CONDA_PREFIX/lib"
    "-DCMAKE_SHARED_LINKER_FLAGS=-Wl,-headerpad_max_install_names -Wl,-dead_strip_dylibs -L$CONDA_PREFIX/lib")
fi
colcon --log-base "$plugin_workspace/log" build --base-paths "$plugin_workspace/src" \
  --merge-install --build-base "$plugin_workspace/build" --install-base "$plugin_workspace/install" \
  --cmake-args "${plugin_cmake[@]}"
source "$plugin_workspace/install/setup.bash"
"$plugin_underlay/lib/litesim3d_plugin_tests/plugin_odometry_probe" \
  litesim3d_example_encoder_odometry/ExampleEncoderOdometry \
  "$plugin_workspace/install/share/litesim3d_example_encoder_odometry/config/parameters.yaml" \
  > "$plugin_workspace/parameter-contract.json"
python3 - "$plugin_workspace" <<'PY'
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import sys
workspace = Path(sys.argv[1])
model = Path(get_package_share_directory('litesim3d_core')) / 'urdf/r01.urdf'
xml = model.read_text().replace('</robot>', '''<litesim3d version="1">
  <chassis name="drive" class="litesim3d_diff_drive/DifferentialDrive">
    <parameters file="package://litesim3d_diff_drive/config/parameters.yaml"/>
    <overrides>limits: {unlimited_acceleration: true}</overrides>
  </chassis>
  <odometry name="external_odom" class="litesim3d_example_encoder_odometry/ExampleEncoderOdometry">
    <parameters file="package://litesim3d_example_encoder_odometry/config/parameters.yaml"/>
    <overrides>distance_scale: 1.25</overrides>
  </odometry>
</litesim3d></robot>''')
(workspace / 'composed.urdf').write_text(xml)
(workspace / 'runtime.yaml').write_text('schema_version: 1\nunits: m_rad_s\nrobot: {urdf: composed.urdf}\nlidars: []\nentities: []\n')
PY
"$plugin_underlay/bin/sim_headless" "$plugin_workspace/runtime.yaml" 200 "$plugin_workspace/run.json" 1 0 0
python3 - "$plugin_workspace" <<'PY'
from pathlib import Path
import hashlib, json, sys
workspace = Path(sys.argv[1])
before = json.loads((workspace / 'core-before.json').read_text())
after = hashlib.sha256(Path(before['binary']).read_bytes()).hexdigest()
run = json.loads((workspace / 'run.json').read_text())
contract = json.loads((workspace / 'parameter-contract.json').read_text())
assert contract['parameter_file_complete'] and abs(contract['x_m'] - 1.0) < 1e-9
truth = run['final_pose']['xyz'][0]
odom = run['final_odom_pose']['xyz'][0]
runtime_after = {path: hashlib.sha256(Path(path).read_bytes()).hexdigest() for path in before['runtime_sha256']}
assert before['runtime_sha256'] == runtime_after, 'existing core executable or shared library changed during external package build'
assert abs(truth - 1.0) < 1e-9, truth
assert abs(odom - 1.25) < 1e-9, odom
result = {'passed': True, 'core_binary_sha256_before': before['sha256'], 'core_binary_sha256_after': after,
          'external_parameter_contract': contract,
          'core_runtime_sha256_before': before['runtime_sha256'], 'core_runtime_sha256_after': runtime_after,
          'truth_x_m': truth, 'external_odom_x_m': odom, 'plugin': run['config']['plugins'][1]['class']}
(workspace / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
print(json.dumps(result, indent=2))
PY
