#!/usr/bin/env bash
# Build the third-party controller against an existing install, without rebuilding LiteSim.
set -eo pipefail
controller_root="$(cd "$(dirname "$0")/.." && pwd)"
controller_workspace="${1:?usage: test_external_controller.sh /absolute/addon-workspace /absolute/base-install}"
controller_underlay="${2:?provide the existing LiteSim3D install prefix}"
case "$controller_workspace:$controller_underlay" in /*:/*) ;; *) echo 'both paths must be absolute' >&2; exit 2;; esac
source "$controller_underlay/setup.bash"
python3 - "$controller_root" "$controller_workspace" "$controller_underlay" <<'PY'
from pathlib import Path
import hashlib, json, shutil, sys
root, workspace, prefix = map(Path, sys.argv[1:])
package = 'litesim3d_example_altitude_controller'
shutil.copytree(root / 'examples/external_plugins' / package, workspace / 'src' / package,
                dirs_exist_ok=True, ignore=shutil.ignore_patterns('COLCON_IGNORE'))
files = [prefix / 'lib/litesim3d_ros2/sim_node', prefix / 'lib/liblitesim3d_core.so', prefix / 'lib/liblitesim3d_system.so']
assert all(p.is_file() for p in files), files
(workspace / 'before.json').write_text(json.dumps({str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}, indent=2))
PY
MAKEFLAGS=-j1 CMAKE_BUILD_PARALLEL_LEVEL=1 colcon --log-base "$controller_workspace/log" build \
  --base-paths "$controller_workspace/src" --merge-install --executor sequential \
  --build-base "$controller_workspace/build" --install-base "$controller_workspace/install"
source "$controller_workspace/install/setup.bash"
export LITESIM_EXTERNAL_CONTROLLER_PARAMS="$controller_workspace/install/share/litesim3d_example_altitude_controller/config/controllers.yaml"
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
python3 -m pytest "$controller_root/examples/drone_3d/drone_3d_sim/test/test_ros2_control.py" \
  -k external_controller -v --basetemp="$controller_workspace/test-results"
python3 - "$controller_workspace" <<'PY'
from pathlib import Path
import hashlib, json, sys
workspace = Path(sys.argv[1]); before = json.loads((workspace / 'before.json').read_text())
after = {name: hashlib.sha256(Path(name).read_bytes()).hexdigest() for name in before}
assert before == after, 'simulator or hardware adapter changed during external controller build'
(workspace / 'result.json').write_text(json.dumps({'passed': True, 'underlay_sha256': after,
    'controller': 'litesim3d_example_altitude_controller/AltitudeController'}, indent=2))
PY
