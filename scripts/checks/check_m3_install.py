#!/usr/bin/env python3
"""Fresh native build, hidden source tree, installed M1 + M3 launch and PCL pipeline."""
import json
import os
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from paths import REPO_ROOT, EVIDENCE_DIR
import subprocess

ROOT = REPO_ROOT
out=Path(os.environ.get('LWS_INSTALL_EVIDENCE',str(EVIDENCE_DIR / 'm3/install'))).resolve()
out.mkdir(parents=True,exist_ok=True)
env=os.environ.copy();env['LWS_INSTALL_EVIDENCE']=str(out)
first=subprocess.run([sys.executable,str(ROOT/'scripts/checks/check_install.py')],env=env)
if first.returncode:raise SystemExit(first.returncode)
record=json.loads((out/'install-check.json').read_text());prefix=record['prefix']
def run(name,arguments):
    command=['bash','-c','source "$1/setup.bash"; shift; exec "$@"','m3-install',prefix,*arguments]
    with (out/(name+'.log')).open('w') as log:
        return subprocess.run(command,cwd=record['stage'],env=env,stdout=log,stderr=subprocess.STDOUT,timeout=180).returncode
env.update(ROS_DOMAIN_ID='4',ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR=str(out/'ros_logs'),PYTEST_DISABLE_PLUGIN_AUTOLOAD='1')
record['m3_launch_exit']=run('world-launch',['launch_test',str(ROOT/'litesim3d_ros2/test/launch_m3.py'),
                                           '--junit-xml',str(out/'world-launch.xml')])
record['adapter_exit']=run('pcl',[sys.executable,'-m','pytest','-q',
    str(ROOT/'litesim3d_ros2/test/test_m3_ros.py')+'::test_EXT06_vendor_bytes_nan_timing_and_actual_pcl_read',
    '--basetemp',str(out/'pcl'),'--junitxml',str(out/'pcl.xml'),'-o',f'cache_dir={out / "pytest-cache"}'])
record['sdf_examples_exit']=run('sdf-examples',[sys.executable,'-m','pytest','-q',
    str(ROOT/'scripts/integration/test_example_assets.py')+'::test_complex_examples_import_from_installed_resources',
    str(ROOT/'examples/drone_3d/drone_3d_sim/test/test_drone.py')+'::test_flight_xyz_yaw_sensors_pause_and_watchdog',
    str(ROOT/'examples/drone_3d/drone_3d_sim/test/test_drone.py')+'::test_descent_stops_at_floor_with_budget_disabled',
    '--basetemp',str(out/'sdf-examples'),'--junitxml',str(out/'sdf-examples.xml'),'-o',f'cache_dir={out / "pytest-cache"}'])
record['status']='PASS' if record['m3_launch_exit']==record['adapter_exit']==record['sdf_examples_exit']==0 else 'FAIL'
(out/'install-check.json').write_text(json.dumps(record,indent=2))
print(json.dumps(record,indent=2))
raise SystemExit(int(record['status']!='PASS'))
