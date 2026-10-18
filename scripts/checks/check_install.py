#!/usr/bin/env python3
"""Build a disposable source copy, hide it, then run installed artifacts."""
import json
import os
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from paths import REPO_ROOT, EVIDENCE_DIR
import shutil
import subprocess
import tempfile

ROOT = REPO_ROOT
out=Path(os.environ.get("LWS_INSTALL_EVIDENCE",str(EVIDENCE_DIR / "final"))).resolve()
out.mkdir(parents=True,exist_ok=True)
stage=Path(tempfile.mkdtemp(prefix="lws-install-check-",dir="/private/tmp" if os.uname().sysname=="Darwin" else "/tmp"))
project=stage/"project"
shutil.copytree(ROOT,project,ignore=shutil.ignore_patterns("__pycache__", ".pytest_cache", ".git"))
env=os.environ.copy();env["LWS_BUILD_ROOT"]=str(stage/"artifacts")
env["LITESIM3D_ARTIFACTS_DIR"]=str(stage/"artifacts")
env["PYTHONDONTWRITEBYTECODE"]="1"
# A full GUI/SDF build takes over seven minutes on the Ubuntu CI runner.
# Bound the whole process group so a timeout cannot leave compilers running
# during the subsequent integration and graphics checks.
built=subprocess.run([sys.executable,str(ROOT/"scripts/checks/run_checked.py"),
    "--timeout","900","--xml",str(out/"install-build-runner.xml"),
    "--log",str(out/"install-isolated-build.log"),"--",
    "bash",str(project/"scripts/build.sh")],env=env)
if built.returncode:
    (out/"install-check.json").write_text(json.dumps({"status":"FAIL","phase":"build","stage":str(stage),
        "build_exit":built.returncode}))
    raise SystemExit(built.returncode)
prefix=stage/"artifacts/install"
project.rename(stage/"sources-unavailable")
assert not project.exists()
command=['bash','-c','source "$1/setup.bash"; exec launch_test "$2" --junit-xml "$3"',
         'install-check',str(prefix),str(ROOT/"litesim3d_ros2/test/launch_m1.py"),str(out/"install-launch.xml")]
env.update(ROS_DOMAIN_ID="167",ROS_LOCALHOST_ONLY="1",ROS_LOG_DIR=str(out/"install-ros-logs"))
with (out/"install-launch.log").open("w") as log:
    launch=subprocess.run(command,cwd=stage,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=30)
headless=subprocess.run([str(prefix/"bin/sim_headless"),str(prefix/"share/litesim3d_core/config/demo.yaml"),
                         "200",str(out/"install-manifest.json"),".4","0"],cwd=stage,env=env,capture_output=True,text=True,timeout=30)
(out/"install-headless.log").write_text(headless.stdout+headless.stderr)
mesh_command=['bash','-c','source "$1/setup.bash"; export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1; exec "$2" -m pytest -q "$3" --basetemp "$4" --junitxml "$5" -o "cache_dir=$6"',
              'install-mesh',str(prefix),sys.executable,
              str(ROOT/"litesim3d_ros2/test/test_delivery.py")+"::test_T_URD_11_installed_mesh_resources",
              str(out/"install-mesh"),str(out/"install-mesh.xml"),str(out/"pytest-cache")]
with (out/"install-mesh.log").open("w") as log:
    mesh=subprocess.run(mesh_command,cwd=stage,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=30)
ok=launch.returncode==0 and headless.returncode==0 and mesh.returncode==0
(out/"install-check.json").write_text(json.dumps({"status":"PASS" if ok else "FAIL","platform":os.uname().sysname,
    "stage":str(stage),"prefix":str(prefix),"source_hidden":True,"build_exit":built.returncode,
    "launch_exit":launch.returncode,"headless_exit":headless.returncode,"mesh_exit":mesh.returncode},indent=2)+"\n")
print(json.dumps({"status":"PASS" if ok else "FAIL","prefix":str(prefix)}))
raise SystemExit(0 if ok else 1)
