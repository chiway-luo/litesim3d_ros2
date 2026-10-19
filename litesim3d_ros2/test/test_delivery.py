import json
from pathlib import Path
import subprocess
import time

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory



def test_T_URD_11_installed_mesh_resources(tmp_path):
    share=Path(get_package_share_directory("litesim3d_core"))
    exe=Path(get_package_prefix("litesim3d_core"))/"bin/sim_headless"
    for ext in ("obj","stl","dae"):
        config=yaml.safe_load((share/"config/demo.yaml").read_text())
        config["robot"]["urdf"]=f"package://litesim3d_core/urdf/r02_{ext}.urdf"
        config["lidars"][0]["columns"]=8
        cfg=tmp_path/f"{ext}.yaml";cfg.write_text(yaml.safe_dump(config))
        output=tmp_path/f"{ext}.json"
        run=subprocess.run([str(exe),str(cfg),"20",str(output)],cwd=tmp_path,capture_output=True,text=True,timeout=20)
        assert run.returncode==0,run.stderr
        manifest=json.loads(output.read_text())
        assert manifest["frames"]==1
        assert manifest["model"]["resources"]
        for path in manifest["model"]["resources"]:
            assert Path(path).is_relative_to(share)


def test_T_COR_11_actual_pacing(tmp_path):
    share=Path(get_package_share_directory("litesim3d_core"))
    exe=Path(get_package_prefix("litesim3d_core"))/"bin/sim_headless"
    results=[];durations=[]
    for rtf in (.5,1,0):
        path=tmp_path/f"pacing-{rtf}.json";start=time.monotonic()
        run=subprocess.run([str(exe),str(share/"config/demo.yaml"),"100",str(path),".2",".1",str(rtf)],
                           capture_output=True,text=True,cwd=tmp_path,timeout=20)
        assert run.returncode==0,run.stderr
        durations.append(time.monotonic()-start);results.append(json.loads(path.read_text()))
    assert all(r["final_pose"]==results[0]["final_pose"] for r in results)
    assert [r["frames"] for r in results]==[5,5,5]
    assert durations[0] >= 1 and durations[1] >= .5
    (tmp_path/"pacing.json").write_text(json.dumps(durations))
