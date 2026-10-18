import importlib.util
import json
import os
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET

import pytest
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory

ROOT=Path(__file__).resolve().parents[2]


def test_T_URD_18_xacro(tmp_path):
    spec=importlib.util.spec_from_file_location("prepare_xacro",ROOT/"scripts/prepare_xacro.py")
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    share=Path(get_package_share_directory("litesim3d_core"))
    a=module.expand(share/"urdf/r04.xacro",tmp_path/"a.urdf",["length:=0.8"])
    b=module.expand(share/"urdf/r04.xacro",tmp_path/"b.urdf",["length:=1.2"])
    assert a["expanded_sha256"]!=b["expanded_sha256"] and a["includes"]
    assert ET.parse(tmp_path/"a.urdf").find(".//box").get("size").startswith("0.8")
    assert ET.parse(tmp_path/"b.urdf").find(".//box").get("size").startswith("1.2")
    bad=tmp_path/"bad.xacro"
    bad.write_text('<robot xmlns:xacro="http://www.ros.org/wiki/xacro"><xacro:include filename="missing.xacro"/></robot>')
    with pytest.raises(subprocess.CalledProcessError):module.expand(bad,tmp_path/"bad.urdf")
    assert not (tmp_path/"bad.urdf").exists()


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
            assert not Path(path).resolve().is_relative_to(ROOT / "litesim3d_core")


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
