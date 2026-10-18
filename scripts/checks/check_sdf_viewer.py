#!/usr/bin/env python3
"""Render OBJ, DAE and SDF PBR fixtures and exercise native camera input."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix, get_package_share_directory

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from paths import EVIDENCE_DIR


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=EVIDENCE_DIR / "sdf-viewer")
    parser.add_argument("--binary", type=Path,
                        default=Path(get_package_prefix("litesim3d_gui")).parent / "build/litesim3d_gui/viewer_camera_test")
    args=parser.parse_args()
    if args.output.exists() and any(args.output.iterdir()):
        parser.error("output must be a new or empty directory")
    args.output.mkdir(parents=True,exist_ok=True)
    importer=Path(get_package_prefix("litesim3d_assets")) / "lib/litesim3d_assets/sdf_import"
    runtime=Path(get_package_share_directory("wheel_car_4w_sim")) / "config/runtime.yaml"
    result={}
    for variant in ("obj","dae","pbr"):
        folder=args.output / variant
        assets=folder / "assets"
        shutil.copytree(get_package_share_directory("litesim_models"),assets)
        model=assets / "models/textured_crate/model.sdf"
        tree=ET.parse(model)
        if variant=="dae":
            tree.getroot().find("model/link/visual/geometry/mesh/uri").text="meshes/crate.dae"
        if variant=="pbr":
            material=ET.SubElement(tree.getroot().find("model/link/visual"),"material")
            ET.SubElement(material,"diffuse").text="1 1 1 1"
            metal=ET.SubElement(ET.SubElement(material,"pbr"),"metal")
            for key,value in {"albedo_map":"checker.png","normal_map":"normal.png","roughness_map":"roughness.png",
                              "metalness_map":"black.png","emissive_map":"black.png","light_map":"white.png"}.items():
                ET.SubElement(metal,key).text="materials/textures/"+value
        tree.write(model,encoding="unicode")
        env={**os.environ,"GZ_SIM_RESOURCE_PATH":str(assets / "models"),"ROS_LOCALHOST_ONLY":"1","ROS_DOMAIN_ID":"199"}
        plugin=Path(get_package_prefix("litesim3d_gui")) / "lib/litesim3d_gui/ogre2"
        if (plugin / "RenderSystem_Metal.dylib").is_file():
            env["OGRE2_RESOURCE_PATH"]=str(plugin)
        with (folder / "import.log").open("w") as log:
            subprocess.run([str(importer),str(assets / "worlds/warehouse.sdf"),str(runtime),str(folder / "scene"),"false"],
                           env=env,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=60)
        with (folder / "camera.log").open("w") as log:
            test=subprocess.run([str(args.binary),str(folder / "scene/scene.json"),str(folder / "images")],
                                env=env,stdout=log,stderr=subprocess.STDOUT,timeout=60)
        result[variant]="PASS" if test.returncode==0 else "FAIL"
    (args.output / "result.json").write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))
    return int("FAIL" in result.values())


if __name__=="__main__":
    raise SystemExit(main())
