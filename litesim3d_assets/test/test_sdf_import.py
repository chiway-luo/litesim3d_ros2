"""Installed SDF import contracts: resource relocation, semantic frames and fidelity."""
import copy
import json
import os
from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET

import pytest
import yaml
from ament_index_python.packages import get_package_prefix


@pytest.fixture
def scene(tmp_path):
    models = tmp_path / "assets"
    shutil.copytree(Path(__file__).parent / "fixtures/assets", models)
    runtime = tmp_path / "bindings.yaml"
    shutil.copy(Path(__file__).parent / "fixtures/runtime.yaml", runtime)
    return models, runtime


def run(scene, tmp_path, budget=False):
    assets, runtime = scene
    output = tmp_path / ("enabled" if budget else "disabled")
    env = {**os.environ, "GZ_SIM_RESOURCE_PATH": str(assets / "models")}
    tool = Path(get_package_prefix("litesim3d_assets")) / "lib/litesim3d_assets/sdf_import"
    result = subprocess.run([str(tool), str(assets / "worlds/warehouse.sdf"), str(runtime),
                             str(output), str(budget).lower()], cwd=tmp_path, env=env,
                            text=True, capture_output=True, timeout=30)
    return result, output


def alter(path, callback):
    tree = ET.parse(path)
    callback(tree.getroot())
    tree.write(path, encoding="unicode")


def test_camera_description_keeps_optical_binding_and_disabled_default(scene, tmp_path):
    result, output = run(scene, tmp_path)
    assert result.returncode == 0, result.stderr
    data = json.loads((output / "scene.json").read_text())
    camera = data["cameras"][0]
    assert camera["enabled"] is False and camera["type"] == "rgb"
    assert (camera["width"], camera["height"], camera["rate_hz"]) == (640, 480, 15)
    assert data["initial_link_poses"][camera["link"]][:3] == pytest.approx([.4, 0, .375])
    # q and -q describe the same rotation; compare their absolute dot product.
    quaternion=data["initial_link_poses"][camera["link"]][3:]
    assert abs(sum(a*b for a,b in zip(quaternion,[-.5,.5,-.5,.5]))) == pytest.approx(1)


def test_camera_omitted_type_defaults_to_rgb(scene, tmp_path):
    data = yaml.safe_load(scene[1].read_text())
    del data["cameras"][0]["type"]
    data["cameras"][0]["enabled"] = True
    scene[1].write_text(yaml.safe_dump(data))
    result, output = run(scene, tmp_path)
    assert result.returncode == 0, result.stderr
    camera = json.loads((output / "scene.json").read_text())["cameras"][0]
    assert camera["enabled"] is True and camera["type"] == "rgb"


@pytest.mark.parametrize("field,value", [("rate_hz",0), ("width",0), ("type","stereo"),
    ("link","missing_optical"), ("near_clip_m",20), ("unexpected_parameter",True)])
def test_camera_invalid_configuration_fails_before_launch(scene, tmp_path, field, value):
    data=yaml.safe_load(scene[1].read_text());data["cameras"][0][field]=value
    scene[1].write_text(yaml.safe_dump(data))
    result, _ = run(scene, tmp_path)
    assert result.returncode != 0 and ("camera" in result.stderr.lower()), result.stderr


def test_standalone_rejects_enabled_camera_instead_of_silently_omitting_images(scene, tmp_path):
    data=yaml.safe_load(scene[1].read_text());data["cameras"][0]["enabled"]=True
    scene[1].write_text(yaml.safe_dump(data))
    result, output=run(scene,tmp_path)
    assert result.returncode==0,result.stderr
    binary=Path(get_package_prefix("litesim3d_core"))/"bin/sim_headless"
    result=subprocess.run([str(binary),str(output/"runtime.yaml"),"20",str(tmp_path/"report.json")],
        capture_output=True,text=True,timeout=10)
    assert result.returncode!=0 and "has no renderer" in result.stderr


def test_installed_scene_keeps_mesh_material_slots_and_resource_manifest(scene, tmp_path):
    result, output = run(scene, tmp_path, True)
    assert result.returncode == 0, result.stderr
    data = json.loads((output / "scene.json").read_text())
    crate = next(v for v in data["visuals"] if "textured_crate" in v["id"])
    assert Path(crate["geometry"]["uri"]).is_file()
    assert "diffuse" not in crate["material"]  # preserve the OBJ/MTL's two slots
    assert any(p.endswith("checker.png") for p in data["resources"])
    assert any(p.endswith("crate.mtl") for p in data["resources"])
    assert data["initial_link_poses"]["lidar_link"][:3] == pytest.approx([.12, 0, .49])
    generated = yaml.safe_load((output / "runtime.yaml").read_text())
    assert generated["simulation"]["collision_budget_enabled"] is True
    assert len([v for v in generated["entities"] if "textured_crate" in v["id"] and "collision" in v]) == 1


def test_nested_static_frames_resolve_through_attached_to(scene, tmp_path):
    path = scene[0] / "worlds/warehouse.sdf"
    extra = ET.fromstring('''<model name="outer"><static>true</static><pose>10 0 0 0 0 0</pose>
      <model name="inner"><pose>2 0 0 0 0 0</pose>
        <frame name="anchor" attached_to="__model__"><pose>1 3 0 0 0 0</pose></frame>
        <link name="part"><pose relative_to="anchor">0 0 1 0 0 0</pose>
          <visual name="display"><pose>0 2 0 0 0 0</pose><geometry><box><size>1 1 1</size></box></geometry></visual>
        </link></model></model>''')
    alter(path, lambda root: root.find("world").append(extra))
    result, output = run(scene, tmp_path)
    assert result.returncode == 0, result.stderr
    data = json.loads((output / "scene.json").read_text())
    item = next(v for v in data["visuals"] if v["id"] == "outer::inner/part/display")
    assert item["pose"][:3] == pytest.approx([13, 5, 1])


def test_joint_axis_expressed_in_resolves_in_generated_joint_frame(scene, tmp_path):
    path = scene[0] / "models/wheel_car_4w/model.sdf"
    def add(root):
        model = root.find("model")
        for xml in ['<frame name="rotated" attached_to="base_link"><pose>0 0 0 0 0 1.5707963267948966</pose></frame>',
                    '<link name="test_tip"><pose relative_to="base_link">0 0 .7 0 0 0</pose></link>',
                    '<joint name="test_axis" type="revolute"><parent>base_link</parent><child>test_tip</child><axis><xyz expressed_in="rotated">1 0 0</xyz><limit><lower>-1</lower><upper>1</upper><velocity>1</velocity></limit></axis></joint>']:
            model.append(ET.fromstring(xml))
    alter(path, add)
    result, output = run(scene, tmp_path)
    assert result.returncode == 0, result.stderr
    axis = ET.parse(output / "robot.urdf").find("joint[@name='test_axis']/axis").get("xyz")
    assert list(map(float, axis.split())) == pytest.approx([0, 1, 0], abs=1e-10)


def test_missing_texture_fails_instead_of_gray_fallback(scene, tmp_path):
    (scene[0] / "models/textured_crate/materials/textures/checker.png").unlink()
    result, output = run(scene, tmp_path)
    assert result.returncode != 0 and "missing resource" in result.stderr
    assert not (output / "scene.json").exists()


@pytest.mark.parametrize("xml,expected", [
    ('<plugin name="ignored" filename="not_supported.so"/>', "unsupported SDF"),
    ('<frame name="broken" attached_to="missing"/>', "missing"),
])
def test_unsupported_behavior_and_broken_frames_fail(scene, tmp_path, xml, expected):
    alter(scene[0] / "models/ground/model.sdf", lambda root: root.find("model").append(ET.fromstring(xml)))
    result, _ = run(scene, tmp_path)
    assert result.returncode != 0 and expected in result.stderr


def test_classic_material_scripts_are_explicitly_rejected(scene, tmp_path):
    def add(root):
        material = root.find("model/link/visual/material")
        material.append(ET.fromstring('<script><uri>file://missing.material</uri><name>Custom/Shader</name></script>'))
    alter(scene[0] / "models/ground/model.sdf", add)
    result, _ = run(scene, tmp_path)
    assert result.returncode != 0 and "<script>" in result.stderr


def dense_mesh(scene, collision):
    base = scene[0] / "models/textured_crate"
    (base / "meshes/dense.obj").write_text("v 0 0 0\nv 0.2 0 0\nv 0 0.2 0\n" + "f 1 2 3\n" * 10002)
    def change(root):
        root.find("model/link/visual/geometry/mesh/uri").text = "meshes/dense.obj"
        if collision:
            target = root.find("model/link/collision/geometry")
            target.clear()
            target.append(ET.fromstring('<mesh><uri>meshes/dense.obj</uri></mesh>'))
    alter(base / "model.sdf", change)


def test_visual_complexity_is_not_charged_to_collision_budget(scene, tmp_path):
    dense_mesh(scene, False)
    result, _ = run(scene, tmp_path, True)
    assert result.returncode == 0, result.stderr


def test_large_collision_mesh_allowed_when_budget_off_rejected_when_on(scene, tmp_path):
    dense_mesh(scene, True)
    result, _ = run(scene, tmp_path, False)
    assert result.returncode == 0, result.stderr
    result, _ = run(scene, tmp_path, True)
    assert result.returncode != 0 and "collision budget" in result.stderr


def test_many_collision_shapes_respects_toggle(scene, tmp_path):
    def change(root):
        link = root.find("model/link[@name='base_link']")
        original = link.find("collision")
        for i in range(65):
            extra = copy.deepcopy(original)
            extra.set("name", "extra_" + str(i))
            link.append(extra)
    alter(scene[0] / "models/wheel_car_4w/model.sdf", change)
    result, _ = run(scene, tmp_path, False)
    assert result.returncode == 0, result.stderr
    result, _ = run(scene, tmp_path, True)
    assert result.returncode != 0 and "64 shapes" in result.stderr


def test_source_geometry_cannot_be_overridden_by_runtime_yaml(scene, tmp_path):
    data = yaml.safe_load(scene[1].read_text())
    data["robot"]["initial_pose"] = {"xyz": [100, 0, 0]}
    scene[1].write_text(yaml.safe_dump(data))
    result, _ = run(scene, tmp_path)
    assert result.returncode != 0 and "duplicate SDF" in result.stderr


def test_sdf_timing_and_scene_colors_are_authoritative(scene, tmp_path):
    def change(root):
        root.find("world/physics/max_step_size").text="0.01"
        root.find("world/physics/real_time_factor").text="0.5"
        root.find("world/scene/ambient").text="0.2 0.3 0.4 1"
    alter(scene[0] / "worlds/warehouse.sdf", change)
    result, output=run(scene,tmp_path)
    assert result.returncode==0,result.stderr
    runtime=yaml.safe_load((output / "runtime.yaml").read_text())
    assert runtime["simulation"]["step_size"]==.01
    assert runtime["simulation"]["real_time_factor"]==.5
    assert json.loads((output / "scene.json").read_text())["ambient"]==pytest.approx([.2,.3,.4,1])


def test_misspelled_entity_binding_is_not_silently_ignored(scene, tmp_path):
    data=yaml.safe_load(scene[1].read_text())
    data["entity_bindings"]["missing_ground"]={"support":True}
    scene[1].write_text(yaml.safe_dump(data))
    result,_=run(scene,tmp_path)
    assert result.returncode!=0 and "unknown entity binding" in result.stderr


def test_existing_output_is_never_overwritten(scene, tmp_path):
    result,output=run(scene,tmp_path)
    assert result.returncode==0,result.stderr
    original=(output / "scene.json").read_bytes()
    result,_=run(scene,tmp_path)
    assert result.returncode!=0 and "directory must be empty" in result.stderr
    assert (output / "scene.json").read_bytes()==original


def test_dae_multi_material_texture_resources(scene, tmp_path):
    def change(root):
        root.find("model/link/visual/geometry/mesh/uri").text="meshes/crate.dae"
    alter(scene[0] / "models/textured_crate/model.sdf",change)
    result,output=run(scene,tmp_path)
    assert result.returncode==0,result.stderr
    resources=json.loads((output / "scene.json").read_text())["resources"]
    assert any(p.endswith("crate.dae") for p in resources)
    assert any(p.endswith("checker.png") for p in resources)


def test_ambiguous_dae_up_axis_is_rejected(scene, tmp_path):
    def change(root):
        root.find("model/link/visual/geometry/mesh/uri").text="meshes/crate.dae"
    alter(scene[0] / "models/textured_crate/model.sdf",change)
    dae=scene[0] / "models/textured_crate/meshes/crate.dae"
    dae.write_text(dae.read_text().replace("Z_UP","Y_UP"))
    result,_=run(scene,tmp_path)
    assert result.returncode!=0 and "Z_UP" in result.stderr


def test_pbr_maps_are_preserved_and_manifested(scene, tmp_path):
    maps={"albedo_map":"checker.png","normal_map":"normal.png","roughness_map":"roughness.png",
          "metalness_map":"black.png","emissive_map":"black.png","light_map":"white.png"}
    material=ET.Element("material")
    ET.SubElement(material,"diffuse").text="1 1 1 1"
    metal=ET.SubElement(ET.SubElement(material,"pbr"),"metal")
    for key,value in maps.items():
        ET.SubElement(metal,key).text="materials/textures/"+value
    alter(scene[0] / "models/textured_crate/model.sdf",lambda r:r.find("model/link/visual").append(material))
    result,output=run(scene,tmp_path)
    assert result.returncode==0,result.stderr
    data=json.loads((output / "scene.json").read_text())
    item=next(v for v in data["visuals"] if "textured_crate" in v["id"])
    for key,value in maps.items():
        assert item["material"][key].endswith(value)
        assert item["material"][key] in data["resources"]
