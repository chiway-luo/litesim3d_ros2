"""Cross-package installed example assets: repository integration suite."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET
import importlib.util

import pytest
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from litesim3d_bringup.model import prepare_config


def test_desktop_fixture_imports_explicit_installed_simulation_model(tmp_path):
    script = Path(__file__).resolve().parents[1] / 'checks/check_sdf_viewer.py'
    spec = importlib.util.spec_from_file_location('sdf_viewer_check', script)
    viewer = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(viewer)
    runtime = viewer.prepare_runtime(tmp_path / 'prepared')
    models = Path(get_package_share_directory('wheel_car_4w_models'))
    importer = Path(get_package_prefix('litesim3d_assets')) / 'lib/litesim3d_assets/sdf_import'
    scene_dir = tmp_path / 'scene'
    result = subprocess.run([str(importer), str(models / 'worlds/warehouse.sdf'),
                             str(runtime), str(scene_dir), 'false'],
                            env={**os.environ, 'GZ_SIM_RESOURCE_PATH': str(models / 'models')},
                            capture_output=True, text=True, timeout=60)
    assert result.returncode == 0, result.stdout + result.stderr
    scene = json.loads((scene_dir / 'scene.json').read_text())
    model = Path(scene['robot_model']['source'])
    assert model.is_file()
    assert ET.parse(model).find('./litesim3d/chassis') is not None
    assert scene['visuals']


@pytest.mark.skipif(os.uname().sysname != 'Linux', reason='ELF dependency regression')
def test_hardware_plugin_does_not_link_unrelated_mock_plugins():
    library = Path(get_package_prefix('litesim3d_ros2_control')) / 'lib/liblitesim3d_system.so'
    dependencies = subprocess.check_output(['readelf', '-d', str(library)], text=True)
    # GCC's --as-needed can remove the interface DSO when its used methods are
    # inline. The real manager integration verifies loading; this guards only
    # the unwanted plugin registrations that caused the namespace collision.
    assert 'libmock_components.so' not in dependencies
    assert 'libfake_components.so' not in dependencies


@pytest.mark.parametrize('world,package,models_package,description_package,visual_min', [
    ('office.sdf','wheel_car_4w_sim','wheel_car_4w_models','wheel_car_4w_description',170),
    ('drone_arena.sdf','drone_3d_sim','drone_3d_models','drone_3d_sim',90)])
@pytest.mark.parametrize('budget',[False,True])
def test_complex_examples_import_from_installed_resources(tmp_path,world,package,models_package,description_package,visual_min,budget):
    # Relocate just this example's installed resources. Successful import must
    # not depend on the other example's package, source tree, or texture paths.
    assets=tmp_path/'assets'
    shutil.copytree(get_package_share_directory(models_package),assets)
    runtime=assets/'runtime.yaml'
    sim_share=Path(get_package_share_directory(package)).resolve()
    robot_share=Path(get_package_share_directory(description_package)).resolve()
    shutil.copy(sim_share/'config/runtime.yaml',runtime)
    # Like the example launch, supply the composed model explicitly: runtime
    # deliberately leaves robot.urdf empty for portable package lookup.
    runtime=prepare_config(runtime,assets/'prepared',robot_urdf=str(robot_share/'urdf/simulation.urdf.xacro'))
    prepared=yaml.safe_load(runtime.read_text())
    parameter_files=set()
    if description_package:
        model=ET.parse(prepared['robot']['urdf'])
        for plugin in model.findall('./litesim3d/*'):
            plugin_share=Path(get_package_share_directory(plugin.attrib['class'].split('/')[0])).resolve()
            for parameter in plugin.findall('parameters'):
                path=Path(parameter.attrib['file']).resolve()
                assert any(path.is_relative_to(base) for base in (assets.resolve(),robot_share,sim_share,plugin_share))
                parameter_files.add(path)
    output=tmp_path/'imported'
    tool=Path(get_package_prefix('litesim3d_assets'))/'lib/litesim3d_assets/sdf_import'
    result=subprocess.run([str(tool),str(assets/'worlds'/world),str(runtime),str(output),str(budget).lower()],
        env={**os.environ,'GZ_SIM_RESOURCE_PATH':str(assets/'models')},cwd=tmp_path,capture_output=True,text=True,timeout=30)
    assert result.returncode==0,result.stderr
    scene=json.loads((output/'scene.json').read_text())
    assert len(scene['visuals'])>=visual_min
    assert len(scene['view_pose'])==7
    assert any(path.endswith('checker.png') for path in scene['resources'])
    # Geometry/textures stay in the relocated scene. Composed robots may also
    # reference exact parameter files from their own installed description and
    # plugin packages, never arbitrary files from another example or source tree.
    assert all(Path(path).resolve().is_relative_to(assets.resolve()) or Path(path).resolve() in parameter_files
               for path in scene['resources'])
    assert parameter_files <= {Path(path).resolve() for path in scene['resources']}
    if description_package:
        assert scene['robot_model']['source']==prepared['robot']['urdf']
    cfg=yaml.safe_load((output/'runtime.yaml').read_text())
    assert cfg['simulation']['collision_budget_enabled']==budget
    binary=Path(get_package_prefix('litesim3d_core'))/'bin/sim_headless'
    result=subprocess.run([str(binary),str(output/'runtime.yaml'),'20',str(tmp_path/'manifest.json')],
        capture_output=True,text=True,timeout=30)
    assert result.returncode==0,result.stderr
    report=json.loads((tmp_path/'manifest.json').read_text())
    assert report['status']['contact_count']==0
