"""Cross-package installed example assets: repository integration suite."""
import json
import os
from pathlib import Path
import shutil
import subprocess

import pytest
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory


@pytest.mark.parametrize('world,package,models_package,visual_min', [
    ('office.sdf','wheel_car_4w_sim','wheel_car_4w_models',170),
    ('drone_arena.sdf','drone_3d_sim','drone_3d_models',90)])
@pytest.mark.parametrize('budget',[False,True])
def test_complex_examples_import_from_installed_resources(tmp_path,world,package,models_package,visual_min,budget):
    # Relocate just this example's installed resources. Successful import must
    # not depend on the other example's package, source tree, or texture paths.
    assets=tmp_path/'assets'
    shutil.copytree(get_package_share_directory(models_package),assets)
    runtime=assets/'runtime.yaml'
    shutil.copy(Path(get_package_share_directory(package))/'config/runtime.yaml',runtime)
    output=tmp_path/'imported'
    tool=Path(get_package_prefix('litesim3d_assets'))/'lib/litesim3d_assets/sdf_import'
    result=subprocess.run([str(tool),str(assets/'worlds'/world),str(runtime),str(output),str(budget).lower()],
        env={**os.environ,'GZ_SIM_RESOURCE_PATH':str(assets/'models')},cwd=tmp_path,capture_output=True,text=True,timeout=30)
    assert result.returncode==0,result.stderr
    scene=json.loads((output/'scene.json').read_text())
    assert len(scene['visuals'])>=visual_min
    assert len(scene['view_pose'])==7
    assert any(path.endswith('checker.png') for path in scene['resources'])
    assert all(Path(path).resolve().is_relative_to(assets.resolve()) for path in scene['resources'])
    cfg=yaml.safe_load((output/'runtime.yaml').read_text())
    assert cfg['simulation']['collision_budget_enabled']==budget
    binary=Path(get_package_prefix('litesim3d_core'))/'bin/sim_headless'
    result=subprocess.run([str(binary),str(output/'runtime.yaml'),'20',str(tmp_path/'manifest.json')],
        capture_output=True,text=True,timeout=30)
    assert result.returncode==0,result.stderr
    report=json.loads((tmp_path/'manifest.json').read_text())
    assert report['status']['contact_count']==0
