"""Installed native commands, real Xacro and compatibility entry points."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

import pytest
import yaml
from ament_index_python.packages import get_package_prefix


def executable(name):
    return str(Path(get_package_prefix('litesim3d_bringup')) / 'lib/litesim3d_bringup' / name)


def run(name, *args, **kwargs):
    return subprocess.run([executable(name), *map(str, args)], text=True, capture_output=True, timeout=30, **kwargs)


def test_prepare_config_returns_only_machine_readable_result_and_protects_sources(tmp_path):
    source = tmp_path / 'runtime.yaml'; source.write_text('robot: {}\n')
    result = run('prepare_config', source, tmp_path / 'output')
    assert result.returncode == 0, result.stderr
    config = Path(json.loads(result.stdout)['config'])
    assert config == tmp_path / 'output/model/config.yaml'
    assert yaml.safe_load(config.read_text()) == {'robot': {}}
    assert source.read_text() == 'robot: {}\n'
    result = run('prepare_config', source, tmp_path / 'output')
    assert result.returncode != 0 and 'must be empty' in result.stderr and not result.stdout
    from litesim3d_bringup.model import prepare_config
    with pytest.raises(ValueError, match='source directory'):
        prepare_config(source, tmp_path)
    assert source.read_text() == 'robot: {}\n'


def test_prepare_importer_preserves_argv_environment_and_json_boundary(tmp_path, monkeypatch):
    prefix = tmp_path / 'overlay'
    marker = prefix / 'share/ament_index/resource_index/packages/litesim3d_assets'
    marker.parent.mkdir(parents=True); marker.touch()
    tool = prefix / 'lib/litesim3d_assets/sdf_import'; tool.parent.mkdir(parents=True)
    captured = tmp_path / 'import.json'
    tool.write_text('#!/usr/bin/env python3\nimport json,os,sys\nfrom pathlib import Path\n'
                    f'Path({str(captured)!r}).write_text(json.dumps([sys.argv[1:], os.environ["GZ_SIM_RESOURCE_PATH"]]))\n'
                    'print("importer status goes to stderr")\n')
    tool.chmod(0o755)
    monkeypatch.setenv('AMENT_PREFIX_PATH', str(prefix) + os.pathsep + os.environ.get('AMENT_PREFIX_PATH', ''))
    monkeypatch.setenv('GZ_SIM_RESOURCE_PATH', '/existing')
    source = tmp_path / 'source.yaml'; source.write_text('robot: {}')
    result = run('prepare_config', source, tmp_path / 'out', '--world', tmp_path / 'world with space.sdf',
                 '--assets-root', tmp_path / 'assets', '--real-time-factor', '0', '--collision-budget-enabled', 'true')
    assert result.returncode == 0, result.stderr
    assert json.loads(result.stdout)['scene_description'] == str(tmp_path / 'out/scene/scene.json')
    arguments, resources = json.loads(captured.read_text())
    assert arguments[-2:] == ['true', '0']
    assert arguments[0] == str(tmp_path / 'world with space.sdf')
    assert resources == f'{tmp_path}/assets/models:{tmp_path}/assets/worlds:/existing'
    assert 'importer status' in result.stderr


def test_native_xacro_provenance_with_spaces_and_failure_preserves_previous_output(tmp_path):
    include = tmp_path / 'included with space.xacro'; include.write_text('<robot><link name="base_link"/></robot>')
    mesh = tmp_path / 'mesh space.stl'; mesh.write_text('mesh bytes')
    source = tmp_path / 'source.xacro'
    source.write_text(f'<robot xmlns:xacro="http://www.ros.org/wiki/xacro"><xacro:include filename="{include}"/>'
                      f'<link name="body"><visual><geometry><mesh filename="{mesh.name}"/></geometry></visual></link></robot>')
    output = tmp_path / 'generated/model.urdf'
    result = run('prepare_xacro', source, output)
    assert result.returncode == 0, result.stderr
    provenance = json.loads(output.with_suffix('.urdf.json').read_text())
    assert provenance['source_sha256'] == hashlib.sha256(source.read_bytes()).hexdigest()
    assert provenance['includes'][str(include)] == hashlib.sha256(include.read_bytes()).hexdigest()
    assert provenance['resources'][str(mesh)] == hashlib.sha256(mesh.read_bytes()).hexdigest()
    assert provenance['expanded_sha256'] == hashlib.sha256(output.read_bytes()).hexdigest()
    before = output.read_bytes(), output.with_suffix('.urdf.json').read_bytes()
    mesh.unlink()
    result = run('prepare_xacro', source, output)
    assert result.returncode != 0
    assert before == (output.read_bytes(), output.with_suffix('.urdf.json').read_bytes())


def test_rsp_native_parameters_and_legacy_command_match(tmp_path):
    model = tmp_path / 'model.urdf'; model.write_text('<robot name="r"><link name="base_link"/></robot>')
    config = tmp_path / 'config.yaml'; config.write_text('robot: {urdf: model.urdf}')
    output = tmp_path / 'rsp.yaml'
    result = run('make_rsp_params', config, output, '--frame-prefix', 'r0/')
    assert result.returncode == 0, result.stderr
    params = yaml.safe_load(output.read_text())['/**']['ros__parameters']
    assert params['use_sim_time'] is True and params['frame_prefix'] == 'r0/'
    assert ET.fromstring(params['robot_description']).get('name') == 'r'
    assert run('make_rsp_params', config, output, '--frame-prefix', '/absolute/').returncode != 0
    script = Path(__file__).resolve().parents[2] / 'docs/examples/make_rsp_params.py'
    result = subprocess.run([sys.executable, str(script), str(config), str(output), '--frame-prefix', 'r0/'],
                            text=True, capture_output=True, timeout=30)
    assert result.returncode == 0, result.stderr
    assert yaml.safe_load(output.read_text())['/**']['ros__parameters'] == params


@pytest.mark.parametrize('name', ['prepare_config', 'prepare_xacro', 'make_rsp_params'])
def test_native_help_and_invalid_arguments(name):
    assert run(name, '--help').returncode == 0
    assert run(name).returncode != 0
