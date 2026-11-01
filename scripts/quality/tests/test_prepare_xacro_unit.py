"""Xacro orchestration units with a mocked executable boundary."""
import hashlib
import importlib.util
import json
from pathlib import Path
import runpy
import subprocess
import sys
from types import SimpleNamespace as NS
from unittest.mock import Mock
import xml.etree.ElementTree as ET

import pytest


SOURCE = Path(__file__).resolve().parents[2] / 'prepare_xacro.py'


@pytest.fixture
def module():
    spec = importlib.util.spec_from_file_location('prepare_xacro_unit', SOURCE)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def inputs(tmp_path):
    source = tmp_path / 'robot.xacro'
    source.write_text('<robot name="unit"/>')
    include = tmp_path / 'included.xacro'
    include.write_text('<robot/>')
    resource = tmp_path / 'resources'
    resource.mkdir()
    (resource / 'mesh.stl').write_bytes(b'unit mesh bytes\x00')
    return source, include, resource


def test_digest_reads_exact_bytes(module, tmp_path):
    path = tmp_path / 'bytes'
    path.write_bytes(b'abc')
    assert module.digest(path) == 'ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad'


def test_expand_rebases_only_local_meshes_and_records_exact_provenance(module, monkeypatch, tmp_path):
    source, include, base = inputs(tmp_path)
    output = tmp_path / 'generated/robot.urdf'
    xml = '<robot><mesh filename="mesh.stl"/><mesh filename="package://pkg/mesh.stl"/><mesh filename="mesh.stl"/></robot>'
    run = Mock(side_effect=[NS(stdout=xml), NS(stdout=str(include))])
    monkeypatch.setattr(module.subprocess, 'run', run)
    provenance = module.expand(source, output, ['wheel:=4', 'name:=unit'], resource_base=base)
    assert [call.args[0] for call in run.call_args_list] == [
        ['xacro', str(source), 'wheel:=4', 'name:=unit'],
        ['xacro', '--deps', str(source), 'wheel:=4', 'name:=unit']]
    assert all(call.kwargs == {'capture_output': True, 'text': True, 'check': True, 'timeout': 30}
               for call in run.call_args_list)
    meshes = ET.parse(output).findall('.//mesh')
    assert [mesh.get('filename') for mesh in meshes] == [str(base / 'mesh.stl'), 'package://pkg/mesh.stl', str(base / 'mesh.stl')]
    assert provenance == json.loads(output.with_suffix('.urdf.json').read_text())
    assert provenance['source'] == str(source)
    assert provenance['source_sha256'] == hashlib.sha256(source.read_bytes()).hexdigest()
    assert provenance['resource_base'] == str(base)
    assert provenance['mappings'] == ['wheel:=4', 'name:=unit']
    assert provenance['includes'] == {str(include): hashlib.sha256(include.read_bytes()).hexdigest()}
    assert provenance['resources'] == {str(base / 'mesh.stl'): hashlib.sha256((base / 'mesh.stl').read_bytes()).hexdigest()}
    assert provenance['expanded_sha256'] == hashlib.sha256(output.read_bytes()).hexdigest()
    assert set(output.parent.iterdir()) == {output, output.with_suffix('.urdf.json')}


def test_expand_defaults_to_source_parent_without_local_meshes(module, monkeypatch, tmp_path):
    source, _, _ = inputs(tmp_path)
    output = tmp_path / 'robot.urdf'
    monkeypatch.setattr(module.subprocess, 'run', Mock(side_effect=[NS(stdout='<robot/>'), NS(stdout='')]))
    provenance = module.expand(source, output)
    assert provenance['resource_base'] == str(source.parent)
    assert provenance['mappings'] == [] and provenance['includes'] == {} and provenance['resources'] == {}
    assert ET.parse(output).getroot().tag == 'robot'


@pytest.mark.parametrize('failure', ['xacro', 'deps', 'xml', 'resource'])
def test_preparation_failure_preserves_previous_output(module, monkeypatch, tmp_path, failure):
    source, _, _ = inputs(tmp_path)
    output = tmp_path / 'robot.urdf'
    output.write_text('previous expansion')
    metadata = output.with_suffix('.urdf.json')
    metadata.write_text('previous provenance')
    error = subprocess.CalledProcessError(2, 'xacro')
    responses = [NS(stdout='<robot/>'), NS(stdout='')]
    expected = subprocess.CalledProcessError
    if failure == 'xacro':
        responses[0] = error
    elif failure == 'deps':
        responses[1] = error
    elif failure == 'xml':
        responses[0] = NS(stdout='<broken')
        expected = ET.ParseError
    else:
        responses[0] = NS(stdout='<robot><mesh filename="missing.stl"/></robot>')
        expected = FileNotFoundError
    monkeypatch.setattr(module.subprocess, 'run', Mock(side_effect=responses))
    with pytest.raises(expected):
        module.expand(source, output)
    assert output.read_text() == 'previous expansion'
    assert metadata.read_text() == 'previous provenance'
    assert set(tmp_path.glob('robot.urdf.*')) == {metadata}


def test_atomic_replace_failure_removes_temporary_file_and_retains_previous_output(module, monkeypatch, tmp_path):
    source, _, _ = inputs(tmp_path)
    output = tmp_path / 'robot.urdf'
    output.write_text('previous')
    monkeypatch.setattr(module.subprocess, 'run', Mock(side_effect=[NS(stdout='<robot/>'), NS(stdout='')]))
    monkeypatch.setattr(module.Path, 'replace', Mock(side_effect=OSError('replace failed')))
    with pytest.raises(OSError, match='replace failed'):
        module.expand(source, output)
    assert output.read_text() == 'previous'
    assert list(tmp_path.glob('robot.urdf.*')) == []


def test_cli_parses_mappings_and_resource_base_without_starting_xacro(module, monkeypatch, tmp_path):
    source, _, base = inputs(tmp_path)
    output = tmp_path / 'robot.urdf'
    run = Mock(side_effect=[NS(stdout='<robot/>'), NS(stdout='')])
    monkeypatch.setattr(subprocess, 'run', run)
    monkeypatch.setattr(sys, 'argv', [str(SOURCE), str(source), str(output), 'wheel:=4', '--resource-base', str(base)])
    runpy.run_path(str(SOURCE), run_name='__main__')
    assert output.is_file()
    assert run.call_args_list[0].args[0][-1] == 'wheel:=4'
    assert json.loads(output.with_suffix('.urdf.json').read_text())['resource_base'] == str(base)
