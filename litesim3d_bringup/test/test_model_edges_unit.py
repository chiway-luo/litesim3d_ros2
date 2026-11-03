"""Thin Python/XML adapter tests; no native process is launched."""
import json
import copy
from pathlib import Path
import subprocess
from types import SimpleNamespace
import xml.etree.ElementTree as ET
from unittest.mock import Mock
import pytest
from litesim3d_bringup import model


def test_request_preserves_json_and_reports_native_failure(monkeypatch):
    monkeypatch.setattr(model, '_executable', lambda: '/native/prepare_config')
    run = Mock(return_value=SimpleNamespace(returncode=0, stdout='{"value": true}', stderr=''))
    monkeypatch.setattr(model.subprocess, 'run', run)
    assert model._request('example', value='space and unicode 中文') == {'value': True}
    assert json.loads(run.call_args.kwargs['input']) == {'operation': 'example', 'value': 'space and unicode 中文'}
    run.return_value = SimpleNamespace(returncode=1, stderr='native error')
    with pytest.raises(ValueError, match='native error'):
        model._request('example')
    run.return_value.stderr = ''
    with pytest.raises(ValueError, match='native model preparation failed'):
        model._request('example')


def test_xml_adapter_preserves_element_identity(monkeypatch):
    sensor = ET.fromstring('<sensor name="s"/>')
    monkeypatch.setattr(model, '_request', lambda *a, **k: '<sensor name="s"><overrides>enabled: false</overrides></sensor>')
    model.override(sensor, {'enabled': False})
    assert sensor.get('name') == 's' and sensor.find('overrides').text == 'enabled: false'
    root = ET.fromstring('<robot><old/></robot>')
    data = {'old': 1}
    monkeypatch.setattr(model, '_request', lambda *a, **k: {'data': {'new': 2}, 'xml': '<robot><new/></robot>'})
    model.apply_noise_config(data, root, '/noise', '/base')
    assert data == {'new': 2} and root.find('new') is not None and root.find('old') is None
    model.apply_noise_config(data, None, '/noise', '/base')


def test_path_and_model_argument_types_cross_the_adapter(monkeypatch):
    request = Mock(return_value='/generated/config.yaml')
    monkeypatch.setattr(model, '_request', request)
    assert model.prepare_config(Path('/source'), Path('/out'), mappings={1: 'invalid'}) == Path('/generated/config.yaml')
    assert request.call_args.kwargs['mappings_yaml'] == "1: invalid\n"
    assert model.resolve('model.urdf', Path('/base')) == Path('/generated/config.yaml')
    model.ros_robot_description('/model'); model.profile('mid360', '/profiles')
    model.merge({}, {}); model.load_yaml('robot: {}'); model.lidar_preview_topics('/config', 'native', 1)


def test_compatibility_main_executes_native_program(monkeypatch):
    monkeypatch.setattr(model, '_executable', lambda: '/native/prepare_config')
    monkeypatch.setattr(model.sys, 'argv', ['prepare_config', 'source', 'output'])
    execute = Mock()
    monkeypatch.setattr(model.os, 'execv', execute)
    model.main()
    execute.assert_called_once_with('/native/prepare_config', ['/native/prepare_config', 'source', 'output'])


@pytest.mark.parametrize('value, expected', [
    ('true', True), (' TRUE \n', True), (True, True),
    ('false', False), ('\tFalse ', False), (False, False),
])
def test_launch_boolean_is_explicit_and_whitespace_tolerant(value, expected):
    assert model.boolean(value, 'cameras') is expected


@pytest.mark.parametrize('value', ['', 'yes', '0', '1', 0, 1, None, [], 'true false'])
def test_launch_boolean_rejects_non_boolean_values_with_option_name(value):
    with pytest.raises(ValueError, match='cameras must be true or false'):
        model.boolean(value, 'cameras')


def test_executable_resolves_real_package_index_not_working_directory(monkeypatch, tmp_path):
    prefix = tmp_path / 'prefix with 中文 space'
    marker = prefix / 'share/ament_index/resource_index/packages/litesim3d_bringup'
    marker.parent.mkdir(parents=True)
    marker.write_text('')
    monkeypatch.setenv('AMENT_PREFIX_PATH', str(prefix))
    monkeypatch.chdir(tmp_path)
    assert model._executable() == str(prefix / 'lib/litesim3d_bringup/prepare_config')


def test_request_uses_argument_vector_timeout_and_json_without_mutating_values(monkeypatch):
    monkeypatch.setattr(model, '_executable', lambda: '/prefix with space/prepare_config')
    values = {'nested': ['$(not-a-shell-command)', None, {'enabled': False}]}
    previous = copy.deepcopy(values)
    run = Mock(return_value=SimpleNamespace(returncode=0, stdout='[1, null, false]', stderr='unused warning'))
    monkeypatch.setattr(model.subprocess, 'run', run)
    assert model._request('merge', base=values) == [1, None, False]
    run.assert_called_once_with(
        ['/prefix with space/prepare_config', '--request'],
        input=json.dumps({'operation': 'merge', 'base': values}),
        text=True, capture_output=True, timeout=180)
    assert values == previous


@pytest.mark.parametrize('failure', [
    OSError('native executable unavailable'),
    subprocess.TimeoutExpired('prepare_config', 180),
])
def test_request_propagates_transport_failure_without_fallback(monkeypatch, failure):
    monkeypatch.setattr(model, '_executable', lambda: '/native/prepare_config')
    run = Mock(side_effect=failure)
    monkeypatch.setattr(model.subprocess, 'run', run)
    with pytest.raises(type(failure)) as caught:
        model._request('load_yaml', text='{}')
    assert caught.value is failure
    assert run.call_count == 1


def test_request_rejects_unserializable_input_before_starting_native_command(monkeypatch):
    monkeypatch.setattr(model, '_executable', lambda: '/native/prepare_config')
    run = Mock()
    monkeypatch.setattr(model.subprocess, 'run', run)
    with pytest.raises(TypeError):
        model._request('merge', base={'unsupported': object()})
    run.assert_not_called()


def test_request_rejects_corrupt_native_json_and_strips_error_text(monkeypatch):
    monkeypatch.setattr(model, '_executable', lambda: '/native/prepare_config')
    run = Mock(return_value=SimpleNamespace(returncode=0, stdout='{bad-json', stderr=''))
    monkeypatch.setattr(model.subprocess, 'run', run)
    with pytest.raises(json.JSONDecodeError):
        model._request('load_yaml', text='{}')
    run.return_value = SimpleNamespace(returncode=2, stdout='{}', stderr=' \n native detail \t')
    with pytest.raises(ValueError, match='^native detail$'):
        model._request('load_yaml', text='{}')
    run.return_value.stderr = '\n\t '
    with pytest.raises(ValueError, match='^native model preparation failed$'):
        model._request('load_yaml', text='{}')


def test_xml_replacement_removes_stale_state_and_bad_xml_preserves_original():
    target = ET.fromstring('<sensor stale="yes">old<old/></sensor>')
    target.tail = 'outside sensor'
    original = ET.tostring(target)
    with pytest.raises(ET.ParseError):
        model._replace_element(target, '<broken')
    assert ET.tostring(target) == original
    alias = target
    model._replace_element(target, '<sensor name="new">new<child gain="2"/></sensor>')
    assert target is alias
    assert target.attrib == {'name': 'new'}
    assert target.text == 'new' and target.tail == 'outside sensor'
    assert target.find('old') is None
    assert target.find('child').attrib == {'gain': '2'}


def test_noise_provider_failure_preserves_caller_owned_mapping_and_tree(monkeypatch):
    root = ET.fromstring('<robot marker="original"><link name="base"/></robot>')
    data = {'nested': {'enabled': True}}
    previous, xml = copy.deepcopy(data), ET.tostring(root)
    request = Mock(side_effect=ValueError('native validation'))
    monkeypatch.setattr(model, '_request', request)
    with pytest.raises(ValueError, match='native validation'):
        model.apply_noise_config(data, root, 'noise.yaml', '/source')
    assert data == previous and ET.tostring(root) == xml


def test_prepare_adapter_preserves_every_option_and_does_not_mutate_mappings(monkeypatch):
    mappings = {'name': '中文 space', 'enabled': False, 'count': 2}
    before = copy.deepcopy(mappings)
    request = Mock(return_value='/generated/config.yaml')
    monkeypatch.setattr(model, '_request', request)
    assert model.prepare_config(
        Path('/source/config.yaml'), Path('/output'), robot_urdf=Path('/model.urdf'),
        mappings=mappings, cameras='false', lidar_profile=Path('/profile.yaml'),
        profiles_dir=Path('/profiles'), noise_config=Path('/noise.yaml')) == Path('/generated/config.yaml')
    operation, = request.call_args.args
    assert operation == 'prepare'
    assert request.call_args.kwargs == {
        'source': '/source/config.yaml', 'directory': '/output', 'robot_urdf': '/model.urdf',
        'mappings_yaml': model.yaml.safe_dump(mappings), 'cameras': 'false',
        'lidar_profile': '/profile.yaml', 'profiles_dir': '/profiles', 'noise_config': '/noise.yaml'}
    assert mappings == before


def test_compatibility_main_propagates_exec_failure_and_preserves_original_arguments(monkeypatch):
    argv = ['legacy-name', '--', 'space value', 'name:=中文']
    monkeypatch.setattr(model.sys, 'argv', argv)
    monkeypatch.setattr(model, '_executable', lambda: '/native/prepare_config')
    failure = OSError('exec unavailable')
    execute = Mock(side_effect=failure)
    monkeypatch.setattr(model.os, 'execv', execute)
    with pytest.raises(OSError) as caught:
        model.main()
    assert caught.value is failure
    assert argv == ['legacy-name', '--', 'space value', 'name:=中文']
    execute.assert_called_once_with('/native/prepare_config',
                                  ['/native/prepare_config', '--', 'space value', 'name:=中文'])
