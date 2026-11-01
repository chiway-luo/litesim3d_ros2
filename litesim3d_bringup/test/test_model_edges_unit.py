"""Thin Python/XML adapter tests; no native process is launched."""
import json
from pathlib import Path
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
