"""Launch ParameterFile ownership and native controller adapter tests."""
from pathlib import Path
from unittest.mock import Mock
import pytest
import yaml
from launch import LaunchContext
from launch.substitutions import LaunchConfiguration
from litesim3d_bringup import controller_parameters as module


def test_native_geometry_and_controller_operations_preserve_contract(monkeypatch):
    request = Mock(return_value={'resolved': []})
    monkeypatch.setattr(module, '_request', request)
    assert module.model_geometry('<robot/>', 'base', []) == {'resolved': []}
    request.assert_called_with('model_geometry', description='<robot/>', base_link='base', requests=[])
    module.resolve_controller_parameters({'configuration': 1}, '<robot/>')
    request.assert_called_with('controller_parameters', configuration={'configuration': 1}, description='<robot/>', base_link='base_link')


def test_parameter_file_is_deferred_cached_and_cleaned_without_mutating_source(monkeypatch, tmp_path):
    path = tmp_path / 'controllers.yaml'
    path.write_text('controller: {ros__parameters: {gain: 2}}')
    before = path.read_bytes()
    monkeypatch.setattr(module, 'load_yaml', yaml.safe_load)
    resolve = Mock(return_value={'controller': {'ros__parameters': {'gain': 2, 'wheel_radius': .2}}})
    monkeypatch.setattr(module, 'resolve_controller_parameters', resolve)
    params = module.UrdfControllerParameters(LaunchConfiguration('controllers'), '<robot/>')
    context = LaunchContext(); context.launch_configurations['controllers'] = str(path)
    generated = params.evaluate(context)
    assert isinstance(generated, Path) and generated != path
    assert yaml.safe_load(generated.read_text())['controller']['ros__parameters']['wheel_radius'] == .2
    assert params.evaluate(context) == generated
    resolve.assert_called_once()
    params.cleanup(); params.cleanup()
    assert not generated.exists() and path.read_bytes() == before


def test_native_validation_failure_does_not_cache_a_parameter_file(monkeypatch, tmp_path):
    source = tmp_path / 'controllers.yaml'; source.write_text('{}')
    monkeypatch.setattr(module, 'load_yaml', yaml.safe_load)
    monkeypatch.setattr(module, 'resolve_controller_parameters', Mock(side_effect=ValueError('geometry mismatch')))
    params = module.UrdfControllerParameters(str(source), '<robot/>')
    with pytest.raises(ValueError, match='geometry mismatch'):
        params.evaluate(LaunchContext())
    assert params._generated is None
    params.cleanup()


def test_parameter_file_write_failure_removes_partial_output(monkeypatch, tmp_path):
    source = tmp_path / 'controllers.yaml'; source.write_text('{}')
    monkeypatch.setattr(module, 'load_yaml', yaml.safe_load)
    monkeypatch.setattr(module, 'resolve_controller_parameters', Mock(return_value={}))
    generated = []
    def fail_write(values, stream, **kwargs):
        generated.append(Path(stream.name))
        raise OSError('write failed')
    monkeypatch.setattr(module.yaml, 'safe_dump', fail_write)
    params = module.UrdfControllerParameters(str(source), '<robot/>')
    with pytest.raises(OSError, match='write failed'):
        params.evaluate(LaunchContext())
    assert params._generated is None and len(generated) == 1 and not generated[0].exists()
