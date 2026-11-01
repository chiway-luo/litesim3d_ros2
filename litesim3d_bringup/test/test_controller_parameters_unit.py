"""Pure controller-parameter logic units with explicit native-extractor boundaries."""
import copy
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock
import xml.etree.ElementTree as ET

import pytest
import yaml
from launch import LaunchContext
from launch.substitutions import LaunchConfiguration
from litesim3d_bringup import controller_parameters as module


def configuration(kind):
    implementations = {'diff': 'diff_drive_controller/DiffDriveController',
                       'ack': 'ackermann_steering_controller/AckermannSteeringController',
                       'omni': 'litesim3d_ros2_control/OmniDriveController'}
    bindings = {'diff': {'left_wheel_names': ['left'], 'right_wheel_names': ['right']},
                'ack': {'rear_wheels_names': ['right', 'left'], 'front_wheels_names': ['steer_right', 'steer_left']},
                'omni': {'wheel_joints': ['w0', 'w1', 'w2']}}
    return {'/manager': {'ros__parameters': {'drive': {'type': implementations[kind]}}},
            '/namespace/drive': {'ros__parameters': copy.deepcopy(bindings[kind])}}


def geometry(kind):
    if kind == 'diff':
        resolved = {'wheel_radius_m': .2, 'wheel_separation_m': .8}
    elif kind == 'ack':
        resolved = {'wheelbase_m': .9, 'track_width_m': .7, 'wheel_radius_m': .2}
    else:
        resolved = {'wheels': [{'x_m': x, 'y_m': y, 'radius_m': .2, 'rolling_angle_rad': angle}
                               for x, y, angle in [(1., 0., 0.), (-.5, .8, 2.), (-.5, -.8, -2.)]]}
    return {'resolved': [resolved], 'joints': {name: {'max_velocity': limit}
                                              for name, limit in zip(['w0', 'w1', 'w2'], [10., 20., 30.])}}


def test_model_geometry_resolves_resources_and_calls_native_boundary(monkeypatch, tmp_path):
    mesh, texture = tmp_path / 'mesh.dae', tmp_path / 'texture.png'
    mesh.touch(); texture.touch()
    xml = f'<robot><mesh filename="{mesh}"/><texture filename="{texture}"/><mesh/></robot>'
    requests = [{'kind': 'unit', 'parameters': {'joint': 'left'}}]
    seen = []
    def run(command, **kwargs):
        parsed = ET.parse(command[1])
        assert [e.get('filename') for e in parsed.findall('.//mesh')] == [str(mesh), None]
        assert parsed.find('.//texture').get('filename') == str(texture)
        seen.append(command[1])
        assert command[0] == '/core/bin/model_geometry' and command[2:] == ['base', '--resolve']
        assert json.loads(kwargs['input']) == requests
        assert kwargs['text'] and kwargs['capture_output'] and kwargs['timeout'] == 60
        return NS(returncode=0, stdout='{"resolved": []}', stderr='')
    monkeypatch.setattr(module, 'get_package_prefix', lambda _: '/core')
    monkeypatch.setattr(module.subprocess, 'run', run)
    assert module.model_geometry(xml, 'base', requests) == {'resolved': []}
    assert not Path(seen[0]).exists()


@pytest.mark.parametrize('stderr,message', [(' invalid joints\n', 'invalid joints'), ('', 'URDF geometry extraction failed')])
def test_model_geometry_reports_provider_failure_without_fallback(monkeypatch, stderr, message):
    monkeypatch.setattr(module, 'get_package_prefix', lambda _: '/core')
    monkeypatch.setattr(module.subprocess, 'run', Mock(return_value=NS(returncode=2, stderr=stderr)))
    with pytest.raises(ValueError, match=message):
        module.model_geometry('<robot/>', 'base', [])


@pytest.mark.parametrize('names,count', [(None, None), ('a', None), ([], None), ([''], None),
    ([1], None), (['a', 'a'], None), (['a'], 2)])
def test_names_rejects_empty_duplicate_typed_and_count_mismatches(names, count):
    with pytest.raises(ValueError, match='joints: invalid joint bindings'):
        module._names({'joints': names}, 'joints', count)
    assert module._names({'joints': ['a', 'b']}, 'joints', 2) == ['a', 'b']


@pytest.mark.parametrize('configured', [True, '1', float('inf'), float('nan'), .1])
def test_derived_refuses_nonfinite_non_numeric_or_disagreeing_values(configured):
    settings = {'radius': configured}
    with pytest.raises(ValueError, match='configured geometry disagrees'):
        module._derived(settings, 'radius', .2)
    assert settings['radius'] is configured


def test_derived_recurses_lists_and_wraps_angles():
    values = {'angle': [2 * math.pi + .1, -.2], 'nested': [[1., 2.], [3.]]}
    module._derived(values, 'angle', [.1, -.2], angle=True)
    module._derived(values, 'nested', [[1., 2.], [3.]])
    assert values['angle'] == [.1, -.2]
    for bad in (1., [1.], [[1., 2.], [4.]]):
        with pytest.raises(ValueError, match='disagrees'):
            module._derived({'nested': bad}, 'nested', [[1., 2.], [3.]])


@pytest.mark.parametrize('kind', ['diff', 'ack', 'omni'])
def test_controller_bindings_and_derived_geometry_do_not_mutate_caller(monkeypatch, kind):
    source = configuration(kind)
    before = copy.deepcopy(source)
    provider = Mock(return_value=geometry(kind))
    monkeypatch.setattr(module, 'model_geometry', provider)
    result = module.resolve_controller_parameters(source, '<robot/>')
    assert source == before and result is not source
    actual = result['/namespace/drive']['ros__parameters']
    request = provider.call_args.args[2]
    if kind == 'diff':
        assert actual['wheel_radius'] == .2 and actual['wheel_separation'] == .8
        expected = {'kind': 'differential', 'parameters': {'left_joints': ['left'], 'right_joints': ['right']}}
    elif kind == 'ack':
        assert actual['wheelbase'] == .9 and actual['front_wheel_track'] == actual['rear_wheel_track'] == .7
        assert actual['front_wheels_radius'] == actual['rear_wheels_radius'] == .2
        expected = {'kind': 'ackermann', 'parameters': {'rear_left_joint': 'left', 'rear_right_joint': 'right',
                     'left_steering_joint': 'steer_left', 'right_steering_joint': 'steer_right'}}
    else:
        assert actual['wheel_x'] == [1., -.5, -.5] and actual['wheel_radius'] == [.2] * 3
        assert actual['max_wheel_velocity'] == [10., 20., 30.]
        expected = {'kind': 'omni', 'parameters': {'wheels': [{'joint': name} for name in ['w0', 'w1', 'w2']]}}
    assert request == [expected]
    assert provider.call_args.args[:2] == ('<robot/>', 'base_link')


@pytest.mark.parametrize('bad', [None, [], 'controllers'])
def test_controller_configuration_must_be_mapping(bad):
    with pytest.raises(ValueError, match='ROS parameter mapping'):
        module.resolve_controller_parameters(bad, 'unused')


def test_unknown_implementations_and_unrelated_node_sections_pass_through(monkeypatch):
    source = {'/bad': 1, '/other': {'ros__parameters': []}, '/plain': {'ros__parameters': {'gain': 3}},
              '/manager': {'ros__parameters': {'custom': {'type': 'vendor/Unknown'}}}}
    provider = Mock()
    monkeypatch.setattr(module, 'model_geometry', provider)
    assert module.resolve_controller_parameters(source, 'unused') == source
    provider.assert_not_called()


@pytest.mark.parametrize('failure', ['conflicting_type', 'duplicate_section', 'missing_section',
                                     'bad_section', 'bad_parameters', 'base_frame', 'rear_steering'])
def test_controller_binding_ambiguities_are_rejected_before_provider(monkeypatch, failure):
    source = configuration('ack')
    expected = 'unambiguous'
    if failure == 'conflicting_type':
        source['/other_manager'] = {'ros__parameters': {'drive': {'type': 'vendor/Different'}}}
        expected = 'ambiguous controller type'
    elif failure == 'duplicate_section':
        source['/drive'] = copy.deepcopy(source['/namespace/drive'])
    elif failure == 'missing_section':
        source.pop('/namespace/drive')
    elif failure == 'bad_section':
        source['/namespace/drive'] = 1
    elif failure == 'bad_parameters':
        source['/namespace/drive'] = {'ros__parameters': []}
    elif failure == 'base_frame':
        source['/namespace/drive']['ros__parameters']['base_frame_id'] = 'other'
        expected = 'base_frame_id differs'
    else:
        source['/namespace/drive']['ros__parameters']['front_steering'] = False
        expected = 'requires front steering'
    provider = Mock()
    monkeypatch.setattr(module, 'model_geometry', provider)
    with pytest.raises(ValueError, match=expected):
        module.resolve_controller_parameters(source, 'unused')
    provider.assert_not_called()


@pytest.mark.parametrize('limit', [None, 'fast', float('nan'), float('inf'), 0., -1.])
def test_omni_requires_finite_positive_provider_velocity_limits(monkeypatch, limit):
    value = geometry('omni')
    value['joints']['w1']['max_velocity'] = limit
    monkeypatch.setattr(module, 'model_geometry', Mock(return_value=value))
    with pytest.raises(ValueError, match='finite positive URDF velocity limits'):
        module.resolve_controller_parameters(configuration('omni'), 'unused')


@pytest.mark.parametrize('caps', ['bad', [], [1., 2.], [True, 2., 3.], ['1', 2., 3.],
                                  [float('nan'), 2., 3.], [0., 2., 3.], [11., 2., 3.]])
def test_omni_rejects_invalid_or_over_limit_caller_caps(monkeypatch, caps):
    source = configuration('omni')
    source['/namespace/drive']['ros__parameters']['max_wheel_velocity'] = caps
    monkeypatch.setattr(module, 'model_geometry', Mock(return_value=geometry('omni')))
    with pytest.raises(ValueError, match='max_wheel_velocity exceeds URDF limits'):
        module.resolve_controller_parameters(source, 'unused')


def test_omni_preserves_safe_caps_and_equivalent_angular_geometry(monkeypatch):
    source = configuration('omni')
    source['/namespace/drive']['ros__parameters'].update(max_wheel_velocity=[5., 10., 15.],
                                                        wheel_angle=[2 * math.pi, 2., -2.])
    monkeypatch.setattr(module, 'model_geometry', Mock(return_value=geometry('omni')))
    result = module.resolve_controller_parameters(source, 'unused')['/namespace/drive']['ros__parameters']
    assert result['max_wheel_velocity'] == [5., 10., 15.]
    assert result['wheel_angle'] == [0., 2., -2.]


def test_deferred_parameter_file_is_cached_and_cleanup_is_repeatable(monkeypatch, tmp_path):
    source = configuration('diff')
    path = tmp_path / 'controllers.yaml'
    path.write_text(yaml.safe_dump(source))
    original = path.read_bytes()
    monkeypatch.setattr(module, 'model_geometry', Mock(return_value=geometry('diff')))
    parameters = module.UrdfControllerParameters(LaunchConfiguration('controllers'), '<robot/>')
    context = LaunchContext()
    context.launch_configurations['controllers'] = str(path)
    generated = parameters.evaluate(context)
    assert generated != path and generated.is_file()
    assert yaml.safe_load(generated.read_text())['/namespace/drive']['ros__parameters']['wheel_separation'] == .8
    assert parameters.evaluate(context) == generated
    module.model_geometry.assert_called_once()
    parameters.cleanup()
    parameters.cleanup()
    assert not generated.exists() and path.read_bytes() == original
