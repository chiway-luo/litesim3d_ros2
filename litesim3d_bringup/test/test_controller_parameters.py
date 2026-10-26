"""Exercise controller preparation against the real core URDF reader."""
import copy
import math
from pathlib import Path

from launch import LaunchContext
from launch.substitutions import LaunchConfiguration
import pytest
import yaml

from litesim3d_bringup.controller_parameters import (
    UrdfControllerParameters, resolve_controller_parameters,
)


def wheel(name, x, y, radius, angle=0, parent='base_link'):
    return f'''<link name="{name}_link"><collision><origin rpy="{-math.pi/2} 0 0"/>
      <geometry><cylinder radius="{radius}" length="0.04"/></geometry></collision></link>
      <joint name="{name}" type="continuous"><parent link="{parent}"/><child link="{name}_link"/>
      <origin xyz="{x} {y} 0" rpy="0 0 {angle}"/><axis xyz="0 1 0"/>
      <limit effort="20" velocity="30"/></joint>'''


def model(kind, radius=.17, spacing=.7):
    parts = ['<robot name="test"><link name="base_link"/>']
    if kind == 'diff':
        parts += [wheel('left', 0, spacing/2, radius), wheel('right', 0, -spacing/2, radius)]
    elif kind == 'ack':
        for side, sign in [('left', 1), ('right', -1)]:
            parts += [wheel(side, 0, sign*spacing/2, radius),
                      f'''<link name="{side}_steer_link"/><joint name="{side}_steer" type="revolute">
                      <parent link="base_link"/><child link="{side}_steer_link"/>
                      <origin xyz="0.8 {sign*spacing/2} 0"/><axis xyz="0 0 1"/>
                      <limit lower="-0.5" upper="0.55" effort="20" velocity="5"/></joint>''',
                      wheel('front_'+side, 0, 0, radius, parent=side+'_steer_link')]
    else:
        for i in range(3):
            theta = 2*math.pi*i/3
            parts.append(wheel(f'wheel_{i}', spacing*math.cos(theta), spacing*math.sin(theta),
                               radius, theta+math.pi/2))
    return ''.join(parts) + '</robot>'


def config(kind):
    if kind == 'diff':
        implementation = 'diff_drive_controller/DiffDriveController'
        bindings = {'left_wheel_names': ['left'], 'right_wheel_names': ['right']}
    elif kind == 'ack':
        implementation = 'ackermann_steering_controller/AckermannSteeringController'
        bindings = {'rear_wheels_names': ['right', 'left'],
                    'front_wheels_names': ['right_steer', 'left_steer'], 'front_steering': True}
    else:
        implementation = 'litesim3d_ros2_control/OmniDriveController'
        bindings = {'wheel_joints': ['wheel_0', 'wheel_1', 'wheel_2']}
    return {'/controller_manager': {'ros__parameters': {'drive': {'type': implementation}}},
            '/drive': {'ros__parameters': bindings}}


@pytest.mark.parametrize('kind', ['diff', 'ack', 'omni'])
def test_only_model_changes_propagate_without_mutating_config(kind):
    source = config(kind); before = copy.deepcopy(source)
    first = resolve_controller_parameters(source, model(kind))['/drive']['ros__parameters']
    changed = resolve_controller_parameters(source, model(kind, .23, .9))['/drive']['ros__parameters']
    assert source == before
    if kind == 'diff':
        assert first['wheel_radius'] == .17 and changed['wheel_radius'] == .23
        assert first['wheel_separation'] == .7 and changed['wheel_separation'] == .9
    elif kind == 'ack':
        assert first['wheelbase'] == .8
        assert first['front_wheel_track'] == first['rear_wheel_track'] == .7
        assert changed['front_wheels_radius'] == changed['rear_wheels_radius'] == .23
        assert changed['front_wheel_track'] == changed['rear_wheel_track'] == .9
    else:
        assert first['wheel_x'][0] == .7 and changed['wheel_x'][0] == .9
        assert changed['wheel_radius'] == [.23]*3
        assert first['wheel_angle'][0] == pytest.approx(math.pi/2)
        assert first['max_wheel_velocity'] == [30]*3


@pytest.mark.parametrize('kind,key,value', [('diff', 'wheel_radius', .1), ('ack', 'wheelbase', .6),
                                         ('omni', 'wheel_x', [.3, -.15, -.15])])
def test_stale_geometry_rejected(kind, key, value):
    source = config(kind); source['/drive']['ros__parameters'][key] = value
    with pytest.raises(ValueError, match='disagrees with URDF'):
        resolve_controller_parameters(source, model(kind))


def test_incomplete_or_ambiguous_model_cannot_fall_back_to_yaml():
    source = config('diff'); source['/drive']['ros__parameters']['wheel_radius'] = .17
    invalid = model('diff').replace('<cylinder radius="0.17" length="0.04"/>', '<box size="0.2 0.2 0.2"/>')
    with pytest.raises(ValueError, match='collision cylinder'):
        resolve_controller_parameters(source, invalid)
    with pytest.raises(ValueError, match='wheel joint not found|unknown joint'):
        resolve_controller_parameters(source, model('diff').replace('name="left"', 'name="missing"'))
    with pytest.raises(ValueError, match='layout/axis'):
        resolve_controller_parameters(source, model('diff').replace('rpy="0 0 0"', 'rpy="0 0 0.2"'))


def test_operational_limits_and_unknown_controllers_are_preserved():
    source = config('omni'); p = source['/drive']['ros__parameters']
    p['max_wheel_velocity'] = [12]*3
    assert resolve_controller_parameters(source, model('omni'))['/drive']['ros__parameters']['max_wheel_velocity'] == [12]*3
    p['max_wheel_velocity'] = [31]*3
    with pytest.raises(ValueError, match='exceeds URDF'):
        resolve_controller_parameters(source, model('omni'))
    source = {'/controller_manager': {'ros__parameters': {'custom': {'type': 'vendor/Controller'}}}}
    assert resolve_controller_parameters(source, 'unused') == source


def test_deferred_parameter_file_evaluation_and_cleanup(tmp_path):
    path = tmp_path / 'controllers.yaml'; path.write_text(yaml.safe_dump(config('diff')))
    before = path.read_bytes()
    params = UrdfControllerParameters(LaunchConfiguration('controllers'), model('diff'))
    context = LaunchContext(); context.launch_configurations['controllers'] = str(path)
    generated = params.evaluate(context)
    assert isinstance(generated, Path) and generated != path
    assert yaml.safe_load(generated.read_text())['/drive']['ros__parameters']['wheel_separation'] == .7
    assert params.evaluate(context) == generated and path.read_bytes() == before
    params.cleanup(); assert not generated.exists() and path.exists()


def test_conflicting_controller_sections_and_base_frame_rejected():
    source = config('diff'); source['/drive']['ros__parameters']['base_frame_id'] = 'other'
    with pytest.raises(ValueError, match='base_frame_id'):
        resolve_controller_parameters(source, model('diff'))
    source = config('diff'); source['/namespace/drive'] = source['/drive']
    with pytest.raises(ValueError, match='unambiguous'):
        resolve_controller_parameters(source, model('diff'))
