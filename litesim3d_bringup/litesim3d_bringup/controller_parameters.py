"""Populate controller geometry from the simulator's public URDF inspection tool."""
import copy
import json
import math
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix
from launch_ros.parameter_descriptions import ParameterFile
import yaml

from .model import load_yaml, resolve


def model_geometry(robot_description, base_link, requests):
    """Reuse core parsing/FK/radius validation; do not implement a second URDF reader."""
    root = ET.fromstring(robot_description)
    for element in root.findall('.//mesh') + root.findall('.//texture'):
        if element.get('filename'):
            element.set('filename', str(resolve(element.get('filename'), Path.cwd())))
    with tempfile.TemporaryDirectory(prefix='litesim-controller-model-') as directory:
        model = Path(directory) / 'robot.urdf'
        ET.ElementTree(root).write(model, encoding='utf-8', xml_declaration=True)
        executable = Path(get_package_prefix('litesim3d_core')) / 'bin/model_geometry'
        result = subprocess.run([str(executable), str(model), base_link, '--resolve'],
                                input=json.dumps(requests), text=True, capture_output=True, timeout=60)
        if result.returncode:
            raise ValueError(result.stderr.strip() or 'URDF geometry extraction failed')
        return json.loads(result.stdout)


def _names(parameters, key, count=None):
    names = parameters.get(key)
    if (not isinstance(names, list) or not names or
            any(not isinstance(n, str) or not n for n in names) or len(set(names)) != len(names) or
            (count is not None and len(names) != count)):
        raise ValueError(f'{key}: invalid joint bindings')
    return names


def _derived(parameters, key, value, angle=False):
    def equal(a, b):
        if isinstance(b, list):
            return isinstance(a, list) and len(a) == len(b) and all(equal(x, y) for x, y in zip(a, b))
        if isinstance(a, bool) or not isinstance(a, (int, float)) or not math.isfinite(a):
            return False
        delta = a - b
        if angle:
            delta = math.atan2(math.sin(delta), math.cos(delta))
        return abs(delta) < 1e-6
    if key in parameters and not equal(parameters[key], value):
        raise ValueError(f'{key}: configured geometry disagrees with URDF; remove the duplicate value')
    parameters[key] = value


def resolve_controller_parameters(configuration, robot_description, base_link='base_link'):
    """Return a copy with only recognized controllers' mechanical fields derived.

    Controller type, joint roles, gains, command limits and names remain caller-owned.
    Unknown controller implementations pass through unchanged.
    """
    result = copy.deepcopy(configuration)
    if not isinstance(result, dict):
        raise ValueError('controllers: expected a ROS parameter mapping')
    types = {}
    for node in result.values():
        if not isinstance(node, dict) or not isinstance(node.get('ros__parameters'), dict):
            continue
        for name, value in node['ros__parameters'].items():
            if isinstance(value, dict) and isinstance(value.get('type'), str):
                if name in types and types[name] != value['type']:
                    raise ValueError(f'ambiguous controller type: {name}')
                types[name] = value['type']
    supported = {'diff_drive_controller/DiffDriveController': 'differential',
                 'ackermann_steering_controller/AckermannSteeringController': 'ackermann',
                 'litesim3d_ros2_control/OmniDriveController': 'omni'}
    targets, requests = [], []
    for name, implementation in types.items():
        kind = supported.get(implementation)
        if kind is None:
            continue
        matches = [node for key, node in result.items() if key.rstrip('/').split('/')[-1] == name]
        if (len(matches) != 1 or not isinstance(matches[0], dict) or
                not isinstance(matches[0].get('ros__parameters'), dict)):
            raise ValueError(f'{name}: require one unambiguous controller parameter section')
        p = matches[0]['ros__parameters']
        if p.get('base_frame_id', base_link) != base_link:
            raise ValueError(f'{name}: base_frame_id differs from the URDF geometry reference {base_link}')
        if kind == 'differential':
            bindings = {'left_joints': _names(p, 'left_wheel_names'),
                        'right_joints': _names(p, 'right_wheel_names')}
        elif kind == 'ackermann':
            if p.get('front_steering', True) is not True:
                raise ValueError('URDF geometry adapter currently requires front steering')
            right, left = _names(p, 'rear_wheels_names', 2)
            steer_right, steer_left = _names(p, 'front_wheels_names', 2)
            bindings = {'rear_left_joint': left, 'rear_right_joint': right,
                        'left_steering_joint': steer_left, 'right_steering_joint': steer_right}
        else:
            bindings = {'wheels': [{'joint': joint} for joint in _names(p, 'wheel_joints')]}
        targets.append((kind, p))
        requests.append({'kind': kind, 'parameters': bindings})
    if not requests:
        return result
    geometry = model_geometry(robot_description, base_link, requests)
    for (kind, p), derived in zip(targets, geometry['resolved']):
        if kind == 'differential':
            _derived(p, 'wheel_radius', derived['wheel_radius_m'])
            _derived(p, 'wheel_separation', derived['wheel_separation_m'])
        elif kind == 'ackermann':
            for key, source in [('wheelbase', 'wheelbase_m'), ('front_wheel_track', 'track_width_m'),
                                ('rear_wheel_track', 'track_width_m'), ('front_wheels_radius', 'wheel_radius_m'),
                                ('rear_wheels_radius', 'wheel_radius_m')]:
                _derived(p, key, derived[source])
        else:
            for key, source in [('wheel_x', 'x_m'), ('wheel_y', 'y_m'),
                                ('wheel_radius', 'radius_m'), ('wheel_angle', 'rolling_angle_rad')]:
                _derived(p, key, [w[source] for w in derived['wheels']], angle=key == 'wheel_angle')
            limits = [geometry['joints'][name]['max_velocity'] for name in p['wheel_joints']]
            if any(not isinstance(v, (int, float)) or not math.isfinite(v) or v <= 0 for v in limits):
                raise ValueError('omni wheels require finite positive URDF velocity limits')
            if 'max_wheel_velocity' in p:
                caps = p['max_wheel_velocity']
                if (not isinstance(caps, list) or len(caps) != len(limits) or
                        any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or
                            not 0 < v <= limit for v, limit in zip(caps, limits))):
                    raise ValueError('max_wheel_velocity exceeds URDF limits')
            else:
                p['max_wheel_velocity'] = limits
    return result


class UrdfControllerParameters(ParameterFile):
    """Launch parameter file with deferred paths and standard temporary-file cleanup."""
    def __init__(self, param_file, robot_description, base_link='base_link'):
        self._generated = None
        super().__init__(param_file)
        self._robot_description = robot_description
        self._base_link = base_link

    def evaluate(self, context):
        if self._generated is None:
            source = super().evaluate(context)
            values = resolve_controller_parameters(load_yaml(source.read_text()),
                                                   self._robot_description, self._base_link)
            with tempfile.NamedTemporaryFile(mode='w', prefix='litesim-controllers-', suffix='.yaml',
                                             encoding='utf-8', delete=False) as stream:
                self._generated = Path(stream.name)
                yaml.safe_dump(values, stream, sort_keys=False)
        return self._generated

    def cleanup(self):
        if self._generated is not None:
            self._generated.unlink(missing_ok=True)
            self._generated = None
        super().cleanup()
