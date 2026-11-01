"""ROS-test fixture/readiness regressions; no ROS context or peers are started.

Compile the actual pure helpers without importing the surrounding integration
modules, whose imports require installed ROS interfaces. Callback delivery is an
explicit deterministic schedule, not a substitute for DDS integration coverage.
"""
import ast
from collections import deque
import copy
import itertools
import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[2]


def omni4_parameters(root=ROOT):
    if (root / 'plugins').is_dir():
        # A missing checked-out YAML must fail, not select an installed copy.
        path = root / 'plugins/litesim3d_omni4_drive/config/parameters.yaml'
    else:
        # The quality runner stages the bridge package on its own after building
        # all packages; its sibling plugins are available through ament_index.
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory('litesim3d_omni4_drive')) / 'config/parameters.yaml'
    return yaml.safe_load(path.read_text())


def helpers(filename, *names):
    path = Path(__file__).with_name(filename)
    source = ast.parse(path.read_text(), filename=str(path))
    functions = [node for node in source.body
                 if isinstance(node, ast.FunctionDef) and node.name in names]
    if {node.name for node in functions} != set(names):
        raise AssertionError(f'missing helpers in {path}: {names}')
    namespace = {'ET': ET, 'math': math, 'yaml': yaml}
    exec(compile(ast.Module(body=functions, type_ignores=[]), str(path), 'exec'), namespace)
    return NS(**{name: namespace[name] for name in names})


def tf_message(*pairs):
    return NS(transforms=[NS(header=NS(frame_id=parent), child_frame_id=child)
                          for parent, child in pairs])


class CallbackHarness:
    """A ready TF listener does not invoke the independent raw callback."""
    def __init__(self, initial=(), pending=()):
        self.messages = {'tf': list(initial)}
        self.pending = deque(pending)
        self.deliveries = 0
        self.until_calls = 0
        self.buffer = NS(can_transform=lambda *args: True)

    def until(self, predicate):
        self.until_calls += 1
        while not predicate():
            if not self.pending:
                raise AssertionError('condition timeout: required raw TF edge not observed')
            self.messages['tf'].append(self.pending.popleft())
            self.deliveries += 1


class IntegrationExpectationTests(unittest.TestCase):
    def test_omni_fixture_uses_joint_only_bindings_and_urdf_geometry(self):
        function = helpers('test_plugins_ros.py', 'add_plugins', 'omni4_robot').omni4_robot
        parameters = omni4_parameters()
        wheels = parameters['wheels']
        before = copy.deepcopy(wheels)
        self.assertEqual(len(wheels), 4)
        self.assertTrue(all(set(wheel) == {'joint'} for wheel in wheels))
        robot = function(wheels)
        self.assertEqual(wheels, before)
        joints = robot.findall('joint')
        self.assertEqual({joint.get('name') for joint in joints}, {wheel['joint'] for wheel in wheels})
        rows = []
        for joint in joints:
            self.assertEqual(joint.get('type'), 'continuous')
            self.assertEqual(joint.find('parent').get('link'), 'base_link')
            x, y, z = map(float, joint.find('origin').get('xyz').split())
            self.assertEqual((abs(x), abs(y), z), (.3, .3, .1))
            axis = list(map(float, joint.find('axis').get('xyz').split()))
            self.assertAlmostEqual(sum(value * value for value in axis), 1)
            self.assertEqual(axis[2], 0)
            link = robot.find(f"link[@name='{joint.find('child').get('link')}']")
            collision = link.find('collision')
            roll, pitch, yaw = map(float, collision.find('origin').get('rpy').split())
            self.assertAlmostEqual(roll, -math.pi / 2)
            self.assertEqual(pitch, 0)
            self.assertAlmostEqual(axis[0], -math.sin(yaw))
            self.assertAlmostEqual(axis[1], math.cos(yaw))
            cylinder = collision.find('geometry/cylinder')
            self.assertEqual(float(cylinder.get('radius')), .1)
            self.assertEqual(float(cylinder.get('length')), .04)
            # Independent no-slip constraint from the model's axle and position.
            c, s = axis[1], -axis[0]
            rows.append((c / .1, s / .1, (x * s - y * c) / .1))
        # A lateral command is meaningful only with a full-rank planar layout.
        determinants = [a[0] * (b[1] * c[2] - b[2] * c[1])
                        - a[1] * (b[0] * c[2] - b[2] * c[0])
                        + a[2] * (b[0] * c[1] - b[1] * c[0])
                        for a, b, c in itertools.combinations(rows, 3)]
        self.assertGreater(max(map(abs, determinants)), 1)
        plugins = robot.find('litesim3d')
        self.assertEqual(plugins.find('chassis/parameters').get('file'),
                         'package://litesim3d_omni4_drive/config/parameters.yaml')
        self.assertEqual(yaml.safe_load(plugins.find('odometry/overrides').text), {'wheels': before})

    def test_omni_parameters_prefer_checked_out_yaml(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path = root / 'plugins/litesim3d_omni4_drive/config/parameters.yaml'
            path.parent.mkdir(parents=True)
            path.write_text('location: checkout\n')
            installed = Mock(side_effect=AssertionError('checkout must not use installed YAML'))
            with patch.dict(sys.modules, {'ament_index_python.packages': NS(get_package_share_directory=installed)}):
                self.assertEqual(omni4_parameters(root), {'location': 'checkout'})
            installed.assert_not_called()

    def test_omni_parameters_use_ament_share_when_bridge_is_staged_alone(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            share = root / 'install/share/litesim3d_omni4_drive'
            (share / 'config').mkdir(parents=True)
            (share / 'config/parameters.yaml').write_text('location: installed\n')
            installed = Mock(return_value=str(share))
            with patch.dict(sys.modules, {'ament_index_python.packages': NS(get_package_share_directory=installed)}):
                self.assertEqual(omni4_parameters(root / 'staged'), {'location': 'installed'})
            installed.assert_called_once_with('litesim3d_omni4_drive')

    def test_missing_checked_out_omni_yaml_is_not_replaced_by_installed_copy(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'plugins').mkdir()
            installed = Mock(side_effect=AssertionError('missing checkout YAML must fail'))
            with patch.dict(sys.modules, {'ament_index_python.packages': NS(get_package_share_directory=installed)}):
                with self.assertRaises(FileNotFoundError):
                    omni4_parameters(root)
            installed.assert_not_called()

    def test_omni_fixture_rejects_incomplete_or_extra_wheel_bindings(self):
        function = helpers('test_plugins_ros.py', 'add_plugins', 'omni4_robot').omni4_robot
        for count in (3, 5):
            with self.subTest(count=count), self.assertRaises(ValueError):
                function([{'joint': f'wheel_{k}'} for k in range(count)])

    def test_tf_waits_for_raw_callbacks_after_listener_is_ready(self):
        function = helpers('test_m3_ros.py', 'world_tf_pairs').world_tf_pairs
        for count in (2, 4):
            with self.subTest(count=count):
                required = {(f'r{k}/odom', f'r{k}/base_link') for k in range(count)}
                wheels = {(f'r{k}/base_link', f'r{k}/wheel') for k in range(count)}
                delayed = [tf_message(pair) for pair in sorted(required, reverse=True)]
                harness = CallbackHarness([tf_message(*wheels)], delayed)
                self.assertTrue(all(harness.buffer.can_transform(f'r{k}/odom', f'r{k}/lidar_link', None)
                                    for k in range(count)))
                observed = {(t.header.frame_id, t.child_frame_id)
                            for message in harness.messages['tf'] for t in message.transforms}
                # This is the old assertion's failure despite a ready listener.
                self.assertFalse(required <= observed)
                self.assertEqual(function(harness, count), required | wheels)
                self.assertEqual(harness.deliveries, count)
                self.assertEqual(harness.until_calls, 1)

    def test_tf_wait_returns_immediately_when_all_raw_edges_are_present(self):
        function = helpers('test_m3_ros.py', 'world_tf_pairs').world_tf_pairs
        required = {('r0/odom', 'r0/base_link'), ('r1/odom', 'r1/base_link')}
        harness = CallbackHarness([tf_message(*required)])
        self.assertEqual(function(harness, 2), required)
        self.assertEqual(harness.deliveries, 0)

    def test_tf_wait_does_not_confuse_message_count_with_required_edges(self):
        function = helpers('test_m3_ros.py', 'world_tf_pairs').world_tf_pairs
        unrelated = tf_message(('r0/base_link', 'r0/wheel'))
        final = tf_message(('r0/odom', 'r0/base_link'), ('r1/odom', 'r1/base_link'))
        harness = CallbackHarness([unrelated] * 20, [unrelated, final])
        result = function(harness, 2)
        self.assertIn(('r1/odom', 'r1/base_link'), result)
        self.assertEqual(harness.deliveries, 2)

    def test_tf_wait_fails_if_a_robot_edge_never_arrives(self):
        function = helpers('test_m3_ros.py', 'world_tf_pairs').world_tf_pairs
        harness = CallbackHarness([tf_message(('r0/odom', 'r0/base_link'))])
        with self.assertRaisesRegex(AssertionError, 'condition timeout'):
            function(harness, 2)

    def test_tf_wait_rejects_wrong_parent_or_direction(self):
        function = helpers('test_m3_ros.py', 'world_tf_pairs').world_tf_pairs
        for edge in [('r1/other', 'r1/base_link'), ('r1/base_link', 'r1/odom')]:
            with self.subTest(edge=edge):
                harness = CallbackHarness([tf_message(('r0/odom', 'r0/base_link'), edge)])
                with self.assertRaisesRegex(AssertionError, 'condition timeout'):
                    function(harness, 2)

    def test_tf_wait_preserves_cross_namespace_rejection(self):
        function = helpers('test_m3_ros.py', 'world_tf_pairs').world_tf_pairs
        harness = CallbackHarness([tf_message(('r0/odom', 'r0/base_link'),
                                             ('r1/odom', 'r1/base_link'),
                                             ('r0/base_link', 'r1/wheel'))])
        with self.assertRaises(AssertionError):
            function(harness, 2)


if __name__ == '__main__':
    unittest.main()
