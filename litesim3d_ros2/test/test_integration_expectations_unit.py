"""ROS-test fixture/readiness regressions; no ROS context or peers are started.

Compile the actual pure helpers without importing the surrounding integration
modules, whose imports require installed ROS interfaces. Callback delivery is an
explicit deterministic schedule, not a substitute for DDS integration coverage.
"""
import ast
from collections import deque
import copy
import itertools
import importlib.util
import json
import math
from pathlib import Path
import re
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


def helpers(filename, *names, bindings=None):
    path = Path(filename)
    if not path.is_absolute():
        path = Path(__file__).with_name(filename)
    source = ast.parse(path.read_text(), filename=str(path))
    functions = [node for node in ast.walk(source)
                 if isinstance(node, ast.FunctionDef) and node.name in names]
    if {node.name for node in functions} != set(names):
        raise AssertionError(f'missing helpers in {path}: {names}')
    namespace = {'ET': ET, 'math': math, 'yaml': yaml, 'json': json,
                 'Time': lambda: None, 'ns': lambda value: value.sec * 1_000_000_000 + value.nanosec,
                 'stamp': lambda value: value.sec * 1_000_000_000 + value.nanosec}
    namespace.update(bindings or {})
    exec(compile(ast.Module(body=functions, type_ignores=[]), str(path), 'exec'), namespace)
    return NS(**{name: namespace[name] for name in names})


def support_helpers(*names, bindings=None):
    path = ROOT / 'litesim3d_test_support/litesim3d_test_support/composed_models.py'
    if not path.is_file():
        # The isolated bridge staging retains installed test-support Python.
        # Resolve the package without importing its ROS-dependent __init__.
        spec = importlib.util.find_spec('litesim3d_test_support')
        if spec is None or spec.origin is None:
            raise AssertionError('installed composed-model helpers are missing')
        path = Path(spec.origin).with_name('composed_models.py')
    return helpers(path, *names, bindings=bindings)


def joint_command(targets, *, run_id='current', stamp_ns=11):
    return NS(run_id=run_id, header=NS(stamp=NS(sec=0, nanosec=stamp_ns)),
              name=list(targets), mode=[target[0] for target in targets.values()],
              command=[target[1] for target in targets.values()])


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


def scheduled_harness(pending=()):
    """Deliver independent callbacks in a chosen order without time or transport."""
    harness = NS(pending=deque(pending), deliveries=0, until_calls=0)

    def until(predicate, timeout=6):
        harness.until_calls += 1
        while not predicate():
            if not harness.pending:
                raise AssertionError('condition timeout: scheduled callback did not establish readiness')
            harness.pending.popleft()()
            harness.deliveries += 1

    harness.until = until
    return harness


def handoff_harness(*, targets_delivered=True, motion_delivered=True, outcomes=(False, True, True)):
    """Run the real handoff sequence with only its ROS/process boundary replaced."""
    harness = scheduled_harness()
    published, requests = {}, []
    names = ['left_steering_joint', 'right_steering_joint', 'front_left_wheel_joint',
             'rear_left_wheel_joint', 'rear_right_wheel_joint']

    def joints(stamp_ns, moving=False):
        return NS(header=NS(stamp=NS(sec=0, nanosec=stamp_ns)), name=names,
                  position=[.3, .25, .1, .1, .1] if moving else [0.] * 5,
                  velocity=[0., 0., 0., 2., 2.] if moving else [0.] * 5)

    def truth(stamp_ns, moving=False):
        return NS(header=NS(stamp=NS(sec=0, nanosec=stamp_ns)),
                  twist=NS(twist=NS(linear=NS(x=0., y=0.), angular=NS(z=.1 if moving else 0.))))

    harness.messages = {'joints': [joints(10)], 'truth': [truth(10)],
                        'clock': [NS(clock=NS(sec=0, nanosec=10))],
                        'commands': [], 'odom': [], 'status': []}
    harness.status = lambda: {'run_id': 'current'}
    harness.subscribe = Mock()
    harness.environment, harness.log = {}, None
    harness.close, harness.spin = Mock(), Mock()

    def create_publisher(kind, topic, qos):
        published[topic] = []
        def publish(message):
            published[topic].append(message.data)
            if topic == '/wheel_velocity_controller/commands':
                targets = {'rear_left_wheel_joint': (2, 2.0), 'rear_right_wheel_joint': (2, 2.0),
                           'left_steering_joint': (1, .3), 'right_steering_joint': (1, .25)}
                if targets_delivered:
                    harness.pending.append(lambda: harness.messages['commands'].append(joint_command(targets)))
                if motion_delivered:
                    harness.pending.append(lambda: harness.messages['truth'].append(truth(12, True)))
                harness.pending.append(lambda: harness.messages['joints'].append(joints(12, True)))
        return NS(get_subscription_count=lambda: 1, publish=publish)

    def call_async(request):
        requests.append((request.activate_controllers, request.deactivate_controllers))
        if len(requests) == 3:
            harness.pending.append(lambda: harness.messages['joints'].append(joints(13)))
        return NS(done=lambda: True, result=lambda: NS(ok=outcomes[len(requests) - 1]))

    client = NS(wait_for_service=lambda **kwargs: True, call_async=call_async)
    harness.node = NS(create_publisher=create_publisher, create_client=lambda *args: client,
        destroy_client=Mock(),
        get_subscriptions_info_by_topic=lambda topic: [NS(node_name='ackermann_controller')],
        get_publishers_info_by_topic=lambda topic: [NS(node_name='joint_state_broadcaster')])

    class Request:
        STRICT = 2
        def __init__(self, **kwargs):
            self.__dict__.update(kwargs)
            self.timeout = NS(sec=0)

    bindings = {'setup': lambda *args, **kwargs: harness, 'move': Mock(),
                'subprocess': NS(Popen=lambda *args, **kwargs: NS(poll=lambda: 0, returncode=0), STDOUT=None),
                'SwitchController': NS(Request=Request), 'JointCommand': NS(POSITION=1, VELOCITY=2),
                'Float64MultiArray': lambda data: NS(data=data),
                'message_to_ordereddict': lambda value: json.loads(json.dumps(value, default=vars))}
    function = support_helpers('wait_for_joint_targets', 'wait_for_direct_motion',
                               'verify_control_handoff', bindings=bindings).verify_control_handoff
    return harness, published, requests, function


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

    def test_world_tf_listener_waits_after_all_topic_callbacks_and_raw_tf(self):
        function = helpers('test_m3_ros.py', 'world_transforms_ready').world_transforms_ready
        for count in (2, 4):
            with self.subTest(count=count):
                ready = set()
                expected = {(f'r{k}/odom', f'r{k}/lidar_link') for k in range(count)}
                pending = deque(lambda pair=pair: ready.add(pair) for pair in sorted(expected, reverse=True))
                harness = scheduled_harness(pending)
                # A complete raw collector and all data streams do not fill the listener.
                harness.messages = {'tf': [tf_message(*[(f'r{k}/odom', f'r{k}/base_link')
                                                        for k in range(count)])],
                                    'clock': [object()], **{f'r{k}-odom': [object()] for k in range(count)}}
                calls = []
                harness.buffer = NS(can_transform=lambda parent, child, time:
                                    calls.append((parent, child, time)) or (parent, child) in ready)
                self.assertFalse(expected <= ready)
                function(harness, count)
                self.assertEqual(ready, expected)
                self.assertEqual(harness.deliveries, count)
                self.assertTrue(all((parent, child) in expected and time is None
                                    for parent, child, time in calls))

    def test_world_tf_listener_wait_does_not_accept_missing_or_wrong_namespace(self):
        function = helpers('test_m3_ros.py', 'world_transforms_ready').world_transforms_ready
        for ready in ({('r0/odom', 'r0/lidar_link')},
                      {('r0/odom', 'r0/lidar_link'), ('r1/odom', 'r0/lidar_link')},
                      {('r0/odom', 'r0/lidar_link'), ('r1/base_link', 'r1/odom')}):
            with self.subTest(ready=ready):
                harness = scheduled_harness()
                harness.buffer = NS(can_transform=lambda parent, child, time: (parent, child) in ready)
                with self.assertRaisesRegex(AssertionError, 'condition timeout'):
                    function(harness, 2)

    def test_world_tf_listener_wait_is_immediate_when_ready(self):
        function = helpers('test_m3_ros.py', 'world_transforms_ready').world_transforms_ready
        harness = scheduled_harness()
        harness.buffer = NS(can_transform=lambda *args: True)
        function(harness, 4)
        self.assertEqual(harness.deliveries, 0)
        self.assertEqual(harness.until_calls, 1)

    def test_sensor_readiness_requires_real_reader_and_exact_writer(self):
        function = helpers('test_ros.py', 'wait_for_readers').wait_for_readers
        topics = ('/clock', '/points_raw')
        for publishers, subscribers, matches in ((0, 1, 1), (2, 1, 2), (1, 0, 0), (1, 2, 1)):
            with self.subTest(publishers=publishers, subscribers=subscribers, matches=matches):
                harness = scheduled_harness()
                harness.node = NS(count_publishers=lambda topic: publishers,
                                  count_subscribers=lambda topic: subscribers)
                harness.status = lambda: {'ready': 'true',
                    'subscription_matches': json.dumps({topic: matches for topic in topics})}
                with self.assertRaisesRegex(AssertionError, 'condition timeout'):
                    function(harness, *topics)

    def test_sensor_readiness_waits_for_publisher_side_matches_not_ready_flag(self):
        function = helpers('test_ros.py', 'wait_for_readers').wait_for_readers
        counts = {}
        harness = scheduled_harness([lambda: counts.update({'/clock': 1}),
                                     lambda: counts.update({'/points_raw': 2})])
        harness.node = NS(count_publishers=lambda topic: 1,
                          count_subscribers=lambda topic: 2 if topic == '/points_raw' else 1)
        harness.status = lambda: {'ready': 'true', 'subscription_matches': json.dumps(counts)}
        function(harness, '/clock', '/points_raw')
        self.assertEqual(harness.deliveries, 2)
        self.assertEqual(harness.until_calls, 1)

    def test_point_samples_wait_for_five_actual_clouds_and_their_clock(self):
        function = helpers('test_ros.py', 'point_samples').point_samples
        for order in (('points', 'clock'), ('clock', 'points')):
            with self.subTest(order=order):
                harness = scheduled_harness()
                harness.messages = {'points': [], 'clock': []}
                harness.wait_for_readers = Mock()
                clouds = [NS(header=NS(stamp=NS(sec=0, nanosec=i))) for i in range(1, 6)]
                updates = {'points': lambda: harness.messages['points'].extend(clouds),
                           'clock': lambda: harness.messages['clock'].append(NS(clock=NS(sec=0, nanosec=5)))}
                harness.pending.extend(updates[key] for key in order)
                self.assertEqual(function(harness, 5), clouds)
                harness.wait_for_readers.assert_called_once_with('/clock', '/points_raw')
                self.assertEqual(harness.deliveries, 2)

    def test_point_samples_do_not_accept_four_clouds_or_a_stale_clock(self):
        function = helpers('test_ros.py', 'point_samples').point_samples
        for count, clock in ((4, 10), (5, 4)):
            with self.subTest(count=count, clock=clock):
                harness = scheduled_harness()
                harness.wait_for_readers = Mock()
                harness.messages = {'points': [NS(header=NS(stamp=NS(sec=0, nanosec=i)))
                                               for i in range(1, count + 1)],
                                    'clock': [NS(clock=NS(sec=0, nanosec=clock))]}
                with self.assertRaisesRegex(AssertionError, 'condition timeout'):
                    function(harness, 5)

    def test_handoff_target_observer_waits_for_both_independent_command_paths(self):
        function = support_helpers('wait_for_joint_targets').wait_for_joint_targets
        targets = {'rear_left_wheel_joint': (2, 2.0), 'rear_right_wheel_joint': (2, 2.0),
                   'left_steering_joint': (1, .3), 'right_steering_joint': (1, .25)}
        for order in (('wheels', 'steering'), ('steering', 'wheels')):
            with self.subTest(order=order):
                partial = dict(targets)
                partial.update({name: (mode, 0.0) for name, (mode, value) in partial.items()})
                harness = scheduled_harness()
                harness.messages = {'commands': [joint_command(partial, stamp_ns=10)]}

                def update(part):
                    keys = ('rear_left_wheel_joint', 'rear_right_wheel_joint') if part == 'wheels' else (
                        'left_steering_joint', 'right_steering_joint')
                    partial.update({key: targets[key] for key in keys})
                    harness.messages['commands'].append(joint_command(partial))

                harness.pending.extend(lambda part=part: update(part) for part in order)
                observed = function(harness, targets, run_id='current', after=10)
                self.assertEqual(dict(zip(observed.name, zip(observed.mode, observed.command))), targets)
                self.assertEqual(harness.deliveries, 2)
                self.assertEqual(len(harness.messages['commands']), 3)

    def test_handoff_target_observer_rejects_stale_session_and_layout_or_mode_errors(self):
        function = support_helpers('wait_for_joint_targets').wait_for_joint_targets
        targets = {'wheel': (2, 2.0), 'steering': (1, .3)}
        bad = [joint_command(targets, run_id='previous'), joint_command(targets, stamp_ns=10),
               joint_command({'wheel': (1, 2.0), 'steering': (1, .3)}),
               joint_command({'wheel': (2, 2.0)}),
               joint_command({**targets, 'extra': (2, 2.0)}),
               joint_command({'wheel': (2, float('nan')), 'steering': (1, .3)})]
        malformed = joint_command(targets); malformed.command.pop(); bad.append(malformed)
        duplicated = joint_command(targets); duplicated.name[1] = 'wheel'; bad.append(duplicated)
        for message in bad:
            with self.subTest(message=message):
                harness = scheduled_harness()
                harness.messages = {'commands': [message]}
                with self.assertRaisesRegex(AssertionError, 'condition timeout'):
                    function(harness, targets, run_id='current', after=10)

    def test_handoff_target_observer_does_not_publish_or_accept_truth_as_delivery_ack(self):
        function = support_helpers('wait_for_joint_targets').wait_for_joint_targets
        targets = {'wheel': (2, 2.0)}
        harness = scheduled_harness()
        harness.messages = {'commands': [], 'truth': [NS(twist=NS(twist=NS(angular=NS(z=.1))))]}
        # A truth sample (possibly from before the switch) is not wire acceptance.
        with self.assertRaisesRegex(AssertionError, 'condition timeout'):
            function(harness, targets, run_id='current', after=10)
        message = joint_command(targets)
        harness.messages['commands'].append(message)
        self.assertIs(function(harness, targets, run_id='current', after=10), message)
        self.assertEqual(harness.deliveries, 0)

    def test_direct_motion_ignores_old_motion_and_waits_for_fresh_truth(self):
        function = support_helpers('wait_for_direct_motion').wait_for_direct_motion
        for kind in ('ackermann', 'omni3', 'omni4'):
            with self.subTest(kind=kind):
                harness = scheduled_harness()
                def truth(stamp_ns, value):
                    return NS(header=NS(stamp=NS(sec=0, nanosec=stamp_ns)),
                              twist=NS(twist=NS(angular=NS(z=value), linear=NS(y=value))))
                harness.messages = {'truth': [truth(9, .2)]}
                harness.pending.extend([lambda: harness.messages['truth'].append(truth(11, 0)),
                                        lambda: harness.messages['truth'].append(truth(12, .2))])
                observed = function(harness, kind, after=10)
                self.assertIs(observed, harness.messages['truth'][-1])
                self.assertEqual(observed.header.stamp.nanosec, 12)
                self.assertEqual(harness.deliveries, 2)

    def test_direct_motion_preserves_strict_axis_threshold_and_freshness(self):
        function = support_helpers('wait_for_direct_motion').wait_for_direct_motion
        for kind, stamp_ns, angular, lateral in (('ackermann', 10, .2, .2),
                                                ('ackermann', 11, .05, .2),
                                                ('ackermann', 11, 0, .2),
                                                ('omni3', 11, .2, .15),
                                                ('omni4', 11, .2, 0)):
            with self.subTest(kind=kind, stamp_ns=stamp_ns, angular=angular, lateral=lateral):
                harness = scheduled_harness()
                harness.messages = {'truth': [NS(header=NS(stamp=NS(sec=0, nanosec=stamp_ns)),
                    twist=NS(twist=NS(angular=NS(z=angular), linear=NS(y=lateral))))]}
                with self.assertRaisesRegex(AssertionError, 'condition timeout'):
                    function(harness, kind, after=10)

    def test_full_handoff_preserves_conflict_rejection_one_shot_commands_and_restore(self):
        harness, published, requests, function = handoff_harness()
        with tempfile.TemporaryDirectory() as directory:
            function(Path(directory), 'ackermann')
            evidence = json.loads((Path(directory) / 'control-evidence.json').read_text())
        self.assertEqual(published, {'/steering_position_controller/commands': [[.3, .25]],
                                     '/wheel_velocity_controller/commands': [[2., 2.]]})
        direct = ['wheel_velocity_controller', 'steering_position_controller']
        self.assertEqual(requests, [(direct, []), (direct, ['ackermann_controller']),
                                    (['ackermann_controller'], direct)])
        self.assertEqual([item['ok'] for item in evidence['handoff']['switches']], [False, True, True])
        self.assertEqual(evidence['handoff']['phase'], 'complete')
        harness.close.assert_called_once_with()
        harness.spin.assert_called_once_with(.2)

    def test_full_handoff_reports_delivery_separately_from_physics_and_always_closes(self):
        for targets, motion, phase in ((False, True, 'observe_direct_targets'),
                                       (True, False, 'observe_direct_motion')):
            with self.subTest(targets=targets, motion=motion):
                harness, published, requests, function = handoff_harness(
                    targets_delivered=targets, motion_delivered=motion)
                with tempfile.TemporaryDirectory() as directory:
                    with self.assertRaisesRegex(AssertionError, phase):
                        function(Path(directory), 'ackermann')
                    evidence = json.loads((Path(directory) / 'control-evidence.json').read_text())
                self.assertEqual(evidence['handoff']['phase'], phase)
                self.assertEqual([item['ok'] for item in evidence['handoff']['switches']], [False, True])
                self.assertTrue(all(len(values) == 1 for values in published.values()))
                harness.close.assert_called_once_with()

    def test_full_handoff_still_fails_if_conflicting_ownership_is_accepted(self):
        harness, published, requests, function = handoff_harness(outcomes=(True, True, True))
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(AssertionError, 'reject_conflicting_ownership'):
                function(Path(directory), 'ackermann')
        self.assertEqual(len(requests), 1)
        self.assertEqual(published, {})
        harness.close.assert_called_once_with()

    def test_full_handoff_still_rejects_replayed_speed_after_restore(self):
        harness, published, requests, function = handoff_harness()
        def replay(seconds):
            harness.messages['joints'][-1].velocity[-1] = 2.
        harness.spin.side_effect = replay
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(AssertionError, 'restore_chassis_controller'):
                function(Path(directory), 'ackermann')
        self.assertEqual(len(requests), 3)
        harness.close.assert_called_once_with()

    def test_support_helpers_resolve_staged_package_without_importing_ros_harness(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            package = root / 'installed/litesim3d_test_support'
            package.mkdir(parents=True)
            (package / 'composed_models.py').write_text('def sentinel():\n    return 42\n')
            resolver = Mock(return_value=NS(origin=str(package / '__init__.py')))
            with patch.dict(support_helpers.__globals__, ROOT=root / 'staged'), \
                    patch.object(importlib.util, 'find_spec', resolver):
                self.assertEqual(support_helpers('sentinel').sentinel(), 42)
            resolver.assert_called_once_with('litesim3d_test_support')

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


class ChildExitDiagnosticTests(unittest.TestCase):
    def functions(self):
        path = ROOT / 'litesim3d_test_support/litesim3d_test_support/harness.py'
        if not path.is_file():
            spec = importlib.util.find_spec('litesim3d_test_support')
            self.assertIsNotNone(spec)
            path = Path(spec.origin).with_name('harness.py')
        return helpers(path, 'child_failure_report', 'close', bindings={
            're': re, 'signal': NS(SIGINT=2, SIGKILL=9),
            'os': NS(killpg=Mock(side_effect=ProcessLookupError)),
            'time': NS(monotonic=lambda: 0, sleep=Mock())})

    def harness(self, directory, output):
        directory = Path(directory)
        (directory / 'launch.log').write_text(output)
        return NS(directory=directory, process=NS(pid=123, poll=lambda: 0,
                  returncode=0, wait=Mock(), send_signal=Mock()), log=Mock(),
                  listener=Mock(), executor=Mock(), node=Mock(), context=Mock(),
                  graph_snapshot=lambda topics: {}, wall_timeout=lambda seconds: seconds,
                  status=lambda: {})

    def test_failed_child_log_survives_close_assertion_and_cleanup(self):
        output = ('[INFO] [sim_node-4]: process started with pid [321]\n'
                  '[sim_node-4] litesim3d: could not count publishers: invalid context\n'
                  '[camera_node-2] unrelated camera output\n'
                  '[ERROR] [sim_node-4]: process has died [pid 321, exit code 1]\n')
        with tempfile.TemporaryDirectory() as directory:
            harness = self.harness(directory, output)
            with self.assertRaises(AssertionError) as failure:
                self.functions().close(harness, require_clean_children=True)
            self.assertIn('could not count publishers: invalid context', str(failure.exception))
            report = json.loads(str(failure.exception))
            self.assertEqual(report['children']['321']['exit_code'], 1)
            self.assertEqual(report['children']['321']['expected_exit_code'], 0)
            detail = report['failed_child_logs']['321']['log_tail']
            self.assertIn('could not count publishers: invalid context', detail)
            self.assertNotIn('unrelated camera', detail)
            self.assertEqual(json.loads((Path(directory) / 'child-exits.json').read_text()),
                             report['children'])
            for cleanup in (harness.log.close, harness.listener.unregister,
                            harness.executor.shutdown, harness.node.destroy_node,
                            harness.context.shutdown):
                cleanup.assert_called_once()

    def test_expected_nonzero_child_exit_is_still_accepted(self):
        output = ('[INFO] [camera_node-2]: process started with pid [321]\n'
                  '[ERROR] [camera_node-2]: process has died [pid 321, exit code 1]\n')
        with tempfile.TemporaryDirectory() as directory:
            harness = self.harness(directory, output)
            self.functions().close(harness, require_clean_children=True,
                                   expected_child_exits={'321': 1})
            harness.context.shutdown.assert_called_once()

    def test_unobserved_exit_and_empty_child_inventory_still_fail(self):
        for output in ('', '[INFO] [sim_node-4]: process started with pid [321]\n'):
            with self.subTest(output=output), tempfile.TemporaryDirectory() as directory:
                harness = self.harness(directory, output)
                with self.assertRaises(AssertionError):
                    self.functions().close(harness, require_clean_children=True)
                harness.context.shutdown.assert_called_once()

    def test_failure_excerpt_is_bounded_and_exact_child_names_do_not_alias(self):
        children = {'321': {'name': 'sim_node-4', 'exit_code': 1, 'expected_exit_code': 0},
                    '322': {'name': 'sim_node-40', 'exit_code': 0, 'expected_exit_code': 0}}
        output = '[sim_node-4] ' + 'x' * 10000 + '\n[sim_node-4] decisive error\n'
        output += '[sim_node-40] unrelated\n'
        report = json.loads(self.functions().child_failure_report(output, children))
        self.assertEqual(set(report['failed_child_logs']), {'321'})
        detail = report['failed_child_logs']['321']['log_tail']
        self.assertEqual(len(detail), 6000)
        self.assertIn('decisive error', detail)
        self.assertNotIn('unrelated', detail)


if __name__ == '__main__':
    unittest.main()
