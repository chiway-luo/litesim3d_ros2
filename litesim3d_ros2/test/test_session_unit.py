"""Session state-machine units; no ROS context, DDS peer or subprocess is started.

The real Session methods run against explicitly injected graph/process boundaries.
Temporary files are deliberately small unit fixtures, not substituted runtime assets.
"""
import argparse
import collections
import importlib.machinery
import importlib.util
import json
from pathlib import Path
import signal
import subprocess
import threading
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest
import yaml


@pytest.fixture
def module():
    path = Path(__file__).resolve().parents[1] / 'scripts/sim_session'
    loader = importlib.machinery.SourceFileLoader('sim_session_unit', str(path))
    spec = importlib.util.spec_from_loader(loader.name, loader)
    result = importlib.util.module_from_spec(spec)
    loader.exec_module(result)
    return result


@pytest.fixture
def session(module, tmp_path):
    node = module.Session.__new__(module.Session)
    node.args = NS(timeout=2, sensor_reliable=False)
    node.config = tmp_path / 'runtime.yaml'
    (tmp_path / 'robot.urdf').write_text('<robot name="unit"/>')
    node.data = {'robot': {'urdf': 'robot.urdf'}, 'lidars': [{'topic': 'points_raw'}],
                 'imus': [{'topic': '/imu'}]}
    node.config.write_text(yaml.safe_dump(node.data))
    node.directory = tmp_path / 'output'
    node.directory.mkdir()
    node.members = []
    node.input_topics = {'/clock', '/odom', '/joint_states', '/points_raw', '/imu'}
    node.member_topics = {}
    node.owned_names = {'sim_session', 'litesim3d', 'robot_state_publisher'}
    node.observers = {'observer'}
    node.processes = []
    node.condition = threading.Condition()
    node.mutation = threading.Lock()
    node.status = {}
    node.status_sequence = 0
    node.run_id = 'old-run'
    node.epoch = 0
    node.transitioning = False
    node.failed = False
    node.stopping = False
    node.cache = collections.OrderedDict()
    node.record = Mock()
    node.get_subscriptions_info_by_topic = Mock(return_value=[])
    node.get_publishers_info_by_topic = Mock(return_value=[])
    return node


def pose(values=(1., 2., 3., 0., 0., 0., 2.)):
    return NS(position=NS(**dict(zip('xyz', values[:3]))),
              orientation=NS(**dict(zip('xyzw', values[3:]))))


def request(**overrides):
    return NS(**dict({'request_id': 'reset-1', 'expected_run_id': 'old-run',
                      'set_initial_pose': False, 'initial_pose': pose()}, **overrides))


def response():
    return NS(success=False, message='', run_id='')


def endpoint(name='consumer', namespace='/', topic_type='sensor_msgs/msg/PointCloud2', qos='qos'):
    return NS(node_name=name, node_namespace=namespace, topic_type=topic_type, qos_profile=qos)


def test_resolve_and_resource_relocation(module, monkeypatch, tmp_path):
    monkeypatch.setattr(module, 'get_package_prefix', lambda package: '/prefix/' + package)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda package: '/share/' + package)
    assert module.executable('pkg', 'tool') == '/prefix/pkg/lib/pkg/tool'
    assert module.resolve('package://pkg/mesh/a.stl', tmp_path) == Path('/share/pkg/mesh/a.stl')
    assert module.resolve('file://robot.urdf', tmp_path) == tmp_path / 'robot.urdf'
    assert module.resolve('relative/../robot.urdf', tmp_path) == tmp_path / 'robot.urdf'
    data = {'robot': {'urdf': 'robot.urdf'}, 'lidars': [{'pattern_file': 'pattern.csv'}, {}],
            'entities': [{'geometry': {'type': 'mesh', 'uri': 'a.dae'},
                          'visual': {'type': 'mesh', 'uri': 'package://pkg/b.dae'},
                          'collision': {'type': 'box'}, 'sensor_geometry': {'type': 'mesh', 'uri': 'c.dae'}}, {}]}
    assert module.resolve_config_resources(data, tmp_path) is data
    assert data['robot']['urdf'] == str(tmp_path / 'robot.urdf')
    assert data['lidars'][0]['pattern_file'] == str(tmp_path / 'pattern.csv')
    assert data['entities'][0]['visual']['uri'] == '/share/pkg/b.dae'
    assert data['entities'][0]['sensor_geometry']['uri'] == str(tmp_path / 'c.dae')


def initialize(module, monkeypatch, tmp_path, members):
    monkeypatch.setattr(module.Node, '__init__', lambda self, name: None)
    subscription = Mock(return_value='subscription')
    service = Mock(return_value='service')
    monkeypatch.setattr(module.Session, 'create_subscription', subscription)
    monkeypatch.setattr(module.Session, 'create_service', service)
    config = tmp_path / 'config.yaml'
    config.write_text(yaml.safe_dump({'robot': {'urdf': 'robot.urdf'}, 'lidars': [{}], 'imus': [{}]}))
    member_file = tmp_path / 'members.json'
    member_file.write_text(json.dumps(members))
    args = NS(config=str(config), output=str(tmp_path / 'session'),
              members=str(member_file), observer=['z-monitor', 'a-monitor'])
    return module.Session(args), subscription, service


def test_initialization_validates_members_and_records_creation(module, monkeypatch, tmp_path):
    node, sub, srv = initialize(module, monkeypatch, tmp_path, [
        {'name': 'consumer', 'argv': ['consumer', '--flag'], 'ready_topics': ['/clock', '/points_raw']}])
    assert node.member_topics == {'consumer': {'/clock', '/points_raw'}}
    assert '/imu/data_raw' in node.input_topics
    assert node.transitioning and not node.failed and not node.stopping
    assert node.owned_names == {'consumer', 'sim_session', 'litesim3d', 'robot_state_publisher'}
    assert sub.call_args.args[1:3] == ('/sim/status', node.on_status)
    assert srv.call_args.args[1:3] == ('/sim/reset', node.reset)
    row = json.loads((node.directory / 'session.jsonl').read_text())
    assert row['event'] == 'created'
    assert row['epoch'] == 0 and row['run_id'] == ''
    assert row['observers'] == ['a-monitor', 'z-monitor']


@pytest.mark.parametrize('members,match', [
    ({}, 'bounded list'), ([{}] * 33, 'bounded list'),
    ([1], 'name and nonempty argv'), ([{'name': 'a'}], 'name and nonempty argv'),
    ([{'name': 'a', 'argv': []}], 'name and nonempty argv'),
    ([{'name': 'a', 'argv': 'echo'}], 'name and nonempty argv'),
    ([{'name': 'a', 'argv': ['echo'], 'extra': 1}], 'name and nonempty argv'),
    ([{'name': 'a', 'argv': ['']}], 'nonempty strings'),
    ([{'name': 'a', 'argv': [1]}], 'nonempty strings'),
    ([{'name': '', 'argv': ['echo']}], 'unique'),
    ([{'name': 1, 'argv': ['echo']}], 'unique'),
    ([{'name': 'a', 'argv': ['echo']}] * 2, 'unique'),
    ([{'name': 'a', 'argv': ['echo'], 'ready_topics': 'clock'}], 'configured'),
    ([{'name': 'a', 'argv': ['echo'], 'ready_topics': ['/clock'] * 65}], 'configured'),
    ([{'name': 'a', 'argv': ['echo'], 'ready_topics': ['/not_configured']}], 'configured'),
])
def test_initialization_rejects_invalid_members(module, monkeypatch, tmp_path, members, match):
    with pytest.raises(ValueError, match=match):
        initialize(module, monkeypatch, tmp_path, members)


def test_status_and_record_are_atomic_and_append_only(module, session):
    session.on_status(NS(status=[]))
    assert session.status_sequence == 0
    session.on_status(NS(status=[NS(values=[NS(key='ready', value='true'), NS(key='run_id', value='r2')])]))
    assert session.status == {'ready': 'true', 'run_id': 'r2'}
    assert session.status_sequence == 1
    module.Session.record(session, 'α', extra=3)
    module.Session.record(session, 'next', epoch=7)
    rows = [json.loads(row) for row in (session.directory / 'session.jsonl').read_text().splitlines()]
    assert [r['event'] for r in rows] == ['α', 'next']
    assert rows[0]['extra'] == 3 and rows[0]['epoch'] == 0
    assert rows[1]['epoch'] == 7 and rows[1]['run_id'] == 'old-run'
    assert all(isinstance(row['wall_time_ns'], int) for row in rows)


def test_spawn_injects_epoch_environment_and_closes_parent_log(module, session, monkeypatch):
    proc = NS(pid=123)
    popen = Mock(return_value=proc)
    monkeypatch.setattr(module.subprocess, 'Popen', popen)
    monkeypatch.setenv('UNIT_PARENT_ENV', 'retained')
    assert session.spawn('consumer', ['consumer', '--flag'], session.directory) is proc
    call = popen.call_args
    assert call.args == (['consumer', '--flag'],)
    assert call.kwargs['env']['LWS_RUN_ID'] == 'old-run'
    assert call.kwargs['env']['LWS_RUN_DIR'] == str(session.directory)
    assert call.kwargs['env']['UNIT_PARENT_ENV'] == 'retained'
    assert call.kwargs['start_new_session'] is True
    assert call.kwargs['stderr'] == subprocess.STDOUT
    assert call.kwargs['stdout'].closed
    assert session.processes == [('consumer', proc)]
    session.record.assert_called_once_with('spawn', name='consumer', pid=123, argv=['consumer', '--flag'])


def test_spawn_failure_closes_log_without_registering_process(module, session, monkeypatch):
    captured = []
    def fail(*args, **kwargs):
        captured.append(kwargs['stdout'])
        raise OSError('cannot spawn')
    monkeypatch.setattr(module.subprocess, 'Popen', fail)
    with pytest.raises(OSError, match='cannot spawn'):
        session.spawn('bad', ['bad'], session.directory)
    assert captured[0].closed and not session.processes
    session.record.assert_not_called()


def test_wait_startup_poll_success_and_error_boundaries(module, session, monkeypatch):
    session.processes = [('alive', NS(poll=lambda: None))]
    session.wait_startup(lambda: True, -1, 'already ready')
    session.processes = [('exited', NS(poll=lambda: 0))]
    with pytest.raises(RuntimeError, match='required process exited.*ready'):
        session.wait_startup(lambda: True, 99, 'ready')
    session.processes = []
    monkeypatch.setattr(module.time, 'monotonic', lambda: 10)
    with pytest.raises(TimeoutError, match='consumer readiness timed out: match'):
        session.wait_startup(lambda: False, 10, 'match')
    predicate = Mock(side_effect=[False, True])
    session.condition = Mock()
    session.condition.__enter__ = Mock(return_value=None)
    session.condition.__exit__ = Mock(return_value=None)
    session.wait_startup(predicate, 11, 'soon')
    session.condition.wait.assert_called_once_with(.02)


def test_remember_member_topics_requires_owned_root_namespace(session):
    session.member_topics = {'consumer': {'/odom'}}
    session.get_subscriptions_info_by_topic.side_effect = lambda topic: [
        endpoint('consumer'), endpoint('other'), endpoint('consumer', '/nested')]
    session.remember_member_topics()
    assert session.member_topics == {'consumer': session.input_topics | {'/odom'}}


@pytest.mark.parametrize('mode', ['absent', 'missing_service', 'unknown', 'empty', 'bad_namespace', 'complete'])
def test_member_graph_discovery_is_conservative(module, session, mode):
    session.members = [{'name': 'consumer'}]
    session.get_service_names_and_types_by_node = Mock(return_value=[('/consumer/get_parameters', ['type'])])
    if mode == 'absent':
        session.get_service_names_and_types_by_node.side_effect = module.NodeNameNonExistentError('consumer', '/')
    elif mode == 'missing_service':
        session.get_service_names_and_types_by_node.return_value = []
    elif mode != 'complete':
        name, namespace = {'unknown': ('_NODE_NAME_UNKNOWN_', '/'), 'empty': ('', '/'),
                           'bad_namespace': ('consumer', 'relative')}[mode]
        session.get_subscriptions_info_by_topic.return_value = [endpoint(name, namespace)]
    assert session.member_graph_complete() is (mode == 'complete')


def test_check_participants_allows_readonly_observers_but_not_controllers(session):
    session.get_subscriptions_info_by_topic.return_value = [endpoint('observer'), endpoint('litesim3d')]
    session.check_participants()
    session.get_publishers_info_by_topic.return_value = [endpoint('observer')]
    with pytest.raises(RuntimeError, match='uncoordinated participants: observer'):
        session.check_participants()
    session.get_subscriptions_info_by_topic.return_value = [endpoint('z-unknown'), endpoint('a-unknown')]
    with pytest.raises(RuntimeError, match='a-unknown, observer, z-unknown'):
        session.check_participants()


@pytest.mark.parametrize('returncodes,forced,raises', [([0, -signal.SIGINT], False, False),
    ([4, 0], False, True), ([None, None], False, False), ([None], True, True)])
def test_stop_owned_reverses_order_and_reports_unclean_exit(module, session, monkeypatch,
                                                          returncodes, forced, raises):
    signals = []
    monkeypatch.setattr(module.os, 'killpg', lambda pid, sig: signals.append((pid, sig)))
    processes = []
    for index, code in enumerate(returncodes):
        proc = Mock(pid=100 + index, returncode=code)
        proc.poll.return_value = code
        def wait(timeout, proc=proc):
            if forced and timeout == 5:
                raise subprocess.TimeoutExpired('child', 5)
            proc.returncode = -signal.SIGKILL if forced else -signal.SIGINT
            return proc.returncode
        proc.wait.side_effect = wait
        processes.append((str(index), proc))
    session.processes = processes[:]
    if raises:
        with pytest.raises(RuntimeError, match='unclean process exit'):
            session.stop_owned()
    else:
        session.stop_owned()
    assert session.processes == []
    assert [call.kwargs['name'] for call in session.record.call_args_list] == list(reversed([n for n, _ in processes]))
    assert all(call.kwargs['forced'] is forced for call in session.record.call_args_list)
    expected = []
    for _, proc in reversed(processes):
        if proc.poll.return_value is None:
            expected.append((proc.pid, signal.SIGINT))
            if forced:
                expected.append((proc.pid, signal.SIGKILL))
    assert signals == expected


@pytest.mark.parametrize('values,message', [
    ((float('nan'), 0., 0., 0., 0., 0., 1.), 'non-finite'),
    ((0., 0., 0., 0., float('inf'), 0., 1.), 'non-finite'),
    ((0., 0., 0., 0., 0., 0., 0.), 'zero quaternion')])
def test_validate_pose_rejects_before_running_preflight(module, session, monkeypatch, values, message):
    run = Mock()
    monkeypatch.setattr(module.subprocess, 'run', run)
    with pytest.raises(ValueError, match=message):
        session.validate_pose(request(initial_pose=pose(values)))
    run.assert_not_called()
    assert not (session.directory / 'reset-candidate.yaml').exists()


@pytest.mark.parametrize('exitcode', [0, 7])
def test_validate_pose_normalizes_and_preflights_transaction_candidate(module, session, monkeypatch, exitcode):
    monkeypatch.setattr(module, 'get_package_prefix', lambda _: '/core')
    run = Mock(return_value=NS(returncode=exitcode, stdout=b'preflight diagnostic\n'))
    monkeypatch.setattr(module.subprocess, 'run', run)
    original = session.config.read_text()
    req = request(initial_pose=pose((1., 2., 3., 0., 2., 0., 2.)))
    if exitcode:
        with pytest.raises(ValueError, match='previous epoch retained'):
            session.validate_pose(req)
    else:
        values = session.validate_pose(req)
        assert values[:3] == [1., 2., 3.]
        assert sum(v*v for v in values[3:]) == pytest.approx(1.)
    candidate = yaml.safe_load((session.directory / 'reset-candidate.yaml').read_text())
    assert candidate['robot']['initial_pose']['xyz'] == [1., 2., 3.]
    assert candidate['robot']['initial_pose']['rpy'][1] == pytest.approx(1.57079632679)
    assert candidate['robot']['urdf'] == str(session.config.parent / 'robot.urdf')
    assert session.config.read_text() == original
    assert (session.directory / 'preflight.log').read_bytes() == b'preflight diagnostic\n'
    assert run.call_args.args[0] == ['/core/bin/sim_headless', str(session.directory / 'reset-candidate.yaml'),
                                      '1', str(session.directory / 'preflight.json')]
    assert run.call_args.kwargs['timeout'] == 2


def reset_boundaries(session):
    events = []
    session.check_participants = lambda: events.append('check')
    session.remember_member_topics = lambda: events.append('remember')
    session.stop_owned = lambda: events.append('stop')
    def start(override):
        events.append(('start', override))
        session.run_id = 'new-run'
        session.transitioning = False
    session.start_epoch = start
    session.validate_pose = Mock(return_value=[1., 2., 3., 0., 0., 0., 1.])
    return events


def test_reset_success_is_idempotent_even_after_stopping(session):
    events = reset_boundaries(session)
    req = request(set_initial_pose=True)
    result = session.reset(req, response())
    assert result.success and result.run_id == 'new-run'
    assert events == ['check', 'remember', 'stop', ('start', session.validate_pose.return_value)]
    assert session.cache['reset-1'][1] == (True, 'new epoch ready and paused', 'new-run')
    session.stopping = True
    cached = session.reset(req, response())
    assert vars(cached) == vars(result)
    assert len(events) == 4
    assert session.mutation.acquire(blocking=False)
    session.mutation.release()


def test_reset_cache_rejects_same_id_with_different_payload(session):
    reset_boundaries(session)
    session.reset(request(), response())
    result = session.reset(request(expected_run_id='different'), response())
    assert not result.success and 'reused with different payload' in result.message
    assert len(session.cache) == 1


def test_reset_without_pose_skips_validation(session):
    events = reset_boundaries(session)
    assert session.reset(request(), response()).success
    session.validate_pose.assert_not_called()
    assert events[-1] == ('start', None)


@pytest.mark.parametrize('flag', ['failed', 'stopping', 'transitioning'])
def test_reset_rejects_unready_states_without_mutation(session, flag):
    events = reset_boundaries(session)
    setattr(session, flag, True)
    result = session.reset(request(), response())
    assert not result.success and result.message == 'session not ready'
    assert not events and not session.cache and result.run_id == 'old-run'


@pytest.mark.parametrize('request_id,full', [('', False), ('x' * 129, False), ('valid', True)])
def test_reset_rejects_invalid_or_full_request_cache(session, request_id, full):
    events = reset_boundaries(session)
    if full:
        session.cache.update((str(i), None) for i in range(4096))
    result = session.reset(request(request_id=request_id), response())
    assert not result.success and 'invalid request_id or full request cache' == result.message
    assert not events and result.run_id == 'old-run'


def test_reset_stale_expected_run_and_participant_failure_leave_epoch_intact(session):
    events = reset_boundaries(session)
    result = session.reset(request(expected_run_id='stale'), response())
    assert not result.success and 'stale or missing' in result.message
    assert not events and not session.failed and not session.transitioning
    session.check_participants = Mock(side_effect=RuntimeError('unknown consumer'))
    result = session.reset(request(), response())
    assert not result.success and result.message == 'unknown consumer'
    assert not session.failed and not session.transitioning and not session.cache
    session.record.assert_called_with('reset_rejected', reason='unknown consumer')


@pytest.mark.parametrize('phase', ['preflight', 'stop', 'start'])
def test_reset_preflight_retains_old_epoch_and_restart_failures_poison_session(session, phase):
    events = reset_boundaries(session)
    if phase == 'preflight':
        session.validate_pose.side_effect = ValueError('invalid pose')
    elif phase == 'stop':
        session.stop_owned = Mock(side_effect=RuntimeError('stop failed'))
    else:
        session.start_epoch = Mock(side_effect=RuntimeError('start failed'))
    result = session.reset(request(set_initial_pose=True), response())
    assert not result.success and not session.cache and result.run_id == 'old-run'
    assert session.failed is (phase != 'preflight')
    assert session.transitioning is (phase != 'preflight')
    if phase == 'preflight':
        assert events == ['check']
    session.record.assert_called_with('reset_rejected' if phase == 'preflight' else 'reset_failed',
                                      reason=result.message)


def test_reset_lock_contention_returns_without_releasing_foreign_lock(session):
    session.mutation = Mock()
    session.mutation.acquire.return_value = False
    result = session.reset(request(), response())
    assert not result.success and result.message == 'reset already in progress'
    session.mutation.acquire.assert_called_once_with(timeout=.5)
    session.mutation.release.assert_not_called()


def test_start_epoch_writes_parameters_and_waits_for_new_run(module, session, monkeypatch):
    monkeypatch.setattr(module, 'get_package_prefix', lambda package: '/prefix/' + package)
    session.members = [{'name': 'consumer', 'argv': ['consumer', '--flag']}]
    calls = []
    def spawn(name, argv, directory):
        calls.append((name, argv, directory))
        if name == 'litesim3d':
            session.status.update(ready='true', run_id='new-run', time_ns='0')
    session.spawn = spawn
    session.wait_members = Mock()
    override = [1., 2., 3., 0., 0., 0., 1.]
    session.transitioning = True
    session.start_epoch(override)
    assert session.epoch == 1 and session.run_id == 'new-run' and not session.transitioning
    directory = session.directory / 'epoch-0001'
    assert [call[0] for call in calls] == ['robot_state_publisher', 'litesim3d', 'consumer']
    rsp = yaml.safe_load((directory / 'rsp.yaml').read_text())['/**']['ros__parameters']
    assert rsp == {'robot_description': '<robot name="unit"/>', 'use_sim_time': True, 'publish_frequency': 200.}
    sim = yaml.safe_load((directory / 'sim.yaml').read_text())['/**']['ros__parameters']
    assert sim['initial_pose_override'] == override and sim['start_paused'] is True
    assert sim['config'] == str(session.config) and sim['sensor_reliable'] is False
    assert sim['manifest_path'] == str(directory / 'manifest.json')
    assert calls[-1][1] == ['consumer', '--flag']
    session.wait_members.assert_called_once()
    session.record.assert_called_with('ready', time_ns='0')


@pytest.mark.parametrize('mode', ['exit', 'timeout', 'wait_then_ready'])
def test_start_epoch_startup_failure_and_condition_wait(module, session, monkeypatch, mode):
    monkeypatch.setattr(module, 'get_package_prefix', lambda package: '/prefix/' + package)
    session.spawn = Mock()
    session.wait_members = Mock()
    now = iter([0., 3.])
    monkeypatch.setattr(module.time, 'monotonic', lambda: next(now))
    if mode == 'exit':
        session.processes = [('sim', NS(poll=lambda: 8))]
        with pytest.raises(RuntimeError, match='required process exited during startup'):
            session.start_epoch()
    elif mode == 'timeout':
        with pytest.raises(TimeoutError, match='new epoch readiness timed out'):
            session.start_epoch()
    else:
        monkeypatch.setattr(module.time, 'monotonic', lambda: 0.)
        class Condition:
            def __enter__(self):
                return self
            def __exit__(self, *args):
                pass
            def wait(self, seconds):
                assert seconds == .05
                session.status.update(ready='true', run_id='new-run')
        session.condition = Condition()
        session.start_epoch()
        assert session.run_id == 'new-run'
    sim = yaml.safe_load((session.directory / 'epoch-0001/sim.yaml').read_text())['/**']['ros__parameters']
    assert 'initial_pose_override' not in sim


def member_boundaries(session, future):
    session.members = [{'name': 'consumer'}]
    session.member_topics = {'consumer': {'/points_raw'}}
    client = Mock()
    client.service_is_ready.return_value = True
    client.call_async.return_value = future
    session.create_client = Mock(return_value=client)
    session.destroy_client = Mock()
    session.member_graph_complete = Mock(return_value=True)
    session.remember_member_topics = Mock()
    return client


def test_wait_members_empty_inventory_still_remembers_topics(session):
    session.remember_member_topics = Mock()
    session.wait_members(10.)
    session.remember_member_topics.assert_called_once_with()


@pytest.mark.parametrize('mode', ['ready', 'missing_publisher', 'duplicate_publisher', 'missing_owned',
                                 'wrong_type', 'bad_qos', 'wrong_run', 'few_matches', 'graph_stale'])
def test_wait_members_requires_real_executor_graph_qos_and_match_counts(module, session, monkeypatch, mode):
    future = Mock()
    future.done.return_value = True
    client = member_boundaries(session, future)
    session.get_publishers_info_by_topic.return_value = [endpoint('litesim3d')]
    session.get_subscriptions_info_by_topic.return_value = [endpoint('consumer'), endpoint('observer')]
    if mode == 'missing_publisher':
        session.get_publishers_info_by_topic.return_value = [endpoint('litesim3d', '/other')]
    if mode == 'duplicate_publisher':
        session.get_publishers_info_by_topic.return_value *= 2
    if mode == 'missing_owned':
        session.get_subscriptions_info_by_topic.return_value = [endpoint('consumer', '/other')]
    if mode == 'wrong_type':
        session.get_subscriptions_info_by_topic.return_value[0].topic_type = 'wrong/msg/Type'
    compatibility = module.QoSCompatibility.ERROR if mode == 'bad_qos' else module.QoSCompatibility.OK
    monkeypatch.setattr(module, 'qos_check_compatible', lambda *args: (compatibility, ''))
    calls = []
    def wait(predicate, deadline, detail):
        calls.append(detail)
        if 'graph ownership' in detail:
            assert predicate() is False
            if mode != 'graph_stale':
                session.status_sequence += 1
                session.status.update(run_id=session.run_id)
        if 'subscription' in detail:
            session.status.update(run_id='stale' if mode == 'wrong_run' else session.run_id,
                                  subscription_matches=json.dumps({'/points_raw': 1 if mode == 'few_matches' else 2}))
        if not predicate():
            raise TimeoutError(detail)
    session.wait_startup = wait
    if mode in {'wrong_type', 'bad_qos'}:
        with pytest.raises(RuntimeError, match='incompatible consumer subscription: consumer /points_raw'):
            session.wait_members(10.)
    elif mode != 'ready':
        with pytest.raises(TimeoutError):
            session.wait_members(10.)
    else:
        session.wait_members(10.)
        session.record.assert_called_with('member_ready', name='consumer', topics=['/points_raw'])
        future.result.assert_called_once()
    assert calls[0] == 'consumer parameter service'
    session.destroy_client.assert_called_once_with(client)
    session.create_client.assert_called_once()


def test_wait_members_executor_probe_retry_cancels_pending_request(module, session, monkeypatch):
    pending = Mock()
    pending.done.return_value = False
    ready = Mock()
    ready.done.return_value = True
    client = member_boundaries(session, pending)
    client.call_async.side_effect = [pending, ready]
    session.member_topics = {}
    monkeypatch.setattr(module.time, 'monotonic', lambda: 1.)
    def wait(predicate, deadline, detail):
        if 'executor' in detail:
            # A read-only request may not have a matched response endpoint yet.
            return
        if 'graph ownership' in detail:
            assert not predicate()
            session.status_sequence += 1
            session.status['run_id'] = session.run_id
        assert predicate()
    session.wait_startup = wait
    session.wait_members(9.)
    client.remove_pending_request.assert_called_once_with(pending)
    pending.cancel.assert_called_once_with()
    ready.result.assert_called_once()
    session.record.assert_any_call('member_executor_probe_retry', name='consumer', attempt=1)
    session.destroy_client.assert_called_once_with(client)


@pytest.mark.parametrize('mode', ['deadline', 'future_error', 'service_error'])
def test_wait_members_probe_errors_always_destroy_client(module, session, monkeypatch, mode):
    future = Mock()
    future.done.return_value = mode == 'future_error'
    future.result.side_effect = RuntimeError('executor response failed')
    client = member_boundaries(session, future)
    monkeypatch.setattr(module.time, 'monotonic', lambda: 10.)
    def wait(predicate, deadline, detail):
        if mode == 'service_error':
            raise TimeoutError('parameter unavailable')
        return
    session.wait_startup = wait
    with pytest.raises((TimeoutError, RuntimeError)):
        session.wait_members(10.)
    if mode == 'deadline':
        client.remove_pending_request.assert_called_once_with(future)
        future.cancel.assert_called_once()
    session.destroy_client.assert_called_once_with(client)


@pytest.mark.parametrize('timeout', ['0', '121', 'nan', 'inf'])
def test_main_rejects_invalid_timeout_before_ros_initialization(module, monkeypatch, timeout):
    monkeypatch.setattr(module.sys if hasattr(module, 'sys') else __import__('sys'), 'argv',
                        ['sim_session', '--config', 'unused', '--output', 'unused', '--timeout', timeout])
    init = Mock()
    monkeypatch.setattr(module.rclpy, 'init', init)
    with pytest.raises(SystemExit) as error:
        module.main()
    assert error.value.code == 2
    init.assert_not_called()


@pytest.mark.parametrize('mode', ['clean', 'process_exit', 'start_error', 'cleanup_error',
                                 'interrupt', 'external_shutdown', 'spin_interrupt', 'spin_shutdown'])
def test_main_lifecycle_with_injected_ros_executor_and_process_boundaries(module, monkeypatch, mode):
    monkeypatch.setattr(__import__('sys'), 'argv', ['sim_session', '--config', 'unit', '--output', 'unit'])
    callbacks = {}
    monkeypatch.setattr(module.signal, 'signal', lambda signum, callback: callbacks.setdefault(signum, callback))
    node = NS(mutation=threading.Lock(), transitioning=False, failed=False, stopping=False,
              processes=[], record=Mock(), stop_owned=Mock(), destroy_node=Mock())
    node.start_epoch = Mock()
    if mode == 'process_exit':
        node.processes = [('consumer', NS(poll=lambda: 7, returncode=7))]
    if mode == 'start_error':
        node.start_epoch.side_effect = RuntimeError('startup failed')
    if mode == 'cleanup_error':
        node.stop_owned.side_effect = RuntimeError('cleanup failed')
    if mode == 'interrupt':
        node.start_epoch.side_effect = KeyboardInterrupt()
    if mode == 'external_shutdown':
        node.start_epoch.side_effect = module.ExternalShutdownException()
    monkeypatch.setattr(module, 'Session', Mock(return_value=node))
    executor = Mock()
    if mode == 'spin_interrupt':
        executor.spin.side_effect = KeyboardInterrupt()
    if mode == 'spin_shutdown':
        executor.spin.side_effect = module.ExternalShutdownException()
    monkeypatch.setattr(module, 'MultiThreadedExecutor', Mock(return_value=executor))
    class Thread:
        def __init__(self, target, daemon):
            assert daemon
            self.target = target
        def start(self):
            self.target()
        def join(self, timeout):
            assert timeout == 3
    monkeypatch.setattr(module.threading, 'Thread', Thread)
    monkeypatch.setattr(module.rclpy, 'init', Mock())
    monkeypatch.setattr(module.rclpy, 'shutdown', Mock())
    # Execute one control-loop iteration, then its deterministic stop condition.
    monkeypatch.setattr(module.rclpy, 'ok', Mock(side_effect=[True, False, True]))
    def sleep(seconds):
        assert seconds == .05
        callbacks[signal.SIGTERM](signal.SIGTERM, None)
    monkeypatch.setattr(module.time, 'sleep', sleep)
    expected = int(mode in {'process_exit', 'start_error', 'cleanup_error'})
    assert module.main() == expected
    assert node.stopping
    node.stop_owned.assert_called_once()
    node.record.assert_any_call('exit', returncode=expected)
    executor.add_node.assert_called_once_with(node)
    executor.shutdown.assert_called_once_with(timeout_sec=3)
    node.destroy_node.assert_called_once()
    if mode == 'process_exit':
        node.record.assert_any_call('required_process_exit', name='consumer', returncode=7)
    if mode == 'start_error':
        node.record.assert_any_call('failure', reason='startup failed')
    if mode == 'cleanup_error':
        node.record.assert_any_call('cleanup_failure', reason='cleanup failed')
