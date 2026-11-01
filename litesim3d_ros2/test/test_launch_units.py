"""Launch-description units: construct actions, never execute processes or DDS."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
EXAMPLES = [
    ('dual_mid360/dual_mid360_bringup/launch/bringup.launch.py', 'diff_drive_controller', 'workshop.sdf'),
    ('ackermann/ackermann_sim/launch/ackermann.launch.py', 'ackermann_controller', 'room.sdf'),
    ('omni3/omni3_sim/launch/omni3.launch.py', 'omni_controller', 'room.sdf'),
    ('omni4/omni4_sim/launch/omni4.launch.py', 'omni_controller', 'room.sdf'),
    ('wheel_car_4w/wheel_car_4w_sim/launch/wheel_car_4w.launch.py', 'diff_drive_controller', 'office.sdf'),
    ('drone_3d/drone_3d_sim/launch/drone_3d.launch.py', 'body_velocity_controller', 'drone_arena.sdf'),
]
LEGACY = ['wheel_car_4w/wheel_car_4w_sim/launch/legacy_twist.launch.py',
          'drone_3d/drone_3d_sim/launch/legacy_twist.launch.py']


def launch_source(path):
    relative = Path(path)
    if relative.parts[0] == 'litesim3d_ros2':
        return PACKAGE_ROOT.joinpath(*relative.parts[1:])
    repository = PACKAGE_ROOT.parent
    if (repository / 'examples').is_dir():
        # Coverage runs must exercise checkout sources, not an older install.
        return repository / relative
    # Independent-package validation installs every package before its test
    # pass, then restores only the package under test into current-source.
    return Path(get_package_share_directory(relative.parts[2])).joinpath(*relative.parts[3:])


def load(path):
    path = launch_source(path)
    spec = importlib.util.spec_from_file_location('launch_unit_' + path.parent.parent.name + path.stem, path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def test_launch_sources_use_checkout_paths_without_installed_fallback(monkeypatch, tmp_path):
    package = tmp_path / 'litesim3d_ros2'
    (tmp_path / 'examples').mkdir()
    monkeypatch.setitem(globals(), 'PACKAGE_ROOT', package)
    installed = Mock(side_effect=AssertionError('checkout tests must not use installed launch files'))
    monkeypatch.setitem(globals(), 'get_package_share_directory', installed)
    assert launch_source('litesim3d_ros2/launch/demo.launch.py') == package / 'launch/demo.launch.py'
    for filename in [entry[0] for entry in EXAMPLES] + LEGACY:
        assert launch_source('examples/' + filename) == tmp_path / 'examples' / filename
    installed.assert_not_called()


def test_launch_sources_use_staged_bridge_and_installed_examples(monkeypatch, tmp_path):
    package = tmp_path / 'packages/current-source'
    share = tmp_path / 'install/share'
    monkeypatch.setitem(globals(), 'PACKAGE_ROOT', package)
    installed = Mock(side_effect=lambda name: str(share / name))
    monkeypatch.setitem(globals(), 'get_package_share_directory', installed)
    bridge = package / 'launch/world.launch.py'
    bridge.parent.mkdir(parents=True)
    bridge.write_text("location = 'staged-bridge'\n")
    assert load('litesim3d_ros2/launch/world.launch.py').location == 'staged-bridge'
    installed.assert_not_called()
    filenames = [entry[0] for entry in EXAMPLES] + LEGACY
    for filename in filenames:
        relative = Path(filename)
        target = share.joinpath(*relative.parts[1:])
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text("location = 'installed-example'\n")
        module = load('examples/' + filename)
        assert Path(module.__file__) == target and module.location == 'installed-example'
    assert [call.args[0] for call in installed.call_args_list] == [Path(name).parts[1] for name in filenames]


def test_missing_checkout_launch_file_is_not_replaced_by_installed_copy(monkeypatch, tmp_path):
    (tmp_path / 'examples').mkdir()
    monkeypatch.setitem(globals(), 'PACKAGE_ROOT', tmp_path / 'litesim3d_ros2')
    installed = Mock(side_effect=AssertionError('missing source must remain a test failure'))
    monkeypatch.setitem(globals(), 'get_package_share_directory', installed)
    with pytest.raises(FileNotFoundError):
        load('examples/wheel_car_4w/wheel_car_4w_sim/launch/missing.launch.py')
    installed.assert_not_called()


def capture(module, monkeypatch):
    nodes, exits, includes = [], [], []
    real_node = module.Node
    real_exit = module.OnProcessExit
    def node(**kwargs):
        action = real_node(**kwargs)
        nodes.append((action, kwargs))
        return action
    def on_exit(**kwargs):
        exits.append(kwargs)
        return real_exit(**kwargs)
    monkeypatch.setattr(module, 'Node', node)
    monkeypatch.setattr(module, 'OnProcessExit', on_exit)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda package: '/share/' + package)
    if hasattr(module, 'IncludeLaunchDescription'):
        real_include = module.IncludeLaunchDescription
        def include(*args, **kwargs):
            action = real_include(*args, **kwargs)
            includes.append((action, kwargs))
            return action
        monkeypatch.setattr(module, 'IncludeLaunchDescription', include)
    return NS(nodes=nodes, exits=exits, includes=includes)


def declare(description, **overrides):
    context = LaunchContext()
    context.launch_configurations.update(overrides)
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    return context


def assert_exit_contract(module):
    assert module.required_exit(NS(process_name='unit-peer', returncode=7), NS(is_shutdown=True)) is None
    with pytest.raises(RuntimeError, match='unit-peer exited unexpectedly.*7'):
        module.required_exit(NS(process_name='unit-peer', returncode=7), NS(is_shutdown=False))


@pytest.mark.parametrize('filename,controller,world', EXAMPLES)
def test_example_graph_preserves_controller_model_and_readiness_ownership(monkeypatch, filename, controller, world):
    module = load('examples/' + filename)
    captured = capture(module, monkeypatch)
    xacro = Mock(side_effect=lambda source: NS(toxml=lambda: '<robot name="unit"/>'))
    monkeypatch.setattr(module.xacro, 'process_file', xacro)
    description = module.generate_launch_description()
    context = declare(description)
    assert context.launch_configurations['world'] == world
    assert context.launch_configurations['controller_startup_timeout'] == '60'
    assert context.launch_configurations['joint_states_topic'] == '/joint_states'
    # Deferred RSP construction is tested directly, not by executing a launch service.
    if hasattr(module, 'setup_rsp'):
        rsp_parameters = '/share/' + Path(filename).parts[1] + '/config/rsp.yaml'
        actions = module.setup_rsp(context, rsp_parameters)
        assert len(actions) == 2
    nodes = captured.nodes
    rsp = [(a, kw) for a, kw in nodes if kw['package'] == 'robot_state_publisher']
    assert len(rsp) == 1
    assert rsp[0][1]['parameters'][0].endswith('/config/rsp.yaml')
    assert [name for name, _ in rsp[0][1]['remappings']] == ['joint_states', '/tf', '/tf_static', 'robot_description']
    managers = [(a, kw) for a, kw in nodes if '--executable' in kw.get('arguments', [])]
    assert len(managers) == 1
    manager = managers[0][1]
    assert manager['package'] == 'litesim3d_ros2_control' and manager['executable'] == 'spawn_when_ready.py'
    assert manager['arguments'][:2] == ['--executable', 'ros2_control_node']
    assert 'name' not in manager
    assert any(source == '/' + controller + ('/reference_unstamped' if controller == 'ackermann_controller'
                                           else '/cmd_vel_unstamped' if controller == 'diff_drive_controller'
                                           else '/cmd_vel') and target == '/cmd_vel'
               for source, target in manager['remappings'])
    spawners = [(a, kw) for a, kw in nodes if kw['executable'] == 'spawn_when_ready.py' and '--executable' not in kw['arguments']]
    assert len(spawners) == 1 and controller in spawners[0][1]['arguments']
    assert ('joint_state_broadcaster' in spawners[0][1]['arguments']) is (controller != 'body_velocity_controller')
    assert len(captured.includes) == 1
    forwarded = dict(captured.includes[0][1]['launch_arguments'])
    assert {'assets_root', 'world', 'runtime', 'robot_urdf', 'params'} <= set(forwarded)
    assert str(forwarded['robot_urdf']).endswith('simulation.urdf.xacro') or hasattr(forwarded['robot_urdf'], 'perform')
    assert xacro.call_count == 2
    assert all(call.args[0].endswith('.urdf.xacro') for call in xacro.call_args_list)
    spawner_action = spawners[0][0]
    callbacks = [entry['on_exit'] for entry in captured.exits if entry['target_action'] is spawner_action]
    assert len(callbacks) == 1
    assert callbacks[0](NS(process_name='spawner', returncode=0), NS(is_shutdown=False)) is None
    with pytest.raises(RuntimeError, match='spawner exited unexpectedly.*1'):
        callbacks[0](NS(process_name='spawner', returncode=1), NS(is_shutdown=False))
    assert callbacks[0](NS(process_name='spawner', returncode=1), NS(is_shutdown=True)) is None
    assert_exit_contract(module)


@pytest.mark.parametrize('profile,explicit,expected', [('mid360', '', 'mid360.rviz'),
                                                      ('config', '', 'wheel_car_4w.rviz'),
                                                      ('mid360', '/custom.rviz', '/custom.rviz')])
def test_wheel_rviz_selection_keeps_explicit_config(monkeypatch, profile, explicit, expected):
    module = load('examples/wheel_car_4w/wheel_car_4w_sim/launch/wheel_car_4w.launch.py')
    context = LaunchContext()
    context.launch_configurations.update(lidar_profile=profile, rviz_config=explicit)
    actions = module.select_rviz(context, '/share/wheels')
    for action in actions:
        action.execute(context)
    assert len(actions) == (0 if explicit else 1)
    assert context.launch_configurations['rviz_config'] == (expected if explicit else '/share/wheels/rviz/' + expected)


@pytest.mark.parametrize('filename', LEGACY)
@pytest.mark.parametrize('prefix', ['', 'robot/'])
def test_legacy_rsp_uses_prepared_model_and_simulator_frame_prefix(monkeypatch, tmp_path, filename, prefix):
    module = load('examples/' + filename)
    captured = capture(module, monkeypatch)
    model = tmp_path / 'prepared.urdf'
    model.write_text('<robot name="unit"/>')
    config = tmp_path / 'runtime.yaml'
    config.write_text(yaml.safe_dump({'robot': {'urdf': model.name}}))
    params = tmp_path / 'ros.yaml'
    params.write_text(yaml.safe_dump({
        '/**': {'ros__parameters': {'use_sim_time': True, 'publish_frequency': 10.}},
        '/robot_state_publisher': {'ros__parameters': {'frame_prefix': 'incorrect/', 'publish_frequency': 25.}},
        '/litesim3d': {'ros__parameters': {'frame_prefix': prefix}}}))
    description = module.generate_launch_description()
    context = declare(description, params=str(params), prepared_config=str(config))
    actions = module.setup_rsp(context)
    assert len(actions) == 2 and len(captured.nodes) == 1
    settings = captured.nodes[0][1]['parameters'][0]
    assert settings['use_sim_time'] is True and settings['publish_frequency'] == 25.
    assert settings['frame_prefix'] == prefix
    assert settings['robot_description'].evaluate(context) == '<robot name="unit" />'
    assert [source for source, _ in captured.nodes[0][1]['remappings']] == ['joint_states', '/tf', '/tf_static', 'robot_description']
    assert_exit_contract(module)


@pytest.mark.parametrize('profile,explicit,expected', [('mid360', '', 'mid360.rviz'),
                                                      ('config', '', 'wheel_car_4w.rviz'),
                                                      ('mid360', '/custom.rviz', '/custom.rviz')])
def test_legacy_wheel_include_selects_preview_without_overriding_user(monkeypatch, profile, explicit, expected):
    module = load('examples/wheel_car_4w/wheel_car_4w_sim/launch/legacy_twist.launch.py')
    captured = capture(module, monkeypatch)
    context = declare(module.generate_launch_description(), lidar_profile=profile, rviz_config=explicit)
    actions = module.include_simulation(context, '/share/wheels', '/share/bringup', ['rviz_config', 'lidar_profile', 'gui'])
    assert len(actions) == 1
    forwarded = dict(captured.includes[-1][1]['launch_arguments'])
    value = forwarded['rviz_config']
    actual = value.perform(context) if hasattr(value, 'perform') else value
    assert actual == (expected if explicit else '/share/wheels/rviz/' + expected)
    assert forwarded['lidar_profile'].perform(context) == profile


@pytest.mark.parametrize('launch_file', ['demo.launch.py', 'world.launch.py'])
@pytest.mark.parametrize('uri', ['relative.urdf', 'file://relative.urdf', 'package://robot/relative.urdf'])
def test_bridge_launch_resolves_resources_and_checks_shutdown(monkeypatch, tmp_path, launch_file, uri):
    module = load('litesim3d_ros2/launch/' + launch_file)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda package: str(tmp_path / package))
    expected = tmp_path / ('robot/relative.urdf' if uri.startswith('package://') else 'relative.urdf')
    assert module.resolve(uri, tmp_path) == expected
    context = LaunchContext()
    context.launch_configurations['flag'] = '  TRUE  '
    assert module.boolean(context, 'flag') is True
    context.launch_configurations['flag'] = 'FALSE'
    assert module.boolean(context, 'flag') is False
    context.launch_configurations['flag'] = '1'
    with pytest.raises(ValueError, match='flag must be true or false'):
        module.boolean(context, 'flag')
    assert_exit_contract(module)


@pytest.mark.parametrize('launch_file', ['demo.launch.py', 'world.launch.py'])
@pytest.mark.parametrize('external', ['', ' first, ,second '])
def test_bridge_launch_builds_authoritative_simulator_and_prefixed_rsp(monkeypatch, tmp_path, launch_file, external):
    module = load('litesim3d_ros2/launch/' + launch_file)
    captured = capture(module, monkeypatch)
    urdf = tmp_path / 'robot.urdf'
    urdf.write_text('<robot name="unit"/>')
    runtime = tmp_path / 'runtime.yaml'
    runtime.write_text(yaml.safe_dump({'robot': {'urdf': urdf.name}}))
    config = runtime
    if launch_file == 'world.launch.py':
        config = tmp_path / 'world.yaml'
        config.write_text(yaml.safe_dump({'config': runtime.name, 'robots': [{'id': 'first'}, {'id': 'second'}]}))
    context = declare(module.generate_launch_description(), config=str(config), namespace='robot', frame_prefix='robot/',
                      manifest='/evidence/unit-manifest.json', paused='true', rviz='false',
                      external_odom_tf_robots=external)
    actions = module.setup(context)
    sim = next(kw for _, kw in captured.nodes if kw['package'] == 'litesim3d_ros2')
    assert sim['parameters'][0]['config'] == str(config)
    assert sim['parameters'][0]['use_sim_time'] is True and sim['parameters'][0]['start_paused'] is True
    assert sim['parameters'][0]['manifest_path'] == '/evidence/unit-manifest.json'
    rsps = [kw for _, kw in captured.nodes if kw['package'] == 'robot_state_publisher']
    if launch_file == 'world.launch.py':
        assert len(rsps) == 2 and len(actions) == 6
        assert [kw['namespace'] for kw in rsps] == ['first', 'second']
        assert [kw['parameters'][0]['frame_prefix'] for kw in rsps] == ['first/', 'second/']
        settings = sim['parameters'][0]
        assert settings['sensor_reliable'] is False and settings['publish_odom_tf'] is True
        if external:
            assert settings['external_odom_tf_robots'] == ['first', 'second']
        else:
            assert 'external_odom_tf_robots' not in settings
    else:
        assert len(rsps) == 1 and len(actions) == 5
        assert rsps[0]['namespace'] == 'robot' and rsps[0]['parameters'][0]['frame_prefix'] == 'robot/'
        rviz = next((node, kw) for node, kw in captured.nodes if kw['package'] == 'rviz2')
        assert rviz[0].condition.evaluate(context) is False
    assert all(kw['parameters'][0]['robot_description'] == urdf.read_text() and
               kw['parameters'][0]['publish_frequency'] == 200. for kw in rsps)
    assert len(captured.exits) == len(rsps) + 1
