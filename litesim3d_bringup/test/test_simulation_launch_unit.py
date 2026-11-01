"""Construct the real launch graph and exercise callbacks without launching it."""
import importlib.util
import json
from pathlib import Path
import subprocess
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, RegisterEventHandler


@pytest.fixture
def module():
    path = Path(__file__).resolve().parents[1] / 'launch/simulation.launch.py'
    spec = importlib.util.spec_from_file_location('simulation_launch_unit', path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


@pytest.fixture
def launch_case(module, monkeypatch, tmp_path):
    config = tmp_path / 'runtime.yaml'
    config.write_text('robot: {urdf: robot.urdf}\n')
    params = tmp_path / 'params.yaml'
    params.write_text(yaml.safe_dump({'/**': {'ros__parameters': {'use_sim_time': True}}}))
    rviz = tmp_path / 'source.rviz'
    rviz.write_text(yaml.safe_dump({'Visualization Manager': {'Global Options': {}, 'Displays': []}}))
    monkeypatch.setattr(module, 'get_package_share_directory', lambda _: str(tmp_path))
    monkeypatch.setattr(module, 'get_package_prefix', lambda package: '/prefix/' + package)
    context = LaunchContext()
    context.launch_configurations.update(params=str(params), rviz_config=str(rviz))
    description = module.generate_launch_description()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    nodes, exits = [], []
    real_node, real_exit = module.Node, module.OnProcessExit
    def capture_node(**kwargs):
        node = real_node(**kwargs)
        nodes.append((node, kwargs))
        return node
    def capture_exit(**kwargs):
        exits.append(kwargs)
        return real_exit(**kwargs)
    monkeypatch.setattr(module, 'Node', capture_node)
    monkeypatch.setattr(module, 'OnProcessExit', capture_exit)
    case = NS(context=context, config=config, params=params, rviz=rviz, nodes=nodes, exits=exits,
              root=tmp_path, description=description)
    yield case
    for _, kwargs in nodes:
        if kwargs.get('name') == 'rviz2':
            Path(kwargs['arguments'][1]).unlink(missing_ok=True)


def legacy(case, **overrides):
    case.context.launch_configurations.update(config=str(case.config), model_prepared='true', **overrides)


def test_argument_contract_and_exit_callbacks(module, launch_case, tmp_path):
    declared = {a.name for a in launch_case.description.entities if isinstance(a, DeclareLaunchArgument)}
    assert declared == {'assets_root', 'world', 'runtime', 'robot_urdf', 'model_arguments', 'params',
        'joint_states_topic', 'joint_commands_topic', 'joint_feedback_topic', 'tf_topic', 'tf_static_topic',
        'robot_description_topic', 'gui', 'cameras', 'camera_view', 'collision_budget_enabled',
        'real_time_factor', 'config', 'lidar_profile', 'noise_config', 'lidar_interface', 'xfer_format',
        'livox_lidar_id', 'lidar_profiles_dir', 'rviz', 'rviz_config', 'paused', 'manifest'}
    event = NS(process_name='simulator', returncode=9)
    for shutting_down in (True, False):
        context = NS(is_shutdown=shutting_down)
        if shutting_down:
            assert module.required_exit(event, context) is None
            assert module.viewer_exit(event, context, tmp_path / 'absent') is None
        else:
            with pytest.raises(RuntimeError, match='required process simulator.*9'):
                module.required_exit(event, context)
            with pytest.raises(RuntimeError, match='before first rendered frames.*9'):
                module.viewer_exit(event, context, tmp_path / 'absent')
    ready = tmp_path / 'ready'
    ready.touch()
    assert module.viewer_exit(event, NS(is_shutdown=False), ready) is None
    module.remove_temporary_config(None, ready)
    module.remove_temporary_config(None, ready)
    assert not ready.exists()
    directory = tmp_path / 'cleanup'
    directory.mkdir()
    (directory / 'file').write_text('remove')
    module.cleanup_directory(None, directory)
    module.cleanup_directory(None, directory)
    assert not directory.exists()


@pytest.mark.parametrize('document', [None, [], 'not a mapping', 123])
def test_legacy_requires_parameter_yaml_mapping(module, launch_case, monkeypatch, document):
    legacy(launch_case)
    launch_case.params.write_text(yaml.safe_dump(document))
    prepare = Mock()
    monkeypatch.setattr(module, 'prepare_config', prepare)
    with pytest.raises(ValueError, match='ROS parameter YAML mapping'):
        module.setup(launch_case.context)
    prepare.assert_not_called()
    assert not launch_case.nodes


@pytest.mark.parametrize('section,value', [
    ('/**', []), ('/**', {}), ('/**', {'ros__parameters': []}),
    ('/litesim3d', {'ros__parameters': {}, 'extra': 1}),
    ('/rviz2', 'invalid')])
def test_legacy_rejects_invalid_parameter_sections(module, launch_case, section, value):
    legacy(launch_case)
    document = {'/**': {'ros__parameters': {'use_sim_time': True}}, section: value}
    launch_case.params.write_text(yaml.safe_dump(document))
    with pytest.raises(ValueError, match='invalid parameter section'):
        module.setup(launch_case.context)
    assert not launch_case.nodes


@pytest.mark.parametrize('name,value', [('/litesim3d', False), ('/litesim3d', 'true'),
                                        ('/rviz2', 1), ('/rviz2', None)])
def test_simulator_and_rviz_require_boolean_sim_time(module, launch_case, name, value):
    legacy(launch_case)
    launch_case.params.write_text(yaml.safe_dump({'/**': {'ros__parameters': {'use_sim_time': True}},
                                                 name: {'ros__parameters': {'use_sim_time': value}}}))
    with pytest.raises(ValueError, match=name + ': use_sim_time must be true'):
        module.setup(launch_case.context)
    assert not launch_case.nodes


@pytest.mark.parametrize('prefix', ['/absolute/', 'missing_slash', 1, None])
def test_legacy_frame_prefix_is_relative_and_slash_terminated(module, launch_case, prefix):
    legacy(launch_case)
    launch_case.params.write_text(yaml.safe_dump({'/**': {'ros__parameters': {'use_sim_time': True, 'frame_prefix': prefix}}}))
    with pytest.raises(ValueError, match='frame_prefix must be empty or relative'):
        module.setup(launch_case.context)
    assert not launch_case.nodes


def test_legacy_overrides_and_rviz_dictionary_string_topics_are_preserved(module, launch_case, monkeypatch):
    case = launch_case
    legacy(case, paused='true', manifest=str(case.root / 'manifest.json'), rviz='true',
           scene_description='/scene/scene.json', viewer_ready_file='/scene/viewer.ready')
    case.params.write_text(yaml.safe_dump({
        '/**': {'ros__parameters': {'use_sim_time': True, 'frame_prefix': 'robot/'}},
        '/litesim3d': {'ros__parameters': {'xfer_format': 0}},
        '/rviz2': {'ros__parameters': {'custom_rviz_parameter': 7}},
        '/unused': {'caller_owned': True}}))
    original = {'Visualization Manager': {'Global Options': {'Fixed Frame': 'old'}, 'Displays': [
        {'Class': 'rviz_default_plugins/RobotModel', 'TF Prefix': 'old'},
        {'Class': 'rviz_default_plugins/PointCloud2', 'Topic': {'Value': '/raw', 'Depth': 9}},
        {'Class': 'rviz_default_plugins/PointCloud2', 'Topic': '/raw'},
        {'Class': 'rviz_default_plugins/PointCloud2', 'Topic': '/unrelated'},
        {'Class': 'rviz_default_plugins/PointCloud2', 'Topic': {'Value': '/unrelated'}},
        {'Class': 'rviz_default_plugins/PointCloud2'},
        {'Class': 'rviz_default_plugins/Grid'}]}}
    case.rviz.write_text(yaml.safe_dump(original))
    topics = Mock(return_value={'/raw': '/preview'})
    monkeypatch.setattr(module, 'lidar_preview_topics', topics)
    actions = module.setup(case.context)
    assert {kw['name'] for _, kw in case.nodes} == {'litesim3d', 'rviz2'}
    sim = case.nodes[0][1]['parameters'][0]
    assert sim['start_paused'] is True and sim['manifest_path'] == str(case.root / 'manifest.json')
    assert sim['scene_description'] == '/scene/scene.json'
    assert sim['viewer_ready_file'] == '/scene/viewer.ready'
    assert sim['lidar_interface'] == 'auto' and sim['livox_lidar_id'] == 0 and sim['xfer_format'] == 0
    rviz = case.nodes[1][1]
    assert rviz['parameters'] == [{'use_sim_time': True, 'frame_prefix': 'robot/', 'custom_rviz_parameter': 7}]
    generated = yaml.safe_load(Path(rviz['arguments'][1]).read_text())['Visualization Manager']
    assert generated['Global Options']['Fixed Frame'] == 'robot/odom'
    displays = generated['Displays']
    assert displays[0]['TF Prefix'] == 'robot'
    assert displays[1]['Topic'] == {'Value': '/preview', 'Depth': 9}
    assert displays[2]['Topic'] == '/preview'
    assert displays[3]['Topic'] == '/unrelated' and displays[4]['Topic'] == {'Value': '/unrelated'}
    assert yaml.safe_load(case.rviz.read_text()) == original
    assert case.context.launch_configurations['prepared_config'] == str(case.config)
    assert sum(isinstance(a, RegisterEventHandler) for a in actions) == 2
    topics.assert_called_once_with(case.config, 'auto', 0)


def test_legacy_prepares_model_once_and_registers_cleanup(module, launch_case, monkeypatch):
    case = launch_case
    case.context.launch_configurations.update(config=str(case.config), model_arguments='{wheel: 4}')
    directory = case.root / 'model'
    directory.mkdir()
    prepared = directory / 'prepared.yaml'
    prepare = Mock(return_value=prepared)
    monkeypatch.setattr(module.tempfile, 'mkdtemp', Mock(return_value=str(directory)))
    monkeypatch.setattr(module, 'prepare_config', prepare)
    actions = module.setup(case.context)
    assert len(case.nodes) == 1 and case.nodes[0][1]['name'] == 'litesim3d'
    assert case.nodes[0][1]['parameters'][0]['config'] == str(prepared)
    assert case.context.launch_configurations['prepared_config'] == str(prepared)
    prepare.assert_called_once_with(case.config, str(directory), robot_urdf='', mappings={'wheel': 4},
                                    lidar_profile='config', profiles_dir='', noise_config='')
    assert isinstance(actions[0], RegisterEventHandler)


@pytest.mark.parametrize('error', [ValueError('bad model'), KeyboardInterrupt()])
def test_preparation_errors_remove_legacy_temporary_directory(module, launch_case, monkeypatch, error):
    case = launch_case
    case.context.launch_configurations['config'] = str(case.config)
    directory = case.root / 'failed-model'
    directory.mkdir()
    (directory / 'partial-output').write_text('temporary')
    monkeypatch.setattr(module.tempfile, 'mkdtemp', lambda **kwargs: str(directory))
    monkeypatch.setattr(module, 'prepare_config', Mock(side_effect=error))
    with pytest.raises(type(error)):
        module.setup(case.context)
    assert not directory.exists() and not case.nodes


@pytest.mark.parametrize('overrides,match', [
    ({'paused': 'maybe'}, 'paused must be true or false'),
    ({'config': '/legacy', 'cameras': 'true'}, 'camera overrides require SDF mode'),
    ({}, 'assets_root is required'),
    ({'assets_root': '/assets'}, 'world is required'),
    ({'assets_root': '/assets', 'world': 'world.sdf'}, 'runtime is required'),
    ({'assets_root': '/assets', 'world': 'world.sdf', 'runtime': '/runtime', 'gui': '1'}, 'gui must be true or false'),
    ({'assets_root': '/assets', 'world': 'world.sdf', 'runtime': '/runtime', 'collision_budget_enabled': 'yes'},
     'collision_budget_enabled must be true or false')])
def test_launch_rejects_bad_input_before_any_preparation(module, launch_case, monkeypatch, overrides, match):
    launch_case.context.launch_configurations.update(overrides)
    prepare = Mock()
    run = Mock()
    mkdir = Mock()
    monkeypatch.setattr(module, 'prepare_config', prepare)
    monkeypatch.setattr(module.subprocess, 'run', run)
    monkeypatch.setattr(module.tempfile, 'mkdtemp', mkdir)
    with pytest.raises(ValueError, match=match):
        module.setup(launch_case.context)
    prepare.assert_not_called()
    run.assert_not_called()
    mkdir.assert_not_called()


def sdf_boundaries(module, case, monkeypatch, cameras=(), stderr='', returncode=0):
    directory = case.root / 'sdf'
    directory.mkdir()
    scene = directory / 'scene'
    scene.mkdir()
    (scene / 'scene.json').write_text(json.dumps({'cameras': list(cameras)}))
    (scene / 'runtime.yaml').write_text('robot: {urdf: robot.urdf}\n')
    monkeypatch.setattr(module.tempfile, 'mkdtemp', Mock(return_value=str(directory)))
    prepare = Mock(return_value=directory / 'model/runtime.yaml')
    monkeypatch.setattr(module, 'prepare_config', prepare)
    run = Mock(return_value=NS(returncode=returncode, stderr=stderr, stdout='importer summary\n'))
    monkeypatch.setattr(module.subprocess, 'run', run)
    case.context.launch_configurations.update(assets_root=str(case.root / 'assets'), world='office.sdf',
                                              runtime=str(case.config))
    return directory, prepare, run


@pytest.mark.parametrize('stderr,expected', [(' bad geometry\n', 'bad geometry'), ('', 'SDF import failed')])
def test_sdf_importer_failure_is_reported_and_cleans_temporary_tree(module, launch_case, monkeypatch, stderr, expected):
    directory, _, run = sdf_boundaries(module, launch_case, monkeypatch, stderr=stderr, returncode=5)
    with pytest.raises(RuntimeError, match=expected):
        module.setup(launch_case.context)
    assert not directory.exists() and not launch_case.nodes
    assert run.call_args.kwargs['timeout'] == 120


def test_sdf_importer_timeout_preserves_error_and_cleans_tree(module, launch_case, monkeypatch):
    directory, _, run = sdf_boundaries(module, launch_case, monkeypatch)
    run.side_effect = subprocess.TimeoutExpired('sdf_import', 120)
    with pytest.raises(subprocess.TimeoutExpired):
        module.setup(launch_case.context)
    assert not directory.exists() and not launch_case.nodes


@pytest.mark.parametrize('gui,camera_view,enabled', [(False, False, False), (False, False, True),
                                                  (False, True, True), (True, True, True)])
def test_sdf_graph_renderer_viewer_independence_and_resource_environment(module, launch_case, monkeypatch,
                                                                       capsys, gui, camera_view, enabled):
    case = launch_case
    camera = {'name': 'front', 'enabled': enabled, 'topic': '/front/image', 'type': 'rgb',
              'rate_hz': 20., 'near_clip_m': .1, 'far_clip_m': 50.}
    directory, prepare, run = sdf_boundaries(module, case, monkeypatch, cameras=[camera, {**camera, 'name': 'disabled', 'enabled': False}])
    case.context.environment['GZ_SIM_RESOURCE_PATH'] = ':'.join([str(case.root / 'assets/models'), '/external/models', ''])
    case.context.launch_configurations.update(gui=str(gui).lower(), camera_view=str(camera_view).lower(),
                                              cameras='true', real_time_factor='0', collision_budget_enabled='true',
                                              model_arguments='{sensor: enabled}')
    actions = module.setup(case.context)
    names = {kw['name'] for _, kw in case.nodes}
    expected = {'litesim3d'}
    if enabled:
        expected.add('litesim_camera_renderer')
        if camera_view:
            expected.add('camera_view_front')
    if gui:
        expected.add('litesim_viewer')
    assert names == expected
    assert 'LiteSim SDF: importer summary' in capsys.readouterr().out
    command = run.call_args.args[0]
    assert command == ['/prefix/litesim3d_assets/lib/litesim3d_assets/sdf_import',
        str(case.root / 'assets/worlds/office.sdf'), str(directory / 'model/runtime.yaml'),
        str(directory / 'scene'), 'true', '0']
    env = run.call_args.kwargs['env']
    assert env['GZ_SIM_RESOURCE_PATH'].split(':') == [str(case.root / 'assets/models'),
                                                      str(case.root / 'assets/worlds'), '/external/models']
    assert run.call_args.kwargs['text'] is True and run.call_args.kwargs['capture_output'] is True
    assert prepare.call_args.kwargs['mappings'] == {'sensor': 'enabled'}
    assert prepare.call_args.kwargs['cameras'] == 'true'
    sim = next(kw for _, kw in case.nodes if kw['name'] == 'litesim3d')['parameters'][0]
    assert sim['scene_description'] == str(directory / 'scene/scene.json')
    assert sim['viewer_ready_file'] == (str(directory / 'viewer.ready') if gui else '')
    if camera_view and enabled:
        view = next(kw for _, kw in case.nodes if kw['name'] == 'camera_view_front')
        assert view['parameters'] == [{key: camera[key] for key in ('topic', 'type', 'rate_hz', 'near_clip_m', 'far_clip_m')}]
    if gui:
        viewer = next(kw for _, kw in case.nodes if kw['name'] == 'litesim_viewer')
        assert viewer['parameters'][0]['ready_file'] == str(directory / 'viewer.ready')
        handler = next(e['on_exit'] for e in case.exits if e['target_action'] is next(
            node for node, kw in case.nodes if kw['name'] == 'litesim_viewer'))
        with pytest.raises(RuntimeError, match='before first rendered frames'):
            handler(NS(returncode=3), NS(is_shutdown=False))
        (directory / 'viewer.ready').touch()
        assert handler(NS(returncode=0), NS(is_shutdown=False)) is None
    assert len(actions) >= len(case.nodes)
    module.cleanup_directory(case.context, directory)


@pytest.mark.parametrize('world', ['/external/world.sdf', 'relative/world.sdf'])
def test_sdf_world_paths_and_optional_rtf_are_forwarded_without_rewriting(module, launch_case, monkeypatch, world):
    directory, _, run = sdf_boundaries(module, launch_case, monkeypatch)
    launch_case.context.launch_configurations.update(world=world, gui='false')
    module.setup(launch_case.context)
    assert run.call_args.args[0][1] == str(Path(world).resolve())
    assert len(run.call_args.args[0]) == 5
    module.cleanup_directory(None, directory)


@pytest.mark.parametrize('metal_exists', [True, False])
def test_darwin_viewer_adds_metal_path_only_when_installed(module, launch_case, monkeypatch, metal_exists):
    directory, _, _ = sdf_boundaries(module, launch_case, monkeypatch)
    monkeypatch.setattr(module.sys, 'platform', 'darwin')
    isfile = Mock(return_value=metal_exists)
    monkeypatch.setattr(module.os.path, 'isfile', isfile)
    actions = module.setup(launch_case.context)
    isfile.assert_called_once_with('/prefix/litesim3d_gui/lib/litesim3d_gui/ogre2/RenderSystem_Metal.dylib')
    from launch.actions import SetEnvironmentVariable
    assert sum(isinstance(a, SetEnvironmentVariable) for a in actions) == (2 if metal_exists else 1)
    module.cleanup_directory(None, directory)
