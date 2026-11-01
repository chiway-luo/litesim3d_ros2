"""Native model preparation through the installed executable boundary."""
import copy
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml
from litesim3d_bringup import model as module


def source(tmp_path, robot=None, **data):
    config = tmp_path / 'runtime.yaml'
    config.write_text(yaml.safe_dump({'robot': {'urdf': robot or ''}, **data}))
    return config


def urdf(tmp_path, xml):
    path = tmp_path / 'source.urdf'
    path.write_text(xml)
    return path


def test_resolve_package_resource_and_ros_description_keep_nonabsolute_urls(monkeypatch, tmp_path):
    mesh = tmp_path / 'mesh.dae'
    mesh.touch()
    marker = tmp_path / 'overlay/share/ament_index/resource_index/packages/unit'
    marker.parent.mkdir(parents=True)
    marker.touch()
    share = tmp_path / 'overlay/share/unit'
    share.mkdir()
    (share / 'mesh.dae').symlink_to(mesh)
    monkeypatch.setenv('AMENT_PREFIX_PATH', str(tmp_path / 'overlay') + os.pathsep + os.environ['AMENT_PREFIX_PATH'])
    assert module.resolve('package://unit/mesh.dae', tmp_path) == mesh
    assert module.resolve('file://mesh.dae', tmp_path) == mesh
    with pytest.raises(ValueError, match='invalid package resource'):
        module.resolve('package://unit//absolute', tmp_path)
    xml = urdf(tmp_path, '<robot><mesh filename="relative.dae"/><texture filename="package://pkg/a.png"/><mesh/></robot>')
    description = ET.fromstring(module.ros_robot_description(xml))
    assert [e.get('filename') for e in description.findall('.//mesh')] == ['relative.dae', None]
    assert description.find('.//texture').get('filename') == 'package://pkg/a.png'
    assert module.boolean(' FALSE ', 'flag') is False
    assert module.boolean(True, 'flag') is True
    with pytest.raises(ValueError, match='flag must be true or false'):
        module.boolean('1', 'flag')


@pytest.mark.parametrize('document', ['[]', 'lidar: []', 'lidar: {}\nextra: 1', 'lidar: {name: changed}',
                                      'lidar: {link: changed}', 'lidar: {topic: changed}'])
def test_profile_rejects_invalid_shape_and_binding_replacement(tmp_path, document):
    path = tmp_path / 'profile.yaml'
    path.write_text(document)
    with pytest.raises(ValueError, match='single lidar mapping|cannot replace'):
        module.profile(str(path), '')


def test_named_profile_and_resource_resolution(tmp_path):
    with pytest.raises(ValueError, match='named lidar_profile requires lidar_profiles_dir'):
        module.profile('mid360', '')
    pattern = tmp_path / 'pattern.csv'
    pattern.write_text('unit-pattern')
    path = tmp_path / 'mid360.yaml'
    path.write_text('lidar: {pattern_file: pattern.csv, scan_pattern: livox_mid360}\n')
    assert module.profile('mid360', str(tmp_path)) == {'pattern_file': str(pattern), 'scan_pattern': 'livox_mid360'}


def test_merge_is_recursive_without_mutating_inputs_and_overrides_validate():
    base = {'nested': {'keep': 1, 'replace': 2}, 'list': [1]}
    overlay = {'nested': {'replace': 3}, 'list': [2]}
    before = copy.deepcopy(base)
    assert module.merge(base, overlay) == {'nested': {'keep': 1, 'replace': 3}, 'list': [2]}
    assert base == before
    for xml, message in [('<sensor name="s"><overrides/><overrides/></sensor>', 'duplicate overrides: s'),
                         ('<sensor><overrides>[]</overrides></sensor>', 'plugin overrides must be a mapping')]:
        with pytest.raises(ValueError, match=message):
            module.override(ET.fromstring(xml), {'enabled': False})
    sensor = ET.fromstring('<sensor><overrides/></sensor>')
    module.override(sensor, {'enabled': False})
    assert yaml.safe_load(sensor.find('overrides').text) == {'enabled': False}


@pytest.mark.parametrize('mode,message', [('duplicate', 'ambiguous sensor name'),
                                         ('unsupported', 'unsupported sensor type'),
                                         ('bad_odometry', 'requires one standard wheel odometry')])
def test_noise_rejects_ambiguous_or_unsupported_bindings(tmp_path, mode, message):
    config = tmp_path / 'noise.yaml'
    config.write_text('sensors: {sensor: {range_sigma_m: 0.1}}')
    data = {}
    root = ET.fromstring('<robot><litesim3d><sensor name="sensor" class="vendor/Unknown"/></litesim3d></robot>')
    if mode == 'duplicate':
        data = {'lidars': [{'name': 'sensor'}]}
    if mode == 'bad_odometry':
        config.write_text('odometry: {}')
        root = ET.fromstring('<robot><litesim3d><odometry class="vendor/Unknown"/></litesim3d></robot>')
    with pytest.raises(ValueError, match=message):
        module.apply_noise_config(data, root, config, tmp_path)


def test_noise_updates_legacy_sensor_values_in_place_without_touching_bindings(tmp_path):
    config = tmp_path / 'noise.yaml'
    config.write_text(yaml.safe_dump({'sensors': {'lidar': {'range_sigma_m': .1}, 'imu': {'gyro_sigma': [.1]*3},
        'color': {'color_bias_dn': [1, 2, 3]}, 'depth': {'depth_sigma_m': .2}}}))
    data = {'lidars': [{'name': 'lidar', 'topic': '/points'}], 'imus': [{'name': 'imu'}],
            'cameras': [{'name': 'color'}, {'name': 'depth', 'type': 'depth'}]}
    module.apply_noise_config(data, None, config, tmp_path)
    assert data['lidars'][0] == {'name': 'lidar', 'topic': '/points', 'range_sigma_m': .1}
    assert data['imus'][0]['gyro_sigma'] == [.1]*3
    assert data['cameras'][0]['color_bias_dn'] == [1, 2, 3] and data['cameras'][1]['depth_sigma_m'] == .2


@pytest.mark.parametrize('document', ['[]', '{}', 'robot: []'])
def test_prepare_requires_robot_mapping(tmp_path, document):
    config = tmp_path / 'runtime.yaml'
    config.write_text(document)
    with pytest.raises(ValueError, match='configuration must contain a robot mapping'):
        module.prepare_config(config, tmp_path / 'out')


@pytest.mark.parametrize('mappings', [{1: 'name'}, {'key': {}}, {'key': []}])
def test_prepare_rejects_non_scalar_xacro_mappings(tmp_path, mappings):
    config = source(tmp_path)
    with pytest.raises(ValueError, match='mapping of scalar Xacro arguments'):
        module.prepare_config(config, tmp_path / 'out', mappings=mappings)


def test_model_arguments_require_xacro_and_robot_root(monkeypatch, tmp_path):
    config = source(tmp_path)
    with pytest.raises(ValueError, match='model_arguments requires robot.urdf'):
        module.prepare_config(config, tmp_path / 'out', mappings={'key': 1})
    model = urdf(tmp_path, '<robot/>')
    config = source(tmp_path, str(model))
    with pytest.raises(ValueError, match='model_arguments requires a .xacro model'):
        module.prepare_config(config, tmp_path / 'out', mappings={'key': 1})
    model.write_text('<not_robot/>')
    with pytest.raises(ValueError, match='model must contain a robot root'):
        module.prepare_config(config, tmp_path / 'out')
    xacro = tmp_path / 'source.xacro'
    xacro.write_text('<robot xmlns:xacro="http://www.ros.org/wiki/xacro"><xacro:arg name="enabled" default="true"/>'
                     '<xacro:arg name="count" default="1"/><link name="$(arg enabled)_$(arg count)"/></robot>')
    config = source(tmp_path, str(xacro))
    prepared = module.prepare_config(config, tmp_path / 'expanded', mappings={'enabled': False, 'count': 3})
    expanded = ET.parse(yaml.safe_load(prepared.read_text())['robot']['urdf'])
    assert expanded.find('link').get('name') == 'false_3'


def test_resource_pinning_covers_meshes_parameter_sections_and_inline_patterns(tmp_path):
    for name in ('mesh.dae', 'texture.png', 'pattern.csv'):
        (tmp_path / name).write_text('unit resource')
    (tmp_path / 'nested.yaml').write_text('outer: {scan: {pattern_file: pattern.csv}}')
    (tmp_path / 'plain.yaml').write_text('rate_hz: 10')
    (tmp_path / 'list.yaml').write_text('[]')
    model = urdf(tmp_path, '''<robot><link><visual><geometry><mesh filename="mesh.dae"/></geometry>
        <material><texture filename="texture.png"/></material></visual></link><mesh/>
        <litesim3d><sensor name="scan" class="litesim3d_lidar/Lidar">
          <parameters file="nested.yaml" section="outer.scan"/><parameters file="plain.yaml"/>
          <parameters file="list.yaml"/><overrides>pattern_file: pattern.csv</overrides>
        </sensor><sensor name="other" class="vendor/Unknown"/></litesim3d></robot>''')
    prepared = module.prepare_config(source(tmp_path, str(model)), tmp_path / 'out')
    root = ET.parse(yaml.safe_load(prepared.read_text())['robot']['urdf']).getroot()
    assert root.find('.//mesh').get('filename') == str(tmp_path / 'mesh.dae')
    assert root.find('.//texture').get('filename') == str(tmp_path / 'texture.png')
    sensor = root.find('./litesim3d/sensor')
    params = sensor.findall('parameters')
    pinned = yaml.safe_load(Path(params[0].get('file')).read_text())
    assert pinned['outer']['scan']['pattern_file'] == str(tmp_path / 'pattern.csv')
    assert params[1].get('file') == str(tmp_path / 'plain.yaml')
    assert params[2].get('file') == str(tmp_path / 'list.yaml')
    assert yaml.safe_load(sensor.find('overrides').text)['pattern_file'] == str(tmp_path / 'pattern.csv')


@pytest.mark.parametrize('enabled', ['true', 'false'])
def test_camera_toggle_applies_to_all_plugin_kinds_and_legacy_definitions(tmp_path, enabled):
    classes = ['litesim3d_rgb_camera/RgbCamera', 'litesim3d_depth_camera/DepthCamera', 'litesim3d_rgbd_camera/RgbdCamera']
    model = urdf(tmp_path, '<robot><litesim3d>' + ''.join(
        f'<sensor name="camera{i}" class="{kind}"/>' for i, kind in enumerate(classes)) + '</litesim3d></robot>')
    config = source(tmp_path, str(model), cameras=[{'name': 'legacy', 'enabled': None}])
    prepared = module.prepare_config(config, tmp_path / 'out', cameras=enabled)
    data = yaml.safe_load(prepared.read_text())
    sensors = ET.parse(data['robot']['urdf']).findall('./litesim3d/sensor')
    assert all(yaml.safe_load(s.find('overrides').text) == {'enabled': enabled == 'true'} for s in sensors)
    assert data['cameras'][0]['enabled'] is (enabled == 'true')


@pytest.mark.parametrize('profile_values,message', [({'point_layout': 'unsupported'}, 'only xyzirt_v1'),
                                                   ({'rings': 2, 'vertical_angles_deg': [0]}, 'rings must match')])
def test_plugin_lidar_profile_rejects_layout_and_ring_mismatches(tmp_path, profile_values, message):
    model = urdf(tmp_path, '<robot><litesim3d><sensor name="scan" class="litesim3d_lidar/Lidar"/></litesim3d></robot>')
    profile = tmp_path / 'profile.yaml'
    profile.write_text(yaml.safe_dump({'lidar': profile_values}))
    with pytest.raises(ValueError, match=message):
        module.prepare_config(source(tmp_path, str(model)), tmp_path / 'out', lidar_profile=str(profile))


def test_legacy_lidar_profile_preserves_binding_and_pins_patterns_and_entity_roles(tmp_path):
    pattern = tmp_path / 'pattern.csv'
    pattern.write_text('unit')
    mesh = tmp_path / 'mesh.dae'
    mesh.write_text('unit')
    profile = tmp_path / 'profile.yaml'
    profile.write_text('lidar: {pattern_file: pattern.csv, columns: 32}')
    entity = {role: {'type': 'mesh', 'uri': 'mesh.dae'} for role in ('geometry', 'visual', 'collision', 'sensor_geometry')}
    config = source(tmp_path, lidars=[{'name': 'scan', 'link': 'lidar', 'topic': 'points', 'columns': 1}],
                    entities=[entity, {'geometry': 'not a mapping'}, {'visual': {'type': 'box'}}])
    prepared = module.prepare_config(config, tmp_path / 'out', lidar_profile=str(profile))
    result = yaml.safe_load(prepared.read_text())
    assert result['lidars'] == [{'name': 'scan', 'link': 'lidar', 'topic': 'points', 'columns': 32, 'pattern_file': str(pattern)}]
    assert all(geometry['uri'] == str(mesh) for geometry in result['entities'][0].values())


def test_lidar_preview_skips_nonlidar_plugins_and_legacy_without_pattern(tmp_path):
    model = urdf(tmp_path, '<robot><litesim3d><sensor name="camera" class="litesim3d_rgb_camera/RgbCamera"/></litesim3d></robot>')
    config = source(tmp_path, str(model), lidars=[{'name': 'scan', 'topic': 'points'}])
    prepared = module.prepare_config(config, tmp_path / 'out')
    assert module.lidar_preview_topics(prepared, 'auto', 1) == {}


@pytest.mark.parametrize('mode', ['missing', 'multiple', 'mixed'])
def test_lidar_profile_rejects_missing_multiple_or_mixed_bindings(tmp_path, mode):
    profile = tmp_path / 'profile.yaml'
    profile.write_text('lidar: {columns: 16}')
    count = 2 if mode == 'multiple' else 1
    model = urdf(tmp_path, '<robot><litesim3d>' + ''.join(
        f'<sensor name="scan{i}" class="litesim3d_lidar/Lidar"/>' for i in range(count)) + '</litesim3d></robot>')
    config = source(tmp_path, str(model) if mode != 'missing' else '',
                    **({'lidars': [{'name': 'legacy'}]} if mode == 'mixed' else {}))
    with pytest.raises(ValueError, match='exactly one lidar and no mixed'):
        module.prepare_config(config, tmp_path / 'out', lidar_profile=str(profile))
