"""Coverage gates must reject missing runtime evidence, not just low percentages."""
import copy
import json
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import xml.etree.ElementTree as ET

import pytest
from checks.check_coverage import check


CORE = 'litesim3d_core/src/'
ROS = 'litesim3d_ros2/'
WHEEL = 'examples/wheel_car_4w/wheel_car_4w_sim/launch/wheel_car_4w.launch.py'


def complete_reports(directory, milestone='M3'):
    core = ['geometry', 'model', 'config', 'simulator']
    bridge = [CORE + 'headless.cpp', ROS + 'src/sim_node.cpp']
    python = ['scripts/prepare_xacro.py', ROS + 'launch/demo.launch.py']
    if milestone in ('M2', 'M3'):
        core += ['sensors', 'terrain']
        python += ['scripts/benchmarks/evaluate_m2.py', 'scripts/checks/run_checked.py', ROS + 'scripts/sim_session']
    if milestone == 'M3':
        core += ['world']
        bridge += [ROS + 'src/' + n + '.cpp' for n in
                   ('sim_world_node', 'point_adapter', 'sim_ouster_adapter', 'sim_cloud_probe')]
        python += [ROS + 'launch/world.launch.py', WHEEL]
    summary = {'lines': {'count': 100, 'covered': 100}, 'branches': {'count': 20, 'covered': 20}}
    reports = {}
    for group, paths in [('core', [CORE + n + '.cpp' for n in core]), ('bridge', bridge)]:
        reports[group] = {'data': [{'files': [{'filename': '/checkout/' + p, 'summary': copy.deepcopy(summary)}
                                             for p in paths]}]}
    reports['python'] = {'files': {p: {'summary': {'num_statements': 100, 'covered_lines': 100,
                                                  'num_branches': 20, 'covered_branches': 20}} for p in python}}
    write_reports(directory, reports)
    return reports


def write_reports(directory, reports):
    for group, report in reports.items():
        (directory / (group + '-coverage.json')).write_text(json.dumps(report))


@pytest.mark.parametrize('milestone', ['M1', 'M2', 'M3'])
def test_complete_report_and_installed_launch_alias_pass(tmp_path, milestone):
    reports = complete_reports(tmp_path, milestone)
    key = ROS + 'launch/demo.launch.py'
    reports['python']['files']['/install/share/litesim3d_ros2/launch/demo.launch.py'] = reports['python']['files'].pop(key)
    write_reports(tmp_path, reports)
    assert check(tmp_path, milestone)['status'] == 'PASS'


@pytest.mark.parametrize('group,missing', [('core', CORE + 'world.cpp'),
                                         ('core', CORE + 'config.cpp'),
                                         ('bridge', ROS + 'src/sim_world_node.cpp'),
                                         ('bridge', CORE + 'headless.cpp'),
                                         ('python', ROS + 'scripts/sim_session'),
                                         ('python', WHEEL)])
def test_missing_runtime_source_fails_even_with_perfect_remainder(tmp_path, group, missing):
    reports = complete_reports(tmp_path)
    if group == 'python':
        del reports[group]['files'][missing]
    else:
        files = reports[group]['data'][0]['files']
        files[:] = [f for f in files if not f['filename'].endswith(missing)]
    write_reports(tmp_path, reports)
    result = check(tmp_path, 'M3')
    assert result['status'] == 'FAIL'
    assert 'missing coverage source: ' + missing in result['failures']
    assert ET.parse(tmp_path / 'coverage.xml').getroot().get('failures') == '1'


def test_one_core_file_cannot_stand_in_for_project(tmp_path):
    reports = complete_reports(tmp_path)
    reports['core']['data'][0]['files'] = reports['core']['data'][0]['files'][:1]
    reports['bridge']['data'][0]['files'] = []
    reports['python']['files'] = {}
    write_reports(tmp_path, reports)
    assert check(tmp_path, 'M3')['status'] == 'FAIL'


@pytest.mark.parametrize('case', ['empty', 'missing_report', 'invalid_json', 'zero_lines',
                                 'negative', 'overcount', 'duplicate', 'duplicate_alias', 'wrong_owner', 'wrong_path'])
def test_invalid_or_ambiguous_measurements_fail_closed(tmp_path, case):
    reports = complete_reports(tmp_path)
    files = reports['core']['data'][0]['files']
    if case == 'empty':
        for group in ('core', 'bridge'): reports[group]['data'][0]['files'] = []
        reports['python']['files'] = {}
    if case == 'zero_lines': files[0]['summary']['lines'] = {'count': 0, 'covered': 0}
    if case == 'negative': files[0]['summary']['lines']['covered'] = -1
    if case == 'overcount': files[0]['summary']['branches']['covered'] = 21
    if case == 'duplicate': files.append(copy.deepcopy(files[0]))
    if case == 'duplicate_alias':
        key = ROS + 'scripts/sim_session'
        reports['python']['files']['/install/lib/litesim3d_ros2/sim_session'] = reports['python']['files'][key]
    if case == 'wrong_owner': reports['bridge']['data'][0]['files'].append(files.pop())
    if case == 'wrong_path': files[0]['filename'] = '/unrelated/geometry.cpp'
    write_reports(tmp_path, reports)
    if case == 'missing_report': (tmp_path / 'bridge-coverage.json').unlink()
    if case == 'invalid_json': (tmp_path / 'python-coverage.json').write_text('{')
    assert check(tmp_path, 'M3')['status'] == 'FAIL'


def test_existing_percentage_thresholds_still_apply(tmp_path):
    reports = complete_reports(tmp_path)
    reports['core']['data'][0]['files'][0]['summary']['lines']['covered'] = 89
    write_reports(tmp_path, reports)
    result = check(tmp_path, 'M3')
    assert result['status'] == 'FAIL'
    assert 'core threshold failed: ' + CORE + 'geometry.cpp' in result['failures']
