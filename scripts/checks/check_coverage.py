#!/usr/bin/env python3
"""Validate the runtime source inventory before applying the M1/M2/M3 thresholds."""
import argparse
import json
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import xml.etree.ElementTree as ET


def expected_sources(milestone):
    """Explicit runtime inventory; each report owns its sources exactly once."""
    core = ['geometry', 'model', 'config', 'simulator']
    bridge = ['sim_node']
    python = ['scripts/prepare_xacro.py', 'litesim3d_ros2/launch/demo.launch.py']
    if milestone in ('M2', 'M3'):
        core += ['sensors', 'terrain']
        python += ['scripts/benchmarks/evaluate_m2.py', 'scripts/checks/run_checked.py',
                   'litesim3d_ros2/scripts/sim_session']
    if milestone == 'M3':
        core += ['world']
        bridge += ['sim_world_node', 'point_adapter', 'sim_ouster_adapter', 'sim_cloud_probe']
        python += ['litesim3d_ros2/launch/world.launch.py',
                   'examples/wheel_car_4w/wheel_car_4w_sim/launch/wheel_car_4w.launch.py']
    if milestone not in ('M1', 'M2', 'M3'):
        raise ValueError('unknown milestone: ' + milestone)
    return {
        'core': ['litesim3d_core/src/' + name + '.cpp' for name in core],
        'bridge': ['litesim3d_core/src/headless.cpp'] +
                  ['litesim3d_ros2/src/' + name + '.cpp' for name in bridge],
        'python': python,
    }


def source_identity(path, expected):
    path = Path(path).as_posix()
    for source in expected:
        aliases = [source]
        # Coverage can originate in the source tree or an installed launch/script.
        if '/launch/' in source:
            package = source.split('/launch/')[0].split('/')[-1]
            aliases.append('share/' + package + '/launch/' + source.split('/launch/')[1])
        if source == 'litesim3d_ros2/scripts/sim_session':
            aliases += ['lib/litesim3d_ros2/sim_session', 'build/litesim3d_ros2/sim_session']
        if any(path == alias or path.endswith('/' + alias) for alias in aliases):
            return source
    return None


def check(directory, milestone='M1'):
    expected = expected_sources(milestone)
    records, failures = {}, []

    def add(path, summary, group):
        source = source_identity(path, expected[group])
        if source is None:
            # In older LLVM exports the bridge report also includes core files;
            # only the dedicated core test profile owns those measurements.
            return
        if source in records:
            failures.append('duplicate coverage source: ' + source)
            return
        value = {}
        for kind in ('lines', 'branches'):
            count, covered = summary[kind]['count'], summary[kind]['covered']
            if (type(count) is not int or type(covered) is not int or
                    count < 0 or covered < 0 or covered > count or
                    (kind == 'lines' and count == 0)):
                failures.append('invalid coverage counts: ' + source + ' / ' + kind)
                return
            value[kind] = {'count': count, 'covered': covered,
                           'percent': 100 * covered / count if count else 100.}
        records[source] = value

    for group in ('core', 'bridge', 'python'):
        name = group + '-coverage.json'
        try:
            report = json.loads((directory / name).read_text())
            if group == 'python':
                for path, item in report['files'].items():
                    s = item['summary']
                    add(path, {'lines': {'count': s['num_statements'], 'covered': s['covered_lines']},
                               'branches': {'count': s['num_branches'], 'covered': s['covered_branches']}}, group)
            else:
                for data in report['data']:
                    for item in data['files']:
                        add(item['filename'], item['summary'], group)
        except (OSError, ValueError, KeyError, TypeError, AttributeError) as error:
            failures.append('invalid report ' + name + ': ' + str(error))
    for sources in expected.values():
        for source in sources:
            if source not in records:
                failures.append('missing coverage source: ' + source)

    totals = {}
    for kind in ('lines', 'branches'):
        count = sum(v[kind]['count'] for v in records.values())
        covered = sum(v[kind]['covered'] for v in records.values())
        totals[kind] = {'count': count, 'covered': covered,
                        'percent': 100 * covered / count if count else 0.}
        if not count:
            failures.append('no measured ' + kind)
    for path in expected['core']:
        if path in records and not path.endswith('/config.cpp'):
            value = records[path]
            if value['lines']['percent'] < 90 or value['branches']['percent'] < 80:
                failures.append('core threshold failed: ' + path)
    if totals['lines']['percent'] < 80 or totals['branches']['percent'] < 70:
        failures.append('whole-project threshold failed')
    result = {'status': 'FAIL' if failures else 'PASS', 'files': records, 'totals': totals,
              'failures': failures, 'expected_sources': expected,
              'exclusions': ['Third-party libraries and generated ROS message bindings',
                             'Test code, fixture generators, build/report automation, pure resources',
                             'sim_load_benchmark is a measurement tool, not a simulation runtime',
                             'sim.hpp declarations and accessors; functional logic is measured in .cpp files'],
              'llvm_note': 'No-profile accessors in sim.hpp are excluded, not counted as covered.'}
    (directory / 'coverage-gate.json').write_text(json.dumps(result, indent=2) + '\n')
    suite = ET.Element('testsuite', name=milestone + 'Coverage', tests='1', failures=str(int(bool(failures))))
    case = ET.SubElement(suite, 'testcase', classname='T-DEL-05', name='coverage_thresholds')
    if failures:
        ET.SubElement(case, 'failure', message='; '.join(failures))
    ET.ElementTree(suite).write(directory / 'coverage.xml', encoding='utf-8', xml_declaration=True)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('directory', type=Path)
    parser.add_argument('--milestone', choices=['M1', 'M2', 'M3'], default='M1')
    args = parser.parse_args()
    result = check(args.directory, args.milestone)
    print(json.dumps({'status': result['status'], 'totals': result['totals'], 'failures': result['failures']}, indent=2))
    raise SystemExit(0 if result['status'] == 'PASS' else 1)
