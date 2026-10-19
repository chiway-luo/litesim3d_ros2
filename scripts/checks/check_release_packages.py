#!/usr/bin/env python3
"""Build one ROS package at a time, then test against the complete installed set.

Run in a fresh Humble environment after rosdep install. The work directory must
be outside the checkout. No repository parent, sibling source package, or root
VERSION file is copied alongside the package being built.
Tests run in a second pass because core integration tests load downstream sensor
plugins. Each test restores only its own source tree at the original build path.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]


def packages():
    found = {}
    for manifest in sorted([*ROOT.glob('*/package.xml'), *(ROOT / 'examples').rglob('package.xml'),
                            *(ROOT / 'plugins').rglob('package.xml')]):
        if (manifest.parent / 'COLCON_IGNORE').exists():
            continue  # External overlay examples have their own isolated-build acceptance script.
        xml = ET.parse(manifest).getroot()
        name = xml.findtext('name')
        if name in found:
            raise ValueError(f'Duplicate package: {name}')
        dependencies = {e.text for e in xml if e.tag == 'depend' or e.tag.endswith('_depend')}
        found[name] = (manifest.parent, dependencies)
    groups = {}
    for name, (source, _) in found.items():
        parts = source.relative_to(ROOT).parts
        groups[name] = parts[1] if parts[0] == 'examples' else None
    for name, (_, dependencies) in found.items():
        for dependency in dependencies & found.keys():
            if groups[dependency] is not None and groups[name] != groups[dependency]:
                raise ValueError(f'{name} must not depend on example package {dependency}; use a common module')
    ordered = []
    pending = set(found)
    while pending:
        ready = sorted(name for name in pending if not (found[name][1] & pending))
        if not ready:
            raise ValueError(f'Local dependency cycle: {sorted(pending)}')
        ordered.extend(ready)
        pending.difference_update(ready)
    return found, ordered


def fingerprint(source):
    digest = hashlib.sha256()
    for path in sorted(source.rglob('*')):
        if (path.is_file() and path.name != '.DS_Store' and not path.name.startswith('._')
                and not {'.pytest_cache', '__pycache__'} & set(path.parts)):
            digest.update(path.relative_to(source).as_posix().encode())
            digest.update(b'\0')
            digest.update(path.read_bytes())
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--work-dir', type=Path, required=True)
    parser.add_argument('--jobs', type=int, default=2)
    parser.add_argument('--resume', action='store_true', help='Reuse unchanged successful packages in this work directory')
    parser.add_argument('--continue-on-test-failure', action='store_true',
                        help='Finish building/testing other packages after a test failure; retain failure status and exit nonzero')
    args = parser.parse_args()
    work = args.work_dir.resolve()
    if work.is_relative_to(ROOT) or ROOT.is_relative_to(work):
        parser.error('--work-dir must be outside the repository and cannot contain it')
    if args.jobs < 1:
        parser.error('--jobs must be positive')
    if os.environ.get('ROS_DISTRO') != 'humble':
        parser.error('Source the ROS 2 Humble environment first')
    record_path = work / 'validation.json'
    if args.resume:
        record = json.loads(record_path.read_text())
        if record['source'] != str(ROOT):
            parser.error('Resume source does not match this checkout')
    else:
        if work.exists() and any(work.iterdir()):
            parser.error('Use a new empty work directory, or --resume for an existing validation')
        work.mkdir(parents=True, exist_ok=True)
        record = {'source': str(ROOT), 'ros_distro': 'humble', 'packages': {}, 'status': 'RUNNING'}
    found, order = packages()
    record['order'] = order
    record['status'] = 'RUNNING'
    log_dir = work / 'evidence'
    log_dir.mkdir(exist_ok=True)
    prefix = work / 'install'
    current = work / 'current-source'
    env = {**os.environ, 'CMAKE_BUILD_PARALLEL_LEVEL': str(args.jobs), 'MAKEFLAGS': f'-j{args.jobs}',
           'ROS_LOCALHOST_ONLY': '1', 'ROS_LOG_DIR': str(log_dir / 'ros-logs'),
           'PYTHONDONTWRITEBYTECODE': '1', 'PYTEST_DISABLE_PLUGIN_AUTOLOAD': '1'}

    def save():
        record_path.write_text(json.dumps(record, indent=2) + '\n')

    def run(name, stage, command, timeout):
        log = log_dir / f'{name}-{stage}.log'
        print(f'{name}: {stage} (log: {log})', flush=True)
        with log.open('w') as stream:
            try:
                process = subprocess.Popen(
                    ['bash', '-c', 'set -eo pipefail; if [[ -f "$1/setup.bash" ]]; then source "$1/setup.bash"; fi; shift; exec "$@"',
                     'release-check', str(prefix), *command], cwd=work, env=env,
                    stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
                return process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
                stream.write(f'\nTIMEOUT after {timeout} seconds\n')
                return 124

    test_failures = []
    cmake_args = ['-DCMAKE_BUILD_TYPE=RelWithDebInfo', '-DBUILD_TESTING=ON']
    if sys.platform == 'darwin' and env.get('CONDA_PREFIX'):
        conda = env['CONDA_PREFIX']
        cmake_args += ['-DCMAKE_C_COMPILER=/usr/bin/clang', '-DCMAKE_CXX_COMPILER=/usr/bin/clang++',
                       '-DCMAKE_BUILD_WITH_INSTALL_RPATH=ON', f'-DCMAKE_INSTALL_RPATH={prefix}/lib;{conda}/lib',
                       f'-DCMAKE_EXE_LINKER_FLAGS=-Wl,-headerpad_max_install_names -Wl,-dead_strip_dylibs -L{conda}/lib',
                       f'-DCMAKE_SHARED_LINKER_FLAGS=-Wl,-headerpad_max_install_names -Wl,-dead_strip_dylibs -L{conda}/lib']

    def restore_source(source):
        if current.exists():
            shutil.rmtree(current)
        shutil.copytree(source, current, ignore=shutil.ignore_patterns('__pycache__', '.pytest_cache', '.DS_Store', '._*'))

    def package_args(name):
        return ['--base-paths', str(current), '--build-base', str(work / 'build'),
                '--install-base', str(prefix), '--merge-install', '--packages-select', name,
                '--executor', 'sequential', '--event-handlers', 'console_direct+']

    save()
    for name in order:
        source, dependencies = found[name]
        own_hash = fingerprint(source)
        parent_hashes = {dep: record['packages'][dep]['signature'] for dep in sorted(dependencies & found.keys())}
        signature = hashlib.sha256(json.dumps([own_hash, parent_hashes], sort_keys=True).encode()).hexdigest()
        old = record['packages'].get(name, {})
        if old.get('build_exit') == 0 and old.get('signature') == signature:
            print(f'{name}: unchanged installed package; test reuse checked after all plugins are installed', flush=True)
            continue
        restore_source(source)
        entry = {'source_sha256': own_hash, 'signature': signature, 'dependencies': parent_hashes,
                 'status': 'RUNNING', 'started_at': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())}
        record['packages'][name] = entry
        save()
        entry['build_exit'] = run(name, 'build', ['colcon', '--log-base', str(work / 'log'), 'build', *package_args(name),
            '--cmake-force-configure', '--cmake-args', *cmake_args], 1800)
        entry['status'] = 'BUILT' if entry['build_exit'] == 0 else 'FAIL'
        save()
        if entry['build_exit'] != 0:
            record['status'] = 'FAIL'; save()
            print(f'{name}: BUILD FAILED; see {log_dir}', flush=True)
            return 1

    # Recheck all tests if any installed runtime changes, including a plugin
    # which depends on the core rather than being a core build dependency.
    runtime_signature = hashlib.sha256(json.dumps({name: record['packages'][name]['signature']
        for name in order}, sort_keys=True).encode()).hexdigest()
    record['test_runtime_signature'] = runtime_signature
    for name in order:
        entry = record['packages'][name]
        if entry.get('status') == 'PASS' and entry.get('test_runtime_signature') == runtime_signature:
            print(f'{name}: unchanged verified runtime', flush=True)
            continue
        restore_source(found[name][0])
        entry['test_exit'] = run(name, 'test', ['colcon', '--log-base', str(work / 'log'), 'test', *package_args(name),
                                               '--return-code-on-test-failure'], 1500)
        entry['results_exit'] = run(name, 'results', ['colcon', 'test-result', '--test-result-base',
                                                    str(work / 'build' / name), '--verbose'], 60)
        entry['test_runtime_signature'] = runtime_signature
        entry['status'] = 'PASS' if all(entry.get(key) == 0 for key in ('build_exit', 'test_exit', 'results_exit')) else 'FAIL'
        save()
        if entry['status'] != 'PASS':
            if args.continue_on_test_failure and entry['build_exit'] == 0:
                test_failures.append(name)
                print(f'{name}: TEST FAILURE retained; continuing independent package checks', flush=True)
                continue
            record['status'] = 'FAIL'
            save()
            print(f'{name}: FAILED; see {log_dir}', flush=True)
            return 1
        print(f'{name}: PASS', flush=True)
    if current.exists():
        shutil.rmtree(current)
    # A plain CMake client must use only the installed SDK. CMake does not
    # automatically search AMENT_PREFIX_PATH, so include the ROS underlays.
    consumer_build = work / 'consumer-build'
    search_paths = [str(prefix)]
    for variable in ('CMAKE_PREFIX_PATH', 'AMENT_PREFIX_PATH'):
        search_paths.extend(filter(None, env.get(variable, '').split(os.pathsep)))
    consumer = record['installed_consumer'] = {}
    consumer_args = [f"-DCMAKE_BUILD_RPATH={env['CONDA_PREFIX']}/lib"] if sys.platform == 'darwin' and env.get('CONDA_PREFIX') else []
    consumer['configure_exit'] = run('installed_consumer', 'configure', [
        'cmake', '-S', str(ROOT / 'scripts/fixtures/core_consumer'), '-B', str(consumer_build),
        '-DCMAKE_PREFIX_PATH=' + ';'.join(dict.fromkeys(search_paths)), *consumer_args], 120)
    if consumer['configure_exit'] == 0:
        consumer['build_exit'] = run('installed_consumer', 'build', [
            'cmake', '--build', str(consumer_build), '--parallel', str(args.jobs)], 180)
    if consumer.get('build_exit') == 0:
        version = ET.parse(found['litesim3d_core'][0] / 'package.xml').findtext('version')
        consumer['run_exit'] = run('installed_consumer', 'run', [str(consumer_build / 'consumer'),
            str(prefix / 'share/litesim3d_core/config/demo.yaml'), version], 60)
    consumer['status'] = 'PASS' if all(consumer.get(key) == 0 for key in (
        'configure_exit', 'build_exit', 'run_exit')) else 'FAIL'
    if consumer['status'] != 'PASS':
        record['status'] = 'FAIL'
        save()
        print(f'Installed CMake consumer: FAILED; see {log_dir}', flush=True)
        return 1
    record['status'] = 'FAIL' if test_failures else 'PASS'
    record['test_failures'] = test_failures
    save()
    if test_failures:
        print(f'All {len(order)} packages built; tests failed for {test_failures}: {record_path}', flush=True)
        return 1
    print(f'All {len(order)} packages passed independent build/install/tests: {record_path}', flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
