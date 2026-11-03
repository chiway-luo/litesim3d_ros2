"""Compatibility adapter units; native provenance rules have C++ tests."""
import importlib.util
import json
from pathlib import Path
import runpy
import subprocess
import sys
from types import SimpleNamespace
from unittest.mock import Mock
import pytest

SOURCE = Path(__file__).resolve().parents[2] / 'prepare_xacro.py'


@pytest.fixture
def module():
    spec = importlib.util.spec_from_file_location('xacro_adapter', SOURCE)
    result = importlib.util.module_from_spec(spec); spec.loader.exec_module(result)
    return result


@pytest.mark.parametrize('base', [None, Path('/resource base')])
def test_expand_delegates_to_native_and_returns_provenance(module, monkeypatch, base):
    monkeypatch.setattr(module, '_executable', lambda: '/native/prepare_xacro')
    run = Mock(return_value=SimpleNamespace(stdout='{"source":"robot"}'))
    monkeypatch.setattr(module.subprocess, 'run', run)
    assert module.expand(Path('/source'), Path('/output'), ['name:=中文 space'], base) == {'source': 'robot'}
    command = ['/native/prepare_xacro', '/source', '/output', 'name:=中文 space']
    if base is not None:
        command += ['--resource-base', str(base)]
    run.assert_called_once_with(command, text=True, capture_output=True, check=True, timeout=90)


def test_failed_native_preparation_propagates_without_python_fallback(module, monkeypatch):
    monkeypatch.setattr(module, '_executable', lambda: '/native/prepare_xacro')
    monkeypatch.setattr(module.subprocess, 'run', Mock(side_effect=subprocess.CalledProcessError(1, 'native')))
    with pytest.raises(subprocess.CalledProcessError):
        module.expand(Path('/source'), Path('/output'))


def test_cli_executes_native_without_reparsing_arguments(monkeypatch):
    import ament_index_python.packages
    import os
    monkeypatch.setattr(ament_index_python.packages, 'get_package_prefix', lambda _: '/prefix')
    monkeypatch.setattr(sys, 'argv', [str(SOURCE), 'source', 'output', 'x:=true'])
    execute = Mock(); monkeypatch.setattr(os, 'execv', execute)
    runpy.run_path(str(SOURCE), run_name='__main__')
    executable = '/prefix/lib/litesim3d_bringup/prepare_xacro'
    execute.assert_called_once_with(executable, [executable, 'source', 'output', 'x:=true'])


def test_executable_uses_real_ament_package_index(module, monkeypatch, tmp_path):
    prefix = tmp_path / 'native prefix 中文'
    marker = prefix / 'share/ament_index/resource_index/packages/litesim3d_bringup'
    marker.parent.mkdir(parents=True)
    marker.write_text('')
    monkeypatch.setenv('AMENT_PREFIX_PATH', str(prefix))
    assert module._executable() == str(prefix / 'lib/litesim3d_bringup/prepare_xacro')


@pytest.mark.parametrize('failure', [OSError('missing native executable'),
                                      subprocess.TimeoutExpired('prepare_xacro', 90)])
def test_native_transport_errors_propagate_without_retry_or_fallback(module, monkeypatch, failure):
    monkeypatch.setattr(module, '_executable', lambda: '/native/prepare_xacro')
    run = Mock(side_effect=failure)
    monkeypatch.setattr(module.subprocess, 'run', run)
    with pytest.raises(type(failure)) as caught:
        module.expand('source', 'output')
    assert caught.value is failure
    assert run.call_count == 1


def test_corrupt_native_provenance_is_not_silently_replaced(module, monkeypatch):
    monkeypatch.setattr(module, '_executable', lambda: '/native/prepare_xacro')
    run = Mock(return_value=SimpleNamespace(stdout='{truncated'))
    monkeypatch.setattr(module.subprocess, 'run', run)
    with pytest.raises(json.JSONDecodeError):
        module.expand('source', 'output')
    assert run.call_count == 1


def test_empty_explicit_resource_base_is_not_confused_with_absent_option(module, monkeypatch):
    monkeypatch.setattr(module, '_executable', lambda: '/native/prepare_xacro')
    run = Mock(return_value=SimpleNamespace(stdout='{}'))
    monkeypatch.setattr(module.subprocess, 'run', run)
    mappings = ['literal:=$(not-a-shell-command)', 'name:=space value']
    original = list(mappings)
    assert module.expand('source', 'output', mappings, resource_base='') == {}
    assert mappings == original
    run.assert_called_once_with(
        ['/native/prepare_xacro', 'source', 'output', *original, '--resource-base', ''],
        text=True, capture_output=True, check=True, timeout=90)


def test_legacy_cli_propagates_exec_failure_without_replacing_argv(monkeypatch):
    import ament_index_python.packages
    import os
    monkeypatch.setattr(ament_index_python.packages, 'get_package_prefix', lambda _: '/prefix')
    argv = [str(SOURCE), '--', 'name:=中文 space']
    monkeypatch.setattr(sys, 'argv', argv)
    failure = OSError('exec unavailable')
    execute = Mock(side_effect=failure)
    monkeypatch.setattr(os, 'execv', execute)
    with pytest.raises(OSError) as caught:
        runpy.run_path(str(SOURCE), run_name='__main__')
    assert caught.value is failure
    assert argv == [str(SOURCE), '--', 'name:=中文 space']
    executable = '/prefix/lib/litesim3d_bringup/prepare_xacro'
    execute.assert_called_once_with(executable, [executable, '--', 'name:=中文 space'])
