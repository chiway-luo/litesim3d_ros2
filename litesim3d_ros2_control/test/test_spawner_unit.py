"""Legacy executable name only forwards to the native waiter."""
import importlib.util
from pathlib import Path
import runpy
from unittest.mock import Mock
import pytest


def test_legacy_name_exec_preserves_arguments_and_process_identity(monkeypatch):
    path = Path(__file__).resolve().parents[1] / 'scripts/spawn_when_ready.py'
    spec = importlib.util.spec_from_file_location('waiter_adapter', path)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    forwarded = ['--timeout', '5', '--', '--ros-args', '-r', 'a:=b']
    monkeypatch.setattr(module.sys, 'argv', [str(path), *forwarded])
    execute = Mock(); monkeypatch.setattr(module.os, 'execv', execute)
    module.main()
    executable = str(path.with_name('spawn_when_ready'))
    execute.assert_called_once_with(executable, [executable, *forwarded])


def test_legacy_entrypoint_resolves_its_own_sibling_not_working_directory(monkeypatch, tmp_path):
    import os
    import sys
    path = Path(__file__).resolve().parents[1] / 'scripts/spawn_when_ready.py'
    monkeypatch.chdir(tmp_path)
    arguments = ['unrelated-wrapper-name', '--', 'literal:=中文 space', '$(not-a-shell)']
    monkeypatch.setattr(sys, 'argv', arguments)
    execute = Mock()
    monkeypatch.setattr(os, 'execv', execute)
    runpy.run_path(str(path), run_name='__main__')
    executable = str(path.with_name('spawn_when_ready'))
    execute.assert_called_once_with(executable, [executable, *arguments[1:]])
    assert arguments[0] == 'unrelated-wrapper-name'


def test_legacy_entrypoint_propagates_exec_failure_without_retry(monkeypatch):
    import os
    import sys
    path = Path(__file__).resolve().parents[1] / 'scripts/spawn_when_ready.py'
    monkeypatch.setattr(sys, 'argv', [str(path)])
    failure = OSError('native waiter unavailable')
    execute = Mock(side_effect=failure)
    monkeypatch.setattr(os, 'execv', execute)
    with pytest.raises(OSError) as caught:
        runpy.run_path(str(path), run_name='__main__')
    assert caught.value is failure
    executable = str(path.with_name('spawn_when_ready'))
    execute.assert_called_once_with(executable, [executable])
