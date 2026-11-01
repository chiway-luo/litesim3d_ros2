"""Legacy executable name only forwards to the native waiter."""
import importlib.util
from pathlib import Path
from unittest.mock import Mock


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
