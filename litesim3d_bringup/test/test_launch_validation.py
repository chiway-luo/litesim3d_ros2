"""Invalid launch input must fail before loading or importing resources."""
import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext


def test_invalid_paused_is_rejected_before_resource_loading(monkeypatch):
    path = Path(__file__).resolve().parents[1] / "launch/simulation.launch.py"
    spec = importlib.util.spec_from_file_location("simulation_launch_validation", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    # Deliberately omit resources: argument validation must precede access to
    # required paths, temporary directories, and any importer subprocess.
    context.launch_configurations["paused"] = "yes"
    def unexpected(*args, **kwargs):
        pytest.fail("invalid paused input reached resource preparation")
    monkeypatch.setattr(module, "prepare_config", unexpected)
    monkeypatch.setattr(module.subprocess, "run", unexpected)
    monkeypatch.setattr(module.tempfile, "mkdtemp", unexpected)
    with pytest.raises(ValueError, match="paused must be true or false"):
        module.setup(context)
