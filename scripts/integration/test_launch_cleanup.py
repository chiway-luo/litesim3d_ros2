"""Failed launch cleanup must isolate the next real DDS simulation."""
import json
import os
import signal
import xml.etree.ElementTree as ET

import pytest
import xacro
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

from litesim3d_test_support.harness import SimulationHarness
from litesim3d_test_support.composed_models import verify_motion_and_model


def test_failed_import_reaps_rsp_before_same_domain_ackermann(tmp_path, monkeypatch):
    # Recreate the original failure: RSP starts, then the SDF importer rejects
    # a sensor parameter and launch exits before recording the process-start event.
    model = Path(get_package_share_directory("drone_3d_sim")) / "urdf/simulation.urdf.xacro"
    root = ET.fromstring(xacro.process_file(str(model)).toxml())
    lidar = root.find("./litesim3d/sensor[@name='roof_lidar']")
    ET.SubElement(lidar, "overrides").text = "point_layout: xyzirt_v1"
    invalid = tmp_path / "invalid.urdf"
    invalid.write_text(ET.tostring(root, encoding="unicode"))
    # Force reuse within this serial test, rather than hiding the leak by
    # assigning the following simulation a fresh domain.
    serial = SimulationHarness.serial
    rig = SimulationHarness(tmp_path / "failed-import", package="drone_3d_sim", launch_file="drone_3d.launch.py")
    try:
        try:
            with pytest.raises(AssertionError, match="unknown parameter point_layout"):
                rig.start(robot_urdf=str(invalid), joint_states=False)
            assert rig.process.poll() == 1
            assert rig.node.count_publishers("/tf_static") == 1
        finally:
            rig.close()
        with pytest.raises(ProcessLookupError):
            os.killpg(rig.process.pid, 0)
        cleanup = json.loads((rig.directory / "exit.json").read_text())
        assert cleanup["leader_exited_before_cleanup"]
        assert not cleanup["forced_kill"] and not cleanup["group_exists_after_cleanup"]
        monkeypatch.setattr(SimulationHarness, "serial", serial)
        verify_motion_and_model(tmp_path / "following-ackermann", "ackermann")
        graph = json.loads((tmp_path / "following-ackermann/tf-graph.json").read_text())
        assert graph["domain_id"] == int(rig.environment["ROS_DOMAIN_ID"])
        assert graph["topics"]["/tf_static"]["publisher_count"] == 1
    finally:
        # The regression must remain self-cleaning even against the pre-fix
        # harness; only this test's start_new_session process group is signaled.
        if rig.process:
            try:
                os.killpg(rig.process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
