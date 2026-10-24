"""The generic bridge must work when optional vendor packages are absent."""
import pytest
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from sensor_msgs.msg import PointCloud2
from test_ros import h


def test_auto_without_mid360_keeps_native_output(h):
    h.subscribe("points", PointCloud2, "/points_raw", reliable=False)
    h.sim(start_paused=True, lidar_interface="auto")
    h.ready()
    h.until(lambda: h.node.count_publishers("/points_raw") == 1)
    assert h.step(20).success
    h.until(lambda: h.messages["points"])
    assert h.messages["points"][-1].point_step == 24
    assert h.node.count_publishers("/livox/lidar") == 0


def test_missing_output_plugin_fails_with_actionable_error(h):
    try:
        get_package_share_directory("litesim3d_mid360")
    except PackageNotFoundError:
        pass
    else:
        pytest.skip("run against the bridge-only installation to exercise a missing plugin")
    h.config["lidars"][0].update(
        scan_pattern="livox_mid360",
        pattern_file="package://litesim3d_core/lidar_patterns/mid360.csv",
        columns=20000, rings=1, vertical_angles_deg=[0.0], ring_delays_s=[])
    process = h.sim(lidar_interface="livox")
    h.until(lambda: process.poll() is not None)
    assert process.returncode != 0
    log = (h.directory / "sim.log").read_text()
    assert "requires the litesim3d_mid360 plugin" in log
