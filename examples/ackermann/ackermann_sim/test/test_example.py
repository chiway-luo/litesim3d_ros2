import pytest
from litesim3d_test_support.composed_models import verify_motion_and_model, verify_collision, verify_control_handoff, verify_controller_geometry


def test_composed_motion_tf_and_model(tmp_path):
    verify_motion_and_model(tmp_path, "ackermann")


def test_duplicate_static_tf_writer_is_rejected_with_endpoint_evidence(tmp_path, monkeypatch):
    """A real extra DDS writer must fail the original ownership assertion."""
    import json
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from tf2_msgs.msg import TFMessage
    from litesim3d_test_support import composed_models

    setup = composed_models.setup

    def extra_writer(*args, **kwargs):
        rig = setup(*args, **kwargs)
        # Use the test's own node so normal harness teardown owns this publisher.
        rig.unexpected_tf_writer = rig.node.create_publisher(
            TFMessage, "/tf_static", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        return rig

    monkeypatch.setattr(composed_models, "setup", extra_writer)
    with pytest.raises(AssertionError, match='"publisher_count": 2'):
        verify_motion_and_model(tmp_path, "ackermann")
    graph = json.loads((tmp_path / "tf-graph.json").read_text())
    writers = graph["topics"]["/tf_static"]["publishers"]
    assert {writer["node_name"] for writer in writers} == {"robot_state_publisher", "litesim_example_test"}
    assert len({writer["gid"] for writer in writers}) == 2


@pytest.mark.parametrize("enabled", [False, True])
def test_collision_and_encoder_odometry(tmp_path, enabled):
    verify_collision(tmp_path, "ackermann", enabled)


def test_ros2_control_handoff_and_timeout(tmp_path):
    verify_control_handoff(tmp_path, "ackermann")


def test_controller_geometry_and_interfaces():
    verify_controller_geometry("ackermann")
