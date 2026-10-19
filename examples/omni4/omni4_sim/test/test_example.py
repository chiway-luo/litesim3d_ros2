import pytest
from litesim3d_test_support.composed_models import verify_motion_and_model, verify_collision


def test_composed_motion_tf_and_model(tmp_path):
    verify_motion_and_model(tmp_path, "omni4")


@pytest.mark.parametrize("enabled", [False, True])
def test_collision_and_encoder_odometry(tmp_path, enabled):
    verify_collision(tmp_path, "omni4", enabled)
