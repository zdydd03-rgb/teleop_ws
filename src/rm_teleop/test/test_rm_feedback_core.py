from types import SimpleNamespace

import pytest

from rm_teleop.rm_feedback_core import (
    FeedbackStore,
    compare_poses,
    pose_from_fk_result,
    snapshot_from_api2_state,
)
from rm_teleop.target_math import Pose3D


def make_state(*, err_code=0, arm_error_codes=()):
    error_values = list(arm_error_codes) + [0] * (24 - len(arm_error_codes))

    return SimpleNamespace(
        errCode=err_code,
        arm_ip=b"192.168.125.19\0",
        arm_port=8080,
        joint_status=SimpleNamespace(
            joint_position=[0.0, 10.0, 20.0, 30.0, 40.0, 50.0, 0.0],
            joint_speed=[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 0.0],
            joint_en_flag=[True] * 7,
            joint_err_code=[0] * 7,
        ),
        err=SimpleNamespace(
            err_len=len(arm_error_codes),
            err=error_values,
        ),
        waypoint=SimpleNamespace(
            position=SimpleNamespace(x=0.4, y=0.1, z=0.5),
            quaternion=SimpleNamespace(w=1.0, x=0.0, y=0.0, z=0.0),
        ),
        arm_current_status=0,
    )


def test_snapshot_parses_rm65_feedback():
    snapshot = snapshot_from_api2_state(make_state(), 10.0)

    assert snapshot.source_ip == "192.168.125.19"
    assert snapshot.source_port == 8080
    assert snapshot.joint_position_deg == (0.0, 10.0, 20.0, 30.0, 40.0, 50.0)
    assert snapshot.joint_speed_deg_s == (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
    assert snapshot.joint_enabled == (True,) * 6
    assert snapshot.waypoint.position == pytest.approx((0.4, 0.1, 0.5))
    assert snapshot.waypoint.orientation == pytest.approx((0.0, 0.0, 0.0, 1.0))


def test_snapshot_rejects_api_parse_error():
    with pytest.raises(ValueError, match="parse error"):
        snapshot_from_api2_state(make_state(err_code=-3), 10.0)



def test_snapshot_ignores_zero_arm_error_placeholder():
    snapshot = snapshot_from_api2_state(
        make_state(arm_error_codes=(0,)),
        10.0,
    )

    assert snapshot.arm_error_codes == ()

def test_snapshot_keeps_arm_errors():
    snapshot = snapshot_from_api2_state(
        make_state(arm_error_codes=(11, 22)),
        10.0,
    )

    assert snapshot.arm_error_codes == (11, 22)


def test_snapshot_rejects_non_finite_joint():
    state = make_state()
    state.joint_status.joint_position[2] = float("nan")

    with pytest.raises(ValueError, match="non-finite"):
        snapshot_from_api2_state(state, 10.0)


def test_fk_result_quaternion_order_is_converted():
    pose = pose_from_fk_result([0.4, 0.1, 0.5, 0.9, 0.1, 0.2, 0.3])

    assert pose.position == pytest.approx((0.4, 0.1, 0.5))
    assert pose.orientation == pytest.approx(
        (0.1025978352, 0.2051956704, 0.3077935056, 0.9233805169)
    )


def test_pose_comparison_zero_for_quaternion_sign_flip():
    left = Pose3D((0.4, 0.0, 0.5), (0.1, 0.2, 0.3, 0.9))
    right = Pose3D((0.4, 0.0, 0.5), (-0.1, -0.2, -0.3, -0.9))

    result = compare_poses(left, right)

    assert result.position_error_m == pytest.approx(0.0)
    assert result.orientation_error_rad == pytest.approx(0.0)


def test_feedback_store_rate_and_rejection_statistics():
    store = FeedbackStore(rate_window_size=4)
    store.update(snapshot_from_api2_state(make_state(), 1.00))
    store.update(snapshot_from_api2_state(make_state(), 1.01))
    store.update(snapshot_from_api2_state(make_state(), 1.02))
    store.reject("bad packet")

    stats = store.statistics()

    assert stats.received == 3
    assert stats.rejected == 1
    assert stats.current_hz == pytest.approx(100.0)
    assert stats.last_error == "bad packet"
