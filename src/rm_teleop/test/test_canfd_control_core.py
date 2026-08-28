import math

import pytest

from rm_teleop.canfd_control_core import (
    MOTION_CONFIRMATION_TOKEN,
    CanfdSafetyConfig,
    command_interval_valid,
    limit_joint_step,
    motion_permission,
    tracking_error,
)


def test_motion_permission_requires_both_switch_and_exact_token():
    assert motion_permission(False, MOTION_CONFIRMATION_TOKEN) == (
        False,
        "allow_motion_false",
    )
    assert motion_permission(True, "wrong") == (
        False,
        "motion_confirmation_invalid",
    )
    assert motion_permission(True, MOTION_CONFIRMATION_TOKEN) == (True, "ok")


def test_safety_config_rejects_step_above_speed_budget():
    with pytest.raises(ValueError):
        CanfdSafetyConfig(
            control_dt_sec=0.005,
            max_command_joint_speed_deg_s=5.0,
            max_command_joint_step_deg=0.026,
        ).validate()


def test_limit_joint_step_clips_each_joint():
    result = limit_joint_step(
        current_deg=(0.0, 1.0, 2.0, 3.0, 4.0, 5.0),
        target_deg=(1.0, 0.0, 2.02, 2.98, 8.0, -2.0),
        max_step_deg=0.025,
    )
    assert result.command_deg == pytest.approx((0.025, 0.975, 2.02, 2.98, 4.025, 4.975))
    assert result.step_deg == pytest.approx((0.025, -0.025, 0.02, -0.02, 0.025, -0.025))
    assert result.max_abs_step_deg == pytest.approx(0.025)
    assert result.limited


def test_limit_joint_step_leaves_small_target_unchanged():
    target = (0.01, -0.02, 0.02, -0.015, 0.0, 0.025)
    result = limit_joint_step(
        current_deg=(0.0,) * 6,
        target_deg=target,
        max_step_deg=0.025,
    )
    assert result.command_deg == pytest.approx(target)
    assert not result.limited


def test_tracking_error_is_command_minus_actual():
    result = tracking_error(
        actual_deg=(1.0, 2.0, 3.0, 4.0, 5.0, 6.0),
        commanded_deg=(1.5, 1.5, 3.0, 4.25, 4.0, 6.0),
    )
    assert result.error_deg == pytest.approx((0.5, -0.5, 0.0, 0.25, -1.0, 0.0))
    assert result.max_abs_error_deg == pytest.approx(1.0)


def test_command_interval_first_send_is_valid():
    valid, interval = command_interval_valid(
        now_monotonic=10.0,
        previous_send_monotonic=None,
        max_interval_sec=0.010,
    )
    assert valid
    assert interval == -1.0


def test_command_interval_detects_stall():
    valid, interval = command_interval_valid(
        now_monotonic=10.011,
        previous_send_monotonic=10.0,
        max_interval_sec=0.010,
    )
    assert not valid
    assert interval == pytest.approx(0.011)


def test_high_follow_safety_profile_is_valid():
    CanfdSafetyConfig(
        control_dt_sec=0.005,
        max_command_joint_speed_deg_s=5.0,
        max_command_joint_step_deg=0.025,
        max_tracking_error_deg=1.0,
        max_command_interval_sec=0.010,
    ).validate()
