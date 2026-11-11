import math

import pytest

from rm_teleop.target_generator_core import (
    FlangeInput,
    PicoInput,
    TargetGeneratorConfig,
    TargetGeneratorCore,
)
from rm_teleop.target_math import Pose3D, quaternion_angular_distance


IDENTITY = (0.0, 0.0, 0.0, 1.0)
PALM = Pose3D((0.1, 1.1, 0.3), IDENTITY)
FLANGE = Pose3D((0.4, 0.0, 0.5), IDENTITY)


def pico(
    now,
    *,
    seq=1,
    enabled=False,
    valid=True,
    reset_origin=False,
    palm_pose=PALM,
):
    return PicoInput(
        seq=seq,
        received_monotonic=now,
        packet_valid=valid,
        transport_alive=valid,
        hand_teleop_enabled=enabled,
        palm_valid=enabled,
        reset_origin=reset_origin,
        palm_pose=palm_pose,
    )


def prepare_active_core(config=None):
    core = TargetGeneratorCore(config or TargetGeneratorConfig())
    core.update_flange(FlangeInput(0.0, FLANGE))
    core.update_pico(pico(0.0, enabled=False))
    assert core.step(0.0).reason == "waiting_for_enable"
    core.update_pico(pico(0.01, seq=2, enabled=True))
    output = core.step(0.01)
    assert output.active
    assert output.reason == "activated"
    return core


def test_requires_disabled_state_before_first_activation():
    core = TargetGeneratorCore(TargetGeneratorConfig())
    core.update_flange(FlangeInput(0.0, FLANGE))
    core.update_pico(pico(0.0, enabled=True))

    output = core.step(0.0)

    assert not output.active
    assert output.reason == "awaiting_enable_release"


def test_activation_first_output_equals_current_flange_pose():
    core = prepare_active_core()
    output = core.step(0.02)

    assert output.active
    assert output.target_valid
    assert output.limited_target_pose.position == pytest.approx(FLANGE.position)
    assert quaternion_angular_distance(
        output.limited_target_pose.orientation,
        FLANGE.orientation,
    ) < 1.0e-9


def test_translation_mapping_and_speed_limit():
    config = TargetGeneratorConfig(
        control_hz=100.0,
        max_translation_speed_mps=0.1,
        max_rotation_speed_rad_s=math.radians(8.0),
        motion_gate_enabled=False,
        input_filter_time_constant_sec=0.0,
    )
    core = prepare_active_core(config)

    moved_palm = Pose3D((0.2, 1.1, 0.3), IDENTITY)
    core.update_flange(FlangeInput(0.02, FLANGE))
    core.update_pico(pico(0.02, seq=3, enabled=True, palm_pose=moved_palm))
    output = core.step(0.02)

    # PICO +X maps to robot +Z; scale=0.25, raw delta=0.025 m.
    assert output.raw_target_pose.position == pytest.approx((0.4, 0.0, 0.525))
    # 0.1 m/s at 100 Hz allows only 0.001 m this cycle.
    assert output.limited_target_pose.position == pytest.approx((0.4, 0.0, 0.501))


def test_rotation_axis_limit_is_applied_in_base_frame():
    config = TargetGeneratorConfig(
        control_hz=100.0,
        max_rotation_axis_speeds_rad_s=tuple(map(math.radians, (10.0, 20.0, 30.0))),
        motion_gate_enabled=False,
        input_filter_time_constant_sec=0.0,
    )
    core = prepare_active_core(config)
    # PICO +Rz maps to Base +Rx, which is limited to 10 deg/s.
    angle = math.radians(20.0)
    moved = Pose3D(PALM.position, (0.0, 0.0, math.sin(angle / 2), math.cos(angle / 2)))
    core.update_flange(FlangeInput(0.02, FLANGE))
    core.update_pico(pico(0.02, seq=3, enabled=True, palm_pose=moved))
    output = core.step(0.02)
    assert quaternion_angular_distance(
        output.limited_target_pose.orientation, FLANGE.orientation
    ) == pytest.approx(math.radians(0.1), abs=1e-7)


def test_pico_stale_exits_active_and_requires_new_edge():
    core = prepare_active_core()

    output = core.step(0.20)
    assert not output.active
    assert output.reason == "pico_stale"

    core.update_flange(FlangeInput(0.21, FLANGE))
    core.update_pico(pico(0.21, seq=3, enabled=True))
    output = core.step(0.21)
    assert not output.active
    assert output.reason == "awaiting_new_enable_rising_edge"


def test_reset_origin_exits_active():
    core = prepare_active_core()
    core.update_flange(FlangeInput(0.02, FLANGE))
    core.update_pico(pico(0.02, seq=3, enabled=True, reset_origin=True))

    output = core.step(0.02)

    assert not output.active
    assert not output.target_valid
    assert output.reason == "reset_origin"


def test_failed_activation_consumes_rising_edge():
    core = TargetGeneratorCore(TargetGeneratorConfig())
    core.update_pico(pico(0.0, enabled=False))
    core.step(0.0)
    core.update_pico(pico(0.01, seq=2, enabled=True))

    rejected = core.step(0.01)
    assert rejected.reason == "activation_rejected:no_flange_pose"

    core.update_flange(FlangeInput(0.02, FLANGE))
    still_inactive = core.step(0.02)
    assert still_inactive.reason == "awaiting_new_enable_rising_edge"


def test_workspace_violation_exits_active():
    config = TargetGeneratorConfig(
        motion_gate_enabled=False,
        input_filter_time_constant_sec=0.0,
        workspace_limit_enabled=True,
        workspace_min_xyz=(0.0, -0.5, 0.0),
        workspace_max_xyz=(0.41, 0.5, 1.0),
    )
    core = prepare_active_core(config)
    moved_palm = Pose3D((0.1, 1.1, -0.2), IDENTITY)
    core.update_flange(FlangeInput(0.02, FLANGE))
    core.update_pico(pico(0.02, seq=3, enabled=True, palm_pose=moved_palm))

    output = core.step(0.02)

    assert not output.active
    assert output.reason == "workspace_violation"



def z_rotation(degrees: float):
    half = math.radians(degrees) * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def motion_config(**overrides):
    values = dict(
        control_hz=100.0,
        pico_freshness_timeout_sec=1.0,
        flange_freshness_timeout_sec=1.0,
        max_translation_speed_mps=0.1,
        max_rotation_speed_rad_s=math.radians(30.0),
        motion_gate_enabled=True,
        input_filter_time_constant_sec=0.0,
        motion_detection_window_sec=0.10,
        motion_start_translation_m=0.004,
        motion_start_rotation_rad=math.radians(2.0),
        motion_stop_translation_m=0.001,
        motion_stop_rotation_rad=math.radians(0.5),
        motion_stop_hold_sec=0.10,
    )
    values.update(overrides)
    return TargetGeneratorConfig(**values)


def update_active(core, now, pose, seq):
    core.update_flange(FlangeInput(now, FLANGE))
    core.update_pico(pico(now, seq=seq, enabled=True, palm_pose=pose))
    return core.step(now)


def test_input_filter_smooths_single_pico_step():
    config = motion_config(
        motion_gate_enabled=False,
        input_filter_time_constant_sec=0.10,
    )
    core = prepare_active_core(config)
    moved = Pose3D((0.2, 1.1, 0.3), z_rotation(10.0))

    output = update_active(core, 0.11, moved, 3)

    # dt=0.10s and tau=0.10s gives alpha=1-exp(-1), so the filtered
    # position and rotation must lie strictly between the two samples.
    assert PALM.position[0] < output.filtered_pico_pose.position[0] < moved.position[0]
    filtered_angle = math.degrees(
        quaternion_angular_distance(IDENTITY, output.filtered_pico_pose.orientation)
    )
    assert 0.0 < filtered_angle < 10.0


def test_small_static_jitter_remains_still_and_freezes_target():
    core = prepare_active_core(motion_config())

    for index, now in enumerate((0.05, 0.10, 0.15, 0.20, 0.25), start=3):
        sign = -1.0 if index % 2 else 1.0
        jitter = Pose3D(
            (0.1 + sign * 0.0008, 1.1, 0.3),
            z_rotation(sign * 0.25),
        )
        output = update_active(core, now, jitter, index)
        assert output.input_motion_state == "STILL"
        assert output.still_hold_active
        assert output.reason == "still_hold"
        assert output.limited_target_pose.position == pytest.approx(FLANGE.position)
        assert quaternion_angular_distance(
            output.limited_target_pose.orientation,
            FLANGE.orientation,
        ) < 1.0e-9


def test_windowed_motion_starts_only_after_threshold_and_reanchors_without_jump():
    core = prepare_active_core(motion_config())

    below = Pose3D((0.103, 1.1, 0.3), IDENTITY)
    output = update_active(core, 0.08, below, 3)
    assert output.input_motion_state == "STILL"

    trigger = Pose3D((0.105, 1.1, 0.3), IDENTITY)
    output = update_active(core, 0.12, trigger, 4)
    assert output.input_motion_state == "MOVING"
    # The transition cycle is re-anchored at the held target, so it is zero jump.
    assert output.limited_target_pose.position == pytest.approx(FLANGE.position)

    moved_further = Pose3D((0.110, 1.1, 0.3), IDENTITY)
    output = update_active(core, 0.16, moved_further, 5)
    assert output.input_motion_state == "MOVING"
    assert output.raw_target_pose.position[2] > FLANGE.position[2]


def test_motion_stop_latches_final_target_and_settles_before_still():
    config = motion_config(
        max_translation_speed_mps=0.05,
        motion_detection_window_sec=0.05,
        motion_stop_hold_sec=0.05,
    )
    core = prepare_active_core(config)

    trigger = Pose3D((0.110, 1.1, 0.3), IDENTITY)
    output = update_active(core, 0.07, trigger, 3)
    assert output.input_motion_state == "MOVING"

    far = Pose3D((0.210, 1.1, 0.3), IDENTITY)
    output = update_active(core, 0.08, far, 4)
    assert output.raw_target_pose.position[2] - output.limited_target_pose.position[2] > 0.01

    update_active(core, 0.15, far, 5)
    output = update_active(core, 0.21, far, 6)
    assert output.input_motion_state == "SETTLING"
    assert output.settling_active
    assert output.reason == "settling"
    latched_target = output.raw_target_pose
    assert output.settling_remaining_translation_m > 0.0

    # While settling, small PICO jitter must not change the latched endpoint.
    now = 0.21
    for seq in range(7, 100):
        now += 0.01
        jitter = Pose3D(
            (far.position[0] + (0.0003 if seq % 2 else -0.0003), 1.1, 0.3),
            IDENTITY,
        )
        output = update_active(core, now, jitter, seq)
        assert output.raw_target_pose.position == pytest.approx(
            latched_target.position
        )
        if output.input_motion_state == "STILL":
            break

    assert output.input_motion_state == "STILL"
    assert output.reason == "still_hold"
    assert not output.settling_active
    assert output.limited_target_pose.position == pytest.approx(
        latched_target.position
    )


def test_rotation_backlog_is_completed_during_settling():
    config = motion_config(
        max_rotation_speed_rad_s=math.radians(20.0),
        motion_detection_window_sec=0.05,
        motion_stop_hold_sec=0.05,
    )
    core = prepare_active_core(config)

    trigger = Pose3D(PALM.position, z_rotation(3.0))
    update_active(core, 0.07, trigger, 3)
    far = Pose3D(PALM.position, z_rotation(33.0))
    moving = update_active(core, 0.08, far, 4)
    assert math.degrees(
        quaternion_angular_distance(
            moving.raw_target_pose.orientation,
            moving.limited_target_pose.orientation,
        )
    ) > 10.0

    update_active(core, 0.15, far, 5)
    output = update_active(core, 0.21, far, 6)
    assert output.input_motion_state == "SETTLING"
    latched_orientation = output.raw_target_pose.orientation

    now = 0.21
    for seq in range(7, 260):
        now += 0.01
        output = update_active(core, now, far, seq)
        if output.input_motion_state == "STILL":
            break

    assert output.input_motion_state == "STILL"
    assert quaternion_angular_distance(
        output.limited_target_pose.orientation,
        latched_orientation,
    ) < 1.0e-9


def test_new_motion_interrupts_settling_without_target_jump():
    config = motion_config(
        max_translation_speed_mps=0.02,
        motion_detection_window_sec=0.05,
        motion_stop_hold_sec=0.05,
    )
    core = prepare_active_core(config)

    trigger = Pose3D((0.110, 1.1, 0.3), IDENTITY)
    update_active(core, 0.07, trigger, 3)
    far = Pose3D((0.210, 1.1, 0.3), IDENTITY)
    update_active(core, 0.08, far, 4)
    update_active(core, 0.15, far, 5)
    settling = update_active(core, 0.21, far, 6)
    assert settling.input_motion_state == "SETTLING"
    previous_target = settling.limited_target_pose

    moved_again = Pose3D((0.230, 1.1, 0.3), IDENTITY)
    output = update_active(core, 0.27, moved_again, 7)
    assert output.input_motion_state == "MOVING"
    assert not output.settling_active
    # Re-anchoring at the interruption point makes the transition zero-jump.
    assert output.limited_target_pose.position == pytest.approx(
        previous_target.position
    )


def test_config_rejects_motion_start_threshold_below_stop_threshold():
    config = motion_config(
        motion_start_translation_m=0.001,
        motion_stop_translation_m=0.002,
    )
    with pytest.raises(ValueError, match="motion_start_translation_m"):
        config.validate()
