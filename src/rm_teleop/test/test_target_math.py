import math

import pytest

from rm_teleop.target_math import (
    Pose3D,
    generate_raw_target,
    limit_pose_step,
    matrix_multiply,
    matrix_to_quaternion,
    pose_within_workspace,
    quaternion_angular_distance,
    quaternion_to_matrix,
)


IDENTITY = (0.0, 0.0, 0.0, 1.0)


def axis_angle_quaternion(axis, angle_rad):
    half = angle_rad * 0.5
    scale = math.sin(half)
    return (
        axis[0] * scale,
        axis[1] * scale,
        axis[2] * scale,
        math.cos(half),
    )


def assert_vector_close(actual, expected, tolerance=1.0e-9):
    assert actual == pytest.approx(expected, abs=tolerance)


def quaternion_axis(quaternion, angle_rad):
    scale = math.sin(angle_rad * 0.5)
    return (
        quaternion[0] / scale,
        quaternion[1] / scale,
        quaternion[2] / scale,
    )


def test_position_mapping_uses_confirmed_matrix():
    pico_anchor = Pose3D((0.0, 0.0, 0.0), IDENTITY)
    flange_anchor = Pose3D((0.5, 0.0, 0.4), IDENTITY)

    target = generate_raw_target(
        pico_anchor,
        Pose3D((1.0, 2.0, 3.0), IDENTITY),
        flange_anchor,
        0.25,
    )

    # [PICO x, y, z] -> [Base -z, -y, +x].
    assert_vector_close(target.position, (-0.25, -0.5, 0.65))


def test_translation_basis_directions_are_fixed():
    anchor = Pose3D((0.0, 0.0, 0.0), IDENTITY)
    cases = (
        ((1.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
        ((0.0, 1.0, 0.0), (0.0, -1.0, 0.0)),
        ((0.0, 0.0, 1.0), (-1.0, 0.0, 0.0)),
    )

    for pico_position, expected_base_position in cases:
        target = generate_raw_target(
            anchor,
            Pose3D(pico_position, IDENTITY),
            anchor,
            1.0,
        )
        assert_vector_close(target.position, expected_base_position)


def test_rotation_basis_directions_match_physical_view_axes():
    anchor = Pose3D((0.0, 0.0, 0.0), IDENTITY)
    angle = math.radians(20.0)
    cases = (
        # PICO +Rx -> Base -Rz.
        ((1.0, 0.0, 0.0), (0.0, 0.0, -1.0)),
        # PICO +Ry -> Base +Ry.
        ((0.0, 1.0, 0.0), (0.0, 1.0, 0.0)),
        # PICO +Rz -> Base +Rx.
        ((0.0, 0.0, 1.0), (1.0, 0.0, 0.0)),
    )

    for pico_axis, expected_base_axis in cases:
        target = generate_raw_target(
            anchor,
            Pose3D((0.0, 0.0, 0.0), axis_angle_quaternion(pico_axis, angle)),
            anchor,
            1.0,
        )
        assert_vector_close(
            quaternion_axis(target.orientation, angle),
            expected_base_axis,
            tolerance=1.0e-8,
        )


def test_rotation_mapping_uses_fixed_pico_axes_at_nonidentity_anchor():
    angle = math.radians(15.0)
    anchor_rotation = quaternion_to_matrix(
        axis_angle_quaternion((0.3, 0.7, -0.2), math.radians(45.0))
    )
    fixed_pico_delta = quaternion_to_matrix(
        axis_angle_quaternion((0.0, 0.0, 1.0), angle)
    )
    current_rotation = matrix_multiply(fixed_pico_delta, anchor_rotation)

    target = generate_raw_target(
        Pose3D((0.0, 0.0, 0.0), matrix_to_quaternion(anchor_rotation)),
        Pose3D((0.0, 0.0, 0.0), matrix_to_quaternion(current_rotation)),
        Pose3D((0.0, 0.0, 0.0), IDENTITY),
        1.0,
    )

    # Fixed PICO +Rz remains Base +Rx regardless of activation hand attitude.
    assert_vector_close(
        quaternion_axis(target.orientation, angle),
        (1.0, 0.0, 0.0),
        tolerance=1.0e-8,
    )


def test_activation_anchors_produce_zero_delta_target():
    pico = Pose3D((0.1, 1.2, 0.3), (0.2, -0.1, 0.3, 0.9))
    flange = Pose3D((0.4, -0.2, 0.5), (0.1, 0.2, -0.1, 0.95))

    target = generate_raw_target(pico, pico, flange, 0.25)

    assert_vector_close(target.position, flange.position)
    assert quaternion_angular_distance(target.orientation, flange.orientation) < 1.0e-7


def test_quaternion_sign_flip_does_not_change_target():
    pico_anchor = Pose3D((0.0, 0.0, 0.0), (0.2, -0.1, 0.3, 0.9))
    pico_current = Pose3D(
        (0.0, 0.0, 0.0),
        tuple(-value for value in pico_anchor.orientation),
    )
    flange = Pose3D((0.4, 0.0, 0.5), IDENTITY)

    target = generate_raw_target(pico_anchor, pico_current, flange, 0.25)

    assert quaternion_angular_distance(target.orientation, flange.orientation) < 1.0e-9


def test_pose_step_limits_translation_and_rotation():
    current = Pose3D((0.0, 0.0, 0.0), IDENTITY)
    target = Pose3D(
        (1.0, 0.0, 0.0),
        axis_angle_quaternion((0.0, 0.0, 1.0), math.radians(90.0)),
    )

    limited = limit_pose_step(
        current,
        target,
        max_translation_distance=0.01,
        max_rotation_angle_rad=math.radians(5.0),
    )

    assert_vector_close(limited.position, (0.01, 0.0, 0.0))
    assert quaternion_angular_distance(
        current.orientation,
        limited.orientation,
    ) == pytest.approx(math.radians(5.0), abs=1.0e-9)


def test_zero_speed_freezes_both_components():
    current = Pose3D((0.1, 0.2, 0.3), IDENTITY)
    target = Pose3D(
        (0.5, -0.2, 0.9),
        axis_angle_quaternion((1.0, 0.0, 0.0), math.radians(30.0)),
    )

    limited = limit_pose_step(current, target, 0.0, 0.0)

    assert limited.position == pytest.approx(current.position)
    assert quaternion_angular_distance(limited.orientation, current.orientation) < 1.0e-12


def test_workspace_is_inclusive_and_rejects_outside():
    minimum = (0.1, -0.5, 0.2)
    maximum = (0.8, 0.5, 1.0)

    assert pose_within_workspace(
        Pose3D(minimum, IDENTITY),
        minimum,
        maximum,
    )
    assert not pose_within_workspace(
        Pose3D((0.09, 0.0, 0.5), IDENTITY),
        minimum,
        maximum,
    )
