from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable


Vector3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]  # x, y, z, w
Matrix3 = tuple[
    tuple[float, float, float],
    tuple[float, float, float],
    tuple[float, float, float],
]


# PICO tracking-space translation axis -> RM65 Base axis.
#
# PICO +X (hand right)   -> RM65 Base +Z (physical right)
# PICO +Y (hand up)      -> RM65 Base -Y (physical up)
# PICO +Z (hand forward) -> RM65 Base -X (physical forward)
PICO_TRANSLATION_TO_RM65_BASE: Matrix3 = (
    (0.0, 0.0, -1.0),
    (0.0, -1.0, 0.0),
    (1.0, 0.0, 0.0),
)


# PICO fixed-view rotation axis -> RM65 Base rotation axis.
#
# The physical rotation axes were verified on the robot, and all three
# directions were found to be opposite to the initial mapping.  Therefore the
# complete rotation vector mapping is the negative of the translation mapping:
#
# PICO +Rx (right axis)   -> RM65 Base -Rz
# PICO +Ry (up axis)      -> RM65 Base +Ry
# PICO +Rz (forward axis) -> RM65 Base +Rx
#
# Only this rotation-vector mapping is negated.  The validated translation
# mapping above is unchanged.  The matrix is applied to the vector part of the
# relative quaternion; the scalar part remains unchanged, which reverses the
# commanded rotation direction while preserving the verified rotation axis.
PICO_ROTATION_VECTOR_TO_RM65_BASE: Matrix3 = (
    (0.0, 0.0, 1.0),
    (0.0, 1.0, 0.0),
    (-1.0, 0.0, 0.0),
)


@dataclass(frozen=True)
class Pose3D:
    position: Vector3
    orientation: Quaternion


IDENTITY_QUATERNION: Quaternion = (0.0, 0.0, 0.0, 1.0)
ZERO_VECTOR: Vector3 = (0.0, 0.0, 0.0)
IDENTITY_POSE = Pose3D(ZERO_VECTOR, IDENTITY_QUATERNION)


def _tuple3(values: Iterable[float]) -> Vector3:
    result = tuple(float(value) for value in values)
    if len(result) != 3:
        raise ValueError("expected exactly 3 values")
    return result  # type: ignore[return-value]


def _tuple4(values: Iterable[float]) -> Quaternion:
    result = tuple(float(value) for value in values)
    if len(result) != 4:
        raise ValueError("expected exactly 4 values")
    return result  # type: ignore[return-value]


def is_finite_vector3(value: Vector3) -> bool:
    return all(math.isfinite(component) for component in value)


def vector_add(left: Vector3, right: Vector3) -> Vector3:
    return (
        left[0] + right[0],
        left[1] + right[1],
        left[2] + right[2],
    )


def vector_subtract(left: Vector3, right: Vector3) -> Vector3:
    return (
        left[0] - right[0],
        left[1] - right[1],
        left[2] - right[2],
    )


def vector_scale(value: Vector3, scale: float) -> Vector3:
    return (value[0] * scale, value[1] * scale, value[2] * scale)


def vector_norm(value: Vector3) -> float:
    return math.sqrt(sum(component * component for component in value))


def matrix_transpose(matrix: Matrix3) -> Matrix3:
    return (
        (matrix[0][0], matrix[1][0], matrix[2][0]),
        (matrix[0][1], matrix[1][1], matrix[2][1]),
        (matrix[0][2], matrix[1][2], matrix[2][2]),
    )


def matrix_multiply(left: Matrix3, right: Matrix3) -> Matrix3:
    return tuple(
        tuple(
            sum(left[row][index] * right[index][column] for index in range(3))
            for column in range(3)
        )
        for row in range(3)
    )  # type: ignore[return-value]


def matrix_vector_multiply(matrix: Matrix3, vector: Vector3) -> Vector3:
    return (
        sum(matrix[0][index] * vector[index] for index in range(3)),
        sum(matrix[1][index] * vector[index] for index in range(3)),
        sum(matrix[2][index] * vector[index] for index in range(3)),
    )


def normalize_quaternion(
    quaternion: Iterable[float],
    *,
    minimum_norm: float = 1.0e-9,
) -> Quaternion:
    q = _tuple4(quaternion)
    if not all(math.isfinite(component) for component in q):
        raise ValueError("quaternion contains a non-finite component")

    norm = math.sqrt(sum(component * component for component in q))
    if norm < minimum_norm:
        raise ValueError("quaternion norm is too small")

    return (
        q[0] / norm,
        q[1] / norm,
        q[2] / norm,
        q[3] / norm,
    )


def quaternion_to_matrix(quaternion: Quaternion) -> Matrix3:
    x, y, z, w = normalize_quaternion(quaternion)

    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    return (
        (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
        (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
        (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
    )


def matrix_to_quaternion(matrix: Matrix3) -> Quaternion:
    trace = matrix[0][0] + matrix[1][1] + matrix[2][2]

    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (matrix[2][1] - matrix[1][2]) / scale
        y = (matrix[0][2] - matrix[2][0]) / scale
        z = (matrix[1][0] - matrix[0][1]) / scale
    elif matrix[0][0] > matrix[1][1] and matrix[0][0] > matrix[2][2]:
        scale = math.sqrt(
            max(
                0.0,
                1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2],
            )
        ) * 2.0
        if scale <= 1.0e-12:
            raise ValueError("invalid rotation matrix")
        w = (matrix[2][1] - matrix[1][2]) / scale
        x = 0.25 * scale
        y = (matrix[0][1] + matrix[1][0]) / scale
        z = (matrix[0][2] + matrix[2][0]) / scale
    elif matrix[1][1] > matrix[2][2]:
        scale = math.sqrt(
            max(
                0.0,
                1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2],
            )
        ) * 2.0
        if scale <= 1.0e-12:
            raise ValueError("invalid rotation matrix")
        w = (matrix[0][2] - matrix[2][0]) / scale
        x = (matrix[0][1] + matrix[1][0]) / scale
        y = 0.25 * scale
        z = (matrix[1][2] + matrix[2][1]) / scale
    else:
        scale = math.sqrt(
            max(
                0.0,
                1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1],
            )
        ) * 2.0
        if scale <= 1.0e-12:
            raise ValueError("invalid rotation matrix")
        w = (matrix[1][0] - matrix[0][1]) / scale
        x = (matrix[0][2] + matrix[2][0]) / scale
        y = (matrix[1][2] + matrix[2][1]) / scale
        z = 0.25 * scale

    return normalize_quaternion((x, y, z, w))


def quaternion_dot(left: Quaternion, right: Quaternion) -> float:
    return sum(left[index] * right[index] for index in range(4))


def quaternion_angular_distance(left: Quaternion, right: Quaternion) -> float:
    left_normalized = normalize_quaternion(left)
    right_normalized = normalize_quaternion(right)
    dot = abs(quaternion_dot(left_normalized, right_normalized))
    dot = max(-1.0, min(1.0, dot))
    return 2.0 * math.acos(dot)


def quaternion_slerp(
    start: Quaternion,
    target: Quaternion,
    fraction: float,
) -> Quaternion:
    if not math.isfinite(fraction):
        raise ValueError("SLERP fraction must be finite")

    fraction = max(0.0, min(1.0, fraction))
    start_q = normalize_quaternion(start)
    target_q = normalize_quaternion(target)

    dot = quaternion_dot(start_q, target_q)
    if dot < 0.0:
        target_q = tuple(-component for component in target_q)  # type: ignore[assignment]
        dot = -dot

    dot = max(-1.0, min(1.0, dot))

    if dot > 0.9995:
        interpolated = tuple(
            start_q[index] + fraction * (target_q[index] - start_q[index])
            for index in range(4)
        )
        return normalize_quaternion(interpolated)

    angle = math.acos(dot)
    sin_angle = math.sin(angle)
    if abs(sin_angle) <= 1.0e-12:
        return start_q

    start_scale = math.sin((1.0 - fraction) * angle) / sin_angle
    target_scale = math.sin(fraction * angle) / sin_angle

    return normalize_quaternion(
        tuple(
            start_scale * start_q[index] + target_scale * target_q[index]
            for index in range(4)
        )
    )


def validate_pose(pose: Pose3D) -> Pose3D:
    position = _tuple3(pose.position)
    if not is_finite_vector3(position):
        raise ValueError("pose position contains a non-finite component")

    orientation = normalize_quaternion(pose.orientation)
    return Pose3D(position=position, orientation=orientation)


def generate_raw_target(
    pico_anchor: Pose3D,
    pico_current: Pose3D,
    flange_anchor: Pose3D,
    position_scale: float,
) -> Pose3D:
    if not math.isfinite(position_scale) or position_scale < 0.0:
        raise ValueError("position_scale must be finite and non-negative")

    pico_anchor = validate_pose(pico_anchor)
    pico_current = validate_pose(pico_current)
    flange_anchor = validate_pose(flange_anchor)

    # Translation is measured in the PICO tracking frame and then mapped once
    # into the RM65 Base frame.
    pico_delta = vector_subtract(pico_current.position, pico_anchor.position)
    robot_delta = matrix_vector_multiply(
        PICO_TRANSLATION_TO_RM65_BASE,
        pico_delta,
    )
    target_position = vector_add(
        flange_anchor.position,
        vector_scale(robot_delta, position_scale),
    )

    pico_anchor_rotation = quaternion_to_matrix(pico_anchor.orientation)
    pico_current_rotation = quaternion_to_matrix(pico_current.orientation)
    flange_anchor_rotation = quaternion_to_matrix(flange_anchor.orientation)

    # Relative hand rotation expressed in the fixed PICO tracking/view frame:
    #
    #     delta_spatial = R_current * R_anchor^T
    #
    # The user tests rotations around fixed PICO axes:
    # +X right, +Y up, +Z forward. Using R_anchor^T * R_current here would
    # instead express the rotation in the hand-local frame at activation and
    # would mix the observed X/Y/Z axes whenever the hand starts tilted.
    pico_spatial_delta = matrix_multiply(
        pico_current_rotation,
        matrix_transpose(pico_anchor_rotation),
    )

    # Convert the relative rotation to a quaternion, map its vector part as an
    # axis-angle vector, and keep the scalar part unchanged.  This gives the
    # exact physical/view mapping above without the sign reversal caused by an
    # improper change-of-basis conjugation.
    pico_delta_quaternion = matrix_to_quaternion(pico_spatial_delta)
    mapped_vector = matrix_vector_multiply(
        PICO_ROTATION_VECTOR_TO_RM65_BASE,
        (
            pico_delta_quaternion[0],
            pico_delta_quaternion[1],
            pico_delta_quaternion[2],
        ),
    )
    robot_delta_quaternion = normalize_quaternion(
        (
            mapped_vector[0],
            mapped_vector[1],
            mapped_vector[2],
            pico_delta_quaternion[3],
        )
    )
    robot_base_delta = quaternion_to_matrix(robot_delta_quaternion)

    # Left multiplication applies the mapped relative rotation about fixed
    # RM65 Base axes. Right multiplication would instead rotate about the
    # flange/tool-local axes.
    target_rotation = matrix_multiply(
        robot_base_delta,
        flange_anchor_rotation,
    )

    return Pose3D(
        position=target_position,
        orientation=matrix_to_quaternion(target_rotation),
    )


def limit_translation(
    current: Vector3,
    target: Vector3,
    max_distance: float,
) -> Vector3:
    if not math.isfinite(max_distance) or max_distance < 0.0:
        raise ValueError("max_distance must be finite and non-negative")

    delta = vector_subtract(target, current)
    distance = vector_norm(delta)
    if distance <= max_distance or distance <= 1.0e-12:
        return target

    return vector_add(current, vector_scale(delta, max_distance / distance))


def limit_rotation(
    current: Quaternion,
    target: Quaternion,
    max_angle_rad: float,
) -> Quaternion:
    if not math.isfinite(max_angle_rad) or max_angle_rad < 0.0:
        raise ValueError("max_angle_rad must be finite and non-negative")

    angle = quaternion_angular_distance(current, target)
    if angle <= max_angle_rad or angle <= 1.0e-12:
        return normalize_quaternion(target)

    return quaternion_slerp(current, target, max_angle_rad / angle)


def limit_pose_step(
    current: Pose3D,
    target: Pose3D,
    max_translation_distance: float,
    max_rotation_angle_rad: float,
) -> Pose3D:
    current = validate_pose(current)
    target = validate_pose(target)

    return Pose3D(
        position=limit_translation(
            current.position,
            target.position,
            max_translation_distance,
        ),
        orientation=limit_rotation(
            current.orientation,
            target.orientation,
            max_rotation_angle_rad,
        ),
    )


def validate_workspace_bounds(
    minimum_xyz: Vector3,
    maximum_xyz: Vector3,
) -> tuple[Vector3, Vector3]:
    minimum = _tuple3(minimum_xyz)
    maximum = _tuple3(maximum_xyz)

    for index in range(3):
        if not math.isfinite(minimum[index]) or not math.isfinite(maximum[index]):
            raise ValueError("workspace limits must be finite")
        if minimum[index] >= maximum[index]:
            raise ValueError("workspace minimum must be smaller than maximum")

    return minimum, maximum


def pose_within_workspace(
    pose: Pose3D,
    minimum_xyz: Vector3,
    maximum_xyz: Vector3,
) -> bool:
    pose = validate_pose(pose)
    minimum, maximum = validate_workspace_bounds(minimum_xyz, maximum_xyz)

    return all(
        minimum[index] <= pose.position[index] <= maximum[index]
        for index in range(3)
    )