import pytest

from rm_teleop.rm_api2_adapter import RmApi2ReadOnlyAdapter
from rm_teleop.target_math import Pose3D


class FakeMatrix:
    def __init__(self, row=None, col=None, data=None):
        self.row = row
        self.col = col
        self.data = data


class FakeModule:
    rm_Mat_t = FakeMatrix


def test_pose_to_rm_matrix_builds_absolute_4x4_transform():
    adapter = object.__new__(RmApi2ReadOnlyAdapter)
    adapter.module = FakeModule()

    matrix = adapter._pose_to_rm_matrix(
        Pose3D(
            position=(0.4, -0.2, 0.5),
            orientation=(0.0, 0.0, 0.0, 1.0),
        )
    )

    assert matrix.row == 4
    assert matrix.col == 4
    expected = [
        [1.0, 0.0, 0.0, 0.4],
        [0.0, 1.0, 0.0, -0.2],
        [0.0, 0.0, 1.0, 0.5],
        [0.0, 0.0, 0.0, 1.0],
    ]

    for actual_row, expected_row in zip(matrix.data, expected):
        assert actual_row == pytest.approx(expected_row)
