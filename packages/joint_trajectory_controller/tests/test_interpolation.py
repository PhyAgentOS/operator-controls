from __future__ import annotations

import pytest
from forge_joint_trajectory_controller.interpolation import prepare_trajectory, sample_trajectory
from forge_msgs import JointTrajectory, JointTrajectoryPoint


def _point(
    position: float,
    time_ns: int,
    *,
    velocity: float | None = None,
    acceleration: float | None = None,
    effort: float | None = None,
) -> JointTrajectoryPoint:
    return JointTrajectoryPoint(
        positions=[position],
        velocities=[] if velocity is None else [velocity],
        accelerations=[] if acceleration is None else [acceleration],
        effort=[] if effort is None else [effort],
        time_from_start_ns=time_ns,
    )


def test_linear_interpolation() -> None:
    prepared = prepare_trajectory(
        JointTrajectory(joint_names=["j1"], points=[_point(0.0, 0), _point(1.0, 1_000_000_000)]),
        (0.0,),
        (0.0,),
    )

    sample = sample_trajectory(prepared, 250_000_000)

    assert sample.positions == pytest.approx((0.25,))
    assert sample.velocities == pytest.approx((1.0,))
    assert sample.point_index == 1


def test_cubic_hermite_interpolation() -> None:
    prepared = prepare_trajectory(
        JointTrajectory(
            joint_names=["j1"],
            points=[_point(0.0, 0, velocity=0.0), _point(1.0, 1_000_000_000, velocity=0.0)],
        ),
        (0.0,),
        (0.0,),
    )

    sample = sample_trajectory(prepared, 500_000_000)

    assert sample.positions == pytest.approx((0.5,))
    assert sample.velocities == pytest.approx((1.5,))


def test_inserts_virtual_actual_start() -> None:
    prepared = prepare_trajectory(
        JointTrajectory(joint_names=["j1"], points=[_point(1.0, 1_000_000_000)]),
        (0.25,),
        (0.0,),
    )

    assert prepared.has_virtual_start
    assert sample_trajectory(prepared, 0).positions == pytest.approx((0.25,))
    assert sample_trajectory(prepared, 500_000_000).positions == pytest.approx((0.625,))


def test_rejects_mixed_velocities_accelerations_and_effort() -> None:
    with pytest.raises(ValueError, match="velocities"):
        prepare_trajectory(
            JointTrajectory(
                joint_names=["j1"],
                points=[_point(0.0, 0), _point(1.0, 1_000_000_000, velocity=0.0)],
            ),
            (0.0,),
            (0.0,),
        )
    with pytest.raises(ValueError, match="accelerations"):
        prepare_trajectory(
            JointTrajectory(joint_names=["j1"], points=[_point(0.0, 0, acceleration=0.0)]),
            (0.0,),
            (0.0,),
        )
    with pytest.raises(ValueError, match="effort"):
        prepare_trajectory(
            JointTrajectory(joint_names=["j1"], points=[_point(0.0, 0, effort=0.0)]),
            (0.0,),
            (0.0,),
        )
