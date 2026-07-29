from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

from forge_msgs import JointTrajectory, JointTrajectoryPoint

InterpolationMode = Literal["linear", "cubic"]


@dataclass(frozen=True, slots=True)
class PreparedTrajectory:
    points: tuple[JointTrajectoryPoint, ...]
    mode: InterpolationMode
    duration_ns: int
    has_virtual_start: bool


@dataclass(frozen=True, slots=True)
class TrajectorySample:
    positions: tuple[float, ...]
    velocities: tuple[float, ...]
    point_index: int


def prepare_trajectory(
    trajectory: JointTrajectory,
    current_positions: tuple[float, ...],
    current_velocities: tuple[float, ...],
) -> PreparedTrajectory:
    """Validate supported fields and add an actual-state t=0 point when needed."""

    joint_count = len(trajectory.joint_names)
    if len(current_positions) != joint_count or len(current_velocities) != joint_count:
        raise ValueError(
            "current state must contain one position and velocity per trajectory joint"
        )
    if any(point.accelerations for point in trajectory.points):
        raise ValueError("trajectory accelerations are not supported")
    if any(point.effort for point in trajectory.points):
        raise ValueError("trajectory effort is not supported")

    velocity_presence = [bool(point.velocities) for point in trajectory.points]
    if any(velocity_presence) and not all(velocity_presence):
        raise ValueError("trajectory velocities must be specified for every point or none")
    mode: InterpolationMode = "cubic" if all(velocity_presence) else "linear"

    points = list(trajectory.points)
    has_virtual_start = points[0].time_from_start_ns > 0
    if has_virtual_start:
        points.insert(
            0,
            JointTrajectoryPoint(
                positions=list(current_positions),
                velocities=list(current_velocities) if mode == "cubic" else [],
                accelerations=[],
                effort=[],
                time_from_start_ns=0,
            ),
        )
    return PreparedTrajectory(
        points=tuple(points),
        mode=mode,
        duration_ns=points[-1].time_from_start_ns,
        has_virtual_start=has_virtual_start,
    )


def sample_trajectory(trajectory: PreparedTrajectory, elapsed_ns: int) -> TrajectorySample:
    if elapsed_ns < 0:
        raise ValueError("elapsed_ns must be non-negative")

    points = trajectory.points
    if elapsed_ns <= points[0].time_from_start_ns:
        return _point_sample(points[0], 0)
    if elapsed_ns >= trajectory.duration_ns:
        point_index = len(points) - 1 - (1 if trajectory.has_virtual_start else 0)
        return _point_sample(points[-1], max(0, point_index))

    upper_index = 1
    while elapsed_ns > points[upper_index].time_from_start_ns:
        upper_index += 1
    lower = points[upper_index - 1]
    upper = points[upper_index]
    segment_ns = upper.time_from_start_ns - lower.time_from_start_ns
    offset_ns = elapsed_ns - lower.time_from_start_ns
    ratio = offset_ns / segment_ns
    duration_s = segment_ns / 1_000_000_000.0

    if trajectory.mode == "linear":
        positions = tuple(
            start + ratio * (end - start)
            for start, end in zip(lower.positions, upper.positions, strict=True)
        )
        velocities = tuple(
            (end - start) / duration_s
            for start, end in zip(lower.positions, upper.positions, strict=True)
        )
    else:
        positions, velocities = _sample_cubic(lower, upper, ratio, duration_s)

    point_index = upper_index - (1 if trajectory.has_virtual_start else 0)
    return TrajectorySample(
        positions=positions,
        velocities=velocities,
        point_index=max(0, point_index),
    )


def _point_sample(point: JointTrajectoryPoint, point_index: int) -> TrajectorySample:
    velocities = point.velocities or [0.0] * len(point.positions)
    return TrajectorySample(
        positions=tuple(point.positions),
        velocities=tuple(velocities),
        point_index=point_index,
    )


def _sample_cubic(
    lower: JointTrajectoryPoint,
    upper: JointTrajectoryPoint,
    ratio: float,
    duration_s: float,
) -> tuple[tuple[float, ...], tuple[float, ...]]:
    u = ratio
    u2 = u * u
    u3 = u2 * u
    h00 = 2.0 * u3 - 3.0 * u2 + 1.0
    h10 = u3 - 2.0 * u2 + u
    h01 = -2.0 * u3 + 3.0 * u2
    h11 = u3 - u2

    dh00 = (6.0 * u2 - 6.0 * u) / duration_s
    dh10 = 3.0 * u2 - 4.0 * u + 1.0
    dh01 = (-6.0 * u2 + 6.0 * u) / duration_s
    dh11 = 3.0 * u2 - 2.0 * u

    positions: list[float] = []
    velocities: list[float] = []
    for p0, v0, p1, v1 in zip(
        lower.positions,
        lower.velocities,
        upper.positions,
        upper.velocities,
        strict=True,
    ):
        positions.append(h00 * p0 + h10 * duration_s * v0 + h01 * p1 + h11 * duration_s * v1)
        velocities.append(dh00 * p0 + dh10 * v0 + dh01 * p1 + dh11 * v1)
    return tuple(positions), tuple(velocities)
