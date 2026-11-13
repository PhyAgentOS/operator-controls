from __future__ import annotations

import math
from collections import OrderedDict, deque
from dataclasses import dataclass
from typing import Literal

from forge_msgs import (
    FollowJointTrajectoryFeedback,
    FollowJointTrajectoryGoal,
    FollowJointTrajectoryResult,
    JointCommand,
    JointState,
    JointTolerance,
    JointTrajectoryPoint,
)

from .config import ControllerConfig
from .interpolation import (
    PreparedTrajectory,
    TrajectorySample,
    prepare_trajectory,
    sample_trajectory,
)

GoalStatus = Literal["succeeded", "aborted", "canceled"]
ControllerState = Literal["IDLE", "ACTIVE", "SETTLING", "HOLDING"]


@dataclass(frozen=True, slots=True)
class ControllerOutput:
    goal_id: str | None = None
    command: JointCommand | None = None
    feedback: FollowJointTrajectoryFeedback | None = None
    result: FollowJointTrajectoryResult | None = None
    goal_status: GoalStatus | None = None


@dataclass(slots=True)
class _ActualState:
    positions: tuple[float, ...]
    velocities: tuple[float, ...]
    received_ns: int


@dataclass(slots=True)
class _ActiveGoal:
    goal_id: str
    goal: FollowJointTrajectoryGoal
    trajectory: PreparedTrajectory
    started_ns: int
    path_position_tolerance: tuple[float, ...]
    goal_position_tolerance: tuple[float, ...]
    goal_velocity_tolerance: tuple[float, ...]
    goal_time_tolerance_ns: int
    sequence: int = 0


class JointTrajectoryController:
    """Deterministic position-mode FollowJointTrajectory controller core."""

    def __init__(self, config: ControllerConfig) -> None:
        self.config = config
        self.state: ControllerState = "IDLE"
        self._actual: _ActualState | None = None
        self._active: _ActiveGoal | None = None
        # Rolling (timestamp, measured positions) history used for stall detection.
        self._history: deque[tuple[int, tuple[float, ...]]] = deque()
        self._hold_positions: tuple[float, ...] | None = None
        self._result_cache: OrderedDict[str, ControllerOutput] = OrderedDict()
        self._last_now_ns: int | None = None

    @property
    def active_goal_id(self) -> str | None:
        return self._active.goal_id if self._active else None

    def update_joint_state(self, state: JointState, now_ns: int) -> None:
        self._validate_now(now_ns)
        if not state.position:
            raise ValueError("JointState.position must be complete")
        position_by_name = dict(zip(state.name, state.position, strict=True))
        missing = [name for name in self.config.joint_names if name not in position_by_name]
        if missing:
            raise ValueError(f"JointState is missing configured joints: {missing}")
        positions = tuple(float(position_by_name[name]) for name in self.config.joint_names)
        if any(not math.isfinite(value) for value in positions):
            raise ValueError("JointState positions must be finite")

        if state.velocity:
            velocity_by_name = dict(zip(state.name, state.velocity, strict=True))
            velocities = tuple(float(velocity_by_name[name]) for name in self.config.joint_names)
            if any(not math.isfinite(value) for value in velocities):
                raise ValueError("JointState velocities must be finite")
        elif self._actual is not None and now_ns > self._actual.received_ns:
            dt_s = (now_ns - self._actual.received_ns) / 1_000_000_000.0
            velocities = tuple(
                (position - previous) / dt_s
                for position, previous in zip(positions, self._actual.positions, strict=True)
            )
        else:
            velocities = (0.0,) * len(positions)

        self._actual = _ActualState(
            positions=positions,
            velocities=velocities,
            received_ns=now_ns,
        )

    def accept_goal(
        self,
        goal_id: str,
        goal: FollowJointTrajectoryGoal,
        now_ns: int,
    ) -> ControllerOutput | None:
        self._validate_goal_id(goal_id)
        self._validate_now(now_ns)

        cached = self._cached_result(goal_id)
        if cached is not None:
            return cached
        if self._active is not None:
            if self._active.goal_id == goal_id:
                if goal != self._active.goal:
                    raise ValueError("goal_id is already active with a different payload")
                return None
            return self._reject(goal_id, "BUSY", "controller already has an active goal")
        if not self._feedback_is_fresh(now_ns):
            return self._reject(
                goal_id,
                "NO_FRESH_ROBOT_STATE",
                "fresh JointState feedback is required before accepting a goal",
            )
        if tuple(goal.trajectory.joint_names) != self.config.joint_names:
            return self._reject(
                goal_id,
                "INVALID_JOINTS",
                "trajectory joint_names must exactly match the configured joint order",
            )

        assert self._actual is not None
        try:
            prepared = prepare_trajectory(
                goal.trajectory,
                self._actual.positions,
                self._actual.velocities,
            )
            path_tolerance, goal_position_tolerance, goal_velocity_tolerance = (
                self._resolve_tolerances(goal)
            )
        except ValueError as error:
            return self._reject(goal_id, "INVALID_GOAL", str(error))

        first = goal.trajectory.points[0]
        if first.time_from_start_ns == 0 and any(
            abs(target - actual) > self.config.start_position_tolerance
            for target, actual in zip(first.positions, self._actual.positions, strict=True)
        ):
            return self._reject(
                goal_id,
                "START_STATE_MISMATCH",
                "first trajectory point does not match the current state",
            )

        goal_time_tolerance_ns = (
            goal.goal_time_tolerance_ns
            if goal.goal_time_tolerance_ns is not None
            else self.config.goal_time_tolerance_ns
        )
        self._active = _ActiveGoal(
            goal_id=goal_id,
            goal=goal,
            trajectory=prepared,
            started_ns=now_ns,
            path_position_tolerance=path_tolerance,
            goal_position_tolerance=goal_position_tolerance,
            goal_velocity_tolerance=goal_velocity_tolerance,
            goal_time_tolerance_ns=goal_time_tolerance_ns,
        )
        self.state = "ACTIVE"
        self._hold_positions = None
        self._history.clear()
        return None

    def reject_invalid_goal(
        self,
        goal_id: str,
        message: str,
        now_ns: int,
    ) -> ControllerOutput | None:
        """Register a transport-level invalid goal without disturbing the same active ID."""

        self._validate_goal_id(goal_id)
        self._validate_now(now_ns)
        cached = self._cached_result(goal_id)
        if cached is not None:
            return cached
        if self._active is not None and self._active.goal_id == goal_id:
            return None
        return self._reject(goal_id, "INVALID_GOAL", message)

    def cancel_goal(self, goal_id: str, now_ns: int) -> ControllerOutput | None:
        self._validate_goal_id(goal_id)
        self._validate_now(now_ns)
        cached = self._cached_result(goal_id)
        if cached is not None:
            return cached
        if self._active is None or self._active.goal_id != goal_id:
            return None
        hold = self._actual.positions if self._actual is not None else None
        return self._finish_active(
            error_code="CANCELED",
            message="goal canceled by client",
            goal_status="canceled",
            now_ns=now_ns,
            desired=None,
            hold_positions=hold,
        )

    def tick(self, now_ns: int) -> ControllerOutput | None:
        self._validate_now(now_ns)
        if self._active is None:
            if self._hold_positions is None:
                return None
            return ControllerOutput(command=self._position_command(self._hold_positions))

        active = self._active
        if not self._feedback_is_fresh(now_ns):
            hold = self._actual.positions if self._actual is not None else None
            return self._finish_active(
                error_code="FEEDBACK_STALE",
                message="JointState feedback became stale during execution",
                goal_status="aborted",
                now_ns=now_ns,
                desired=None,
                hold_positions=hold,
            )

        assert self._actual is not None
        elapsed_ns = max(0, now_ns - active.started_ns)
        sample = sample_trajectory(active.trajectory, elapsed_ns)
        command = self._position_command(sample.positions)
        feedback = self._feedback(active, sample, elapsed_ns)
        position_error = tuple(
            desired - actual
            for desired, actual in zip(sample.positions, self._actual.positions, strict=True)
        )

        if elapsed_ns < active.trajectory.duration_ns:
            # Plant capability is deliberately NOT judged per tick: a small tracking
            # lag is normal and depends on velocity/servo tuning. Only two coarse
            # conditions abort the motion: a hard deviation, or a stalled plant.
            self._history.append((now_ns, tuple(self._actual.positions)))
            window_start_ns = now_ns - self.config.stuck_window_ns
            while self._history and self._history[0][0] < window_start_ns:
                self._history.popleft()

            worst_index = max(
                range(len(position_error)),
                key=lambda index: abs(position_error[index]),
            )
            worst_error = abs(position_error[worst_index])
            if worst_error > self.config.path_abort_tolerance:
                joint = self.config.joint_names[worst_index]
                return self._finish_active(
                    error_code="PATH_TOLERANCE_VIOLATED",
                    message=(
                        f"deviation {worst_error:.3f} rad on {joint} exceeds abort tolerance "
                        f"{self.config.path_abort_tolerance:.3f} rad "
                        f"(planned {sample.positions[worst_index]:.3f}, "
                        f"actual {self._actual.positions[worst_index]:.3f})"
                    ),
                    goal_status="aborted",
                    now_ns=now_ns,
                    desired=sample,
                    hold_positions=self._actual.positions,
                )

            if self._stalled(active, sample, now_ns=now_ns, elapsed_ns=elapsed_ns):
                return self._finish_active(
                    error_code="PATH_TOLERANCE_VIOLATED",
                    message=(
                        "no measurable progress for "
                        f"{self.config.stuck_window_ns / 1_000_000_000:.2f} s while the reference "
                        "kept moving; the joint is blocked or cannot follow"
                    ),
                    goal_status="aborted",
                    now_ns=now_ns,
                    desired=sample,
                    hold_positions=self._actual.positions,
                )
            active.sequence += 1
            return ControllerOutput(
                goal_id=active.goal_id,
                command=command,
                feedback=feedback,
            )

        self.state = "SETTLING"
        goal_reached = self._goal_reached(active, sample)
        if goal_reached:
            return self._finish_active(
                error_code="SUCCESS",
                message="trajectory goal reached",
                goal_status="succeeded",
                now_ns=now_ns,
                desired=sample,
                hold_positions=sample.positions,
                feedback=feedback,
            )
        if elapsed_ns > active.trajectory.duration_ns + active.goal_time_tolerance_ns:
            return self._finish_active(
                error_code="GOAL_TOLERANCE_VIOLATED",
                message="trajectory goal tolerance was not reached before the deadline",
                goal_status="aborted",
                now_ns=now_ns,
                desired=sample,
                hold_positions=self._actual.positions,
                feedback=feedback,
            )

        active.sequence += 1
        return ControllerOutput(
            goal_id=active.goal_id,
            command=command,
            feedback=feedback,
        )

    def _stalled(
        self,
        active: _ActiveGoal,
        sample: TrajectorySample,
        *,
        now_ns: int,
        elapsed_ns: int,
    ) -> bool:
        """True when the reference keeps moving but the plant does not move at all."""
        if len(self._history) < 2 or self._actual is None:
            return False
        oldest_ns, oldest_positions = self._history[0]
        window_elapsed_ns = max(0, elapsed_ns - (now_ns - oldest_ns))
        reference_old = sample_trajectory(active.trajectory, window_elapsed_ns)
        reference_moved = max(
            abs(current - previous)
            for current, previous in zip(
                sample.positions, reference_old.positions, strict=True
            )
        )
        actual_moved = max(
            abs(current - previous)
            for current, previous in zip(
                self._actual.positions, oldest_positions, strict=True
            )
        )
        return (
            reference_moved >= self.config.stuck_reference_epsilon
            and actual_moved < self.config.stuck_actual_epsilon
        )

    def _goal_reached(self, active: _ActiveGoal, sample: TrajectorySample) -> bool:
        assert self._actual is not None
        position_ok = all(
            abs(desired - actual) <= tolerance
            for desired, actual, tolerance in zip(
                sample.positions,
                self._actual.positions,
                active.goal_position_tolerance,
                strict=True,
            )
        )
        velocity_ok = all(
            abs(desired - actual) <= tolerance
            for desired, actual, tolerance in zip(
                sample.velocities,
                self._actual.velocities,
                active.goal_velocity_tolerance,
                strict=True,
            )
        )
        return position_ok and velocity_ok

    def _feedback(
        self,
        active: _ActiveGoal,
        sample: TrajectorySample,
        elapsed_ns: int,
    ) -> FollowJointTrajectoryFeedback:
        assert self._actual is not None
        desired = JointTrajectoryPoint(
            positions=list(sample.positions),
            velocities=list(sample.velocities),
            accelerations=[],
            effort=[],
            time_from_start_ns=min(elapsed_ns, active.trajectory.duration_ns),
        )
        actual = JointTrajectoryPoint(
            positions=list(self._actual.positions),
            velocities=list(self._actual.velocities),
            accelerations=[],
            effort=[],
            time_from_start_ns=elapsed_ns,
        )
        error = JointTrajectoryPoint(
            positions=[
                desired_value - actual_value
                for desired_value, actual_value in zip(
                    sample.positions,
                    self._actual.positions,
                    strict=True,
                )
            ],
            velocities=[
                desired_value - actual_value
                for desired_value, actual_value in zip(
                    sample.velocities,
                    self._actual.velocities,
                    strict=True,
                )
            ],
            accelerations=[],
            effort=[],
            time_from_start_ns=elapsed_ns,
        )
        return FollowJointTrajectoryFeedback(
            sequence=active.sequence,
            point_index=sample.point_index,
            elapsed_ns=elapsed_ns,
            duration_ns=active.trajectory.duration_ns,
            desired=desired,
            actual=actual,
            error=error,
        )

    def _resolve_tolerances(
        self,
        goal: FollowJointTrajectoryGoal,
    ) -> tuple[tuple[float, ...], tuple[float, ...], tuple[float, ...]]:
        path = [self.config.path_position_tolerance] * len(self.config.joint_names)
        goal_position = [self.config.goal_position_tolerance] * len(self.config.joint_names)
        goal_velocity = [self.config.goal_velocity_tolerance] * len(self.config.joint_names)
        index_by_name = {name: index for index, name in enumerate(self.config.joint_names)}

        for tolerance in goal.path_tolerance:
            self._validate_path_tolerance(tolerance)
            if tolerance.position is not None:
                path[index_by_name[tolerance.joint_name]] = tolerance.position
        for tolerance in goal.goal_tolerance:
            self._validate_goal_tolerance(tolerance)
            index = index_by_name[tolerance.joint_name]
            if tolerance.position is not None:
                goal_position[index] = tolerance.position
            if tolerance.velocity is not None:
                goal_velocity[index] = tolerance.velocity
        return tuple(path), tuple(goal_position), tuple(goal_velocity)

    @staticmethod
    def _validate_path_tolerance(tolerance: JointTolerance) -> None:
        if tolerance.velocity is not None or tolerance.acceleration is not None:
            raise ValueError("path velocity and acceleration tolerances are not supported")

    @staticmethod
    def _validate_goal_tolerance(tolerance: JointTolerance) -> None:
        if tolerance.acceleration is not None:
            raise ValueError("goal acceleration tolerance is not supported")

    def _finish_active(
        self,
        *,
        error_code: str,
        message: str,
        goal_status: GoalStatus,
        now_ns: int,
        desired: TrajectorySample | None,
        hold_positions: tuple[float, ...] | None,
        feedback: FollowJointTrajectoryFeedback | None = None,
    ) -> ControllerOutput:
        assert self._active is not None
        active = self._active
        joint_names: list[str] = []
        position_error: list[float] = []
        velocity_error: list[float] = []
        if self._actual is not None and desired is not None:
            joint_names = list(self.config.joint_names)
            position_error = [
                target - actual
                for target, actual in zip(
                    desired.positions,
                    self._actual.positions,
                    strict=True,
                )
            ]
            velocity_error = [
                target - actual
                for target, actual in zip(
                    desired.velocities,
                    self._actual.velocities,
                    strict=True,
                )
            ]
        result = FollowJointTrajectoryResult(
            error_code=error_code,  # type: ignore[arg-type]
            message=message,
            elapsed_ns=max(0, now_ns - active.started_ns),
            joint_names=joint_names,
            final_position_error=position_error,
            final_velocity_error=velocity_error,
        )
        self._active = None
        self._hold_positions = hold_positions
        self.state = "HOLDING" if hold_positions is not None else "IDLE"
        output = ControllerOutput(
            goal_id=active.goal_id,
            command=self._position_command(hold_positions) if hold_positions is not None else None,
            feedback=feedback,
            result=result,
            goal_status=goal_status,
        )
        self._cache_result(
            ControllerOutput(
                goal_id=active.goal_id,
                result=result,
                goal_status=goal_status,
            )
        )
        return output

    def _reject(self, goal_id: str, error_code: str, message: str) -> ControllerOutput:
        result = FollowJointTrajectoryResult(
            error_code=error_code,  # type: ignore[arg-type]
            message=message,
            elapsed_ns=0,
            joint_names=[],
            final_position_error=[],
            final_velocity_error=[],
        )
        output = ControllerOutput(
            goal_id=goal_id,
            result=result,
            goal_status="aborted",
        )
        self._cache_result(output)
        return output

    def _cached_result(self, goal_id: str) -> ControllerOutput | None:
        output = self._result_cache.get(goal_id)
        if output is not None:
            self._result_cache.move_to_end(goal_id)
        return output

    def _cache_result(self, output: ControllerOutput) -> None:
        assert output.goal_id is not None
        self._result_cache[output.goal_id] = output
        self._result_cache.move_to_end(output.goal_id)
        while len(self._result_cache) > self.config.result_cache_size:
            self._result_cache.popitem(last=False)

    def _position_command(self, positions: tuple[float, ...]) -> JointCommand:
        return JointCommand(
            name=list(self.config.joint_names),
            mode="position",
            position=list(positions),
            velocity=[],
            effort=[],
            kp=[],
            kd=[],
        )

    def _feedback_is_fresh(self, now_ns: int) -> bool:
        return (
            self._actual is not None
            and now_ns >= self._actual.received_ns
            and now_ns - self._actual.received_ns <= self.config.feedback_timeout_ns
        )

    def _validate_now(self, now_ns: int) -> None:
        if now_ns < 0:
            raise ValueError("now_ns must be non-negative")
        if self._last_now_ns is not None and now_ns < self._last_now_ns:
            raise ValueError("now_ns must be monotonic")
        self._last_now_ns = now_ns

    @staticmethod
    def _validate_goal_id(goal_id: str) -> None:
        if not goal_id:
            raise ValueError("goal_id must be non-empty")
