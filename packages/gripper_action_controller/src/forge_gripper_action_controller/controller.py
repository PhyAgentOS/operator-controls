from __future__ import annotations

import math
from collections import OrderedDict
from dataclasses import dataclass
from typing import Literal

from forge_msgs import (
    GripperCommandErrorCode,
    GripperCommandFeedback,
    GripperCommandGoal,
    GripperCommandResult,
    JointCommand,
    JointState,
)

from .config import ControllerConfig

GoalStatus = Literal["succeeded", "aborted", "canceled"]
ControllerState = Literal["IDLE", "ACTIVE", "HOLDING"]
TerminalCondition = Literal[
    "cancel",
    "feedback_stale",
    "goal_reached",
    "goal_timeout",
    "stall",
]

_TERMINAL_TIE_PRIORITY: dict[TerminalCondition, int] = {
    "cancel": 0,
    "feedback_stale": 1,
    "goal_reached": 2,
    "goal_timeout": 3,
    "stall": 4,
}


@dataclass(frozen=True, slots=True)
class ControllerOutput:
    goal_id: str | None = None
    command: JointCommand | None = None
    feedback: GripperCommandFeedback | None = None
    result: GripperCommandResult | None = None
    goal_status: GoalStatus | None = None


@dataclass(slots=True)
class _ActualState:
    position: float
    velocity: float | None
    effort: float | None
    received_ns: int


@dataclass(slots=True)
class _ActiveGoal:
    goal_id: str
    goal: GripperCommandGoal
    started_ns: int
    last_movement_ns: int | None
    reached_since_ns: int | None
    feedback_stale_ns: int | None = None
    stall_ns: int | None = None


@dataclass(frozen=True, slots=True)
class _TerminalEvent:
    condition: TerminalCondition
    occurred_ns: int


class GripperActionController:
    """Deterministic position-only GripperCommand controller core."""

    def __init__(self, config: ControllerConfig) -> None:
        self.config = config
        self.state: ControllerState = "IDLE"
        self._actual: _ActualState | None = None
        self._active: _ActiveGoal | None = None
        self._hold_position: float | None = None
        self._terminal_history: OrderedDict[str, None] = OrderedDict()
        self._last_now_ns: int | None = None

    @property
    def active_goal_id(self) -> str | None:
        return self._active.goal_id if self._active else None

    def update_joint_state(self, state: JointState, now_ns: int) -> None:
        self._validate_now(now_ns)
        if not state.position:
            raise ValueError("JointState.position must be complete")

        position_by_name = dict(zip(state.name, state.position, strict=True))
        if self.config.joint_name not in position_by_name:
            raise ValueError(f"JointState is missing configured joint: {self.config.joint_name}")
        position = float(position_by_name[self.config.joint_name])
        if not math.isfinite(position):
            raise ValueError("JointState position must be finite")

        velocity: float | None
        if state.velocity:
            velocity_by_name = dict(zip(state.name, state.velocity, strict=True))
            velocity = float(velocity_by_name[self.config.joint_name])
            if not math.isfinite(velocity):
                raise ValueError("JointState velocity must be finite")
        elif self._actual is not None and now_ns > self._actual.received_ns:
            dt_s = (now_ns - self._actual.received_ns) / 1_000_000_000.0
            velocity = (position - self._actual.position) / dt_s
        else:
            velocity = None

        effort: float | None = None
        if state.effort:
            effort_by_name = dict(zip(state.name, state.effort, strict=True))
            effort = float(effort_by_name[self.config.joint_name])
            if not math.isfinite(effort):
                raise ValueError("JointState effort must be finite")

        self._latch_elapsed_conditions(now_ns)
        self._actual = _ActualState(
            position=position,
            velocity=velocity,
            effort=effort,
            received_ns=now_ns,
        )
        self._observe_goal_position(now_ns, position)
        self._observe_movement(now_ns, velocity)

    def accept_goal(
        self,
        goal_id: str,
        goal: GripperCommandGoal,
        now_ns: int,
    ) -> ControllerOutput | None:
        self._validate_goal_id(goal_id)
        self._validate_now(now_ns)

        if self._is_terminal(goal_id):
            return None
        if self._active is not None:
            if self._active.goal_id == goal_id:
                if goal != self._active.goal:
                    raise ValueError("goal_id is already active with a different payload")
                return None
            return self._reject(goal_id, "BUSY", "controller already has an active goal")
        if goal.max_velocity is not None:
            return self._reject(
                goal_id,
                "UNSUPPORTED_VELOCITY",
                "position-only controller does not support max_velocity",
            )
        if goal.max_effort is not None:
            return self._reject(
                goal_id,
                "UNSUPPORTED_EFFORT",
                "position-only controller does not support max_effort",
            )
        if not self.config.min_position <= goal.position <= self.config.max_position:
            return self._reject(
                goal_id,
                "POSITION_LIMIT_VIOLATION",
                "goal position is outside the configured limits",
            )
        if not self._feedback_is_fresh(now_ns):
            return self._reject(
                goal_id,
                "NO_FRESH_ROBOT_STATE",
                "fresh JointState feedback is required before accepting a goal",
            )

        assert self._actual is not None
        self._active = _ActiveGoal(
            goal_id=goal_id,
            goal=goal,
            started_ns=now_ns,
            last_movement_ns=now_ns if self._actual.velocity is not None else None,
            reached_since_ns=(
                now_ns
                if abs(goal.position - self._actual.position) <= self.config.goal_tolerance
                else None
            ),
        )
        self._hold_position = None
        self.state = "ACTIVE"
        return None

    def reject_invalid_goal(
        self,
        goal_id: str,
        message: str,
        now_ns: int,
    ) -> ControllerOutput | None:
        """Register an undecodable goal without disturbing the same active ID."""

        self._validate_goal_id(goal_id)
        self._validate_now(now_ns)
        if self._is_terminal(goal_id):
            return None
        if self._active is not None and self._active.goal_id == goal_id:
            return None
        return self._reject(goal_id, "INVALID_GOAL", message)

    def cancel_goal(self, goal_id: str, now_ns: int) -> ControllerOutput | None:
        self._validate_goal_id(goal_id)
        self._validate_now(now_ns)
        if self._is_terminal(goal_id):
            return None
        if self._active is None or self._active.goal_id != goal_id:
            return None

        terminal = self._select_terminal_event(now_ns, cancel_requested=True)
        assert terminal is not None
        return self._finish_terminal_event(terminal, now_ns)

    def tick(self, now_ns: int) -> ControllerOutput | None:
        self._validate_now(now_ns)
        if self._active is None:
            if self._hold_position is None:
                return None
            return ControllerOutput(command=self._position_command(self._hold_position))

        terminal = self._select_terminal_event(now_ns)
        if terminal is not None:
            return self._finish_terminal_event(terminal, now_ns)

        active = self._active
        assert active is not None
        assert self._actual is not None
        elapsed_ns = max(0, now_ns - active.started_ns)
        return ControllerOutput(
            goal_id=active.goal_id,
            command=self._position_command(active.goal.position),
            feedback=self._feedback(elapsed_ns, stalled=False, reached_goal=False),
        )

    def _observe_goal_position(self, now_ns: int, position: float) -> None:
        if self._active is None:
            return
        within_tolerance = abs(self._active.goal.position - position) <= self.config.goal_tolerance
        if within_tolerance:
            if self._active.reached_since_ns is None:
                self._active.reached_since_ns = now_ns
        else:
            if self._active.reached_since_ns is not None:
                self._active.last_movement_ns = now_ns
            self._active.reached_since_ns = None

    def _observe_movement(self, now_ns: int, velocity: float | None) -> None:
        if self._active is None or velocity is None:
            return
        if (
            self._active.last_movement_ns is None
            or abs(velocity) > self.config.stall_velocity_threshold
        ):
            self._active.last_movement_ns = now_ns

    def _latch_elapsed_conditions(self, now_ns: int) -> None:
        active = self._active
        if active is None:
            return

        if active.feedback_stale_ns is None and self._actual is not None:
            stale_ns = self._actual.received_ns + self.config.feedback_timeout_ns + 1
            if now_ns >= stale_ns:
                active.feedback_stale_ns = stale_ns

        if (
            active.stall_ns is None
            and active.reached_since_ns is None
            and active.last_movement_ns is not None
        ):
            stall_ns = active.last_movement_ns + self.config.stall_timeout_ns
            if now_ns >= stall_ns:
                active.stall_ns = stall_ns

    def _select_terminal_event(
        self,
        now_ns: int,
        *,
        cancel_requested: bool = False,
    ) -> _TerminalEvent | None:
        active = self._active
        assert active is not None
        self._latch_elapsed_conditions(now_ns)

        candidates: list[_TerminalEvent] = []
        if cancel_requested:
            candidates.append(_TerminalEvent("cancel", now_ns))
        if active.feedback_stale_ns is not None:
            candidates.append(_TerminalEvent("feedback_stale", active.feedback_stale_ns))
        if active.reached_since_ns is not None:
            candidates.append(_TerminalEvent("goal_reached", active.reached_since_ns))

        goal_timeout_ns = active.started_ns + self.config.goal_timeout_ns
        if now_ns >= goal_timeout_ns:
            candidates.append(_TerminalEvent("goal_timeout", goal_timeout_ns))
        if active.stall_ns is not None:
            candidates.append(_TerminalEvent("stall", active.stall_ns))

        if not candidates:
            return None
        return min(
            candidates,
            key=lambda event: (
                event.occurred_ns,
                _TERMINAL_TIE_PRIORITY[event.condition],
            ),
        )

    def _finish_terminal_event(
        self,
        terminal: _TerminalEvent,
        now_ns: int,
    ) -> ControllerOutput:
        active = self._active
        actual = self._actual
        assert active is not None
        assert actual is not None
        elapsed_ns = max(0, now_ns - active.started_ns)

        if terminal.condition == "cancel":
            return self._finish_active(
                error_code="CANCELED",
                message="goal canceled by client",
                goal_status="canceled",
                now_ns=now_ns,
                hold_position=actual.position,
            )
        if terminal.condition == "feedback_stale":
            return self._finish_active(
                error_code="FEEDBACK_STALE",
                message="JointState feedback became stale during execution",
                goal_status="aborted",
                now_ns=now_ns,
                hold_position=actual.position,
            )
        if terminal.condition == "goal_reached":
            feedback = self._feedback(elapsed_ns, stalled=False, reached_goal=True)
            return self._finish_active(
                error_code="SUCCESS",
                message="gripper goal reached",
                goal_status="succeeded",
                now_ns=now_ns,
                hold_position=active.goal.position,
                feedback=feedback,
            )
        if terminal.condition == "goal_timeout":
            feedback = self._feedback(elapsed_ns, stalled=False, reached_goal=False)
            return self._finish_active(
                error_code="EXECUTION_TIMED_OUT",
                message="gripper goal was not reached before the deadline",
                goal_status="aborted",
                now_ns=now_ns,
                hold_position=actual.position,
                feedback=feedback,
            )

        assert terminal.condition == "stall"
        feedback = self._feedback(elapsed_ns, stalled=True, reached_goal=False)
        if self.config.allow_stalling:
            return self._finish_active(
                error_code="SUCCESS",
                message="gripper stalled and stalling is allowed",
                goal_status="succeeded",
                now_ns=now_ns,
                hold_position=active.goal.position,
                feedback=feedback,
            )
        return self._finish_active(
            error_code="STALLED",
            message="gripper stalled before reaching the goal",
            goal_status="aborted",
            now_ns=now_ns,
            hold_position=actual.position,
            feedback=feedback,
        )

    def _feedback(
        self,
        elapsed_ns: int,
        *,
        stalled: bool,
        reached_goal: bool,
    ) -> GripperCommandFeedback:
        assert self._actual is not None
        return GripperCommandFeedback(
            elapsed_ns=elapsed_ns,
            position=self._actual.position,
            velocity=self._actual.velocity,
            effort=self._actual.effort,
            stalled=stalled,
            reached_goal=reached_goal,
        )

    def _finish_active(
        self,
        *,
        error_code: GripperCommandErrorCode,
        message: str,
        goal_status: GoalStatus,
        now_ns: int,
        hold_position: float | None,
        feedback: GripperCommandFeedback | None = None,
    ) -> ControllerOutput:
        assert self._active is not None
        active = self._active
        actual = self._actual
        result = GripperCommandResult(
            error_code=error_code,
            message=message,
            elapsed_ns=max(0, now_ns - active.started_ns),
            position=actual.position if actual is not None else None,
            velocity=actual.velocity if actual is not None else None,
            effort=actual.effort if actual is not None else None,
            stalled=feedback.stalled if feedback is not None else False,
            reached_goal=feedback.reached_goal if feedback is not None else False,
        )

        self._active = None
        self._hold_position = hold_position
        self._remember_terminal(active.goal_id)
        self.state = "HOLDING" if hold_position is not None else "IDLE"
        return ControllerOutput(
            goal_id=active.goal_id,
            command=self._position_command(hold_position) if hold_position is not None else None,
            feedback=feedback,
            result=result,
            goal_status=goal_status,
        )

    def _reject(
        self,
        goal_id: str,
        error_code: GripperCommandErrorCode,
        message: str,
    ) -> ControllerOutput:
        actual = self._actual
        result = GripperCommandResult(
            error_code=error_code,
            message=message,
            elapsed_ns=0,
            position=actual.position if actual is not None else None,
            velocity=actual.velocity if actual is not None else None,
            effort=actual.effort if actual is not None else None,
            stalled=False,
            reached_goal=False,
        )
        self._remember_terminal(goal_id)
        return ControllerOutput(
            goal_id=goal_id,
            result=result,
            goal_status="aborted",
        )

    def _is_terminal(self, goal_id: str) -> bool:
        if goal_id not in self._terminal_history:
            return False
        self._terminal_history.move_to_end(goal_id)
        return True

    def _remember_terminal(self, goal_id: str) -> None:
        self._terminal_history[goal_id] = None
        self._terminal_history.move_to_end(goal_id)
        while len(self._terminal_history) > self.config.terminal_history_size:
            self._terminal_history.popitem(last=False)

    def _position_command(self, position: float) -> JointCommand:
        return JointCommand(
            name=[self.config.joint_name],
            mode="position",
            position=[position],
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
