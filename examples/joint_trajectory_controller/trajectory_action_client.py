#!/usr/bin/env python3
"""End-to-end Dora Action client for the joint trajectory controller smoke test."""

from __future__ import annotations

import argparse
import json
import math
import time
from collections import defaultdict
from pathlib import Path
from typing import Any

import pyarrow as pa
from dora import Node
from forge_msgs import (
    FollowJointTrajectoryFeedback,
    FollowJointTrajectoryGoal,
    FollowJointTrajectoryResult,
    JointCommand,
    JointState,
    JointTrajectory,
    JointTrajectoryPoint,
)

JOINT_NAMES = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")
NORMAL_GAIN = 0.45


class SmokeFailure(RuntimeError):
    pass


def _parameters(event: dict[str, Any]) -> dict[str, Any]:
    metadata = event.get("metadata")
    if not isinstance(metadata, dict):
        raise SmokeFailure("Action event is missing metadata")
    parameters = metadata.get("parameters")
    return parameters if isinstance(parameters, dict) else metadata


def _metadata(goal_id: str) -> dict[str, Any]:
    return {"goal_id": goal_id}


def _plant_control(response_gain: float, publish_state: bool) -> pa.RecordBatch:
    return pa.RecordBatch.from_pydict(
        {
            "response_gain": pa.array([response_gain], type=pa.float64()),
            "publish_state": pa.array([publish_state], type=pa.bool_()),
        }
    )


def _close(left: list[float], right: list[float], tolerance: float) -> bool:
    return len(left) == len(right) and all(
        abs(a - b) <= tolerance for a, b in zip(left, right, strict=True)
    )


class SmokeScenario:
    def __init__(self, node: Node, result_file: Path, timeout_s: float) -> None:
        self.node = node
        self.result_file = result_file
        self.deadline = time.monotonic() + timeout_s
        self.phase = "waiting_for_state"
        self.latest_positions: list[float] | None = None
        self.latest_command: list[float] | None = None
        self.feedback: dict[str, list[FollowJointTrajectoryFeedback]] = defaultdict(list)
        self.results: dict[str, list[FollowJointTrajectoryResult]] = defaultdict(list)
        self.statuses: dict[str, list[str]] = defaultdict(list)
        self.goals: dict[str, FollowJointTrajectoryGoal] = {}
        self.cancel_hold: list[float] | None = None
        self.cancel_stable_start: list[float] | None = None
        self.cancel_hold_states = 0
        self.busy_a_done = False
        self.busy_b_done = False
        self.busy_b_sent = False
        self.duplicate_sent = False
        self.duplicate_elapsed_ns = 0
        self.completed = False

    def handle(self, event: dict[str, Any]) -> None:
        if time.monotonic() > self.deadline:
            raise SmokeFailure(f"smoke test timed out in phase {self.phase}")
        if event.get("type") == "ERROR":
            raise SmokeFailure(f"Dora node error: {event.get('error', 'unknown')}")
        if event.get("type") != "INPUT":
            return

        input_id = event.get("id")
        if input_id == "tick":
            return
        if input_id == "joint_state":
            self._handle_joint_state(JointState.from_arrow(event["value"]))
        elif input_id == "joint_command":
            self._handle_joint_command(JointCommand.from_arrow(event["value"]))
        elif input_id == "trajectory_feedback":
            self._handle_feedback(event)
        elif input_id == "trajectory_result":
            self._handle_result(event)

    def _handle_joint_state(self, state: JointState) -> None:
        if tuple(state.name) != JOINT_NAMES or len(state.position) != len(JOINT_NAMES):
            raise SmokeFailure("Fake plant emitted an invalid joint order or incomplete state")
        if any(not math.isfinite(value) for value in state.position):
            raise SmokeFailure("Fake plant emitted a non-finite position")
        self.latest_positions = list(state.position)

        if self.phase == "waiting_for_state":
            self._start_success()
        elif self.phase == "cancel_hold":
            assert self.cancel_stable_start is not None
            self.cancel_hold_states += 1
            if self.cancel_hold_states >= 6:
                if not _close(self.latest_positions, self.cancel_stable_start, 0.02):
                    raise SmokeFailure(
                        "cancel did not hold the fake plant near its actual position"
                    )
                self._start_stale()
        elif self.phase == "waiting_fresh_after_stale":
            self._start_slow()
        elif self.phase == "waiting_fresh_after_slow":
            self._start_busy()

    def _handle_joint_command(self, command: JointCommand) -> None:
        if tuple(command.name) != JOINT_NAMES:
            raise SmokeFailure("Controller emitted JointCommand with an invalid joint order")
        if command.mode != "position" or len(command.position) != len(JOINT_NAMES):
            raise SmokeFailure("Controller emitted an incomplete non-position JointCommand")
        self.latest_command = list(command.position)
        if self.phase == "waiting_cancel_hold_command":
            assert self.cancel_hold is not None
            if _close(self.latest_command, self.cancel_hold, 1e-12):
                self.cancel_stable_start = list(self.latest_positions or self.cancel_hold)
                self.cancel_hold_states = 0
                self.phase = "cancel_hold"

    def _handle_feedback(self, event: dict[str, Any]) -> None:
        parameters = _parameters(event)
        goal_id = str(parameters.get("goal_id", ""))
        if not goal_id:
            raise SmokeFailure("feedback metadata has an empty goal_id")
        feedback = FollowJointTrajectoryFeedback.from_arrow(event["value"])
        self.feedback[goal_id].append(feedback)

        if (
            goal_id == "cancel"
            and self.phase == "cancel_active"
            and feedback.elapsed_ns >= 200_000_000
        ):
            self._send_cancel("cancel")
            self.phase = "cancel_requested"
        elif goal_id == "stale" and self.phase == "stale_active":
            self._send_plant(NORMAL_GAIN, publish_state=False)
            self.phase = "stale_feedback_suppressed"
        elif goal_id == "busy-a" and self.phase == "busy_active" and not self.busy_b_sent:
            self.busy_b_sent = True
            self._send_goal("busy-b", duration_ns=400_000_000, delta=0.1)
        elif goal_id == "duplicate" and self.phase == "duplicate_active":
            if not self.duplicate_sent:
                self.duplicate_sent = True
                self.duplicate_elapsed_ns = feedback.elapsed_ns
                self._send_existing_goal("duplicate")
                self.phase = "duplicate_active_retried"

    def _handle_result(self, event: dict[str, Any]) -> None:
        parameters = _parameters(event)
        goal_id = str(parameters.get("goal_id", ""))
        goal_status = str(parameters.get("goal_status", ""))
        if not goal_id or goal_status not in {"succeeded", "aborted", "canceled"}:
            raise SmokeFailure("result metadata has invalid goal_id or goal_status")
        result = FollowJointTrajectoryResult.from_arrow(event["value"])
        self.results[goal_id].append(result)
        self.statuses[goal_id].append(goal_status)

        if goal_id == "success":
            self._expect_terminal(goal_id, "SUCCESS", "succeeded", count=1)
            self._start_cancel()
        elif goal_id == "cancel":
            self._expect_terminal(goal_id, "CANCELED", "canceled", count=1)
            if self.latest_positions is None:
                raise SmokeFailure("cancel result arrived before any JointState")
            self.cancel_hold = list(self.latest_positions)
            self.phase = "waiting_cancel_hold_command"
        elif goal_id == "stale":
            self._expect_terminal(goal_id, "FEEDBACK_STALE", "aborted", count=1)
            self._send_plant(NORMAL_GAIN, publish_state=True)
            self.phase = "waiting_fresh_after_stale"
        elif goal_id == "slow":
            self._expect_terminal(goal_id, "PATH_TOLERANCE_VIOLATED", "aborted", count=1)
            self._send_plant(NORMAL_GAIN, publish_state=True)
            self.phase = "waiting_fresh_after_slow"
        elif goal_id == "busy-b":
            self._expect_terminal(goal_id, "BUSY", "aborted", count=1)
            self.busy_b_done = True
            self._maybe_finish_busy()
        elif goal_id == "busy-a":
            self._expect_terminal(goal_id, "SUCCESS", "succeeded", count=1)
            self.busy_a_done = True
            self._maybe_finish_busy()
        elif goal_id == "duplicate":
            self._handle_duplicate_result()
        else:
            raise SmokeFailure(f"unexpected terminal result for goal {goal_id!r}")

    def _start_success(self) -> None:
        self._send_plant(NORMAL_GAIN, publish_state=True)
        self._send_goal("success", duration_ns=600_000_000, delta=0.25)
        self.phase = "success_active"

    def _start_cancel(self) -> None:
        self._send_goal("cancel", duration_ns=1_200_000_000, delta=0.5)
        self.phase = "cancel_active"

    def _start_stale(self) -> None:
        self._send_goal("stale", duration_ns=1_000_000_000, delta=0.25)
        self.phase = "stale_active"

    def _start_slow(self) -> None:
        self._send_plant(0.0, publish_state=True)
        self._send_goal("slow", duration_ns=500_000_000, delta=0.8)
        self.phase = "slow_active"

    def _start_busy(self) -> None:
        self._send_plant(NORMAL_GAIN, publish_state=True)
        self._send_goal("busy-a", duration_ns=700_000_000, delta=0.3)
        self.phase = "busy_active"

    def _maybe_finish_busy(self) -> None:
        if self.busy_a_done and self.busy_b_done:
            if not self.feedback["busy-a"]:
                raise SmokeFailure("BUSY test terminated active goal A")
            self._send_goal("duplicate", duration_ns=600_000_000, delta=0.2)
            self.phase = "duplicate_active"

    def _handle_duplicate_result(self) -> None:
        count = len(self.results["duplicate"])
        if count == 1:
            self._expect_terminal("duplicate", "SUCCESS", "succeeded", count=1)
            if not self.duplicate_sent:
                raise SmokeFailure("duplicate goal completed before the active retry was sent")
            if not any(
                item.elapsed_ns > self.duplicate_elapsed_ns for item in self.feedback["duplicate"]
            ):
                raise SmokeFailure("active duplicate appears to have restarted trajectory progress")
            self._send_existing_goal("duplicate")
            self.phase = "duplicate_terminal_retried"
        elif count == 2:
            self._expect_terminal("duplicate", "SUCCESS", "succeeded", count=2)
            if self.results["duplicate"][0] != self.results["duplicate"][1]:
                raise SmokeFailure("terminal duplicate retry did not return the cached result")
            self._finish()
        else:
            raise SmokeFailure("duplicate goal produced more results than requested")

    def _send_goal(self, goal_id: str, *, duration_ns: int, delta: float) -> None:
        if self.latest_positions is None:
            raise SmokeFailure(f"cannot send {goal_id}: no JointState received")
        target = [
            value + (delta if index % 2 == 0 else -0.5 * delta)
            for index, value in enumerate(self.latest_positions)
        ]
        goal = FollowJointTrajectoryGoal(
            trajectory=JointTrajectory(
                joint_names=list(JOINT_NAMES),
                points=[
                    JointTrajectoryPoint(
                        positions=list(self.latest_positions),
                        velocities=[],
                        accelerations=[],
                        effort=[],
                        time_from_start_ns=0,
                    ),
                    JointTrajectoryPoint(
                        positions=target,
                        velocities=[],
                        accelerations=[],
                        effort=[],
                        time_from_start_ns=duration_ns,
                    ),
                ],
            ),
            path_tolerance=[],
            goal_tolerance=[],
            goal_time_tolerance_ns=None,
        )
        self.goals[goal_id] = goal
        self._send_existing_goal(goal_id)

    def _send_existing_goal(self, goal_id: str) -> None:
        self.node.send_output(
            "trajectory_goal",
            self.goals[goal_id].to_arrow(),
            metadata=_metadata(goal_id),
        )

    def _send_cancel(self, goal_id: str) -> None:
        self.node.send_output(
            "trajectory_cancel",
            pa.array([True], type=pa.bool_()),
            metadata=_metadata(goal_id),
        )

    def _send_plant(self, response_gain: float, *, publish_state: bool) -> None:
        self.node.send_output(
            "plant_control",
            _plant_control(response_gain, publish_state),
        )

    def _expect_terminal(
        self,
        goal_id: str,
        error_code: str,
        goal_status: str,
        *,
        count: int,
    ) -> None:
        if len(self.results[goal_id]) != count:
            raise SmokeFailure(f"goal {goal_id} produced an unexpected terminal result count")
        if self.results[goal_id][-1].error_code != error_code:
            actual = self.results[goal_id][-1].error_code
            raise SmokeFailure(f"goal {goal_id} returned {actual}, expected {error_code}")
        if self.statuses[goal_id][-1] != goal_status:
            actual = self.statuses[goal_id][-1]
            raise SmokeFailure(f"goal {goal_id} status was {actual}, expected {goal_status}")

    def _finish(self) -> None:
        expected_counts = {
            "success": 1,
            "cancel": 1,
            "stale": 1,
            "slow": 1,
            "busy-a": 1,
            "busy-b": 1,
            # One execution result plus one intentional cached-result retransmission.
            "duplicate": 2,
        }
        actual_counts = {goal_id: len(items) for goal_id, items in self.results.items()}
        if actual_counts != expected_counts:
            raise SmokeFailure(
                f"terminal result counts differ: actual={actual_counts}, expected={expected_counts}"
            )
        report = {
            "status": "passed",
            "checks": [
                "success",
                "cancel_hold",
                "feedback_stale",
                "path_tolerance",
                "busy_preserves_active_goal",
                "active_duplicate_does_not_restart",
                "terminal_retry_returns_cached_result",
                "dora_metadata_and_arrow_round_trip",
                "one_terminal_result_per_execution",
            ],
            "terminal_result_counts": actual_counts,
        }
        self.result_file.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        self.phase = "complete"
        self.completed = True


def main() -> int:
    parser = argparse.ArgumentParser(description="Joint trajectory Dora integration smoke client")
    parser.add_argument("--result-file", type=Path, default=Path("smoke-result.json"))
    parser.add_argument("--timeout-s", type=float, default=20.0)
    args = parser.parse_args()
    args.result_file.unlink(missing_ok=True)

    node = Node()
    scenario = SmokeScenario(node, args.result_file, args.timeout_s)
    try:
        for event in node:
            if event.get("type") == "STOP":
                break
            scenario.handle(event)
            if scenario.completed:
                print("joint trajectory controller Dora smoke: PASSED", flush=True)
                return 0
    except Exception as error:
        report = {"status": "failed", "phase": scenario.phase, "error": str(error)}
        args.result_file.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(f"joint trajectory controller Dora smoke: FAILED: {error}", flush=True)
        return 1

    if not scenario.completed:
        report = {"status": "failed", "phase": scenario.phase, "error": "dataflow stopped early"}
        args.result_file.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
