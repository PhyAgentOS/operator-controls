from __future__ import annotations

import logging
from collections.abc import Iterator
from typing import Any

import pytest
from forge_gripper_action_controller.config import ControllerConfig
from forge_gripper_action_controller.controller import GripperActionController
from forge_gripper_action_controller.node import (
    GripperActionControllerNode,
    run_dora_controller_node,
)
from forge_msgs import (
    GripperCommandFeedback,
    GripperCommandGoal,
    GripperCommandResult,
    JointCommand,
    JointState,
)


class FakeNode:
    def __init__(self, events: list[dict[str, Any]] | None = None) -> None:
        self.events = events or []
        self.sent: list[tuple[str, Any, dict[str, Any] | None]] = []

    def __iter__(self) -> Iterator[dict[str, Any]]:
        return iter(self.events)

    def send_output(self, output_id: str, value: Any, **kwargs: Any) -> None:
        self.sent.append((output_id, value, kwargs.get("metadata")))


class FailingTerminalResultNode(FakeNode):
    def send_output(self, output_id: str, value: Any, **kwargs: Any) -> None:
        if output_id == "gripper_result":
            raise ValueError("terminal result send failed")
        super().send_output(output_id, value, **kwargs)


def _config() -> ControllerConfig:
    return ControllerConfig(
        joint_name="gripper",
        coordinate_type="prismatic",
        min_position=0.0,
        max_position=1.0,
        feedback_timeout_ns=1000,
        goal_tolerance=0.01,
        stall_velocity_threshold=0.01,
        stall_timeout_ns=500,
        goal_timeout_ns=900,
    )


def _adapter() -> GripperActionControllerNode:
    return GripperActionControllerNode(GripperActionController(_config()))


def _state(position: float, velocity: float | None = 0.0) -> JointState:
    return JointState(
        name=["gripper"],
        position=[position],
        velocity=[] if velocity is None else [velocity],
        effort=[],
    )


def _goal(position: float = 0.8) -> GripperCommandGoal:
    return GripperCommandGoal(position=position)


def _metadata(goal_id: str) -> dict[str, Any]:
    return {"parameters": {"goal_id": goal_id}}


def test_adapter_accepts_flat_metadata_and_routes_sparse_feedback() -> None:
    adapter = _adapter()
    node = FakeNode()
    adapter.handle_event(
        node,
        {"type": "INPUT", "id": "joint_state", "value": _state(0.2).to_arrow()},
        0,
    )
    adapter.handle_event(
        node,
        {
            "type": "INPUT",
            "id": "gripper_goal",
            "value": _goal().to_arrow(),
            "metadata": {"goal_id": "goal"},
        },
        0,
    )
    adapter.handle_event(node, {"type": "INPUT", "id": "tick"}, 10)

    assert [item[0] for item in node.sent] == ["joint_command", "gripper_feedback"]
    command = JointCommand.from_arrow(node.sent[0][1])
    assert command.name == ["gripper"]
    assert command.position == [0.8]
    assert command.velocity == []
    assert command.effort == []
    feedback = GripperCommandFeedback.from_arrow(node.sent[1][1])
    assert feedback.position == 0.2
    assert node.sent[1][2] == {"goal_id": "goal"}


def test_adapter_emits_final_feedback_and_result_metadata() -> None:
    adapter = _adapter()
    node = FakeNode()
    adapter.handle_event(
        node,
        {"type": "INPUT", "id": "joint_state", "value": _state(0.8).to_arrow()},
        0,
    )
    adapter.handle_event(
        node,
        {
            "type": "INPUT",
            "id": "gripper_goal",
            "value": _goal().to_arrow(),
            "metadata": _metadata("goal"),
        },
        0,
    )
    adapter.handle_event(node, {"type": "INPUT", "id": "tick"}, 1)

    assert [item[0] for item in node.sent] == [
        "joint_command",
        "gripper_feedback",
        "gripper_result",
    ]
    result = GripperCommandResult.from_arrow(node.sent[-1][1])
    assert result.error_code == "SUCCESS"
    assert result.reached_goal
    assert node.sent[-1][2] == {"goal_id": "goal", "goal_status": "succeeded"}


def test_adapter_cancel_holds_current_position_and_emits_once() -> None:
    adapter = _adapter()
    node = FakeNode()
    adapter.handle_event(
        node,
        {"type": "INPUT", "id": "joint_state", "value": _state(0.3).to_arrow()},
        0,
    )
    adapter.handle_event(
        node,
        {
            "type": "INPUT",
            "id": "gripper_goal",
            "value": _goal().to_arrow(),
            "metadata": _metadata("goal"),
        },
        0,
    )
    cancel = {
        "type": "INPUT",
        "id": "gripper_cancel",
        "value": b"",
        "metadata": _metadata("goal"),
    }
    adapter.handle_event(node, cancel, 1)
    adapter.handle_event(node, cancel, 2)

    assert [item[0] for item in node.sent] == ["joint_command", "gripper_result"]
    command = JointCommand.from_arrow(node.sent[0][1])
    assert command.position == [0.3]
    result = GripperCommandResult.from_arrow(node.sent[1][1])
    assert result.error_code == "CANCELED"
    assert node.sent[1][2] == {"goal_id": "goal", "goal_status": "canceled"}


def test_adapter_rejects_unsupported_and_malformed_goals_once() -> None:
    adapter = _adapter()
    node = FakeNode()
    adapter.handle_event(
        node,
        {"type": "INPUT", "id": "joint_state", "value": _state(0.0).to_arrow()},
        0,
    )
    unsupported = {
        "type": "INPUT",
        "id": "gripper_goal",
        "value": GripperCommandGoal(position=0.5, max_velocity=0.0).to_arrow(),
        "metadata": _metadata("unsupported"),
    }
    adapter.handle_event(node, unsupported, 0)
    adapter.handle_event(node, unsupported, 1)

    result = GripperCommandResult.from_arrow(node.sent[0][1])
    assert result.error_code == "UNSUPPORTED_VELOCITY"
    assert len(node.sent) == 1

    adapter.handle_event(
        node,
        {
            "type": "INPUT",
            "id": "gripper_goal",
            "value": b"not-arrow",
            "metadata": _metadata("malformed"),
        },
        2,
    )
    malformed = GripperCommandResult.from_arrow(node.sent[-1][1])
    assert malformed.error_code == "INVALID_GOAL"
    assert node.sent[-1][2]["goal_status"] == "aborted"


def test_adapter_ignores_missing_goal_id_and_malformed_active_duplicate() -> None:
    adapter = _adapter()
    node = FakeNode()
    adapter.handle_event(
        node,
        {"type": "INPUT", "id": "joint_state", "value": _state(0.0).to_arrow()},
        0,
    )
    adapter.handle_event(
        node,
        {"type": "INPUT", "id": "gripper_goal", "value": _goal().to_arrow()},
        0,
    )
    assert adapter.controller.active_goal_id is None

    adapter.handle_event(
        node,
        {
            "type": "INPUT",
            "id": "gripper_goal",
            "value": _goal().to_arrow(),
            "metadata": _metadata("active"),
        },
        1,
    )
    adapter.handle_event(
        node,
        {
            "type": "INPUT",
            "id": "gripper_goal",
            "value": b"not-arrow",
            "metadata": _metadata("active"),
        },
        2,
    )

    assert node.sent == []
    assert adapter.controller.active_goal_id == "active"


def test_runner_uses_injected_node_and_clock() -> None:
    node = FakeNode(
        [
            {"type": "INPUT", "id": "joint_state", "value": _state(0.0).to_arrow()},
            {
                "type": "INPUT",
                "id": "gripper_goal",
                "value": _goal(0.0).to_arrow(),
                "metadata": _metadata("runner"),
            },
            {"type": "INPUT", "id": "tick"},
            {"type": "STOP"},
        ]
    )
    times = iter([0, 0, 1, 2])

    result = run_dora_controller_node(
        _config(),
        node_factory=lambda: node,
        clock=lambda: next(times),
    )

    assert result == 0
    assert [item[0] for item in node.sent] == [
        "joint_command",
        "gripper_feedback",
        "gripper_result",
    ]
    terminal = GripperCommandResult.from_arrow(node.sent[-1][1])
    assert terminal.error_code == "SUCCESS"


def test_terminal_result_send_failure_propagates_and_stops_runner(
    caplog: pytest.LogCaptureFixture,
) -> None:
    node = FailingTerminalResultNode(
        [
            {"type": "INPUT", "id": "joint_state", "value": _state(0.0).to_arrow()},
            {
                "type": "INPUT",
                "id": "gripper_goal",
                "value": _goal(0.0).to_arrow(),
                "metadata": _metadata("runner"),
            },
            {"type": "INPUT", "id": "tick"},
            {"type": "INPUT", "id": "tick"},
        ]
    )
    times = iter([0, 0, 1, 2])

    with (
        caplog.at_level(logging.WARNING),
        pytest.raises(ValueError, match="terminal result send failed"),
    ):
        run_dora_controller_node(
            _config(),
            node_factory=lambda: node,
            clock=lambda: next(times),
        )

    assert [item[0] for item in node.sent] == ["joint_command", "gripper_feedback"]
    assert "Ignoring invalid tick input" not in caplog.text


def test_adapter_stops_on_error_and_stop() -> None:
    adapter = _adapter()
    node = FakeNode()

    assert not adapter.handle_event(node, {"type": "ERROR", "error": "boom"}, 0)
    assert not adapter.handle_event(node, {"type": "STOP"}, 0)
