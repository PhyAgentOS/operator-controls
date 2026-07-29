from __future__ import annotations

from pathlib import Path
from typing import Any

import pytest
from forge_joint_trajectory_controller.config import controller_config_from_mapping, load_config
from forge_joint_trajectory_controller.controller import JointTrajectoryController
from forge_joint_trajectory_controller.node import JointTrajectoryControllerNode
from forge_msgs import (
    FollowJointTrajectoryFeedback,
    FollowJointTrajectoryGoal,
    FollowJointTrajectoryResult,
    JointCommand,
    JointState,
    JointTrajectory,
    JointTrajectoryPoint,
)


class FakeNode:
    def __init__(self) -> None:
        self.sent: list[tuple[str, Any, dict[str, Any] | None]] = []

    def __iter__(self):
        return iter(())

    def send_output(self, output_id: str, value: Any, **kwargs: Any) -> None:
        self.sent.append((output_id, value, kwargs.get("metadata")))


def _metadata(goal_id: str) -> dict[str, Any]:
    return {"parameters": {"goal_id": goal_id}}


def _goal() -> FollowJointTrajectoryGoal:
    return FollowJointTrajectoryGoal(
        trajectory=JointTrajectory(
            joint_names=["j1"],
            points=[
                JointTrajectoryPoint(
                    positions=[0.0],
                    velocities=[],
                    accelerations=[],
                    effort=[],
                    time_from_start_ns=0,
                ),
                JointTrajectoryPoint(
                    positions=[1.0],
                    velocities=[],
                    accelerations=[],
                    effort=[],
                    time_from_start_ns=100,
                ),
            ],
        ),
        path_tolerance=[],
        goal_tolerance=[],
        goal_time_tolerance_ns=None,
    )


def _adapter() -> JointTrajectoryControllerNode:
    config = controller_config_from_mapping(
        {
            "joint_names": ["j1"],
            "feedback_timeout_ms": 1,
            "path_position_tolerance": 2.0,
            "goal_position_tolerance": 0.01,
            "goal_velocity_tolerance": 0.01,
        }
    )
    return JointTrajectoryControllerNode(JointTrajectoryController(config))


def test_adapter_routes_goal_feedback_command_and_result_metadata() -> None:
    adapter = _adapter()
    node = FakeNode()
    adapter.handle_event(
        node,
        {
            "type": "INPUT",
            "id": "joint_state",
            "value": JointState(name=["j1"], position=[0.0], velocity=[0.0], effort=[]).to_arrow(),
        },
        0,
    )
    adapter.handle_event(
        node,
        {
            "type": "INPUT",
            "id": "trajectory_goal",
            "value": _goal().to_arrow(),
            "metadata": _metadata("goal-1"),
        },
        0,
    )
    adapter.handle_event(node, {"type": "INPUT", "id": "tick"}, 50)

    assert [item[0] for item in node.sent] == ["joint_command", "trajectory_feedback"]
    command = JointCommand.from_arrow(node.sent[0][1])
    assert command.position == [0.5]
    feedback = FollowJointTrajectoryFeedback.from_arrow(node.sent[1][1])
    assert feedback.elapsed_ns == 50
    assert node.sent[1][2] == _metadata("goal-1")

    adapter.handle_event(
        node,
        {
            "type": "INPUT",
            "id": "joint_state",
            "value": JointState(name=["j1"], position=[1.0], velocity=[0.0], effort=[]).to_arrow(),
        },
        100,
    )
    adapter.handle_event(node, {"type": "INPUT", "id": "tick"}, 100)

    assert [item[0] for item in node.sent[-3:]] == [
        "joint_command",
        "trajectory_feedback",
        "trajectory_result",
    ]
    result = FollowJointTrajectoryResult.from_arrow(node.sent[-1][1])
    assert result.error_code == "SUCCESS"
    assert node.sent[-1][2] == {"parameters": {"goal_id": "goal-1", "goal_status": "succeeded"}}


def test_adapter_cancel_uses_goal_id_and_emits_canceled_result() -> None:
    adapter = _adapter()
    node = FakeNode()
    adapter.handle_event(
        node,
        {
            "type": "INPUT",
            "id": "joint_state",
            "value": JointState(name=["j1"], position=[0.25], velocity=[], effort=[]).to_arrow(),
        },
        0,
    )
    future_goal = FollowJointTrajectoryGoal(
        trajectory=JointTrajectory(
            joint_names=["j1"],
            points=[
                JointTrajectoryPoint(
                    positions=[1.0],
                    velocities=[],
                    accelerations=[],
                    effort=[],
                    time_from_start_ns=100,
                )
            ],
        ),
        path_tolerance=[],
        goal_tolerance=[],
        goal_time_tolerance_ns=None,
    )
    adapter.handle_event(
        node,
        {
            "type": "INPUT",
            "id": "trajectory_goal",
            "value": future_goal.to_arrow(),
            "metadata": _metadata("goal-2"),
        },
        0,
    )
    adapter.handle_event(
        node,
        {
            "type": "INPUT",
            "id": "trajectory_cancel",
            "value": b"",
            "metadata": _metadata("goal-2"),
        },
        1,
    )

    assert [item[0] for item in node.sent] == ["joint_command", "trajectory_result"]
    assert JointCommand.from_arrow(node.sent[0][1]).position == [0.25]
    result = FollowJointTrajectoryResult.from_arrow(node.sent[1][1])
    assert result.error_code == "CANCELED"
    assert node.sent[1][2]["parameters"]["goal_status"] == "canceled"


def test_malformed_duplicate_does_not_terminate_the_active_goal() -> None:
    adapter = _adapter()
    node = FakeNode()
    adapter.handle_event(
        node,
        {
            "type": "INPUT",
            "id": "joint_state",
            "value": JointState(name=["j1"], position=[0.0], velocity=[0.0], effort=[]).to_arrow(),
        },
        0,
    )
    adapter.handle_event(
        node,
        {
            "type": "INPUT",
            "id": "trajectory_goal",
            "value": _goal().to_arrow(),
            "metadata": _metadata("active"),
        },
        0,
    )
    adapter.handle_event(
        node,
        {
            "type": "INPUT",
            "id": "trajectory_goal",
            "value": b"not-arrow",
            "metadata": _metadata("active"),
        },
        1,
    )

    assert node.sent == []
    assert adapter.controller.active_goal_id == "active"


def test_adapter_rejects_malformed_goal_and_ignores_missing_goal_id() -> None:
    adapter = _adapter()
    node = FakeNode()

    adapter.handle_event(
        node,
        {
            "type": "INPUT",
            "id": "trajectory_goal",
            "value": b"not-arrow",
            "metadata": _metadata("bad"),
        },
        0,
    )
    result = FollowJointTrajectoryResult.from_arrow(node.sent[0][1])
    assert result.error_code == "INVALID_GOAL"
    assert node.sent[0][2]["parameters"]["goal_status"] == "aborted"

    adapter.handle_event(
        node,
        {"type": "INPUT", "id": "trajectory_goal", "value": _goal().to_arrow()},
        1,
    )
    assert len(node.sent) == 1


def test_config_mapping_and_yaml_use_milliseconds(tmp_path: Path) -> None:
    config_path = tmp_path / "controller.yaml"
    config_path.write_text(
        """
joint_names: [j1, j2]
control_period_ms: 20
feedback_timeout_ms: 150
goal_time_tolerance_ms: 400
path_position_tolerance: 0.2
""".strip(),
        encoding="utf-8",
    )

    config = load_config(config_path)

    assert config.joint_names == ("j1", "j2")
    assert config.control_period_ns == 20_000_000
    assert config.feedback_timeout_ns == 150_000_000
    assert config.goal_time_tolerance_ns == 400_000_000
    with pytest.raises(ValueError, match="unknown"):
        controller_config_from_mapping({"joint_names": ["j1"], "unexpected": True})
    with pytest.raises(ValueError, match="positive integer"):
        controller_config_from_mapping({"joint_names": ["j1"], "result_cache_size": True})
