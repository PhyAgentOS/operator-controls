from __future__ import annotations

import pytest
from forge_gripper_action_controller.config import ControllerConfig
from forge_gripper_action_controller.controller import GripperActionController
from forge_msgs import GripperCommandGoal, JointState


def _config(**overrides: object) -> ControllerConfig:
    values: dict[str, object] = {
        "joint_name": "gripper",
        "coordinate_type": "prismatic",
        "min_position": 0.0,
        "max_position": 0.04,
        "control_period_ns": 20_000_000,
        "feedback_timeout_ns": 60_000_000,
        "goal_tolerance": 0.0005,
        "stall_velocity_threshold": 0.0001,
        "stall_timeout_ns": 300_000_000,
        "allow_stalling": False,
        "goal_timeout_ns": 2_000_000_000,
    }
    values.update(overrides)
    return ControllerConfig(**values)  # type: ignore[arg-type]


def _state(position: float) -> JointState:
    return JointState(name=["gripper"], position=[position], velocity=[], effort=[])


def test_fake_position_plant_reaches_goal_with_estimated_velocity() -> None:
    period_ns = 20_000_000
    controller = GripperActionController(_config())
    actual = 0.0
    target = actual
    controller.update_joint_state(_state(actual), 0)
    assert controller.accept_goal("fake-plant", GripperCommandGoal(position=0.04), 0) is None

    terminal = None
    saw_estimated_velocity = False
    for step in range(1, 101):
        now_ns = step * period_ns
        output = controller.tick(now_ns)
        assert output is not None
        if output.command is not None:
            assert output.command.name == ["gripper"]
            assert output.command.mode == "position"
            assert output.command.velocity == []
            assert output.command.effort == []
            target = output.command.position[0]
        actual += 0.45 * (target - actual)
        controller.update_joint_state(_state(actual), now_ns)
        if output.feedback is not None and output.feedback.velocity is not None:
            saw_estimated_velocity = True
        if output.result is not None:
            terminal = output
            break

    assert terminal is not None and terminal.result is not None
    assert terminal.result.error_code == "SUCCESS"
    assert terminal.goal_status == "succeeded"
    assert saw_estimated_velocity
    assert actual == pytest.approx(0.04, abs=0.0005)


def test_frozen_fake_plant_stalls_and_holds_observed_position() -> None:
    period_ns = 20_000_000
    controller = GripperActionController(
        _config(
            feedback_timeout_ns=3 * period_ns,
            stall_timeout_ns=100_000_000,
            goal_timeout_ns=1_000_000_000,
        )
    )
    actual = 0.01
    controller.update_joint_state(_state(actual), 0)
    assert controller.accept_goal("frozen", GripperCommandGoal(position=0.04), 0) is None

    terminal = None
    for step in range(1, 20):
        now_ns = step * period_ns
        output = controller.tick(now_ns)
        assert output is not None
        controller.update_joint_state(_state(actual), now_ns)
        if output.result is not None:
            terminal = output
            break

    assert terminal is not None and terminal.result is not None
    assert terminal.result.error_code == "STALLED"
    assert terminal.result.stalled
    assert terminal.command is not None
    assert terminal.command.position == [actual]
