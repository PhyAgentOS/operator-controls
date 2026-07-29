from __future__ import annotations

import pytest
from forge_joint_trajectory_controller.config import ControllerConfig
from forge_joint_trajectory_controller.controller import JointTrajectoryController
from forge_msgs import FollowJointTrajectoryGoal, JointState, JointTrajectory, JointTrajectoryPoint


def test_fake_plant_reaches_trajectory_goal() -> None:
    period_ns = 20_000_000
    config = ControllerConfig(
        joint_names=("j1", "j2"),
        control_period_ns=period_ns,
        feedback_timeout_ns=3 * period_ns,
        path_position_tolerance=0.2,
        goal_position_tolerance=0.01,
        goal_velocity_tolerance=0.02,
        goal_time_tolerance_ns=1_000_000_000,
    )
    controller = JointTrajectoryController(config)
    actual = [0.0, 0.0]
    controller.update_joint_state(
        JointState(name=["j1", "j2"], position=actual, velocity=[0.0, 0.0], effort=[]),
        0,
    )
    goal = FollowJointTrajectoryGoal(
        trajectory=JointTrajectory(
            joint_names=["j1", "j2"],
            points=[
                JointTrajectoryPoint(
                    positions=[0.0, 0.0],
                    velocities=[],
                    accelerations=[],
                    effort=[],
                    time_from_start_ns=0,
                ),
                JointTrajectoryPoint(
                    positions=[1.0, -0.5],
                    velocities=[],
                    accelerations=[],
                    effort=[],
                    time_from_start_ns=1_000_000_000,
                ),
            ],
        ),
        path_tolerance=[],
        goal_tolerance=[],
        goal_time_tolerance_ns=None,
    )
    assert controller.accept_goal("fake-plant", goal, 0) is None

    terminal = None
    previous = list(actual)
    for step in range(1, 151):
        now_ns = step * period_ns
        output = controller.tick(now_ns)
        assert output is not None
        if output.command is not None:
            for index, target in enumerate(output.command.position):
                actual[index] += 0.45 * (target - actual[index])
        velocities = [
            (position - old_position) / (period_ns / 1_000_000_000.0)
            for position, old_position in zip(actual, previous, strict=True)
        ]
        previous = list(actual)
        controller.update_joint_state(
            JointState(
                name=["j1", "j2"],
                position=list(actual),
                velocity=velocities,
                effort=[],
            ),
            now_ns,
        )
        if output.result is not None:
            terminal = output
            break

    assert terminal is not None
    assert terminal.result is not None
    assert terminal.result.error_code == "SUCCESS"
    assert terminal.goal_status == "succeeded"
    assert actual == pytest.approx([1.0, -0.5], abs=0.01)
