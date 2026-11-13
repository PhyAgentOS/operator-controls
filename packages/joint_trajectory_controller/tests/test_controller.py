from __future__ import annotations

import pytest
from forge_joint_trajectory_controller.config import ControllerConfig
from forge_joint_trajectory_controller.controller import JointTrajectoryController
from forge_msgs import (
    FollowJointTrajectoryGoal,
    JointState,
    JointTolerance,
    JointTrajectory,
    JointTrajectoryPoint,
)


def _config(**overrides: object) -> ControllerConfig:
    values: dict[str, object] = {
        "joint_names": ("j1", "j2"),
        "feedback_timeout_ns": 100,
        "start_position_tolerance": 0.1,
        "path_position_tolerance": 0.2,
        "goal_position_tolerance": 0.02,
        "goal_velocity_tolerance": 0.05,
        "goal_time_tolerance_ns": 50,
    }
    values.update(overrides)
    if "path_abort_tolerance" not in values:
        values["path_abort_tolerance"] = max(0.5, float(values["path_position_tolerance"]))
    return ControllerConfig(**values)  # type: ignore[arg-type]


def _state(
    positions: tuple[float, float] = (0.0, 0.0),
    velocities: tuple[float, float] | None = (0.0, 0.0),
    *,
    reversed_order: bool = False,
) -> JointState:
    names = ["j1", "j2"]
    pos = list(positions)
    vel = [] if velocities is None else list(velocities)
    if reversed_order:
        names.reverse()
        pos.reverse()
        vel.reverse()
    return JointState(name=names, position=pos, velocity=vel, effort=[])


def _goal(
    target: tuple[float, float] = (1.0, -1.0),
    *,
    first: tuple[float, float] = (0.0, 0.0),
    duration_ns: int = 100,
    first_time_ns: int = 0,
    path_tolerance: list[JointTolerance] | None = None,
    goal_tolerance: list[JointTolerance] | None = None,
    goal_time_tolerance_ns: int | None = None,
) -> FollowJointTrajectoryGoal:
    points = [
        JointTrajectoryPoint(
            positions=list(first),
            velocities=[],
            accelerations=[],
            effort=[],
            time_from_start_ns=first_time_ns,
        )
    ]
    if duration_ns != first_time_ns:
        points.append(
            JointTrajectoryPoint(
                positions=list(target),
                velocities=[],
                accelerations=[],
                effort=[],
                time_from_start_ns=duration_ns,
            )
        )
    return FollowJointTrajectoryGoal(
        trajectory=JointTrajectory(joint_names=["j1", "j2"], points=points),
        path_tolerance=path_tolerance or [],
        goal_tolerance=goal_tolerance or [],
        goal_time_tolerance_ns=goal_time_tolerance_ns,
    )


def test_requires_fresh_complete_feedback_and_accepts_reordered_state() -> None:
    controller = JointTrajectoryController(_config())

    missing = controller.accept_goal("missing", _goal(), 0)
    assert missing is not None
    assert missing.result is not None
    assert missing.result.error_code == "NO_FRESH_ROBOT_STATE"

    with pytest.raises(ValueError, match="missing"):
        controller.update_joint_state(
            JointState(name=["j1"], position=[0.0], velocity=[], effort=[]),
            1,
        )

    controller.update_joint_state(_state(reversed_order=True), 2)
    assert controller.accept_goal("accepted", _goal(), 2) is None
    assert controller.active_goal_id == "accepted"


def test_rejects_busy_start_mismatch_and_wrong_joint_order() -> None:
    controller = JointTrajectoryController(_config())
    controller.update_joint_state(_state(), 0)

    wrong_order = _goal()
    wrong_order.trajectory.joint_names = ["j2", "j1"]
    wrong = controller.accept_goal("wrong", wrong_order, 0)
    assert wrong is not None and wrong.result is not None
    assert wrong.result.error_code == "INVALID_JOINTS"

    mismatch = controller.accept_goal("mismatch", _goal(first=(0.5, 0.0)), 0)
    assert mismatch is not None and mismatch.result is not None
    assert mismatch.result.error_code == "START_STATE_MISMATCH"

    assert controller.accept_goal("active", _goal(), 0) is None
    with pytest.raises(ValueError, match="different payload"):
        controller.accept_goal("active", _goal(target=(0.5, -0.5)), 0)
    assert controller.active_goal_id == "active"
    busy = controller.accept_goal("busy", _goal(), 0)
    assert busy is not None and busy.result is not None
    assert busy.result.error_code == "BUSY"


def test_tick_emits_full_position_command_and_feedback() -> None:
    controller = JointTrajectoryController(_config(path_position_tolerance=2.0))
    controller.update_joint_state(_state(), 0)
    controller.accept_goal("goal", _goal(), 0)

    output = controller.tick(25)

    assert output is not None
    assert output.goal_id == "goal"
    assert output.command is not None
    assert output.command.name == ["j1", "j2"]
    assert output.command.mode == "position"
    assert output.command.position == pytest.approx([0.25, -0.25])
    assert output.feedback is not None
    assert output.feedback.elapsed_ns == 25


def test_path_violation_aborts_and_holds_actual() -> None:
    controller = JointTrajectoryController(
        _config(path_position_tolerance=0.01, path_abort_tolerance=0.01)
    )
    controller.update_joint_state(_state(), 0)
    controller.accept_goal("goal", _goal(), 0)

    output = controller.tick(25)

    assert output is not None and output.result is not None
    assert output.result.error_code == "PATH_TOLERANCE_VIOLATED"
    assert output.goal_status == "aborted"
    assert output.command is not None
    assert output.command.position == [0.0, 0.0]
    assert controller.state == "HOLDING"
    hold = controller.tick(26)
    assert hold is not None and hold.command is not None
    assert hold.command.position == [0.0, 0.0]


def test_feedback_stale_aborts_and_holds() -> None:
    controller = JointTrajectoryController(
        _config(feedback_timeout_ns=10, path_position_tolerance=2.0)
    )
    controller.update_joint_state(_state(), 0)
    controller.accept_goal("goal", _goal(), 0)

    output = controller.tick(11)

    assert output is not None and output.result is not None
    assert output.result.error_code == "FEEDBACK_STALE"
    assert output.goal_status == "aborted"
    assert output.command is not None
    assert output.command.position == [0.0, 0.0]


def test_cancel_is_goal_scoped_idempotent_and_holds_actual() -> None:
    controller = JointTrajectoryController(_config())
    controller.update_joint_state(_state((0.2, -0.1)), 0)
    assert (
        controller.accept_goal(
            "goal",
            _goal(first_time_ns=100, duration_ns=100),
            0,
        )
        is None
    )

    assert controller.cancel_goal("other", 1) is None
    canceled = controller.cancel_goal("goal", 1)
    assert canceled is not None and canceled.result is not None
    assert canceled.result.error_code == "CANCELED"
    assert canceled.goal_status == "canceled"
    assert canceled.command is not None
    assert canceled.command.position == [0.2, -0.1]
    cached_cancel = controller.cancel_goal("goal", 2)
    assert cached_cancel is not None
    assert cached_cancel.result == canceled.result
    assert cached_cancel.command is None
    assert cached_cancel.feedback is None
    cached_goal = controller.accept_goal("goal", _goal(), 2)
    assert cached_goal == cached_cancel


def test_settles_then_succeeds_and_holds_final_target() -> None:
    controller = JointTrajectoryController(_config(path_position_tolerance=2.0))
    controller.update_joint_state(_state(), 0)
    controller.accept_goal("goal", _goal(), 0)

    controller.update_joint_state(_state((0.98, -0.98), (0.2, -0.2)), 100)
    settling = controller.tick(100)
    assert settling is not None and settling.result is None
    assert controller.state == "SETTLING"

    controller.update_joint_state(_state((1.0, -1.0), (0.0, 0.0)), 120)
    succeeded = controller.tick(120)
    assert succeeded is not None and succeeded.result is not None
    assert succeeded.result.error_code == "SUCCESS"
    assert succeeded.goal_status == "succeeded"
    assert succeeded.command is not None
    assert succeeded.command.position == [1.0, -1.0]
    assert controller.tick(121).command.position == [1.0, -1.0]  # type: ignore[union-attr]


def test_goal_tolerance_timeout_and_override_validation() -> None:
    controller = JointTrajectoryController(_config(path_position_tolerance=2.0))
    controller.update_joint_state(_state(), 0)
    unsupported = _goal(path_tolerance=[JointTolerance(joint_name="j1", velocity=0.1)])
    invalid = controller.accept_goal("invalid", unsupported, 0)
    assert invalid is not None and invalid.result is not None
    assert invalid.result.error_code == "INVALID_GOAL"

    controller.accept_goal("goal", _goal(goal_time_tolerance_ns=10), 0)
    controller.update_joint_state(_state(), 100)
    assert controller.tick(100).result is None  # type: ignore[union-attr]
    controller.update_joint_state(_state(), 111)
    timed_out = controller.tick(111)
    assert timed_out is not None and timed_out.result is not None
    assert timed_out.result.error_code == "GOAL_TOLERANCE_VIOLATED"


def test_goal_velocity_tolerance_compares_desired_and_actual() -> None:
    controller = JointTrajectoryController(_config(path_position_tolerance=2.0))
    controller.update_joint_state(_state(), 0)
    goal = FollowJointTrajectoryGoal(
        trajectory=JointTrajectory(
            joint_names=["j1", "j2"],
            points=[
                JointTrajectoryPoint(
                    positions=[0.0, 0.0],
                    velocities=[0.0, 0.0],
                    accelerations=[],
                    effort=[],
                    time_from_start_ns=0,
                ),
                JointTrajectoryPoint(
                    positions=[1.0, -1.0],
                    velocities=[1.0, -1.0],
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
    assert controller.accept_goal("moving-end", goal, 0) is None
    controller.update_joint_state(_state((1.0, -1.0), (1.0, -1.0)), 100)

    output = controller.tick(100)

    assert output is not None and output.result is not None
    assert output.result.error_code == "SUCCESS"


def test_moderate_tracking_lag_does_not_abort() -> None:
    """A lag inside the hard band is normal; plant capability is not judged per tick."""
    controller = JointTrajectoryController(_config(path_abort_tolerance=0.5))
    controller.update_joint_state(_state(), 0)
    assert controller.accept_goal("goal", _goal(), 0) is None

    controller.update_joint_state(_state(positions=(0.0, 0.0)), 25)
    output = controller.tick(25)

    assert output is not None
    assert output.result is None
    assert output.command is not None


def test_hard_deviation_aborts_and_reports_the_joint() -> None:
    controller = JointTrajectoryController(_config(path_abort_tolerance=0.5))
    controller.update_joint_state(_state(), 0)
    assert controller.accept_goal(
        "goal", _goal(target=(0.2, -2.0), duration_ns=1_000_000_000), 0
    ) is None

    controller.update_joint_state(_state(positions=(0.0, 0.0)), 500_000_000)
    output = controller.tick(500_000_000)

    assert output is not None and output.result is not None
    assert output.result.error_code == "PATH_TOLERANCE_VIOLATED"
    assert "j2" in output.result.message
    assert "exceeds abort tolerance" in output.result.message


def test_stalled_plant_aborts_after_the_window() -> None:
    """Reference keeps moving while the plant never moves -> stall abort."""
    controller = JointTrajectoryController(
        _config(path_abort_tolerance=5.0, stuck_window_ns=200_000_000)
    )
    controller.update_joint_state(_state(), 0)
    assert controller.accept_goal("goal", _goal(duration_ns=2_000_000_000), 0) is None

    result = None
    for elapsed_ns in range(0, 1_000_000_000, 50_000_000):
        controller.update_joint_state(_state(positions=(0.0, 0.0)), elapsed_ns)
        output = controller.tick(elapsed_ns)
        if output is not None and output.result is not None:
            result = output
            break

    assert result is not None, "a frozen plant must abort instead of finishing silently"
    assert result.result is not None
    assert result.result.error_code == "PATH_TOLERANCE_VIOLATED"
    assert "no measurable progress" in result.result.message
