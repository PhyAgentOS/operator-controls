from __future__ import annotations

import math

import pytest
from forge_gripper_action_controller.config import ControllerConfig
from forge_gripper_action_controller.controller import GripperActionController
from forge_msgs import GripperCommandGoal, JointState

NS = 1_000_000_000


def _config(**overrides: object) -> ControllerConfig:
    values: dict[str, object] = {
        "joint_name": "gripper",
        "coordinate_type": "prismatic",
        "min_position": 0.0,
        "max_position": 1.0,
        "control_period_ns": 20,
        "feedback_timeout_ns": 200,
        "goal_tolerance": 0.01,
        "stall_velocity_threshold": 0.05,
        "stall_timeout_ns": 100,
        "allow_stalling": False,
        "goal_timeout_ns": 500,
        "terminal_history_size": 128,
    }
    values.update(overrides)
    return ControllerConfig(**values)  # type: ignore[arg-type]


def _state(
    position: float,
    velocity: float | None = 0.0,
    effort: float | None = None,
) -> JointState:
    return JointState(
        name=["gripper"],
        position=[position],
        velocity=[] if velocity is None else [velocity],
        effort=[] if effort is None else [effort],
    )


def _goal(
    position: float = 0.8,
    *,
    max_velocity: float | None = None,
    max_effort: float | None = None,
) -> GripperCommandGoal:
    return GripperCommandGoal(
        position=position,
        max_velocity=max_velocity,
        max_effort=max_effort,
    )


def test_requires_fresh_state_and_accepts_reordered_extra_joint_state() -> None:
    controller = GripperActionController(_config())

    missing = controller.accept_goal("missing", _goal(), 0)
    assert missing is not None and missing.result is not None
    assert missing.result.error_code == "NO_FRESH_ROBOT_STATE"
    assert missing.result.position is None

    with pytest.raises(ValueError, match="missing configured joint"):
        controller.update_joint_state(
            JointState(name=["arm"], position=[0.0], velocity=[], effort=[]),
            1,
        )

    controller.update_joint_state(
        JointState(
            name=["arm", "gripper"],
            position=[9.0, 0.2],
            velocity=[1.0, 0.0],
            effort=[2.0, 3.0],
        ),
        2,
    )
    assert controller.accept_goal("accepted", _goal(), 2) is None
    assert controller.active_goal_id == "accepted"

    stale = GripperActionController(_config(feedback_timeout_ns=10))
    stale.update_joint_state(_state(0.0), 0)
    rejected = stale.accept_goal("stale", _goal(), 11)
    assert rejected is not None and rejected.result is not None
    assert rejected.result.error_code == "NO_FRESH_ROBOT_STATE"


def test_observed_and_differential_velocity_and_nullable_effort_feedback() -> None:
    controller = GripperActionController(
        _config(
            feedback_timeout_ns=2 * NS,
            stall_timeout_ns=2 * NS,
            goal_timeout_ns=5 * NS,
        )
    )
    controller.update_joint_state(_state(0.0, velocity=None), 0)
    controller.accept_goal("goal", _goal(), 0)

    initial = controller.tick(0)
    assert initial is not None and initial.feedback is not None
    assert initial.feedback.velocity is None
    assert initial.feedback.effort is None

    controller.update_joint_state(_state(0.2, velocity=None, effort=1.5), NS)
    estimated = controller.tick(NS)
    assert estimated is not None and estimated.feedback is not None
    assert estimated.feedback.velocity == pytest.approx(0.2)
    assert estimated.feedback.effort == pytest.approx(1.5)

    controller.update_joint_state(_state(0.3, velocity=-0.4), NS + NS // 2)
    observed = controller.tick(NS + NS // 2)
    assert observed is not None and observed.feedback is not None
    assert observed.feedback.velocity == pytest.approx(-0.4)
    assert observed.feedback.effort is None

    controller.update_joint_state(_state(0.4, velocity=None), NS + NS // 2)
    same_timestamp = controller.tick(NS + NS // 2)
    assert same_timestamp is not None and same_timestamp.feedback is not None
    assert same_timestamp.feedback.velocity is None


def test_rejects_invalid_joint_state_and_non_monotonic_time() -> None:
    controller = GripperActionController(_config())

    with pytest.raises(ValueError, match="position"):
        controller.update_joint_state(_state(math.inf), 0)

    controller = GripperActionController(_config())
    controller.update_joint_state(_state(0.0), 1)
    with pytest.raises(ValueError, match="monotonic"):
        controller.tick(0)


def test_rejects_all_non_null_limits_and_out_of_range_positions_once() -> None:
    controller = GripperActionController(_config())
    controller.update_joint_state(_state(0.2), 0)

    velocity = controller.accept_goal("velocity", _goal(max_velocity=0.0), 0)
    assert velocity is not None and velocity.result is not None
    assert velocity.result.error_code == "UNSUPPORTED_VELOCITY"
    assert velocity.command is None
    assert controller.accept_goal("velocity", _goal(), 0) is None
    assert controller.cancel_goal("velocity", 0) is None

    effort = controller.accept_goal("effort", _goal(max_effort=0.0), 0)
    assert effort is not None and effort.result is not None
    assert effort.result.error_code == "UNSUPPORTED_EFFORT"

    limit = controller.accept_goal("limit", _goal(1.01), 0)
    assert limit is not None and limit.result is not None
    assert limit.result.error_code == "POSITION_LIMIT_VIOLATION"

    lower = GripperActionController(_config())
    lower.update_joint_state(_state(0.5), 0)
    assert lower.accept_goal("lower", _goal(0.0), 0) is None

    upper = GripperActionController(_config())
    upper.update_joint_state(_state(0.5), 0)
    assert upper.accept_goal("upper", _goal(1.0), 0) is None


def test_active_duplicate_is_idempotent_and_other_goal_is_busy_once() -> None:
    controller = GripperActionController(_config())
    controller.update_joint_state(_state(0.0), 0)
    goal = _goal()

    assert controller.accept_goal("active", goal, 0) is None
    assert controller.accept_goal("active", goal, 1) is None
    with pytest.raises(ValueError, match="different payload"):
        controller.accept_goal("active", _goal(0.7), 1)

    busy = controller.accept_goal("busy", goal, 1)
    assert busy is not None and busy.result is not None
    assert busy.result.error_code == "BUSY"
    assert controller.accept_goal("busy", goal, 2) is None
    assert controller.active_goal_id == "active"


def test_terminal_history_is_bounded_and_recent_retries_remain_idempotent() -> None:
    controller = GripperActionController(_config(terminal_history_size=2))
    controller.update_joint_state(_state(0.2), 0)

    first = controller.accept_goal("first", _goal(max_velocity=0.0), 0)
    second = controller.accept_goal("second", _goal(max_effort=0.0), 0)
    assert first is not None and first.result is not None
    assert second is not None and second.result is not None

    assert controller.accept_goal("first", _goal(), 0) is None
    third = controller.accept_goal("third", _goal(1.1), 0)
    assert third is not None and third.result is not None

    assert controller.accept_goal("first", _goal(), 0) is None
    assert controller.accept_goal("second", _goal(), 0) is None
    assert controller.active_goal_id == "second"


def test_tick_emits_only_sparse_position_command_and_observed_feedback() -> None:
    controller = GripperActionController(_config())
    controller.update_joint_state(_state(0.2, velocity=0.1, effort=2.5), 0)
    controller.accept_goal("goal", _goal(0.8), 0)

    output = controller.tick(10)

    assert output is not None
    assert output.goal_id == "goal"
    assert output.command is not None
    assert output.command.name == ["gripper"]
    assert output.command.mode == "position"
    assert output.command.position == [0.8]
    assert output.command.velocity == []
    assert output.command.effort == []
    assert output.command.kp == []
    assert output.command.kd == []
    assert output.feedback is not None
    assert output.feedback.elapsed_ns == 10
    assert output.feedback.position == pytest.approx(0.2)
    assert output.feedback.velocity == pytest.approx(0.1)
    assert output.feedback.effort == pytest.approx(2.5)
    assert not output.feedback.stalled
    assert not output.feedback.reached_goal


def test_success_holds_target_and_never_reemits_terminal_result() -> None:
    controller = GripperActionController(_config())
    controller.update_joint_state(_state(0.795, velocity=0.0), 0)
    goal = _goal(0.8)
    controller.accept_goal("goal", goal, 0)

    succeeded = controller.tick(1)

    assert succeeded is not None and succeeded.result is not None
    assert succeeded.result.error_code == "SUCCESS"
    assert succeeded.result.reached_goal
    assert not succeeded.result.stalled
    assert succeeded.goal_status == "succeeded"
    assert succeeded.command is not None
    assert succeeded.command.position == [0.8]
    assert controller.state == "HOLDING"

    hold = controller.tick(2)
    assert hold is not None and hold.command is not None
    assert hold.command.position == [0.8]
    assert hold.result is None
    assert controller.accept_goal("goal", goal, 3) is None
    assert controller.cancel_goal("goal", 4) is None


def test_cancel_is_goal_scoped_and_holds_current_position_once() -> None:
    controller = GripperActionController(_config())
    controller.update_joint_state(_state(0.25, velocity=None, effort=1.0), 0)
    controller.accept_goal("goal", _goal(0.8), 0)

    assert controller.cancel_goal("other", 1) is None
    canceled = controller.cancel_goal("goal", 1)

    assert canceled is not None and canceled.result is not None
    assert canceled.result.error_code == "CANCELED"
    assert canceled.goal_status == "canceled"
    assert canceled.command is not None
    assert canceled.command.position == [0.25]
    assert canceled.command.velocity == []
    assert canceled.command.effort == []
    assert controller.cancel_goal("goal", 2) is None
    hold = controller.tick(3)
    assert hold is not None and hold.result is None
    assert hold.command is not None and hold.command.position == [0.25]


def test_stale_feedback_aborts_and_holds_latest_observation() -> None:
    controller = GripperActionController(_config(feedback_timeout_ns=10))
    controller.update_joint_state(_state(0.3), 0)
    controller.accept_goal("goal", _goal(), 0)

    stale = controller.tick(11)

    assert stale is not None and stale.result is not None
    assert stale.result.error_code == "FEEDBACK_STALE"
    assert stale.goal_status == "aborted"
    assert stale.command is not None and stale.command.position == [0.3]
    assert controller.tick(12).result is None  # type: ignore[union-attr]


@pytest.mark.parametrize("allow_stalling", [False, True])
def test_stall_terminal_behavior_and_hold_policy(allow_stalling: bool) -> None:
    controller = GripperActionController(
        _config(
            allow_stalling=allow_stalling,
            feedback_timeout_ns=1000,
            stall_timeout_ns=100,
            goal_timeout_ns=500,
        )
    )
    controller.update_joint_state(_state(0.2, velocity=0.0, effort=4.0), 0)
    controller.accept_goal("goal", _goal(0.8), 0)

    assert controller.tick(99).result is None  # type: ignore[union-attr]
    stalled = controller.tick(100)

    assert stalled is not None and stalled.result is not None
    assert stalled.result.stalled
    assert not stalled.result.reached_goal
    assert stalled.feedback is not None and stalled.feedback.stalled
    assert stalled.command is not None
    if allow_stalling:
        assert stalled.result.error_code == "SUCCESS"
        assert stalled.goal_status == "succeeded"
        assert stalled.command.position == [0.8]
    else:
        assert stalled.result.error_code == "STALLED"
        assert stalled.goal_status == "aborted"
        assert stalled.command.position == [0.2]


def test_initially_reached_goal_can_depart_after_stall_deadline_without_latched_stall() -> None:
    controller = GripperActionController(
        _config(
            feedback_timeout_ns=1000,
            stall_timeout_ns=100,
            goal_timeout_ns=500,
        )
    )
    controller.update_joint_state(_state(0.8, velocity=0.0), 0)
    controller.accept_goal("goal", _goal(0.8), 0)

    controller.update_joint_state(_state(0.7, velocity=-1.0), 150)
    departed = controller.tick(150)

    assert departed is not None
    assert departed.result is None
    assert departed.feedback is not None and not departed.feedback.stalled
    assert controller.active_goal_id == "goal"


def test_low_velocity_stall_timer_starts_at_departure_from_goal_tolerance() -> None:
    controller = GripperActionController(
        _config(
            feedback_timeout_ns=1000,
            stall_timeout_ns=100,
            goal_timeout_ns=500,
        )
    )
    controller.update_joint_state(_state(0.8, velocity=0.0), 0)
    controller.accept_goal("goal", _goal(0.8), 0)

    controller.update_joint_state(_state(0.7, velocity=0.0), 150)
    before_deadline = controller.tick(249)
    stalled = controller.tick(250)

    assert before_deadline is not None and before_deadline.result is None
    assert stalled is not None and stalled.result is not None
    assert stalled.result.error_code == "STALLED"
    assert stalled.result.stalled


def test_differential_movement_starts_and_resets_stall_timer() -> None:
    controller = GripperActionController(
        _config(
            feedback_timeout_ns=NS,
            stall_velocity_threshold=0.5,
            stall_timeout_ns=100_000_000,
            goal_timeout_ns=500_000_000,
        )
    )
    controller.update_joint_state(_state(0.0, velocity=None), 0)
    controller.accept_goal("goal", _goal(0.8), 0)

    unknown_velocity = controller.tick(99_000_000)
    assert unknown_velocity is not None and unknown_velocity.result is None

    controller.update_joint_state(_state(0.1, velocity=None), 100_000_000)
    moving = controller.tick(199_000_000)
    assert moving is not None and moving.feedback is not None
    assert moving.feedback.velocity == pytest.approx(1.0)
    assert moving.result is None

    controller.update_joint_state(_state(0.1, velocity=None), 200_000_000)
    stalled = controller.tick(200_000_000)
    assert stalled is not None and stalled.result is not None
    assert stalled.result.error_code == "STALLED"


def test_goal_timeout_wins_when_its_deadline_is_earlier() -> None:
    controller = GripperActionController(
        _config(
            feedback_timeout_ns=1000,
            stall_timeout_ns=1000,
            goal_timeout_ns=100,
        )
    )
    controller.update_joint_state(_state(0.0, velocity=1.0), 0)
    controller.accept_goal("goal", _goal(), 0)

    timed_out = controller.tick(100)

    assert timed_out is not None and timed_out.result is not None
    assert timed_out.result.error_code == "EXECUTION_TIMED_OUT"
    assert timed_out.goal_status == "aborted"
    assert timed_out.command is not None and timed_out.command.position == [0.0]


def test_goal_observed_after_hard_deadline_times_out() -> None:
    controller = GripperActionController(
        _config(
            feedback_timeout_ns=1000,
            stall_timeout_ns=1000,
            goal_timeout_ns=100,
        )
    )
    controller.update_joint_state(_state(0.0, velocity=1.0), 0)
    controller.accept_goal("goal", _goal(), 0)
    controller.update_joint_state(_state(0.8, velocity=1.0), 101)

    timed_out = controller.tick(101)

    assert timed_out is not None and timed_out.result is not None
    assert timed_out.result.error_code == "EXECUTION_TIMED_OUT"
    assert timed_out.goal_status == "aborted"
    assert not timed_out.result.reached_goal
    assert timed_out.command is not None and timed_out.command.position == [0.8]


def test_terminal_tick_before_same_timestamp_goal_observation_cannot_be_revised() -> None:
    controller = GripperActionController(
        _config(
            feedback_timeout_ns=1000,
            stall_timeout_ns=1000,
            goal_timeout_ns=100,
        )
    )
    goal = _goal()
    controller.update_joint_state(_state(0.0, velocity=1.0), 0)
    controller.accept_goal("goal", goal, 0)

    timed_out = controller.tick(100)
    controller.update_joint_state(_state(0.8, velocity=0.0), 100)

    assert timed_out is not None and timed_out.result is not None
    assert timed_out.result.error_code == "EXECUTION_TIMED_OUT"
    assert controller.accept_goal("goal", goal, 100) is None
    later = controller.tick(100)
    assert later is not None and later.result is None


def test_goal_observed_at_hard_deadline_can_succeed_on_delayed_tick() -> None:
    controller = GripperActionController(
        _config(
            feedback_timeout_ns=1000,
            stall_timeout_ns=1000,
            goal_timeout_ns=100,
        )
    )
    controller.update_joint_state(_state(0.0, velocity=1.0), 0)
    controller.accept_goal("goal", _goal(), 0)
    controller.update_joint_state(_state(0.8, velocity=0.0), 100)
    controller.update_joint_state(_state(0.8, velocity=0.0), 101)

    succeeded = controller.tick(101)

    assert succeeded is not None and succeeded.result is not None
    assert succeeded.result.error_code == "SUCCESS"
    assert succeeded.goal_status == "succeeded"
    assert succeeded.result.reached_goal


def test_departure_then_late_reentry_does_not_restore_on_time_goal_reach() -> None:
    controller = GripperActionController(
        _config(
            feedback_timeout_ns=1000,
            stall_timeout_ns=1000,
            goal_timeout_ns=100,
        )
    )
    controller.update_joint_state(_state(0.0, velocity=1.0), 0)
    controller.accept_goal("goal", _goal(), 0)
    controller.update_joint_state(_state(0.8, velocity=0.0), 80)
    controller.update_joint_state(_state(0.7, velocity=-1.0), 90)
    controller.update_joint_state(_state(0.8, velocity=1.0), 150)

    timed_out = controller.tick(150)

    assert timed_out is not None and timed_out.result is not None
    assert timed_out.result.error_code == "EXECUTION_TIMED_OUT"
    assert timed_out.goal_status == "aborted"
    assert not timed_out.result.reached_goal


def test_earlier_goal_timeout_wins_when_delayed_tick_also_sees_stale_feedback() -> None:
    controller = GripperActionController(
        _config(
            feedback_timeout_ns=200,
            stall_timeout_ns=1000,
            goal_timeout_ns=100,
        )
    )
    controller.update_joint_state(_state(0.0, velocity=1.0), 0)
    controller.accept_goal("goal", _goal(), 0)

    timed_out = controller.tick(250)

    assert timed_out is not None and timed_out.result is not None
    assert timed_out.result.error_code == "EXECUTION_TIMED_OUT"
    assert timed_out.goal_status == "aborted"
    assert timed_out.command is not None and timed_out.command.position == [0.0]


def test_earlier_stale_feedback_wins_when_delayed_tick_also_sees_goal_timeout() -> None:
    controller = GripperActionController(
        _config(
            feedback_timeout_ns=100,
            stall_timeout_ns=1000,
            goal_timeout_ns=200,
        )
    )
    controller.update_joint_state(_state(0.0, velocity=1.0), 0)
    controller.accept_goal("goal", _goal(), 0)

    stale = controller.tick(250)

    assert stale is not None and stale.result is not None
    assert stale.result.error_code == "FEEDBACK_STALE"
    assert stale.goal_status == "aborted"
    assert stale.command is not None and stale.command.position == [0.0]


def test_goal_timeout_wins_a_tie_with_stall() -> None:
    controller = GripperActionController(
        _config(
            feedback_timeout_ns=1000,
            stall_timeout_ns=100,
            goal_timeout_ns=100,
        )
    )
    controller.update_joint_state(_state(0.0, velocity=0.0), 0)
    controller.accept_goal("goal", _goal(), 0)

    timed_out = controller.tick(150)

    assert timed_out is not None and timed_out.result is not None
    assert timed_out.result.error_code == "EXECUTION_TIMED_OUT"
    assert not timed_out.result.stalled


def test_earlier_stall_wins_over_goal_reached_before_delayed_tick() -> None:
    controller = GripperActionController(
        _config(
            feedback_timeout_ns=1000,
            stall_timeout_ns=100,
            goal_timeout_ns=500,
        )
    )
    controller.update_joint_state(_state(0.0, velocity=0.0), 0)
    controller.accept_goal("goal", _goal(), 0)
    controller.update_joint_state(_state(0.8, velocity=0.0), 150)

    stalled = controller.tick(150)

    assert stalled is not None and stalled.result is not None
    assert stalled.result.error_code == "STALLED"
    assert stalled.result.stalled
    assert not stalled.result.reached_goal
    assert stalled.result.position == pytest.approx(0.8)
    assert stalled.command is not None and stalled.command.position == [0.8]


def test_on_time_goal_reach_held_through_late_updates_succeeds() -> None:
    controller = GripperActionController(
        _config(
            feedback_timeout_ns=1000,
            stall_timeout_ns=100,
            goal_timeout_ns=100,
        )
    )
    controller.update_joint_state(_state(0.0, velocity=0.0), 0)
    controller.accept_goal("goal", _goal(), 0)
    controller.update_joint_state(_state(0.8, velocity=0.0), 80)
    controller.update_joint_state(_state(0.8, velocity=0.0), 150)

    succeeded = controller.tick(150)

    assert succeeded is not None and succeeded.result is not None
    assert succeeded.result.error_code == "SUCCESS"
    assert succeeded.result.reached_goal
    assert not succeeded.result.stalled
    assert succeeded.command is not None and succeeded.command.position == [0.8]


def test_state_update_after_stale_gap_does_not_erase_stale_terminal() -> None:
    controller = GripperActionController(
        _config(
            feedback_timeout_ns=100,
            stall_timeout_ns=1000,
            goal_timeout_ns=500,
        )
    )
    controller.update_joint_state(_state(0.0, velocity=1.0), 0)
    controller.accept_goal("goal", _goal(), 0)
    controller.update_joint_state(_state(0.4, velocity=1.0), 150)

    stale = controller.tick(150)

    assert stale is not None and stale.result is not None
    assert stale.result.error_code == "FEEDBACK_STALE"
    assert stale.result.position == pytest.approx(0.4)
    assert stale.command is not None and stale.command.position == [0.4]


def test_cancel_after_earlier_goal_timeout_emits_timeout_only_once() -> None:
    controller = GripperActionController(
        _config(
            feedback_timeout_ns=1000,
            stall_timeout_ns=1000,
            goal_timeout_ns=100,
        )
    )
    controller.update_joint_state(_state(0.25, velocity=1.0), 0)
    controller.accept_goal("goal", _goal(), 0)

    timed_out = controller.cancel_goal("goal", 150)

    assert timed_out is not None and timed_out.result is not None
    assert timed_out.result.error_code == "EXECUTION_TIMED_OUT"
    assert timed_out.goal_status == "aborted"
    assert timed_out.command is not None and timed_out.command.position == [0.25]
    assert controller.cancel_goal("goal", 151) is None
    hold = controller.tick(152)
    assert hold is not None and hold.result is None
