# Forge Gripper Action Controller

A deterministic, position-only `GripperCommand` controller for one configured gripper
coordinate in Dora dataflows. It consumes fresh `JointState` observations and emits sparse,
single-joint position-mode `JointCommand` updates.

## Architecture

The package separates:

- `GripperActionController`: pure controller core with caller-supplied monotonic nanoseconds;
- `GripperActionControllerNode`: thin Dora event, Arrow, and Action metadata adapter;
- `run_dora_controller_node`: injectable Dora node runner used by the CLI and tests.

This is a soft real-time reference generator and monitor. Robot firmware remains responsible for
the low-level servo loop.

## Dora ports

Inputs:

- `tick`
- `joint_state`: `forge_msgs.JointState`
- `gripper_goal`: `forge_msgs.GripperCommandGoal`, metadata `goal_id`
- `gripper_cancel`: payload ignored, metadata `goal_id`

Outputs:

- `joint_command`: sparse `forge_msgs.JointCommand`
- `gripper_feedback`: `forge_msgs.GripperCommandFeedback`, metadata `goal_id`
- `gripper_result`: `forge_msgs.GripperCommandResult`, metadata `goal_id` and lowercase
  `goal_status`

## Configuration

```yaml
joint_name: gripper_joint
coordinate_type: prismatic
min_position: 0.0
max_position: 0.04

control_period_ms: 20
feedback_timeout_ms: 200
goal_tolerance: 0.001
stall_velocity_threshold: 0.001
stall_timeout_ms: 1000
allow_stalling: false
goal_timeout_ms: 5000
terminal_history_size: 128
```

`coordinate_type` only defines units; it does not scale, invert, or transform values:

- `prismatic`: position in meters, velocity in meters per second, effort in Newtons;
- `revolute`: position in radians, velocity in radians per second, effort in Newton-meters.

`control_period_ms` documents the expected external Dora tick period. The dataflow timer remains
the source of ticks and must be configured consistently.

## Version 0.1 behavior

- exactly one configured joint is controlled; incoming state may contain extra joints in any order;
- the goal position must be inside the inclusive configured limits and is never silently clipped;
- fresh position state is required at admission and throughout execution;
- observed velocity is used when supplied, otherwise it is estimated from consecutive positions;
- low velocity outside goal tolerance for `stall_timeout_ms` is a stall;
- a stall aborts unless `allow_stalling` is true, in which case it succeeds with
  `stalled=true` and `reached_goal=false`;
- cancel, stale feedback, disallowed stall, and goal timeout publish the latest observed position
  as a hold command; normal or allowed-stall success holds the target;
- every command is sparse and contains only `joint_name` in position mode;
- terminal `goal_id` values are retained in a bounded LRU history; retries for retained IDs never
  emit a second result. `terminal_history_size` controls that bound.

### Timing and terminal precedence

Terminal results are selected by when each condition occurred, not by the order of checks in a
late tick:

| Condition | Occurrence time |
| --- | --- |
| Goal reached | Start of the current uninterrupted run of in-tolerance state observations |
| Goal timeout | Goal acceptance time plus `goal_timeout_ms` |
| Stall | Last movement while outside tolerance, or the latest tolerance departure, plus `stall_timeout_ms` |
| Stale feedback | First nanosecond for which state age is greater than `feedback_timeout_ms` |
| Cancel | Cancel receipt time |

An out-of-tolerance state clears the current goal-reached run. Therefore, a target observed on time
and still in tolerance through later updates can succeed on a delayed tick, while departure and
re-entry after the goal deadline cannot. Observing the target exactly at the goal deadline may
succeed.

Among terminal conditions known in the current event call, the earliest occurrence wins. Equal
known timestamps use this deterministic precedence: cancel, stale feedback, goal reached, goal
timeout, then stall. In particular, goal timeout wins a known tie with stall; stall wins only when
its known deadline is strictly earlier. A cancel received after an already known timeout, stale
condition, stall, or continuously held goal reach reports that earlier condition rather than
cancellation.

Precedence cannot inspect future events. For Dora inputs with the same timestamp, input arrival order
is authoritative: a terminal tick or cancel processed before a same-timestamp state update cannot be
revised by that later state; when the state arrives first, its observation is available to the
terminal call.

Before replacing a state observation, the core latches any stale gap or elapsed out-of-tolerance
stall deadline from the preceding interval. Stall time does not accrue during an in-tolerance run;
departing tolerance starts a new stall interval at the departure observation. A later fresh or
moving update cannot erase valid prior terminal evidence. Result fields and abort/cancel hold
commands still use the latest accepted state, including a state received after a latched stale gap.
Processing a retained terminal `goal_id` remains silent and never emits a second result.

This release is strictly position-only. Any non-null `max_velocity`, including `0.0`, is rejected
with `UNSUPPORTED_VELOCITY`; any non-null `max_effort` is rejected with `UNSUPPORTED_EFFORT`.
Neither field is copied into `JointCommand.velocity` or `JointCommand.effort`. Future support
requires a dedicated limit interface and an explicit downstream driver contract; silently treating
limits as low-level velocity or effort commands would be incorrect.

## Run

```bash
forge-gripper-action-controller --config gripper.yaml
```
