# Forge Joint Trajectory Controller

A deterministic, position-mode subset of ROS 2's `joint_trajectory_controller` for Dora
dataflows. It consumes Forge `FollowJointTrajectory` action payloads and fresh `JointState`
feedback, then emits complete position-mode `JointCommand` references.

## Architecture

The package separates:

- `JointTrajectoryController`: pure controller core with an injected monotonic nanosecond clock;
- `JointTrajectoryControllerNode`: thin Dora event/metadata adapter;
- `interpolation`: position-only linear and fully velocity-specified cubic Hermite sampling.

The controller owns one configured, complete joint group. It is the only writer to the robot
driver while active or holding.

## Dora ports

Inputs:

- `tick`
- `joint_state`: `forge_msgs.JointState`
- `trajectory_goal`: `forge_msgs.FollowJointTrajectoryGoal`, metadata `goal_id`
- `trajectory_cancel`: empty payload, metadata `goal_id`

Outputs:

- `joint_command`: `forge_msgs.JointCommand`
- `trajectory_feedback`: `forge_msgs.FollowJointTrajectoryFeedback`, metadata `goal_id`
- `trajectory_result`: `forge_msgs.FollowJointTrajectoryResult`, metadata `goal_id` and lowercase
  `goal_status`

## Version 0.1 behavior

- exact configured trajectory joint order; partial trajectories are rejected;
- fresh complete feedback is required, but incoming `JointState` may include extra joints and use
  a different order;
- position trajectories use linear interpolation;
- trajectories with complete velocities on every point use cubic Hermite interpolation;
- mixed velocity specification, accelerations, effort, path velocity tolerance, and acceleration
  tolerances are rejected rather than silently ignored;
- path position and goal position/velocity tolerances are monitored;
- cancel, stale feedback, and tolerance violations actively hold the latest actual positions;
- successful goals hold the final desired positions;
- terminal results are cached with a bounded LRU for idempotent retries.

This is a soft real-time trajectory reference generator and monitor. The robot firmware remains
responsible for the hard real-time motor servo loop.

## Run

```bash
forge-joint-trajectory-controller --config piper_arm.yaml
```

See `controls/examples/joint_trajectory_controller/` for a Piper configuration and dataflow
example.
