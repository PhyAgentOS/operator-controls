# Forge Controls

Reusable motion execution and control capabilities for Forge/Dora robotics dataflows.

The first package is a lightweight, position-mode subset of ROS 2's
`joint_trajectory_controller`. It consumes the Dora `FollowJointTrajectory`
action payloads from `forge_msgs`, samples trajectories against fresh
`JointState` feedback, and emits `JointCommand` references.

Packages:

- `packages/joint_trajectory_controller`: deterministic controller core and thin Dora adapter.

This workspace does not provide hard real-time motor control. Hardware firmware remains
responsible for the low-level servo loop.
