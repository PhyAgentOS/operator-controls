# Forge Controls

Reusable motion execution and control capabilities for Forge/Dora robotics dataflows.

The workspace provides lightweight motion controllers that consume Dora action payloads from
`forge_msgs`, monitor fresh `JointState` feedback, and emit position-mode `JointCommand`
references.

Packages:

- `packages/gripper_action_controller`: position-only single-joint gripper action controller.
- `packages/joint_trajectory_controller`: position-mode trajectory controller for fixed joint groups.

Each package separates a deterministic controller core from its thin Dora transport adapter and
includes focused unit and fake-plant tests.

This workspace does not provide hard real-time motor control. Hardware firmware remains
responsible for the low-level servo loop.
