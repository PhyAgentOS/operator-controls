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

## Standalone executables

Run `scripts/build_pyinstaller.sh` from any directory to build these single-file artifacts:

- `dist/gripper_action_controller`
- `dist/joint_trajectory_controller`
- `dist/fake_joint_plant`

The build uses an isolated Python 3.12 environment and verifies each executable's help command.
It also checks that missing or invalid required arguments produce argparse exit code 2.
