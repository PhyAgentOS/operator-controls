# Forge Controls

[English](README.md) | [简体中文](README_zh.md)

## BREAKING CHANGE: Forge 2.x / Dora 1.x migration

This migration from Forge 1.x / Dora 0.4.x requires coordinated deployment. Align the Forge packages, Dora CLI/coordinator/daemon, this node and its peer nodes before connecting a dataflow; replacing only one node in a Forge 1.x deployment is unsupported. Rebuild and redeploy executable artifacts with the matching environment. Keep `stable/1.x` for deployments that still use the previous stack.

The validated Python baseline is `forge-msgs==2.1.0`, `Pillow==13.0.0`, `forge-common==2.1.0`, the applicable `forge-robot` / `forge-policy` / `forge-kinematics` / `forge-tool` packages at `2.0.0`, and `dora-rs==1.0.1`. Respect this repository's Python and SDK platform requirements. Forge package families have independent versions. Rust nodes retain the published `forge_msgs` crate `2.0.0` with Dora `1.0.1`; Python Msgs `2.1.0` does not require a nonexistent Rust `2.1.0` release. The `forge_msgs.v1` message namespace remains unchanged.

This notice describes the deployment break from the previous stack; it does not declare hardware, real-model or sustained-run acceptance complete. See [this round's verification report](migration/msgs-2.1.0-report.json) for actual tests and outstanding gates.

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

## License

[Apache License 2.0](LICENSE). Runtime dependencies are not distributed with
this source repository and retain their respective licenses; see
[NOTICE](NOTICE) and [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
Review the complete bundled artifact before publishing standalone executables.

## Forge 2.x migration

This branch prepares node version `2.0.0` for Dora 1.x, frozen to Dora `1.0.1` for migration validation. It uses the official Forge 2.x distributions; Common is at least `2.1.0`. Regenerated locks and all executable entrypoints are part of the change.

Every change report must list repository, changes and test evidence, including per-node `dora run` records and a chained test containing `adapter-forge-gateway`. Unavailable hardware/model checks remain pending. Source PRs do not publish stable releases or synchronize GitLab; synchronization follows full acceptance.
