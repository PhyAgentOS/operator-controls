# Joint Trajectory Controller Example

`piper_arm.yaml` configures a complete six-joint Piper arm group at a 20 ms Dora tick.
`dataflow.example.yaml` shows the intended boundary:

```text
motion_server --FollowJointTrajectory--> joint_trajectory_controller
joint_trajectory_controller --JointCommand--> robot_piper
robot_piper --JointState--> joint_trajectory_controller
```

A goal and every feedback/result event are correlated through Dora `goal_id` metadata. Terminal
results additionally carry lowercase `goal_status` metadata.

## Dora fake-plant smoke

`dataflow.smoke.yaml` replaces the real Piper driver with a deliberately small position-mode plant:

```text
trajectory_action_client -> joint_trajectory_controller -> fake_joint_plant
          ^                         |                         |
          |                         +-- feedback/result -----+
          +------------------------- joint state/command ----+
```

The plant follows each position reference with:

```python
actual += response_gain * (command - actual)
```

The action client runs these checks in separate Dora processes:

- normal execution reaches `SUCCESS`;
- cancel emits `CANCELED` and actively holds the observed position;
- stopped state publication produces `FEEDBACK_STALE`;
- a frozen plant produces `PATH_TOLERANCE_VIOLATED`;
- goal B receives `BUSY` while active goal A continues to success;
- an active duplicate goal does not restart execution;
- retrying a terminal goal returns the cached result;
- Forge Arrow payloads and Dora `goal_id`/`goal_status` metadata survive real transport;
- each execution emits one terminal result (apart from the intentional cached-result retry).

From the `controls` repository root, prepare the environment and run:

```bash
uv sync --locked
dora run examples/joint_trajectory_controller/dataflow.smoke.yaml
```

The direct dataflow prints the client verdict; stop the remaining timer-driven example nodes with
`Ctrl+C`. For a machine-checked run that applies a timeout and cleans up the full process group:

```bash
uv run python examples/joint_trajectory_controller/smoke_test.py
```

The runner accepts `--dora /path/to/dora`, or the same path through `DORA_CLI`, when the CLI is not
on `PATH`. The Dora CLI should be compatible with the package's `dora-rs==0.4.1` dependency.
`smoke-result.json` is generated beside the dataflow and is intentionally not committed.
