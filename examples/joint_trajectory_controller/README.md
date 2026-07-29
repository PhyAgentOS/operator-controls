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
