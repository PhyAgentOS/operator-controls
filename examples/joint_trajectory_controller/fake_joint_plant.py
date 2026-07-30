#!/usr/bin/env python3
"""Minimal position-controlled joint plant for Dora integration smoke tests."""

from __future__ import annotations

import argparse
import math

from dora import Node
from forge_msgs import JointCommand, JointState
from forge_msgs.arrow import ensure_record_batch


def _plant_control(value: object) -> tuple[float, bool]:
    batch = ensure_record_batch(value)  # type: ignore[arg-type]
    if batch.num_rows != 1:
        raise ValueError("plant_control must contain exactly one row")
    gain = float(batch["response_gain"][0].as_py())
    publish_state = bool(batch["publish_state"][0].as_py())
    if not math.isfinite(gain) or not 0.0 <= gain <= 1.0:
        raise ValueError("response_gain must be finite and in [0, 1]")
    return gain, publish_state


def main() -> int:
    parser = argparse.ArgumentParser(description="Dora fake position-mode joint plant")
    parser.add_argument(
        "--joint-names",
        default="joint1,joint2,joint3,joint4,joint5,joint6",
        help="Comma-separated joint names",
    )
    parser.add_argument("--response-gain", type=float, default=0.45)
    parser.add_argument("--period-ms", type=float, default=20.0)
    args = parser.parse_args()

    joint_names = tuple(name.strip() for name in args.joint_names.split(",") if name.strip())
    if not joint_names or len(set(joint_names)) != len(joint_names):
        raise ValueError("--joint-names must contain unique non-empty names")
    if not math.isfinite(args.response_gain) or not 0.0 <= args.response_gain <= 1.0:
        raise ValueError("--response-gain must be finite and in [0, 1]")
    if not math.isfinite(args.period_ms) or args.period_ms <= 0.0:
        raise ValueError("--period-ms must be positive and finite")

    period_s = args.period_ms / 1_000.0
    response_gain = args.response_gain
    publish_state = True
    actual = [0.0] * len(joint_names)
    target = list(actual)
    previous = list(actual)

    node = Node()
    for event in node:
        event_type = event.get("type")
        if event_type == "STOP":
            break
        if event_type == "ERROR":
            raise RuntimeError(f"Dora node error: {event.get('error', 'unknown')}")
        if event_type != "INPUT":
            continue

        input_id = event.get("id")
        if input_id == "joint_command":
            command = JointCommand.from_arrow(event["value"])
            if tuple(command.name) != joint_names:
                raise ValueError("JointCommand names do not match the fake plant joint order")
            if command.mode != "position" or len(command.position) != len(joint_names):
                raise ValueError("fake plant requires a complete position-mode JointCommand")
            target = list(command.position)
        elif input_id == "plant_control":
            response_gain, publish_state = _plant_control(event["value"])
        elif input_id == "tick":
            for index, desired in enumerate(target):
                actual[index] += response_gain * (desired - actual[index])
            velocities = [
                (position - old_position) / period_s
                for position, old_position in zip(actual, previous, strict=True)
            ]
            previous = list(actual)
            if publish_state:
                node.send_output(
                    "joint_state",
                    JointState(
                        name=list(joint_names),
                        position=list(actual),
                        velocity=velocities,
                        effort=[],
                    ).to_arrow(),
                )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
