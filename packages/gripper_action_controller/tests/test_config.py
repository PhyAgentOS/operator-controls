from __future__ import annotations

import math
from pathlib import Path

import pytest
from forge_gripper_action_controller.config import (
    ControllerConfig,
    controller_config_from_mapping,
    load_config,
)


def test_config_mapping_and_yaml_use_milliseconds(tmp_path: Path) -> None:
    config_path = tmp_path / "gripper.yaml"
    config_path.write_text(
        """
joint_name: gripper_joint
coordinate_type: prismatic
min_position: 0.0
max_position: 0.04
control_period_ms: 10
feedback_timeout_ms: 150
goal_tolerance: 0.0005
stall_velocity_threshold: 0.0002
stall_timeout_ms: 750
allow_stalling: true
goal_timeout_ms: 4000
terminal_history_size: 32
""".strip(),
        encoding="utf-8",
    )

    config = load_config(config_path)

    assert config.joint_name == "gripper_joint"
    assert config.coordinate_type == "prismatic"
    assert config.control_period_ns == 10_000_000
    assert config.feedback_timeout_ns == 150_000_000
    assert config.stall_timeout_ns == 750_000_000
    assert config.goal_timeout_ns == 4_000_000_000
    assert config.terminal_history_size == 32
    assert config.allow_stalling


def test_config_defaults_and_revolute_coordinate() -> None:
    config = controller_config_from_mapping(
        {
            "joint_name": "finger_hinge",
            "coordinate_type": "revolute",
            "min_position": -0.5,
            "max_position": 0.8,
        }
    )

    assert config.coordinate_type == "revolute"
    assert config.control_period_ns == 20_000_000
    assert config.feedback_timeout_ns == 200_000_000
    assert config.goal_tolerance == pytest.approx(0.001)
    assert config.stall_velocity_threshold == pytest.approx(0.001)
    assert config.stall_timeout_ns == 1_000_000_000
    assert config.goal_timeout_ns == 5_000_000_000
    assert config.terminal_history_size == 128
    assert not config.allow_stalling


def test_config_rejects_unknown_missing_and_invalid_fields() -> None:
    base = {
        "joint_name": "gripper",
        "coordinate_type": "prismatic",
        "min_position": 0.0,
        "max_position": 0.1,
    }

    with pytest.raises(ValueError, match="unknown"):
        controller_config_from_mapping({**base, "unexpected": True})
    with pytest.raises(ValueError, match="min_position is required"):
        controller_config_from_mapping(
            {
                "joint_name": "gripper",
                "coordinate_type": "prismatic",
                "max_position": 0.1,
            }
        )
    with pytest.raises(ValueError, match="coordinate_type"):
        controller_config_from_mapping({**base, "coordinate_type": "linear"})
    with pytest.raises(ValueError, match="allow_stalling"):
        controller_config_from_mapping({**base, "allow_stalling": 1})
    with pytest.raises(ValueError, match="greater than zero"):
        controller_config_from_mapping({**base, "goal_timeout_ms": 0})
    with pytest.raises(ValueError, match="number"):
        controller_config_from_mapping({**base, "feedback_timeout_ms": True})
    with pytest.raises(ValueError, match="positive integer"):
        controller_config_from_mapping({**base, "terminal_history_size": True})


def test_dataclass_rejects_invalid_limits_numbers_and_time() -> None:
    with pytest.raises(ValueError, match="min_position"):
        ControllerConfig(
            joint_name="gripper",
            coordinate_type="prismatic",
            min_position=1.0,
            max_position=0.0,
        )
    with pytest.raises(ValueError, match="finite"):
        ControllerConfig(
            joint_name="gripper",
            coordinate_type="prismatic",
            min_position=0.0,
            max_position=math.inf,
        )
    with pytest.raises(ValueError, match="goal_tolerance"):
        ControllerConfig(
            joint_name="gripper",
            coordinate_type="prismatic",
            min_position=0.0,
            max_position=1.0,
            goal_tolerance=-0.1,
        )
    with pytest.raises(ValueError, match="stall_timeout_ns"):
        ControllerConfig(
            joint_name="gripper",
            coordinate_type="prismatic",
            min_position=0.0,
            max_position=1.0,
            stall_timeout_ns=0,
        )
    with pytest.raises(ValueError, match="terminal_history_size"):
        ControllerConfig(
            joint_name="gripper",
            coordinate_type="prismatic",
            min_position=0.0,
            max_position=1.0,
            terminal_history_size=0,
        )
