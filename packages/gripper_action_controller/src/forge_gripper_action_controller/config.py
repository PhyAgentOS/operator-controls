from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal

import yaml

CoordinateType = Literal["prismatic", "revolute"]


@dataclass(frozen=True, slots=True)
class ControllerConfig:
    """Configuration for one position-controlled gripper coordinate."""

    joint_name: str
    coordinate_type: CoordinateType
    min_position: float
    max_position: float
    control_period_ns: int = 20_000_000
    feedback_timeout_ns: int = 200_000_000
    goal_tolerance: float = 0.001
    stall_velocity_threshold: float = 0.001
    stall_timeout_ns: int = 1_000_000_000
    allow_stalling: bool = False
    goal_timeout_ns: int = 5_000_000_000
    terminal_history_size: int = 128

    def __post_init__(self) -> None:
        if not isinstance(self.joint_name, str) or not self.joint_name:
            raise ValueError("joint_name must be a non-empty string")
        if self.coordinate_type not in {"prismatic", "revolute"}:
            raise ValueError("coordinate_type must be 'prismatic' or 'revolute'")

        for name in (
            "control_period_ns",
            "feedback_timeout_ns",
            "stall_timeout_ns",
            "goal_timeout_ns",
        ):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int):
                raise ValueError(f"{name} must be an integer number of nanoseconds")
            if value <= 0:
                raise ValueError(f"{name} must be greater than zero")

        for name in ("min_position", "max_position"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int | float):
                raise ValueError(f"{name} must be a number")
            if not math.isfinite(float(value)):
                raise ValueError(f"{name} must be finite")
        if self.min_position > self.max_position:
            raise ValueError("min_position must not be greater than max_position")

        for name in ("goal_tolerance", "stall_velocity_threshold"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int | float):
                raise ValueError(f"{name} must be a number")
            if not math.isfinite(float(value)) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")

        if not isinstance(self.allow_stalling, bool):
            raise ValueError("allow_stalling must be a boolean")
        if (
            isinstance(self.terminal_history_size, bool)
            or not isinstance(self.terminal_history_size, int)
            or self.terminal_history_size <= 0
        ):
            raise ValueError("terminal_history_size must be a positive integer")


def load_config(path: str | Path) -> ControllerConfig:
    """Load a human-readable YAML controller configuration."""

    config_path = Path(path)
    data = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("controller config must be a YAML mapping")
    return controller_config_from_mapping(data)


def controller_config_from_mapping(data: dict[str, Any]) -> ControllerConfig:
    allowed = {
        "joint_name",
        "coordinate_type",
        "min_position",
        "max_position",
        "control_period_ms",
        "feedback_timeout_ms",
        "goal_tolerance",
        "stall_velocity_threshold",
        "stall_timeout_ms",
        "allow_stalling",
        "goal_timeout_ms",
        "terminal_history_size",
    }
    unknown = sorted(set(data) - allowed)
    if unknown:
        raise ValueError(f"unknown controller config fields: {unknown}")

    joint_name = data.get("joint_name")
    if not isinstance(joint_name, str):
        raise ValueError("joint_name must be a string")
    coordinate_type = data.get("coordinate_type")
    if coordinate_type not in {"prismatic", "revolute"}:
        raise ValueError("coordinate_type must be 'prismatic' or 'revolute'")

    allow_stalling = data.get("allow_stalling", False)
    if not isinstance(allow_stalling, bool):
        raise ValueError("allow_stalling must be a boolean")

    return ControllerConfig(
        joint_name=joint_name,
        coordinate_type=coordinate_type,
        min_position=_required_number(data, "min_position"),
        max_position=_required_number(data, "max_position"),
        control_period_ns=_milliseconds_to_ns(
            data.get("control_period_ms", 20), "control_period_ms"
        ),
        feedback_timeout_ns=_milliseconds_to_ns(
            data.get("feedback_timeout_ms", 200), "feedback_timeout_ms"
        ),
        goal_tolerance=_number(data.get("goal_tolerance", 0.001), "goal_tolerance"),
        stall_velocity_threshold=_number(
            data.get("stall_velocity_threshold", 0.001),
            "stall_velocity_threshold",
        ),
        stall_timeout_ns=_milliseconds_to_ns(
            data.get("stall_timeout_ms", 1000), "stall_timeout_ms"
        ),
        allow_stalling=allow_stalling,
        goal_timeout_ns=_milliseconds_to_ns(data.get("goal_timeout_ms", 5000), "goal_timeout_ms"),
        terminal_history_size=_positive_int(
            data.get("terminal_history_size", 128), "terminal_history_size"
        ),
    )


def _required_number(data: dict[str, Any], name: str) -> float:
    if name not in data:
        raise ValueError(f"{name} is required")
    return _number(data[name], name)


def _number(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise ValueError(f"{name} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _positive_int(value: Any, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"{name} must be a positive integer")
    return value


def _milliseconds_to_ns(value: Any, name: str) -> int:
    milliseconds = _number(value, name)
    if milliseconds <= 0.0:
        raise ValueError(f"{name} must be greater than zero")
    return int(round(milliseconds * 1_000_000))
