from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


@dataclass(frozen=True, slots=True)
class ControllerConfig:
    """Configuration for one fixed, complete joint group."""

    joint_names: tuple[str, ...]
    control_period_ns: int = 20_000_000
    feedback_timeout_ns: int = 200_000_000
    start_position_tolerance: float = 0.1
    path_position_tolerance: float = 0.15
    goal_position_tolerance: float = 0.03
    goal_velocity_tolerance: float = 0.05
    goal_time_tolerance_ns: int = 500_000_000
    result_cache_size: int = 128

    def __post_init__(self) -> None:
        if not isinstance(self.joint_names, tuple):
            raise ValueError("joint_names must be a tuple")
        if not self.joint_names:
            raise ValueError("joint_names must contain at least one joint")
        if any(not isinstance(name, str) or not name for name in self.joint_names):
            raise ValueError("joint_names items must be non-empty strings")
        if len(set(self.joint_names)) != len(self.joint_names):
            raise ValueError("joint_names items must be unique")
        for name in (
            "control_period_ns",
            "feedback_timeout_ns",
            "goal_time_tolerance_ns",
        ):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int):
                raise ValueError(f"{name} must be an integer number of nanoseconds")
            if value < 0:
                raise ValueError(f"{name} must be non-negative")
        if self.control_period_ns == 0:
            raise ValueError("control_period_ns must be greater than zero")
        if (
            isinstance(self.result_cache_size, bool)
            or not isinstance(self.result_cache_size, int)
            or self.result_cache_size <= 0
        ):
            raise ValueError("result_cache_size must be a positive integer")
        for name in (
            "start_position_tolerance",
            "path_position_tolerance",
            "goal_position_tolerance",
            "goal_velocity_tolerance",
        ):
            value = getattr(self, name)
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")


def load_config(path: str | Path) -> ControllerConfig:
    """Load a human-readable YAML controller configuration."""

    config_path = Path(path)
    data = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("controller config must be a YAML mapping")
    return controller_config_from_mapping(data)


def controller_config_from_mapping(data: dict[str, Any]) -> ControllerConfig:
    allowed = {
        "joint_names",
        "control_period_ms",
        "feedback_timeout_ms",
        "start_position_tolerance",
        "path_position_tolerance",
        "goal_position_tolerance",
        "goal_velocity_tolerance",
        "goal_time_tolerance_ms",
        "result_cache_size",
    }
    unknown = sorted(set(data) - allowed)
    if unknown:
        raise ValueError(f"unknown controller config fields: {unknown}")
    joint_names = data.get("joint_names")
    if not isinstance(joint_names, list) or any(not isinstance(name, str) for name in joint_names):
        raise ValueError("joint_names must be a list of strings")

    return ControllerConfig(
        joint_names=tuple(joint_names),
        control_period_ns=_milliseconds_to_ns(
            data.get("control_period_ms", 20), "control_period_ms"
        ),
        feedback_timeout_ns=_milliseconds_to_ns(
            data.get("feedback_timeout_ms", 200), "feedback_timeout_ms"
        ),
        start_position_tolerance=float(data.get("start_position_tolerance", 0.1)),
        path_position_tolerance=float(data.get("path_position_tolerance", 0.15)),
        goal_position_tolerance=float(data.get("goal_position_tolerance", 0.03)),
        goal_velocity_tolerance=float(data.get("goal_velocity_tolerance", 0.05)),
        goal_time_tolerance_ns=_milliseconds_to_ns(
            data.get("goal_time_tolerance_ms", 500), "goal_time_tolerance_ms"
        ),
        result_cache_size=_positive_int(data.get("result_cache_size", 128), "result_cache_size"),
    )


def _positive_int(value: Any, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"{name} must be a positive integer")
    return value


def _milliseconds_to_ns(value: Any, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise ValueError(f"{name} must be a number")
    if not math.isfinite(float(value)) or value < 0:
        raise ValueError(f"{name} must be finite and non-negative")
    return int(round(float(value) * 1_000_000))
