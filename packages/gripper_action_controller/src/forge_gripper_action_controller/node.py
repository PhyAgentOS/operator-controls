from __future__ import annotations

import logging
import time
from collections.abc import Callable, Iterator
from typing import Any, Protocol, cast

from forge_msgs import GripperCommandGoal, GripperCommandResult, JointState

from .config import ControllerConfig
from .controller import ControllerOutput, GripperActionController

logger = logging.getLogger(__name__)


class DoraNodeLike(Protocol):
    def __iter__(self) -> Iterator[dict[str, Any]]: ...

    def send_output(self, output_id: str, value: Any, **kwargs: Any) -> None: ...


class GripperActionControllerNode:
    """Thin Dora transport adapter around the deterministic controller core."""

    def __init__(self, controller: GripperActionController) -> None:
        self.controller = controller

    def handle_event(
        self,
        node: DoraNodeLike,
        event: dict[str, Any],
        now_ns: int,
    ) -> bool:
        """Handle one event. Return False when the node loop should stop."""

        event_type = event.get("type")
        if event_type == "STOP":
            return False
        if event_type == "ERROR":
            logger.error("Dora node error: %s", event.get("error", "unknown"))
            return False
        if event_type != "INPUT":
            return True

        input_id = event.get("id")
        output: ControllerOutput | None = None
        try:
            if input_id == "joint_state":
                self.controller.update_joint_state(JointState.from_arrow(event["value"]), now_ns)
            elif input_id == "gripper_goal":
                output = self._handle_goal(event, now_ns)
            elif input_id == "gripper_cancel":
                output = self._handle_cancel(event, now_ns)
            elif input_id == "tick":
                output = self.controller.tick(now_ns)
        except (KeyError, TypeError, ValueError) as error:
            logger.warning("Ignoring invalid %s input: %s", input_id, error)
            return True

        self._emit(node, output)
        return True

    def _handle_goal(
        self,
        event: dict[str, Any],
        now_ns: int,
    ) -> ControllerOutput | None:
        goal_id = _goal_id(event)
        if goal_id is None:
            logger.warning("Ignoring gripper_goal without goal_id metadata")
            return None
        try:
            goal = GripperCommandGoal.from_arrow(event["value"])
        except Exception as error:
            return self.controller.reject_invalid_goal(
                goal_id,
                f"failed to decode gripper goal: {error}",
                now_ns,
            )
        return self.controller.accept_goal(goal_id, goal, now_ns)

    def _handle_cancel(
        self,
        event: dict[str, Any],
        now_ns: int,
    ) -> ControllerOutput | None:
        goal_id = _goal_id(event)
        if goal_id is None:
            logger.warning("Ignoring gripper_cancel without goal_id metadata")
            return None
        return self.controller.cancel_goal(goal_id, now_ns)

    def _emit(self, node: DoraNodeLike, output: ControllerOutput | None) -> None:
        if output is None:
            return
        if output.command is not None:
            node.send_output("joint_command", output.command.to_arrow())
        if output.feedback is not None:
            if output.goal_id is None:
                raise ValueError("feedback output requires goal_id")
            node.send_output(
                "gripper_feedback",
                output.feedback.to_arrow(),
                metadata=_action_metadata(output.goal_id),
            )
        if output.result is not None:
            if output.goal_id is None or output.goal_status is None:
                raise ValueError("result output requires goal_id and goal_status")
            self._send_result(node, output.goal_id, output.result, output.goal_status)

    @staticmethod
    def _send_result(
        node: DoraNodeLike,
        goal_id: str,
        result: GripperCommandResult,
        goal_status: str,
    ) -> None:
        node.send_output(
            "gripper_result",
            result.to_arrow(),
            metadata=_action_metadata(goal_id, goal_status),
        )


def run_dora_controller_node(
    config: ControllerConfig,
    *,
    node_factory: Callable[[], DoraNodeLike] | None = None,
    clock: Callable[[], int] = time.monotonic_ns,
) -> int:
    factory = node_factory
    if factory is None:
        from dora import Node

        factory = cast(Callable[[], DoraNodeLike], Node)
    node = factory()
    adapter = GripperActionControllerNode(GripperActionController(config))
    for event in node:
        if not adapter.handle_event(node, event, clock()):
            break
    return 0


def _goal_id(event: dict[str, Any]) -> str | None:
    metadata = event.get("metadata")
    if not isinstance(metadata, dict):
        return None
    parameters = metadata.get("parameters")
    source = parameters if isinstance(parameters, dict) else metadata
    value = source.get("goal_id")
    if value is None:
        return None
    goal_id = str(value)
    return goal_id if goal_id else None


def _action_metadata(goal_id: str, goal_status: str | None = None) -> dict[str, Any]:
    metadata: dict[str, Any] = {"goal_id": goal_id}
    if goal_status is not None:
        if goal_status not in {"succeeded", "aborted", "canceled"}:
            raise ValueError(f"invalid Dora goal_status: {goal_status}")
        metadata["goal_status"] = goal_status
    return metadata
