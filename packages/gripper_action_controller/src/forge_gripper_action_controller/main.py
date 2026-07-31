from __future__ import annotations

import argparse

from .config import load_config
from .node import run_dora_controller_node


def main() -> int:
    parser = argparse.ArgumentParser(description="Forge gripper action controller Dora node")
    parser.add_argument("--config", required=True, help="Path to the controller YAML config")
    args = parser.parse_args()
    return run_dora_controller_node(load_config(args.config))


if __name__ == "__main__":
    raise SystemExit(main())
