"""PyInstaller entry point preserving package-relative imports."""

from forge_joint_trajectory_controller.main import main

if __name__ == "__main__":
    raise SystemExit(main())
