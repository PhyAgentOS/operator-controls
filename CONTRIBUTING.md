# Contributing

Thank you for contributing to Forge Controls.

## Development

Use Python 3.12 and `uv`:

```bash
uv sync --frozen --all-groups
uv run pytest -q
uv run forge-gripper-action-controller --help
uv run forge-joint-trajectory-controller --help
```

Optional standalone executables can be built with
`scripts/build_pyinstaller.sh`.

Keep changes focused and add tests for behavior changes. Keep the deterministic
controller core separate from its Dora transport adapter, and document any
change to controller behavior, configuration schema, or message handling.

## Security and generated data

- Report vulnerabilities privately as described in `SECURITY.md`.
- Never commit credentials, private repository URLs, recordings, runtime state,
  machine-specific paths, or personal data.
- Do not run tests that command physical hardware without independent safety
  controls and an operator-accessible emergency stop.

By submitting a contribution, you agree that it is licensed under Apache
License 2.0.
