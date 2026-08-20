## Summary

-

## Validation

- [ ] `uv sync --frozen --all-groups` succeeds.
- [ ] `uv run pytest -q` passes.
- [ ] Controller CLI help checks pass.
- [ ] Locked runtime dependencies pass `pip-audit`.
- [ ] No secrets, recordings, machine paths, or private repository URLs are included.
- [ ] License notices are updated when dependencies or bundled resources change.

## Deployment and safety

- [ ] Controller behavior changes keep the deterministic core separate from the
      Dora transport adapter and are covered by tests.
- [ ] Any physical robot validation used independent safety controls and an
      operator-accessible emergency stop.

Describe deployment and physical validation performed:
