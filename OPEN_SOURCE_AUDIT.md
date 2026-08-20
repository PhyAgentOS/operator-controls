# Open-source release audit

Audit date: 2026-08-20

## Decision

The audited release is suitable for publication with its existing Git history,
branches, and tags. Project-owned code, configuration, static resources, and
documentation use Apache-2.0. No third-party code is vendored into this
repository; runtime dependencies resolve from public package indexes.

## Completed checks

- Public dependency resolution succeeds with `uv sync --frozen --all-groups`.
- The former local path override for `forge-msgs` was replaced with the public
  `forge-msgs==1.0.1` from PyPI.
- All 61 tests pass on Python 3.12 without physical hardware.
- `ruff check` passes on the publishable tree.
- Controller CLIs (`forge-gripper-action-controller`,
  `forge-joint-trajectory-controller`) report help correctly.
- `pip-audit` reports no known vulnerabilities in locked runtime dependencies.
- `detect-secrets` reports zero findings in the git-tracked source tree.
- A `detect-secrets` scan across all 53 historical Git blobs reports zero
  findings.
- Private repository URLs and machine-specific paths are absent from the
  current publishable tree. Historical commits retain development provenance,
  including former repository locations.
- No file approaches GitHub's 100 MiB hard limit; the largest tracked file
  (`uv.lock`) is under 110 KiB.
- GitHub Actions, Dependabot, issue, pull-request, contribution, and security
  policy files are included.

## License findings

- Project-owned Python, configuration, static resources, tests, and
  documentation: Apache-2.0.
- Runtime dependencies are not vendored and retain the licenses declared by
  their distributions:
  - `dora-rs`: MIT
  - `forge-msgs`: Apache-2.0
  - `PyYAML`: MIT
- PyInstaller is a build-only dependency under GPL-2.0-or-later with its
  special exception.

See `THIRD_PARTY_NOTICES.md` for binary-distribution cautions.

## Known limitations

- These controllers do not provide hard real-time motor control. Hardware
  firmware remains responsible for the low-level servo loop and safety limits.
- CI and this audit do not command physical hardware.
- PyInstaller binary releases require a separate artifact-level license and
  security review.
- The private vulnerability reporting URL in `.github/ISSUE_TEMPLATE/config.yml`
  targets the public `Forgelab-Robotics/adapter_controls` repository.
- This audit is an engineering review, not legal advice, penetration testing,
  or safety certification.

## Publication model

The audited current tree is published together with the repository's existing
commit graph, branches, and tags. Historical commits are retained for
traceability and may contain obsolete internal repository locations; they must
not be treated as current installation instructions.
