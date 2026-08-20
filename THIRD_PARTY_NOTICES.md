# Third-party notices

The repository's original Python code, configuration, static resources, and
documentation are licensed under Apache License 2.0 unless a file states
otherwise.

## Runtime dependencies

Runtime dependencies are installed from package indexes and are not vendored
into this source repository. They retain their respective licenses:

- `dora-rs`: MIT
- `forge-msgs`: Apache-2.0
- `PyYAML`: MIT

Transitive dependencies retain the licenses declared by their distributions.
Consult the locked environment and installed package metadata for the complete
dependency graph.

## Build dependencies and binary releases

PyInstaller is used only to produce optional standalone executables and is not
vendored into this repository. A bundled executable contains runtime
dependencies and may create license, notice, source-offer, codec, or platform
obligations beyond source distribution. Review the complete bundled artifact
and all dependency licenses before publishing a binary release.
