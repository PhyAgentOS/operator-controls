# -*- mode: python ; coding: utf-8 -*-
"""Build the joint trajectory controller and fake joint plant as one-file executables."""

from pathlib import Path

from PyInstaller.utils.hooks import collect_all, collect_submodules


def collect_package(package, datas, binaries, hiddenimports):
    package_datas, package_binaries, package_hiddenimports = collect_all(package)
    datas.extend(package_datas)
    binaries.extend(package_binaries)
    hiddenimports.extend(package_hiddenimports)


spec_dir = Path(SPEC).resolve().parent
root_dir = spec_dir.parent
controller_src = root_dir / "packages" / "joint_trajectory_controller" / "src"

datas = []
binaries = []
hiddenimports = ["_yaml", "pyarrow.lib", "yaml"]

for dependency in ("dora", "forge_msgs", "numpy", "pyarrow", "yaml"):
    collect_package(dependency, datas, binaries, hiddenimports)

hiddenimports.extend(collect_submodules("forge_joint_trajectory_controller"))
hiddenimports = sorted(set(hiddenimports))


def build_executable(entry, name, pathex):
    analysis = Analysis(
        [str(entry)],
        pathex=[str(path) for path in pathex],
        binaries=binaries,
        datas=datas,
        hiddenimports=hiddenimports,
        hookspath=[],
        hooksconfig={},
        runtime_hooks=[],
        excludes=[],
        noarchive=False,
        optimize=0,
    )
    pyz = PYZ(analysis.pure)
    return EXE(
        pyz,
        analysis.scripts,
        analysis.binaries,
        analysis.datas,
        [],
        name=name,
        debug=False,
        bootloader_ignore_signals=False,
        strip=False,
        upx=False,
        console=True,
        disable_windowed_traceback=False,
        argv_emulation=False,
        target_arch=None,
        codesign_identity=None,
        entitlements_file=None,
    )


controller = build_executable(
    spec_dir / "joint_trajectory_controller_entry.py",
    "joint_trajectory_controller",
    [controller_src],
)
fake_plant = build_executable(
    root_dir / "examples" / "joint_trajectory_controller" / "fake_joint_plant.py",
    "fake_joint_plant",
    [controller_src, root_dir / "examples" / "joint_trajectory_controller"],
)
