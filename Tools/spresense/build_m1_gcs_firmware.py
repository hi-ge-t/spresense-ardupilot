#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

"""Build and verify the output-disabled Spresense M1 GCS firmware."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

from mavlink_headers import MAVLINK_COMMIT, generate


SDK_COMMIT = "7fd61b2c03f06a4ff0302b84c755e58c338788b2"
PROFILE = "spresense-m1-gcs"
REQUIRED_CONFIG = {
    "CONFIG_SPRESENSE_M1_GCS=y",
    'CONFIG_SPRESENSE_M1_GCS_DEVICE="/dev/ttyS0"',
    "CONFIG_SPRESENSE_M1_GCS_BAUD=115200",
    "# CONFIG_CXD56_GNSS is not set",
    "CONFIG_CXD56_GNSS_ADDON=y",
    "CONFIG_SENSORS_CXD5610_GNSS=y",
    "CONFIG_SPECIFIC_DRIVERS=y",
    "CONFIG_CXD56_GNSS_RAM=y",
    "CONFIG_CXD56_GNSS_HEAP=y",
    "CONFIG_CXD56_I2C0=y",
    "# CONFIG_CXD56_I2C0_SCUSEQ is not set",
    "CONFIG_CXD56_UART1=y",
    "CONFIG_UART1_SERIALDRIVER=y",
    "# CONFIG_UART1_SERIAL_CONSOLE is not set",
    "CONFIG_UART2_SERIAL_CONSOLE=y",
    "# CONFIG_SYSTEM_NSH is not set",
    "# CONFIG_SYSTEM_CDCACM is not set",
    "# CONFIG_SYSTEM_USBMSC is not set",
    "# CONFIG_CXD56_PWM is not set",
    "# CONFIG_PWM is not set",
    'CONFIG_INIT_ENTRYPOINT="spresense_main"',
    "CONFIG_INIT_PRIORITY=180",
    "CONFIG_INIT_STACKSIZE=32768",
}


def run(command, *, cwd: Path, env: dict[str, str], capture=False):
    return subprocess.run(
        [str(value) for value in command],
        cwd=cwd,
        env=env,
        check=True,
        capture_output=capture,
        text=capture,
    )


def git_value(path: Path, *arguments: str) -> str:
    return subprocess.run(
        ["git", "-C", str(path), *arguments],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def verify_config(config: Path) -> None:
    lines = set(config.read_text(encoding="utf-8").splitlines())
    missing = sorted(REQUIRED_CONFIG - lines)
    if missing:
        raise RuntimeError("configuration contract: " + " | ".join(missing))


def generate_kconfig(root: Path, sdk_root: Path, env: dict[str, str]) -> None:
    sdk = sdk_root / "sdk"
    mkkconfig = sdk / "tools/mkkconfig.py"
    examples = sdk_root / "examples"
    externals = sdk_root / "externals"
    spresense_home = root / "Tools/spresense"
    run(
        [sys.executable, mkkconfig, "-m", "Examples", "-o",
         examples / "Kconfig", *sorted(examples.iterdir())],
        cwd=sdk,
        env=env,
    )
    run(
        [sys.executable, mkkconfig, "-m", "Externals", "-o",
         externals / "Kconfig", *sorted(externals.iterdir())],
        cwd=sdk,
        env=env,
    )
    run(
        [sys.executable, mkkconfig, "-m", "ArduPilot Spresense bring-up",
         "-o", spresense_home / "Kconfig",
         spresense_home / "m1_gcs_app"],
        cwd=sdk,
        env=env,
    )
    run(
        [sys.executable, mkkconfig, "-m", "Spresense SDK", "-o",
         sdk / "Kconfig", sdk / "modules", sdk / "system", examples,
         externals, spresense_home],
        cwd=sdk,
        env=env,
    )


def parser() -> argparse.ArgumentParser:
    value = argparse.ArgumentParser(description=__doc__)
    value.add_argument(
        "--allow-dirty",
        action="store_true",
        help="development build only; dirty artifacts remain non-flashable",
    )
    value.add_argument("--jobs", type=int, default=2)
    value.add_argument(
        "--reuse-config",
        action="store_true",
        help="reuse the current verified Sony SDK configuration",
    )
    return value


def main() -> int:
    arguments = parser().parse_args()
    root = Path(__file__).resolve().parents[2]
    sdk_root = root / "modules/Spresense"
    sdk = sdk_root / "sdk"
    nuttx = sdk_root / "nuttx"
    toolchain = Path.home() / "spresenseenv/usr/bin"
    compiler = toolchain / "arm-none-eabi-gcc"
    artifact_dir = root / "build/spresense-m1-gcs-artifacts"
    env = os.environ.copy()
    env["PATH"] = f"{toolchain}:{env.get('PATH', '')}"
    env["SPRESENSE_HOME"] = str(root / "Tools/spresense")
    env["GCCVER"] = "10"

    try:
        if arguments.jobs < 1 or arguments.jobs > 8:
            raise RuntimeError("--jobs must be in the range 1..8")
        if not compiler.is_file():
            raise RuntimeError(f"Sony toolchain is missing: {compiler}")
        gcc_version = run(
            [compiler, "-dumpfullversion", "-dumpversion"],
            cwd=root,
            env=env,
            capture=True,
        ).stdout.strip()
        if gcc_version != "10.3.1":
            raise RuntimeError(f"Sony GCC version mismatch: {gcc_version}")
        sdk_commit = git_value(sdk_root, "rev-parse", "HEAD")
        if sdk_commit != SDK_COMMIT:
            raise RuntimeError(f"Sony SDK commit mismatch: {sdk_commit}")
        project_commit = git_value(root, "rev-parse", "HEAD")
        dirty = bool(git_value(root, "status", "--porcelain"))
        if dirty and not arguments.allow_dirty:
            raise RuntimeError(
                "worktree is dirty; commit first or use --allow-dirty for a "
                "non-flashable development build"
            )

        if arguments.reuse_config:
            if not (root / "build/spresense-m1-generated/ardupilotmega/mavlink.h").is_file():
                raise RuntimeError("generated MAVLink headers are missing")
        else:
            generate(root)
            generate_kconfig(root, sdk_root, env)
            run(
                [sys.executable, "tools/config.py", "default",
                 "m1_gcs_app/gcs"],
                cwd=sdk,
                env=env,
            )
        verify_config(nuttx / ".config")
        libgcc = run(
            [compiler, "-mlittle-endian", "-march=armv7e-m",
             "-mtune=cortex-m4", "-mfpu=fpv4-sp-d16",
             "-mfloat-abi=hard", "-mthumb", "--print-libgcc-file-name"],
            cwd=root,
            env=env,
            capture=True,
        ).stdout.strip()
        make_args = [f"COMPILER_RT_LIB={libgcc}"]
        if not arguments.reuse_config:
            run(
                ["make", "-C", nuttx, "-j1", *make_args, "context",
                 "depend"],
                cwd=root,
                env=env,
            )
        run(
            ["make", "-C", sdk, f"-j{arguments.jobs}", *make_args],
            cwd=root,
            env=env,
        )

        sources = {
            "nuttx.spk": sdk / "nuttx.spk",
            "nuttx": sdk / "nuttx",
            "nuttx.map": nuttx / "nuttx.map",
            "nuttx.config": nuttx / ".config",
        }
        for name, source in sources.items():
            if not source.is_file():
                raise RuntimeError(f"build output is missing: {source}")
        artifact_dir.mkdir(parents=True, exist_ok=True)
        for name, source in sources.items():
            shutil.copy2(source, artifact_dir / name)

        map_report = artifact_dir / "memory-layout.json"
        run(
            [sys.executable, root / "Tools/spresense/verify_m1_gcs_map.py",
             "--map", artifact_dir / "nuttx.map", "--output", map_report],
            cwd=root,
            env=env,
        )
        nm_output = run(
            [toolchain / "arm-none-eabi-nm", artifact_dir / "nuttx"],
            cwd=root,
            env=env,
            capture=True,
        ).stdout.splitlines()
        entry = [line for line in nm_output if line.endswith(" spresense_main")]
        if len(entry) != 1 or " T " not in f" {entry[0]} ":
            raise RuntimeError(f"spresense_main symbol is not unique/strong: {entry}")

        manifest_values = {
            "profile": PROFILE,
            "project_commit": project_commit,
            "project_tree": "dirty" if dirty else "clean",
            "sdk_commit": SDK_COMMIT,
            "mavlink_commit": MAVLINK_COMMIT,
            "gcc_version": gcc_version,
            "m1.gcs.transport": "/dev/ttyS0@115200",
            "m1.gcs.bidirectional": "required",
            "m1.gnss.builtin": "disabled",
            "m1.gnss.addon": "required",
            "m1.gnss.device": "/dev/gps2",
            "m1.gnss.ram": "required",
            "m1.outputs": "disabled",
            "m1.arming": "always-denied",
            "m1.physical_write_expected": "0",
            "m1.flight_ready": "false",
        }
        for name in (*sources, "memory-layout.json"):
            manifest_values[f"artifact.{name}.sha256"] = sha256(
                artifact_dir / name
            )
        manifest = artifact_dir / "ARTIFACTS.manifest"
        manifest.write_text(
            "".join(f"{key}={value}\n" for key, value in manifest_values.items()),
            encoding="utf-8",
        )
        json.loads(map_report.read_text(encoding="utf-8"))
    except (
        RuntimeError,
        OSError,
        UnicodeError,
        json.JSONDecodeError,
        subprocess.CalledProcessError,
    ) as error:
        print(f"spresense_m1_gcs_build=FAIL reason={error}", file=sys.stderr)
        return 1

    print(
        "spresense_m1_gcs_build=PASS "
        f"project_commit={project_commit} tree={'dirty' if dirty else 'clean'} "
        f"gcc={gcc_version} artifact_dir={artifact_dir}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
