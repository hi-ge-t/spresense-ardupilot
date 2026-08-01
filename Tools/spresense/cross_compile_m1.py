#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


SONY_GCC_VERSION = "10.3.1"
SPRESENSE_COMMIT = "7fd61b2c03f06a4ff0302b84c755e58c338788b2"


def parser() -> argparse.ArgumentParser:
    argument_parser = argparse.ArgumentParser(description="Compile M1 objects for Sony Spresense/NuttX")
    argument_parser.add_argument(
        "--sdk-root",
        type=Path,
        default=None,
        help="Spresense SDK checkout; defaults to modules/Spresense",
    )
    return argument_parser


def main() -> int:
    arguments = parser().parse_args()
    root = Path(__file__).resolve().parents[2]
    sdk_root = (arguments.sdk_root or root / "modules/Spresense").resolve()
    compiler = shutil.which("arm-none-eabi-g++")
    if compiler is None:
        print("spresense_m1_cross_compile=HOLD reason=arm-none-eabi-g++-not-found", file=sys.stderr)
        return 2

    version = subprocess.run(
        [compiler, "-dumpfullversion", "-dumpversion"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    if version != SONY_GCC_VERSION:
        print(
            f"spresense_m1_cross_compile=HOLD reason=toolchain-version actual={version} expected={SONY_GCC_VERSION}",
            file=sys.stderr,
        )
        return 2

    try:
        sdk_commit = subprocess.run(
            ["git", "-C", str(sdk_root), "rev-parse", "HEAD"],
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
    except subprocess.CalledProcessError:
        print("spresense_m1_cross_compile=HOLD reason=sdk-git-unavailable", file=sys.stderr)
        return 2
    if sdk_commit != SPRESENSE_COMMIT:
        print(
            f"spresense_m1_cross_compile=HOLD reason=sdk-commit actual={sdk_commit} expected={SPRESENSE_COMMIT}",
            file=sys.stderr,
        )
        return 2

    required_directories = [
        sdk_root / "nuttx/include",
        sdk_root / "nuttx/arch/arm/include",
        sdk_root / "sdk/include",
    ]
    for required_directory in required_directories:
        if not required_directory.is_dir():
            print(
                f"spresense_m1_cross_compile=HOLD reason=sdk-include-missing path={required_directory}",
                file=sys.stderr,
            )
            return 2

    config_path = sdk_root / "nuttx/.config"
    config_header = sdk_root / "nuttx/include/nuttx/config.h"
    if not config_path.is_file() or not config_header.is_file():
        print(
            "spresense_m1_cross_compile=HOLD reason=sdk-context-missing "
            "prepare='config.py default feature/gnss_addon; make -C nuttx -j1 context'",
            file=sys.stderr,
        )
        return 2

    config_lines = set(config_path.read_text(encoding="utf-8").splitlines())
    required_config = {
        "# CONFIG_CXD56_GNSS is not set",
        "CONFIG_CXD56_GNSS_ADDON=y",
        "CONFIG_SENSORS_CXD5610_GNSS=y",
        "CONFIG_CXD56_I2C0=y",
        "# CONFIG_CXD56_I2C0_SCUSEQ is not set",
        "CONFIG_CXD56_GNSS_RAM=y",
        "CONFIG_CXD56_GNSS_HEAP=y",
    }
    missing_config = sorted(required_config - config_lines)
    if missing_config:
        print(
            "spresense_m1_cross_compile=HOLD reason=gnss-contract "
            + " ".join(missing_config),
            file=sys.stderr,
        )
        return 2

    sources = [
        root / "libraries/AP_HAL_Spresense/SafeBringup.cpp",
        root / "libraries/AP_HAL_Spresense/UARTDriver.cpp",
        root / "libraries/AP_HAL_Spresense/Storage.cpp",
        root / "libraries/AP_HAL_Spresense/Scheduler.cpp",
        root / "libraries/AP_HAL_Spresense/RCOutput.cpp",
    ]
    common_flags = [
        "-std=gnu++17",
        "-mcpu=cortex-m4",
        "-mthumb",
        "-mfpu=fpv4-sp-d16",
        "-mfloat-abi=hard",
        "-ffunction-sections",
        "-fdata-sections",
        "-fno-exceptions",
        "-fno-rtti",
        "-Wall",
        "-Wextra",
        "-Werror",
        "-Wno-unused-parameter",
        "-Wno-expansion-to-defined",
        "-D__NuttX__",
        "-DCONFIG_HAL_BOARD=HAL_BOARD_SPRESENSE",
        "-DAP_SCRIPTING_ENABLED=0",
        "-I",
        str(required_directories[0]),
        "-I",
        str(required_directories[1]),
        "-I",
        str(required_directories[2]),
        "-I",
        str(root / "libraries"),
    ]

    with tempfile.TemporaryDirectory(prefix="spresense-m1-cross-") as temporary_directory:
        output_directory = Path(temporary_directory)
        for source in sources:
            output = output_directory / f"{source.stem}.o"
            subprocess.run(
                [compiler, *common_flags, "-c", str(source), "-o", str(output)],
                cwd=root,
                check=True,
                env=os.environ.copy(),
            )

    print(
        "spresense_m1_cross_compile=PASS "
        f"gcc={version} sdk_commit={sdk_commit} objects={len(sources)} link=HOLD"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
