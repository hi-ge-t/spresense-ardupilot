#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


def main() -> int:
    root = Path(__file__).resolve().parents[2]
    compiler = os.environ.get("CXX") or shutil.which("c++")
    if compiler is None:
        print("spresense_m1_host=HOLD reason=cxx-not-found", file=sys.stderr)
        return 2

    source = root / "libraries/AP_HAL_Spresense/SafeBringup.cpp"
    test_source = root / "libraries/AP_HAL_Spresense/tests/test_safe_bringup.cpp"
    adapter_sources = [
        root / "libraries/AP_HAL_Spresense/UARTDriver.cpp",
        root / "libraries/AP_HAL_Spresense/Storage.cpp",
        root / "libraries/AP_HAL_Spresense/Scheduler.cpp",
        root / "libraries/AP_HAL_Spresense/RCOutput.cpp",
    ]
    with tempfile.TemporaryDirectory(prefix="spresense-m1-host-") as temporary_directory:
        temporary_path = Path(temporary_directory)
        binary = Path(temporary_directory) / "test_safe_bringup"
        command = [
            compiler,
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-pedantic",
            str(source),
            str(test_source),
            "-o",
            str(binary),
        ]
        subprocess.run(command, cwd=root, check=True)
        subprocess.run([str(binary)], cwd=root, check=True)

        for adapter_source in adapter_sources:
            adapter_object = temporary_path / f"{adapter_source.stem}.o"
            adapter_command = [
                compiler,
                "-std=c++17",
                "-Wall",
                "-Wextra",
                "-Werror",
                "-Wno-unused-parameter",
                "-Wno-gnu-zero-variadic-macro-arguments",
                "-Wno-expansion-to-defined",
                "-DCONFIG_HAL_BOARD=HAL_BOARD_SPRESENSE",
                "-DAP_SCRIPTING_ENABLED=0",
                "-I",
                str(root / "libraries"),
                "-c",
                str(adapter_source),
                "-o",
                str(adapter_object),
            ]
            subprocess.run(adapter_command, cwd=root, check=True)

    subprocess.run(
        [sys.executable, str(root / "Tools/spresense/verify_m1_contract.py")],
        cwd=root,
        check=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
