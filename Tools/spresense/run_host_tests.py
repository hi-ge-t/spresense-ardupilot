#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

from mavlink_headers import generate


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
        root / "libraries/AP_HAL_Spresense/Semaphores.cpp",
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

        sensor_binary = temporary_path / "test_sensor_bridge"
        sensor_command = [
            compiler,
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-pedantic",
            str(root / "libraries/AP_HAL_Spresense/SensorBridge.cpp"),
            str(
                root /
                "libraries/AP_HAL_Spresense/tests/test_sensor_bridge.cpp"
            ),
            "-o",
            str(sensor_binary),
        ]
        subprocess.run(sensor_command, cwd=root, check=True)
        subprocess.run([str(sensor_binary)], cwd=root, check=True)

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

        mavlink_headers = generate(root)
        for name, defines in (
            ("legacy", []),
            (
                "pwbimu_gnss",
                [
                    "-DCONFIG_SPRESENSE_M1_GNSS_RUNTIME_REQUIRED=1",
                    "-DCONFIG_SPRESENSE_M1_PWBIMU_REQUIRED=1",
                ],
            ),
        ):
            gcs_binary = temporary_path / f"test_m1_gcs_protocol_{name}"
            gcs_command = [
                os.environ.get("CC") or shutil.which("cc") or "cc",
                "-std=c11",
                "-Wall",
                "-Wextra",
                "-Werror",
                *defines,
                "-I",
                str(mavlink_headers),
                "-I",
                str(root / "Tools/spresense/m1_gcs_app"),
                str(root / "Tools/spresense/m1_gcs_app/m1_gcs_protocol.c"),
                str(root / "Tools/spresense/tests/test_m1_gcs_protocol.c"),
                "-o",
                str(gcs_binary),
            ]
            subprocess.run(gcs_command, cwd=root, check=True)
            subprocess.run([str(gcs_binary)], cwd=root, check=True)

    subprocess.run(
        [sys.executable, str(root / "Tools/spresense/verify_m1_contract.py")],
        cwd=root,
        check=True,
    )
    subprocess.run(
        [sys.executable,
         str(root / "Tools/spresense/verify_m1_copter_contract.py")],
        cwd=root,
        check=True,
    )
    subprocess.run(
        [sys.executable,
         str(root / "Tools/spresense/tests/test_verify_m1_copter_map.py")],
        cwd=root,
        check=True,
    )
    subprocess.run(
        [sys.executable,
         str(root / "Tools/spresense/tests/test_m1_copter_serial_check.py")],
        cwd=root,
        check=True,
    )
    subprocess.run(
        [sys.executable,
         str(root / "Tools/spresense/tests/test_m1_copter_bench_check.py")],
        cwd=root,
        check=True,
    )
    subprocess.run(
        [sys.executable,
         str(root / "Tools/spresense/tests/test_m1_copter_build_guard.py")],
        cwd=root,
        check=True,
    )
    subprocess.run(
        [sys.executable,
         str(root / "Tools/spresense/verify_m1_rover_contract.py")],
        cwd=root,
        check=True,
    )
    subprocess.run(
        [sys.executable,
         str(root / "Tools/spresense/tests/test_m1_rover_build_guard.py")],
        cwd=root,
        check=True,
    )
    subprocess.run(
        [sys.executable,
         str(root / "Tools/spresense/tests/test_m1_rover_serial_check.py")],
        cwd=root,
        check=True,
    )
    subprocess.run(
        [sys.executable, "-m", "py_compile",
         str(root / "Tools/spresense/build_m1_copter_firmware.py"),
         str(root / "Tools/spresense/build_m1_rover_firmware.py"),
         str(root / "Tools/spresense/m1_copter_bench_check.py"),
         str(root / "Tools/spresense/m1_copter_serial_check.py"),
         str(root / "Tools/spresense/m1_rover_serial_check.py"),
         str(root / "Tools/spresense/verify_m1_copter_map.py"),
         str(root / "Tools/spresense/verify_m1_rover_contract.py")],
        cwd=root,
        check=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
