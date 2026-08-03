#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

"""Verify the fail-closed M1 full-Copter software contract."""

import json
from pathlib import Path
import sys


EXPECTED = {
    "upstream.vehicle": "Copter",
    "upstream.tag": "Copter-4.7.0",
    "upstream.commit": "1511f27194f1dcc3728270883047bdf022b3fd53",
    "sdk.commit": "7fd61b2c03f06a4ff0302b84c755e58c338788b2",
    "target.os": "Sony Spresense SDK NuttX",
    "target.profile": "spresense-m1-copter-link",
    "target.entrypoint": "arducopter_spresense_main",
    "copter.full_vehicle_archive": True,
    "copter.scheduler": "nuttx-pthread",
    "copter.sensor_hal_integration": "HOLD",
    "copter.runtime": "hardware-HOLD",
    "copter.flight_ready": False,
    "gnss.builtin": "disabled",
    "gnss.addon": "required",
    "gnss.device": "/dev/gps2",
    "gnss.ram": "required-complete-heap",
    "gnss.hal_integration": "HOLD",
    "gnss.runtime_fallback": "disabled",
    "pwbimu.addon": "required",
    "pwbimu.device": "/dev/imu0",
    "pwbimu.bus": "SPI5",
    "pwbimu.pinshare": "eMMC",
    "pwbimu.hal_integration": "HOLD",
    "pwbimu.runtime_fallback": "disabled",
    "safety.arming": "copter-path-always-reject",
    "safety.actuator_driver": "reject-only-no-physical-backend",
    "safety.outputs": "disabled",
    "safety.physical_write_expected": 0,
    "gcs.transport": "/dev/ttyS0",
    "gcs.baud": 115200,
    "gcs.full_copter": True,
    "gcs.runtime": "hardware-HOLD",
    "storage.development": "microSD",
    "storage.final_candidate": "eMMC",
    "storage.automatic_fallback": "disabled",
    "storage.pwbimu_emmc_coexistence": "hardware-design-HOLD",
    "distribution.binary": "disabled",
    "evidence.sensor_hal_integration": "HOLD",
    "evidence.sensor_runtime": "HOLD",
    "evidence.timing": "HOLD",
    "evidence.gnss_accuracy": "HOLD",
    "evidence.flight": "out-of-scope",
}


def nested_value(document, dotted_key):
    value = document
    for component in dotted_key.split("."):
        value = value[component]
    return value


def require_tokens(failures, label, text, tokens):
    for token in tokens:
        if token not in text:
            failures.append(f"{label}-token-missing={token}")


def main() -> int:
    root = Path(__file__).resolve().parents[2]
    document = json.loads(
        (root / "spresense-m1-copter-manifest.json").read_text(
            encoding="utf-8"
        )
    )
    failures = []
    for key, expected in EXPECTED.items():
        try:
            actual = nested_value(document, key)
        except KeyError:
            failures.append(f"{key}=missing")
            continue
        if actual != expected:
            failures.append(f"{key}={actual!r} expected={expected!r}")

    board = (root / "libraries/AP_HAL/board/spresense.h").read_text(
        encoding="utf-8"
    )
    require_tokens(
        failures,
        "board",
        board,
        (
            "#define HAL_NUM_CAN_IFACES 0",
            "#define HAL_INS_DEFAULT HAL_INS_NONE",
            "#define HAL_SPRESENSE_OUTPUT_DISABLED 1",
            "#define AP_NETWORKING_ENABLED 0",
            "#define HAL_LOGGING_FILESYSTEM_ENABLED 0",
        ),
    )

    arming = (root / "ArduCopter/AP_Arming_Copter.cpp").read_text(
        encoding="utf-8"
    )
    require_tokens(
        failures,
        "arming",
        arming,
        (
            "HAL_SPRESENSE_OUTPUT_DISABLED",
            '"Arm: Spresense outputs disabled"',
            "return false;",
        ),
    )

    hal_root = root / "libraries/AP_HAL_Spresense"
    implementation = "\n".join(
        path.read_text(encoding="utf-8") for path in hal_root.glob("*.cpp")
    )
    for forbidden in ("/dev/pwm", "/dev/dshot", "/dev/can"):
        if forbidden in implementation.lower():
            failures.append(f"physical-output-path={forbidden}")
    require_tokens(
        failures,
        "hal",
        implementation,
        (
            "_guard.request_write",
            "physical_write_count() const",
            "return 0U;",
            "pthread_create",
            "PTHREAD_PRIO_INHERIT",
            'Spresense::UARTDriver serial0_driver("/dev/ttyS0")',
            'SPRESENSE_M1_COPTER_BOOT=HAL',
            'SPRESENSE_M1_COPTER_BOOT=SCHEDULER_FAIL',
            "Empty::I2CDeviceManager i2c_manager",
            "Empty::SPIDeviceManager spi_manager",
        ),
    )

    profile = (
        root / "Tools/spresense/copter_app/configs/output_disabled/defconfig"
    ).read_text(encoding="utf-8")
    require_tokens(
        failures,
        "copter-profile",
        profile,
        (
            "+SPRESENSE_M1_COPTER_LINK=y",
            "+CXD56_GNSS_ADDON=y",
            "+SENSORS_CXD5610_GNSS=y",
            "+SENSORS_CXD5602PWBIMU=y",
            "+CXD56_GNSS_RAM=y",
            "+CXD56_GNSS_HEAP=y",
            '+INIT_ENTRYPOINT="arducopter_spresense_main"',
            "-CXD56_GNSS=y",
            "-CXD56_EMMC=y",
            "-CXD56_PWM=y",
            "-PWM=y",
        ),
    )

    build = (
        root / "Tools/spresense/build_m1_copter_firmware.py"
    ).read_text(encoding="utf-8")
    require_tokens(
        failures,
        "build",
        build,
        (
            "verify_m1_copter_map.py",
            "arducopter_spresense_main",
            "libarducopter.a",
            "libArduCopter_libs.a",
            "m1.physical_write_expected",
            "m1.sensor.hal_integration",
        ),
    )

    flash = (root / "Tools/flash_spresense.sh").read_text(encoding="utf-8")
    require_tokens(
        failures,
        "flash",
        flash,
        (
            "spresense-m1-copter-link",
            "m1.copter.full=true",
            "m1.copter.entry=arducopter_spresense_main",
            "m1.outputs=disabled",
            "m1.arming=always-denied",
            "m1.physical_write_expected=0",
            "m1.sensor_fallback=disabled",
        ),
    )

    if failures:
        print(
            "spresense_m1_copter_contract=FAIL " + " ".join(failures),
            file=sys.stderr,
        )
        return 1
    print(
        "spresense_m1_copter_contract=PASS "
        "outputs=disabled sensors=HAL-HOLD flight_ready=false"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
