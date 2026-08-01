#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

import json
from pathlib import Path
import sys


EXPECTED = {
    "upstream.vehicle": "Copter",
    "upstream.tag": "Copter-4.7.0",
    "upstream.commit": "1511f27194f1dcc3728270883047bdf022b3fd53",
    "sdk.commit": "7fd61b2c03f06a4ff0302b84c755e58c338788b2",
    "target.os": "Sony Spresense SDK NuttX",
    "target.profile": "spresense-m1-gcs",
    "gnss.builtin": "disabled",
    "gnss.addon": "required",
    "gnss.device": "/dev/gps2",
    "gnss.ram": "required",
    "safety.arming": "guard-always-reject",
    "safety.actuator_driver": "absent",
    "safety.outputs": "disabled",
    "gcs.transport": "/dev/ttyS0",
    "gcs.baud": 115200,
    "gcs.heartbeat": "MAV_AUTOPILOT_ARDUPILOTMEGA",
    "gcs.parameter_surface": "fixed-read-only-diagnostic",
    "gcs.arm_command": "always-denied",
    "gcs.full_copter": False,
    "storage.development": "microSD",
    "storage.final_candidate": "eMMC",
    "storage.automatic_fallback": "disabled",
    "distribution.binary": "disabled",
    "evidence.sony_object_compile": "required",
    "evidence.sony_gcs_firmware_link": "required",
    "evidence.linker_map_guard": "required",
    "evidence.sony_copter_link": "HOLD",
}


def nested_value(document, dotted_key):
    value = document
    for component in dotted_key.split("."):
        value = value[component]
    return value


def main() -> int:
    root = Path(__file__).resolve().parents[2]
    manifest_path = root / "spresense-m1-manifest.json"
    document = json.loads(manifest_path.read_text(encoding="utf-8"))
    failures = []
    for key, expected in EXPECTED.items():
        try:
            actual = nested_value(document, key)
        except KeyError:
            failures.append(f"{key}=missing")
            continue
        if actual != expected:
            failures.append(f"{key}={actual!r} expected={expected!r}")

    board_header = (root / "libraries/AP_HAL/AP_HAL_Boards.h").read_text(encoding="utf-8")
    if "#define HAL_BOARD_SPRESENSE 14" not in board_header:
        failures.append("HAL_BOARD_SPRESENSE=missing")
    if "#include <AP_HAL/board/spresense.h>" not in board_header:
        failures.append("spresense-board-header=missing")

    implementation = "\n".join(
        path.read_text(encoding="utf-8")
        for path in (root / "libraries/AP_HAL_Spresense").glob("*.cpp")
    )
    for forbidden in ("/dev/pwm", "/dev/dshot", "/dev/can"):
        if forbidden in implementation.lower():
            failures.append(f"physical-output-path={forbidden}")

    gcs_root = root / "Tools/spresense/m1_gcs_app"
    gcs_source = "\n".join(
        path.read_text(encoding="utf-8")
        for path in gcs_root.glob("*.c")
    )
    required_gcs_tokens = (
        "MAV_AUTOPILOT_ARDUPILOTMEGA",
        "MAV_CMD_COMPONENT_ARM_DISARM",
        "MAV_RESULT_DENIED",
        "m1_gcs_physical_write_count",
    )
    for token in required_gcs_tokens:
        if token not in gcs_source:
            failures.append(f"gcs-safety-token-missing={token}")

    profile = (
        gcs_root / "configs/gcs/defconfig"
    ).read_text(encoding="utf-8")
    for required in (
        "+CXD56_GNSS_ADDON=y",
        "+SENSORS_CXD5610_GNSS=y",
        "+CXD56_GNSS_RAM=y",
        "+CXD56_GNSS_HEAP=y",
        "-CXD56_GNSS=y",
        "-CXD56_PWM=y",
        "-PWM=y",
        "-SYSTEM_NSH=y",
    ):
        if required not in profile.splitlines():
            failures.append(f"gcs-profile-token-missing={required}")

    if failures:
        print("spresense_m1_contract=FAIL " + " ".join(failures), file=sys.stderr)
        return 1
    print("spresense_m1_contract=PASS outputs=disabled automatic_fallback=disabled")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
