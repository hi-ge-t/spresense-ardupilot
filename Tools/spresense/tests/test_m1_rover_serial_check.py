#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "Tools/spresense"))

import m1_copter_serial_check as checker  # noqa: E402


def main() -> int:
    diagnostics = bytearray(
        b"SPRESENSE_M1_ROVER_BOOT=LOOP\n"
        b"SPRESENSE_M1_PWBIMU=SAMPLE\n"
        b"SPRESENSE_M1_GNSS=SAMPLE\n"
        b"SPRESENSE_M1_GNSS=ATTACH\n"
        b"SPRESENSE_M1_GNSS=CONSUMED\n"
        b"SPRESENSE_M1_OUTPUT=SHADOW_ONLY\n"
        b"SPRESENSE_M1_OUTPUT=WRITE_REJECTED\n"
    )
    checker.require_runtime_markers(diagnostics, "rover")
    if checker.missing_runtime_markers(diagnostics, "rover"):
        raise AssertionError("complete Rover marker set was rejected")
    runtime_commit = checker.require_runtime_identity(
        bytearray(b"Init ArduRover V4.7.0 (01234567)\n"),
        "0123456789abcdef0123456789abcdef01234567",
        "rover",
    )
    if runtime_commit != "01234567":
        raise AssertionError("Rover runtime identity was not returned")
    try:
        checker.require_runtime_markers(
            bytearray(b"SPRESENSE_M1_ROVER_BOOT=LOOP\n"), "rover"
        )
    except checker.CheckError as error:
        if "SPRESENSE_M1_OUTPUT=WRITE_REJECTED" not in str(error):
            raise
    else:
        raise AssertionError("missing output rejection marker was accepted")

    print(
        "spresense_m1_rover_serial_host=PASS "
        "runtime_identity=required shadow_output=required "
        "output_rejection=required"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
