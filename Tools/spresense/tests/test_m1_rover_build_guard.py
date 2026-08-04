#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

from pathlib import Path
import sys
import tempfile


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "Tools/spresense"))

import build_m1_copter_firmware as builder  # noqa: E402


def main() -> int:
    commit = "0123456789abcdef0123456789abcdef01234567"
    with tempfile.TemporaryDirectory(
        prefix="spresense-m1-rover-build-guard-"
    ) as temporary_directory:
        firmware = Path(temporary_directory) / "nuttx"
        firmware.write_bytes(
            b"prefix\0ArduRover V4.7.0 (01234567)\0suffix"
        )
        builder.verify_embedded_commit(firmware, commit, "ArduRover")

        firmware.write_bytes(
            b"prefix\0ArduCopter V4.7.0 (01234567)\0suffix"
        )
        try:
            builder.verify_embedded_commit(firmware, commit, "ArduRover")
        except RuntimeError as error:
            if "does not match" not in str(error):
                raise
        else:
            raise AssertionError("Copter archive was accepted as Rover")

    symbols, entries = builder.parse_nm(
        ["00000000 T ardurover_spresense_main"],
        "ardurover_spresense_main",
    )
    if "ardurover_spresense_main" not in symbols or len(entries) != 1:
        raise AssertionError("Rover entrypoint was not identified")

    print(
        "spresense_m1_rover_build_guard_host=PASS "
        "vehicle_identity=required entrypoint=required"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
