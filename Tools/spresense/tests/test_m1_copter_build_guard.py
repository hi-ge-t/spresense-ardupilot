#!/usr/bin/env python3

from pathlib import Path
import sys
import tempfile


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "Tools/spresense"))

import build_m1_copter_firmware as builder  # noqa: E402


def expect(condition, message):
    if not condition:
        raise AssertionError(message)


def main() -> int:
    commit = "e4382f725e63acfd246078b2d2a05cee288ac49d"
    with tempfile.TemporaryDirectory(
        prefix="spresense-m1-build-guard-"
    ) as temporary_directory:
        firmware = Path(temporary_directory) / "nuttx"
        firmware.write_bytes(
            b"prefix\0ArduCopter V4.7.0 (e4382f72)\0suffix"
        )
        builder.verify_embedded_commit(firmware, commit)

        firmware.write_bytes(
            b"prefix\0ArduCopter V4.7.0 (307003b7)\0suffix"
        )
        try:
            builder.verify_embedded_commit(firmware, commit)
        except RuntimeError as error:
            expect(
                "does not match" in str(error),
                "stale archive identity is reported",
            )
        else:
            raise AssertionError("stale archive identity was accepted")

    print(
        "spresense_m1_copter_build_guard_host=PASS "
        "embedded_commit=required"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
