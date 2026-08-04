#!/usr/bin/env python3

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "Tools/spresense"))

import m1_copter_serial_check as checker  # noqa: E402


class FakeLink:
    def __init__(self, results):
        self.results = iter(results)

    def recv(self, size=None):
        _ = size
        result = next(self.results)
        if isinstance(result, BaseException):
            raise result
        return result


def expect(condition, message):
    if not condition:
        raise AssertionError(message)


def main() -> int:
    transient = OSError(
        checker.TRANSIENT_ZERO_READ + " (device disconnected?)"
    )
    link = FakeLink([transient, b"next"])
    checker.tolerate_transient_zero_reads(link)
    expect(link.recv(8) == b"", "transient zero read is tolerated")
    expect(link.recv(8) == b"next", "subsequent data is preserved")

    fatal = FakeLink([OSError("unrelated serial failure")])
    checker.tolerate_transient_zero_reads(fatal)
    try:
        fatal.recv(8)
    except OSError as error:
        expect(str(error) == "unrelated serial failure", "fatal error preserved")
    else:
        raise AssertionError("unrelated serial error was swallowed")

    diagnostics = bytearray(
        b"SPRESENSE_M1_COPTER_BOOT=LOOP\n"
        b"SPRESENSE_M1_PWBIMU=SAMPLE\n"
        b"SPRESENSE_M1_GNSS=SAMPLE\n"
        b"SPRESENSE_M1_GNSS=ATTACH\n"
        b"SPRESENSE_M1_GNSS=CONSUMED\n"
    )
    checker.require_runtime_markers(diagnostics)
    try:
        checker.require_runtime_markers(
            bytearray(b"SPRESENSE_M1_COPTER_BOOT=LOOP\n")
        )
    except checker.CheckError as error:
        expect(
            "SPRESENSE_M1_PWBIMU=SAMPLE" in str(error),
            "missing runtime marker is reported",
        )
    else:
        raise AssertionError("missing runtime markers were accepted")

    print(
        "spresense_m1_copter_serial_host=PASS "
        "transient_zero_read=tolerated runtime_markers=required"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
