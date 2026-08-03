#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

"""Host regression tests for the output-disabled Copter linker-map guard."""

from pathlib import Path
import sys
import tempfile


TOOLS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS))

from verify_m1_copter_map import MapError, inspect  # noqa: E402


MAP_TEMPLATE = """Memory Configuration

Name             Origin             Length
ram              0x000000000d000000 0x0000000000180000
gnssram          0x0000000009000000 0x00000000000a0000

Linker script and memory map

.gnssram.text   0x0000000009000000        0x0
                0x0000000009000000                _sgnsstext = ABSOLUTE (.)
.gnssram.data
.gnssram.bss    0x0000000009000000        0x0
                0x0000000009000000                _gnssramsbss = ABSOLUTE (.)
                0x0000000009000000                _gnssramebss = ABSOLUTE (.)
                0x0000000009000000                _sgnssheap = ABSOLUTE (.)
                {gnss_end}                _egnssheap = (ORIGIN (gnssram) + LENGTH (gnssram))
 .text          0x000000000d000100       0x80 fixture.o
 .rodata        0x000000000d000200       0x40 fixture.o
.data           0x000000002d000300       0x20 load address 0x000000000d000300
.bss            0x000000002d000320       0x40 load address 0x000000000d000320
                0x000000002d000360                _ebss = .
                0x000000002d180000                __stack = .
Cross Reference Table
"""


def write_map(directory: Path, gnss_end: str) -> Path:
    path = directory / "nuttx.map"
    path.write_text(
        MAP_TEMPLATE.format(gnss_end=gnss_end), encoding="utf-8"
    )
    return path


def main() -> int:
    with tempfile.TemporaryDirectory(prefix="spresense-m1-map-") as value:
        directory = Path(value)
        report = inspect(write_map(directory, "0x00000000090a0000"))
        if report["gnss_ram"]["heap"]["bytes"] != 640 * 1024:
            raise RuntimeError("full GNSS heap was not reported")

        try:
            inspect(write_map(directory, "0x0000000009090000"))
        except MapError as error:
            if "complete GNSS RAM" not in str(error):
                raise
        else:
            raise RuntimeError("short GNSS heap was accepted")

        nonempty_static = MAP_TEMPLATE.replace(
            ".gnssram.text   0x0000000009000000        0x0",
            ".gnssram.text   0x0000000009000000       0x20",
        ).format(gnss_end="0x00000000090a0000")
        nonempty_path = directory / "nonempty.map"
        nonempty_path.write_text(nonempty_static, encoding="utf-8")
        try:
            inspect(nonempty_path)
        except MapError as error:
            if "must be empty" not in str(error):
                raise
        else:
            raise RuntimeError("non-empty GNSS static section was accepted")

    print("spresense_m1_copter_map_host=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
