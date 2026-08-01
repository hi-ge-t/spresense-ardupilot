#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

"""Generate the pinned ArduPilot MAVLink C headers used by M1."""

from pathlib import Path
import subprocess
import sys


MAVLINK_COMMIT = "288b907c384a892c8519bfe271682424b1e1a3a0"


def generate(root: Path) -> Path:
    mavlink = root / "modules/mavlink"
    output = root / "build/spresense-m1-generated"
    generator = mavlink / "pymavlink/tools/mavgen.py"
    dialect = mavlink / "message_definitions/v1.0/ardupilotmega.xml"
    if not generator.is_file():
        raise RuntimeError(
            "MAVLink submodule is incomplete; run git submodule update "
            "--init --recursive modules/mavlink"
        )
    commit = subprocess.run(
        ["git", "-C", str(mavlink), "rev-parse", "HEAD"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    if commit != MAVLINK_COMMIT:
        raise RuntimeError(
            f"MAVLink commit mismatch: {commit} != {MAVLINK_COMMIT}"
        )
    output.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        [
            sys.executable,
            str(generator),
            "--lang",
            "C",
            "--wire-protocol",
            "2.0",
            "--output",
            str(output),
            str(dialect),
        ],
        cwd=root,
        check=True,
    )
    header = output / "ardupilotmega/mavlink.h"
    if not header.is_file():
        raise RuntimeError(f"generated MAVLink header is missing: {header}")
    return output


def main() -> int:
    root = Path(__file__).resolve().parents[2]
    try:
        output = generate(root)
    except (RuntimeError, subprocess.CalledProcessError) as error:
        print(f"spresense_m1_mavlink_headers=FAIL reason={error}", file=sys.stderr)
        return 1
    print(
        "spresense_m1_mavlink_headers=PASS "
        f"commit={MAVLINK_COMMIT} output={output}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
