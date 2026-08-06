#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

"""Run the upstream regular-steering Rover mission sequence in SITL."""

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import subprocess
import sys
import time


def build_command(root, skip_build=False):
    targets = [] if skip_build else ["build.Rover"]
    return [
        sys.executable,
        str(root / "Tools/autotest/autotest.py"),
        "--no-debug",
        "--speedup=5",
        *targets,
        "test.Rover.DriveMission",
    ]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--skip-build", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    command = build_command(root, args.skip_build)
    environment = os.environ.copy()
    mavlink_path = str(root / "modules/mavlink")
    python_bin = str(Path(sys.executable).parent)
    environment["PATH"] = python_bin + os.pathsep + environment["PATH"]
    existing_pythonpath = environment.get("PYTHONPATH")
    environment["PYTHONPATH"] = (
        mavlink_path if not existing_pythonpath
        else mavlink_path + os.pathsep + existing_pythonpath
    )
    started = time.monotonic()
    result = subprocess.run(
        command, cwd=root, env=environment, check=False
    )
    duration = time.monotonic() - started
    if result.returncode != 0:
        print(
            "spresense_m1_rover_sitl_autonomy=FAIL "
            f"returncode={result.returncode}",
            file=sys.stderr,
        )
        return result.returncode

    commit = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=root,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    evidence = {
        "format": "spresense-m1-rover-sitl-autonomy-v1",
        "captured_at": datetime.now(timezone.utc).isoformat(),
        "project_commit": commit,
        "scenario": "Rover.DriveMission",
        "mission_file": "Tools/autotest/ArduRover_Tests/DriveMission/rover1.txt",
        "gcs_mission_upload_verified": True,
        "auto_mode_verified": True,
        "software_arming_verified": True,
        "waypoint_progression_verified": True,
        "mission_complete_verified": True,
        "software_disarm_verified": True,
        "regular_front_steering_simulation": True,
        "duration_seconds": round(duration, 3),
        "spresense_hardware_verified": False,
        "physical_outputs_verified": False,
        "driving_verified": False,
    }
    if args.output is not None:
        output = args.output if args.output.is_absolute() else root / args.output
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(
            json.dumps(evidence, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
    print(
        "spresense_m1_rover_sitl_autonomy=PASS "
        "mission=DriveMission sequence=upload-arm-auto-waypoints-complete-disarm "
        "hardware_driving=false"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
