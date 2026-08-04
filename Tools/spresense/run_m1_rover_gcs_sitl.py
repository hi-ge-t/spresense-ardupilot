#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

"""Exercise the Rover GCS mission transaction against a real SITL binary."""

import argparse
from datetime import datetime, timezone
import hashlib
import os
from pathlib import Path
import subprocess
import sys
import time

import m1_rover_gcs_sequence as gcs


HOME_LATITUDE_E7 = 400713770
HOME_LONGITUDE_E7 = -1052297900
HOME_ALTITUDE_M = 1583
DEFAULT_INSTANCE = 8
STARTUP_DISARMED_HEARTBEATS = 10


def build_rover_command(root):
    return [
        sys.executable,
        str(root / "Tools/autotest/autotest.py"),
        "--no-debug",
        "build.Rover",
    ]


def build_sitl_command(root, instance):
    return [
        str(root / "build/sitl/bin/ardurover"),
        f"-I{instance}",
        "--model",
        "rover",
        "--home",
        (
            f"{HOME_LATITUDE_E7 / 1.0e7},"
            f"{HOME_LONGITUDE_E7 / 1.0e7},"
            f"{HOME_ALTITUDE_M},0"
        ),
        "--serial0",
        "tcp:0",
        "-w",
        "--defaults",
        str(root / "Tools/autotest/default_params/rover.parm"),
    ]


def sitl_environment(root):
    environment = os.environ.copy()
    python_bin = str(Path(sys.executable).parent)
    environment["PATH"] = python_bin + os.pathsep + environment["PATH"]
    mavlink_path = str(root / "modules/mavlink")
    existing_pythonpath = environment.get("PYTHONPATH")
    environment["PYTHONPATH"] = (
        mavlink_path if not existing_pythonpath
        else mavlink_path + os.pathsep + existing_pythonpath
    )
    return environment


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def wait_sitl_connection(mavutil, connection, process, timeout):
    deadline = time.monotonic() + timeout
    last_error = None
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise gcs.common.CheckError(
                f"Rover SITL exited before GCS connection: {process.returncode}"
            )
        try:
            link = mavutil.mavlink_connection(
                connection,
                source_system=255,
                source_component=mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER,
                autoreconnect=False,
                dialect="ardupilotmega",
            )
        except OSError as error:
            last_error = error
            time.sleep(0.25)
            continue
        heartbeat = link.wait_heartbeat(timeout=3.0)
        if heartbeat is not None:
            return link, heartbeat
        link.close()
        time.sleep(0.25)
    suffix = "" if last_error is None else f": {last_error}"
    raise gcs.common.CheckError(f"timeout connecting to Rover SITL{suffix}")


def stop_sitl(process):
    if process is None or process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=10.0)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=5.0)


def git_commit(root):
    return subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=root,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()


def git_tree_state(root):
    status = subprocess.run(
        ["git", "status", "--porcelain"],
        cwd=root,
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    return "clean" if not status.strip() else "dirty"


def read_log_tail(path, line_count=30):
    try:
        return "\n".join(
            path.read_text(encoding="utf-8", errors="replace").splitlines()[
                -line_count:
            ]
        )
    except OSError:
        return ""


def parse_arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--skip-build", action="store_true")
    parser.add_argument("--instance", type=int, default=DEFAULT_INSTANCE)
    parser.add_argument("--startup-timeout", type=float, default=90.0)
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def main():
    args = parse_arguments()
    if args.instance < 0 or args.instance > 50:
        print("SITL instance must be between 0 and 50", file=sys.stderr)
        return 2

    root = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(root / "modules/mavlink"))
    run_dir = root / "build/spresense-m1-rover-gcs-sitl"
    run_dir.mkdir(parents=True, exist_ok=True)
    log_path = run_dir / "ardurover.log"
    binary = root / "build/sitl/bin/ardurover"
    connection = f"tcp:127.0.0.1:{5760 + 10 * args.instance}"
    process = None
    link = None
    log_stream = None
    diagnostics = bytearray()
    initial_mission = None
    backup_mission = None
    mission_modified = False
    mission_restored = False
    initial_state_restored = False
    hold_entered = False
    auto_result = None
    auto_entered = False
    system = None
    component = None
    evidence = {
        "format": "spresense-m1-rover-gcs-sitl-v1",
        "captured_at": datetime.now(timezone.utc).isoformat(),
        "project_commit": git_commit(root),
        "project_tree": git_tree_state(root),
        "scenario": "real-Rover-SITL-MAVLink-mission-round-trip",
        "gcs_source_component": "MAV_COMP_ID_MISSIONPLANNER",
        "connection": connection,
        "vehicle_type": "GROUND_ROVER",
        "regular_front_steering_simulation": True,
        "spresense_hardware_verified": False,
        "physical_outputs_verified": False,
        "driving_verified": False,
        "autonomous_motion_verified": False,
        "autonomous_mission_completion_verified": False,
    }

    try:
        from pymavlink import mavutil

        if not args.skip_build:
            subprocess.run(
                build_rover_command(root),
                cwd=root,
                env=sitl_environment(root),
                check=True,
            )
        if not binary.is_file():
            raise gcs.common.CheckError(f"Rover SITL binary missing: {binary}")
        evidence["sitl_binary_sha256"] = sha256_file(binary)

        log_stream = log_path.open("w", encoding="utf-8")
        process = subprocess.Popen(
            build_sitl_command(root, args.instance),
            cwd=run_dir,
            stdout=log_stream,
            stderr=subprocess.STDOUT,
            text=True,
            env=sitl_environment(root),
        )
        link, heartbeat = wait_sitl_connection(
            mavutil, connection, process, args.startup_timeout
        )
        if heartbeat.type != mavutil.mavlink.MAV_TYPE_GROUND_ROVER:
            raise gcs.common.CheckError(
                f"unexpected SITL vehicle type: {heartbeat.type}"
            )
        if heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            raise gcs.common.CheckError("Rover SITL initially advertised armed")
        system = heartbeat.get_srcSystem()
        component = heartbeat.get_srcComponent()
        link.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            mavutil.mavlink.MAV_STATE_ACTIVE,
        )
        gcs.wait_disarmed_heartbeats(
            link,
            mavutil.mavlink,
            system,
            diagnostics,
            STARTUP_DISARMED_HEARTBEATS,
            20.0,
        )

        initial_mission = gcs.download_mission(
            link, mavutil.mavlink, system, component, diagnostics
        )
        backup_fixture = gcs.build_dry_run_mission(
            mavutil.mavlink,
            HOME_LATITUDE_E7 - 1000,
            HOME_LONGITUDE_E7 - 1000,
        )
        mission_modified = True
        gcs.upload_mission(
            link, mavutil.mavlink, system, component,
            backup_fixture, diagnostics,
        )
        backup_mission = gcs.download_mission(
            link, mavutil.mavlink, system, component, diagnostics
        )
        difference = gcs.mission_difference(backup_fixture, backup_mission)
        if difference is not None:
            raise gcs.common.CheckError(
                f"SITL backup fixture round-trip failed: {difference}"
            )

        test_mission = gcs.build_dry_run_mission(
            mavutil.mavlink, HOME_LATITUDE_E7, HOME_LONGITUDE_E7
        )
        gcs.upload_mission(
            link, mavutil.mavlink, system, component,
            test_mission, diagnostics,
        )
        downloaded = gcs.download_mission(
            link, mavutil.mavlink, system, component, diagnostics
        )
        difference = gcs.mission_difference(test_mission, downloaded)
        if difference is not None:
            raise gcs.common.CheckError(
                f"SITL test mission round-trip failed: {difference}"
            )

        mode_map = mavutil.mode_mapping_byname(
            mavutil.mavlink.MAV_TYPE_GROUND_ROVER
        )
        _, hold_entered = gcs.request_mode(
            link, mavutil.mavlink, system, component,
            mode_map["HOLD"], diagnostics,
        )
        if not hold_entered:
            raise gcs.common.CheckError("Rover SITL did not enter HOLD")
        auto_result, auto_entered = gcs.request_mode(
            link, mavutil.mavlink, system, component,
            mode_map["AUTO"], diagnostics,
        )
        if not auto_entered:
            raise gcs.common.CheckError(
                f"Rover SITL did not enter disarmed AUTO: result={auto_result}"
            )
        _, final_hold_entered = gcs.request_mode(
            link, mavutil.mavlink, system, component,
            mode_map["HOLD"], diagnostics,
        )
        if not final_hold_entered:
            raise gcs.common.CheckError("Rover SITL did not return to HOLD")

        gcs.upload_mission(
            link, mavutil.mavlink, system, component,
            backup_mission, diagnostics,
        )
        restored = gcs.download_mission(
            link, mavutil.mavlink, system, component, diagnostics
        )
        difference = gcs.mission_difference(backup_mission, restored)
        if difference is not None:
            raise gcs.common.CheckError(
                f"SITL backup restoration failed: {difference}"
            )
        mission_restored = True

        gcs.upload_mission(
            link, mavutil.mavlink, system, component,
            initial_mission, diagnostics,
        )
        final_mission = gcs.download_mission(
            link, mavutil.mavlink, system, component, diagnostics
        )
        difference = gcs.mission_difference(initial_mission, final_mission)
        if difference is not None:
            raise gcs.common.CheckError(
                f"SITL initial mission cleanup failed: {difference}"
            )
        initial_state_restored = True
        mission_modified = False

        evidence.update({
            "sequence_passed": True,
            "armed_observed": False,
            "initial_mission_count": len(initial_mission),
            "backup_fixture_count": len(backup_mission),
            "test_mission_count": len(test_mission),
            "mission_upload_verified": True,
            "mission_download_verified": True,
            "mission_restored": mission_restored,
            "initial_state_restored": initial_state_restored,
            "hold_mode_entered": hold_entered,
            "auto_mode_request_result": int(auto_result),
            "auto_mode_entered_disarmed": auto_entered,
            "final_hold_mode_entered": final_hold_entered,
            "arm_command_sent": False,
            "startup_disarmed_heartbeats": STARTUP_DISARMED_HEARTBEATS,
        })
        gcs.write_evidence(root, args.output, evidence)
    except (
        gcs.common.CheckError,
        ImportError,
        OSError,
        subprocess.SubprocessError,
    ) as error:
        restoration_error = None
        if link is not None and system is not None and component is not None:
            try:
                from pymavlink import mavutil

                mode_map = mavutil.mode_mapping_byname(
                    mavutil.mavlink.MAV_TYPE_GROUND_ROVER
                )
                gcs.request_mode(
                    link, mavutil.mavlink, system, component,
                    mode_map["HOLD"], diagnostics,
                )
                if mission_modified and initial_mission is not None:
                    gcs.upload_mission(
                        link, mavutil.mavlink, system, component,
                        initial_mission, diagnostics,
                    )
                    restored = gcs.download_mission(
                        link, mavutil.mavlink, system, component, diagnostics
                    )
                    initial_state_restored = gcs.missions_equal(
                        initial_mission, restored
                    )
                    if not initial_state_restored:
                        raise gcs.common.CheckError(
                            "SITL failure cleanup did not restore initial mission"
                        )
            except (
                gcs.common.CheckError,
                ImportError,
                OSError,
            ) as cleanup_error:
                restoration_error = cleanup_error
        evidence.update({
            "sequence_passed": False,
            "failure_reason": str(error),
            "armed_observed": False,
            "mission_restored": mission_restored,
            "initial_state_restored": initial_state_restored,
            "restoration_error": (
                None if restoration_error is None else str(restoration_error)
            ),
            "arm_command_sent": False,
        })
        gcs.write_evidence(root, args.output, evidence)
        print(
            "spresense_m1_rover_gcs_sitl=FAIL "
            f"reason={error} initial_state_restored="
            f"{str(initial_state_restored).lower()}",
            file=sys.stderr,
        )
        log_tail = read_log_tail(log_path)
        if log_tail:
            print(log_tail, file=sys.stderr)
        return 1
    finally:
        if link is not None:
            link.close()
        stop_sitl(process)
        if log_stream is not None:
            log_stream.close()

    print(
        "spresense_m1_rover_gcs_sitl=PASS "
        "mission=upload-download-restore auto=entered-disarmed "
        "arm_command=false hardware_driving=false"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
