#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

"""Exercise a reversible output-disabled Rover GCS mission sequence."""

import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import time

import m1_copter_serial_check as common


@dataclass(frozen=True)
class MissionItem:
    seq: int
    frame: int
    command: int
    current: int
    autocontinue: int
    param1: float
    param2: float
    param3: float
    param4: float
    x: int
    y: int
    z: float


def receive_message(link, message_types, predicate, timeout, diagnostics):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        message = link.recv_match(blocking=True, timeout=0.5)
        if common.capture_bad_data(message, diagnostics):
            continue
        if (
            message is not None and
            message.get_type() in message_types and
            predicate(message)
        ):
            return message
    raise common.CheckError(
        "timeout waiting for " + "/".join(sorted(message_types))
    )


def mission_item_from_message(message):
    if message.get_type() == "MISSION_ITEM_INT":
        x = int(message.x)
        y = int(message.y)
    else:
        x = int(round(float(message.x) * 1.0e7))
        y = int(round(float(message.y) * 1.0e7))
    return MissionItem(
        seq=int(message.seq),
        frame=int(message.frame),
        command=int(message.command),
        current=int(message.current),
        autocontinue=int(message.autocontinue),
        param1=float(message.param1),
        param2=float(message.param2),
        param3=float(message.param3),
        param4=float(message.param4),
        x=x,
        y=y,
        z=float(message.z),
    )


def send_mission_item(link, system, component, item):
    link.mav.mission_item_int_send(
        system,
        component,
        item.seq,
        item.frame,
        item.command,
        item.current,
        item.autocontinue,
        item.param1,
        item.param2,
        item.param3,
        item.param4,
        item.x,
        item.y,
        item.z,
    )


def download_mission(link, mavlink, system, component, diagnostics):
    deadline = time.monotonic() + 15.0
    next_request = 0.0
    count_message = None
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_request:
            link.mav.mission_request_list_send(system, component)
            next_request = now + 2.0
        message = link.recv_match(blocking=True, timeout=0.5)
        if common.capture_bad_data(message, diagnostics):
            continue
        if (
            message is not None and
            message.get_type() == "MISSION_COUNT" and
            message.get_srcSystem() == system
        ):
            count_message = message
            break
    if count_message is None:
        raise common.CheckError("timeout waiting for MISSION_COUNT")

    items = []
    for seq in range(int(count_message.count)):
        item_message = None
        item_deadline = time.monotonic() + 10.0
        next_request = 0.0
        while time.monotonic() < item_deadline:
            now = time.monotonic()
            if now >= next_request:
                link.mav.mission_request_int_send(system, component, seq)
                next_request = now + 2.0
            message = link.recv_match(blocking=True, timeout=0.5)
            if common.capture_bad_data(message, diagnostics):
                continue
            if (
                message is not None and
                message.get_type() in {"MISSION_ITEM", "MISSION_ITEM_INT"} and
                message.get_srcSystem() == system and
                int(message.seq) == seq
            ):
                item_message = message
                break
        if item_message is None:
            raise common.CheckError(f"timeout waiting for mission item {seq}")
        items.append(mission_item_from_message(item_message))
    link.mav.mission_ack_send(system, component, mavlink.MAV_MISSION_ACCEPTED)
    return items


def clear_mission(link, mavlink, system, component, diagnostics):
    link.mav.mission_clear_all_send(system, component)
    acknowledgement = receive_message(
        link,
        {"MISSION_ACK"},
        lambda value: value.get_srcSystem() == system,
        10.0,
        diagnostics,
    )
    if acknowledgement.type != mavlink.MAV_MISSION_ACCEPTED:
        raise common.CheckError(
            f"mission clear rejected: result={acknowledgement.type}"
        )


def upload_mission(link, mavlink, system, component, items, diagnostics):
    if not items:
        clear_mission(link, mavlink, system, component, diagnostics)
        return
    for seq, item in enumerate(items):
        if item.seq != seq:
            raise common.CheckError("mission sequence numbers are not contiguous")

    link.mav.mission_count_send(system, component, len(items))
    sent = set()
    deadline = time.monotonic() + 30.0
    next_count = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        message = link.recv_match(blocking=True, timeout=0.5)
        if common.capture_bad_data(message, diagnostics):
            continue
        if message is None:
            if time.monotonic() >= next_count and not sent:
                link.mav.mission_count_send(system, component, len(items))
                next_count = time.monotonic() + 3.0
            continue
        if (
            message.get_type() in {"MISSION_REQUEST", "MISSION_REQUEST_INT"} and
            message.get_srcSystem() == system
        ):
            seq = int(message.seq)
            if seq < 0 or seq >= len(items):
                raise common.CheckError(f"target requested invalid mission item {seq}")
            send_mission_item(link, system, component, items[seq])
            sent.add(seq)
            continue
        if (
            message.get_type() == "MISSION_ACK" and
            message.get_srcSystem() == system
        ):
            if message.type != mavlink.MAV_MISSION_ACCEPTED:
                raise common.CheckError(
                    f"mission upload rejected: result={message.type}"
                )
            if sent != set(range(len(items))):
                raise common.CheckError("mission upload acknowledged before every item")
            return
    raise common.CheckError("timeout waiting for mission upload acknowledgement")


def missions_equal(expected, actual):
    if len(expected) != len(actual):
        return False
    for left, right in zip(expected, actual):
        if (
            left.seq != right.seq or
            left.frame != right.frame or
            left.command != right.command or
            left.current != right.current or
            left.autocontinue != right.autocontinue or
            left.x != right.x or
            left.y != right.y
        ):
            return False
        for left_value, right_value in (
            (left.param1, right.param1),
            (left.param2, right.param2),
            (left.param3, right.param3),
            (left.param4, right.param4),
            (left.z, right.z),
        ):
            if not math.isclose(left_value, right_value, abs_tol=1.0e-3):
                return False
    return True


def build_dry_run_mission(mavlink, latitude_e7, longitude_e7):
    return [
        MissionItem(
            0, mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1,
            0.0, 0.0, 0.0, 0.0, latitude_e7, longitude_e7, 0.0,
        ),
        MissionItem(
            1, mavlink.MAV_FRAME_GLOBAL,
            mavlink.MAV_CMD_DO_CHANGE_SPEED, 0, 1,
            0.0, 1.0, -1.0, 0.0, 0, 0, 0.0,
        ),
        MissionItem(
            2, mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1,
            0.0, 0.0, 0.0, 0.0,
            latitude_e7 + 450, longitude_e7 + 450, 0.0,
        ),
    ]


def request_mode(link, mavlink, system, component, custom_mode, diagnostics):
    link.mav.command_long_send(
        system,
        component,
        mavlink.MAV_CMD_DO_SET_MODE,
        0,
        mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        custom_mode,
        0,
        0,
        0,
        0,
        0,
    )
    acknowledgement = common.wait_message(
        link,
        "COMMAND_ACK",
        lambda value: (
            value.get_srcSystem() == system and
            value.command == mavlink.MAV_CMD_DO_SET_MODE
        ),
        8.0,
        diagnostics,
    )
    accepted = acknowledgement.result == mavlink.MAV_RESULT_ACCEPTED
    entered = False
    if accepted:
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            heartbeat = link.recv_match(blocking=True, timeout=0.5)
            if common.capture_bad_data(heartbeat, diagnostics):
                continue
            if (
                heartbeat is None or
                heartbeat.get_type() != "HEARTBEAT" or
                heartbeat.get_srcSystem() != system
            ):
                continue
            if heartbeat.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                raise common.CheckError(
                    "Rover armed during output-disabled mode request"
                )
            if int(heartbeat.custom_mode) == custom_mode:
                entered = True
                break
    return int(acknowledgement.result), entered


def request_shadow_outputs(link, mavlink, system, component, diagnostics):
    common.request_message(
        link,
        mavlink,
        system,
        component,
        mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,
        diagnostics,
    )
    return common.wait_requested_message(
        link,
        mavlink,
        system,
        component,
        mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,
        "SERVO_OUTPUT_RAW",
        lambda value: value.get_srcSystem() == system,
        15.0,
        diagnostics,
    )


def parse_arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--reset-dtr", action="store_true")
    parser.add_argument("--startup-timeout", type=float, default=150.0)
    parser.add_argument("--artifact-dir", type=Path)
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def main():
    args = parse_arguments()
    root = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(root / "modules/mavlink"))
    link = None
    diagnostics = bytearray()
    backup = None
    mission_modified = False
    mission_restored = False
    system = None
    component = None
    try:
        from pymavlink import mavutil

        artifact_argument = args.artifact_dir or Path(
            common.VEHICLES["rover"]["artifact_directory"]
        )
        artifact_dir = (
            artifact_argument if artifact_argument.is_absolute()
            else root / artifact_argument
        )
        manifest = common.read_manifest(
            artifact_dir / "ARTIFACTS.manifest", "rover"
        )
        common.check_port(args.port)
        if args.reset_dtr:
            common.reset_target(args.port, args.baud)
        link = mavutil.mavlink_connection(
            args.port,
            baud=args.baud,
            source_system=255,
            source_component=mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER,
            autoreconnect=False,
            dialect="ardupilotmega",
        )
        common.tolerate_transient_zero_reads(link)
        heartbeat = common.wait_vehicle_startup(
            link, mavutil.mavlink, args.startup_timeout, diagnostics, "rover"
        )
        runtime_commit = common.require_runtime_identity(
            diagnostics, manifest["project_commit"], "rover"
        )
        if heartbeat.type != mavutil.mavlink.MAV_TYPE_GROUND_ROVER:
            raise common.CheckError(f"unexpected vehicle type: {heartbeat.type}")
        if heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            raise common.CheckError("target initially advertises armed state")
        system = heartbeat.get_srcSystem()
        component = heartbeat.get_srcComponent()
        link.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            mavutil.mavlink.MAV_STATE_ACTIVE,
        )

        normal_arm = common.request_arm(
            link, mavutil.mavlink, system, component, 0, diagnostics
        )
        forced_arm = common.request_arm(
            link, mavutil.mavlink, system, component, 2989, diagnostics
        )
        common.wait_runtime_markers(link, diagnostics, 120.0, "rover")

        common.request_message(
            link,
            mavutil.mavlink,
            system,
            component,
            mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,
            diagnostics,
        )
        gps = common.wait_requested_message(
            link,
            mavutil.mavlink,
            system,
            component,
            mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,
            "GPS_RAW_INT",
            lambda value: value.get_srcSystem() == system,
            30.0,
            diagnostics,
        )

        backup = download_mission(
            link, mavutil.mavlink, system, component, diagnostics
        )
        test_mission = build_dry_run_mission(
            mavutil.mavlink, int(gps.lat), int(gps.lon)
        )
        mission_modified = True
        upload_mission(
            link, mavutil.mavlink, system, component,
            test_mission, diagnostics,
        )
        downloaded = download_mission(
            link, mavutil.mavlink, system, component, diagnostics
        )
        if not missions_equal(test_mission, downloaded):
            raise common.CheckError("downloaded mission differs from upload")

        mode_map = mavutil.mode_mapping_byname(
            mavutil.mavlink.MAV_TYPE_GROUND_ROVER
        )
        hold_result, hold_entered = request_mode(
            link, mavutil.mavlink, system, component,
            mode_map["HOLD"], diagnostics,
        )
        if not hold_entered:
            raise common.CheckError(
                f"failed to enter HOLD before AUTO request: result={hold_result}"
            )
        auto_result, auto_entered = request_mode(
            link, mavutil.mavlink, system, component,
            mode_map["AUTO"], diagnostics,
        )
        if (
            auto_result == mavutil.mavlink.MAV_RESULT_ACCEPTED and
            not auto_entered
        ):
            raise common.CheckError("AUTO mode was accepted but not entered")
        shadow = request_shadow_outputs(
            link, mavutil.mavlink, system, component, diagnostics
        )
        final_hold_result, final_hold_entered = request_mode(
            link, mavutil.mavlink, system, component,
            mode_map["HOLD"], diagnostics,
        )
        if not final_hold_entered:
            raise common.CheckError(
                "failed to return to HOLD after AUTO request: "
                f"result={final_hold_result}"
            )

        upload_mission(
            link, mavutil.mavlink, system, component, backup, diagnostics
        )
        restored = download_mission(
            link, mavutil.mavlink, system, component, diagnostics
        )
        if not missions_equal(backup, restored):
            raise common.CheckError("original mission restoration did not verify")
        mission_restored = True
        mission_modified = False

        if b"SPRESENSE_M1_OUTPUT=SHADOW_ONLY" not in diagnostics:
            raise common.CheckError("shadow-only output marker missing")
        evidence = {
            "format": "spresense-m1-rover-gcs-sequence-v1",
            "captured_at": datetime.now(timezone.utc).isoformat(),
            "profile": common.VEHICLES["rover"]["profile"],
            "project_commit": manifest["project_commit"],
            "runtime_commit": runtime_commit,
            "image_sha256": manifest["artifact.nuttx.spk.sha256"],
            "vehicle_type": "GROUND_ROVER",
            "normal_arm_result": normal_arm,
            "forced_arm_result": forced_arm,
            "armed_observed": False,
            "mission_backup_count": len(backup),
            "mission_test_count": len(test_mission),
            "mission_upload_verified": True,
            "mission_download_verified": True,
            "mission_restored": mission_restored,
            "hold_mode_entered": hold_entered,
            "auto_mode_request_result": auto_result,
            "auto_mode_entered_disarmed": auto_entered,
            "final_hold_mode_entered": final_hold_entered,
            "shadow_output_marker_received": True,
            "shadow_steering_pwm": int(shadow.servo1_raw),
            "shadow_throttle_pwm": int(shadow.servo3_raw),
            "physical_outputs_verified": False,
            "physical_write_expected": 0,
            "autonomous_motion_verified": False,
            "autonomous_mission_completion_verified": False,
            "sitl_autonomy_sequence_required": True,
            "gnss_fix_type": int(gps.fix_type),
            "gnss_accuracy_verified": False,
        }
        if args.output is not None:
            output = args.output if args.output.is_absolute() else root / args.output
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(
                json.dumps(evidence, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
    except (common.CheckError, ImportError, OSError, UnicodeError) as error:
        restoration_error = None
        hold_error = None
        if link is not None and system is not None and component is not None:
            try:
                from pymavlink import mavutil
                hold_mode = mavutil.mode_mapping_byname(
                    mavutil.mavlink.MAV_TYPE_GROUND_ROVER
                )["HOLD"]
                _, hold_entered = request_mode(
                    link, mavutil.mavlink, system, component,
                    hold_mode, diagnostics,
                )
                if not hold_entered:
                    raise common.CheckError("HOLD mode was not restored")
            except (common.CheckError, ImportError, OSError) as mode_error:
                hold_error = mode_error
        if link is not None and mission_modified and backup is not None:
            try:
                from pymavlink import mavutil
                upload_mission(
                    link, mavutil.mavlink, system, component,
                    backup, diagnostics,
                )
                restored = download_mission(
                    link, mavutil.mavlink, system, component, diagnostics
                )
                if not missions_equal(backup, restored):
                    raise common.CheckError(
                        "original mission restoration did not verify"
                    )
                mission_restored = True
                mission_modified = False
            except (common.CheckError, OSError) as restore_error:
                restoration_error = restore_error
        print(
            "spresense_m1_rover_gcs_sequence=FAIL "
            f"reason={error} mission_restored={str(mission_restored).lower()}",
            file=sys.stderr,
        )
        if restoration_error is not None:
            print(
                f"mission_restoration=FAIL reason={restoration_error}",
                file=sys.stderr,
            )
        if hold_error is not None:
            print(f"hold_restoration=FAIL reason={hold_error}", file=sys.stderr)
        return 1
    finally:
        if link is not None:
            link.close()

    print(
        "spresense_m1_rover_gcs_sequence=PASS "
        f"mission_items={len(test_mission)} restored=true "
        f"auto_entered_disarmed={str(auto_entered).lower()} "
        "outputs=shadow-only physical_writes=0"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
