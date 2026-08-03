#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

"""Verify output-disabled M1 Copter heartbeat and hard ARM rejection."""

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import stat
import subprocess
import sys
import time


class CheckError(RuntimeError):
    pass


def read_manifest(path: Path) -> dict[str, str]:
    values = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        key, separator, value = line.partition("=")
        if not separator or key in values:
            raise CheckError(f"invalid artifact manifest line: {line}")
        values[key] = value
    required = {
        "profile": "spresense-m1-copter-link",
        "project_tree": "clean",
        "m1.copter.full": "true",
        "m1.copter.entry": "arducopter_spresense_main",
        "m1.gnss.hal_integration": "AP_GPS_Spresense",
        "m1.pwbimu.hal_integration": "AP_InertialSensor_Spresense",
        "m1.sensor.hal_integration": "GNSS+INS",
        "m1.outputs": "disabled",
        "m1.arming": "always-denied",
        "m1.physical_write_expected": "0",
        "m1.flight_ready": "false",
    }
    for key, expected in required.items():
        if values.get(key) != expected:
            raise CheckError(f"artifact contract mismatch: {key}={expected}")
    return values


def wait_message(link, message_type, predicate, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        message = link.recv_match(type=message_type, blocking=True, timeout=0.5)
        if message is not None and predicate(message):
            return message
    raise CheckError(f"timeout waiting for {message_type}")


def check_port(port: str) -> None:
    try:
        mode = os.stat(port).st_mode
    except OSError as error:
        raise CheckError(f"serial port unavailable: {port}") from error
    if not stat.S_ISCHR(mode):
        raise CheckError(f"not a character device: {port}")
    if sys.platform == "darwin" and not port.startswith("/dev/cu."):
        raise CheckError("use a /dev/cu.* device on macOS")
    if "Bluetooth" in port or "debug-console" in port:
        raise CheckError(f"refusing non-Spresense port: {port}")
    if subprocess.run(
        ["lsof", port], check=False, capture_output=True
    ).stdout:
        raise CheckError(f"serial port is already open: {port}")


def request_arm(link, mavlink, system, component, force_value):
    link.mav.command_long_send(
        system,
        component,
        mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,
        force_value,
        0,
        0,
        0,
        0,
        0,
    )
    acknowledgement = wait_message(
        link,
        "COMMAND_ACK",
        lambda value: (
            value.get_srcSystem() == system and
            value.command == mavlink.MAV_CMD_COMPONENT_ARM_DISARM
        ),
        5.0,
    )
    if acknowledgement.result != mavlink.MAV_RESULT_FAILED:
        raise CheckError(
            "ARM did not reach the expected fail-closed Copter path: "
            f"result={acknowledgement.result}"
        )
    heartbeat = wait_message(
        link,
        "HEARTBEAT",
        lambda value: value.get_srcSystem() == system,
        3.0,
    )
    if heartbeat.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
        raise CheckError("target advertised armed state after ARM rejection")
    return int(acknowledgement.result)


def request_message(link, mavlink, system, component, message_id):
    link.mav.command_long_send(
        system,
        component,
        mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        message_id,
        200000,
        0,
        0,
        0,
        0,
        0,
    )
    acknowledgement = wait_message(
        link,
        "COMMAND_ACK",
        lambda value: (
            value.get_srcSystem() == system and
            value.command == mavlink.MAV_CMD_SET_MESSAGE_INTERVAL
        ),
        5.0,
    )
    if acknowledgement.result != mavlink.MAV_RESULT_ACCEPTED:
        raise CheckError(
            "message interval request rejected: "
            f"message_id={message_id} result={acknowledgement.result}"
        )


def verify_sensor_messages(link, mavlink, system, component):
    request_message(
        link,
        mavlink,
        system,
        component,
        mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,
    )
    gps = wait_message(
        link,
        "GPS_RAW_INT",
        lambda value: value.get_srcSystem() == system,
        15.0,
    )
    if gps.fix_type < mavlink.GPS_FIX_TYPE_NO_FIX:
        raise CheckError(
            "AP_GPS did not report the CXD5610 backend: "
            f"fix_type={gps.fix_type}"
        )

    request_message(
        link,
        mavlink,
        system,
        component,
        mavlink.MAVLINK_MSG_ID_RAW_IMU,
    )
    first = wait_message(
        link,
        "RAW_IMU",
        lambda value: value.get_srcSystem() == system,
        10.0,
    )
    second = wait_message(
        link,
        "RAW_IMU",
        lambda value: (
            value.get_srcSystem() == system and
            value.time_usec > first.time_usec
        ),
        10.0,
    )
    first_axes = (
        first.xacc, first.yacc, first.zacc,
        first.xgyro, first.ygyro, first.zgyro,
    )
    second_axes = (
        second.xacc, second.yacc, second.zacc,
        second.xgyro, second.ygyro, second.zgyro,
    )
    if not any(first_axes) or not any(second_axes):
        raise CheckError("AP_InertialSensor reported an all-zero IMU sample")
    if first_axes == second_axes:
        raise CheckError("AP_InertialSensor IMU sample did not change")
    return gps, first, second


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--startup-timeout",
        type=float,
        default=90.0,
        help="seconds to wait for the first Copter heartbeat",
    )
    parser.add_argument(
        "--artifact-dir",
        type=Path,
        default=Path("build/spresense-m1-copter-link-artifacts"),
    )
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(root / "modules/mavlink"))
    link = None
    try:
        from pymavlink import mavutil

        if args.startup_timeout < 10.0 or args.startup_timeout > 180.0:
            raise CheckError("startup timeout must be in the range 10..180")

        artifact_dir = (
            args.artifact_dir if args.artifact_dir.is_absolute()
            else root / args.artifact_dir
        )
        manifest = read_manifest(artifact_dir / "ARTIFACTS.manifest")
        check_port(args.port)
        link = mavutil.mavlink_connection(
            args.port,
            baud=args.baud,
            source_system=255,
            source_component=mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER,
            autoreconnect=False,
            dialect="ardupilotmega",
        )
        heartbeat = wait_message(
            link,
            "HEARTBEAT",
            lambda value: (
                value.autopilot ==
                mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA
            ),
            args.startup_timeout,
        )
        if heartbeat.type != mavutil.mavlink.MAV_TYPE_QUADROTOR:
            raise CheckError(f"unexpected vehicle type: {heartbeat.type}")
        if heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            raise CheckError("target initially advertises armed state")
        target_system = heartbeat.get_srcSystem()
        target_component = heartbeat.get_srcComponent()

        link.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            mavutil.mavlink.MAV_STATE_ACTIVE,
        )
        gps, first_imu, second_imu = verify_sensor_messages(
            link,
            mavutil.mavlink,
            target_system,
            target_component,
        )
        normal_result = request_arm(
            link, mavutil.mavlink, target_system, target_component, 0
        )
        forced_result = request_arm(
            link, mavutil.mavlink, target_system, target_component, 2989
        )

        evidence = {
            "format": "spresense-m1-copter-runtime-v2",
            "captured_at": datetime.now(timezone.utc).isoformat(),
            "port": args.port,
            "baud": args.baud,
            "startup_timeout_seconds": args.startup_timeout,
            "profile": "spresense-m1-copter-link",
            "project_commit": manifest["project_commit"],
            "image_sha256": manifest["artifact.nuttx.spk.sha256"],
            "target_system": target_system,
            "target_component": target_component,
            "autopilot": "ARDUPILOTMEGA",
            "vehicle_type": "QUADROTOR",
            "heartbeat_received": True,
            "normal_arm_result": normal_result,
            "forced_arm_result": forced_result,
            "armed_observed": False,
            "physical_outputs_verified": False,
            "gnss_driver_linked": True,
            "gnss_runtime_verified": True,
            "gnss_fix_type": int(gps.fix_type),
            "gnss_satellites_visible": int(gps.satellites_visible),
            "gnss_accuracy_verified": False,
            "pwbimu_driver_linked": True,
            "pwbimu_runtime_verified": True,
            "pwbimu_first_time_usec": int(first_imu.time_usec),
            "pwbimu_second_time_usec": int(second_imu.time_usec),
            "pwbimu_first_axes": [int(value) for value in (
                first_imu.xacc, first_imu.yacc, first_imu.zacc,
                first_imu.xgyro, first_imu.ygyro, first_imu.zgyro,
            )],
            "pwbimu_second_axes": [int(value) for value in (
                second_imu.xacc, second_imu.yacc, second_imu.zacc,
                second_imu.xgyro, second_imu.ygyro, second_imu.zgyro,
            )],
            "pwbimu_orientation_verified": False,
            "timing_verified": False,
            "flight_verified": False,
        }
        if args.output is not None:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(
                json.dumps(evidence, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
    except (CheckError, ImportError, OSError, UnicodeError) as error:
        print(f"spresense_m1_copter_serial=FAIL reason={error}", file=sys.stderr)
        return 1
    finally:
        if link is not None:
            link.close()

    print(
        "spresense_m1_copter_serial=PASS heartbeat=ARDUPILOTMEGA "
        f"gps_fix_type={gps.fix_type} imu=live "
        f"normal_arm={normal_result} forced_arm={forced_result} armed=false"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
