#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

"""Verify bidirectional M1 MAVLink exchange over the Spresense main USB."""

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
        ["lsof", port],
        check=False,
        capture_output=True,
    ).stdout:
        raise CheckError(f"serial port is already open: {port}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--verify-arm-denied", action="store_true")
    parser.add_argument(
        "--require-pwbimu",
        action="store_true",
        help="require the Multi-IMU profile and a successful startup probe",
    )
    parser.add_argument(
        "--require-gnss",
        action="store_true",
        help="require a successful bounded GNSS Add-on startup sample",
    )
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(root / "modules/mavlink"))
    try:
        from pymavlink import mavutil

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
                value.get_srcSystem() == 1 and
                value.get_srcComponent() == 1 and
                value.autopilot == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA
            ),
            30.0,
        )
        if heartbeat.type != mavutil.mavlink.MAV_TYPE_QUADROTOR:
            raise CheckError(f"unexpected vehicle type: {heartbeat.type}")
        if heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            raise CheckError("target advertises armed state")
        target_system = heartbeat.get_srcSystem()
        target_component = heartbeat.get_srcComponent()

        link.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            mavutil.mavlink.MAV_STATE_ACTIVE,
        )
        link.mav.command_long_send(
            target_system,
            target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        version = wait_message(
            link,
            "AUTOPILOT_VERSION",
            lambda value: value.get_srcSystem() == target_system,
            5.0,
        )
        custom_version = bytes(version.flight_custom_version)
        if args.require_gnss:
            expected_version = b"M1PGN001"
        elif args.require_pwbimu:
            expected_version = b"M1PIM001"
        else:
            expected_version = b"M1GCS001"
        if custom_version != expected_version:
            raise CheckError(
                f"runtime identity mismatch: {custom_version!r}"
            )
        if version.flight_sw_version >> 24 != 4:
            raise CheckError("unexpected ArduPilot major version")

        link.mav.param_request_list_send(target_system, target_component)
        expected_parameters = {
            "M1_OUT_EN": 0.0,
            "M1_GNSS_REQ": 1.0,
            "M1_GNSS_RAM": 1.0,
            "M1_STAGE": 1.0,
        }
        if args.require_gnss:
            expected_parameters["M1_GNSS_OK"] = 1.0
            expected_parameters["M1_GNSS_ERR"] = 0.0
        if args.require_pwbimu:
            expected_parameters.update({
                "M1_IMU_REQ": 1.0,
                "M1_IMU_OK": 1.0,
            })
        parameters = {}
        deadline = time.monotonic() + 5.0
        while (time.monotonic() < deadline and
               len(parameters) < len(expected_parameters)):
            value = link.recv_match(
                type="PARAM_VALUE", blocking=True, timeout=0.5
            )
            if value is None or value.get_srcSystem() != target_system:
                continue
            parameter_id = value.param_id
            if isinstance(parameter_id, bytes):
                parameter_id = parameter_id.decode("ascii").rstrip("\x00")
            parameters[str(parameter_id)] = float(value.param_value)
        if parameters != expected_parameters:
            raise CheckError(f"parameter contract mismatch: {parameters}")
        if ((args.require_pwbimu or args.require_gnss) and
                heartbeat.system_status !=
                mavutil.mavlink.MAV_STATE_STANDBY):
            raise CheckError(
                f"sensor probe not ready: system_status={heartbeat.system_status}"
            )

        arm_result = "not-requested"
        if args.verify_arm_denied:
            link.mav.command_long_send(
                target_system,
                target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1,
                0,
                0,
                0,
                0,
                0,
                0,
            )
            ack = wait_message(
                link,
                "COMMAND_ACK",
                lambda value: (
                    value.get_srcSystem() == target_system and
                    value.command ==
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
                ),
                5.0,
            )
            if ack.result != mavutil.mavlink.MAV_RESULT_DENIED:
                raise CheckError(f"arm result is not DENIED: {ack.result}")
            arm_result = "DENIED"
            later_heartbeat = wait_message(
                link,
                "HEARTBEAT",
                lambda value: value.get_srcSystem() == target_system,
                3.0,
            )
            if (later_heartbeat.base_mode &
                    mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                raise CheckError("target became armed after denied request")

        evidence = {
            "format": "spresense-m1-gcs-runtime-v2",
            "captured_at": datetime.now(timezone.utc).isoformat(),
            "port": args.port,
            "baud": args.baud,
            "target_system": target_system,
            "target_component": target_component,
            "autopilot": "ARDUPILOTMEGA",
            "vehicle_type": "QUADROTOR",
            "runtime_identity": custom_version.decode("ascii"),
            "profile": (
                "spresense-m1-pwbimu-gnss-gcs" if args.require_pwbimu
                else "spresense-m1-gcs"
            ),
            "parameters": parameters,
            "arm_request": arm_result,
            "armed_observed": False,
            "physical_outputs_verified": False,
            "pwbimu_runtime_verified": args.require_pwbimu,
            "gnss_runtime_verified": args.require_gnss,
            "gnss_fix_verified": False,
            "flight_verified": False,
        }
        if args.output is not None:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(
                json.dumps(evidence, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
        link.close()
    except (CheckError, ImportError, OSError, UnicodeError) as error:
        print(f"spresense_m1_gcs_serial=FAIL reason={error}", file=sys.stderr)
        return 1

    print(
        "spresense_m1_gcs_serial=PASS "
        f"heartbeat=ARDUPILOTMEGA version={expected_version.decode('ascii')} "
        f"params={len(expected_parameters)} "
        f"gnss={'PASS' if args.require_gnss else 'not-required'} "
        f"pwbimu={'PASS' if args.require_pwbimu else 'not-required'} "
        f"arm={arm_result}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
