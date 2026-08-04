#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

"""Verify output-disabled M1 vehicle heartbeat and hard ARM rejection."""

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import stat
import subprocess
import sys
import time


class CheckError(RuntimeError):
    pass


TRANSIENT_ZERO_READ = "device reports readiness to read but returned no data"
VEHICLES = {
    "copter": {
        "profile": "spresense-m1-copter-link",
        "entry_key": "m1.copter.entry",
        "entry": "arducopter_spresense_main",
        "full_key": "m1.copter.full",
        "artifact_directory": "build/spresense-m1-copter-link-artifacts",
        "boot_marker": b"SPRESENSE_M1_COPTER_BOOT=LOOP",
        "runtime_name": b"ArduCopter",
        "mav_type_name": "QUADROTOR",
    },
    "rover": {
        "profile": "spresense-m1-rover-link",
        "entry_key": "m1.rover.entry",
        "entry": "ardurover_spresense_main",
        "full_key": "m1.rover.full",
        "artifact_directory": "build/spresense-m1-rover-link-artifacts",
        "boot_marker": b"SPRESENSE_M1_ROVER_BOOT=LOOP",
        "runtime_name": b"ArduRover",
        "mav_type_name": "GROUND_ROVER",
    },
}


def capture_bad_data(message, diagnostics: bytearray) -> bool:
    if message is None or message.get_type() != "BAD_DATA":
        return False
    data = message.data
    if isinstance(data, str):
        data = data.encode("latin1", errors="replace")
    else:
        data = bytes(data)
    diagnostics.extend(data)
    if len(diagnostics) > 16384:
        del diagnostics[:-16384]
    return True


def tolerate_transient_zero_reads(link) -> None:
    original_recv = link.recv

    def recv(size=None):
        try:
            return original_recv(size)
        except OSError as error:
            if TRANSIENT_ZERO_READ in str(error):
                return b""
            raise

    link.recv = recv


def read_manifest(path: Path, vehicle: str = "copter") -> dict[str, str]:
    values = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        key, separator, value = line.partition("=")
        if not separator or key in values:
            raise CheckError(f"invalid artifact manifest line: {line}")
        values[key] = value
    specification = VEHICLES[vehicle]
    required = {
        "profile": specification["profile"],
        "project_tree": "clean",
        specification["full_key"]: "true",
        specification["entry_key"]: specification["entry"],
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


def wait_message(link, message_type, predicate, timeout, diagnostics):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        message = link.recv_match(blocking=True, timeout=0.5)
        if capture_bad_data(message, diagnostics):
            continue
        if (
            message is not None and
            message.get_type() == message_type and
            predicate(message)
        ):
            return message
    raise CheckError(f"timeout waiting for {message_type}")


def wait_vehicle_startup(
    link, mavlink, timeout, diagnostics, vehicle="copter"
):
    deadline = time.monotonic() + timeout
    heartbeat = None
    marker = VEHICLES[vehicle]["boot_marker"] + b"\n"
    while time.monotonic() < deadline:
        message = link.recv_match(blocking=True, timeout=0.5)
        if message is None:
            continue
        message_type = message.get_type()
        if (
            message_type == "HEARTBEAT" and
            message.autopilot == mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA
        ):
            heartbeat = message
        else:
            capture_bad_data(message, diagnostics)
        if heartbeat is not None and marker in diagnostics:
            return heartbeat
    if heartbeat is None:
        raise CheckError(f"timeout waiting for {vehicle} heartbeat")
    raise CheckError(
        "timeout waiting for " + marker.decode("ascii").strip()
    )


def wait_copter_startup(link, mavlink, timeout, diagnostics):
    return wait_vehicle_startup(
        link, mavlink, timeout, diagnostics, "copter"
    )


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


def reset_target(port: str, baud: int) -> None:
    try:
        import serial

        reset_link = serial.Serial(port, baud, timeout=0.1)
        reset_link.dtr = False
        time.sleep(0.2)
        reset_link.reset_input_buffer()
        reset_link.dtr = True
        reset_link.close()
    except (ImportError, OSError) as error:
        raise CheckError(f"DTR reset failed: {error}") from error


def missing_runtime_markers(
    diagnostics: bytearray, vehicle: str = "copter"
) -> list[str]:
    required = [
        VEHICLES[vehicle]["boot_marker"],
        b"SPRESENSE_M1_PWBIMU=SAMPLE",
        b"SPRESENSE_M1_GNSS=SAMPLE",
        b"SPRESENSE_M1_GNSS=ATTACH",
        b"SPRESENSE_M1_GNSS=CONSUMED",
    ]
    if vehicle == "rover":
        required.append(b"SPRESENSE_M1_OUTPUT=WRITE_REJECTED")
    return [
        marker.decode("ascii") for marker in required
        if marker not in diagnostics
    ]


def require_runtime_markers(
    diagnostics: bytearray, vehicle: str = "copter"
) -> None:
    missing = missing_runtime_markers(diagnostics, vehicle)
    if missing:
        raise CheckError("runtime marker missing: " + " | ".join(missing))


def wait_runtime_markers(
    link, diagnostics: bytearray, timeout: float, vehicle: str = "copter"
) -> None:
    deadline = time.monotonic() + timeout
    while (
        missing_runtime_markers(diagnostics, vehicle) and
        time.monotonic() < deadline
    ):
        message = link.recv_match(blocking=True, timeout=0.5)
        capture_bad_data(message, diagnostics)
    require_runtime_markers(diagnostics, vehicle)


def require_runtime_identity(
    diagnostics: bytearray, project_commit: str, vehicle: str = "copter"
) -> str:
    runtime_name = VEHICLES[vehicle]["runtime_name"]
    match = re.search(
        rb"Init " + runtime_name + rb" [^\r\n]* \(([0-9a-fA-F]{8,40})\)",
        diagnostics,
    )
    if match is None:
        raise CheckError(
            f"runtime {runtime_name.decode('ascii')} commit marker missing"
        )
    runtime_commit = match.group(1).decode("ascii").lower()
    if not project_commit.lower().startswith(runtime_commit):
        raise CheckError(
            "runtime commit does not match artifact manifest: "
            f"runtime={runtime_commit} manifest={project_commit}"
        )
    return runtime_commit


def request_arm(
    link, mavlink, system, component, force_value, diagnostics
):
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
        diagnostics,
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
        diagnostics,
    )
    if heartbeat.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
        raise CheckError("target advertised armed state after ARM rejection")
    return int(acknowledgement.result)


def request_message(
    link, mavlink, system, component, message_id, diagnostics
):
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
        diagnostics,
    )
    if acknowledgement.result != mavlink.MAV_RESULT_ACCEPTED:
        raise CheckError(
            "message interval request rejected: "
            f"message_id={message_id} result={acknowledgement.result}"
        )


def verify_sensor_messages(
    link, mavlink, system, component, diagnostics
):
    request_message(
        link,
        mavlink,
        system,
        component,
        mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,
        diagnostics,
    )
    gps = wait_message(
        link,
        "GPS_RAW_INT",
        lambda value: (
            value.get_srcSystem() == system and
            value.fix_type >= mavlink.GPS_FIX_TYPE_NO_FIX
        ),
        30.0,
        diagnostics,
    )

    request_message(
        link,
        mavlink,
        system,
        component,
        mavlink.MAVLINK_MSG_ID_RAW_IMU,
        diagnostics,
    )
    first = wait_message(
        link,
        "RAW_IMU",
        lambda value: value.get_srcSystem() == system,
        10.0,
        diagnostics,
    )
    second = wait_message(
        link,
        "RAW_IMU",
        lambda value: (
            value.get_srcSystem() == system and
            value.time_usec > first.time_usec
        ),
        10.0,
        diagnostics,
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


def verify_rover_manual_control(
    link, mavlink, system, component, diagnostics
):
    link.mav.manual_control_send(system, 0, 750, 500, 0, 0)
    request_message(
        link,
        mavlink,
        system,
        component,
        mavlink.MAVLINK_MSG_ID_RC_CHANNELS,
        diagnostics,
    )
    channels = wait_message(
        link,
        "RC_CHANNELS",
        lambda value: (
            value.get_srcSystem() == system and
            value.chan1_raw > 1500 and
            value.chan3_raw > 1500
        ),
        5.0,
        diagnostics,
    )
    heartbeat = wait_message(
        link,
        "HEARTBEAT",
        lambda value: value.get_srcSystem() == system,
        3.0,
        diagnostics,
    )
    if heartbeat.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
        raise CheckError("Rover armed while processing dry-run manual control")
    return channels


def main(default_vehicle: str = "copter") -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--vehicle", choices=sorted(VEHICLES), default=default_vehicle
    )
    parser.add_argument(
        "--reset-dtr",
        action="store_true",
        help="reset the target immediately before opening the MAVLink stream",
    )
    parser.add_argument(
        "--startup-timeout",
        type=float,
        default=150.0,
        help="seconds to wait for both Copter heartbeat and HAL LOOP marker",
    )
    parser.add_argument(
        "--sensor-timeout",
        type=float,
        default=120.0,
        help="seconds to wait for GNSS and PWBIMU runtime markers",
    )
    parser.add_argument(
        "--artifact-dir",
        type=Path,
        default=None,
    )
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(root / "modules/mavlink"))
    link = None
    diagnostics = bytearray()
    try:
        from pymavlink import mavutil

        if args.startup_timeout < 30.0 or args.startup_timeout > 240.0:
            raise CheckError("startup timeout must be in the range 30..240")
        if args.sensor_timeout < 30.0 or args.sensor_timeout > 240.0:
            raise CheckError("sensor timeout must be in the range 30..240")

        artifact_argument = args.artifact_dir or Path(
            VEHICLES[args.vehicle]["artifact_directory"]
        )
        artifact_dir = (
            artifact_argument if artifact_argument.is_absolute()
            else root / artifact_argument
        )
        manifest = read_manifest(
            artifact_dir / "ARTIFACTS.manifest", args.vehicle
        )
        check_port(args.port)
        if args.reset_dtr:
            reset_target(args.port, args.baud)
        link = mavutil.mavlink_connection(
            args.port,
            baud=args.baud,
            source_system=255,
            source_component=mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER,
            autoreconnect=False,
            dialect="ardupilotmega",
        )
        tolerate_transient_zero_reads(link)
        heartbeat = wait_vehicle_startup(
            link, mavutil.mavlink, args.startup_timeout, diagnostics,
            args.vehicle,
        )
        runtime_commit = require_runtime_identity(
            diagnostics, manifest["project_commit"], args.vehicle
        )
        expected_type = (
            mavutil.mavlink.MAV_TYPE_QUADROTOR
            if args.vehicle == "copter"
            else mavutil.mavlink.MAV_TYPE_GROUND_ROVER
        )
        if heartbeat.type != expected_type:
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
            diagnostics,
        )
        wait_runtime_markers(
            link, diagnostics, args.sensor_timeout, args.vehicle
        )
        normal_result = request_arm(
            link, mavutil.mavlink, target_system, target_component, 0,
            diagnostics,
        )
        forced_result = request_arm(
            link, mavutil.mavlink, target_system, target_component, 2989,
            diagnostics,
        )
        manual_channels = None
        if args.vehicle == "rover":
            manual_channels = verify_rover_manual_control(
                link,
                mavutil.mavlink,
                target_system,
                target_component,
                diagnostics,
            )
        evidence = {
            "format": f"spresense-m1-{args.vehicle}-runtime-v1",
            "captured_at": datetime.now(timezone.utc).isoformat(),
            "port": args.port,
            "baud": args.baud,
            "startup_timeout_seconds": args.startup_timeout,
            "sensor_timeout_seconds": args.sensor_timeout,
            "reset_dtr": args.reset_dtr,
            "profile": VEHICLES[args.vehicle]["profile"],
            "project_commit": manifest["project_commit"],
            "runtime_commit": runtime_commit,
            "image_sha256": manifest["artifact.nuttx.spk.sha256"],
            "target_system": target_system,
            "target_component": target_component,
            "autopilot": "ARDUPILOTMEGA",
            "vehicle_type": VEHICLES[args.vehicle]["mav_type_name"],
            "heartbeat_received": True,
            "hal_loop_marker_received": True,
            "normal_arm_result": normal_result,
            "forced_arm_result": forced_result,
            "armed_observed": False,
            "physical_outputs_verified": False,
            "output_write_rejection_marker_received": (
                b"SPRESENSE_M1_OUTPUT=WRITE_REJECTED" in diagnostics
            ),
            "regular_front_steering_profile": args.vehicle == "rover",
            "manual_control_rc_override_verified": manual_channels is not None,
            "manual_control_steering_pwm": (
                int(manual_channels.chan1_raw)
                if manual_channels is not None else None
            ),
            "manual_control_throttle_pwm": (
                int(manual_channels.chan3_raw)
                if manual_channels is not None else None
            ),
            "gnss_driver_linked": True,
            "gnss_runtime_verified": True,
            "gnss_sample_marker_received": True,
            "gnss_attached_to_ap_gps": True,
            "gnss_consumed_by_ap_gps": True,
            "gnss_fix_type": int(gps.fix_type),
            "gnss_satellites_visible": int(gps.satellites_visible),
            "gnss_accuracy_verified": False,
            "pwbimu_driver_linked": True,
            "pwbimu_runtime_verified": True,
            "pwbimu_sample_marker_received": True,
            "pwbimu_recovery_attempted": (
                b"SPRESENSE_M1_PWBIMU=RESTART_OK" in diagnostics or
                b"SPRESENSE_M1_PWBIMU=RESTART_FAIL" in diagnostics
            ),
            "pwbimu_recovery_succeeded": (
                b"SPRESENSE_M1_PWBIMU=RESTART_OK" in diagnostics
            ),
            "pwbimu_recovery_exhausted": (
                b"SPRESENSE_M1_PWBIMU=RECOVERY_EXHAUSTED" in diagnostics
            ),
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
        if diagnostics:
            decoded = diagnostics.decode("latin1", errors="replace")
            print(
                f"spresense_m1_{args.vehicle}_diagnostics={decoded!r}",
                file=sys.stderr,
            )
        print(
            f"spresense_m1_{args.vehicle}_serial=FAIL reason={error}",
            file=sys.stderr,
        )
        return 1
    finally:
        if link is not None:
            link.close()

    print(
        f"spresense_m1_{args.vehicle}_serial=PASS "
        "heartbeat=ARDUPILOTMEGA "
        f"gps_fix_type={gps.fix_type} imu=live "
        f"normal_arm={normal_result} forced_arm={forced_result} armed=false"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
