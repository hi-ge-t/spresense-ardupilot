#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

"""Run a non-flight endurance and persistence check on M1 Copter."""

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import time

import m1_copter_serial_check as runtime_checker


SAFE_PARAMETER = "GCS_PID_MASK"
MIN_DURATION_SECONDS = 60.0
MAX_DURATION_SECONDS = 3600.0


def parameter_name(message) -> str:
    name = message.param_id
    if isinstance(name, bytes):
        return name.decode("ascii").rstrip("\x00")
    return str(name).rstrip("\x00")


def choose_test_value(original: float) -> float:
    return 0.0 if int(original) == 1 else 1.0


def wait_heartbeat(link, mavlink, timeout, diagnostics):
    return runtime_checker.wait_message(
        link,
        "HEARTBEAT",
        lambda message: (
            message.autopilot == mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA and
            message.type == mavlink.MAV_TYPE_QUADROTOR
        ),
        timeout,
        diagnostics,
    )


def open_link(args, mavutil):
    link = mavutil.mavlink_connection(
        args.port,
        baud=args.baud,
        source_system=255,
        source_component=mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER,
        autoreconnect=False,
        dialect="ardupilotmega",
    )
    runtime_checker.tolerate_transient_zero_reads(link)
    return link


def send_gcs_heartbeat(link, mavlink) -> None:
    link.mav.heartbeat_send(
        mavlink.MAV_TYPE_GCS,
        mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        mavlink.MAV_STATE_ACTIVE,
    )


def require_disarmed(heartbeat, mavlink) -> None:
    if heartbeat.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
        raise runtime_checker.CheckError("target advertised armed state")


def read_parameter(
    link, system, component, name, diagnostics, attempts=3
):
    for _ in range(attempts):
        link.mav.param_request_read_send(
            system, component, name.encode("ascii"), -1
        )
        try:
            return runtime_checker.wait_message(
                link,
                "PARAM_VALUE",
                lambda message: (
                    message.get_srcSystem() == system and
                    parameter_name(message) == name
                ),
                5.0,
                diagnostics,
            )
        except runtime_checker.CheckError:
            continue
    raise runtime_checker.CheckError(f"parameter unavailable: {name}")


def write_parameter(
    link, system, component, name, value, parameter_type, diagnostics
):
    for _ in range(3):
        link.mav.param_set_send(
            system,
            component,
            name.encode("ascii"),
            float(value),
            parameter_type,
        )
        try:
            message = runtime_checker.wait_message(
                link,
                "PARAM_VALUE",
                lambda candidate: (
                    candidate.get_srcSystem() == system and
                    parameter_name(candidate) == name and
                    abs(float(candidate.param_value) - float(value)) < 0.01
                ),
                5.0,
                diagnostics,
            )
            return message
        except runtime_checker.CheckError:
            continue
    raise runtime_checker.CheckError(
        f"parameter write not acknowledged: {name}={value}"
    )


def wait_running(args, mavutil, link, diagnostics):
    heartbeat = wait_heartbeat(
        link, mavutil.mavlink, args.startup_timeout, diagnostics
    )
    require_disarmed(heartbeat, mavutil.mavlink)
    send_gcs_heartbeat(link, mavutil.mavlink)
    return heartbeat


def reboot_and_open(args, mavutil, manifest):
    runtime_checker.check_port(args.port)
    runtime_checker.reset_target(args.port, args.baud)
    link = open_link(args, mavutil)
    diagnostics = bytearray()
    try:
        heartbeat = runtime_checker.wait_copter_startup(
            link,
            mavutil.mavlink,
            args.startup_timeout,
            diagnostics,
        )
        require_disarmed(heartbeat, mavutil.mavlink)
        runtime_commit = runtime_checker.require_runtime_identity(
            diagnostics, manifest["project_commit"]
        )
        send_gcs_heartbeat(link, mavutil.mavlink)
        return link, heartbeat, diagnostics, runtime_commit
    except Exception:
        link.close()
        raise


def restore_parameter_best_effort(
    args, mavutil, manifest, original, parameter_type
):
    link = None
    diagnostics = bytearray()
    try:
        runtime_checker.check_port(args.port)
        link = open_link(args, mavutil)
        heartbeat = wait_running(args, mavutil, link, diagnostics)
        write_parameter(
            link,
            heartbeat.get_srcSystem(),
            heartbeat.get_srcComponent(),
            SAFE_PARAMETER,
            original,
            parameter_type,
            diagnostics,
        )
        time.sleep(3.0)
        link.close()
        link = None
        link, heartbeat, diagnostics, _ = reboot_and_open(
            args, mavutil, manifest
        )
        restored = read_parameter(
            link,
            heartbeat.get_srcSystem(),
            heartbeat.get_srcComponent(),
            SAFE_PARAMETER,
            diagnostics,
        )
        if abs(float(restored.param_value) - original) >= 0.01:
            raise runtime_checker.CheckError(
                f"restored value did not persist: {restored.param_value}"
            )
        return link, heartbeat, diagnostics
    except Exception:
        if link is not None:
            link.close()
        raise


def verify_parameter_persistence(args, mavutil, manifest, link, heartbeat):
    diagnostics = bytearray()
    system = heartbeat.get_srcSystem()
    component = heartbeat.get_srcComponent()
    original_message = read_parameter(
        link, system, component, SAFE_PARAMETER, diagnostics
    )
    original = float(original_message.param_value)
    parameter_type = int(original_message.param_type)
    test_value = choose_test_value(original)
    changed = False
    result = {
        "parameter": SAFE_PARAMETER,
        "original_value": original,
        "test_value": test_value,
        "write_acknowledged": False,
        "test_value_persisted_after_reboot": False,
        "original_value_restored_after_reboot": False,
    }
    try:
        write_parameter(
            link,
            system,
            component,
            SAFE_PARAMETER,
            test_value,
            parameter_type,
            diagnostics,
        )
        changed = True
        result["write_acknowledged"] = True
        time.sleep(3.0)
        link.close()
        link = None

        link, heartbeat, diagnostics, runtime_commit = reboot_and_open(
            args, mavutil, manifest
        )
        persisted = read_parameter(
            link,
            heartbeat.get_srcSystem(),
            heartbeat.get_srcComponent(),
            SAFE_PARAMETER,
            diagnostics,
        )
        if abs(float(persisted.param_value) - test_value) >= 0.01:
            raise runtime_checker.CheckError(
                f"test value did not persist: {persisted.param_value}"
            )
        result["test_value_persisted_after_reboot"] = True

        write_parameter(
            link,
            heartbeat.get_srcSystem(),
            heartbeat.get_srcComponent(),
            SAFE_PARAMETER,
            original,
            parameter_type,
            diagnostics,
        )
        time.sleep(3.0)
        link.close()
        link = None

        link, heartbeat, diagnostics, runtime_commit = reboot_and_open(
            args, mavutil, manifest
        )
        restored = read_parameter(
            link,
            heartbeat.get_srcSystem(),
            heartbeat.get_srcComponent(),
            SAFE_PARAMETER,
            diagnostics,
        )
        if abs(float(restored.param_value) - original) >= 0.01:
            raise runtime_checker.CheckError(
                f"original value did not persist: {restored.param_value}"
            )
        changed = False
        result["original_value_restored_after_reboot"] = True
        result["runtime_commit_after_restore"] = runtime_commit
        return link, heartbeat, result
    except Exception as error:
        if link is not None:
            link.close()
            link = None
        if changed:
            try:
                link, heartbeat, _ = restore_parameter_best_effort(
                    args, mavutil, manifest, original, parameter_type
                )
                result["original_value_restored_after_reboot"] = True
                link.close()
                link = None
            except Exception as restore_error:
                raise runtime_checker.CheckError(
                    f"parameter test failed: {error}; RESTORE FAILED: "
                    f"{restore_error}"
                ) from restore_error
        raise runtime_checker.CheckError(
            f"parameter persistence test failed; original restored: {error}"
        ) from error


def request_interval(
    link, mavlink, system, component, message_id, interval_us, diagnostics
):
    link.mav.command_long_send(
        system,
        component,
        mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        message_id,
        interval_us,
        0,
        0,
        0,
        0,
        0,
    )
    acknowledgement = runtime_checker.wait_message(
        link,
        "COMMAND_ACK",
        lambda message: (
            message.get_srcSystem() == system and
            message.command == mavlink.MAV_CMD_SET_MESSAGE_INTERVAL
        ),
        5.0,
        diagnostics,
    )
    if acknowledgement.result != mavlink.MAV_RESULT_ACCEPTED:
        raise runtime_checker.CheckError(
            f"message interval rejected: id={message_id} "
            f"result={acknowledgement.result}"
        )


def new_monitor_state(duration_seconds):
    return {
        "duration_requested_seconds": duration_seconds,
        "duration_observed_seconds": 0.0,
        "heartbeat_count": 0,
        "heartbeat_max_gap_seconds": 0.0,
        "armed_observed": False,
        "gps": {
            "message_count": 0,
            "maximum_fix_type": 0,
            "maximum_satellites_visible": 0,
            "last_fix_type": None,
            "last_satellites_visible": None,
        },
        "raw_imu": {
            "message_count": 0,
            "first_time_usec": None,
            "last_time_usec": None,
            "minimum_axes": [None] * 6,
            "maximum_axes": [None] * 6,
        },
        "ekf_status": {
            "message_count": 0,
            "last_flags": None,
            "flags_observed_or": 0,
        },
        "ahrs2": {"message_count": 0},
        "attitude": {"message_count": 0},
        "meminfo": {
            "message_count": 0,
            "minimum_freemem": None,
            "minimum_freemem32": None,
        },
        "sys_status": {
            "message_count": 0,
            "maximum_load": None,
            "last_sensors_present": None,
            "last_sensors_enabled": None,
            "last_sensors_health": None,
        },
        "vibration": {
            "message_count": 0,
            "maximum_xyz": [0.0, 0.0, 0.0],
            "maximum_clipping": [0, 0, 0],
        },
        "statustext": [],
    }


def update_minimum(current, value):
    return value if current is None else min(current, value)


def update_maximum(current, value):
    return value if current is None else max(current, value)


def update_monitor_state(state, message, mavlink, now, last_heartbeat):
    message_type = message.get_type()
    if message_type == "HEARTBEAT":
        state["heartbeat_count"] += 1
        if last_heartbeat is not None:
            state["heartbeat_max_gap_seconds"] = max(
                state["heartbeat_max_gap_seconds"], now - last_heartbeat
            )
        if message.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            state["armed_observed"] = True
        return now
    if message_type == "GPS_RAW_INT":
        gps = state["gps"]
        gps["message_count"] += 1
        gps["maximum_fix_type"] = max(
            gps["maximum_fix_type"], int(message.fix_type)
        )
        gps["maximum_satellites_visible"] = max(
            gps["maximum_satellites_visible"],
            int(message.satellites_visible),
        )
        gps["last_fix_type"] = int(message.fix_type)
        gps["last_satellites_visible"] = int(message.satellites_visible)
    elif message_type == "RAW_IMU":
        imu = state["raw_imu"]
        axes = [int(value) for value in (
            message.xacc, message.yacc, message.zacc,
            message.xgyro, message.ygyro, message.zgyro,
        )]
        imu["message_count"] += 1
        if imu["first_time_usec"] is None:
            imu["first_time_usec"] = int(message.time_usec)
        imu["last_time_usec"] = int(message.time_usec)
        for index, value in enumerate(axes):
            imu["minimum_axes"][index] = update_minimum(
                imu["minimum_axes"][index], value
            )
            imu["maximum_axes"][index] = update_maximum(
                imu["maximum_axes"][index], value
            )
    elif message_type == "EKF_STATUS_REPORT":
        ekf = state["ekf_status"]
        ekf["message_count"] += 1
        ekf["last_flags"] = int(message.flags)
        ekf["flags_observed_or"] |= int(message.flags)
    elif message_type == "AHRS2":
        state["ahrs2"]["message_count"] += 1
        state["ahrs2"]["last"] = message.to_dict()
    elif message_type == "ATTITUDE":
        state["attitude"]["message_count"] += 1
        state["attitude"]["last"] = message.to_dict()
    elif message_type == "MEMINFO":
        memory = state["meminfo"]
        memory["message_count"] += 1
        memory["minimum_freemem"] = update_minimum(
            memory["minimum_freemem"], int(message.freemem)
        )
        memory["minimum_freemem32"] = update_minimum(
            memory["minimum_freemem32"], int(message.freemem32)
        )
    elif message_type == "SYS_STATUS":
        status = state["sys_status"]
        status["message_count"] += 1
        status["maximum_load"] = update_maximum(
            status["maximum_load"], int(message.load)
        )
        status["last_sensors_present"] = int(
            message.onboard_control_sensors_present
        )
        status["last_sensors_enabled"] = int(
            message.onboard_control_sensors_enabled
        )
        status["last_sensors_health"] = int(
            message.onboard_control_sensors_health
        )
    elif message_type == "VIBRATION":
        vibration = state["vibration"]
        vibration["message_count"] += 1
        values = [
            float(message.vibration_x),
            float(message.vibration_y),
            float(message.vibration_z),
        ]
        clips = [
            int(message.clipping_0),
            int(message.clipping_1),
            int(message.clipping_2),
        ]
        for index in range(3):
            vibration["maximum_xyz"][index] = max(
                vibration["maximum_xyz"][index], values[index]
            )
            vibration["maximum_clipping"][index] = max(
                vibration["maximum_clipping"][index], clips[index]
            )
    elif message_type == "STATUSTEXT":
        text = str(message.text).rstrip("\x00")
        if text and text not in state["statustext"]:
            state["statustext"].append(text)
            del state["statustext"][:-64]
    return last_heartbeat


def monitor_runtime(
    args, mavutil, link, heartbeat, diagnostics
):
    system = heartbeat.get_srcSystem()
    component = heartbeat.get_srcComponent()
    requests = (
        (mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 1000000),
        (mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU, 200000),
        (mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT, 1000000),
        (mavutil.mavlink.MAVLINK_MSG_ID_AHRS2, 1000000),
        (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 500000),
        (mavutil.mavlink.MAVLINK_MSG_ID_MEMINFO, 1000000),
        (mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 1000000),
        (mavutil.mavlink.MAVLINK_MSG_ID_VIBRATION, 1000000),
    )
    for message_id, interval_us in requests:
        request_interval(
            link,
            mavutil.mavlink,
            system,
            component,
            message_id,
            interval_us,
            diagnostics,
        )

    state = new_monitor_state(args.duration_seconds)
    started = time.monotonic()
    deadline = started + args.duration_seconds
    next_gcs_heartbeat = started
    last_target_heartbeat = started
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now - last_target_heartbeat > 10.0:
            raise runtime_checker.CheckError(
                "Copter heartbeat gap exceeded 10 seconds"
            )
        if now >= next_gcs_heartbeat:
            send_gcs_heartbeat(link, mavutil.mavlink)
            next_gcs_heartbeat = now + 1.0
        message = link.recv_match(blocking=True, timeout=0.5)
        now = time.monotonic()
        if runtime_checker.capture_bad_data(message, diagnostics):
            continue
        if message is None or message.get_srcSystem() != system:
            continue
        last_target_heartbeat = update_monitor_state(
            state,
            message,
            mavutil.mavlink,
            now,
            last_target_heartbeat,
        )
        if state["armed_observed"]:
            raise runtime_checker.CheckError(
                "target advertised armed state during endurance check"
            )
    finished = time.monotonic()
    state["heartbeat_max_gap_seconds"] = max(
        state["heartbeat_max_gap_seconds"],
        finished - last_target_heartbeat,
    )
    state["duration_observed_seconds"] = finished - started
    validate_monitor_state(state)
    state["gnss_fix_verified"] = (
        state["gps"]["maximum_fix_type"] >= 3
    )
    state["ekf_status_observed"] = (
        state["ekf_status"]["message_count"] > 0
    )
    state["imu_orientation_verified"] = False
    state["timing_qualified"] = False
    return state


def validate_monitor_state(state) -> None:
    duration = float(state["duration_observed_seconds"])
    requested = float(state["duration_requested_seconds"])
    if duration + 1.0 < requested:
        raise runtime_checker.CheckError("runtime monitor ended early")
    minimum_heartbeats = max(2, int(requested / 5.0))
    if state["heartbeat_count"] < minimum_heartbeats:
        raise runtime_checker.CheckError("insufficient Copter heartbeats")
    if state["heartbeat_max_gap_seconds"] > 10.0:
        raise runtime_checker.CheckError(
            "Copter heartbeat gap exceeded 10 seconds"
        )
    if state["armed_observed"]:
        raise runtime_checker.CheckError("armed state observed")
    imu = state["raw_imu"]
    if imu["message_count"] < 2:
        raise runtime_checker.CheckError("insufficient RAW_IMU samples")
    if imu["last_time_usec"] <= imu["first_time_usec"]:
        raise runtime_checker.CheckError("RAW_IMU timestamp did not advance")
    if not any(
        value != 0 for value in imu["minimum_axes"]
        if value is not None
    ):
        raise runtime_checker.CheckError("RAW_IMU remained all-zero")
    if not any(
        minimum != maximum for minimum, maximum in zip(
            imu["minimum_axes"], imu["maximum_axes"]
        )
    ):
        raise runtime_checker.CheckError("RAW_IMU axes did not change")
    if state["gps"]["message_count"] < 1:
        raise runtime_checker.CheckError("GPS_RAW_INT was not observed")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--startup-timeout", type=float, default=150.0
    )
    parser.add_argument(
        "--duration-seconds", type=float, default=1800.0
    )
    parser.add_argument(
        "--artifact-dir",
        type=Path,
        default=Path("build/spresense-m1-copter-link-artifacts"),
    )
    parser.add_argument(
        "--ack-param-write",
        help=f"must be exactly {SAFE_PARAMETER} to run persistence test",
    )
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(root / "modules/mavlink"))
    link = None
    try:
        from pymavlink import mavutil

        if not (
            MIN_DURATION_SECONDS <= args.duration_seconds <=
            MAX_DURATION_SECONDS
        ):
            raise runtime_checker.CheckError(
                "duration must be in the range 60..3600 seconds"
            )
        if args.startup_timeout < 30.0 or args.startup_timeout > 240.0:
            raise runtime_checker.CheckError(
                "startup timeout must be in the range 30..240 seconds"
            )
        if args.ack_param_write not in (None, SAFE_PARAMETER):
            raise runtime_checker.CheckError(
                f"parameter write acknowledgement must be {SAFE_PARAMETER}"
            )
        artifact_dir = (
            args.artifact_dir if args.artifact_dir.is_absolute()
            else root / args.artifact_dir
        )
        manifest = runtime_checker.read_manifest(
            artifact_dir / "ARTIFACTS.manifest"
        )
        runtime_checker.check_port(args.port)
        link = open_link(args, mavutil)
        diagnostics = bytearray()
        heartbeat = wait_running(
            args, mavutil, link, diagnostics
        )

        persistence = {
            "parameter": SAFE_PARAMETER,
            "performed": False,
        }
        if args.ack_param_write == SAFE_PARAMETER:
            link, heartbeat, persistence = verify_parameter_persistence(
                args, mavutil, manifest, link, heartbeat
            )
            persistence["performed"] = True
            diagnostics = bytearray()

        monitor = monitor_runtime(
            args, mavutil, link, heartbeat, diagnostics
        )
        evidence = {
            "format": "spresense-m1-copter-bench-v1",
            "captured_at": datetime.now(timezone.utc).isoformat(),
            "port": args.port,
            "baud": args.baud,
            "profile": manifest["profile"],
            "project_commit": manifest["project_commit"],
            "image_sha256": manifest["artifact.nuttx.spk.sha256"],
            "outputs_enabled": False,
            "physical_output_writes_expected": 0,
            "arming_allowed": False,
            "parameter_persistence": persistence,
            "runtime_monitor": monitor,
            "flight_verified": False,
        }
        if args.output is not None:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(
                json.dumps(evidence, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
    except (
        runtime_checker.CheckError,
        ImportError,
        OSError,
        UnicodeError,
    ) as error:
        print(
            f"spresense_m1_copter_bench=FAIL reason={error}",
            file=sys.stderr,
        )
        return 1
    finally:
        if link is not None:
            link.close()

    ekf = (
        "observed" if monitor["ekf_status_observed"]
        else "not-observed"
    )
    gnss = "FIX" if monitor["gnss_fix_verified"] else "NO_FIX"
    parameter = "PASS" if persistence.get(
        "original_value_restored_after_reboot", False
    ) else "not-run"
    print(
        "spresense_m1_copter_bench=PASS "
        f"duration_s={monitor['duration_observed_seconds']:.1f} "
        f"param_persistence={parameter} ekf={ekf} gnss={gnss} "
        "imu=live armed=false"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
