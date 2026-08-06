#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

"""Verify the fail-closed regular-front-steering M1 Rover contract."""

import json
from pathlib import Path
import sys


EXPECTED = {
    "upstream.vehicle": "Rover",
    "upstream.tag": "Rover-4.7.0",
    "upstream.commit": "1511f27194f1dcc3728270883047bdf022b3fd53",
    "sdk.commit": "7fd61b2c03f06a4ff0302b84c755e58c338788b2",
    "target.os": "Sony Spresense SDK NuttX",
    "target.profile": "spresense-m1-rover-link",
    "target.entrypoint": "ardurover_spresense_main",
    "rover.full_vehicle_archive": True,
    "rover.frame": "regular-front-steering",
    "rover.chassis": "Tamiya CC-02",
    "rover.chassis_scale": "1/10",
    "rover.frame_construction": "ladder-frame",
    "rover.motor_layout": "longitudinal-front-mid",
    "rover.drivetrain": "shaft-driven-4WD-single-ESC",
    "rover.drive_transfer": "gearbox-propeller-shafts-front-and-rear",
    "rover.differential_type": "front-and-rear-3-bevel",
    "rover.differential_configuration": "hardware-HOLD-not-inspected",
    "rover.suspension": "front-and-rear-4-link-rigid",
    "rover.dampers": "front-and-rear-CVA-oil",
    "rover.wheelbase_mm": 252,
    "rover.wheelbase_status": "official-cc02m-nominal-user-adopted",
    "rover.official_reference.model": (
        "Tamiya Toyota Land Cruiser 40 CC-02M Item 58715"
    ),
    "rover.official_reference.source_url": (
        "https://www.tamiya.com/japan/products/58715/index.html"
    ),
    "rover.official_reference.status": (
        "official-cc02m-90mm-nominal-user-adopted"
    ),
    "rover.official_reference.wheelbase_class": "CC-02M",
    "rover.official_reference.wheelbase_mm": 252,
    "rover.official_reference.front_track_mm": 164,
    "rover.official_reference.rear_track_mm": 167,
    "rover.official_reference.tire_width_mm": 33,
    "rover.official_reference.tire_diameter_mm": 90,
    "rover.official_reference.kit_standard_pinion_teeth": 16,
    "rover.official_reference.kit_standard_gear_ratio": 17.33,
    "rover.official_reference.supported_gear_ratio_min": 11.09,
    "rover.official_reference.supported_gear_ratio_max": 29.28,
    "rover.official_reference.kit_motor_class": "RS540",
    "rover.official_reference.esc": "separately-required",
    "rover.wheelbase_reference_delta_mm": 0,
    "rover.front_track_mm": 164,
    "rover.rear_track_mm": 167,
    "rover.track_status": "official-cc02m-nominal-user-adopted",
    "rover.tire_diameter_mm": 90,
    "rover.tire_diameter_status": "official-cc02m-nominal-user-adopted",
    "rover.tire_width_mm": 33,
    "rover.tire_width_status": "official-cc02m-nominal-user-adopted",
    "rover.loaded_rolling_circumference_mm": None,
    "rover.installed_pinion_teeth": None,
    "rover.installed_gear_ratio": None,
    "rover.gear_ratio_status": (
        "hardware-HOLD-installed-configuration-unverified"
    ),
    "rover.installed_motor": "13.5T-brushless",
    "rover.installed_motor_type": "brushless",
    "rover.installed_motor_turns": 13.5,
    "rover.installed_motor_status": "user-specified-model-unverified",
    "rover.installed_esc": None,
    "rover.steering_top_view_displacement_mm": 50,
    "rover.steering_displacement_span": "lock-to-lock",
    "rover.steering_displacement_reference": "tire-leading-edge",
    "rover.steering_displacement_status": "user-specified",
    "rover.steering_angle_deg": 30,
    "rover.steering_angle_status": "user-selected-nominal-not-measured",
    "rover.turn_radius_m": 0.436,
    "rover.turn_radius_model": "wheelbase-over-tan-steering-angle",
    "rover.turn_radius_status": "calculated-not-measured",
    "rover.steering_function": "GroundSteering/CH1",
    "rover.throttle_function": "Throttle/CH3",
    "rover.manual_control_axes": "y=steering,z=throttle",
    "rover.shadow_output": "hal-readback-only",
    "rover.autonomy_sequence": (
        "gcs-hardware-dry-run-plus-sitl-drive-mission"
    ),
    "rover.scheduler": "nuttx-pthread",
    "rover.loop_rate_default_hz": 100,
    "rover.sensor_hal_integration": "GNSS+INS",
    "rover.runtime": "hardware-HOLD",
    "rover.vehicle_ready": False,
    "gnss.builtin": "disabled",
    "gnss.addon": "required",
    "gnss.device": "/dev/gps2",
    "gnss.ram": "required-complete-heap",
    "gnss.runtime_fallback": "disabled",
    "pwbimu.addon": "required",
    "pwbimu.device": "/dev/imu0",
    "pwbimu.bus": "SPI5",
    "pwbimu.runtime_fallback": "disabled",
    "safety.arming": "rover-path-always-reject",
    "safety.actuator_driver": "reject-only-no-physical-backend",
    "safety.outputs": "disabled",
    "safety.physical_write_expected": 0,
    "gcs.heartbeat_vehicle_type": "GROUND_ROVER",
    "gcs.manual_control": "accepted-for-dry-run-only",
    "gcs.mission_protocol": "reversible-upload-download-restore",
    "gcs.auto_mode": "request-only-while-disarmed",
    "gcs.shadow_output": "SERVO_OUTPUT_RAW",
    "gcs.autonomous_motion": "sitl-only",
    "storage.development": "microSD",
    "storage.mount_policy": "explicit-fixed-device",
    "storage.final_candidate": "eMMC",
    "storage.automatic_fallback": "disabled",
    "distribution.binary": "disabled",
    "evidence.physical_steering": "out-of-scope",
    "evidence.physical_throttle": "out-of-scope",
    "evidence.driving": "out-of-scope",
}


def nested_value(document, dotted_key):
    value = document
    for component in dotted_key.split("."):
        value = value[component]
    return value


def require_tokens(failures, label, text, tokens):
    for token in tokens:
        if token not in text:
            failures.append(f"{label}-token-missing={token}")


def main() -> int:
    root = Path(__file__).resolve().parents[2]
    document = json.loads(
        (root / "spresense-m1-rover-manifest.json").read_text(
            encoding="utf-8"
        )
    )
    failures = []
    for key, expected in EXPECTED.items():
        try:
            actual = nested_value(document, key)
        except KeyError:
            failures.append(f"{key}=missing")
            continue
        if actual != expected:
            failures.append(f"{key}={actual!r} expected={expected!r}")

    board = (root / "libraries/AP_HAL/board/spresense.h").read_text(
        encoding="utf-8"
    )
    require_tokens(
        failures,
        "board",
        board,
        (
            "#define HAL_INS_DEFAULT HAL_INS_SPRESENSE",
            "#define HAL_GPS1_TYPE_DEFAULT 27",
            "#define SCHEDULER_DEFAULT_LOOP_RATE 100",
            "#define HAL_SPRESENSE_OUTPUT_DISABLED 1",
            "#define HAL_NUM_CAN_IFACES 0",
            "#define HAL_LOGGING_FILESYSTEM_ENABLED 0",
        ),
    )

    arming = (root / "Rover/AP_Arming_Rover.cpp").read_text(
        encoding="utf-8"
    )
    require_tokens(
        failures,
        "arming",
        arming,
        (
            "HAL_SPRESENSE_OUTPUT_DISABLED",
            '"Arm: Spresense outputs disabled"',
            "return false;",
        ),
    )

    parameters = (root / "Rover/Parameters.cpp").read_text(encoding="utf-8")
    require_tokens(
        failures,
        "regular-frame-defaults",
        parameters,
        (
            "SRV_Channels::set_default_function(CH_1, SRV_Channel::k_steering)",
            "SRV_Channels::set_default_function(CH_3, SRV_Channel::k_throttle)",
        ),
    )
    motors = (
        root / "libraries/AR_Motors/AP_MotorsUGV.cpp"
    ).read_text(encoding="utf-8")
    require_tokens(
        failures,
        "regular-frame-output",
        motors,
        (
            "output_regular(armed, ground_speed, _steering, _throttle);",
            "output_throttle(SRV_Channel::k_throttle, throttle);",
            "SRV_Channels::set_output_scaled(SRV_Channel::k_steering, steering);",
        ),
    )
    gcs = (root / "Rover/GCS_MAVLink_Rover.cpp").read_text(encoding="utf-8")
    require_tokens(
        failures,
        "rover-gcs",
        gcs,
        (
            "MAV_TYPE_GROUND_ROVER",
            "manual_override(rover.channel_steer, packet.y",
            "manual_override(rover.channel_throttle, packet.z",
        ),
    )

    hal_root = root / "libraries/AP_HAL_Spresense"
    implementation = "\n".join(
        path.read_text(encoding="utf-8") for path in hal_root.glob("*.cpp")
    )
    for forbidden in ("/dev/pwm", "/dev/dshot", "/dev/can"):
        if forbidden in implementation.lower():
            failures.append(f"physical-output-path={forbidden}")
    require_tokens(
        failures,
        "hal",
        implementation,
        (
            "_guard.request_write",
            "physical_write_count() const",
            "return 0U;",
            "SPRESENSE_M1_OUTPUT=WRITE_REJECTED",
            "SPRESENSE_M1_OUTPUT=SHADOW_ONLY",
            "shadow_period_us",
            'Spresense::UARTDriver serial0_driver("/dev/ttyS0")',
            "SPRESENSE_M1_ROVER_BOOT=LOOP",
        ),
    )

    profile = (
        root / "Tools/spresense/rover_app/configs/output_disabled/defconfig"
    ).read_text(encoding="utf-8")
    require_tokens(
        failures,
        "rover-profile",
        profile,
        (
            "+SPRESENSE_M1_ROVER_LINK=y",
            "+CXD56_GNSS_ADDON=y",
            "+SENSORS_CXD5610_GNSS=y",
            "+SENSORS_CXD5602PWBIMU=y",
            "+CXD56_GNSS_RAM=y",
            "+CXD56_GNSS_HEAP=y",
            "+CXD56_SDIO=y",
            "-FS_AUTOMOUNTER=y",
            '+INIT_ENTRYPOINT="ardurover_spresense_main"',
            "-CXD56_GNSS=y",
            "-CXD56_EMMC=y",
            "-CXD56_PWM=y",
            "-PWM=y",
        ),
    )

    app_makefile = (root / "Tools/spresense/rover_app/Makefile").read_text(
        encoding="utf-8"
    )
    require_tokens(
        failures,
        "rover-app",
        app_makefile,
        (
            "CXXSRCS += SensorBridge.cpp",
            "BIN := $(M1_ROVER_SDK_BUILD_DIR)/libAP_HAL_Spresense_sdk.a",
        ),
    )

    build = (
        root / "Tools/spresense/build_m1_copter_firmware.py"
    ).read_text(encoding="utf-8")
    require_tokens(
        failures,
        "build",
        build,
        (
            '"rover": {',
            '"ardurover_spresense_main"',
            '"build/spresense/lib/bin/libardurover.a"',
            '"build/spresense/lib/libRover_libs.a"',
            '"waf_vehicle": "rover"',
            '"m1.rover.frame"',
            '"m1.rover.chassis"',
            '"tamiya-cc02"',
            '"m1.rover.chassis_scale"',
            '"m1.rover.frame_construction"',
            '"ladder-frame"',
            '"m1.rover.motor_layout"',
            '"longitudinal-front-mid"',
            '"m1.rover.drive_transfer"',
            '"m1.rover.differential_type"',
            '"m1.rover.differential_configuration"',
            '"m1.rover.suspension"',
            '"front-and-rear-4-link-rigid"',
            '"m1.rover.dampers"',
            '"m1.rover.wheelbase_mm"',
            '"m1.rover.official_reference_model"',
            '"m1.rover.official_reference_wheelbase_class"',
            '"m1.rover.official_reference_wheelbase_mm"',
            '"m1.rover.wheelbase_reference_delta_mm"',
            '"m1.rover.official_reference_front_track_mm"',
            '"m1.rover.official_reference_rear_track_mm"',
            '"m1.rover.front_track_mm"',
            '"m1.rover.rear_track_mm"',
            '"m1.rover.track_status"',
            '"m1.rover.tire_diameter_mm"',
            '"m1.rover.official_reference_tire_width_mm"',
            '"m1.rover.tire_width_mm"',
            '"m1.rover.loaded_rolling_circumference_mm"',
            '"m1.rover.official_reference_kit_standard_pinion_teeth"',
            '"m1.rover.official_reference_kit_standard_gear_ratio"',
            '"m1.rover.official_supported_gear_ratio_min"',
            '"m1.rover.official_supported_gear_ratio_max"',
            '"m1.rover.installed_pinion_teeth"',
            '"m1.rover.installed_gear_ratio"',
            '"m1.rover.official_reference_kit_motor_class"',
            '"m1.rover.official_reference_esc"',
            '"m1.rover.installed_motor"',
            '"m1.rover.installed_motor_type"',
            '"m1.rover.installed_motor_turns"',
            '"m1.rover.installed_motor_status"',
            '"m1.rover.installed_esc"',
            '"m1.rover.steering_top_view_displacement_mm"',
            '"m1.rover.steering_displacement_span"',
            '"lock-to-lock"',
            '"m1.rover.steering_displacement_reference"',
            '"tire-leading-edge"',
            '"m1.rover.steering_angle_deg"',
            '"user-selected-nominal-not-measured"',
            '"m1.rover.turn_radius_m"',
            '"wheelbase-over-tan-steering-angle"',
            '"calculated-not-measured"',
            '"m1.rover.shadow_output"',
            '"m1.gcs.autonomy_sequence"',
            'env["SPRESENSE_AP_MAIN"]',
        ),
    )

    gcs_sequence = (
        root / "Tools/spresense/m1_rover_gcs_sequence.py"
    ).read_text(encoding="utf-8")
    require_tokens(
        failures,
        "gcs-sequence",
        gcs_sequence,
        (
            "mission_request_list_send",
            "mission_count_send",
            "mission_item_int_send",
            "MISSION_REQUEST_INT",
            "is_primary_mission_message",
            "MAV_MISSION_TYPE_MISSION",
            "MAV_CMD_DO_SET_MODE",
            "MAVLINK_MSG_ID_SERVO_OUTPUT_RAW",
            "GCS_RUNTIME_MARKERS",
            "gnss_snapshot_received",
            "mission_restored",
            '"autonomous_motion_verified": False',
        ),
    )

    sitl_sequence = (
        root / "Tools/spresense/run_m1_rover_sitl_autonomy.py"
    ).read_text(encoding="utf-8")
    require_tokens(
        failures,
        "sitl-sequence",
        sitl_sequence,
        (
            "test.Rover.DriveMission",
            '"mission_complete_verified": True',
            '"spresense_hardware_verified": False',
            '"driving_verified": False',
        ),
    )

    gcs_sitl_sequence = (
        root / "Tools/spresense/run_m1_rover_gcs_sitl.py"
    ).read_text(encoding="utf-8")
    require_tokens(
        failures,
        "gcs-sitl-sequence",
        gcs_sitl_sequence,
        (
            "real-Rover-SITL-MAVLink-mission-round-trip",
            "MAV_COMP_ID_MISSIONPLANNER",
            '"auto_mode_entered_disarmed": auto_entered',
            '"arm_command_sent": False',
            '"spresense_hardware_verified": False',
            '"physical_outputs_verified": False',
            '"driving_verified": False',
            '"project_tree": git_tree_state(root)',
            "initial_state_restored",
        ),
    )

    documentation = (root / "docs/SPRESENSE_ROVER.md").read_text(
        encoding="utf-8"
    )
    require_tokens(
        failures,
        "documentation",
        documentation,
        (
            "SERVO1_FUNCTION=26",
            "SERVO3_FUNCTION=70",
            "Tamiya CC-02",
            "adopted nominal 252 mm",
            "front 164 mm, rear 167 mm",
            "33 mm width × 90 mm diameter",
            "16T / 17.33:1",
            "11.09:1 to 29.28:1",
            "installed gearing is unverified",
            "RS540 / ESC separately required",
            "13.5T brushless",
            "manufacturer/model/KV are unverified",
            "Tamiya Item 58715",
            "active configuration contract",
            "252 mm",
            "90 mm",
            "50 mm",
            "lock-to-lock",
            "tire leading edge",
            "30 degrees per side",
            "0.436 m",
            "calculated model value, not a measured turning radius",
            "TODO: unmeasured loaded rolling circumference",
            "shaft-driven 4WD",
            "evidence/SPRESENSE_M1_ROVER_20260804.md",
            "evidence/SPRESENSE_M1_ROVER_GCS_AUTONOMY_20260804.md",
            "evidence/SPRESENSE_M1_ROVER_CC02_GEOMETRY_20260806.md",
            "CH1=1800, CH3=1700 dry-run",
            "MISSION_COUNT",
            "28a899ebae",
            "arm_command_sent=false",
            "initial_state_restored=true",
            "GNSS fix/accuracy HOLD",
            "no automatic device, sensor or storage fallback",
            "HAL_SPRESENSE_OUTPUT_DISABLED=1",
            "It does not mean the car can be driven yet.",
        ),
    )

    autonomy_evidence = (
        root / "docs/evidence/SPRESENSE_M1_ROVER_GCS_AUTONOMY_20260804.md"
    ).read_text(encoding="utf-8")
    require_tokens(
        failures,
        "gcs-autonomy-evidence",
        autonomy_evidence,
        (
            "28a899ebae4b81c659378aa5561763ced58a6cba",
            "31a5aaddb177a54e5bedee9f0a6cd351aea361f9",
            "ead174956de5a15486279026b9ae15ae992ec0fb81546489de001aa496bce6ab",
            "aab89f737f40da59fcf24c70ad1298d397e2b9c420ddd2c9bb1ab006a5ec5a7a",
            "arm_command_sent=false",
            "initial_state_restored=true",
            "MISSION_COUNT",
            "no mission on the board was modified",
            "driving_verified=false",
            "not a full power cycle",
            "expected physical write count is zero",
        ),
    )

    geometry_evidence = (
        root / "docs/evidence/SPRESENSE_M1_ROVER_CC02_GEOMETRY_20260806.md"
    ).read_text(encoding="utf-8")
    require_tokens(
        failures,
        "cc02-geometry-evidence",
        geometry_evidence,
        (
            "209624b7dd131a05aa474b7f85160204e9b50511",
            "e7ac35b4249c8c85b7c705ea482d5f19a56d51b033819887a17ea985316156a6",
            "m1.rover.wheelbase_mm=252",
            "m1.rover.wheelbase_status=official-cc02m-nominal-user-adopted",
            "m1.rover.official_reference_wheelbase_class=CC-02M",
            "m1.rover.official_reference_wheelbase_mm=252",
            "m1.rover.official_reference_front_track_mm=164",
            "m1.rover.official_reference_rear_track_mm=167",
            "m1.rover.front_track_mm=164",
            "m1.rover.rear_track_mm=167",
            "m1.rover.track_status=official-cc02m-nominal-user-adopted",
            "m1.rover.tire_diameter_mm=90",
            "m1.rover.tire_diameter_status=official-cc02m-nominal-user-adopted",
            "m1.rover.official_reference_tire_width_mm=33",
            "m1.rover.tire_width_mm=33",
            "m1.rover.tire_width_status=official-cc02m-nominal-user-adopted",
            "m1.rover.official_reference_kit_standard_pinion_teeth=16",
            "m1.rover.official_reference_kit_standard_gear_ratio=17.33",
            "m1.rover.official_supported_gear_ratio_min=11.09",
            "m1.rover.official_supported_gear_ratio_max=29.28",
            "m1.rover.installed_gear_ratio=hardware-HOLD-unverified",
            "m1.rover.official_reference_kit_motor_class=RS540",
            "m1.rover.installed_motor=13.5T-brushless",
            "m1.rover.installed_motor_turns=13.5",
            "m1.rover.installed_esc=hardware-HOLD-unverified",
            "m1.rover.steering_angle_deg=30",
            "m1.rover.turn_radius_m=0.436",
            "calculated-not-measured",
            "hardware-HOLD-unmeasured",
            "No board power, DTR reset, flash or physical",
        ),
    )

    evidence = (
        root / "docs/evidence/SPRESENSE_M1_ROVER_20260804.md"
    ).read_text(encoding="utf-8")
    require_tokens(
        failures,
        "hardware-evidence",
        evidence,
        (
            "a200430c9ae191b9234fcdee318151a99f1b1037",
            "8b225f139959c709cda0adb61e14729bb0ac5198",
            "ed66ab9c347e4436e05177dca756dd09fbd639ab77798af00201e7f1ca67237a",
            "MAV_TYPE_GROUND_ROVER",
            "CH1=1800",
            "CH3=1700",
            "explicitly a no-fix result",
            "not driving evidence",
        ),
    )

    flash = (root / "Tools/flash_spresense.sh").read_text(encoding="utf-8")
    require_tokens(
        failures,
        "flash",
        flash,
        (
            "spresense-m1-rover-link",
            "m1.rover.entry=ardurover_spresense_main",
            "m1.rover.frame=regular-front-steering",
            "m1.rover.shadow_output=hal-readback-only",
            "m1.gcs.mission_protocol=reversible-upload-download-restore",
            "m1.gcs.autonomy_sequence=hardware-dry-run-plus-sitl",
            "m1.outputs.shadow_readback=enabled",
            "m1.outputs=disabled",
            "m1.arming=always-denied",
            "m1.physical_write_expected=0",
            "m1.storage.automatic_fallback=disabled",
        ),
    )

    if failures:
        print(
            "spresense_m1_rover_contract=FAIL " + " ".join(failures),
            file=sys.stderr,
        )
        return 1
    print(
        "spresense_m1_rover_contract=PASS "
        "frame=regular-front-steering outputs=disabled vehicle_ready=false"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
