#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

"""Verify the fail-closed M1 full-Copter software contract."""

import json
from pathlib import Path
import sys


EXPECTED = {
    "upstream.vehicle": "Copter",
    "upstream.tag": "Copter-4.7.0",
    "upstream.commit": "1511f27194f1dcc3728270883047bdf022b3fd53",
    "sdk.commit": "7fd61b2c03f06a4ff0302b84c755e58c338788b2",
    "target.os": "Sony Spresense SDK NuttX",
    "target.profile": "spresense-m1-copter-link",
    "target.entrypoint": "arducopter_spresense_main",
    "copter.full_vehicle_archive": True,
    "copter.scheduler": "nuttx-pthread",
    "copter.loop_rate_default_hz": 100,
    "copter.sensor_hal_integration": "GNSS+INS",
    "copter.runtime": "hardware-HOLD",
    "copter.flight_ready": False,
    "gnss.builtin": "disabled",
    "gnss.addon": "required",
    "gnss.device": "/dev/gps2",
    "gnss.ram": "required-complete-heap",
    "gnss.hal_integration": "AP_GPS_Spresense",
    "gnss.runtime_fallback": "disabled",
    "pwbimu.addon": "required",
    "pwbimu.device": "/dev/imu0",
    "pwbimu.bus": "SPI5",
    "pwbimu.pinshare": "eMMC",
    "pwbimu.hal_integration": "AP_InertialSensor_Spresense",
    "pwbimu.orientation": "ROTATION_NONE-hardware-HOLD",
    "pwbimu.runtime_fallback": "disabled",
    "safety.arming": "copter-path-always-reject",
    "safety.actuator_driver": "reject-only-no-physical-backend",
    "safety.outputs": "disabled",
    "safety.physical_write_expected": 0,
    "gcs.transport": "/dev/ttyS0",
    "gcs.baud": 115200,
    "gcs.full_copter": True,
    "gcs.runtime": "hardware-HOLD",
    "storage.development": "microSD",
    "storage.final_candidate": "eMMC",
    "storage.automatic_fallback": "disabled",
    "storage.pwbimu_emmc_coexistence": "hardware-design-HOLD",
    "distribution.binary": "disabled",
    "evidence.sensor_hal_integration": "required",
    "evidence.sensor_runtime": "HOLD",
    "evidence.timing": "HOLD",
    "evidence.gnss_accuracy": "HOLD",
    "evidence.flight": "out-of-scope",
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
        (root / "spresense-m1-copter-manifest.json").read_text(
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
            "#define HAL_NUM_CAN_IFACES 0",
            "#define HAL_INS_DEFAULT HAL_INS_SPRESENSE",
            "#define HAL_GPS1_TYPE_DEFAULT 27",
            "#define SCHEDULER_DEFAULT_LOOP_RATE 100",
            "#define AP_BARO_BACKEND_DEFAULT_ENABLED 0",
            "#define AP_BARO_PROBE_EXTERNAL_I2C_BUSES 0",
            "#define HAL_BARO_ALLOW_INIT_NO_BARO 1",
            "#define HAL_SPRESENSE_OUTPUT_DISABLED 1",
            "#define AP_NETWORKING_ENABLED 0",
            "#define HAL_LOGGING_FILESYSTEM_ENABLED 0",
        ),
    )

    scheduler = (root / "libraries/AP_Scheduler/AP_Scheduler.cpp").read_text(
        encoding="utf-8"
    )
    require_tokens(
        failures,
        "scheduler",
        scheduler,
        (
            "#ifndef SCHEDULER_DEFAULT_LOOP_RATE",
            "#define SCHEDULER_DEFAULT_LOOP_RATE 400",
        ),
    )

    arming = (root / "ArduCopter/AP_Arming_Copter.cpp").read_text(
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

    hal_root = root / "libraries/AP_HAL_Spresense"
    implementation = "\n".join(
        path.read_text(encoding="utf-8") for path in hal_root.glob("*.cpp")
    )
    semaphores = (hal_root / "Semaphores.h").read_text(encoding="utf-8")
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
            "pthread_create",
            "PTHREAD_PRIO_INHERIT",
            'Spresense::UARTDriver serial0_driver("/dev/ttyS0")',
            'SPRESENSE_M1_COPTER_BOOT=HAL',
            'SPRESENSE_M1_COPTER_BOOT=SCHEDULER_FAIL',
            "sensor_bridge_platform_ready",
            "Empty::I2CDeviceManager i2c_manager",
            "Empty::SPIDeviceManager spi_manager",
        ),
    )

    sensor_bridge = (hal_root / "SensorBridge.cpp").read_text(
        encoding="utf-8"
    )
    require_tokens(
        failures,
        "sensor-bridge",
        sensor_bridge,
        (
            "CONFIG_SPRESENSE_M1_COPTER_GNSS_DEVICE",
            "CONFIG_SPRESENSE_M1_COPTER_PWBIMU_DEVICE",
            "CXD56_GNSS_IOCTL_START",
            "GNSS_INIT_PRIORITY = 110",
            "GNSS_INIT_STACK_BYTES = 8192U",
            "gnss_init_thread",
            "start_gnss_init_thread",
            "run_gnss_reader()",
            "gnss_position",
            "PTHREAD_EXPLICIT_SCHED",
            "SNIOC_SSAMPRATE",
            "GNSS_NOTIFICATION_SIGNAL = 18",
            "GNSS_NOTIFICATION_WAIT_MS = 1250U",
            "GNSS_READER_YIELD_US = 1000U",
            "CXD56_GNSS_IOCTL_SIGNAL_SET",
            "nanosleep(&yield_time, &yield_time)",
            "pthread_sigmask(SIG_BLOCK, &mask, nullptr)",
            "return block_gnss_notification();",
            "sigtimedwait",
            "pthread_mutex_trylock(&gnss_sample_mutex)",
            "PWBIMU_STARTUP_POLL_TIMEOUT_MS",
            "ready_to_read(pwbimu_fd, PWBIMU_STARTUP_POLL_TIMEOUT_MS)",
            "O_NONBLOCK",
            "return false;",
        ),
    )
    pwbimu_read = sensor_bridge.split(
        "Spresense::SensorReadStatus Spresense::pwbimu_read", 1
    )[1].split("#else", 1)[0]
    if "ready_to_read(" in pwbimu_read or "poll(" in pwbimu_read:
        failures.append(
            "sensor-bridge: steady-state PWBIMU read must remain nonblocking"
        )
    gps_backend = (root / "libraries/AP_GPS/AP_GPS_Spresense.cpp").read_text(
        encoding="utf-8"
    )
    require_tokens(
        failures,
        "gps-backend",
        gps_backend,
        (
            "Spresense::gnss_read",
            "if (!Spresense::gnss_start())",
            "state.location.lat",
            "state.velocity",
            "return false;",
        ),
    )
    gps_detection = (root / "libraries/AP_GPS/AP_GPS.cpp").read_text(
        encoding="utf-8"
    )
    require_tokens(
        failures,
        "gps-detection",
        gps_detection,
        (
            "case GPS_TYPE_SPRESENSE:",
            "(void)Spresense::gnss_start();",
            "NEW_NOTHROW AP_GPS_Spresense(",
        ),
    )
    ins_backend = (
        root /
        "libraries/AP_InertialSensor/AP_InertialSensor_Spresense.cpp"
    ).read_text(encoding="utf-8")
    require_tokens(
        failures,
        "ins-backend",
        ins_backend,
        (
            "Spresense::pwbimu_read",
            "_notify_new_accel_raw_sample",
            "_notify_new_gyro_raw_sample",
            "ROTATION_NONE",
        ),
    )
    require_tokens(
        failures,
        "semaphore-abi",
        semaphores,
        (
            "uint8_t _mutex_storage[28]",
            "uint8_t _condition_storage[20]",
            "bool _initialized = false;",
        ),
    )
    if "#if defined(__NuttX__)" in semaphores:
        failures.append("semaphore-abi-layout-is-conditional")

    profile = (
        root / "Tools/spresense/copter_app/configs/output_disabled/defconfig"
    ).read_text(encoding="utf-8")
    require_tokens(
        failures,
        "copter-profile",
        profile,
        (
            "+SPRESENSE_M1_COPTER_LINK=y",
            "+CXD56_GNSS_ADDON=y",
            "+SENSORS_CXD5610_GNSS=y",
            "+SENSORS_CXD5610_GNSS_NSIGNALRECEIVERS=4",
            "+SENSORS_CXD5610_GNSS_RX_THREAD_PRIORITY=120",
            "+SENSORS_CXD5610_GNSS_RX_THREAD_STACKSIZE=2048",
            "+SENSORS_CXD5602PWBIMU=y",
            "+CXD56_GNSS_RAM=y",
            "+CXD56_GNSS_HEAP=y",
            '+INIT_ENTRYPOINT="arducopter_spresense_main"',
            "-CXD56_GNSS=y",
            "-CXD56_EMMC=y",
            "-CXD56_PWM=y",
            "-PWM=y",
        ),
    )

    app_makefile = (root / "Tools/spresense/copter_app/Makefile").read_text(
        encoding="utf-8"
    )
    require_tokens(
        failures,
        "copter-app",
        app_makefile,
        ("CXXSRCS += SensorBridge.cpp",),
    )

    build = (
        root / "Tools/spresense/build_m1_copter_firmware.py"
    ).read_text(encoding="utf-8")
    require_tokens(
        failures,
        "build",
        build,
        (
            "verify_m1_copter_map.py",
            "arducopter_spresense_main",
            "libarducopter.a",
            "libArduCopter_libs.a",
            "m1.physical_write_expected",
            "m1.sensor.hal_integration",
        ),
    )

    flash = (root / "Tools/flash_spresense.sh").read_text(encoding="utf-8")
    require_tokens(
        failures,
        "flash",
        flash,
        (
            "spresense-m1-copter-link",
            "m1.copter.full=true",
            "m1.copter.entry=arducopter_spresense_main",
            "m1.outputs=disabled",
            "m1.arming=always-denied",
            "m1.physical_write_expected=0",
            "m1.sensor_fallback=disabled",
            "m1.gnss.hal_integration=AP_GPS_Spresense",
            "m1.pwbimu.hal_integration=AP_InertialSensor_Spresense",
            "m1.sensor.hal_integration=GNSS+INS",
        ),
    )

    if failures:
        print(
            "spresense_m1_copter_contract=FAIL " + " ".join(failures),
            file=sys.stderr,
        )
        return 1
    print(
        "spresense_m1_copter_contract=PASS "
        "outputs=disabled sensors=GNSS+INS flight_ready=false"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
