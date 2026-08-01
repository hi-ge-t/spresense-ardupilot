# Spresense M1 GCS bench evidence — 2026-08-01

## Scope

This record closes only the output-disabled M1 diagnostic boot and GCS
communication gate. It does not prove a Copter port, sensor fusion, actuator
output or flight readiness.

Bench configuration:

- Sony Spresense main board with the CXD5610 GNSS Add-on installed;
- Multi-IMU board removed;
- main USB serial `/dev/cu.usbserial-210` at 115200 baud;
- QGroundControl 5.0.8 (`CFBundleShortVersionString=5.0`, build `5.0.8`).

## Reproducible inputs and artifact

- project commit: `ec88bc958ba5e809a23ee46f06c6777b3694c944`
- upstream Copter baseline: `1511f27194f1dcc3728270883047bdf022b3fd53`
- Sony SDK commit: `7fd61b2c03f06a4ff0302b84c755e58c338788b2`
- MAVLink commit: `288b907c384a892c8519bfe271682424b1e1a3a0`
- compiler: Sony ARM GCC 10.3.1
- `nuttx.spk` SHA-256:
  `1cf4c0ab251790fcbac60effe13f2b9c45d180b8bf1d10ea7f27d792c2a9a25d`
- artifact manifest: `project_tree=clean`, built-in GNSS disabled, GNSS
  Add-on/GNSS RAM required, PWM disabled, arming always denied and expected
  physical writes zero.

The clean cross-build printed:

```text
spresense_m1_gcs_map=PASS gnss_heap_bytes=655128
spresense_m1_gcs_build=PASS project_commit=ec88bc958ba5e809a23ee46f06c6777b3694c944 tree=clean gcc=10.3.1
```

The linker-map guard recorded these non-overlapping ranges:

| Region | code | rodata | data | bss | heap envelope |
|---|---:|---:|---:|---:|---:|
| Application SRAM | 110016 | 10536 | 1276 | 17428 | 1424304 |
| GNSS RAM | 40 | 64 | 64 | 64 | 655128 |

Values are bytes. This is a linker-placement result, not a runtime timing or
memory-pressure measurement.

## Flash and runtime result

The repository flash guard passed for the commit and artifact hash above. The
DTR MainCore install reported all required Sony updater markers:

```text
Package validation is OK.
Saving package to "nuttx"
Restarting the board ...
flash=COMPLETE
spresense_dtr_install=PASS
```

The deterministic MAVLink gate captured at
`2026-08-01T01:55:08.416963+00:00` reported:

```text
spresense_m1_gcs_serial=PASS heartbeat=ARDUPILOTMEGA version=M1GCS001 params=4 arm=DENIED
```

Observed values were system/component `1/1`, vehicle type `QUADROTOR`, fixed
parameters `M1_OUT_EN=0`, `M1_GNSS_REQ=1`, `M1_GNSS_RAM=1`, `M1_STAGE=1`, and
a later heartbeat still not armed after the rejected ARM request.

QGroundControl was then connected manually through the existing
`Spresense Main USB` link. It opened `/dev/cu.usbserial-210` and changed from
`Disconnected` to an `ARDUPILOT` vehicle view showing `Not Ready` and
`Stabilize`. `Not Ready` is expected for this diagnostic image and must not be
interpreted as flight firmware readiness.

## Safety result and remaining HOLD items

- Source, target configuration and host tests establish that no actuator,
  PWM, DShot or CAN output backend is present and the protocol reports zero
  physical writes. No energized physical-output measurement was performed.
- Every tested ARM request was denied and the following heartbeat remained
  unarmed.
- GNSS Add-on driver selection and GNSS RAM placement are confirmed, but GNSS
  samples, fix quality, accuracy and latency are still **HOLD**.
- Full Copter link/startup and `AP_HAL_Spresense` integration are **HOLD**.
- Multi-IMU coexistence is **HOLD** because the board was removed for this run.
- Loop timing, scheduling jitter, sensor rates, flight stability and all flight
  tests are **HOLD** and were not claimed.
- Development storage remains microSD, the final candidate remains eMMC, and
  no automatic fallback was added or tested.

