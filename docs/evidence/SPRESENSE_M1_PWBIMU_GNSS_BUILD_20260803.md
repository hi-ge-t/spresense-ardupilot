# Spresense M1 GNSS + Multi-IMU software evidence — 2026-08-03

## Scope

This record covers source, host and Sony SDK cross-build evidence for the
output-disabled `spresense-m1-pwbimu-gnss-gcs` diagnostic profile. It does not
record a flash or hardware run. The bench remained assigned to PX4 work, so
Multi-IMU sampling, GNSS operation and combined physical coexistence remain
HOLD.

This image is not Copter flight firmware. It contains no actuator, PWM, DShot
or CAN output backend, rejects every ARM request and reports zero expected
physical writes.

## Reproducible inputs

- project commit: `445f79971b23806b36edd7dbec9ba6410d2157b8`
- upstream Copter baseline: `1511f27194f1dcc3728270883047bdf022b3fd53`
- Sony SDK commit: `7fd61b2c03f06a4ff0302b84c755e58c338788b2`
- MAVLink commit: `288b907c384a892c8519bfe271682424b1e1a3a0`
- compiler: Sony ARM GCC 10.3.1
- profile: `spresense-m1-pwbimu-gnss-gcs`
- clean `nuttx.spk` SHA-256:
  `c1474833f47bbf94c9b366db9f3dc36bbee670eebaeca1790edaa9fab722cb4c`

## Host and cross-build result

The host contract passed for the legacy four-parameter protocol and combined
six-parameter protocol. Both retained ARM denial, immutable output enable and
zero physical writes:

```text
spresense_m1_gcs_protocol=PASS heartbeat=ardupilotmega params=4 arm=DENIED physical_writes=0
spresense_m1_gcs_protocol=PASS heartbeat=ardupilotmega params=6 arm=DENIED physical_writes=0
spresense_m1_contract=PASS outputs=disabled automatic_fallback=disabled
```

The legacy GNSS-only Sony SDK profile also completed a non-flashable
development regression build. The combined profile then completed from the
clean project commit above:

```text
spresense_m1_gcs_map=PASS gnss_heap_bytes=655128
spresense_m1_gcs_build=PASS profile=spresense-m1-pwbimu-gnss-gcs project_commit=445f79971b23806b36edd7dbec9ba6410d2157b8 tree=clean gcc=10.3.1
```

The generated configuration mechanically confirmed:

- built-in GNSS, PWM and eMMC disabled;
- CXD5610 GNSS Add-on and `/dev/gps2` driver enabled;
- CXD5602PWBIMU, `/dev/imu0`, SPI5 and SPI5 DMAC enabled;
- GNSS RAM and GNSS heap enabled;
- no sensor or storage automatic fallback.

The final ELF contained unique `spresense_main`,
`m1_pwbimu_probe_once`, `board_cxd5602pwbimu_initialize`,
`cxd5602pwbimu_register` and `cxd5610_gnss_register` symbols. The build script
now fails if the combined profile lacks those sensor symbols, or if the legacy
profile unexpectedly includes the Multi-IMU symbols.

## Linker map result

The map guard recorded non-overlapping Application SRAM and GNSS RAM ranges:

| Region | code | rodata | data | bss | heap envelope |
|---|---:|---:|---:|---:|---:|
| Application SRAM | 117952 | 10828 | 1556 | 16988 | 1416112 |
| GNSS RAM | 40 | 64 | 64 | 64 | 655128 |

Values are bytes. This is linker placement evidence, not runtime memory
pressure, latency or scheduling evidence.

## Remaining HOLD items

- No firmware from this profile was flashed on 2026-08-03.
- `/dev/imu0` open/configure/read/stop and `M1_IMU_OK=1` are hardware HOLD.
- `/dev/gps2` samples, GNSS fix, accuracy and latency are hardware HOLD.
- Simultaneous physical operation of both Add-on boards is hardware HOLD.
- The standard Multi-IMU driver uses SPI5 on pins shared with eMMC. The current
  profile disables eMMC; the final-carrier coexistence design is HOLD.
- Full Copter link/startup, sensor fusion, loop timing, scheduling jitter,
  sensor rates, flight stability and flight tests are HOLD or out of scope.
- Development storage remains microSD and the final candidate remains eMMC.
  No automatic fallback was added.

This work was produced with AI assistance and requires human review before any
hardware use or upstream submission.
