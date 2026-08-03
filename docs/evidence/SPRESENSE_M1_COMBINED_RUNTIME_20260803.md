# Spresense M1 combined GNSS + Multi-IMU bench evidence — 2026-08-03

## Scope

This record closes one output-disabled hardware gate for the
`spresense-m1-pwbimu-gnss-gcs` diagnostic profile. The Sony Spresense main
board was connected with both the CXD5610 GNSS Add-on and CXD5602PWBIMU
Multi-IMU Add-on. The main-board CP2102N was detected at
`/dev/cu.usbserial-2140` and used at 115200 baud.

This image is not Copter flight firmware. It contains no actuator, PWM, DShot
or CAN output backend, rejects every ARM request and reports zero expected
physical writes. No actuator or energized physical-output measurement was
performed.

## Reproducible inputs and artifact

- project commit: `ffa50711535823eba6a5f09dcb32a01bb38ca578`
- upstream Copter baseline: `1511f27194f1dcc3728270883047bdf022b3fd53`
- Sony SDK commit: `7fd61b2c03f06a4ff0302b84c755e58c338788b2`
- MAVLink commit: `288b907c384a892c8519bfe271682424b1e1a3a0`
- compiler: Sony ARM GCC 10.3.1
- profile: `spresense-m1-pwbimu-gnss-gcs`
- clean `nuttx.spk` SHA-256:
  `2779284fc53ea8adb40a6138beb370e8d3dcc4e167f2c5c5613a37ec883851c3`

The combined image completed a clean Sony SDK cross-build. The historical
`spresense-m1-gcs` GNSS-only profile also completed a clean regression build
at the same project commit:

```text
spresense_m1_gcs_map=PASS gnss_heap_bytes=655128
spresense_m1_gcs_build=PASS profile=spresense-m1-pwbimu-gnss-gcs project_commit=ffa50711535823eba6a5f09dcb32a01bb38ca578 tree=clean gcc=10.3.1
spresense_m1_gcs_build=PASS profile=spresense-m1-gcs project_commit=ffa50711535823eba6a5f09dcb32a01bb38ca578 tree=clean gcc=10.3.1
```

The combined linker-map guard recorded these non-overlapping ranges:

| Region | code | rodata | data | bss | heap envelope |
|---|---:|---:|---:|---:|---:|
| Application SRAM | 118400 | 10872 | 1556 | 18540 | 1414064 |
| GNSS RAM | 40 | 64 | 64 | 64 | 655128 |

Values are bytes. This proves linker placement only; it is not runtime memory
pressure, latency or scheduling evidence.

## Flash and runtime result

The guarded MainCore install used DTR reset and reported all required Sony
updater markers:

```text
Package validation is OK.
Saving package to "nuttx"
Restarting the board ...
flash=COMPLETE
spresense_dtr_install=PASS
```

The deterministic MAVLink gate captured at
`2026-08-03T11:03:17.614356+00:00` reported:

```text
spresense_m1_gcs_serial=PASS heartbeat=ARDUPILOTMEGA version=M1PGN001 params=8 gnss=PASS pwbimu=PASS arm=DENIED
```

The same boot returned `M1_GNSS_OK=1`, `M1_GNSS_ERR=0`, `M1_IMU_OK=1` and
`M1_OUT_EN=0`. The ARM command was denied and a later heartbeat remained
unarmed. `M1_GNSS_OK=1` means that one complete positioning notification was
read within the bounded startup probe. It does not prove a GNSS fix, position
accuracy or notification latency. The runtime evidence therefore keeps
`gnss_fix_verified=false`, `flight_verified=false` and
`physical_outputs_verified=false`.

## Diagnostic iteration

The first combined run at `1fc803ef7764e1bcf00febbc9134d33516d841f5`
confirmed the Multi-IMU sample as `M1PIM001`, but did not yet read GNSS. The
first GNSS probe used a three-second notification bound and failed closed. An
error-reporting iteration at `d79bf03b5490bebcf969d0c0b86faf42b8b757f2`
returned `M1_GNSS_ERR=-110` (`ETIMEDOUT`), after device open, wakeup, version
and positioning start had succeeded. The final 15-second bounded probe passed
at the artifact commit above. This sequence does not establish actual GNSS
startup latency; it only shows that three seconds was insufficient in that run
and the notification arrived within the final bound.

## Remaining HOLD items

- GNSS fix acquisition, accuracy and actual startup/notification latency are
  **HOLD**.
- Sustained sensor rates, loop timing, scheduling jitter and long-duration
  combined stability are **HOLD**.
- Full Copter link/startup, `AP_HAL_Spresense` sensor integration, sensor
  fusion, navigation, flight modes, flight stability and flight tests are
  **HOLD** or out of scope.
- QGroundControl discovery was not repeated with this combined image. The
  deterministic bidirectional MAVLink gate passed, but installed-GCS behavior
  for this exact image remains **HOLD**.
- The standard Multi-IMU driver uses SPI5 on pins shared with eMMC. This
  profile keeps eMMC disabled; final-carrier eMMC and Multi-IMU coexistence is
  a hardware-design **HOLD**.
- Development storage remains microSD and the final candidate remains eMMC.
  No automatic storage fallback was added or tested.
- Generated firmware and runtime JSON remain ignored build artifacts; no
  binary is committed or distributed by this record.

This work was produced with AI assistance and requires human review before any
upstream submission or flight-related hardware use.
