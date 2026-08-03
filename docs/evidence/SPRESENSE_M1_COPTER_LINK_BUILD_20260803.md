# Spresense M1 full-Copter link and safe-runtime evidence — 2026-08-03

## Scope

This record covers the link of the real ArduCopter vehicle archive into a Sony
Spresense SDK/NuttX SPK and one narrow hardware runtime gate. The Sony main
board was connected with the CXD5610 GNSS Add-on and CXD5602PWBIMU Multi-IMU
Add-on. PWM, DShot, CAN and physical actuator writes remained disabled. This
is boot/GCS/arming-denial evidence, not sensor integration or flight evidence.

## Reproducible inputs

- upstream Copter baseline: `1511f27194f1dcc3728270883047bdf022b3fd53`
- Sony SDK commit: `7fd61b2c03f06a4ff0302b84c755e58c338788b2`
- compiler: Sony ARM GCC 10.3.1
- profile: `spresense-m1-copter-link`
- entry: `arducopter_spresense_main`
- artifact project commit: `040d5f9a7f9b55980a386c07cd8661e443fd38b2`
- clean `nuttx.spk` SHA-256:
  `038ea451aebf0e31807a4345911447831651d3e4dd27a8cdfe4989b131db671a`

Generated binaries and the detailed runtime JSON remain ignored and are not
distributed.

## Software result

The Sony final link produced a `nuttx.spk` containing the full Copter archive,
NuttX pthread scheduler/semaphore implementation, CXD5610 GNSS Add-on driver
and CXD5602PWBIMU driver. Symbol guards confirmed the unique Copter entry,
Copter arming function and reject-only Spresense RCOutput. The configuration
guard confirmed that built-in GNSS, PWM and eMMC are disabled.

The host contracts passed with output disabled and expected physical writes
zero. The linker-map guard reported:

| Region | code | rodata | data | bss | heap envelope |
|---|---:|---:|---:|---:|---:|
| Application SRAM | 855412 | 142128 | 5508 | 56856 | 504964 |
| GNSS RAM | 0 | 0 | 0 | 0 | 655360 |

Values are bytes. This profile intentionally uses all 640 KiB of GNSS RAM as
the GNSS heap; it does not relocate static Copter sections there.

The original hardware attempt reached static initialization and exposed an ABI
mismatch: the Waf vehicle archive and Sony NuttX application gave
`Spresense::Semaphore` different layouts. Opaque fixed-size pthread storage and
target-side size/alignment assertions now keep both compilation sides
identical. The complete clean cross-build passed after that correction.

## Flash and runtime result

The guarded MainCore install used `/dev/cu.usbserial-210`, DTR reset and the
exact clean artifact above. Sony and the repository guard reported all required
markers:

```text
Package validation is OK.
Saving package to "nuttx"
Restarting the board ...
flash=COMPLETE
spresense_dtr_install=PASS
```

The dedicated profile intentionally selects UART1 as the NuttX console so the
main-board CP2102N remains `/dev/ttyS0`, which is the AP_HAL console/GCS path.
With the startup wait bounded at 90 seconds, the deterministic runtime gate
captured at `2026-08-03T15:30:06.381580+00:00` reported:

```text
spresense_m1_copter_serial=PASS heartbeat=ARDUPILOTMEGA normal_arm=4 forced_arm=4 armed=false
```

Result `4` is `MAV_RESULT_FAILED`. Both ordinary and force-ARM commands reached
the fail-closed Copter path, and no heartbeat advertised an armed state. This
does not prove electrical output behavior; `physical_outputs_verified` remains
false even though the linked backend contains no physical write path.

## Remaining HOLD items

- The current HAL uses empty I2C/SPI managers and `HAL_INS_NONE`.
- `/dev/gps2` and `/dev/imu0` are not yet integrated into ArduPilot GPS/INS.
- Physical outputs, timing, sensor rate/accuracy, GNSS fix/accuracy, control
  stability and flight are unverified or out of scope.
- Development storage remains microSD; final eMMC and Multi-IMU pin
  coexistence remain a hardware-design HOLD. No automatic fallback is added.

This work was produced with AI assistance and requires human review before any
hardware use or upstream submission.
