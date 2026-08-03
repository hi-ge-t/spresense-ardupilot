# Spresense M1 full-Copter link software evidence — 2026-08-03

## Scope

This record covers the software-only link of the real ArduCopter vehicle
archive into a Sony Spresense SDK/NuttX SPK. It is not hardware runtime or
flight evidence. PWM, DShot, CAN and physical actuator writes remain disabled,
and the Copter arming entry rejects normal and forced arming.

## Reproducible inputs

- upstream Copter baseline: `1511f27194f1dcc3728270883047bdf022b3fd53`
- Sony SDK commit: `7fd61b2c03f06a4ff0302b84c755e58c338788b2`
- compiler: Sony ARM GCC 10.3.1
- profile: `spresense-m1-copter-link`
- entry: `arducopter_spresense_main`

The clean project commit and SPK SHA-256 are recorded after the final clean
rebuild. Generated binaries remain ignored and are not distributed.

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
| Application SRAM | 854044 | 142128 | 5508 | 56344 | 507012 |
| GNSS RAM | 0 | 0 | 0 | 0 | 655360 |

Values are bytes. This profile intentionally uses all 640 KiB of GNSS RAM as
the GNSS heap; it does not relocate static Copter sections there.

## Remaining HOLD items

- Clean artifact build, flash preflight and Spresense boot are pending the
  final committed source tree.
- The current HAL uses empty I2C/SPI managers and `HAL_INS_NONE`.
- `/dev/gps2` and `/dev/imu0` are not yet integrated into ArduPilot GPS/INS.
- GCS heartbeat and both normal and forced ARM rejection require a separate
  hardware run.
- Physical outputs, timing, sensor rate/accuracy, GNSS fix/accuracy, control
  stability and flight are unverified or out of scope.
- Development storage remains microSD; final eMMC and Multi-IMU pin
  coexistence remain a hardware-design HOLD. No automatic fallback is added.

This work was produced with AI assistance and requires human review before any
hardware use or upstream submission.
