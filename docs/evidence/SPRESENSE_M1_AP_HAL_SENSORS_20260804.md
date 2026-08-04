# Spresense M1 AP_HAL GNSS/INS safe-runtime evidence — 2026-08-04

## Scope

This record covers one combined-board bench run of the full ArduCopter archive
through `AP_HAL_Spresense`. The Sony Spresense main board was connected with
the CXD5610 GNSS Add-on and CXD5602PWBIMU Multi-IMU Add-on. It confirms only
that both device streams reached their ArduPilot frontends while the Copter
arming path remained fail-closed. PWM, DShot, CAN and physical actuator writes
remained absent from the linked backend. This is not flight evidence.

## Reproducible inputs and artifact identity

- upstream Copter baseline: `1511f27194f1dcc3728270883047bdf022b3fd53`
- project commit: `1446192e7e6486307d86378614b2befba944cee8`
- Sony SDK commit: `7fd61b2c03f06a4ff0302b84c755e58c338788b2`
- MAVLink commit: `288b907c384a892c8519bfe271682424b1e1a3a0`
- compiler: Sony ARM GCC 10.3.1
- profile: `spresense-m1-copter-link`
- clean `nuttx.spk` SHA-256:
  `981c1269ea1bacccc5dae22b4843109255340c47992347e8dec377c540bd0e07`

The runtime checker read `1446192e` from the booted ArduCopter identity and
required it to be a prefix of the manifest project commit. Its independently
calculated SPK hash matched the manifest value above. Generated binaries and
the detailed `runtime-evidence.json` remain ignored build artifacts.

## Build and memory result

The host contracts and clean Sony SDK cross-build passed with the real Copter
archive, dedicated `AP_GPS_Spresense` and `AP_InertialSensor_Spresense`
backends, built-in GNSS disabled, both Add-ons required, GNSS RAM enabled,
eMMC disabled and output/arming guards intact. The linker-map verifier
reported:

| Region | code | rodata | data | bss | heap envelope |
|---|---:|---:|---:|---:|---:|
| Application SRAM | 858032 | 143872 | 5840 | 57520 | 504752 |
| GNSS RAM | 0 | 0 | 0 | 0 | 655360 |

Values are bytes. All Application SRAM sections and heap boundaries were
inside the 1536 KiB region. Static sections in GNSS RAM were empty and the
complete 640 KiB region was available as its required heap.

## Hardware runtime result

The guarded DTR MainCore install targeted `/dev/cu.usbserial-210`. The runtime
capture completed at `2026-08-04T02:20:27.801810+00:00` and passed these
machine checks:

- ArduPilotMega quadrotor heartbeat and the HAL-loop marker were received.
- `SPRESENSE_M1_GNSS=SAMPLE`, `ATTACH` and `CONSUMED` were all received, so
  one CXD5610 stream sample passed through `AP_GPS_Spresense` to AP_GPS.
- `GPS_RAW_INT` reported fix type 1 and zero visible satellites. This is
  explicitly a no-fix result and does not verify position or accuracy.
- `SPRESENSE_M1_PWBIMU=SAMPLE` was received and two different RAW_IMU samples
  were captured. Their axes were `(1308, -1108, -9589, 0, 0, 0)` and
  `(1310, -1111, -9619, 0, 0, 0)` at 11597034 and 11796817 microseconds.
  This confirms a changing nonzero acceleration stream only. The zero gyro
  values in this stationary capture are not a gyro-performance claim.
- A PWBIMU stream timeout triggered the bounded restart path; recovery was
  observed as attempted and successful, without reaching the three-attempt
  exhaustion guard.
- Normal and forced ARM requests both returned result 4
  (`MAV_RESULT_FAILED`), and no heartbeat advertised armed state.

The runtime record therefore sets GNSS and PWBIMU runtime verification true,
but keeps GNSS accuracy, PWBIMU orientation, timing, physical outputs and
flight verification false.

## Safety boundary and remaining HOLD items

- The PWBIMU restart is bounded to three attempts and fails the required
  stream closed after exhaustion. It does not choose a synthetic or alternate
  sensor.
- Source, host and final-link guards require output disabled, arming always
  denied and expected physical writes zero. Electrical output behavior was
  not exercised and remains unverified.
- GNSS fix/accuracy/latency, IMU axis orientation, update-rate timing and
  scheduling jitter remain HOLD.
- Control stability, navigation performance, actuator behavior and flight
  remain HOLD and out of scope for this image.
- Development storage remains microSD. The final candidate remains eMMC, but
  standard Multi-IMU SPI5 pin sharing with eMMC is a hardware-design HOLD.
  No automatic storage fallback was added.

This work was produced with AI assistance and requires human review before any
upstream submission or hardware use.
