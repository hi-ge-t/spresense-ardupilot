# Spresense M1 non-flight ArduCopter bench evidence — 2026-08-04

## Scope

This record covers parameter persistence, GNSS fix acquisition and one
30-minute output-disabled ArduCopter bench run. The Sony Spresense main board
was connected to the CXD5610 GNSS Add-on and CXD5602PWBIMU Multi-IMU Add-on.
The test used MAVLink over the main-board CP2102N UART. It did not enable or
exercise arming, PWM, DShot, CAN or any physical actuator write.

The same run also checked whether EKF3 output was observable. It was not, and
that result remains a HOLD rather than being relabelled as an EKF pass. IMU
axis orientation was not verified because the assembly was not placed in
labelled physical orientations during the capture. This is not flight
evidence.

## Tested firmware identity

- upstream Copter baseline: `1511f27194f1dcc3728270883047bdf022b3fd53`
- project commit: `1446192e7e6486307d86378614b2befba944cee8`
- profile: `spresense-m1-copter-link`
- clean `nuttx.spk` SHA-256:
  `981c1269ea1bacccc5dae22b4843109255340c47992347e8dec377c540bd0e07`
- outputs: disabled
- arming: always denied
- expected physical writes: zero

The bench checker consumed the clean artifact manifest and the restored boot
reported runtime commit `1446192e`. Detailed JSON records remain ignored
build artifacts under `build/spresense-m1-copter-link-artifacts/`.

## Parameter persistence result

The guarded test used only `GCS_PID_MASK`, which controls PID tuning telemetry
selection and does not enable arming, motors or a sensor backend. The checker
required the exact command-line acknowledgement `GCS_PID_MASK` before writing.

The 60-second smoke sequence passed:

1. read original value `0`;
2. write temporary value `1` and receive its `PARAM_VALUE` acknowledgement;
3. reset through DTR and read persistent value `1`;
4. write the original value `0`;
5. reset again and read persistent value `0`;
6. independently re-read `GCS_PID_MASK=0` after the 30-minute test.

This confirms one bounded parameter write/read/reboot/restore sequence through
the explicit microSD-backed AP_HAL storage path. It is not a general SD-card
endurance, filesystem or power-loss test. No eMMC path or automatic storage
fallback was used.

## GNSS and 30-minute runtime result

The continuous capture ended at `2026-08-04T04:08:06.680430+00:00`. The
observed duration was 1800.019 seconds without a checker restart or serial
reconnection.

| Observation | Result |
|---|---:|
| ArduPilot heartbeat count | 1808 |
| Maximum heartbeat gap | 1.305 seconds |
| Armed heartbeat observed | no |
| `GPS_RAW_INT` count | 1792 |
| Maximum GNSS fix type | 4 |
| Maximum visible satellites | 24 |
| `RAW_IMU` count | 8951 |
| First / last IMU time | 10281122 / 1802848483 microseconds |
| `ATTITUDE` count | 3583 |
| `SYS_STATUS` maximum load field | 146 |
| `MEMINFO` minimum free-memory fields | 4096 / 4096 bytes |

The GNSS therefore produced a fix during this bench run. No surveyed reference
position, truth trajectory or timing reference was used, so position accuracy,
latency and update-rate accuracy remain unverified. The `MEMINFO` and load
fields are recorded as MAVLink observations only; they do not replace NuttX
stack high-water, heap high-water, scheduler-jitter or linker-map evidence.

The six recorded RAW_IMU axes ranged from
`(1157, -1231, -9612, -12, -16, -9)` to
`(1455, -1017, -8182, 7, 12, 2)`. This proves that the stream and timestamps
continued to change. It does not prove the assembled board's X/Y/Z direction,
sign, scale or flight-rate suitability. The maximum vibration fields were
`(0.751, 0.636, 5.284)` and the maximum clip counters were `(1, 0, 0)`; these
values are not qualified as performance limits.

## EKF result and cause boundary

`EKF_STATUS_REPORT` and `AHRS2` were not observed during either the 60-second
smoke run or the 30-minute run. `ATTITUDE` continued to be emitted, but that is
not sufficient evidence that EKF3 initialized.

Read-only parameter inspection reported:

- `AHRS_EKF_TYPE=3`, `EK3_ENABLE=1`;
- `EK3_SRC1_POSXY=3`, `EK3_SRC1_VELXY=3`, `EK3_SRC1_VELZ=3` (GPS);
- `EK3_SRC1_POSZ=1` (barometer);
- `EK3_SRC1_YAW=1` (compass);
- `BARO1_DEVID=0` and `COMPASS_DEV_ID=0`.

The current Spresense board contract deliberately disables barometer backends
and exposes empty generic I2C/SPI managers. The dedicated bridge integrates
only the CXD5610 GNSS and six-axis PWBIMU. The configured EKF3 height and yaw
sources therefore have no detected device. This is consistent with the absent
EKF status, but the run does not claim that it is the only possible blocker.
No flight-critical EKF source parameter was changed merely to force a pass.

## Reproduction

Run the guarded persistence smoke test:

```sh
python3 Tools/spresense/m1_copter_bench_check.py \
  --port /dev/cu.usbserial-210 \
  --ack-param-write GCS_PID_MASK \
  --duration-seconds 60 \
  --output build/spresense-m1-copter-link-artifacts/bench-smoke-evidence.json
```

Then run the read-only 30-minute monitor:

```sh
python3 Tools/spresense/m1_copter_bench_check.py \
  --port /dev/cu.usbserial-210 \
  --duration-seconds 1800 \
  --output build/spresense-m1-copter-link-artifacts/bench-endurance-evidence.json
```

## Remaining HOLD items

- labelled physical X/Y/Z orientation and sign check;
- EKF3 initialization with an explicitly reviewed sensor/source design;
- GNSS position accuracy, latency and update-rate measurement;
- NuttX heap and stack high-water plus scheduler-jitter measurement;
- power-cycle and interrupted-write storage testing;
- Multi-IMU SPI5 and final eMMC pin coexistence;
- electrical output behavior, actuator integration, control stability and
  flight.

This work was produced with AI assistance and requires human review before any
upstream submission or hardware use.
