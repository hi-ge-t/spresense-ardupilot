# Spresense M1 output-disabled GCS bring-up

## Scope and current boundary

This fork starts an experimental Sony Spresense lane from upstream
`Copter-4.7.0`, commit `1511f27194f1dcc3728270883047bdf022b3fd53`.
It is not flight firmware.

M1 now contains two separate software slices:

1. `AP_HAL_Spresense` host-verifiable console, UART, monotonic-time, storage,
   scheduler and fail-closed RC-output primitives.
2. Sony SDK/NuttX diagnostic images that prove the future main-USB MAVLink
   boundary before Copter is linked. The default
   `spresense-m1-pwbimu-gnss-gcs` profile requires both the CXD5610 GNSS
   Add-on and CXD5602PWBIMU Multi-IMU Add-on. The earlier
   `spresense-m1-gcs` GNSS-only profile remains available for regression.

The diagnostic uses the ArduPilot-pinned `ardupilotmega` MAVLink v2 definition
and identifies as `MAV_AUTOPILOT_ARDUPILOTMEGA`/quadrotor. It sends a 1 Hz
HEARTBEAT, answers `AUTOPILOT_VERSION`, exposes four fixed read-only diagnostic
parameters and rejects `MAV_CMD_COMPONENT_ARM_DISARM`. The combined profile
adds read-only `M1_GNSS_OK`, `M1_GNSS_ERR`, `M1_IMU_REQ=1` and `M1_IMU_OK`
parameters. `M1_GNSS_ERR` preserves the bounded probe's zero or negative
result for fail-closed diagnosis. It
opens `/dev/gps2`, checks the CXD5610 firmware response, starts positioning,
waits at most 15 seconds for one notification sample and stops positioning.
It then opens
`/dev/imu0` read-only, configures a bounded 60 Hz diagnostic capture, reads one
sample and stops the sensor. A failed probe leaves the corresponding `*_OK`
parameter at zero and reports `MAV_STATE_CRITICAL`; neither probe enables
arming or a physical output. A GNSS sample is not evidence of a position fix,
accuracy or timing. This identity exists to exercise a real GCS connection;
the image does not contain Copter control, navigation, sensor fusion or flight
modes.

## Fixed safety and platform contract

- Every ARM request is answered with `MAV_RESULT_DENIED`.
- No actuator, PWM, DShot or CAN output backend is linked. Sony/NuttX PWM is
  disabled in the target configuration.
- The built-in GNSS is disabled. The CXD5610 GNSS Add-on at `/dev/gps2` and
  GNSS RAM remain mandatory profile requirements. The combined profile has no
  synthetic GNSS fallback and must complete one bounded Add-on sample probe.
- In the combined profile, Sony's CXD5602PWBIMU driver, SPI5 DMAC and
  `/dev/imu0` are mandatory. There is no synthetic sensor or runtime fallback.
- The main-board CP2102N UART is `/dev/ttyS0` at 115200 baud. NSH, CDC-ACM and
  USB mass storage commands are disabled so text cannot corrupt MAVLink.
- Development storage remains microSD and the final candidate remains eMMC.
  No automatic storage fallback is introduced by this slice.
- The standard Multi-IMU driver uses SPI5 on pins shared with eMMC, and this
  diagnostic explicitly keeps eMMC disabled. The final-carrier eMMC and IMU
  coexistence design is therefore HOLD; this profile is not that final design.
- Generated firmware artifacts stay under ignored `build/`; binaries are not
  committed or distributed from this stage.

The machine-readable contract is `spresense-m1-manifest.json`.

## Reproducible host and Sony SDK verification

Initialize the pinned submodules:

```sh
git submodule update --init --recursive modules/Spresense modules/mavlink
```

Run the AP_HAL and MAVLink host contracts:

```sh
python3 Tools/spresense/run_host_tests.py
```

With Sony GCC 10.3.1 installed at `~/spresenseenv/usr/bin`, build the default
combined image from a clean tree:

```sh
python3 Tools/spresense/build_m1_gcs_firmware.py
```

The build creates `build/spresense-m1-pwbimu-gnss-gcs-artifacts/` containing
`nuttx.spk`, ELF, linker map, NuttX configuration, a machine-readable memory
report and an `ARTIFACTS.manifest` with SHA-256 values. The build fails unless
all of these conditions hold:

- Sony SDK and MAVLink commits match the pinned revisions;
- built-in GNSS and PWM are disabled;
- GNSS Add-on, GNSS RAM and GNSS heap are enabled;
- the Multi-IMU driver, `/dev/imu0`, SPI5 and SPI5 DMAC are enabled while eMMC
  is disabled;
- the GNSS Add-on probe, `/dev/gps2` and its driver symbols are linked;
- the main-USB serial/autostart configuration matches the fixed profile;
- `spresense_main` is one unique strong symbol;
- Application SRAM and GNSS RAM code/rodata/data/bss/heap boundaries are
  non-empty and remain inside their linker regions.

`--allow-dirty` exists only for development. Its manifest says
`project_tree=dirty`, and the flash guard refuses it.

The historical GNSS-only regression image remains selectable explicitly:

```sh
python3 Tools/spresense/build_m1_gcs_firmware.py \
  --profile spresense-m1-gcs
```

## Hardware and GCS procedure

Close QGroundControl and every serial terminal before preflight. On macOS use
only the single `/dev/cu.*` path belonging to the Spresense main-board CP2102N.

```sh
SPFC_ARTIFACT_DIR="$PWD/build/spresense-m1-pwbimu-gnss-gcs-artifacts" \
SPFC_PROFILE=spresense-m1-pwbimu-gnss-gcs \
  Tools/flash_spresense.sh /dev/cu.usbserial-210 --preflight
```

After checking the printed full commit, install with the same variables plus
`SPFC_FLASH_ACK=FULL_COMMIT` and `--execute`. The normal MainCore SPK path uses
DTR reset and `--no-set-bootable`. Accept the run only if Sony prints package
validation, save and restart completion and the guard prints `flash=COMPLETE`.

Then run the deterministic bidirectional GCS gate:

```sh
python3 Tools/spresense/m1_gcs_serial_check.py \
  --port /dev/cu.usbserial-210 \
  --require-gnss \
  --require-pwbimu \
  --verify-arm-denied \
  --output build/spresense-m1-pwbimu-gnss-gcs-artifacts/runtime-evidence.json
```

This gate requires an ArduPilot heartbeat, `M1PGN001` AUTOPILOT_VERSION, all
eight diagnostic parameters including `M1_GNSS_OK=1`, `M1_GNSS_ERR=0` and
`M1_IMU_OK=1`, a
denied ARM acknowledgement and a later non-armed heartbeat. `M1_GNSS_OK=1`
means that a bounded Add-on notification was read; it does not mean that a fix
was obtained. QGroundControl is opened separately after this gate to confirm
that an installed GCS discovers the same vehicle.

The first hardware run was completed on 2026-08-01. The committed evidence
summary is `docs/evidence/SPRESENSE_M1_GCS_20260801.md`; that run used the
GNSS-only profile with the Multi-IMU removed. The combined profile's
software-only build record is
`docs/evidence/SPRESENSE_M1_PWBIMU_GNSS_BUILD_20260803.md`. A single combined
GNSS + Multi-IMU bench run was completed on 2026-08-03 and is recorded in
`docs/evidence/SPRESENSE_M1_COMBINED_RUNTIME_20260803.md`. Generated firmware
and the detailed runtime JSON remain ignored build artifacts.

## Evidence and HOLD items

| Item | Status | Evidence or remaining check |
|---|---|---|
| AP_HAL host contract | Confirmed | `run_host_tests.py` PASS on 2026-08-01 |
| MAVLink protocol host contract | Confirmed | heartbeat/version/parameters/ARM denial, physical writes 0 |
| Sony SDK/NuttX object compile | Confirmed | clean build with Sony GCC 10.3.1 |
| Sony SDK diagnostic firmware link/SPK | Confirmed | combined and legacy profile clean builds passed at `ffa5071153...` |
| Combined GNSS + Multi-IMU SDK link/SPK | Confirmed | driver symbols and guarded profile linked on 2026-08-03 |
| Application/GNSS RAM linker boundaries | Confirmed | `memory-layout.json`; runtime timing remains unqualified |
| Spresense boot and bidirectional serial GCS | Confirmed, one bench run | `M1GCS001`, four parameters and ARM `DENIED` on 2026-08-01 |
| QGroundControl discovery | Confirmed, one bench run | QGroundControl 5.0.8 displayed ArduPilot / Not Ready |
| Full Sony NuttX Copter link | HOLD | future M1 slice; this image is not Copter |
| GNSS Add-on bounded sample | Confirmed, one bench run | `M1PGN001`, `M1_GNSS_OK=1`, `M1_GNSS_ERR=0`; fix, accuracy and latency remain HOLD |
| Multi-IMU startup sample | Confirmed, one bench run | `M1PGN001`, `M1_IMU_OK=1` on 2026-08-03 |
| Combined Add-on coexistence | Confirmed, one boot | `M1_GNSS_OK=1` and `M1_IMU_OK=1` from the same boot on 2026-08-03 |
| Multi-IMU + final eMMC pin coexistence | HOLD | standard Multi-IMU profile uses SPI5 pins shared with eMMC; carrier design must resolve this without automatic fallback |
| Timing, GNSS accuracy, stability and flight | HOLD/out of scope | no claim is made by this implementation |

This work was produced with AI assistance and requires human review before any
upstream submission or hardware use.
