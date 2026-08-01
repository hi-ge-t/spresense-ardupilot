# Spresense M1 output-disabled GCS bring-up

## Scope and current boundary

This fork starts an experimental Sony Spresense lane from upstream
`Copter-4.7.0`, commit `1511f27194f1dcc3728270883047bdf022b3fd53`.
It is not flight firmware.

M1 now contains two separate software slices:

1. `AP_HAL_Spresense` host-verifiable console, UART, monotonic-time, storage,
   scheduler and fail-closed RC-output primitives.
2. A Sony SDK/NuttX `spresense-m1-gcs` diagnostic image that proves the future
   main-USB MAVLink boundary before Copter is linked.

The diagnostic uses the ArduPilot-pinned `ardupilotmega` MAVLink v2 definition
and identifies as `MAV_AUTOPILOT_ARDUPILOTMEGA`/quadrotor. It sends a 1 Hz
HEARTBEAT, answers `AUTOPILOT_VERSION`, exposes four fixed read-only diagnostic
parameters and rejects `MAV_CMD_COMPONENT_ARM_DISARM`. This identity exists to
exercise a real GCS connection; the image does not contain Copter control,
navigation, sensor fusion or flight modes.

## Fixed safety and platform contract

- Every ARM request is answered with `MAV_RESULT_DENIED`.
- No actuator, PWM, DShot or CAN output backend is linked. Sony/NuttX PWM is
  disabled in the target configuration.
- The built-in GNSS is disabled. The CXD5610 GNSS Add-on at `/dev/gps2` and
  GNSS RAM remain mandatory profile requirements.
- The main-board CP2102N UART is `/dev/ttyS0` at 115200 baud. NSH, CDC-ACM and
  USB mass storage commands are disabled so text cannot corrupt MAVLink.
- Development storage remains microSD and the final candidate remains eMMC.
  No automatic storage fallback is introduced by this slice.
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

With Sony GCC 10.3.1 installed at `~/spresenseenv/usr/bin`, build a clean
flashable image:

```sh
python3 Tools/spresense/build_m1_gcs_firmware.py
```

The build creates `build/spresense-m1-gcs-artifacts/` containing `nuttx.spk`,
ELF, linker map, NuttX configuration, a machine-readable memory report and an
`ARTIFACTS.manifest` with SHA-256 values. The build fails unless all of these
conditions hold:

- Sony SDK and MAVLink commits match the pinned revisions;
- built-in GNSS and PWM are disabled;
- GNSS Add-on, GNSS RAM and GNSS heap are enabled;
- the main-USB serial/autostart configuration matches the fixed profile;
- `spresense_main` is one unique strong symbol;
- Application SRAM and GNSS RAM code/rodata/data/bss/heap boundaries are
  non-empty and remain inside their linker regions.

`--allow-dirty` exists only for development. Its manifest says
`project_tree=dirty`, and the flash guard refuses it.

## Hardware and GCS procedure

Close QGroundControl and every serial terminal before preflight. On macOS use
only the single `/dev/cu.*` path belonging to the Spresense main-board CP2102N.

```sh
SPFC_ARTIFACT_DIR="$PWD/build/spresense-m1-gcs-artifacts" \
SPFC_PROFILE=spresense-m1-gcs \
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
  --verify-arm-denied \
  --output build/spresense-m1-gcs-artifacts/runtime-evidence.json
```

This gate requires an ArduPilot heartbeat, `M1GCS001` AUTOPILOT_VERSION,
all four diagnostic parameters, a denied ARM acknowledgement and a later
non-armed heartbeat. QGroundControl is opened separately after this gate to
confirm that an installed GCS discovers the same vehicle.

## Evidence and HOLD items

| Item | Status before hardware run | Evidence or remaining check |
|---|---|---|
| AP_HAL host contract | Implemented | `run_host_tests.py` |
| MAVLink protocol host contract | Implemented | heartbeat/version/parameters/ARM denial, physical writes 0 |
| Sony SDK/NuttX object compile | Implemented | Sony GCC 10.3.1 |
| Sony SDK diagnostic firmware link/SPK | Implemented | `build_m1_gcs_firmware.py` |
| Application/GNSS RAM linker boundaries | Implemented | `memory-layout.json`; runtime timing remains unqualified |
| Spresense boot and bidirectional serial GCS | HOLD | clean flash and serial gate required |
| QGroundControl discovery | HOLD | UI confirmation after deterministic gate |
| Full Sony NuttX Copter link | HOLD | future M1 slice; this image is not Copter |
| GNSS Add-on read in this image | HOLD | profile is configured, but this image does not yet publish GNSS data |
| Multi-IMU coexistence | HOLD | Multi-IMU is currently removed |
| Timing, GNSS accuracy, stability and flight | HOLD/out of scope | no claim is made by this implementation |

This work was produced with AI assistance and requires human review before any
upstream submission or hardware use.
