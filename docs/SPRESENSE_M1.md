# Spresense M1 output-disabled bring-up

## Scope

This fork starts an experimental `AP_HAL_Spresense` lane from upstream
`Copter-4.7.0`, commit `1511f27194f1dcc3728270883047bdf022b3fd53`.
It is a bring-up implementation, not flight firmware.

The first slice registers the compile-time `HAL_BOARD_SPRESENSE` identity and
implements host-verifiable primitives and AP_HAL interface
adapters for a NuttX console byte stream, monotonic time, fixed-file storage and
a fail-closed RC output guard. It does not yet add a Sony SDK waf toolchain,
link Copter for Sony NuttX, or boot on Spresense. Timer/IO process registration
deliberately marks the scheduler unhealthy until the Sony NuttX task
implementation is added.

## Fixed safety contract

- The M1 guard rejects every arming request; the Copter arming path is not yet linked.
- Actuator, PWM, DShot and CAN output drivers are absent.
- Output write requests are counted and rejected without a physical write.
- Built-in GNSS is disabled; the CXD5610 GNSS Add-on at `/dev/gps2` is required.
- GNSS RAM is required by the future target profile.
- Development storage is microSD. eMMC remains the final candidate.
- There is no automatic storage or GNSS fallback.
- Firmware binaries are not distributed from this stage.

The machine-readable contract is `spresense-m1-manifest.json`. Run:

```sh
python3 Tools/spresense/run_host_tests.py
```

Initialize the pinned Sony SDK and generate the GNSS Add-on/GNSS RAM context:

```sh
git submodule update --init --recursive modules/Spresense
(cd modules/Spresense/sdk && python3 tools/config.py default feature/gnss_addon)
make -C modules/Spresense/nuttx -j1 context
```

Then, with Sony GCC 10.3.1 on `PATH`, compile the M1 sources against the pinned
Spresense/NuttX headers:

```sh
python3 Tools/spresense/cross_compile_m1.py
```

## Evidence status

| Item | Status | Evidence or next check |
|---|---|---|
| Host C++ compile and contract test | Implemented | `run_host_tests.py` |
| Output and arming rejection | Implemented on host | physical write count must remain zero |
| Copter arming integration | HOLD | no linked Copter arming path yet |
| Sony SDK/NuttX object compile | Implemented | 5 objects, Sony GCC 10.3.1, SDK `7fd61b2c` |
| Sony SDK/NuttX Copter link | HOLD | add waf/SDK link integration |
| Linker map for Application/GNSS RAM | HOLD | reuse the measured M0 memory contract in the target linker |
| Console, UART, storage on Spresense | HOLD | output-disabled hardware run required |
| GNSS Add-on and IMU coexistence | HOLD | Multi-IMU is currently removed |
| Timing, GNSS accuracy, stability, flight | HOLD/out of scope | not claimed by this implementation |

## Next implementation slice

1. Add a Sony SDK-backed waf board and NuttX task dispatch.
2. Link the smallest ArduPilot library example before attempting Copter.
3. Save the Application SRAM/GNSS RAM linker map and apply fixed limits before
   the first hardware boot.

This work was produced with AI assistance and requires human review before any
upstream submission or hardware use.
