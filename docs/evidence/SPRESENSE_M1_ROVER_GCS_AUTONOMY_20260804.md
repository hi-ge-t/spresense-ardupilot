# Spresense M1 Rover GCS/autonomy evidence — 2026-08-04

## Scope and safety boundary

This record covers the output-disabled regular-front-steering Rover additions:
a RAM-only RCOutput shadow, a reversible MAVLink GCS mission sequence, and the
standard ArduPilot Rover `DriveMission` autonomy test in SITL. It also records
the bounded hardware attempts made with Spresense, Multi-IMU and GNSS Add-on
connected.

`HAL_SPRESENSE_OUTPUT_DISABLED=1` remained enabled. The hardware image has no
PWM, DShot or CAN actuator backend, all RCOutput writes are rejected, normal
and forced arming are denied, and the expected physical write count is zero.
No steering servo or propulsion output was enabled or measured. This is not
vehicle-driving evidence.

## Reproducible identities

- current software/test commit: `b455243f80f1906cd9d2fe77273f019f09bc9add`
- flashed hardware artifact commit: `31a5aaddb177a54e5bedee9f0a6cd351aea361f9`
- upstream Rover baseline: `1511f27194f1dcc3728270883047bdf022b3fd53`
- Sony SDK commit: `7fd61b2c03f06a4ff0302b84c755e58c338788b2`
- MAVLink commit: `288b907c384a892c8519bfe271682424b1e1a3a0`
- compiler: Sony ARM GCC 10.3.1
- profile: `spresense-m1-rover-link`
- flashed `nuttx.spk` SHA-256:
  `ead174956de5a15486279026b9ae15ae992ec0fb81546489de001aa496bce6ab`

The firmware artifact predates the final two test-tool hardening commits. Those
commits modify host-side GCS recovery logic only; the installed target image is
therefore identified separately and is not represented as a `b455243f80`
hardware build.

## Host, build and memory results

The host suite passed, including shadow-output state/cork/push behavior,
mission-protocol fake-link tests, backup/restore failure handling, disarmed
heartbeat checks and the manifest/static safety guards. The Sony SDK
cross-build and guarded flash completed successfully for `31a5aadd`.

The final map report retained all static sections in Application SRAM and the
complete 640 KiB GNSS RAM region as heap:

| Region | code | rodata | data | bss | heap envelope |
|---|---:|---:|---:|---:|---:|
| Application SRAM | 769656 | 128456 | 4024 | 59720 | 608176 |
| GNSS RAM | 0 | 0 | 0 | 0 | 655360 |

Values are bytes. The output-disabled artifact contract, built-in GNSS/PWM/
eMMC exclusion and no-automatic-fallback storage policy all passed.

## Tested-code SITL autonomy result — PASS

`run_m1_rover_sitl_autonomy.py` performed a full Rover SITL build from
`b455243f80` and ran `Rover.DriveMission`. The captured sequence completed:

1. GCS mission upload;
2. software ARM;
3. AUTO mode entry;
4. progression through the mission waypoints;
5. mission completion;
6. software DISARM.

The machine-readable evidence reports all six gates true and records
`driving_verified=false`, `physical_outputs_verified=false` and
`spresense_hardware_verified=false`. This is a complete simulated autonomous
sequence, not a claim that the Spresense vehicle can move autonomously.

## Hardware flash and GCS result — partial PASS / HOLD

The guarded `31a5aadd` image was written through the main-board CP2102N UART
using DTR preflight/reset handling. The subsequent GCS sequence observed a
runtime commit of `31a5aadd` and a ground-rover heartbeat. The target also
emitted the new `SPRESENSE_M1_OUTPUT=SHADOW_ONLY` and
`SPRESENSE_M1_OUTPUT=WRITE_REJECTED` markers during the hardware attempts.

The hardware GCS mission round-trip did not pass. The final attempt timed out
waiting for `MISSION_COUNT` after the request-list message. Consequently:

- no existing mission was downloaded;
- no test mission item was uploaded;
- no mission on the board was modified;
- AUTO was not entered;
- autonomous mission completion and motion remain unverified;
- physical writes remained zero by contract.

The same warm-reset sessions opened PWBIMU and produced an initial sample, then
reported a sensor stream failure; the required GNSS sample was not observed.
Earlier evidence at `a200430c` remains the retained proof that this exact
Multi-IMU/GNSS Add-on configuration can ingest both sensors and reject ARM.
The current result is recorded as a new cold-boot retest requirement, not as
proof of a software regression and not as a sensor PASS.

## Reproduction and next hardware gate

The simulation gate is reproducible with:

```sh
python3 Tools/spresense/run_m1_rover_sitl_autonomy.py \
  --output build/spresense-m1-rover-link-artifacts/sitl-autonomy-evidence.json
```

For hardware, first remove and reapply the complete board power. A DTR reset is
not a full power cycle. Start the GCS checker for the cold boot without
rebuilding or reflashing the already installed `31a5aadd` image:

```sh
python3 Tools/spresense/m1_rover_gcs_sequence.py \
  --port /dev/cu.usbserial-XXXXXXXX \
  --startup-timeout 180 \
  --output build/spresense-m1-rover-link-artifacts/gcs-sequence-evidence.json
```

PASS requires mission backup, deterministic test upload/download, disarmed AUTO
request handling, shadow readback and verified restoration of the original
mission. Any failure after modification must still restore the backup. A
separate cold-boot sensor gate is then required because two clients cannot both
consume the same one-time boot diagnostic stream.

## Remaining HOLD items

- cold-boot GCS mission backup/upload/download/restore on Spresense;
- current-image GNSS/PWBIMU runtime gate after full power cycle;
- GUI-specific Mission Planner or QGroundControl interaction;
- GNSS fix/accuracy, sensor timing and labelled IMU orientation;
- physical steering signal, center/direction/range and mechanical stops;
- ESC neutral/brake/reverse/failsafe and propulsion power sequencing;
- independent output enable/readback and emergency stop;
- restrained low-speed driving and any hardware autonomous run.

Development storage remains microSD, eMMC remains a final carrier candidate,
and no automatic device, sensor or storage fallback was added.

This work was produced with AI assistance and requires human review before any
physical-output implementation or hardware use.
