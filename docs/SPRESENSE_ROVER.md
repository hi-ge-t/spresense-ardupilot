# Spresense regular-front-steering ArduRover bring-up

## Status

This profile links the full ArduRover 4.7.0 vehicle archive to the existing
output-disabled `AP_HAL_Spresense`. It targets a conventional four-wheel RC
car with front steering and rear-wheel throttle. It is a software and
output-disabled hardware feasibility artifact, not drive-ready firmware.

The selected upstream frame path is the standard ArduRover regular frame:

| Function | ArduPilot default | Intended later hardware |
|---|---|---|
| Front steering | `SERVO1_FUNCTION=26` (`GroundSteering`, CH1) | steering servo |
| Throttle | `SERVO3_FUNCTION=70` (`Throttle`, CH3) | rear drive ESC |

This is not skid steering, an omni frame or four-wheel steering. No custom
steering mixer was added. `AP_MotorsUGV::output_regular()` remains the source
of steering/throttle calculation.

## Safety boundary

The profile is intentionally unable to move a vehicle:

- `AP_Arming_Rover::arm()` rejects normal and forced arming when
  `HAL_SPRESENSE_OUTPUT_DISABLED=1`.
- `AP_HAL_Spresense::RCOutput` records and rejects output requests. It has no
  `/dev/pwm`, DShot or CAN backend and always reports zero physical writes.
- Sony NuttX is built with `CXD56_PWM` and generic `PWM` disabled.
- MAVLink `MANUAL_CONTROL` may update Rover's CH1/CH3 RC overrides for a
  dry-run. This proves the input mapping only; it cannot enable throttle or a
  physical steering pulse.
- No RC receiver backend, output enable, independent actuator interlock,
  arming permission or physical-output write was added.

Do not connect servo or ESC signal wires while using this artifact as evidence.
Physical steering and throttle require a later, separately approved phase with
an output-enable design, neutral/failsafe proof, emergency stop and explicit
bench procedure.

## Hardware profile

`spresense-m1-rover-link` fixes the same sensor and storage contract as the
validated M1 Copter foundation:

- built-in CXD5602 GNSS disabled;
- CXD5610 GNSS Add-on required at `/dev/gps2`;
- all 640 KiB GNSS RAM enabled as a dedicated heap;
- CXD5602PWBIMU required at `/dev/imu0` on SPI5;
- development storage fixed to microSD at `/mnt/sd0`;
- eMMC remains a final carrier candidate only;
- no automatic device, sensor or storage fallback.

PWBIMU uses pins shared with the eMMC function. PWBIMU and eMMC coexistence is
therefore still a hardware-design HOLD and is not inferred from this build.

## Reproducible build

The Sony GCC 10.3.1 toolchain and pinned Spresense SDK submodule are required.
From the repository root:

```sh
python3 Tools/spresense/run_host_tests.py
python3 Tools/spresense/build_m1_rover_firmware.py --jobs 4
```

The clean-tree build produces
`build/spresense-m1-rover-link-artifacts/` containing:

- `nuttx.spk`: guarded flash image;
- `nuttx`: final ELF;
- `nuttx.map`: linker map;
- `nuttx.config`: exact NuttX configuration;
- `memory-layout.json`: Application SRAM and GNSS RAM
  code/rodata/data/bss/heap boundaries;
- `ARTIFACTS.manifest`: source/toolchain/profile/safety contract and hashes.

The build fails if the embedded vehicle identity is not `ArduRover V4.7.0`,
the entrypoint is not the unique strong `ardurover_spresense_main`, the Rover
arming guard or reject-only RCOutput is absent, built-in GNSS/PWM/eMMC symbols
are present, GNSS RAM is not the complete 640 KiB heap, or the repository and
submodule identities do not match.

## Output-disabled hardware gate

Use the repository flash wrapper; it rejects a dirty artifact, mismatched
profile, missing safety contract or modified hash. The Spresense DTR flash
skill procedure is mandatory for the actual write.

```sh
SPFC_PROFILE=spresense-m1-rover-link \
  Tools/flash_spresense.sh /dev/cu.usbserial-XXXXXXXX --preflight

SPFC_PROFILE=spresense-m1-rover-link \
SPFC_FLASH_ACK=<project_commit> \
  Tools/flash_spresense.sh /dev/cu.usbserial-XXXXXXXX --execute
```

With only Spresense, Multi-IMU, GNSS Add-on and microSD connected, run:

```sh
python3 Tools/spresense/m1_rover_serial_check.py \
  --port /dev/cu.usbserial-XXXXXXXX \
  --reset-dtr \
  --output evidence/spresense-m1-rover-runtime.json
```

The runtime gate requires all of the following in one session:

1. `SPRESENSE_M1_ROVER_BOOT=LOOP` and an ArduPilot heartbeat with
   `MAV_TYPE_GROUND_ROVER`;
2. runtime commit identity matching the artifact manifest;
3. live PWBIMU samples and a GNSS sample consumed by ArduPilot;
4. normal ARM and force ARM both returning failure, followed by an unarmed
   heartbeat;
5. `MANUAL_CONTROL y/z` appearing as positive CH1 steering and CH3 throttle
   RC overrides;
6. `SPRESENSE_M1_OUTPUT=WRITE_REJECTED`, with no physical output backend.

The CH1/CH3 check is a dry-run input-path result. It does not prove servo pulse
width, ESC behavior, steering direction, vehicle geometry or motion.

## Evidence and HOLD table

| Item | Current evidence | Status |
|---|---|---|
| Full Rover archive and regular frame path | host contract plus Sony cross-build | required before commit |
| Application/GNSS RAM boundaries | linker-map verifier and JSON report | required before commit |
| Built-in GNSS/PWM/eMMC exclusion | Kconfig, symbol and artifact guards | required before commit |
| Rover boot and ground-rover heartbeat | combined-board runtime gate | `TODO: 未確認` until saved |
| GNSS/PWBIMU live in Rover | combined-board runtime gate | `TODO: 未確認` until saved |
| Normal/forced arm denial | combined-board runtime gate | `TODO: 未確認` until saved |
| MANUAL_CONTROL to CH1/CH3 override | combined-board dry-run gate | `TODO: 未確認` until saved |
| Physical write count | reject-only implementation and no backend | physical measurement HOLD |
| Steering servo direction/range/neutral | no servo signal connected | HOLD |
| ESC neutral/brake/reverse/failsafe | no ESC signal connected | HOLD |
| Wheel geometry and control tuning | no moving vehicle test | HOLD |
| GNSS accuracy and timing margins | not measured by this gate | HOLD |
| Driving safety or autonomy | out of scope | HOLD |

Passing the output-disabled gate means ArduRover can boot and process a normal
front-steering command path on Spresense without being able to actuate the car.
It does not mean the car can be driven yet.

## Next physical-output phase (not implemented)

A later phase must remain separate and should begin with wheels off the ground
and propulsion power isolated. Before adding a physical backend it must define
and verify at least:

- the exact Spresense PWM pins, timer ownership and voltage interface;
- independent output enable/readback and an accessible emergency stop;
- steering center, direction and mechanical end stops;
- ESC neutral, brake/reverse mode, signal-loss response and power sequencing;
- startup/disarm/watchdog/link-loss behavior at the actual signal pins;
- a staged test from logic analyzer, to servo only, to motor with wheels raised,
  and only then a restrained low-speed rolling test.

Until that phase is explicitly approved, `HAL_SPRESENSE_OUTPUT_DISABLED=1`,
arming rejection and zero physical writes are release conditions.
