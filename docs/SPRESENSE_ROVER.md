# Spresense regular-front-steering ArduRover bring-up

## Status

This profile links the full ArduRover 4.7.0 vehicle archive to the existing
output-disabled `AP_HAL_Spresense`. It targets a conventional four-wheel RC
car with front steering and one drivetrain throttle channel. The selected
vehicle is a Tamiya CC-02M using the adopted nominal 252 mm wheelbase and a
shaft-driven 4WD drivetrain. It is a software and
output-disabled hardware feasibility artifact, not drive-ready firmware. One
combined-board sensor/command hardware gate passed on the original Rover
profile. The latest software adds fail-closed shadow output, a reversible GCS
mission-protocol test and two real Rover SITL gates: a disarmed GCS transaction
and the standard ArduPilot autonomy sequence. Both tested-code SITL gates pass,
while the latest hardware GCS mission round-trip remains HOLD after a warm-
reset sensor-stream failure. See the
[original hardware evidence](evidence/SPRESENSE_M1_ROVER_20260804.md) and the
[GCS/autonomy evidence](evidence/SPRESENSE_M1_ROVER_GCS_AUTONOMY_20260804.md).
The selected chassis inputs and their evidence levels are recorded separately
in the [CC-02 geometry evidence](evidence/SPRESENSE_M1_ROVER_CC02_GEOMETRY_20260806.md).

The selected upstream frame path is the standard ArduRover regular frame:

| Function | ArduPilot default | Intended later hardware |
|---|---|---|
| Front steering | `SERVO1_FUNCTION=26` (`GroundSteering`, CH1) | steering servo |
| Throttle | `SERVO3_FUNCTION=70` (`Throttle`, CH3) | single 4WD drivetrain ESC |

This is not skid steering, an omni frame or four-wheel steering. No custom
steering mixer was added. `AP_MotorsUGV::output_regular()` remains the source
of steering/throttle calculation.

## Target chassis geometry

The vehicle target is fixed as follows:

| Property | Current value | Evidence status |
|---|---:|---|
| Chassis | Tamiya CC-02 | selected by the user |
| Scale | 1/10 | Tamiya CC-02 kit specification |
| Frame | ladder frame | CC-02 chassis specification |
| Steering layout | front steering | regular-frame design assumption |
| Drivetrain | shaft-driven 4WD, single ESC | chassis-level design assumption |
| Motor/driveline layout | longitudinal front-mid motor, gearbox and propeller shafts to both axles | CC-02 chassis specification |
| Differentials | front/rear 3-bevel | type is from the CC-02 specification; installed open/locked state is unverified |
| Suspension/dampers | front/rear 4-link rigid axles with CVA oil dampers | CC-02 chassis specification |
| Wheelbase | 252 mm (`0.252 m`) | official CC-02M nominal; adopted by user direction |
| Tread | front 164 mm, rear 167 mm | official CC-02M/90 mm tire nominal; adopted by user direction |
| Tire size | 33 mm width × 90 mm diameter | official CC-02M nominal; adopted by user direction |
| Loaded rolling circumference | TODO: unmeasured loaded rolling circumference | HOLD |
| Official-reference pinion/gear ratio | 16T / 17.33:1 | Item 58715 kit standard; installed gearing is unverified |
| Official selectable gear-ratio range | 11.09:1 to 29.28:1 | CC-02 chassis capability; installed ratio is unverified |
| Installed motor | 13.5T brushless | user-specified; manufacturer/model/KV are unverified |
| Official-reference motor/ESC | RS540 / ESC separately required | Item 58715 kit reference only; not the installed motor specification |
| Installed ESC | TODO: model and rating unverified | HOLD |
| Top-view steering displacement | 50 mm lock-to-lock | user-specified |
| Steering displacement reference | tire leading edge | user-specified |
| Nominal road-wheel steering angle | 30 degrees per side | user-selected setting; not angle-measured |
| Bicycle-model turn radius | 0.436 m | calculated model value, not a measured turning radius |
| Actual minimum turning radius | TODO: unmeasured turning radius | HOLD |

The nominal geometry source is the Tamiya Toyota Land Cruiser 40 CC-02M kit,
Item 58715. At the user's direction, its official 252 mm wheelbase, 164/167 mm
tread and 33/90 mm tire values are now the active configuration contract rather
than reference-only values. This does not assert that the installed kit is
Item 58715 or that on-vehicle dimensions were measured. The official source is
[Tamiya Item 58715](https://www.tamiya.com/japan/products/58715/index.html),
while the common ladder-frame, driveline and suspension construction is also
described on the
[Tamiya CC-02 chassis page](https://www.tamiya.com/japan/products/product_info_ex.html?genre_item=6502%2Crc_base).
Body envelope, ground clearance, vehicle mass and payload are not populated:
they depend on the selected body, wheels, suspension setup and installed
electronics, and the available chassis specification does not establish the
actual vehicle values.

The nominal model uses `R = L / tan(delta)` with `L = 0.252 m` and
`delta = 30 degrees`, giving `R = 0.436 m`. This is a centerline bicycle-model
estimate. It does not include inner/outer Ackermann angle differences, linkage
compliance, tire slip or mechanical endpoint error, and is not accepted as an
actual minimum turning-radius measurement. For the later restrained geometry
gate, measure the actual road-wheel angles and driven path after the servo horn,
linkage, endpoints and mechanical stops are installed. Until then, do not
apply the model radius as a validated control parameter and do not enable
steering output. The
252 mm wheelbase and 33/90 mm tire size are adopted official nominal
configuration inputs, not measurements made by this software task. A nominal
90 mm circle has a calculated circumference of about 282.7 mm, but tire
deflection, tread and surface slip make that unsuitable as validated odometry
or speed calibration. Measure loaded rolling circumference separately. The
installed motor is recorded as the user-specified 13.5T brushless motor, but
manufacturer, model, KV, voltage rating and matched ESC remain unverified. The
reported 50 mm top-view steering displacement is preserved as a lock-to-lock
raw input measured at the tire leading edge. Together with the nominal 90 mm
diameter it geometrically suggests about 33.7 degrees per side under a centered
pivot approximation; 30 degrees per side is intentionally retained as the
user-selected nominal setting rather than claiming an angular measurement.

## Safety boundary

The profile is intentionally unable to move a vehicle:

- `AP_Arming_Rover::arm()` rejects normal and forced arming when
  `HAL_SPRESENSE_OUTPUT_DISABLED=1`.
- `AP_HAL_Spresense::RCOutput` records requested period/PWM values in a
  16-channel RAM-only shadow state so GCS readback and host tests can inspect
  Rover's logical outputs. It has no `/dev/pwm`, DShot or CAN backend, rejects
  every write request and always reports zero physical writes.
- `cork()`/`push()` update the shadow frame atomically but do not create an
  electrical signal. Runtime markers distinguish `SHADOW_ONLY` from
  `WRITE_REJECTED`.
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

## Reversible GCS mission-protocol gate

`m1_rover_gcs_sequence.py` is an instrumented MAVLink GCS client for the
output-disabled hardware gate. It verifies the ground-rover heartbeat and
disarmed state, backs up the current mission, uploads and downloads a bounded
three-item test mission, requests AUTO only while disarmed, checks shadow
`SERVO_OUTPUT_RAW` readback, and restores the original mission. It also sends
normal and forced ARM requests only to prove that both remain denied. A
failure after mission modification enters the restore path before evidence is
written.

```sh
python3 Tools/spresense/m1_rover_gcs_sequence.py \
  --port /dev/cu.usbserial-XXXXXXXX \
  --reset-dtr \
  --startup-timeout 180 \
  --output build/spresense-m1-rover-link-artifacts/gcs-sequence-evidence.json
```

This checks the same MAVLink mission/command protocol used by a GCS, but is not
a separate Mission Planner or QGroundControl GUI test. It never claims sensor
health from heartbeat alone, cannot arm the vehicle and cannot cause motion.

The mission client accepts both `MISSION_REQUEST` and `MISSION_REQUEST_INT` but
always replies with `MISSION_ITEM_INT`, avoiding legacy float-coordinate
rounding. It filters Fence/Rally traffic, treats item 0 HOME position as
vehicle-generated and ignores the transient `current` flag during round-trip
comparison. Duplicate item requests remain safe and covered by host tests.

The 2026-08-04 run on firmware `31a5aadd` received the ground-rover heartbeat
and matched the boot identity, but timed out waiting for `MISSION_COUNT`. The
mission was never downloaded or modified. This hardware gate is therefore
HOLD, not PASS. A complete board power cycle and cold-boot rerun are required;
a DTR reset is not equivalent to removing and reapplying board power.

## Real Rover SITL GCS transaction

`run_m1_rover_gcs_sitl.py` launches an isolated, wiped Rover SITL instance and
runs the same instrumented GCS client against the real MAVLink mission server.
It waits for mission storage initialization, verifies an initial empty mission,
round-trips distinct backup and test fixtures, enters AUTO while disarmed,
returns to HOLD, restores both the backup and initial state, and terminates the
exact child process. It never sends ARM:

```sh
python3 Tools/spresense/run_m1_rover_gcs_sitl.py \
  --output build/spresense-m1-rover-link-artifacts/gcs-sitl-evidence.json
```

This transaction passed at `28a899ebae`. The evidence records
`arm_command_sent=false`, `initial_state_restored=true` and all hardware,
physical-output, driving and autonomous-completion claims as false. It proves
the GCS transaction and recovery logic against Rover SITL, not a GUI GCS or
Spresense hardware.

## SITL autonomy sequence

The standard ArduPilot Rover `DriveMission` test is the executable autonomy
gate. It rebuilds Rover SITL from the current source tree and performs mission
upload, software ARM, AUTO entry, waypoint progression, mission completion and
DISARM:

```sh
python3 Tools/spresense/run_m1_rover_sitl_autonomy.py \
  --output build/spresense-m1-rover-link-artifacts/sitl-autonomy-evidence.json
```

This sequence passed at `28a899ebae`. It proves the upstream regular-front-
steering navigation sequence in simulation and the test harness around it. It
does not test Spresense sensor timing, electrical outputs or vehicle driving.

## Evidence and HOLD table

| Item | Current evidence | Status |
|---|---|---|
| Full Rover archive and regular frame path | host contract plus Sony cross-build | PASS at `31a5aadd` |
| Application/GNSS RAM boundaries | linker-map verifier and JSON report | PASS |
| Built-in GNSS/PWM/eMMC exclusion | Kconfig, symbol and artifact guards | PASS |
| Shadow output/readback | host tests, static guard and linked firmware | PASS in RAM only; electrical HOLD |
| Original Rover sensor/command gate | `a200430c`: GNSS/PWBIMU, ARM denial and CH1/CH3 dry-run | PASS; GNSS fix/accuracy HOLD |
| Latest Rover boot identity and heartbeat | `31a5aadd`: matching runtime and `MAV_TYPE_GROUND_ROVER` | PASS |
| Latest sensor runtime after warm reset | PWBIMU opened, then stream failed; GNSS sample absent | HOLD; do not regress original PASS claim |
| Latest hardware GCS mission round-trip | timed out waiting for `MISSION_COUNT`; no mission modified | HOLD pending cold boot |
| Real Rover SITL GCS transaction | `28a899ebae`: backup/test upload/download, disarmed AUTO/HOLD, full restore, no ARM command | PASS in simulation only |
| Tested-code SITL autonomous sequence | `28a899ebae`: upload, ARM, AUTO, waypoints, completion, DISARM | PASS in simulation only |
| Normal/forced arm denial | original hardware run and fail-closed implementation | PASS; arming remains impossible |
| MANUAL_CONTROL to CH1/CH3 override | CH1=1800, CH3=1700 dry-run on original hardware run | PASS for input path only |
| Physical write count | reject-only shadow implementation and no backend | software guard PASS; electrical HOLD |
| Steering servo direction/range/neutral | no servo signal connected | HOLD |
| ESC neutral/brake/reverse/failsafe | no ESC signal connected | HOLD |
| Wheel geometry and control tuning | no moving vehicle test | HOLD |
| GNSS fix/accuracy and timing margins | runtime fix type 1; not measured | HOLD |
| Autonomous mission logic | full standard Rover mission in SITL | PASS in simulation only |
| Hardware autonomous driving | no enabled actuator or moving vehicle test | HOLD |

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
