# Spresense M1 regular-front-steering ArduRover evidence — 2026-08-04

## Scope

This record covers one output-disabled ArduRover bench run on Sony Spresense
with the CXD5610 GNSS Add-on and CXD5602PWBIMU Multi-IMU Add-on connected. It
checks the conventional front-steering software path: `GroundSteering` on CH1
and `Throttle` on CH3. It does not enable or measure a servo, ESC, PWM, DShot,
CAN or any other physical actuator output, and it is not driving evidence.

## Reproducible identities

- upstream Rover baseline: `1511f27194f1dcc3728270883047bdf022b3fd53`
- target firmware commit: `a200430c9ae191b9234fcdee318151a99f1b1037`
- runtime-checker commit: `8b225f139959c709cda0adb61e14729bb0ac5198`
- Sony SDK commit: `7fd61b2c03f06a4ff0302b84c755e58c338788b2`
- MAVLink commit: `288b907c384a892c8519bfe271682424b1e1a3a0`
- compiler: Sony ARM GCC 10.3.1
- profile: `spresense-m1-rover-link`
- clean `nuttx.spk` SHA-256:
  `ed66ab9c347e4436e05177dca756dd09fbd639ab77798af00201e7f1ca67237a`

The guarded image was flashed through the main-board CP2102N UART with DTR
reset. The runtime identity reported `a200430c`, matching the clean artifact
manifest. The detailed machine-readable capture remains an ignored build
artifact at
`build/spresense-m1-rover-link-artifacts/runtime-evidence.json`.

## Build and memory result

The host contracts and Sony SDK cross-build passed with the full ArduRover
archive, regular-frame path, built-in GNSS disabled, CXD5610 GNSS Add-on and
PWBIMU required, microSD selected, eMMC/PWM disabled, and arming/output guards
intact. The linker-map verifier reported:

| Region | code | rodata | data | bss | heap envelope |
|---|---:|---:|---:|---:|---:|
| Application SRAM | 769400 | 128424 | 4024 | 60008 | 608176 |
| GNSS RAM | 0 | 0 | 0 | 0 | 655360 |

Values are bytes. Static code/data/bss/rodata remained outside GNSS RAM, and
the complete 640 KiB GNSS region was retained as its required heap.

## Hardware runtime result

The final capture completed at `2026-08-04T06:28:42.608781+00:00` and passed
the combined fail-closed gate:

- an ArduPilotMega heartbeat reported `MAV_TYPE_GROUND_ROVER`;
- the Rover HAL-loop marker and runtime commit identity matched the artifact;
- the CXD5610 sample reached `AP_GPS_Spresense` and was consumed by AP_GPS;
- `GPS_RAW_INT` reported fix type 1 and zero visible satellites. This is
  explicitly a no-fix result, not GNSS position or accuracy evidence;
- PWBIMU produced two time-ordered, nonzero and changing RAW_IMU samples:
  `(-1156, -4317, -8687, 0, 0, 0)` at 9423881 microseconds and
  `(-1158, -4317, -8688, 0, 0, 0)` at 9703264 microseconds;
- the bounded PWBIMU recovery path was attempted and succeeded without
  reaching its exhaustion guard;
- normal and forced ARM requests both returned result 4
  (`MAV_RESULT_FAILED`), and no heartbeat advertised armed state;
- a MAVLink `MANUAL_CONTROL` dry-run appeared as CH1=1800 for front steering
  and CH3=1700 for throttle inside Rover;
- `SPRESENSE_M1_OUTPUT=WRITE_REJECTED` was observed, and the linked HAL still
  contains no physical output backend.

The CH1/CH3 observation proves only the normal front-steering input mapping.
It does not prove an electrical pulse, servo direction, neutral position,
mechanical travel, ESC behavior or vehicle motion.

## Reproduction

After producing and flashing the guarded clean artifact, run:

```sh
python3 Tools/spresense/m1_rover_serial_check.py \
  --port /dev/cu.usbserial-XXXXXXXX \
  --reset-dtr \
  --output build/spresense-m1-rover-link-artifacts/runtime-evidence.json
```

The checker requests normal and forced arming only to prove both requests are
rejected. It then sends a bounded manual-control dry-run while the arming and
physical-output guards remain active.

## Remaining HOLD items

- GNSS fix, position accuracy, latency and update-rate timing;
- labelled PWBIMU axis orientation, signs and scale;
- scheduler jitter and runtime heap/stack high-water measurements;
- physical steering signal, direction, center, range and mechanical stops;
- ESC neutral, brake, reverse, failsafe and propulsion power sequencing;
- independent output enable/readback and emergency stop;
- Multi-IMU SPI5 and final eMMC pin coexistence;
- control tuning, restrained low-speed driving, autonomy and flight.

Development storage remains microSD. eMMC remains a final carrier candidate,
and no automatic storage or sensor fallback was added.

This work was produced with AI assistance and requires human review before any
physical-output implementation or hardware use.
