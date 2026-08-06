# Spresense M1 Rover CC-02 geometry evidence — 2026-08-06

## Scope and evidence levels

This record fixes the intended regular-front-steering vehicle as a Tamiya
CC-02 without enabling an actuator. Values explicitly supplied or selected by
the user are kept separate from calculated model values and hardware
measurements:

| Property | Value | Evidence level |
|---|---:|---|
| Wheelbase | 250 mm | user-specified target |
| Tire diameter | approximately 90 mm | user-specified estimate |
| Steering displacement | 50 mm lock-to-lock at tire leading edge | user-specified raw input |
| Nominal steering angle | 30 degrees per side | user-selected setting; not angle-measured |
| Nominal tire circumference | 282.7 mm | calculated; not loaded rolling circumference |
| Bicycle-model turn radius | 0.433 m | calculated; not measured driving radius |

The radius uses `R = L / tan(delta)` with `L = 0.250 m` and
`delta = 30 degrees`. It excludes Ackermann inner/outer angle differences,
linkage compliance, tire deformation and surface slip. The model value must
not be represented as a measured minimum turning radius or applied as a
validated driving parameter.

## Machine-readable contract

The source manifest and build generator now carry the CC-02 geometry. The
clean cross-build artifact recorded:

```text
m1.rover.chassis=tamiya-cc02
m1.rover.drivetrain=shaft-driven-4wd-single-esc
m1.rover.wheelbase_mm=250
m1.rover.tire_diameter_mm=90
m1.rover.loaded_rolling_circumference_mm=hardware-HOLD-unmeasured
m1.rover.steering_top_view_displacement_mm=50
m1.rover.steering_displacement_span=lock-to-lock
m1.rover.steering_displacement_reference=tire-leading-edge
m1.rover.steering_angle_deg=30
m1.rover.steering_angle_status=user-selected-nominal-not-measured
m1.rover.turn_radius_m=0.433
m1.rover.turn_radius_model=wheelbase-over-tan-steering-angle
m1.rover.turn_radius_status=calculated-not-measured
```

The static contract verifier rejects a source manifest or generator that drops
or reclassifies these values.

## Host and Sony cross-build result — PASS

- tested commit: `373c34be4b812e81571fa4c6b6358de4a8cc49e3`
- project tree: clean
- Sony ARM GCC: 10.3.1
- `nuttx.spk` SHA-256:
  `9414ee977319fed9b238a68c5653ffc89131f7297411c3279d9f5c22b4676249`
- host regression: PASS, outputs disabled and physical writes zero
- Application SRAM heap envelope: 608176 bytes
- GNSS RAM heap: 655360 bytes, complete region
- GNSS RAM code/rodata/data/bss: all zero bytes
- hardware runtime verified: false

The artifact retained `m1.outputs=disabled`, `m1.arming=always-denied` and
`m1.physical_write_expected=0`. No board power, DTR reset, flash or physical
output operation was performed for this evidence.

## Remaining geometry HOLD

- loaded rolling circumference;
- actual inner and outer road-wheel angles at servo endpoints;
- measured minimum turning radius under load;
- steering neutral, direction, linkage compliance and mechanical stops;
- speed calibration, tire slip and control tuning;
- any physical steering, throttle or vehicle motion.

This work was produced with AI assistance and requires human review before any
physical-output implementation or hardware use.
