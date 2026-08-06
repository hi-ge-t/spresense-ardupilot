# Spresense M1 Rover CC-02 geometry evidence — 2026-08-06

## Scope and evidence levels

This record fixes the intended regular-front-steering vehicle as a Tamiya
CC-02 without enabling an actuator. Values explicitly supplied or selected by
the user are kept separate from calculated model values and hardware
measurements:

| Property | Value | Evidence level |
|---|---:|---|
| Wheelbase | 252 mm | official CC-02M nominal; user-adopted |
| Tread | front 164 mm, rear 167 mm | official CC-02M nominal; user-adopted |
| Tire size | 33/90 mm | official CC-02M nominal; user-adopted |
| Installed motor | 13.5T brushless | user-specified; model/KV unverified |
| Steering displacement | 50 mm lock-to-lock at tire leading edge | user-specified raw input |
| Nominal steering angle | 30 degrees per side | user-selected setting; not angle-measured |
| Nominal tire circumference | 282.7 mm | calculated; not loaded rolling circumference |
| Bicycle-model turn radius | 0.436 m | calculated; not measured driving radius |

The radius uses `R = L / tan(delta)` with `L = 0.252 m` and
`delta = 30 degrees`. It excludes Ackermann inner/outer angle differences,
linkage compliance, tire deformation and surface slip. The model value must
not be represented as a measured minimum turning radius or applied as a
validated driving parameter.

## Official CC-02 reference

The adopted official nominal source configuration is the
1/10-scale Tamiya Toyota Land Cruiser 40 CC-02M, Item 58715. Its official
specification records a 252 mm wheelbase, 164 mm front and 167 mm rear tread,
33/90 mm tires, ladder frame, longitudinal motor and shaft-driven 4WD,
front/rear 3-bevel differentials, front/rear 4-link rigid suspension, CVA oil
dampers, a kit-standard 16T pinion with 17.33:1 ratio, RS540-type motor and a
separately required ESC. The generic chassis page lists selectable ratios from
11.09:1 to 29.28:1:

- https://www.tamiya.com/japan/products/58715/index.html
- https://www.tamiya.com/japan/products/product_info_ex.html?genre_item=6502%2Crc_base

At the user's direction, the official wheelbase, tread and tire values are now
the active nominal configuration. They remain catalog values rather than
on-vehicle measurements. The installed motor is user-specified as a 13.5T
brushless motor; manufacturer, model, KV and voltage rating remain unverified.
Installed pinion and ratio, ESC and differential open/locked state remain
hardware HOLD.

## Machine-readable contract

The source manifest and build generator now carry the CC-02 geometry. The
clean cross-build artifact recorded:

```text
m1.rover.chassis=tamiya-cc02
m1.rover.chassis_scale=1/10
m1.rover.frame_construction=ladder-frame
m1.rover.motor_layout=longitudinal-front-mid
m1.rover.drivetrain=shaft-driven-4wd-single-esc
m1.rover.drive_transfer=gearbox-propeller-shafts-front-and-rear
m1.rover.differential_type=front-and-rear-3-bevel
m1.rover.differential_configuration=hardware-HOLD-not-inspected
m1.rover.suspension=front-and-rear-4-link-rigid
m1.rover.dampers=front-and-rear-CVA-oil
m1.rover.wheelbase_mm=252
m1.rover.wheelbase_status=official-cc02m-nominal-user-adopted
m1.rover.official_reference_wheelbase_class=CC-02M
m1.rover.official_reference_wheelbase_mm=252
m1.rover.wheelbase_reference_delta_mm=0
m1.rover.official_reference_front_track_mm=164
m1.rover.official_reference_rear_track_mm=167
m1.rover.front_track_mm=164
m1.rover.rear_track_mm=167
m1.rover.track_status=official-cc02m-nominal-user-adopted
m1.rover.tire_diameter_mm=90
m1.rover.tire_diameter_status=official-cc02m-nominal-user-adopted
m1.rover.official_reference_tire_width_mm=33
m1.rover.tire_width_mm=33
m1.rover.tire_width_status=official-cc02m-nominal-user-adopted
m1.rover.loaded_rolling_circumference_mm=hardware-HOLD-unmeasured
m1.rover.official_reference_kit_standard_pinion_teeth=16
m1.rover.official_reference_kit_standard_gear_ratio=17.33
m1.rover.official_supported_gear_ratio_min=11.09
m1.rover.official_supported_gear_ratio_max=29.28
m1.rover.installed_pinion_teeth=hardware-HOLD-unverified
m1.rover.installed_gear_ratio=hardware-HOLD-unverified
m1.rover.official_reference_kit_motor_class=RS540
m1.rover.official_reference_esc=separately-required
m1.rover.installed_motor=13.5T-brushless
m1.rover.installed_motor_type=brushless
m1.rover.installed_motor_turns=13.5
m1.rover.installed_motor_status=user-specified-model-unverified
m1.rover.installed_esc=hardware-HOLD-unverified
m1.rover.steering_top_view_displacement_mm=50
m1.rover.steering_displacement_span=lock-to-lock
m1.rover.steering_displacement_reference=tire-leading-edge
m1.rover.steering_angle_deg=30
m1.rover.steering_angle_status=user-selected-nominal-not-measured
m1.rover.turn_radius_m=0.436
m1.rover.turn_radius_model=wheelbase-over-tan-steering-angle
m1.rover.turn_radius_status=calculated-not-measured
```

The static contract verifier rejects a source manifest or generator that drops
or reclassifies these values.

## Host and Sony cross-build result — PASS

- tested commit: `209624b7dd131a05aa474b7f85160204e9b50511`
- project tree: clean
- Sony ARM GCC: 10.3.1
- `nuttx.spk` SHA-256:
  `e7ac35b4249c8c85b7c705ea482d5f19a56d51b033819887a17ea985316156a6`
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
- installed pinion, final gear ratio and ESC;
- installed motor manufacturer/model/KV/voltage rating and ESC;
- front/rear differential open or locked configuration;
- actual inner and outer road-wheel angles at servo endpoints;
- measured minimum turning radius under load;
- steering neutral, direction, linkage compliance and mechanical stops;
- speed calibration, tire slip and control tuning;
- any physical steering, throttle or vehicle motion.

This work was produced with AI assistance and requires human review before any
physical-output implementation or hardware use.
