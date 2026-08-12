# Multi-Pose Static Calibration Sealed Holdout

## Scope

This record closes one narrow synthetic question: whether the explicit C99
six-pose gravity-sphere seed improves the declared cold-start bias boundary
without regressing a baseline pass. It does not characterize a physical FCOne
IMU or qualify flight use.

The test was run exactly once from clean commit `2593f6349e3925bf63fd9a532b9f8212332b50db` using
`validation/multipose_static_calibration_protocol_v1.json`:

- `137` residual-bias boundary cases per seed: 1 nominal, 12 axis, 60
  pairwise, and 64 six-dimensional-corner cases;
- eight sealed seeds (`51001, 51003, 51009, 51021, 51031, 51043, 51059,
  51071`), for `1,096` paired trials;
- six static pose means, `400` samples per pose, causal quantized-IMU
  stationarity, and a 100 Hz / 40 s post-start replay; and
- the production C calibration CLI and validation runner, built with CMake
  4.2.3 and GCC 15.2.0.

The runner verified the frozen source manifest before opening the seed set.
Its protocol file SHA-256 was
`39c809b7f4b256ed2d7ccd211bd5789b243cd926e08605b0c8da816875d27fa7`.
The complete retained summary SHA-256 is
`4b1318529a8e16224dcf1cd41bafe3fef6d52cc96619392731bc41d83114c213`;
the full summary remains a disposable build artifact because it contains all
per-trial records. The compact, committed machine-readable record is
[`g0_multipose_static_calibration_sealed_v1.json`](../validation/public/g0_multipose_static_calibration_sealed_v1.json).

## Result

| Metric | One-pose baseline | Six-pose candidate |
| --- | ---: | ---: |
| Completed trials | 1,096 | 1,096 |
| Passed current cold-start gates | 482 | 1,096 |
| Baseline fail -> candidate pass | - | 614 |
| Baseline pass -> candidate fail | - | 0 |
| Calibration acceleration-bias error norm, mean / P95 / max | - | 0.001233 / 0.002044 / 0.002044 m/s2 |

All 96 axis, 480 pairwise, and 512 full-corner candidate trials passed. The
result therefore closes the selected **synthetic** holdout acceptance rule.

## Decision And Remaining Boundary

The public candidate may now be integrated as a reviewed **optional seed
mechanism**. It is not an automatic FCOne v2 enablement decision. Each private
IMU collector must still reject bad pre-arm windows using physical settling,
variance, vibration, clipping, temperature, axis-map revision, sensor
identity, selector generation, and persistence-integrity checks. ADIS, ICM,
and BMI must each produce and retain separate artifacts.

No claim is made here for scale, non-orthogonality, temperature drift,
vibration rectification, inter-IMU switching, GNSS-denied navigation, yaw
truth, target timing, actuator behavior, or flight safety. Those belong to
FCOne v2 hardware characterization, shadow logging, SIL/HIL, and controlled
flight gates.
