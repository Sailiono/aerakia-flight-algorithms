# G0 analyzer gate characterization

This campaign quantifies the current excitation analyzer's behavior before any
estimator-side fixed-lag correction is considered. It is deliberately separate
from the ESKF and cannot qualify a flight controller.

## Protocol

- Null: 32 seconds of `bias_cv_static_hold`, causal IMU stationarity, 10 Hz
  position/velocity aiding, continuous-density generated sensor noise.
- Structural controls: `bias_cv_takeoff_box_land` and
  `bias_cv_yaw_quadrant_hover`, with all generated measurement noise set to
  zero. These controls test geometry only; they are not noisy true-positive
  evidence.
- Rates: 50, 100, 200, and 400 Hz.
- Seeds: an explicit, disjoint seed range recorded in the JSON result.

Run it after building the native host runner:

```bash
python validation/run_bias_observability_gate_characterization.py \
  --runner build/host-regression/aerakia_validation_runner
```

The compact result is
[`validation/public/g0_gate_characterization.json`](../validation/public/g0_gate_characterization.json).

## Result

The reviewed run used seeds 0--15 at all four rates, for 64 stationary null
cases plus eight zero-noise structural controls.

| IMU rate | Stationary structural-ready false positives | Stationary full-rank cases |
| ---: | ---: | ---: |
| 50 Hz | 6/16 | 14/16 |
| 100 Hz | 5/16 | 15/16 |
| 200 Hz | 5/16 | 15/16 |
| 400 Hz | 10/16 | 16/16 |
| **Total** | **26/64 (40.625%)** | **60/64 (93.75%)** |

All 64 stationary ESKF runs had health ratio `1.0` and zero navigation
recoveries. All eight zero-noise structural controls reached full rank and
structural-ready at 50/100/200/400 Hz. The null therefore invalidates the
current noisy analyzer gate while the controls confirm that the selected
maneuvers contain useful ideal geometry.

## Interpretation rules

The primary null metric is the fraction of stationary cases for which the
analyzer reports any `structural_information_ready_analyzer_only` window. A
zero observed count is reported with a one-sided 95% binomial upper bound; it is
not converted into a runtime guarantee. Full-rank-without-ready is retained as
a secondary diagnostic because it can expose threshold/window interactions.

The zero-noise structural controls answer only whether the idealized local
geometry can produce five-dimensional rank under a prescribed maneuver. They do
not establish that noisy filtered-state linearization, process covariance,
reset handling, or real sensor timing will preserve that rank.

## Current engineering decision

Do not wire this analyzer into ESKF correction, supervisor qualification, or
flight-control authority. The observed stationary structural-ready false
positive rate is `26/64`, so the current gate is rejected. The analyzer remains
research-only until process/preintegration covariance,
cross-source aiding correlation, reset Jacobians, and persistence semantics are
modeled and calibrated across rates. The next estimator experiment is a
pre-registered covariance-aware or noise-null-calibrated diagnostic followed by
a clean v2 fixed-lag candidate protocol; no old holdout is reopened for tuning.
