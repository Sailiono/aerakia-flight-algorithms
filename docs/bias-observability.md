# Horizontal accelerometer-bias observability

## Why G0 retains a bias-convergence gate

A stationary accelerometer measures one gravity direction. A small horizontal accelerometer bias
and a small roll/pitch error can produce the same measurement, so a single-pose cold start cannot
identify both independently. The public adapter therefore treats static alignment as coarse tilt
and bias initialization; later motion and external velocity/position constraints must remove the
remaining ambiguity.

For a `0.15 m/s²` horizontal residual, the equivalent small-angle ambiguity is approximately
`0.15 / 9.80665 = 0.0153 rad = 0.876°`. The paired diagnostic measured about that much initial tilt
error for both `+X/-Y` and `-X/+Y` residuals.

## Retained failure and causal isolation

The first deterministic three-sigma box contains 137 cases: zero bias, every signed single axis,
all signed pairs, and all 64 six-dimensional corners. Numerical health and navigation consistency
pass in all cases, but only 120/137 meet every convergence/accuracy gate. All 17 first-run misses
contain the accelerometer `+X/-Y` direction and exceed the frozen 35 s settling budget.

A matched 32-seed diagnostic produced:

| Residual | Misses at 35 s | Final horizontal-bias error median / P95 / maximum |
| --- | ---: | ---: |
| `+X/-Y` | 22/32 | `0.0486 / 0.0650 / 0.0840 m/s²` |
| `-X/+Y` | 8/32 | retained in the paired report |
| zero | 3/32 | retained in the paired report |

All 96 paired trials remained healthy and triggered no navigation recovery. The threshold was not
widened after observing the result.

The strongest isolation used the same 96 sensor streams but supplied the correct initial attitude
while still estimating bias from the static IMU mean. The three groups then became nearly
indistinguishable: final median bias-vector error was about `0.0184 m/s²`, and final P95 was about
`0.0374 m/s²`. This makes an axis-sign bug unlikely and identifies the cold-start tilt/bias
ambiguity as the dominant cause.

The remaining direction difference is consistent with a common `+Y` residual in the synthetic
trajectory adding to one sign and cancelling the mirror sign. The existing trajectory is not
time/symmetry complete, so it cannot be the only tuning or release track.

## Required evidence before changing P or Q

1. Audit endpoint versus interval-average specific-force semantics and raw quantization.
2. Cross-validate with non-multisine VTOL trajectories: hover pulses, takeoff-box-land, yaw
   quadrants, early transition, and gust/landing recovery.
3. Retain exact bias-vector grouping and right-censored non-convergence.
4. Export bias covariance and report 3D bias NEES, per-axis error, terminal five-second statistics,
   and continuous-five-second convergence.
5. Compare current independent startup covariance with a physically derived tilt/bias-correlated
   initialization only after the data-contract audit.
6. Evaluate a small predeclared process-noise/prior grid on train/tune data, select one candidate,
   and open a frozen holdout once.

The frozen cross-validation protocol is
[`validation/bias_observability_protocol_v1.json`](../validation/bias_observability_protocol_v1.json).
It is synthetic evidence, not a substitute for multi-pose calibration, temperature/vibration tests,
dual-antenna or motion-capture truth, or FCOne flight logs.

Its first 15-trial smoke separates execution from capability: all 15 pipelines executed, all five
zero-bias tracks passed, and all ten `+X` / `+X,-Y` boundary tracks failed stable convergence and
were right-censored. Terminal horizontal-bias P95 ranged from `0.0595` to `0.2154 m/s²`. The smoke
therefore has `execution_status=passed` and `capability_status=failed`; the 1,152-trial holdout stays
closed until a candidate design is selected from the train/tune experiment matrix.
