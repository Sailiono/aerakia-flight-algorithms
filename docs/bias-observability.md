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
6. Only after the data-contract audit, pre-register a materially different excitation-aware
   estimator hypothesis, evaluate it on clean train/tune data, and execute a newly sealed holdout
   once. Do not continue scalar P/Q tuning after the rejected static-prior study below.

The frozen cross-validation protocol is
[`validation/bias_observability_protocol_v1.json`](../validation/bias_observability_protocol_v1.json).
It is synthetic evidence, not a substitute for multi-pose calibration, temperature/vibration tests,
dual-antenna or motion-capture truth, or FCOne flight logs.

Its historical first 15-trial smoke separates execution from capability: all 15 pipelines executed,
all five zero-bias tracks passed, and all ten `+X` / `+X,-Y` boundary tracks failed stable
convergence and were right-censored. Terminal horizontal-bias P95 ranged from `0.0595` to
`0.2154 m/s²`.

That smoke also exposed seed `30000` for both v1 holdout trajectories because the original selector
applied smoke to every trajectory. V1 is therefore retained as diagnostic evidence but is not a
blind final holdout. The selector now excludes holdout trajectories in both `smoke` and
`train-tune`, honors `execution.smoke_seed_offset`, and reserves holdout access for `release` only.
The replacement one-shot design is recorded in
[`validation/bias_observability_protocol_v2_plan.json`](../validation/bias_observability_protocol_v2_plan.json):
its concrete holdout seeds and trajectory parameters must remain in an external protected-CI
manifest until clean baseline and candidate commits, gates, and regression tolerances are frozen.

## 2026-07-20 — static-prior A/B candidate rejected

The frozen paired study in
[`validation/public/g0_static_prior_ab_study.json`](../validation/public/g0_static_prior_ab_study.json)
ran `576` train/tune trials for each arm, with `1,152/1,152` successful executions and zero
execution failures. The frozen baseline passed `175/576` trials and had `390` right-censored
trials. The static-prior candidate passed `231/576` and had `341` right-censored trials.

The candidate improved tune-only outcomes: `49` paired trials moved from right-censored to settled
and there were zero baseline-pass-to-candidate-fail regressions. It did not generalize to train:
both arms remained at `0/16` non-zero-bias train passes, while both retained `16/16` zero-bias
train passes. The candidate was rejected and was not copied into the estimator or FCOne product
configuration.

The negative conclusion is intentionally narrow. The study does not prove universal mathematical
unobservability, and it does not justify widening convergence/accuracy gates. It indicates that
the present tilt--horizontal-bias coupling cannot be closed safely by a scalar static-prior or
process-noise adjustment. The next experiment must change the pre-registered excitation or
estimator hypothesis under a clean v2 protocol.

### Evidence and sealing limits

The A/B runs are diagnostic, not blind release evidence. They used compact output from a dirty tree;
historical summaries did not retain per-trial input CSV SHA-256, byte count, or row count; and the
v1 smoke had already exposed seed `30000` in both holdout trajectory families. No missing historical
SHA was fabricated. The runner now records input SHA-256, byte count, and data-row count before
compact deletion and fingerprints the protocol plus marker manifest.

The marker manifest
[`validation/bias_observability_information_markers_v1.json`](../validation/bias_observability_information_markers_v1.json)
is still `draft` and is analyzer-only: it is not an observability proof, rank test, estimator input,
or truth-assisted initializer. A sealed release holdout without a public marker still executes
ordinary metrics and gates and is reported as `unavailable_sealed_holdout`; it is not an execution
failure and must not receive a public information-state label. A future v2 protected-marker path
must remain separate from this public manifest.

## Pre-registered direction for an excitation-aware candidate

The next candidate is not allowed to use trajectory time or commanded motion as proof that bias is
observable. It must form a causal sliding-window information score from the estimated trajectory,
state-transition matrices, and actually accepted velocity/position observations. The intended local
information matrix is:

```text
G = sum(Phi^T H^T R^-1 H Phi)
```

After marginalizing nuisance states, the score targets
`[tilt_x, tilt_y, accel_bias_x, accel_bias_y, accel_bias_z]`. Effective rank, minimum eigenvalue,
condition number, per-direction information, accepted-aiding coverage, and freshness must all meet
frozen thresholds before the estimator may declare the bias observable or apply a coupled
tilt/bias correction. Until then it retains baseline propagation and a conservative unresolved
state. The existing fixed `information_ready_time_s` remains report-only.

The preferred estimator candidate is a sliding-window GNSS-velocity/IMU-preintegration solve that
returns a joint five-dimensional correction and full covariance for injection. It must first pass
causal analyzer-only tests before estimator source changes are allowed.

### Required positive and negative controls

- Static, fixed-attitude translation, yaw-only rotation, stale/rejected aiding, and incomplete
  directional motion must not declare full excitation.
- At least 95% of predeclared full-rank train/tune trajectories must declare information ready in
  the allowed window.
- The classifier may use no truth, future samples, trajectory identifier, or commanded motion.
- Train and tune must each improve terminal horizontal-bias P95 by at least `0.005 m/s2`; at least
  75% of nonzero signed-vector groups must improve.
- The existing `35 s` settling and `0.05 m/s2` terminal-P95 limits remain hard gates, and the P95
  time from information-ready to convergence must be no more than `10 s`.
- Numerical health must remain 100%, navigation recovery must remain zero, and covariance must stay
  finite, symmetric, and positive semidefinite with absolute 5D joint-NEES confidence bounds.

### Input-contract audit before v2

The v1 generator is better specified than the original multisine track, but it is not yet a
physical IMU contract. Acceleration is shifted as interval-start NED ZOH while body specific force
is evaluated at the row attitude; angular rate is an interval-average quaternion increment. During
rotation these are not a strictly common interval-average measurement. Static hints come directly
from trajectory truth, GNSS is idealized at 10 Hz with zero delay and lever arm, noise is expressed
per sample instead of as rate-independent density, and the campaign is fixed at 100 Hz. It omits
quantization, saturation, thermal drift, bias random walk, installation error, timing offset, and
real transition aerodynamics.

The v2 contract must use delta-angle/delta-velocity inputs or rigorously matched interval-average
specific force, replace truth-derived stationarity with a causal detector track, cover at least
50/100/200/400 Hz, and add delay, lever arm, quantization, random walk, and thermal profiles. These
changes require new train/tune inputs and a new sealed holdout; historical v1 trials remain evidence
and are not regenerated.
