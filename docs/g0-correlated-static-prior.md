# G0 Correlated Static-Prior Rejection

## Question

A single stationary gravity direction cannot distinguish small horizontal
accelerometer bias from a small roll/pitch error. This diagnostic candidate
preserved their local physical correlation after static alignment instead of
treating the two uncertainties as independent:

```text
b_accel = [R_nb^T g]x dtheta
```

It changed only the startup covariance. It did not read truth, command input,
or future samples in the estimator, and it did not add a state or alter the
public default profile.

## Method And Boundary

The frozen G0 v1 train/tune protocol ran two arms over the same 576 exact
trial keys: two train trajectories with 16 seeds, one tune trajectory with 32
seeds, and nine declared horizontal residual-bias vectors. Each arm completed
all 576 trials with zero execution failures. The runner persisted an
identity-checked per-trial record, including the input SHA-256, then resumed
interrupted local sessions without rerunning completed keys.

This is a reject/retain diagnostic, not promotion evidence. The v1 input
generator still uses a trajectory-truth static hint, both arms were executed
from a dirty source tree, and the historical v1 holdout was already exposed by
an earlier smoke run. The holdout was not opened here.

## Result

| Metric | Baseline | Candidate | Interpretation |
| --- | ---: | ---: | --- |
| Passed trials | 175 / 576 | 231 / 576 | 49 right-censored trials became settled; none changed from settled to censored. |
| Right-censored trials | 390 | 341 | Improvement is confined to part of the tune coverage. |
| Train terminal horizontal-bias P95 mean | 0.161897 m/s2 | 0.161219 m/s2 | Only 0.000679 m/s2 improvement; no train pass count changed. |
| Tune terminal horizontal-bias P95 mean | 0.041749 m/s2 | 0.036498 m/s2 | Measurable tune-only improvement. |

The candidate is **rejected**. It caused 22 material zero-bias regressions
across terminal error, RMSE, or settling time, and worsened five declared
mirror-symmetry checks. Those protections prevent a startup prior from winning
on injected-bias cases by making nominal cases less stable or more
direction-dependent.

The complete machine-readable result is
[`validation/public/g0_correlated_static_prior_rejection.json`](../validation/public/g0_correlated_static_prior_rejection.json).
Local complete outputs remain under the recorded `build/` paths in that file;
the large compact trial artifacts are intentionally not committed.

## Decision

The experimental runtime initializer was removed. There is no related public
API, default configuration, or FCOne profile change.

Do not continue P/Q or static-prior sweeps. The next materially different
candidate is a causal fixed-lag, excitation-aware joint tilt/accelerometer-
bias correction using actual accepted GNSS position/velocity observations,
proper process/preintegration covariance, and replay after the correction. It
requires the FCOne v2 physical IMU interval, timestamp, causal-stationarity,
and source-quality contract. It is therefore correctly deferred until the
embedded integration path exists.
