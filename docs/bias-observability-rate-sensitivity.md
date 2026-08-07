# G0 Rate Sensitivity Study

This study is a diagnostic of the causal excitation analyzer, not a change to
the ESKF and not a qualification gate.  It uses the frozen
`bias_cv_hover_axis_pulses` trajectory for 32 seconds, a fixed 10 Hz position
and velocity aiding stream, and IMU rates of 50, 100, 200, and 400 Hz.

The three profiles are:

- `continuous_density`: the v2 white-noise-density contract;
- `zero_sensor_noise`: a deterministic geometry control;
- `fixed_sample_noise`: a deliberately non-physical profile that holds
  per-sample noise constant across rates.

The runner is [`validation/run_bias_observability_rate_sensitivity.py`](../validation/run_bias_observability_rate_sensitivity.py).
The compact, hashed result is
[`validation/public/g0_rate_sensitivity.json`](../validation/public/g0_rate_sensitivity.json).

## Result

All 12 cases had ESKF numerical health `100%` and zero navigation recoveries.
The analyzer nevertheless showed rate- and window-dependent structural
information:

| Profile | Rate | Effective full-rank windows | Structural-ready windows | Final effective rank |
| --- | ---: | ---: | ---: | ---: |
| continuous density | 50 Hz | 6 | 0 | 3 |
| continuous density | 100 Hz | 21 | 10 | 3 |
| continuous density | 200 Hz | 21 | 11 | 3 |
| continuous density | 400 Hz | 44 | 34 | 3 |

The zero-noise and fixed-sample-noise controls preserve the same qualitative
result.  Therefore the finding is not explained by sensor noise-density
conversion alone.  The finite trajectory excites several directions early,
then the trailing 20-second window loses the early axis pulses during the
hover/settling segment.  The analyzer also omits process noise, preintegration
covariance, aiding temporal correlations, and error-reset Jacobians; its
condition and rank values are consequently not estimator confidence.

## Decision

Do not lower thresholds and do not use this analyzer to trigger a fixed-lag
correction.  The next experiment must make the semantics explicit and testable:

1. compare trailing-window, cumulative-history, and bounded-memory information
   with the same physical aiding epochs;
2. add the missing preintegration/process covariance and reset handling to the
   diagnostic model, or state a deliberately conservative approximation;
3. require rate-invariant acceptance and persistence/hysteresis behavior before
   implementing any causal estimator correction;
4. run a protected holdout only after those invariants pass.

This closes a modeling question, not the horizontal accelerometer-bias
convergence problem.  The ESKF remains unchanged and suitable for continued
host testing while the analyzer is corrected.
