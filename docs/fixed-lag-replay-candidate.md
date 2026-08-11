# Fixed-Lag Replay Candidate

## Scope

This experiment is the first host-only candidate that performs a complete
state-correction transaction rather than scoring an offline bias proposal. It
uses a causal closed window of accepted GNSS position/velocity updates and
IMU-derived ESKF output:

```text
pre-update P/V innovations
        -> symmetric five-parameter sensitivity replay
        -> prior-regularized joint correction
        -> core nominal-state injection + attitude covariance reset
        -> canonical IMU/aiding replay
        -> innovation-only second-half cross-validation
```

The target is the five-dimensional subspace
`[right_tilt_x, right_tilt_y, accel_bias_x, accel_bias_y, accel_bias_z]`.
The solver reads no truth fields. Truth is used only by the separate campaign
evaluator after the candidate returns. The production ESKF and public API were
not changed.

The exact implementation is split deliberately:

- `src/eskf_internal.h` exposes a private host/validation transaction that
  reuses the core's own nominal injection and attitude reset Jacobian.
- `validation/validation_runner.c` injects at a pre-IMU, pre-aiding boundary
  and records raw pre-update P/V innovations and diagonal innovation variance.
- `validation/fixed_lag_sensitivity_proposal.py` performs the local solve,
  requires identical accepted-event schedules for the perturbation runs, and
  uses the latter half of the window as a truth-free innovation holdout.
- `validation/run_fixed_lag_replay_candidate_campaign.py` binds every trial to
  input and executable hashes and scores the corrected replay outside the
  solver.

## Frozen Development Matrix

Protocol: [`fixed_lag_replay_candidate_protocol_v1.json`](../validation/fixed_lag_replay_candidate_protocol_v1.json)

- 3 causal v2 motions: hover-axis pulses, takeoff/box/land, and yaw-quadrant hover;
- 9 signed accelerometer-bias vectors, including zero and four mirror pairs;
- train seeds `0..7` on the two train motions and tune seeds `1000..1007` on
  the yaw motion;
- 216 paired trials, 20 s windows, 100 Hz, delta-interval v2 input contract;
- baseline and candidate use the same generated input bytes.

The compact result is
[`validation/public/fixed_lag_replay_candidate_v1.json`](../validation/public/fixed_lag_replay_candidate_v1.json).
The matrix is opened development evidence, not a sealed holdout.

## Result

| Scope | Trials with an accepted candidate | Baseline terminal bias P95 mean | Candidate terminal bias P95 mean | Paired delta |
| --- | ---: | ---: | ---: | ---: |
| Train | 61 | `0.16839 m/s2` | `0.16268 m/s2` | `-0.00571 m/s2` |
| Tune | 30 | `0.04172 m/s2` | `0.03812 m/s2` | `-0.00360 m/s2` |
| All accepted windows | 91 | `0.12663 m/s2` | `0.12161 m/s2` | `-0.00502 m/s2` |

The candidate was **rejected for promotion**:

- `125/216` proposals were rejected before scoring: `120` failed the
  innovation holdout, `3` changed the perturbation accepted-event schedule,
  and `2` failed corrected replay health/schedule checks.
- Only `17/24` signed nonzero trajectory/vector groups improved (`70.8%`),
  below the frozen `75%` requirement.
- Four zero-bias trials regressed terminal horizontal-bias P95 by more than
  `0.005 m/s2`.
- Twelve accepted trials had attitude-RMSE regression greater than the frozen
  `0.05 deg` tolerance.

The integrity checks that did pass are still useful: every executed baseline
and candidate replay remained finite and healthy, navigation recovery stayed
at zero, and accepted P/V event schedules matched for the accepted candidates.
They show that the transaction is executable and auditable; they do not show
that the correction is safe or general.

## Engineering Decision

Keep the production 15-state ESKF unchanged. Do not expose this candidate as a
public estimator API, do not enable it in FCOne, and do not grant it control
authority. The result demonstrates a real algorithmic experiment and narrows
the failure mode: fitting causal P/V innovations over one window can improve
some injected-bias cases but still overfit trajectory/model residuals and
produce zero-bias or attitude regressions.

The next candidate must add the missing full-lag information rather than tune
this rejected correction: full 15x15 lag covariance and process/preintegration
noise, explicit reset Jacobians through the window, physical source/arrival
timestamps, and a new clean protocol with a sealed holdout. Until that exists,
the appropriate FCOne behavior is baseline ESKF plus private supervision and
shadow logging.
