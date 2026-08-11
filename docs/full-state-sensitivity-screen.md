# Full-State Sensitivity Screen

## Question

The five-dimensional fixed-lag replay candidate was rejected because it left
position, velocity, yaw, and gyro-bias nuisance states implicit. This screen
tests whether including all 15 error-state components and the full 15x15
boundary covariance fixes that limitation before investing in a larger
candidate campaign.

The diagnostic still uses no truth in the solver:

1. The native runner captures the full covariance at the pre-IMU/pre-aiding
   window boundary.
2. Thirty symmetric replays estimate pre-update P/V innovation sensitivity to
   all 15 error-state coordinates.
3. A full-covariance MAP step is computed from the first half of the window.
4. A pre-registered trust region tries scales `1, 0.5, 0.25, 0.125, 0.0625`.
5. Only a replay that preserves accepted-event schedule and health and reduces
   the innovation norm in the second half of the window is eligible for
   external truth scoring.

The process/preintegration covariance across the lag interval and full
innovation cross covariance are still absent. This is a diagnostic screen, not
a smoother, product estimator, or flight feature.

## Frozen Screen

The protocol is
[`full_state_sensitivity_screen_protocol_v1.json`](../validation/full_state_sensitivity_screen_protocol_v1.json):

- 60 trials: three causal v2 motion families, five bias vectors, four seeds;
- includes zero bias, signed X pair, and a signed diagonal pair;
- all inputs use the same 100 Hz delta-interval v2 synthetic contract;
- all original replay and candidate artifacts are deleted after compact hashes
  and metrics are recorded.

The machine-readable result is
[`full_state_sensitivity_screen_v1.json`](../validation/public/full_state_sensitivity_screen_v1.json).

## Result

| Outcome | Count |
| --- | ---: |
| Full-state candidate accepted for external score | 24 / 60 |
| Rejected by trust-region validation | 34 / 60 |
| Rejected because a finite perturbation changed the event schedule | 2 / 60 |
| Accepted nonzero trials improving terminal horizontal bias P95 | 12 / 19 (63.2%) |
| Accepted zero-bias material regressions | 1 |
| Accepted attitude-RMSE material regressions | 1 |

On the 24 accepted candidates, position RMSE decreased in every case (mean
delta `-0.00307 m`). That is expected because the diagnostic is allowed to
correct position and velocity from the same P/V observations. It is not proof
that the intended accelerometer-bias improvement is reliable:

- terminal horizontal-bias P95 had a mean delta of only `-0.00227 m/s2` and a
  P95 regression of `+0.01108 m/s2`;
- only 13 of 24 accepted trials improved that bias metric;
- one yaw-quadrant positive-X case increased attitude RMSE by `0.10515 deg`;
- one yaw-quadrant zero-bias case increased terminal horizontal-bias P95 by
  `0.01798 m/s2`.

## Decision

The full-state screen is **rejected**. It is a better bounded diagnostic than
the 5D candidate because it captures full boundary covariance and safely rejects
nonlinear gate changes, but it cannot distinguish the desired bias correction
from residual navigation fitting often enough to pass its own zero-bias,
attitude, and nonzero-direction safeguards.

Do not expand this exact solver to a 216-trial campaign; its frozen screen has
already failed. The correct next technical boundary is not another scaling
sweep. It is a genuine fixed-lag/smoothing formulation with complete lag
transition and process/preintegration covariance, innovation cross covariance,
and physical source/arrival timing. That work should be designed with FCOne v2
transport evidence rather than promoted from this host-only diagnostic.
