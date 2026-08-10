# G0 score calibration result

The current analyzer's binary rank gate is unusable under sensor noise: the
stationary campaign produced `26/64` structural-ready false positives. We
therefore tested a continuous diagnostic score, the maximum minimum eigenvalue
over causal windows, without changing the ESKF.

## Frozen development split

- Calibration: stationary seeds `0--7`, four rates, 32 cases.
- Internal validation: stationary seeds `8--15`, four rates, 32 cases.
- External validation: stationary seeds `100--115`, four rates, 64 cases.
- Candidate threshold: `3 x` calibration P99 = `0.004936251852866821`.

The external null score pass rate was `0/64`. The old binary
`structural-ready` result on those same cases was `31/64` (`48.4%`), so the
score rejects the demonstrated analyzer false-positive mechanism.

## Noisy maneuver check

Using the same fixed threshold, continuous-density noisy replays on two
structural candidate maneuvers passed `64/64` each across four rates and seeds
`100--115`:

| Maneuver | Score pass rate | Minimum score |
| --- | ---: | ---: |
| `bias_cv_takeoff_box_land` | 64/64 | 0.09024 |
| `bias_cv_yaw_quadrant_hover` | 64/64 | 139.1852 |

This is a useful analyzer-screening result, not an estimator result. It does
not measure online accelerometer-bias convergence, covariance consistency,
real sensor timing, or flight safety. The score is not connected to ESKF
correction, supervisor qualification, or control authority.

The machine-readable artifacts are
[`g0_gate_null_split_validation.json`](../validation/public/g0_gate_null_split_validation.json)
and
[`g0_excitation_score_campaign.json`](../validation/public/g0_excitation_score_campaign.json).

## Next decision

Keep the score as a research diagnostic and calibrate it again with the FCOne
v2 IMU/timestamp/noise contract. Only after process/preintegration covariance,
reset Jacobians, aiding correlation, rate invariance, and a protected noisy
holdout are implemented may this become an input to a causal fixed-lag
estimator experiment. No threshold from this synthetic study is a flight gate.
