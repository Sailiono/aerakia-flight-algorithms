# Fixed-lag bias proposal boundary

The previous G0 work established that the binary local-rank analyzer is too
noise-sensitive to trigger an estimator correction. The next estimator
hypothesis is now implemented as a host-only proposal solver:

```text
accepted GNSS P/V + IMU + baseline ESKF trajectory
            |
      causal 20 s window
            |
  nuisance-projected whitened design
            |
  prior-regularized MAP solve for
  [tilt_x, tilt_y, accel_bias_x/y/z]
```

`validation/fixed_lag_bias_proposal.py` consumes no truth fields and never
mutates or injects the ESKF state. It emits a correction proposal only when the
previously calibrated minimum-eigenvalue score passes. The prior is the
baseline filter's reviewed 5x5 tilt/bias covariance at the window start; the
`prior_information_scale` is an explicit research parameter, not a product
default.

The proposal is intentionally not called a smoother or estimator yet. It does
not repropagate the nominal state, replay covariance through the correction, or
run controller authority. Those are required before a C implementation can be
considered.

## Campaign and decision

The completed v1 campaign used two causal v2 motions (`takeoff_box_land` and
`yaw_quadrant_hover`), all nine signed residual-bias groups, and eight seeds:
`144` independent baseline replays. Four closed windows were evaluated per
replay where the calibrated information score admitted a proposal. Every
baseline replay remained numerically healthy and none entered navigation
recovery.

The three explicit prior-information scales all failed the non-regression
criterion. The least harmful scale, `1.0`, still made the aggregate result
worse:

| Scope | Baseline mean / P95 | Proposal mean / P95 | Improved observations |
| --- | --- | --- | ---: |
| All score-qualified windows (`475`) | `0.09508 / 0.22055 m/s2` | `0.09729 / 0.22663 m/s2` | `163 / 475` |
| Final window of every replay (`144`) | `0.10189 / 0.22257 m/s2` | `0.10291 / 0.23550 m/s2` | `61 / 144` |

The weaker scales were materially worse. This is a **rejection**, not a tune
request: no proposal, covariance setting, or estimator-core change is promoted.
The compact, input/result-manifest-hashed evidence is
[`fixed_lag_bias_proposal_campaign.json`](../validation/public/fixed_lag_bias_proposal_campaign.json).
It retains the campaign matrix and stratified aggregate statistics; raw replay
CSVs and per-window details remain disposable build artifacts.

The campaign runner defaults to one worker because every trial starts a source
generator and native replay process. Parallelism is opt-in through `--jobs`
after the target workstation has demonstrated stable completion; worker count
does not change the frozen trial matrix or metrics.

## What must precede another correction candidate

This result does not prove that joint tilt/bias correction is impossible. It
does prove that a marginal 5x5 MAP proposal is insufficient. The bounded
host-only delayed-GNSS rewind/replay oracle now proves that a full
state/covariance snapshot, the existing Joseph update, attitude reset, and
canonical event order reproduce the zero-delay reference for isolated,
sequential non-overlapping, and one exact two-event overlapping/reordered P/V
schedule. In that overlap schedule, replay uses only sources delivered so far;
it does not silently accept the older pending observation. It remains a
research oracle, not a product delayed-fusion implementation, does not support
arbitrary overlapping out-of-sequence events, and does not inject this rejected
proposal.

Only after that prerequisite is independently passing may a new candidate be
specified with full 15-state lag covariance, process/preintegration covariance,
atomic nominal-state injection, reset Jacobians, persistence/hysteresis, clean
train/tune data, and a sealed holdout. Physical source/arrival timestamps,
thermal behavior, and selector state remain FCOne-private integration evidence.
