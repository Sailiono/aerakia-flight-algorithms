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

The next campaign evaluates the proposal on causal v2 replays with signed
multi-axis residual-bias vectors. Truth is read only by the separate evaluator
after the solver returns, so the correction path remains auditable.
