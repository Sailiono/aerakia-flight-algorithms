# Process-noise discretization scope

## Decision

The production ESKF keeps its reduced high-rate process-noise mapping.  A full
coupled `Qd` is **not required for the declared 100--1000 Hz, bounded-motion
host profile**.  It remains a mathematical hardening option, not a claim that
the current mapping is exact for every update interval, sensor profile, or
vehicle motion.

This decision applies only to the process-noise increment injected during one
IMU interval.  The production covariance transition already propagates prior
attitude and bias uncertainty through its discrete `F`; the approximation is
the omission of the new, within-interval noise couplings described below.

## Models compared

The 15-error-state order is

```text
[dtheta, dv, dp, d_accel_bias, d_gyro_bias].
```

For a frozen IMU interval, the independent test oracle constructs the complete
first-order continuous model

```text
d(dtheta)/dt = -[omega]x dtheta - d_gyro_bias + gyro_white_noise
d(dv)/dt     = -R[a]x dtheta - R d_accel_bias + accel_white_noise
d(dp)/dt     = dv
d(d_accel_bias)/dt = accel_bias_random_walk
d(d_gyro_bias)/dt  = gyro_bias_random_walk
```

and integrates

```text
dQ/dt = A Q + Q A^T + W
Qd = integral_0^dt Phi(t) W Phi(t)^T dt.
```

`W` contains isotropic gyro/accelerometer white noise and the two independent
bias random walks.  This is the full frozen-coefficient first-order `Qd`
oracle, evaluated independently with RK4.  It intentionally is not an exact
nonlinear trajectory integral: `R`, specific force, angular rate, and noise
statistics are held constant over the sampled IMU interval, matching the
usual local ESKF linearization question.

The production mapping retains the leading terms that are exact for direct
IMU/bias noise and velocity-to-position integration:

```text
Q_theta,theta = sigma_gyr^2 dt
Q_v,v         = sigma_acc^2 dt
Q_v,p         = Q_p,v = sigma_acc^2 dt^2 / 2
Q_p,p         = sigma_acc^2 dt^3 / 3
Q_ba,ba       = sigma_ba^2 dt
Q_bg,bg       = sigma_bg^2 dt.
```

It omits higher-order, within-step terms: gyro-noise propagation through
specific force into velocity/position, gyro-bias random-walk propagation into
attitude/velocity/position, accelerometer-bias random-walk propagation into
velocity/position, and rotation-dependent cross-axis terms.  Those terms are
real; the test explicitly requires the oracle to observe a nonzero difference.

## Executable evidence

`tests/test_eskf_models.c` runs a fixed-seed 1,000-case campaign.  Its
declared synthetic stress envelope is:

| Input | Envelope |
| --- | --- |
| IMU period | 1--10 ms (100--1000 Hz) |
| Body specific-force component | -25 to +25 m/s^2 |
| Body angular-rate component | -6 to +6 rad/s |
| Attitude | yaw/roll arbitrary, pitch -1.35 to +1.35 rad |
| Noise densities | `sigma_acc=0.37`, `sigma_gyr=0.018`, `sigma_acc_bias=0.004`, `sigma_gyr_bias=0.0007` in the public API's documented SI units |

The campaign verifies all of the following:

1. The independently constructed `A` agrees with the production discrete
   transition derivative.
2. The RK4 oracle is finite, symmetric, PSD, and stable when its integration
   step is halved.
3. The omitted couplings are nonzero, so the test cannot accidentally label
   the reduced model as full `Qd`.
4. The Frobenius relative difference
   `||Q_full - Q_reduced||_F / ||Q_full||_F` remains below 1%.

The current deterministic result is:

```text
maximum relative defect:                  5.01673511e-04  (0.0502%)
RK4 20-step versus 40-step maximum error: 1.35308431e-16
transition-derivative maximum error:       3.51718654e-06
```

Together with the separately gated 100/200/400/1000 Hz NIS/NEES and accuracy
invariance suite, this supports the reduced mapping for the stated high-rate
host envelope.  It does not make the approximation a universal `Qd` claim.

## Re-open conditions

Revisit a full coupled production `Qd` (for example, Van Loan or an equivalent
closed-form/numerical method) before claiming any of these conditions:

- IMU periods above 10 ms or uncharacterized scheduler gaps;
- sustained specific force or angular rate outside the tested envelope;
- colored, correlated, temperature-dependent, or target-characterized sensor
  noise that does not match the white-noise/random-walk model;
- target single-precision behavior or covariance inconsistency on STM32H7;
- a repeatable NIS/NEES failure attributable to this per-step approximation.

FCOne integration must still measure real timestamp jitter, sensor noise,
target precision, and covariance consistency.  This host result does not
replace that target evidence.
