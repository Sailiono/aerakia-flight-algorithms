# Fixed-lag measurement cross-covariance prerequisite

## Purpose

The transition/process composition oracle closes only the no-measurement part
of a future fixed-lag formulation. This oracle adds a retained boundary error
state to the live ESKF error state and carries their full joint covariance
through:

- variable-rate IMU prediction with process noise;
- three-axis GNSS-position Joseph updates;
- three-axis GNSS-velocity Joseph updates; and
- the same attitude covariance-reset Jacobian used by production injection.

The validation target is the upper-left 15x15 block: it must remain numerically
identical to the production `ESKF_Handle.P`. The off-diagonal 15x15 block is
the live-state/boundary-state cross covariance required by a later smoother.

This is a validation oracle, not a product history buffer, delayed-fusion API,
or estimator correction.

## Executable oracle

[`fixed_lag_measurement_cross_covariance_oracle.c`](../validation/fixed_lag_measurement_cross_covariance_oracle.c)
constructs a 30x30 augmented covariance with the exact initial relation
`P_current = P_boundary = P_cross`. Each prediction uses the same `F` and `Q`
as `eskf_predict()`. Each P/V update uses the same measurement Jacobian,
innovation covariance, Joseph form, and attitude reset transaction as the core.
The augmented covariance is checked for finite values, symmetry, and positive
semidefiniteness after every operation.

## Results

### Reviewed double build

The deterministic campaign covered 192 chains, 2,202 IMU intervals, and 1,150
position/velocity updates:

| Metric | Maximum difference |
| --- | ---: |
| Prediction upper-left block vs production `P` | `1.0658141e-14` |
| Update upper-left block vs production `P` | `7.10542736e-15` |
| Maximum attitude-reset correction | `6.08450099e-04 rad` |
| Joint covariance health | Passed at every checkpoint |

This closes the declared double-precision P/V cross-covariance transaction.

### Host float evaluation

The production upper-left block still tracks the float ESKF (`3.81e-06`
prediction and `1.91e-06` update maximum difference), but the joint augmented
covariance loses positive semidefiniteness near its intentionally singular
initial boundary relation. 130 checkpoints failed the PSD check; the worst
Cholesky pivot was `-1.79939767e-05` (the diagnostic tolerance is `1e-5`).
The same campaign exercised a maximum reset correction of `6.08450061e-04 rad`
in the float build.

This is a genuine design constraint, not a threshold to hide. A future float
fixed-lag implementation must use a square-root/UD or otherwise stabilized
joint-covariance representation, or prove a bounded PSD-repair policy. The
naive covariance form must not be promoted to STM32H7 from this result.

## Not closed

The oracle deliberately excludes scalar barometer and magnetic/heading updates,
cross-source measurement correlation, nonlinear backward relinearization,
arbitrary event ordering, source-versus-arrival timing, sensor selection, and
target resource measurements. It provides no flight-accuracy or bias-
observability claim.

## Reproduction

Double (the reviewed default):

```bash
cmake -S . -B build/dev -DCMAKE_BUILD_TYPE=Release
cmake --build build/dev --parallel
ctest --test-dir build/dev --output-on-failure \
  -R fixed_lag_measurement_cross_covariance
```

Float is an intentionally failing evaluation until the PSD issue is resolved:

```bash
cmake -S . -B build/cross-float -DCMAKE_BUILD_TYPE=Release \
  -DAERAKIA_ESKF_CORE_PRECISION=float
cmake --build build/cross-float --parallel \
  --target aerakia_fixed_lag_measurement_cross_covariance
build/cross-float/aerakia_fixed_lag_measurement_cross_covariance
```
