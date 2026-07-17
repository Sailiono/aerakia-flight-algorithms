# ESKF mathematical and numerical audit

Status: active  
Audit baseline: `74db219da24d60c29bc0b7ac01fa5b685c0f3cb9`  
First numerical correction set: 2026-07-17

## Scope

This audit checks the portable 15-dimensional error-state implementation before
it is connected to a private flight stack. Passing these checks means that no
blocking defect is known within the tested model and scenarios. It does not mean
that the estimator is certified or ready to command a vehicle.

## Findings closed in this correction set

### N-001 — continuous IMU noise was discretized as sampled noise

`ESKF_Config` defines accelerometer and gyroscope noise as continuous-time noise
densities, in units per square-root hertz. The prediction code previously used
`sigma² * dt²` for attitude and velocity covariance. That expression is for a
per-sample standard deviation and underestimates a noise-density model by a
factor of `dt` (100 times at 100 Hz).

The implementation now uses:

- attitude: `sigma_gyr² * dt`;
- velocity: `sigma_acc² * dt`;
- velocity-position cross covariance: `sigma_acc² * dt² / 2`;
- position: `sigma_acc² * dt³ / 3`;
- bias random walks: `sigma_bias² * dt`.

A zero-prior unit test checks each term directly.

### N-002 — covariance updates used the simplified form

Scalar and three-dimensional updates previously used `(I-KH)P`, followed by
manual symmetrization. Both now use the Joseph form:

`P = (I-KH) P (I-KH)^T + K R K^T`

Repeated tight-noise updates are tested for finite, symmetric and non-negative
diagonal covariance. This is a numerical integrity improvement; it does not
replace a later positive-semidefinite or square-root-filter study.

### N-003 — heading correction geometry was implicit

The heading update is deliberately a NED-yaw pseudo-observation, not a full
Euler-yaw measurement Jacobian. With a right/body-frame attitude error, the
axis for a pure navigation-frame yaw correction is `R^T e_z`, the third row of
the body-to-NED rotation matrix.

This choice is important for magnetic-disturbance containment: replacing it
with the full derivative of the tilt-compensated magnetic heading allowed the
magnetic dip and disturbances to inject roll/pitch corrections. That candidate
failed the `mag_spike` regression with double-digit tilt error and was rejected.

The model is now isolated in `src/eskf_models.c`. Tests verify that:

- the correction axis has unit norm;
- a right-error correction along it equals a pure left NED-yaw rotation;
- trusted and magnetic headings change one-for-one along that axis;
- singular vertical-heading cases are rejected.

### N-004 — prediction transition omitted direct position coupling

The nominal position update contains `0.5 * a * dt²`, but the covariance
transition previously included only the position-to-velocity block. The
right-error transition now also includes the direct position-to-attitude and
position-to-accelerometer-bias blocks:

- `F_p_theta = -0.5 R [a]× dt²`;
- `F_p_ab = -0.5 R dt²`.

The transition construction is isolated in `eskf_model_transition`. A full
15-column finite-difference test perturbs the nominal state using the same
right-error convention, propagates both states, and compares the resulting
error against the analytical transition. The attitude and gyro-bias blocks
remain a stated first-order discretization.

## Evidence from this correction set

Host checks:

- strict C99 compile with `-Wall -Wextra -Werror -pedantic`;
- ESKF core, heading model, magnetic gate and public API tests passed;
- AddressSanitizer and UndefinedBehaviorSanitizer passed with leak detection
  disabled because the execution environment runs under `ptrace`;
- deterministic synthetic thresholds passed for `clean_motion`, `mag_spike`
  and `mag_bias`.

Synthetic ESKF results, seed 7 at 100 Hz:

| Scenario | Full attitude RMSE | Tilt RMSE | Yaw RMSE |
| --- | ---: | ---: | ---: |
| `clean_motion` | 0.174° | 0.069° | 0.286° |
| `mag_spike` | 0.197° | 0.062° | 0.330° |
| `mag_bias` | 0.235° | 0.008° | 0.408° |

Selected private ULog replay results are intentionally not committed with raw
data. The current run covered seven scenarios with explicit input/output row
count checks. Fixed-wing and stationary cases show 0.079–0.609° tilt RMSE and
1.24–2.04° raw yaw RMSE against the PX4 estimate. Two multirotor
magnetic/heading-stress cases show approximately 18° and 49° raw yaw RMSE,
alongside large PX4 reference resets and magnetic rejection.

The ULogs also contain GNSS-velocity-aided GSF yaw. In the two multirotor cases,
Aerakia's globally aligned trajectory is closer to converged GSF yaw than PX4
attitude yaw is, although Aerakia still differs by approximately 10–14° RMSE.
The de-reset PX4 trajectory is explicitly a continuity diagnostic, not physical
truth; estimator resets may be correcting accumulated heading error. See
`docs/ulog-yaw-diagnostics.md`.

## Open audit items

- Monte Carlo NIS/NEES consistency using independent simulated truth;
- higher-order state transition/process-noise coupling and timing-jitter study;
- delayed, out-of-sequence and dropped measurement handling;
- cold-start attitude/bias alignment over varied initial poses;
- independent heading truth and a GNSS-velocity yaw fallback;
- target precision, execution-time, stack and IAR compatibility evidence.

Until these are closed, the correct status is **algorithm audit in progress**.
