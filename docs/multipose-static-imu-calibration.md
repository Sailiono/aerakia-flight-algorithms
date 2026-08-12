# Multi-Pose Static IMU Calibration

## Purpose

This optional preflight procedure estimates a **constant** accelerometer bias
and a constant gyroscope bias from several stationary pose means, before the
first ESKF IMU sample. It addresses a specific cold-start limit: one gravity
direction cannot distinguish a horizontal accelerometer bias from a small
roll/pitch error.

It is not an online estimator feature and does not add states to the 16-state
nominal / 15-error-state ESKF. The ESKF remains the primary navigation
estimator; robust Mahony remains an independent attitude cross-monitor and
bounded degraded-attitude fallback.

## Mathematics

For stationary means `a_i = b_a + g_i`, each corrected vector has the same
magnitude. Pairwise subtraction removes unknown gravity directions:

```text
2 (a_i - a_j)^T b_a = |a_i|^2 - |a_j|^2
```

The C99 calculator solves the overdetermined normal equations from all pose
pairs. Gyroscope bias is the mean stationary angular rate. It then fails
closed unless all of the following hold:

- at least six pose means by default, with a hard maximum of 16;
- finite mean values and nonzero sample count per pose;
- each pose gyro-mean norm is within the stationary threshold;
- the gravity-sphere normal matrix is sufficiently conditioned;
- corrected gravity directions span three dimensions;
- every pose and the aggregate radial gravity residual meet their limits;
- gyro means have acceptable scatter about the fitted common bias; and
- no single pose can move the fitted accelerometer-bias seed beyond the
  leave-one-out influence limit.

No pose is silently removed. A gate-detected bad pose rejects the complete
result, but these checks cannot prove that every possible corrupted pose is
detected.
The procedure estimates neither scale, non-orthogonality, temperature drift,
vibration rectification, lever-arm effects, motor effects, nor magnetic error.

## Integration Boundary

The public calculator takes only already-calibrated, final-frame FRD pose
means. The private FCOne adapter owns raw driver collection, axis transforms,
per-IMU identity, temperature, vibration/clip checks, persistence, and the
pre-arm policy. It must produce a separate artifact for every physical IMU;
three redundant IMUs must never share one bias artifact.

After an accepted result is passed through
`aerakia_eskf_apply_static_imu_calibration`, the adapter:

1. seeds the ESKF exactly once before any streamed IMU sample;
2. subtracts the accepted accelerometer seed before ordinary one-pose tilt
   alignment, so raw static bias is not reinterpreted as tilt;
3. preserves the seed instead of overwriting it with a later one-pose mean;
4. retains conservative default seed covariance until FCOne characterization
   justifies an evidence-backed private profile.

An unaccepted result, a second application, or an application after streaming
is rejected without changing ESKF state. The library does not authenticate a
mutable calibration record: the private product must bind its own calibration
ID, IMU serial/instance, frame revision, temperature range, timestamp, and
integrity hash.

## FCOne Collection Protocol

This is the minimum physical protocol to implement in the private FCOne v2
pre-arm calibration supervisor. It is intentionally not enabled by default in
the public library.

1. Select one physical IMU after private health/voting checks. Disable flight
   actuation and require a known pre-arm state.
2. At six orientations, approximately `+X/-X/+Y/-Y/+Z/-Z` in that IMU's final
   calibrated FRD frame, wait for mounting motion to settle.
3. For each orientation record a fixed-duration raw window, its sample count,
   mean, variance, peak gyro, acceleration-norm scatter, temperature, clip
   counters, selector source/generation, and board/frame revision.
4. Reject a window if private stationarity, clipping, vibration, timing, or
   temperature-stability policy fails. Do not turn a moving/accelerating mean
   into a calibration pose merely because its average is finite.
5. Transform accepted means to the final FRD estimator frame and call the C99
   calculator. Persist the input hash and result diagnostics together.
6. Apply the output only if it belongs to the same IMU, mapping, and approved
   temperature range. Otherwise run the normal one-pose startup with its
   explicitly broader uncertainty.

The public `sample_count` field is an audit input and a nonzero guard. It is
not a variance model or a weighted least-squares claim. Window variance,
settling time, clipping, vibration, and temperature policy remain private
because they depend on the selected FCOne IMU, driver, mounting, and vehicle.

## Host Usage

The host executable accepts CSV columns:

```text
acc_x_m_s2,acc_y_m_s2,acc_z_m_s2,gyro_x_rad_s,gyro_y_rad_s,gyro_z_rad_s,sample_count
```

```bash
cmake -S . -B build/host -DAERAKIA_BUILD_VALIDATION=ON
cmake --build build/host --parallel
build/host/aerakia_static_imu_calibration_cli pose-means.csv calibration.json
```

`calibration.json` is an inspectable result, not a signed production artifact.
The normal replay runner supports `--multipose-bias-seed` solely to make host
A/B tests exercise the same ESKF application API. Production-like campaign
evidence uses `--multipose-pose-means`, which re-runs the C calculator from a
strict, schema-checked CSV. The replay's initial static window is a simulation
stand-in; FCOne must supply causal IMU-window stationarity instead of relying
on a trajectory-derived hint.

## Evidence Status

The initial synthetic development campaign uses six noisy stationary means,
the public C calculator, and paired cold-start replay. It is deliberately
separate from historical exposed G0 holdouts. Its retained result may show
that a known synthetic constant bias is recoverable under the declared
procedure, but it cannot establish physical calibration accuracy, FCOne
temperature performance, cross-IMU transferability, or flight readiness.

Before any FCOne promotion, require:

- a new sealed synthetic holdout that was not used for threshold selection;
- near-threshold geometry/noise/outlier false-accept and false-reject studies;
- per-IMU static multi-orientation hardware data across a declared temperature
  range;
- comparison with the existing one-pose startup and a no-calibration control;
- target timing/RAM evidence and private pre-arm/failsafe review.
