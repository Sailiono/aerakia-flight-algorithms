# Validation methodology

The PC platform exists to make algorithm claims reproducible, reviewable, and progressively harder to fake. A graph from one hand-picked log is not sufficient evidence.

## Evidence ladder

| Level | Evidence | Purpose |
| --- | --- | --- |
| 1 | Unit and numerical tests | API, sign, unit, covariance, and edge-case correctness |
| 2 | Deterministic synthetic scenarios | Controlled bias, noise, timing, dropout, and disturbance experiments |
| 3 | Public datasets with independent ground truth | Reproducible external comparison |
| 4 | Rate table or motion capture | Known physical motion and sensor behavior |
| 5 | Hardware-in-the-loop and flight logs | Integration timing, transport, and real vehicle behavior |

Only Levels 4–5 support strong hardware/flight reliability claims. The public repository establishes Levels 1–2 and provides a private-ULog replay path for Level-5 integration evidence without publishing the hardware integration or raw flight data.

## Comparison tracks

Do not mix fundamentally different estimator scopes in a single leaderboard.

### Attitude track

- uncorrected gyro integration;
- standard Mahony;
- Madgwick (planned baseline);
- Aerakia robust Mahony;
- attitude-only MEKF (future).

Metrics: roll/pitch/yaw RMSE, maximum error, 95th percentile error, convergence time, yaw drift, disturbance recovery time, rejected-sample ratio, CPU time, and state memory.

The primary full-attitude error is the quaternion geodesic angle
`2 acos(abs(q_est · q_ref))`. Primary tilt error is the angle between estimated and reference
gravity directions in body axes. Euler-axis errors remain useful diagnostics but are never combined
into the primary score because roll/yaw representations jump near ±90° pitch.

### Navigation track

- Aerakia 15-state ESKF;
- a separately implemented reference ESKF/MEKF using identical aiding inputs;
- larger production estimators only in a clearly separated system-level comparison.

Metrics: attitude/velocity/position RMSE, bias error, innovation acceptance, NIS/NEES consistency where ground truth is available, covariance health, recovery after aiding loss, CPU time, and memory.

## Fair-comparison rules

1. Every algorithm receives the same calibrated sample values and physical timestamps.
2. Coordinate frames and initial conditions are recorded in the dataset contract.
3. Parameters are frozen before scoring; no per-run tuning on the evaluation set.
4. Warm-up and excluded intervals are declared.
5. Both accuracy and computational cost are reported.
6. Failed or numerically unhealthy runs remain in the report.
7. Synthetic results are labeled synthetic and never presented as flight proof.

The internal numerical gate validates all 15 columns of the discrete transition against finite
differences. All 225 process-noise entries are checked by a complete structure oracle, full-Q
finiteness/symmetry/positive-semidefiniteness, and an independent numerical integration of the
declared reduced continuous model. That reduced model includes direct IMU/bias white noise and
velocity-to-position integration; it does not yet include every within-step higher-order coupling
from attitude, specific force, angular rate, and bias random walk. Multi-rate NIS/NEES evidence
supports the approximation at 100--1000 Hz, but a full coupled Van Loan or equivalent oracle remains
open.

Trusted physical heading uses the complete right-error Jacobian of raw body-X `atan2` heading and
is checked per axis over 10,000 attitudes. Magnetometer fusion is deliberately a separate,
tilt-conditioned NED-yaw-only pseudo correction so magnetic inclination/model error cannot request
roll/pitch correction. Its tuning variance and pseudo-NIS are not represented as general
physical-heading consistency at arbitrary tilt. Near-singular threshold cases fail closed.

Horizontal accelerometer-bias generalization uses the frozen
[`bias_observability_protocol_v1.json`](../validation/bias_observability_protocol_v1.json) and
`run_bias_observability_cross_validation.py`. It contains five non-multisine VTOL profiles, nine
exact horizontal bias vectors, disjoint train/tune/holdout seeds, interval-start ZOH truth, and
per-trajectory/vector release gates. Smoke mode may report rather than enforce capability gates,
but its JSON and Markdown status must still say `capability_failed` whenever any metric fails.

### Tilt and accelerometer-bias consistency

The native runner exports the complete 5-by-5 marginal covariance for
`[δθx, δθy, δba_x, δba_y, δba_z]`. The attitude components are the first two axes of the ESKF
right-error tangent vector, consistent with nominal injection `q_true = q_estimate ⊗ Exp(δθ)`;
the analyzer obtains them from `Log(q_estimate^-1 ⊗ q_truth)`. Bias errors use
`bias_truth - bias_estimate`, matching the error-state injection sign. The exported tilt 2-by-2,
tilt/bias 2-by-3, and accelerometer-bias 3-by-3 blocks retain their cross covariance and are
assembled without conditioning.

The resulting 5D NEES is reported only when quaternion truth and every covariance block are
present and positive definite. Its expected mean is 5; per-sample 95% coverage is diagnostic,
not an independent-trial confidence claim, because adjacent replay rows are correlated. `δθz` is
deliberately excluded: without an accepted trusted-heading source yaw is unobservable, and adding
it would turn a tilt/bias consistency check into a test dominated by arbitrary yaw uncertainty.
This marginal score is valid in the ESKF local-error regime and does not claim full 6D
attitude-plus-bias consistency.

Bias reports also include signed per-axis errors, terminal-five-second statistics, and the first
interval that remains below the declared norm threshold continuously for five seconds. Runs that
never satisfy the dwell requirement are explicitly right-censored at their observed duration;
their final sample is not relabeled as convergence.

## Current deterministic scenarios

- `clean_motion`: combined roll, pitch, and yaw without injected magnetic faults;
- `mag_spike`: short magnetic magnitude spikes;
- `mag_bias`: persistent magnetic bias;
- `yaw_jump`: discontinuous truth case for wrap/continuity testing;
- `cold_start_tilted`: noisy static data at non-zero roll, pitch, and yaw; the ESKF starts from
  identity and CI scores only samples after independently reported tilt/heading alignment.
- `navigation_outage`: stationary cold start, horizontal maneuvers, a five-second GNSS outage, and
  reacquisition with independent synthetic position/velocity truth.
- `trusted_heading_recovery`: magnetometer-disabled cold start using an explicit 10 Hz body-heading
  source, two 90° outliers, a four-second dropout, rejection, coast, and recovery.
- `bias_convergence`: one-pose static initialization followed by multi-axis attitude/specific-force
  excitation and continuous GNSS aiding; reports accelerometer and gyroscope truth error, reduction,
  settling time, navigation accuracy, and consistency separately.

The native `aerakia_validation_runner` replays the public C code. `run_suite.py` generates reports and `check_thresholds.py` turns reviewed error limits into CI gates.

GNSS position and velocity updates report their three-degree-of-freedom normalized innovation
squared (NIS). When the reference is declared synthetic or external truth, the runner additionally
reports posterior six-state `[velocity, position]` normalized estimation error squared (NEES). Reports include
single-sample chi-square coverage as a diagnostic; consecutive replay samples are correlated, so
coverage is not treated as an independent-sample hypothesis test.

Position and velocity are distinct measurement sources in the replay and public API. A receiver
that publishes only position must use the timestamped position observation; validation must not
differentiate positions and relabel the result as measured receiver velocity. The legacy paired API
remains available when both measurements are physically present. Duplicate/freshness accounting
and recovery timestamps are source-specific.

Recorded aiding gaps are scored separately from nominal aided operation. Reports identify the
longest gap, missing nominal epochs, error immediately before the gap, peak position/velocity drift,
the first resumed posterior error, resume NIS, and sustained recovery time. A whole-run RMSE that
mixes a long outage with nominal aiding is retained but is never presented as receiver accuracy.

`run_monte_carlo.py` formalizes the reviewed multi-seed navigation gate. It retains every seed
result, reports failure seeds, aggregates empirical P05/P95 ranges, and at 1,000 or more trials
adds deterministic 10,000-resample 99% bootstrap bounds plus the exact 95% zero-failure probability
upper bound. The current calibrated-input profile randomizes measurement noise, includes a fixed
five-second GNSS outage, draws constant three-axis IMU biases inside a declared per-axis three-sigma
residual-calibration envelope, and applies monotonic interval jitter. The first unbounded-prior
confirmation and its one 4-sigma attitude failure remain public evidence; the later bounded range
uses new seeds. Temperature-varying bias, delay compensation, vibration, and physical calibration
remain separate hardware/profile gates, while deterministic transport reordering/loss/malformed
values are covered by the input-integrity campaign.

The bias scenario deliberately distinguishes observability phases. A stationary mean directly
observes gyro bias, but one gravity direction cannot uniquely separate tilt from horizontal
accelerometer bias. Static alignment therefore keeps accelerometer-bias uncertainty broad; only the
subsequent multi-axis motion/GNSS interval is scored as online accelerometer-bias convergence.

The staged external intake and the limits of each source are documented in
[public datasets](public-datasets.md).

The EuRoC intake also has two deliberately separate tracks. The raw track tests startup and online
bias handling. The reference-bias-corrected track subtracts EuRoC's batch-estimated IMU biases and
therefore tests propagation/update math only; it must never be presented as online bias-estimation
performance. Both use the same trusted initial reference attitude because `MH_01_easy` starts in
motion and provides neither magnetometer nor an absolute heading sensor.

`V1_03_difficult` adds a third, tilt-only cold-start track. Its initial stationary flag is asserted
from external Vicon motion, and the validation runner records a gyro threshold chosen to admit the
measured uncalibrated bias. Tilt completion and post-tilt gravity-direction RMSE are scored even when
heading alignment cannot complete. Full-attitude cold-start scoring still requires an accepted
heading source.

Blackbird `NYC Subway Winter` adds a 270.1 s independent motion-capture track with recorded
100 Hz IMU, aggressive angular motion, and an externally verified five-second static prefix. The
converter keeps one absolute time base across IMU and truth, applies the published body/IMU
extrinsic, and fails closed unless angular-rate correlation and gravity/frame residual checks pass.
Its optional synthetic GNSS remains a navigation-math diagnostic, not receiver evidence.

UrbanNav `Medium-Urban-1` adds 785.5 s of recorded 400 Hz ground-vehicle IMU, 655 valid F9P
position epochs, independent SPAN-CPT postprocessed navigation truth, and a real 131 s receiver
position gap. It tests independent position-only aiding and reacquisition. It does not test receiver
velocity, physical body heading, aircraft dynamics, or fully independent truth because SPAN is a
postprocessed GNSS/INS system.

For independent-truth tracks, `analyze_results.py` also reports an offline Mahony fallback envelope:
for declared entry-error gates it computes the first threshold crossing and worst/P95 truth error
over 0.5, 1, 2, and 5 second windows. This metric is used to challenge supervisor policy, not as an
online health signal—the flight supervisor cannot observe ground-truth error. Blackbird motivated a
continuity check on fallback entry and a finite Mahony-only time budget.

## Private ULog track

`convert_ulog_to_replay.py` extracts calibrated IMU, sparse magnetometer updates, the configured
magnetic declination, PX4 attitude reset metadata, relative GPS NED aiding, direct dual-antenna GNSS
heading when available, GNSS course, PX4 GSF yaw, barometer height, and PX4 local-position
references. It intentionally omits absolute latitude/longitude, hardware IDs, parameter dumps, and
private topics.

`run_ulog_suite.py` consumes a private manifest and runs conversion, the native C runner, metrics, reset diagnostics, and a cross-scenario summary. Static logs must be explicitly marked `assume_stationary`; this assertion is never inferred from their filename or motion.

Yaw reports publish raw PX4 agreement, a logged-reset-compensated diagnostic, and per-reset-segment drift separately. Removing estimator resets does not create ground truth. PX4 GSF is compared only on fresh, low-variance samples while horizontal speed is sufficient. GNSS course-over-ground is never fused as vehicle yaw; only an explicitly valid dual-antenna heading enters the trusted-heading API.

PX4 `vehicle_attitude` and `vehicle_local_position` are engineering references, not independent truth. Large PX4 quaternion resets are reported separately. Two observed failure classes must remain visible in reports:

- persistent magnetic disagreement can cause high yaw error even when tilt remains accurate;
- position accuracy against PX4 depends on origin alignment, aiding availability, and PX4's own estimator configuration.

The correct next comparison is an independent heading source and independent position truth—not tuning metrics against PX4 resets.
