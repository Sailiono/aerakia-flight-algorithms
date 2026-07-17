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

## Current deterministic scenarios

- `clean_motion`: combined roll, pitch, and yaw without injected magnetic faults;
- `mag_spike`: short magnetic magnitude spikes;
- `mag_bias`: persistent magnetic bias;
- `yaw_jump`: discontinuous truth case for wrap/continuity testing;
- `cold_start_tilted`: noisy static data at non-zero roll, pitch, and yaw; the ESKF starts from
  identity and CI scores only samples after independently reported tilt/heading alignment.
- `navigation_outage`: stationary cold start, horizontal maneuvers, a five-second GNSS outage, and
  reacquisition with independent synthetic position/velocity truth.

The native `aerakia_validation_runner` replays the public C code. `run_suite.py` generates reports and `check_thresholds.py` turns reviewed error limits into CI gates.

GNSS position and velocity updates report their three-degree-of-freedom normalized innovation
squared (NIS). When the reference is declared synthetic or external truth, the runner additionally
reports posterior six-state `[velocity, position]` normalized estimation error squared (NEES). Reports include
single-sample chi-square coverage as a diagnostic; consecutive replay samples are correlated, so
coverage is not treated as an independent-sample hypothesis test.

`run_monte_carlo.py` formalizes the first reviewed multi-seed navigation gate. It retains every
seed result, reports failure seeds, and aggregates empirical P05/P95 ranges for accuracy and
consistency metrics. The current phase randomizes measurement noise and includes the fixed
five-second GNSS outage. It does not yet randomize constant/thermal bias, timestamps, transport
delay, reordering, or missing IMU samples; those remain explicit P0 extensions.

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
