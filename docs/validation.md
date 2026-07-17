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

Only Levels 4–5 support strong hardware/flight reliability claims. The public repository currently establishes Levels 1–2 and provides the format needed to add higher-level evidence without publishing the hardware integration.

## Comparison tracks

Do not mix fundamentally different estimator scopes in a single leaderboard.

### Attitude track

- uncorrected gyro integration;
- standard Mahony;
- Madgwick (planned baseline);
- Aerakia robust Mahony;
- attitude-only MEKF (future).

Metrics: roll/pitch/yaw RMSE, maximum error, 95th percentile error, convergence time, yaw drift, disturbance recovery time, rejected-sample ratio, CPU time, and state memory.

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
- `yaw_jump`: discontinuous truth case for wrap/continuity testing.

The native `aerakia_validation_runner` replays the public C code. `run_suite.py` generates reports and `check_thresholds.py` turns reviewed error limits into CI gates.
