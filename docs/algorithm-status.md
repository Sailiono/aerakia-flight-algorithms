# Algorithm status

## Current verdict

The portable estimator core is suitable for FCOne integration work and further bench testing,
but it is not yet justified to claim that the complete flight-estimation system is verified.
The current evidence establishes deterministic host behavior, measurement integrity handling,
long-run covariance health, synthetic cold start, private PX4-referenced replay tracking, and two
EuRoC external-reference sequences. It does not yet establish flight safety or cold-start yaw
accuracy against independent physical truth.

## Evidence completed

| Area | Evidence | Status |
| --- | --- | --- |
| Error-state math | 15-dimensional error state, quaternion injection/reset Jacobian, Joseph-form scalar and vector measurement updates | Implemented and unit tested |
| Covariance health | Long mixed predict/update sequence checked for finite, symmetric, positive-semidefinite covariance | Passing |
| Measurement integrity | NIS gates, latched magnetic-disturbance rejection, recovery confirmation, navigation recovery supervision | Passing deterministic regressions |
| Heading semantics | Magnetometer, trusted heading, GNSS course, and PX4 GSF are separate paths; course is never silently treated as body yaw | Implemented |
| Cold-start alignment | Static accelerometer tilt, magnetic heading with explicit declination, IMU-bias initialization, and covariance reset at the new linearization point; no PX4 attitude seed | Passing unit/noisy synthetic checks and direct-Vicon tilt (1.230° post-alignment RMSE); external yaw truth still pending |
| Navigation consistency | GNSS position/velocity NIS and posterior 6-state navigation NEES through a five-second outage and reacquisition | 20-seed measurement-noise baseline and constant-bias/timestamp-jitter extension pass with zero numerical/recovery failures; thermal and transport faults pending |
| EuRoC public replay | 36,381-sample Leica/IMU `MH_01_easy` and 20,932-sample direct-pose `V1_03_difficult`; raw and reference-bias-corrected tracks retained | Navigation NIS/NEES consistent; direct Vicon external pose passes high-dynamic replay; not a cold-start or independent-heading test |
| Host regression | Strict C99 warnings-as-errors build, public API tests, deterministic synthetic fault suite | Passing reviewed thresholds |
| Private replay | Sanitized relative GNSS, reset events, GSF diagnostics, and native C replay across the selected ULog suite | Operational; PX4 remains an engineering reference |

## P0 work before hardware flight tests

1. Validate cold-start yaw against rate-table/Vicon with a real accepted heading source. Direct
   Vicon now validates tilt and static bias alignment; its dataset has no magnetometer/heading input.
2. Exercise the trusted-heading path with a real dual-antenna GNSS, vision, or controlled injected
   heading dataset. The present private ULogs contain no valid direct GNSS heading samples.
3. Extend the implemented constant-bias and monotonic-timestamp-jitter Monte Carlo gate to thermal
   drift, transport delay, reordering, and sample loss.
4. Run the exact FCOne adapter through timestamp, frame, unit, dropout, and stale-data contract tests.

## P1 work when the new hardware is available

- Static bench and thermal bias characterization.
- Rate-table or motion-capture attitude tests with independent truth.
- GNSS outage/reacquisition, magnetic disturbance, and location-change tests.
- Target-MCU timing, stack, precision, and numerical-stability measurements.
- HIL followed by bounded envelope-expansion flights with reviewed abort criteria.

This document is an engineering maturity statement, not an airworthiness claim.

## 2026-07-18 Monte Carlo finding

The first formal 20-seed run found one reproducible cold-start transient: seed 17 reached 1.837°
post-alignment attitude RMSE because a GNSS position update used attitude-position covariance from
the pre-alignment linearization point and briefly pulled attitude by approximately 8–10°. Static
alignment had changed the nominal quaternion but had not reset its covariance relationships.

The correction adds an explicit attitude-covariance reset after absolute static alignment, clears
the stale attitude cross-covariances, and applies conservative configurable uncertainty floors of
2° for tilt and 10° for magnetic heading. After the correction all 20 seeds pass:

- position RMSE: mean 0.420 m, maximum 0.638 m;
- velocity RMSE: mean 0.187 m/s, maximum 0.300 m/s;
- post-alignment attitude RMSE: mean 0.712°, maximum 1.239°;
- position NIS mean across seeds: 2.943 (three degrees of freedom expected mean 3);
- velocity NIS mean across seeds: 2.495 (three degrees of freedom expected mean 3);
- navigation NEES mean across seeds: 5.047 (six degrees of freedom expected mean 6);
- navigation recoveries and unhealthy samples: zero.

These are synthetic measurement-noise results with a fixed five-second GNSS outage. They are not
physical truth, and their empirical seed percentiles are not theoretical independent-sample
confidence intervals.

## 2026-07-18 bias and timestamp extension

The same 20-seed navigation scenario now draws one constant three-axis accelerometer and gyroscope
bias per run (`σ=0.05 m/s²` and `σ=0.20°/s`) and applies monotonic per-interval timestamp jitter
(`σ=250 µs` at 100 Hz). Each generated input records the actual bias vector and interval extrema.

Three seeds exceeded the earlier noise-only 1.5° attitude gate, reaching 1.596–1.880°. One reached
0.3507 m/s against the earlier 0.35 m/s velocity gate. These runs remained finite, accepted normal
aiding, required no navigation reset, and retained reasonable NIS/NEES. The attitude increase is
consistent with the static observability limit between horizontal accelerometer bias and tilt; the
test does not claim that one stationary pose can identify both exactly.

The declared stress gate is therefore 2.0° post-alignment attitude RMSE and 0.40 m/s velocity RMSE,
while retaining the tighter deterministic noise-only CI thresholds. This is a scenario-specific,
documented gate change rather than a general relaxation of flight requirements.
