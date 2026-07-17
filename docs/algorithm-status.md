# Algorithm status

## Current verdict

The portable estimator core is suitable for FCOne integration work and further bench testing,
but it is not yet justified to claim that the complete flight-estimation system is verified.
The current evidence establishes deterministic host behavior, measurement integrity handling,
long-run covariance health, synthetic cold start, private PX4-referenced replay tracking, and two
EuRoC external-reference sequences. It does not yet establish flight safety or cold-start accuracy
against independent physical truth.

## Evidence completed

| Area | Evidence | Status |
| --- | --- | --- |
| Error-state math | 15-dimensional error state, quaternion injection/reset Jacobian, Joseph-form scalar and vector measurement updates | Implemented and unit tested |
| Covariance health | Long mixed predict/update sequence checked for finite, symmetric, positive-semidefinite covariance | Passing |
| Measurement integrity | NIS gates, latched magnetic-disturbance rejection, recovery confirmation, navigation recovery supervision | Passing deterministic regressions |
| Heading semantics | Magnetometer, trusted heading, GNSS course, and PX4 GSF are separate paths; course is never silently treated as body yaw | Implemented |
| Cold-start alignment | Static accelerometer tilt, magnetic heading with explicit declination, then IMU-bias initialization; no PX4 attitude seed | Passing unit, noisy synthetic, and private stationary-replay checks |
| Navigation consistency | GNSS position/velocity NIS and posterior 6-state navigation NEES through a five-second outage and reacquisition | Passing deterministic bounds; Monte Carlo expansion pending |
| EuRoC public replay | 36,381-sample Leica/IMU `MH_01_easy` and 20,932-sample direct-pose `V1_03_difficult`; raw and reference-bias-corrected tracks retained | Navigation NIS/NEES consistent; direct Vicon external pose passes high-dynamic replay; not a cold-start or independent-heading test |
| Host regression | Strict C99 warnings-as-errors build, public API tests, deterministic synthetic fault suite | Passing reviewed thresholds |
| Private replay | Sanitized relative GNSS, reset events, GSF diagnostics, and native C replay across the selected ULog suite | Operational; PX4 remains an engineering reference |

## P0 work before hardware flight tests

1. Validate cold-start roll/pitch/yaw against rate-table or stationary Vicon truth. Current EuRoC
   sequences start in motion and therefore use a trusted first-attitude seed.
2. Exercise the trusted-heading path with a real dual-antenna GNSS, vision, or controlled injected
   heading dataset. The present private ULogs contain no valid direct GNSS heading samples.
3. Expand the implemented per-run NIS and navigation-substate NEES diagnostics into Monte Carlo
   confidence tests with bias, timing jitter, aiding loss, and recovery cases.
4. Run the exact FCOne adapter through timestamp, frame, unit, dropout, and stale-data contract tests.

## P1 work when the new hardware is available

- Static bench and thermal bias characterization.
- Rate-table or motion-capture attitude tests with independent truth.
- GNSS outage/reacquisition, magnetic disturbance, and location-change tests.
- Target-MCU timing, stack, precision, and numerical-stability measurements.
- HIL followed by bounded envelope-expansion flights with reviewed abort criteria.

This document is an engineering maturity statement, not an airworthiness claim.
