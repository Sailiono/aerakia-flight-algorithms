# Estimator supervision contract

This document fixes the intended roles of the public estimators before FCOne integration. It is a
hardware-independent contract; the mode state machine and control-path handover remain private
FCOne application code.

## Roles

| Estimator | Product role | Valid outputs | Explicit limitations |
| --- | --- | --- | --- |
| 15-error-state ESKF | Primary navigation estimator | attitude, velocity, local position, IMU biases, covariance, aiding and recovery status | Requires completed initialization and sufficient healthy aiding for the requested navigation mode |
| Robust Mahony | Independent short-duration degraded-attitude fallback and cross-monitor | attitude and attitude-filter health | No position, velocity, navigation covariance, GNSS/barometer fusion, full navigation integrity, or indefinite unaided-yaw guarantee |
| Standard Mahony | Reproducible comparison baseline | attitude metrics in validation | Not the planned deployed fallback |

The two deployed candidates consume the same calibrated physical IMU publication, but maintain
separate contexts and state. A defect or reset in one estimator must not overwrite the other.

## Recommended application modes

1. `INITIALIZING`: no control-qualified estimate until the required alignment contract is met.
2. `PRIMARY_ESKF`: use ESKF outputs with per-field validity and freshness.
3. `DEGRADED_ATTITUDE_MAHONY`: expose Mahony attitude only; mark ESKF position, velocity, and
   navigation-dependent outputs invalid or stale. The controller must enter an explicitly reviewed
   limited mode rather than assuming full navigation remains available. Entry is allowed only when
   Mahony is continuous with the last qualified control attitude, and the mode has a finite time
   budget.
4. `ESTIMATE_INVALID`: neither estimator provides a qualified attitude; execute the FCOne safety
   response appropriate to vehicle state.

The supervisor may also report a warning state while retaining `PRIMARY_ESKF`, for example when one
aiding source is rejected but the ESKF remains observable and healthy. A single innovation reject,
magnetometer reject, or Mahony/ESKF disagreement must not cause an immediate source switch.

Hard invalidity and soft degradation are different. A non-finite ESKF state, invalid quaternion,
or invalid covariance must immediately stop ESKF output qualification and select a healthy Mahony
attitude or `ESTIMATE_INVALID`; hysteresis must never keep publishing numerically invalid state.
Loss of an aiding source, temporary lack of navigation observability, or isolated innovation reject
may use confirmation counts and dwell time while the underlying state remains finite and bounded.
The public estimate now separates these cases directly: `healthy` is numerical integrity, while
`horizontal_navigation_valid` expires after the configured interval since the last accepted
horizontal constraint. A supervisor must never publish position or velocity merely because the
former remains true after the latter becomes false.

## Evidence used for a transition

ESKF qualification should combine, at minimum:

- initialization/alignment completion;
- finite state and covariance with reviewed diagonal bounds;
- monotonic, fresh IMU processing and output age;
- sustained aiding acceptance/rejection state appropriate to the active flight mode;
- navigation recovery frequency and repeated-reset limits;
- explicit position, velocity, attitude, and heading observability/validity flags.

Mahony qualification should combine:

- finite quaternion and normalization health;
- fresh, monotonic IMU processing;
- initialization state;
- accelerometer and magnetic trust/gate state;
- output age and sustained disagreement diagnostics.

Mahony/ESKF attitude disagreement is a cross-check, not proof that either particular estimator is
wrong. The supervisor needs aiding health and vehicle context before assigning fault responsibility.

## Handover constraints

- Do not feed Mahony attitude into the ESKF as an unannounced correction.
- Do not switch the controller's attitude reference without an explicit continuity policy.
- Record transition time, reason, source health snapshot, attitude delta, and output-validity mask.
- Compare a candidate Mahony fallback against the last qualified output before selecting it; a
  parallel filter that has already drifted outside the continuity gate is not a valid fallback.
- Give Mahony-only degradation a finite, vehicle-state-specific time budget. Expiry must enter an
  explicit invalid/failsafe state rather than silently extending attitude validity.
- Apply hysteresis and minimum healthy dwell time before returning to the ESKF.
- Never continue publishing old ESKF position or velocity as valid during Mahony-only operation.
- Validate every transition first in host replay and FCOne shadow mode; physical takeover requires
  the hardware test gates in the roadmap.

## Current boundary

This repository validates estimator behavior and the public health evidence needed by a supervisor.
`validation/estimator_supervisor_contract.c` is an executable host oracle for initialization,
hard-invalid handling, soft-degradation hysteresis, attitude-only output invalidation, handover
continuity on both fallback entry and recovery, a finite Mahony-only time budget, recovery dwell,
and transition logging. Its ten-degree entry gate and one-second degraded budget are conservative
test-oracle values derived from the retained Blackbird envelope, not product
constant. It is not a flight-qualified automatic failover implementation. Target timing, actuator
interaction, and in-flight abort behavior require the private FCOne supervisor plus FCOne v2
hardware and HIL/flight evidence.
