# Estimator supervision contract

This document fixes the intended roles of the public estimators before FCOne integration. It is a
hardware-independent contract; the mode state machine and control-path handover remain private
FCOne application code.

## Roles

| Estimator | Product role | Valid outputs | Explicit limitations |
| --- | --- | --- | --- |
| 15-error-state ESKF | Primary navigation estimator | attitude, velocity, local position, IMU biases, covariance, aiding and recovery status | Requires completed initialization and sufficient healthy aiding for the requested navigation mode |
| Robust Mahony | Independent degraded-attitude fallback and cross-monitor | attitude and attitude-filter health | No position, velocity, navigation covariance, GNSS/barometer fusion, or full navigation integrity |
| Standard Mahony | Reproducible comparison baseline | attitude metrics in validation | Not the planned deployed fallback |

The two deployed candidates consume the same calibrated physical IMU publication, but maintain
separate contexts and state. A defect or reset in one estimator must not overwrite the other.

## Recommended application modes

1. `INITIALIZING`: no control-qualified estimate until the required alignment contract is met.
2. `PRIMARY_ESKF`: use ESKF outputs with per-field validity and freshness.
3. `DEGRADED_ATTITUDE_MAHONY`: expose Mahony attitude only; mark ESKF position, velocity, and
   navigation-dependent outputs invalid or stale. The controller must enter an explicitly reviewed
   limited mode rather than assuming full navigation remains available.
4. `ESTIMATE_INVALID`: neither estimator provides a qualified attitude; execute the FCOne safety
   response appropriate to vehicle state.

The supervisor may also report a warning state while retaining `PRIMARY_ESKF`, for example when one
aiding source is rejected but the ESKF remains observable and healthy. A single innovation reject,
magnetometer reject, or Mahony/ESKF disagreement must not cause an immediate source switch.

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
- Apply hysteresis and minimum healthy dwell time before returning to the ESKF.
- Never continue publishing old ESKF position or velocity as valid during Mahony-only operation.
- Validate every transition first in host replay and FCOne shadow mode; physical takeover requires
  the hardware test gates in the roadmap.

## Current boundary

This repository validates estimator behavior and the public health evidence needed by a supervisor.
It does not yet claim a flight-qualified automatic failover policy. FCOne-neutral mock-supervisor
contract tests are a pre-hardware task; target timing, actuator interaction, and in-flight abort
behavior require FCOne v2 hardware and HIL/flight evidence.
