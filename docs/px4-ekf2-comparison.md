# PX4 EKF2 comparison and unaided-navigation boundary

This document keeps comparisons between Aerakia and PX4 on a reproducible engineering basis. It
does not treat a filter name, state count, or one implementation's output as accuracy truth.

The PX4 source baseline reviewed on 2026-07-18 is
[`de8158101c96ad6b04170dc91f087148104c58eb`](https://github.com/PX4/PX4-Autopilot/tree/de8158101c96ad6b04170dc91f087148104c58eb).
PX4 changes over time, so conclusions about current implementation details must be refreshed when
this pinned baseline changes.

## EKF versus ESKF is not the relevant distinction

Aerakia explicitly calls its primary estimator an error-state extended Kalman filter. PX4 calls its
module EKF2, but the reviewed PX4 state definition also stores a nominal quaternion while assigning
only three covariance degrees of freedom to attitude. In that mathematical sense, PX4 EKF2 also
uses an error-state attitude representation. `EKF` versus `ESKF` in the project names therefore
does not imply that one architecture is inherently more accurate.

The meaningful differences are the state and measurement models, time alignment, fault logic,
initialization, tuning, numerical implementation, and amount of physical validation.

| Property | Aerakia current primary | PX4 EKF2 reviewed baseline |
| --- | --- | --- |
| Nominal/error-state size | 16 stored components / 15 error dimensions | 25 stored components / 24 covariance dimensions |
| Common states | attitude, NED velocity/position, gyro bias, accel bias | same |
| Additional estimated states | none in the primary covariance | earth/body magnetic field, horizontal wind, terrain |
| Heading redundancy | magnetic heading plus generic trusted-heading input | magnetic modes, GNSS yaw, external vision yaw, GNSS-velocity EKF-GSF yaw backup |
| Horizontal aiding | GNSS position, GNSS velocity, generic replay contract | GNSS, optical flow, external vision, auxiliary position/velocity, airspeed, sideslip, drag, range/beacon paths depending on build |
| Vertical aiding | GNSS position, barometer, ZUPT | GNSS, barometer, range, external vision and terrain/range logic |
| Delayed observations | timestamp freshness and order checks; no rewind/delayed fusion horizon | delayed fusion horizon plus output predictor and sensor-specific delays |
| Product fault handling | portable gates, source-specific rejection/recovery, host supervisor oracle | mature fusion-control state machines, resets, health publication, multi-instance selection, vehicle failsafes |
| Evidence maturity | extensive host replay; target and flight qualification open | years of broad vehicle deployment and upstream regression, while new GNSS-dead-reckoning mode is still documented as experimental |

PX4's larger model is not automatically better in every condition. It is substantially more
feature-complete and operationally mature, while Aerakia is smaller, easier to audit, and already
suitable for hardware-independent integration and shadow-mode evaluation. The current product gap
is dominated by missing sensor models and recovery policy rather than the nominal propagation
equations alone.

## What `pure inertial` means

Three cases must not be mixed:

1. **IMU-only inertial propagation:** accelerometer and gyro only. Horizontal position and velocity
   drift without bound on ordinary MEMS IMUs because residual bias, scale error, vibration, attitude
   error, and gravity leakage are integrated once or twice.
2. **GNSS-denied but externally aided dead reckoning:** no GNSS, but optical flow, VIO, airspeed,
   sideslip, wheel odometry, range, terrain, or another velocity/position observation is fused.
   This can remain useful for much longer, but it is not pure inertial navigation.
3. **Attitude/height continuation:** attitude may remain usable from gyro plus gravity/magnetic
   observations and height may continue from barometer/range even after horizontal navigation is
   invalid. This also is not proof of usable horizontal position.

The PX4 documentation explicitly says its GNSS-degraded/dead-reckoning mode requires an alternative
position or velocity source and is intended for intermittent dropout, not pure indoor GNSS-denied
operation. In the pinned source, `EKF2_NOAID_TOUT` defaults to 5 seconds and can be configured only
from 0.5 to 10 seconds. After that interval with no measurement constraining velocity drift, PX4
reports horizontal navigation invalid and the vehicle-level position-loss failsafe applies.

Therefore, Aerakia not providing long-duration IMU-only horizontal navigation is normal. The
required behavior is to make the growing uncertainty visible, invalidate navigation before it is
unsafe, preserve any still-observable attitude/height outputs, and reacquire valid aiding without
uncontrolled jumps.

## What current evidence says

UrbanNav `Medium-Urban-1` provides the first long recorded outage with independent SPAN-CPT
reference. During a 131-second interval with no physical F9P position updates, Aerakia's position-
only inertial error reaches 1471.5 m and its velocity error reaches 27.40 m/s. The first resumed
position update returns posterior position error to 5.395 m without a forced navigation reset.

That result establishes two different facts:

- 131 seconds of unaided position from this MEMS input is unusable; it must never be advertised as
  a navigation capability.
- The implementation remains finite and can reacquire a valid position observation, so the
  recovery path is materially better than an unbounded numerical failure.

It does **not** quantify a PX4/Aerakia accuracy ratio. PX4 has not yet been replayed on this immutable
UrbanNav input with an identical initial state, aiding mask, delay model, and cut interval. PX4
outputs in an ordinary ULog are also not independent truth. Any statement such as "PX4 drifts two
times less" would currently be invented.

## Fair same-input comparison gate

A quantitative comparison will be accepted only when both estimators receive the same immutable
sensor stream and scoring reference. The planned gate is:

1. pin the exact PX4 source commit and build configuration;
2. use identical IMU units, frames, timestamps, initial pose/velocity/position, and declared sensor
   covariances;
3. separate IMU-only propagation from no-GNSS operation with other aiding;
4. cut aiding over fixed 5, 10, 30, 60, and 120 second windows without using future truth;
5. report position, velocity, tilt, yaw, bias, covariance consistency, validity time, recovery jump,
   recovery time, CPU time, and memory;
6. retain failures and use an evaluation track that is not used to tune either filter.

Until that gate runs, the defensible overall judgment is:

- PX4 EKF2 is much more complete as a flight-navigation system and is the stronger operational
  benchmark.
- Aerakia's audited 15-error-state core is not known to be mathematically inferior merely because
  it is smaller, but it lacks several important PX4 capabilities.
- Both are unsuitable for long-duration IMU-only horizontal navigation with ordinary flight-controller
  MEMS sensors; PX4 explicitly invalidates that solution rather than claiming otherwise.

## Primary sources

- [PX4 EKF2 tuning and GNSS fault handling](https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf)
- [PX4 GNSS-degraded/dead-reckoning mode](https://docs.px4.io/main/en/advanced_config/gnss_degraded_or_denied_flight)
- [PX4 position-loss failsafe](https://docs.px4.io/main/en/config/safety#position-loss-failsafe)
- [Pinned PX4 generated EKF state definition](https://github.com/PX4/PX4-Autopilot/blob/de8158101c96ad6b04170dc91f087148104c58eb/src/modules/ekf2/EKF/python/ekf_derivation/generated/state.h)
- [Pinned PX4 EKF2 parameter definition](https://github.com/PX4/PX4-Autopilot/blob/de8158101c96ad6b04170dc91f087148104c58eb/src/modules/ekf2/module.yaml)

