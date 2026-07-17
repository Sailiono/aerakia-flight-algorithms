# ULog yaw diagnostics

## What each yaw comparison means

PX4 ULogs can contain three different heading signals with different purposes:

1. `vehicle_attitude` yaw is PX4's fused attitude estimate. It can jump when
   the estimator injects a quaternion reset.
2. The **de-reset diagnostic** subtracts each observed attitude-yaw jump and
   keeps one global alignment. It reconstructs estimator continuity across
   corrections; it is not a physical-heading reference because a reset may be
   correcting accumulated error.
3. `yaw_estimator_status.yaw_composite` is PX4's GNSS-velocity-aided GSF yaw.
   It is independent of magnetometer heading, but it is observable only during
   suitable horizontal motion and is still an estimate rather than ground
   truth. The converter admits it only when reported variance is at most
   0.04 rad².

The per-reset-segment metric realigns every segment independently. It is useful
for checking local shape but can hide long-term drift and therefore is not a
performance metric.

## Current private replay evidence

Seven selected ULogs were replayed after enforcing complete replay/result row
counts. Raw logs and generated per-flight reports remain private. Aggregate
ESKF yaw RMSE in static and fixed-wing cases is approximately 1.2–2.0° against
PX4 attitude yaw. Two multirotor heading-stress cases contain two and four
large PX4 attitude resets; their raw PX4-relative yaw RMSE is approximately
18° and 49°.

In those two cases, globally aligned yaw-trajectory RMSE against converged GSF
yaw is approximately:

| Comparison | Dynamic multirotor | Heading-stress multirotor |
| --- | ---: | ---: |
| Aerakia ESKF vs GSF | 14.2° | 10.2° |
| raw PX4 attitude yaw vs GSF | 30.2° | 51.5° |
| de-reset PX4 yaw vs GSF | 100.7° | 110.2° |

This evidence rejects the simple explanation that Aerakia yaw error is merely
caused by ignoring PX4 resets. It also shows why the de-reset trajectory must
not be treated as physical truth: PX4's resets move the attitude solution
toward its non-magnetic GSF evidence. Aerakia tracks the GSF trajectory better
than PX4 attitude yaw in these two captures, but its remaining 10–14° error is
too large for a flight-readiness claim.

## Engineering conclusion

The available ULogs contain GNSS position and velocity, but not dual-antenna
GNSS heading or an external attitude truth system. The current Aerakia ESKF
uses GNSS position/velocity for navigation corrections; it does not yet use
course-over-ground as a qualified heading observation. Course-derived heading
must be gated by speed, horizontal acceleration/slip conditions, uncertainty,
and vehicle mode before it can safely assist yaw.

Next evidence should combine:

- a portable GNSS-velocity heading-source contract and fault gates;
- replay comparison to GSF only in observable motion;
- controlled turntable or motion-capture heading truth;
- new FCOne hardware tests with magnetic interference cases and known sensor
  timing.
