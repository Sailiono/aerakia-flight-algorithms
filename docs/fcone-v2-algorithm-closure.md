# FCOne v2 pre-hardware algorithm closure

## Purpose and decision

This document closes the hardware-independent portion of the FCOne v2 estimator
work to a defined boundary. It is not a flight-readiness or airworthiness
statement.

The current portable core may enter FCOne v2 as a **private adapter plus shadow
estimator** once the board's exact timestamp, frame, unit, validity, and
scheduler contract is available. It must not yet be the sole control-authority
estimator. The first flight profile is an ESKF-primary profile with robust
Mahony as a time-bounded, attitude-only fallback/cross-monitor. It is never a
second navigation solution.

This boundary lets the project start extracting hardware and flight evidence
without confusing PC confidence with in-air safety.

## Closed hardware-independent gates

| Gate | Evidence and result | Status |
| --- | --- | --- |
| Core ESKF mathematics | 15-error-state covariance, finite-difference transition and raw-heading Jacobians, Joseph updates, covariance PSD/symmetry, and continuous-noise mapping tests | Closed for declared model |
| High-rate process noise | Independent frozen-coefficient RK4 `Qd` oracle over 1,000 deterministic 100--1000 Hz cases; largest relative Frobenius defect `0.0502%` against a predeclared 1% limit | Closed only inside declared envelope |
| Timing and precision | 100/200/400/1000 Hz gates; timestamped aiding delay matrix; real float-vs-double comparison at 400 Hz gives clean-motion maximum deltas of `0.001174 deg`, `3.943 mm`, and `0.670 mm/s`; Cortex-M7 strict cross-compile/layout preflight passes | Closed for host evaluation and target build compatibility; H7 image WCET/stack remain open |
| Input transport robustness | Required-IMU non-finite checks, timestamp disorder/gaps, optional-source isolation, delayed/stale aiding, and burst loss through 100 samples | Closed for public input contract |
| Synthetic fault and consistency | 1,000 calibrated-bias seeds pass stated health, NIS/NEES, recovery, and bootstrap gates; deterministic residual-bias box retains a documented direction-sensitive 35 s convergence defect | Closed as a known boundary, not a universal bias claim |
| Independent truth replays | EuRoC, Blackbird, UrbanNav, electrical-infrastructure UAV RTK track, physical-magnetometer INSANE tracks, and private PX4 ULog replays are separately scoped in `algorithm-status.md` | Closed as evidence, not pooled into one accuracy claim |
| Same-input PX4 baseline | Pinned official `ecl_EKF` and Aerakia use one immutable 65 s / 100 Hz synthetic stream and identical delayed fusion horizons | Closed as host transport/provenance evidence; both filters remain healthy but neither meets the absolute bias-settling gate, so G2 non-inferiority is still open |
| Magnetic fail-safe behavior | Invalid references are rejected; bad magnetic data can be isolated; magnetometer fusion remains disabled by default after physical slow-datum failures | Closed fail-safe policy; source-quality promotion remains open |
| State choice | Current `p,v,q,b_a,b_g` model is documented, tested, and matched to VTOL-hover shadow work | Closed for first v2 shadow profile |

The detailed quantitative results, data provenance, and retained adverse cases
remain in [algorithm status](algorithm-status.md),
[validation](validation.md), and [public datasets](public-datasets.md). This
document does not replace them.

## Required integration boundary

The private FCOne v2 adapter must preserve all of the following before any
flight comparison is credited:

1. IMU timestamps are physical sample times, not task-release times; delta
   angles and delta velocities carry their exact integration intervals.
2. IMU axes are transformed once into the documented body-FRD convention;
   all units are converted before entering the portable API.
3. Each aiding source has independent validity, source identity, age, variance,
   reset generation, and arrival/sample timestamps. GNSS course must not be
   passed as body heading.
4. Dual-antenna RTK is explicitly labelled as an independent trusted-heading
   observation only when its baseline, fix status, covariance, and geometry
   are valid. A loss or disagreement must remove heading authority, not invent
   yaw from course over ground.
5. ESKF, robust Mahony, and source-health diagnostics run in parallel during
   the first bench/HIL/flight campaigns. The controller initially consumes the
   incumbent qualified source, while Aerakia writes comparison logs.

The executable portable adapter tests are necessary but not sufficient: exact
v2 DMA timing, sensor selection, calibration, scheduler latency, and source
status semantics are board-specific work.

## Why the initial model is 16 nominal / 15 error dimensions

The stored nominal state is:

```text
x = [p_NED(3), v_NED(3), q_body_to_NED(4), b_accel(3), b_gyro(3)]
```

It contains sixteen scalar components. The quaternion is constrained to unit
length, so its local perturbation has only three independent components. The
covariance therefore represents:

```text
dx = [dtheta(3), dv(3), dp(3), dba(3), dbg(3)]
```

which is fifteen-dimensional. Both names are correct only when their meaning
is stated: **16 nominal components / 15 local error dimensions**.

This model already estimates the error sources needed for initial hover and
validation flight: attitude, velocity, position, gyro bias, accelerometer
bias, GNSS position/velocity, trusted heading, barometric height, and static
constraints. It is the right starting point for a shrinking-scale test vehicle
because its assumptions are weakest in hover and low-speed operation.

Adding dimensions is not free accuracy. Every new state needs a physical
measurement, process noise, initialization, Jacobians, NIS/NEES tests, fault
behavior, calibration data, a sealed holdout, and a target timing/stack check.
An unobservable state can make the filter look more sophisticated while
spreading uncertainty or creating overconfidence.

## Future extensions, not immediate requirements

| Candidate | New local error dimensions | Promotion trigger | Not a reason to add it now |
| --- | ---: | --- | --- |
| Horizontal wind | +2 (17 total) | Validated fixed-wing/transition true-airspeed data across headings and wind excitation demonstrates better GNSS-denied performance | Hover, low TAS, sideslip, and rotor wash make it weakly observable |
| Local gravity | +3 (18 total) | Long-range/high-altitude truth shows deterministic WGS84/local-gravity updates are the material error source | A change of test location needs deterministic gravity/declination updates, not a random gravity state |
| Barometer bias | +1 (16 total) | An independent vertical reference proves random-walk bias beats source supervision/datum handling | A single barometer cannot observe its own absolute datum bias |
| Pitot scale/offset | +1 or +2 | Calibrated air-data residuals show an in-filter state beats upstream calibration | Pitot blockage, density, and sideslip are source-integrity problems first |
| Full magnetic field | +6 | A full 3D field model and independent absolute heading truth pass a frozen test campaign | It cannot repair the observed slow magnetic datum shift without an independent heading source |

The next plausible extension is a separate 17-error-state wind branch for
fixed-wing cruise. It must stay disabled in hover/transition unless its
observability conditions are met. It is not a prerequisite for the first v2
shadow estimator.

## Dual-antenna RTK and inertial integrity

Dual-antenna RTK should be the primary absolute-heading reference when its
receiver quality is valid. That greatly improves yaw observability, cold start,
and magnetic-disturbance recovery. It does not remove the need for a sound
inertial estimator:

- short RTK dropouts still require stable attitude propagation and an explicit
  heading-validity age;
- baseline faults, multipath, receiver resets, or bad covariance must be
  detected and rejected rather than copied into yaw;
- GNSS position/velocity and heading do not correct timing, frame, scale,
  clipping, thermal bias, or sensor-selection defects at the instant they
  occur;
- long GNSS-denied navigation remains fundamentally error-growing without
  another independent constraint such as airspeed/wind, vision, range/terrain,
  or external navigation.

Thus the v2 design should use dual-antenna RTK as a high-value trusted source,
not as permission to weaken IMU, source-supervision, or degradation policy.

## What remains before control authority

The following cannot be closed from PC datasets alone:

1. Exact v2 adapter and scheduler contract tests against the generated board
   project.
2. STM32H743 worst-case execution time, stack use, precision/numerical-health,
   and watchdog behavior at all enabled rates.
3. Static, multi-pose, thermal, vibration, motor-current magnetic, and
   redundancy/failover characterization with physical sensors.
4. HIL and shadow-flight comparison, including log review and abort criteria.
5. A pre-registered physical absolute-heading campaign using the actual
   dual-antenna RTK, including loss, reset, outlier, and recovery behavior.
6. A formal private supervisor policy that defines controller authority,
   fallback duration, navigation invalidation, and recovery reset semantics.

Until these gates pass, the appropriate next action is **shadow integration**,
not direct replacement of the aircraft's only estimator or control input.
