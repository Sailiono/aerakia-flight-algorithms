# State augmentation policy

## Decision

FCOne v2's first shadow-flight integration uses the existing 16-component nominal state and
15-dimensional local error state. New states are added only through a separately validated,
vehicle-profiled experimental branch. A larger state vector is not a substitute for physical
source quality, timestamp correctness, sensor redundancy, or an observable measurement.

## Current state definition

The nominal state is:

```text
x = [p_NED(3), v_NED(3), q_body_to_NED(4), b_accel(3), b_gyro(3)]
```

It stores 16 scalar components. The quaternion has one unit-norm constraint, so the local error
state uses a three-component rotation perturbation instead of four unconstrained quaternion
components:

```text
dx = [dtheta(3), dv(3), dp(3), dba(3), dbg(3)]
```

The covariance is therefore 15 by 15. Calling this a "15-state ESKF" is correct when referring to
the estimated error degrees of freedom; calling it "16 nominal components / 15 error dimensions"
is the least ambiguous wording.

## Why the first flight profile keeps 15 error dimensions

The current state covers the quantities required for cold start, inertial propagation, GNSS
position/velocity correction, trusted heading, barometric height, static constraints, and IMU-bias
learning. Its mathematical implementation, health behavior, and hardware-neutral adapter contract
have executable evidence. It is also the smallest model compatible with the first VTOL hover
profile, where wind and aerodynamic assumptions are weak and magnetic heading is not trusted by
default.

Adding a state costs more than memory. It adds covariance coupling, process noise, initialization,
observability conditions, NIS/NEES tests, fault policy, tuning data, and a new failure mode when the
state is weakly observed. A state that cannot be constrained by an independent measurement merely
redistributes error and can make covariance appear confident without improving navigation.

## Candidate extensions

| Need | Candidate added error dimensions | When it is justified | Why it is not in the first FCOne v2 profile |
| --- | ---: | --- | --- |
| Location-dependent gravity | 3 | Long-range/high-altitude truth shows deterministic local gravity modelling is a dominant error after WGS84 origin and gravity updates | Changing field or test location does not require estimating gravity; update the deterministic local model first |
| Horizontal wind with true airspeed | 2 | Fixed-wing or qualified transition data provides calibrated TAS across multiple headings and wind excitation | Hover, low airspeed, sideslip and rotor wash make wind weakly observable or invalid; current core has no TAS contract |
| Barometer bias | 1 | A pressure/source model plus independent vertical reference proves a random-walk bias state beats datum/source supervision | A datum offset is not observable from the same barometer alone; source health and a shadow lane come first |
| Pitot scale/offset | 1--2 | Calibrated air-data data demonstrates persistent residuals not resolved by upstream calibration | Pitot error, density, sideslip and blockage are source-level problems before they are estimator states |
| Earth/body magnetic field | 6 | A full three-dimensional magnetic observation, calibrated field model, and independent heading truth pass a frozen evaluation | Current magnetic update is deliberately yaw-only; adding magnetic states would not solve the observed slow datum shift without independent heading |
| Terrain/range | 1 or more | A range/terrain source is a declared product sensor and has a terrain model plus HIL evidence | Not part of the initial FCOne sensor claim |

For example, horizontal wind would produce an 18-component nominal state and a 17-dimensional error
state. Solà's optional gravity formulation produces 19 nominal components and 18 error dimensions.
Neither number is inherently more capable than the current 16/15 model; usefulness follows from the
measurement model and flight regime.

## Compute and memory implication

The persistent double-precision covariance alone occupies `8 * n^2` bytes: 1,800 bytes for
`n = 15`, 2,312 bytes for `n = 17`, 2,592 bytes for `n = 18`, and 3,528 bytes for `n = 21`.
Temporary update matrices and covariance algebra grow similarly or faster. STM32H7 memory is not
the primary reason to avoid a modest extension, but the current code's double precision and
worst-case stack use must be measured before any product claim. More dimensions also increase the
amount of target timing and numerical evidence required.

## Promotion rules

An experimental state branch may become a reviewed FCOne profile only after all of the following:

1. A physical measurement model and coordinate/timestamp contract are documented.
2. Finite-difference Jacobians, process-noise behavior, NIS/NEES, invalid-input, delay, dropout,
   fault, and recovery tests pass.
3. Calibration, development, and sealed holdout datasets demonstrate a material benefit over the
   frozen 15-error-state baseline without hiding failure cases.
4. The enable rule is limited to the flight regimes where the state is observable.
5. H7 WCET, stack, memory, HIL, and private shadow-flight results pass before the branch receives
   control authority.

The next justified extension is the isolated 17-error-state horizontal-wind experiment for
fixed-wing cruise, after the airspeed contract and validation plan are complete. It is not a
prerequisite for the VTOL-hover shadow-flight profile.
