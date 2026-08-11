# TAS and horizontal-wind observability prerequisite

## Decision and scope

This is the first airspeed/wind prerequisite for FCOne v2.  It creates a
**host-only validation oracle** for the question that must be answered before
any 17-error-state wind experiment:

> Given calibrated true airspeed and fresh GNSS ground velocity, is horizontal
> wind causally observable in this flight segment, or must the source be
> rejected/fail closed?

It does **not** add wind states, TAS fields, air-data conversions, or an
airspeed update to the portable production ESKF.  The 16 nominal / 15 local
error-state baseline remains unchanged.  No synthetic result in this document
is a claim of long-duration GNSS-denied navigation, target readiness, or
flight authority.

The executable protocol is
[`validation/airspeed_wind_observability_protocol_v1.json`](../validation/airspeed_wind_observability_protocol_v1.json),
the host runner is
[`validation/run_airspeed_wind_observability.py`](../validation/run_airspeed_wind_observability.py),
and the reviewed compact result is retained under
[`validation/public/`](../validation/public/).

## Measurement, time, and source contract

All vectors use NED.  A calibrated true-airspeed observation is the magnitude
of air-relative velocity, not ground speed and not unconverted IAS/CAS:

```text
TAS = || v_ground_NED - [w_N, w_E, 0] ||
```

For the horizontal wind vector `w = [w_N, w_E]`, the oracle uses:

```text
h(w) = sqrt((v_N - w_N)^2 + (v_E - w_E)^2 + v_D^2)
dh/dw = -[(v_N - w_N), (v_E - w_E)] / h
```

The effective scalar variance includes the declared TAS variance and the
projected three-axis GNSS-velocity variance.  The analytic Jacobian is checked
against a symmetric finite difference before a positive case may pass.

Every production candidate must receive, at minimum:

- physical TAS and arrival timestamps;
- a GNSS-Doppler velocity timestamp, vector, and variance;
- a physical delivery-delay bound and a GNSS-age bound;
- explicit fixed-wing regime qualification;
- source validity plus blocked/stalled pitot, rotor-wash, and sideslip
  qualification.

The first frozen validation limits are `TAS >= 12 m/s`, delivery delay no more
than `0.25 s`, and GNSS velocity age no more than `0.20 s`.  They are a source
contract for the oracle, not final FCOne flight thresholds.

Vertical wind, probe alignment, density/calibration error, angle of attack,
sideslip, blockage, icing, and rotor/propeller wash are **not** silently
absorbed into a horizontal wind state.  The private air-data adapter and flight
regime supervisor must report them as qualified or invalid before a future
estimator branch can use TAS.

## Two deliberately separate lanes

### Known-wind model upper bound

The known-wind lane receives an externally supplied/offline wind reference. It
only validates the TAS equation, frames, units, timestamps, effective
variance, and Jacobian. It is not an estimator and its wind reference must
never be made available to a flight runtime.

### Causal two-state wind oracle

The causal lane receives only runtime-available observations. It holds a
static horizontal wind hypothesis and performs causal nonlinear least squares
with an information matrix.  Truth is stored in a different object and is read
only by the post-run scorer for wind error/NEES reporting.

The source is not declared wind-observable merely because the 2x2 matrix has
local rank two. Scalar range equations from only two velocity centres can have
a mirror solution.  Qualification therefore requires all of the following:

1. at least 24 accepted observations;
2. rank 2, a frozen minimum information eigenvalue, and a bounded condition
   number;
3. at least 35 degrees of air-relative directional separation;
4. three distinct full-circle direction clusters; and
5. 12 additional bootstrap observations after the third direction first
   appears.

The final condition lets the truth-free squared-range initializer resolve the
two-range ambiguity before innovation gating becomes active. It avoids
incorrectly latching a valid new heading merely because a two-direction seed
landed at its mirror solution.

After geometry is qualified, a post-qualification NIS above the frozen bound
is rejected. Three consecutive such rejections latch the source and require
external reauthorization; automatic recovery is deliberately forbidden in this
oracle. This is a validation policy candidate, not FCOne's final private
supervisor implementation.

During a GNSS velocity outage, TAS alone supplies no new wind-information
claim: the terminal state ages to `stale_no_fresh_gnss_tas`. It may retain a
previous estimate for diagnostics, but it cannot remain qualified merely
because airspeed messages continue arriving.

## Required controls

The frozen matrix includes a qualified multi-heading fixed-wing positive case
and these fail-closed controls:

- one straight heading (no 2D information);
- hover, low TAS, VTOL transition, and rotor wash;
- unqualified sideslip;
- blocked or stalled pitot;
- excessive delivery delay and reordered TAS timestamps;
- GNSS velocity outage;
- wind shear, TAS scale/bias, and vertical-wind model mismatch.

An invalid observation must neither change the estimate nor add information.
A reordered sample is a transport control: an otherwise valid prior estimate
may remain qualified, but the duplicated/reordered sample itself must be
rejected with exactly zero state/information mutation.

## Reproduction and promotion boundary

Run the deterministic source-contract matrix with:

```bash
python3 validation/run_airspeed_wind_observability.py
python3 -m unittest tests.test_airspeed_wind_observability -v
```

The compact JSON records protocol and runner hashes, Git identity, every
scenario's decision, offline-only truth score, and limitations. Raw high-rate
replays and physical air-data logs remain outside public Git under the
[workspace artifact policy](workspace-artifact-policy.md).

This prerequisite is complete only when its matrix passes.  It does not
promote a 17-error-state ESKF.  Promotion remains blocked on a separate,
sealed physical fixed-wing train/tune/holdout set with calibrated TAS, source
time provenance, heading diversity, wind excitation, GNSS outage/recovery,
and a measured benefit against the frozen 15-error-state baseline.  Target
precision, timing, stack, memory, private source selection, shadow logging,
and flight evidence are separate gates.
