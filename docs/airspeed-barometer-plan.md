# Airspeed and barometer aiding plan

FCOne v2 includes a barometer and an airspeed sensor. Both should be used, but they constrain
different parts of navigation and neither turns the system into a long-duration GNSS-independent
INS by itself.

## Current state

The 15-error-state ESKF already accepts timestamped relative barometric height and applies a scalar
Down-position update. The adapter rejects duplicate, reordered, future, stale, non-finite, and
invalid-variance observations, and accepted barometer updates refresh vertical-position validity.
It does not estimate barometer bias or drift. The ULog converter currently subtracts the first
valid barometric altitude and assigns a fixed `1 m^2` variance.

No airspeed, pitot, sideslip, drag, or wind state exists in the core, public API, native replay
runner, or automated tests. Existing general navigation-outage scores contain no airspeed benefit;
barometer benefit is now isolated in the separate four-arm study and must not be backfilled into
older results.

## Measurement models

### Barometer

The current model is:

```text
z_baro_down = p_D + noise
```

The target model is:

```text
z_baro_down = p_D + b_baro + noise
b_baro_dot = random_walk
```

Before adding a barometer-bias state, the adapter layer can safely own pressure-to-height
conversion, startup datum, GNSS/barometer datum alignment, quality checks, and a frozen datum during
GNSS outage. The current experimental source supervisor covers physical time, jump, quantization-
aware freeze, latch, two-stage fused-baseline commit, and externally authorized return-to-baseline
recovery. Thermal
drift, pressure conversion, redundant-source voting, and real full-state failover remain open.

### Airspeed and wind

Airspeed is relative to the air mass, not the ground:

```text
v_air_n = v_ground_n - wind_n
TAS = norm(v_air_n)
```

For a calibrated true-airspeed observation, the scalar prediction is:

```text
h(x) = sqrt((v_N - w_N)^2 + (v_E - w_E)^2 + v_D^2)
```

The first estimator candidate should add horizontal wind states `w_N,w_E`, producing a 17-error-
state experimental branch. It should not estimate pitot bias initially. IAS/CAS conversion to TAS,
including static pressure and temperature, remains an upstream sensor-adapter responsibility.

This scalar model assumes zero vertical wind and an ideal, calibrated true-airspeed magnitude. A
real pitot system measures probe-axis dynamic pressure and is sensitive to density, probe alignment,
angle of attack, sideslip, blockage/icing, and rotor/propeller wash. Before a 17-state wind branch is
accepted, negative controls must inject vertical wind, AoA/sideslip, installation misalignment,
scale/offset error, and VTOL transition/rotor-wash invalidity. These conditions require explicit
source-validity gates rather than being silently absorbed into the wind states.

An optional no-sideslip pseudo-observation is:

```text
body_y dot R_bn * (v_ground_n - wind_n) = 0
```

That assumption is valid only in explicitly qualified coordinated fixed-wing flight. It must be
disabled in hover, low airspeed, stall, aggressive sideslip, rotor downwash, and unqualified VTOL
transition regimes.

## Phased implementation

### A0: contracts and synthetic truth

- Define barometer datum/source-generation/reset semantics and TAS type, variance, timestamp,
  latency, quality, blocked/stalled, and flight-regime fields.
- Generate paired truth for wind, pressure altitude, barometer bias, and calibrated TAS.
- Keep the production 15-state estimator unchanged.

### A1: barometer qualification

- Add pressure-to-relative-height and datum management outside the core.
- Test bias, random walk, weather step, temperature drift, freeze, delay, reset, and recovery.
- Compare IMU-only and IMU+barometer with identical truth/noise across 5/10/30/60/120 s GNSS
  outages.
- Add a barometer-bias state only if the external datum model cannot meet the frozen vertical gates.

The first hardware-free A/B exposed the need for source supervision and an uncontaminated shadow
lane. Results, rejected threshold experiments, and the current capability boundary are retained in
[the barometer supervision study](barometer-supervision-study.md).

### A2: airspeed upper bound and wind observability

#### A2.0: validation-only source contract and observability oracle

Before an estimator state or public TAS API exists, freeze the source-time,
quality, and causal-observability boundary.  This is now implemented by the
[TAS/wind observability prerequisite](airspeed-wind-observability.md): it
separates an offline known-wind model upper bound from a truth-free causal
two-state information/least-squares oracle and retains positive plus
fail-closed controls.  It does not modify the production 15-error-state ESKF.

The first multi-seed confirmation retained a real blocker: an **unflagged**
vertical-wind mismatch reached a qualified output in one of 32 draws. This is
not fixed by retuning NIS; scalar TAS plus GNSS velocity cannot prove that the
vertical-wind model assumption holds. The next source-contract revision must
make its independently justified fixed-wing/regime qualification explicit and
then validate it against physical air-data before any 17-state branch opens.

#### A2.1: isolated experimental branch

- First run TAS with known wind to establish the model's upper bound.
- Only after A2.0 passes, implement an isolated 17-error-state wind branch and repeat with estimated wind.
- Use straight, multi-heading, climbing, descending, turning, gust, and sideslip-violation tracks.
- Promote only after train and tune both show improvement and a sealed holdout passes.

### A3: FCOne physical qualification

- Measure pressure, temperature, pitot scale/zero, latency, blockage, motor/propeller interference,
  source reset, and redundant-source switching.
- Run the estimator in shadow mode before it can affect control.
- Freeze regime-specific enable rules for hover, transition, fixed-wing cruise, and landing.

## Paired validation matrix

| Arm | Purpose |
| --- | --- |
| IMU only | unaided drift baseline |
| IMU + barometer | vertical aiding benefit |
| IMU + TAS + known wind | airspeed-model upper bound |
| IMU + TAS + estimated wind | wind observability and practical benefit |
| IMU + barometer + TAS + estimated wind | target fixed-wing degraded-navigation configuration |

Every arm uses identical truth, sensor errors, initialization, and outage windows. Required scenarios
cover hover, transition, fixed-wing straight/turn/climb/descent; still air, head/tail/crosswind,
gust and shear; GNSS outages of 5/10/30/60/120 seconds; barometer and pitot faults; low-airspeed and
sideslip negative controls; and GNSS reacquisition.

Required metrics include vertical/horizontal position and velocity error, yaw, wind error, NIS/NEES,
validity timeout, false acceptance, fault-detection delay, recovery time, and state/output jump.

## Capability boundary

Barometric height can strongly reduce vertical-position drift but does not by itself qualify
vertical velocity. TAS plus a wind model can constrain part of horizontal velocity in fixed-wing
flight and can extend useful degraded operation. Neither supplies absolute horizontal position or
absolute yaw. Wind change, sideslip, airspeed bias, and model mismatch still produce growing
position error during a long GNSS outage. The intended claim is longer controlled degradation and
better fault handling, not indefinite accurate GNSS-denied navigation.
