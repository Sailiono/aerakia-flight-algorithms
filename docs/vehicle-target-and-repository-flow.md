# FCOne/eVTOL target profiles and repository flow

The public estimator remains vehicle-neutral at the C API and state-model level, but accuracy,
noise, fault, and non-inferiority claims are always tied to a declared vehicle and flight regime.
One parameter set per log is forbidden.

## First vehicle-family target

The first product target is the FCOne eVTOL program. Its evaluation is split by flight regime
instead of treating an eVTOL as one stationary statistical process:

1. **VTOL hover, takeoff, and landing:** low groundspeed, rotor vibration, possible magnetic-current
   interference, weak GNSS-velocity yaw observability, and strict vertical/attitude validity.
2. **Transition:** changing vibration and specific-force spectra, changing aerodynamic constraints,
   and explicit mode/state boundaries. This is a separate stress and validation profile.
3. **Fixed-wing cruise and maneuver:** higher groundspeed, GNSS velocity and optional air-data
   observability, longer aiding interruptions, and different process/measurement uncertainty.

The first control-authority candidate is the VTOL/hover profile. Transition and fixed-wing profiles
are added only after the preceding profile passes independent truth, same-input PX4 A/B, shadow,
HIL, and bounded-flight gates. Shared algorithm math is not forked by vehicle; reviewed
configuration profiles and private supervisor policy express justified differences.

## Source-of-truth rule

`aerakia-flight-algorithms` is the only editable source for Mahony, ESKF, their public data
contracts, mathematical tests, and hardware-independent validation. The private FCOne/eVTOL stack
pins a reviewed algorithm commit and owns:

- board drivers and calibrated redundant-sensor publications;
- sensor voting, isolation, switch policy, and switch event logging;
- physical-to-arrival timestamp tracking and delayed-observation handling;
- vehicle-mode profiles, estimator supervision, controller/failsafe policy, and HIL/flight logs;
- hardware schematics, production calibration, target resource results, and confidential datasets.

No ESKF or Mahony source copy may be edited in the private stack. A private hardware finding that
changes algorithm math becomes a minimal hardware-independent reproducer and test in the public
repository first; the private stack then advances its pinned commit.

## Bidirectional development loop

1. The FCOne/eVTOL program defines a concrete task envelope, failure mode, sensor contract, and
   acceptance budget.
2. General math, validity semantics, and reusable fault behavior are implemented and reviewed in
   the public algorithm repository.
3. The private stack integrates an immutable public commit through its adapter and adds redundant
   sensor management, delays, vehicle modes, and safety policy.
4. Shadow/HIL/flight evidence is scored privately. Sanitized, general conclusions and reproducible
   non-sensitive tests return to the public repository.
5. The public release is tagged only after the profile-specific capability gates pass; private
   evidence remains linked to the exact public SHA.

This is not “optimize privately and later copy code public” or “finish an abstract algorithm and
throw it over the wall.” Product requirements flow inward, while algorithm code has one public
source and product-specific engineering remains private.

## Sequencing boundary

Mathematical correctness and estimator accuracy are the current critical path, but safety-relevant
contracts cannot be postponed until the end. Dimension-aware innovation gates, continuous validity,
supervised recovery, timestamps, and transition logs are developed alongside the filter because
they determine whether a numerically accurate state may be used. Hardware-specific voting,
temperature characterization, actuator magnetic testing, WCET, and flight policy remain later
private integration stages.
