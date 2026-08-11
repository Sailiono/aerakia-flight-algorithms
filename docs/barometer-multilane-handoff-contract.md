# Barometer multi-lane handoff contract

## Purpose

The barometer supervision study showed that stopping a faulty scalar height
update after a latch cannot undo the nominal-state and covariance changes made
by earlier accepted samples. Its old Python shadow arm therefore switched only
vertical position, vertical velocity, and health as an offline architecture
upper bound. That is deliberately not a safe estimator handoff.

This host-only contract defines the minimum all-or-nothing transaction for a
future private FCOne supervisor:

```text
active lane: IMU + common aiding + selected barometer
shadow lane: IMU + common aiding, no selected barometer
                     |
                     | qualified source latch
                     v
output lane := complete shadow lane
```

The transaction transfers the complete pointer-free `AerakiaEskf` image, not a
selection of fields. That includes the 16-component nominal state, all 225
entries of the 15x15 covariance, error-state image, physical/aiding timestamps,
validity and alignment state, recovery state, innovation metadata, and
magnetometer-gate state. It records attitude, position, velocity, bias, and
covariance reset deltas for the private downstream controller interface.

This file is validation infrastructure. It does **not** implement FCOne lane
selection, concurrent/atomic transfer, actuator reset handling, redundant
barometer voting, or an automatic return to the barometer lane.

## Executable contract

[`barometer_multilane_handoff_contract.c`](../validation/barometer_multilane_handoff_contract.c)
drives two complete portable adapter instances from an audited common stream.
The only input difference is active-lane barometer enablement. The common stream
contains IMU, position, velocity, and trusted-heading observations; its event
counts and deterministic byte-field hash must match before handoff.

The active lane uses the existing causal barometer supervisor. A frozen relative
height stream is injected after normal motion. The supervisor latches after the
configured repeated-value test, after which exactly one active-to-shadow handoff
may occur.

The transaction refuses to change the output lane when any of these conditions
is false:

- a fault latch exists;
- both lanes have identical common-event provenance;
- configuration identity and barometer source id/generation/quality sequence
  match the expected source context;
- both lane timestamps identify the same IMU boundary;
- the shadow image is finite, healthy, and statically aligned; and
- no lane switch has already completed.

An attempted automatic return to the contaminated barometer lane is explicitly
blocked. A real private recovery policy needs separate external authorization
and controller-level review; this host contract intentionally does not grant it.

## Reviewed result

The deterministic success matrix runs at 100, 200, and 400 Hz. Each rate
completes static alignment, consumes common IMU/position/velocity/heading
events, reaches a real supervisor freeze latch, transfers the entire shadow
image, and remains bit-equivalent to the shadow after one additional common IMU
event.

The 400 Hz representative path carried 158 IMU events, two position updates,
two velocity updates, and two heading updates through the selected output. Its
recorded handoff deltas were:

| Reset diagnostic | Value |
| --- | ---: |
| Attitude | `0 rad` |
| Position norm | `0.10191299 m` |
| Velocity norm | `0.0106834192 m/s` |
| Maximum absolute covariance entry | `0.0982111301` |

The zero attitude delta is expected for this level, vertical synthetic motion.
The nonzero position, velocity, and covariance deltas are the important result:
they prove that a vertical-only output splice would omit estimator context.

The same executable fail-closed matrix independently mutates the shadow
timestamp, common provenance, configuration identity, source generation,
alignment completion, quaternion normalization, and covariance finiteness. It
also tries a missing latch and an automatic return. Every case is blocked and
the preexisting output image remains unchanged.

## Reproduction

```bash
cmake -S . -B build/dev -DCMAKE_BUILD_TYPE=Release
cmake --build build/dev --parallel
ctest --test-dir build/dev --output-on-failure \
  -R barometer_multilane_handoff_contract
```

The target is part of the default CTest suite. It must pass in the reviewed
double build, the host-float evaluation build, and ASan/UBSan before a change
to this contract is accepted.

## Not closed

The test has no physical pressure, temperature, plumbing, vibration, GNSS/RTK
availability, target task scheduling, or controller. Its source identity is a
deterministic host fixture, not FCOne redundant-source policy. It validates
transaction correctness only; it cannot claim barometer fault-tolerant flight
behavior or authorize the public supervisor as a control gate.
