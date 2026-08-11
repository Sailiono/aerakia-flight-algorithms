# v8 diagnostic-only residual lane

## Decision

The v8 policy screen exposed an information limit, not an ESKF arithmetic
defect. With only source time, arrival time, validity, source epoch, and NIS,
the runtime cannot know whether a persistent high residual began before or
after an offline injection boundary. Extending a stale quiet boundary would
therefore improve a score by importing ambiguity into control authority.

The next safe step is to expose a separate host-side diagnostic result:

```text
UNQUALIFIED_PERSISTENT_RESIDUAL
```

This result records that a continuous high residual was observed after the
last complete quiet boundary became too old. It does not classify the source,
does not qualify a control path, and does not authorize a selector, estimator
reset, or controller mode change.

## Exact semantics

The implementation is an orthogonal lane in
`validation/diagnose_airspeed_wind_residual_persistence_v8.py`; the existing
`ProbeState` and `latched` fields retain their prior meaning.

The diagnostic lane can start only when all of the following hold:

1. the source epoch is authorized;
2. source and arrival timestamps are strictly monotonic and inside the gap
   limits;
3. the input is valid and NIS is finite and non-negative;
4. the current epoch has previously completed one full quiet boundary; and
5. the current boundary is no longer recent under the configured bounded age.

It then requires the ordinary strict-high observation count and source-time
span. A low or mid-band sample interrupts an unfinished diagnostic episode;
it cannot bridge evidence. The first completed diagnostic episode in an
authorized epoch creates one immutable snapshot containing the prior quiet
boundary, episode onset, latch time, observation count, and source span.

The snapshot is cleared by a transport/validity/epoch reset. A newly completed
quiet boundary clears the *current* diagnostic status but does not rewrite the
historical snapshot. This is intentionally a small audit surface, not a
general event store.

## What this does not solve

The lane does not distinguish pitot fault, wind change, sideslip, timing error,
GNSS inconsistency, or estimator error. It also does not solve the v8
`graded_evidence` sensitivity misses. Those require an independently audited
causal regime/source transition, such as a validated air-data/GNSS pairing,
source-health transition, or flight-phase/arming epoch. That corroboration
belongs at the private FCOne supervisor boundary and is not yet part of the
public estimator API.

The future control-qualified path must be a separate probationary gate:

```text
residual persistence + fresh onset + independent corroboration
    -> bounded probation -> private supervisor decision
```

`UNQUALIFIED_PERSISTENT_RESIDUAL` alone must always remain diagnostic-only.

## Validation contract

The focused contract is recorded in
`validation/airspeed_wind_residual_persistence_v8_diagnostic_lane_protocol.json`
and tested by
`tests/test_airspeed_wind_residual_persistence_v8_diagnostic_lane.py`.
It covers stale-boundary persistence, normal fresh-boundary latching, startup
high residuals, interruptions, gaps, invalid data, exact age boundaries,
source-time semantics, and immutable snapshots. The focused suite is a
deterministic contract gate; it is not a statistical train, tune, holdout, or
physical-source qualification.

## Repository boundary

No production ESKF, Mahony, C API, FCOne adapter, selector, or supervisor code
is changed by this experiment. Full traces remain under `build/`; only a
compact, hash-linked summary may be committed under `validation/public/` after
the preflight implementation and protocol are frozen.
