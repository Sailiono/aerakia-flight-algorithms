# v8 residual-persistence policy audit

## Scope and verdict

This is a read-only audit of the independent v8 policy-shape probe introduced
after the failed v7 train. It does not modify, rerun, retune, or reinterpret
v7. The v8 probe is useful as a design instrument: it reproduces the retained
`71101` dead-end and makes the recent-boundary / graded-evidence / bounded-retry
trade-offs observable. It is not yet a v8 protocol, source supervisor,
pitot-fault classifier, wind estimator, or flight-control gate.

The current focused evidence is sound for its declared scope:

- 60 hand-authored policy/scenario comparisons (`3` ages × `5` scenarios ×
  `4` policy shapes);
- 11 focused Python tests, all passing;
- one read-only replay of v7 seed `71101`, with `117` monitor-fed samples;
- no changes to the production ESKF, Mahony, public API, v7 protocol, v7
  scorer, or v7 train artifact.

The focused result supports opening v8 design work, but does not select an
age, decay rate, retry budget, or policy. The follow-up
`partial_quiet_probation` candidate is also diagnostic-only and does not rewrite
the earlier three-policy screen.

## What the probe establishes

1. A bounded recent quiet boundary fixes the v7 logical dead-end on the known
   root-cause shape when the boundary age is at least `2 s` in the toy 2 Hz
   trace. The episode still starts with new high observations; old high
   observations are never inherited.
2. An expired boundary and a source/arrival gap remain fail-closed for every
   candidate shape in the focused matrix.
3. Graded evidence can bridge one high/mid interruption at a shorter boundary
   age, but it is intentionally more permissive on the retry-exhaustion trace.
   This is a sensitivity observation, not a safety or promotion result.
4. The replay of seed `71101` is consistent with the recorded v7 diagnosis:
   v7 does not latch; the unselected probes latch at source `70.5 s` (recent
   boundary and bounded retry) or `70.0 s` (graded evidence). This replay is
   root-cause confirmation only and must not be used for threshold selection.

## Invariants that v8 must retain

The next protocol must preserve these v7 audit protections:

- runtime input contains only source/arrival timestamps, source epoch,
  validity, and finite NIS; no injection labels, truth, or case name;
- strict source and arrival monotonicity;
- source-gap, arrival-gap, invalid-source, missing/invalid-NIS, and epoch-reset
  fail-closed behavior;
- explicit episode onset and a bounded quiet-boundary age in the trigger
  snapshot;
- immutable trigger snapshots and bounded diagnostic traces;
- paired common-random streams and exact null pairing;
- `preexisting_episode_ambiguous` remains a failure for persistent detection,
  not a success credited after injection;
- family-level scoring with a deterministic failure sentinel for missed or
  ambiguous members;
- new disjoint v8 seed ranges, with a protected holdout opened only after
  policy and thresholds are frozen.

## Findings requiring resolution before protocol freeze

### 1. Graded evidence starts at zero at episode onset

The first implementation credited the interval before the first high sample
to graded evidence. That was corrected: `_advance_graded_high()` now initializes
the first high sample with zero evidence and accumulates only subsequent
source-time intervals. A focused sentinel locks this behavior. The eventual
v8 protocol must still define whether mid-band decay is integrated over source
time or observation intervals and must test the choice across supported rates.

### 2. Gap authorization semantics are explicit

The probe now matches the v7 transport contract: a source/arrival gap clears
warmup, quiet, and high evidence but retains the explicitly authorized source
epoch, so a later valid sample may requalify within that epoch. Invalid source
data, missing/invalid NIS, non-monotonic timestamps, and an epoch change still
revoke authorization. Focused tests require a gap to remain fail-closed until
a complete requalification path is present. The v8 protocol must carry this
same distinction into the adapter/recovery metrics rather than leaving it as
an implementation accident.

### 3. Boundary refresh and retry budget need separate semantics

The probe refreshes the boundary on every newly completed quiet run and resets
the relevant retry budget. For the recent-boundary, graded-evidence, and
bounded-retry comparators, a mid-band event preserves a recent boundary while
a high episode abort can retry. The partial candidate intentionally differs:
it retires the boundary after degradation and requires a new quiet run. The v8
protocol must distinguish:

- boundary age at first episode onset;
- age of a retry after an aborted episode;
- number of mid-band interruptions;
- whether a low residual is a genuine new quiet run or merely an episode
  abort.

Every latch record should expose these values so a later result cannot hide
optimistic inheritance behind a single `latched` bit.

### 4. Partial probation must not become a startup shortcut

The partial candidate was tightened before its fresh screen. It now requires a
complete quiet boundary in the current authorized epoch before partial
probation is eligible. A mid-band interruption or aborted high episode retires
the current full/partial boundary; a new high episode cannot directly reuse it.
Only a bounded post-degradation quiet run can create a partial boundary, and a
partial-boundary high episode uses an independent finite retry budget. The
focused suite includes startup, post-mid-band, and retry-exhaustion sentinels.
This is the required fail-closed interpretation; a sensitivity loss is not a
reason to remove these constraints.

## Proposed v8 focused acceptance gate

Run this gate on fresh, pre-registered development seeds only after the
candidate implementation and protocol are committed. Do not call it a train
or holdout.

### A. Hand-authored causal contract (must be 100%)

- root-cause mid-band → persistent-high trace: candidate behavior is recorded
  for each boundary age; no age is selected from this trace alone;
- expired boundary, source gap, arrival gap, invalid sample, missing NIS,
  non-monotonic source/arrival, and epoch reset: no latch before a complete
  requalification path;
- first-high evidence sentinel: no pre-high elapsed time is credited;
- one mid-band pulse with no persistent high: no latch;
- retry exhaustion: finite retry budget is visible and cannot become an
  unlimited v6-style inheritance path;
- repeated runs produce identical trigger hashes and telemetry.

### B. Fresh development screen (suggested initial size)

Use at least `32` fresh seed families before any larger train. Each family
should contain paired nulls and, at minimum:

- nominal and calibrated high-noise controls;
- persistent `+/-` TAS offset, scale, horizontal-wind and vertical-wind
  descriptive cases;
- 0.25/0.5/1.0/1.5/2.0 s pulses and one intermittent high/mid pattern;
- aligned, bounded-jitter, 20 Hz burst, and arrival-only-gap delivery;
- source/arrival gaps, invalid/recovery, epoch reset, and reordered input.

The screen must report per-family and per-member counts, not only aggregate
sample counts.

### C. Provisional error budgets (to be frozen before the screen)

These are review starting points, not approved final limits:

- nuisance false-latch point rate ≤ `1%` and 97.5% one-sided CP upper bound
  ≤ `3%`, retaining v7's safety bar;
- persistent family clean-attribution success ≥ `95%` on the development
  screen, with no pre-injection or ambiguous episode counted as success;
- family P95 clean source-detection delay ≤ `3 s`, with missed/ambiguous
  members mapped to a registered failure sentinel;
- no deterministic contract failures and no unexpected source rejection;
- any sensitivity gain over v7 must be reported alongside nuisance and
  ambiguity changes, never as a standalone latch-rate improvement.

The `95%`/`3 s` values are deliberately development-screen targets. They must
be revisited using the observed error bars and declared mission envelope before
any v8 train or physical-source authority.

## Fresh screen result

The first fresh screen used `32` disjoint families (`74101--74132`), `16`
synthetic cases per family, and `512` replays. It produced zero nominal false
latches and zero structural-gap latches for every candidate. Clean persistent
family passes were `19/32`, `25/32`, and `27/32` for recent-boundary ages `1`,
`2`, and `3 s` respectively for the recent-boundary and bounded-retry shapes;
graded evidence produced `20/32`, `26/32`, and `27/32`. These are materially
better than v7's retained `220/512` family result, but still below the
provisional `95%` target and are not promotion evidence. The full trace is in
`build/airspeed_wind_residual_persistence_v8_screen_74101_32.json`; it is not
committed as a large public trace. A compact non-promoting summary with the
trace SHA-256 and source hashes is retained in
`validation/public/airspeed_wind_residual_persistence_v8_screen_74101_32_summary.json`.

The screen also exposed the attribution boundary rather than hiding it. Seed
`74123` has a high episode beginning before the offline injection boundary and
continuing through it, so a later latch is correctly rejected as clean
post-injection evidence. This case must remain an ambiguity failure unless a
future causal policy can establish a new onset without importing truth.

### Follow-up partial-policy screen

After the safety tightening, a new disjoint screen used seeds `74201--74216`,
the same `16` cases per family, and `256` stream replays. Every candidate had
zero nominal false latches and zero structural-gap latches at ages `1`, `2`,
and `3 s`. Clean persistent-family passes were:

| Age | Recent boundary | Graded evidence | Bounded retry | Partial quiet probation |
| --- | ---: | ---: | ---: | ---: |
| `1 s` | 8/16 | 9/16 | 8/16 | 5/16 |
| `2 s` | 13/16 | 15/16 | 13/16 | 5/16 |
| `3 s` | 15/16 | 16/16 | 15/16 | 5/16 |

The partial candidate's lower sensitivity is expected from the newly enforced
post-degradation quiet requirement and is retained as a negative result. The
best comparator in this small screen is graded evidence at age `3 s`, but it
is not selected or promoted: the screen is not a protected holdout and remains
well short of the provisional evidence plan for a production authority.
The full trace is ignored build data at
`build/airspeed_wind_residual_persistence_v8_screen_74201_16.json`; its compact
summary is committed at
`validation/public/airspeed_wind_residual_persistence_v8_screen_74201_16_summary.json`.

## Recommended selection order

1. Freeze the clarified startup, gap, boundary, and retry semantics in a new
   protocol document and extend focused tests.
2. Compare `recent_boundary` first; it is the smallest change that addresses
   the v7 dead-end.
3. Evaluate `graded_evidence` only with an explicit integration/decay rule and
   rate-invariance tests.
4. Keep `bounded_retry` as a safety comparator; do not promote it merely
   because it improves a single intermittent trace.
5. Freeze one candidate, one parameter set, one new protocol, and disjoint
   train/tune/holdout seeds only after the focused screen meets the registered
   error budgets.

## Partial-quiet probation candidate

After the 32-family screen, a fourth shape was added for a bounded follow-up
diagnostic. It first requires a complete quiet boundary in the current epoch;
only after a mid-band/degraded event can two quiet samples spanning at least
`0.5 s` create a probationary boundary. An episode started from that partial
boundary requires five high observations spanning `2.0 s`, and its retries use
an independent finite budget. Full quiet boundaries retain the ordinary
four-observation/`1.5 s` requirement. The candidate records whether the
boundary and episode onset were partial, so later scoring cannot hide the
distinction behind a single latch bit.

This is a causal hypothesis, not a selected solution. It must be evaluated on
fresh seeds and compared against the same nuisance, ambiguity, gap, and pulse
budgets; no existing screen result is retroactively re-scored.

## Boundary of the conclusion

The v8 probe shows that a bounded recent boundary can make persistent-residual
evidence sensitive again while preserving fail-closed continuity. It does not
show that a TAS source is faulty, that wind is observable, that the ESKF should
gain a wind state, or that any policy is safe to run on FCOne hardware. Those
claims still require the separate physical air-data, same-input, and hardware
validation gates already documented elsewhere.
