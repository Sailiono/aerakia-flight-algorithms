# v8 residual-persistence policy-shape diagnostic

This document records a small design experiment after the **failed v7
development train**. It is deliberately separate from v7: no v7 source,
protocol, scorer, seed range, or committed train artifact is changed or
rerun. The experiment is not a source supervisor, a pitot-fault classifier, a
wind estimator, or flight-control authority.

## Why v8 is needed

V7's result is internally consistent but its sensitivity is limited by a
causal dead-end. At 2 Hz, a `1.5 s` high episode needs four high samples. V7
also clears the quiet boundary on every mid-band residual (`2 <= NIS < 4`).
If a persistent mismatch starts after one mid-band sample and the following
quiet samples cover less than the required fresh quiet interval, the monitor
has no causal path back to `QUIET_CONFIRMED`; the later high samples are
correctly ignored. This explains the retained `71101` failure and is not a
math, pairing, or scorer defect.

The v8 question is therefore:

> Can bounded historical quiet evidence or bounded accumulation improve
> sensitivity without inheriting an unbounded pre-onset episode?

## Independent candidate probe

`validation/diagnose_airspeed_wind_residual_persistence_v8.py` contains a
development-only `CausalPolicyProbe`. It accepts only:

- source and arrival timestamps;
- source epoch;
- source-valid flag; and
- finite, non-negative residual NIS (or an explicit missing value).

Scenario names and injection timestamps are used only after replay for an
offline report. They never enter a policy decision. Every candidate resets on
epoch changes, non-monotonic time, source/arrival gaps, invalid source data,
or missing/invalid NIS.

The four shapes currently exposed by the diagnostic probe are:

| Candidate | Causal rule | Risk to investigate |
| --- | --- | --- |
| `recent_boundary` | Preserve the last confirmed quiet boundary for a bounded age; a contiguous high episode may start only while that boundary is recent. The age is checked at episode onset and then snapshotted for the episode. | A longer age can inherit stale baseline evidence and raise nuisance latches. |
| `graded_evidence` | Integrate high evidence in source time; mid-band evidence decays it; require the high observation count and source-time evidence threshold. A recent boundary is still required at onset. | Decay/threshold choices can turn slowly varying nuisance into a latch; rate and noise invariance must be demonstrated. |
| `bounded_retry` | Permit at most a finite number of high-episode restarts after a mid-band interruption while the same boundary remains recent. Exhaustion requires a fresh quiet boundary. | A retry budget that is too large approximates the v6 optimistic inheritance; one that is too small misses intermittent faults. |
| `partial_quiet_probation` | After one complete quiet boundary in the current epoch, a post-degradation quiet run spanning at least `0.5 s` may create a probationary boundary; an episode started from it must meet a stricter `5`-sample/`2 s` high-evidence requirement. A partial episode gets its own finite retry budget. | The extra post-degradation gate may reduce sensitivity, while a shorter quiet run may still admit nuisance structure; this shape needs independent false-latch and rate-invariance evidence. |

The age snapshot is intentional: a valid episode that starts at age `2.0 s`
must not be rejected merely because its required `1.5 s` evidence completes at
age `3.5 s`. A *new* episode or retry must pass the age check again.

The partial candidate has stricter boundary-generation semantics than the
other three probes:

- it cannot bootstrap the initial qualification; one complete quiet boundary
  must be observed first;
- any mid-band interruption or aborted high episode retires the current full
  boundary, so a subsequent high episode cannot inherit it directly;
- only a bounded post-degradation quiet run can establish a partial boundary;
- a partial-boundary high episode consumes an independent finite retry budget,
  and exhaustion requires new quiet evidence.

These rules are deliberate safety constraints. They are not claims that the
partial candidate is the preferred policy.

## Focused diagnostic matrix

The script runs five hand-authored traces at recent-boundary ages `1, 2, 3 s`.
With the four current shapes this is `60` policy/scenario comparisons:

1. mid-band then persistent high (the v7 root-cause shape);
2. expired boundary followed by high residuals;
3. one intermittent high/mid interruption;
4. source-gap then high residuals; and
5. retry-budget exhaustion.

The current, intentionally unselected probe output is:

| Trace | Age 1 s | Age 2 s | Age 3 s | Interpretation |
| --- | --- | --- | --- | --- |
| Mid-band → persistent high | partial only | all non-partial reject | all four latch | The probationary shape uses a 0.5 s partial quiet run and a stricter 2 s high episode. |
| Expired boundary | all reject | all reject | all reject | No candidate accepts stale evidence beyond its age. |
| Intermittent high/mid | graded only | all non-partial | all four | Graded evidence bridges one interruption; this is a sensitivity result, not a promotion result. |
| Gap → high | all reject | all reject | all reject | Continuity remains fail-closed. |
| Retry exhaustion | bounded retry rejects | bounded retry rejects | partial/recent/graded may latch at the longest age | The finite retry budget is observable; permissive shapes remain diagnostic only. |

As a read-only replay of the already retained v7 seed `71101` (one case,
`117` monitor-fed observations), v7 remains unlatched. The unselected probe at
age `3 s` latches the same residual stream with:

- `recent_boundary`: source `70.5 s`, offline delay `1.5 s`;
- `bounded_retry`: source `70.5 s`, offline delay `1.5 s`;
- `graded_evidence`: source `70.0 s`, offline delay `1.0 s`.

This single replay is root-cause confirmation only. It must not be used to
choose a threshold, claim a false-latch rate, or replace a v8 train.

## Required next gate before any v8 train

No candidate is selected yet. Before freezing a v8 protocol, add a new,
disjoint seed registry and pre-register:

- nominal and calibrated high-noise nuisance families;
- persistent positive/negative offsets, scale errors, wind/model mismatch;
- pulses and intermittent high/mid patterns;
- source-time and arrival-time jitter/burst profiles;
- source/arrival gaps, invalid samples, epoch resets, and reordered data;
- episode-onset attribution and pre-existing-episode ambiguity;
- family-level false-latch, miss, delay, and age-sensitivity endpoints; and
- a separate holdout that is not inspected during policy selection.

The first v8 run should be a small development screen on fresh seeds, not a
large train. A candidate can proceed only if it improves the v7 miss mechanism
without exceeding a pre-registered nuisance/error budget. Physical calibrated
TAS/GNSS/regime evidence remains necessary before any private FCOne source
authority or estimator-state promotion.

## Reproduction

```bash
python3 -m unittest tests.test_airspeed_wind_residual_persistence_v8_diagnostics -v

# Tiny semantic matrix; output is ignored build data, not validation/public evidence.
python3 validation/diagnose_airspeed_wind_residual_persistence_v8.py \
  --boundary-ages 1,2,3 \
  --out build/airspeed_wind_residual_persistence_v8_policy_shapes.json

# Optional read-only replay of one retained v7 failure (about one stream only).
python3 validation/diagnose_airspeed_wind_residual_persistence_v8.py \
  --replay-v7-seed 71101 \
  --replay-v7-case persistent_tas_offset_positive \
  --boundary-ages 1,2,3 \
  --out build/airspeed_wind_residual_persistence_v8_replay_71101.json
```

The script writes no file under `validation/public/` and never invokes the v7
train/tune entry point.

## Fresh development screen (not a train)

After the focused semantic fixes, a fresh screen used seeds `74101--74132`
(`32` families, disjoint from all registered v7 ranges), `16` case names, and
`512` stream replays. The run used the frozen v7 synthetic oracle only as a
host-side NIS provider and wrote its full trace to
`build/airspeed_wind_residual_persistence_v8_screen_74101_32.json`.

No candidate produced a nominal false latch (`0/32`) or a structural-gap latch
(`0/32`); the calibrated-high-noise nominal case was geometry-unqualified in
all 32 families and is therefore descriptive only. Persistent clean-family
results were:

| Recent-boundary age | Recent boundary | Graded evidence | Bounded retry |
| --- | ---: | ---: | ---: |
| `1 s` | 19/32 | 20/32 | 19/32 |
| `2 s` | 25/32 | 26/32 | 25/32 |
| `3 s` | 27/32 | 27/32 | 27/32 |

The clean-family gate requires all four persistent offset members (positive,
negative, and bounded-jitter variants) to latch after injection, with no
pre-existing episode. These results are materially better than the retained
v7 train failure, but they are not a pass for the provisional `>=95%` screen
target. In particular, seed `74123` is an intentional ambiguity case: a high
episode begins before the offline injection boundary and continues through it;
counting the later latch as a clean detection would violate the v7 attribution
rule. This is a limitation of causal evidence, not a reason to relax scoring.

Pulse sensitivity also separates the shapes: at age `3 s`, the `1.0 s` pulse
latched `0/32` for every policy, while the `1.5 s` pulse latched `1/32` and
the `2.0 s` pulse latched `28/32`. Descriptive scale/wind latches are retained
per case in the screen JSON but are not treated as fault-classification
evidence. No age or policy is selected from this screen; the next step is to
design a causal partial-quiet/probation policy and a fresh pre-registered v8
protocol, or to accept the observed attribution boundary explicitly.

A compact machine-readable summary (without the large per-event trace) is
retained at
`validation/public/airspeed_wind_residual_persistence_v8_screen_74101_32_summary.json`.
It is explicitly non-promoting and records the full-trace SHA-256 plus the
source-file hashes needed to identify this run.

### Follow-up screen after partial-policy safety fixes

The partial candidate was then screened on a new disjoint range,
`74201--74216` (`16` families, the same `16` cases per family, `256` replays).
The run was performed after adding the startup prohibition, mandatory
post-degradation quiet evidence, and an independent finite partial retry
budget. It did not modify or rerun v7.

All candidates produced `0/16` nominal false latches and `0/16` structural-gap
latches at every tested age. Clean persistent-family passes were:

| Recent-boundary age | Recent boundary | Graded evidence | Bounded retry | Partial quiet probation |
| --- | ---: | ---: | ---: | ---: |
| `1 s` | 8/16 | 9/16 | 8/16 | 5/16 |
| `2 s` | 13/16 | 15/16 | 13/16 | 5/16 |
| `3 s` | 15/16 | 16/16 | 15/16 | 5/16 |

This is still a development screen, not a promotion result. The safety fix
made the partial candidate materially less sensitive; it is therefore rejected
for the next v8 protocol rather than being rescued by relaxing its rules. The
best provisional comparator is `graded_evidence` at age `3 s`, but it remains
below the required evidence standard because this screen is small and not a
protected holdout. The full trace remains under
`build/airspeed_wind_residual_persistence_v8_screen_74201_16.json`; the compact
non-promoting summary is
`validation/public/airspeed_wind_residual_persistence_v8_screen_74201_16_summary.json`.
