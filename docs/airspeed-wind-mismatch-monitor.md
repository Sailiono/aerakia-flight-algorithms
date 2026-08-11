# TAS residual-pattern monitor study

## Why this study exists

The frozen multi-seed TAS/wind confirmation retained one unsafe qualified
vertical-wind case. The existing v1 source latch only counted consecutive
individual NIS rejections, so a low individual residual could reset its count
even while the short residual history remained concerning.

This study explores a narrower question than wind estimation: can a causal,
short-window residual screen distinguish a one-off source spike from a
*persisting* source/model mismatch after the existing geometry bootstrap? It
does not modify the production ESKF or public API.

## Rejected v1 development prototype

The first uncommitted v1 prototype must **not** be interpreted as passing
evidence. Its 32-seed nominal control had only three post-qualification NIS
samples per replication, while its monitor required four samples. It therefore
formed zero complete nominal windows; `32/32 not latched` meant “the monitor
could not run”, not “zero false latches”. Conversely, a mismatch case could
latch on its first high residual plus three historical nominal residuals. It
did not establish persistent-pattern detection.

That review finding is retained because it changes the protocol, not merely a
threshold: the v1 result never enters `validation/public/`, does not affect
the current 15-state ESKF, and cannot support TAS/wind promotion.

## v2 opened development protocol

[`airspeed_wind_mismatch_monitor_protocol_v2.json`](../validation/airspeed_wind_mismatch_monitor_protocol_v2.json)
starts every replication with the frozen multi-heading source stream, then
requires at least 40 post-qualification clean observations and 37 complete
monitor windows before an injection is allowed. It records those counts for
every replication and fails closed if they are missing.

The monitor is terminal and has deliberately explicit time semantics:

- four observations must fit inside `1.75 s`;
- an inter-observation gap above `1.0 s` clears unfinished history;
- every individual contribution is capped at NIS `6.0`;
- a latch needs cumulative clipped NIS `>= 18.0` **and** at least three raw
  contributions `>= 4.0`.

These values are a development design choice, not a calibrated physical
false-alarm probability. The v2 matrix separately exercises long nominal and
high-noise nominal controls, a single TAS impulse, a gap followed by an
impulse, and persistent TAS scale, horizontal-wind-step, and vertical-wind
mismatches. A latch in a persistent case also needs at least two injected
samples in its triggering window and must occur within the declared deadline.

The first v2 parameter set was rejected on this opened window: it produced
nominal and single-impulse latches. Its retained development result was
`170/224` passing cases. The second opened v2 setting requires three high
contributors and caps a single contribution at `6.0`. It passes all `224/224`
development replications: 30,528 source observations and 17,723 complete
monitor windows. Each of the four non-latch controls stayed unlatched in all
32 development seeds; each persistent mismatch latched in all 32, with a
triggering window containing at least two injected samples.

This is a selection result, not a holdout claim. An intended v3 holdout was
invalidated before release when its first seed (`42001`) was used in an
implementation smoke run. Its `7/7` output is discarded rather than used for
selection; no parameter changed in response. The fresh, previously unexecuted
`43001--43128` set is instead frozen in
[`airspeed_wind_mismatch_monitor_protocol_v4.json`](../validation/airspeed_wind_mismatch_monitor_protocol_v4.json).
That protocol locks the base JSON, base oracle/generator, candidate runner,
parameters, cases, and seeds. Its run requires a clean Git worktree and cannot
use a development seed set.

## v4 sealed-holdout result

The v4 protocol was executed exactly once from clean commit `f085488`, using
four strict raw-record shards and one fail-closed merge. The result is retained
at [`airspeed_wind_mismatch_monitor_v4.json`](../validation/public/airspeed_wind_mismatch_monitor_v4.json).
It is a **failed holdout**, not a tuning input:

- `889/896` replication cases passed (`99.21875%`); `7` failed.
- The campaign processed `122,112` source observations and `70,917` complete
  monitor windows.
- The two nominal/non-persistent controls were mostly clean, but a single TAS
  impulse produced two false latches (seeds `43033`, `43108`).
- Persistent TAS scale bias latched before injection in one case (seed `43061`).
- A horizontal-wind step latched from only one injected sample (seed `43067`).
- Vertical-wind cases had one early latch and two late latches (seeds `43074`,
  `43065`, `43102`).

The monitor therefore **does not qualify as a source supervisor**. Do not
rerun, retune, or reinterpret v4; any correction requires a new development
protocol and a new, separately sealed holdout. The failures point to three
development questions: whether one large residual still dominates the short
window, whether low-coverage/bootstrap conditions must disable the screen, and
whether the vertical-mismatch deadline is physically justified. None of these
questions changes the production ESKF, its 15-error-state model, or the public
TAS API.

## v5 opened structural diagnostic

The separately named v5 protocol uses development seeds `52001--52064` and
common-random-number prefixes across paired controls. It replaces the rolling
window with a consecutive-evidence episode: low residuals, source-time gaps,
and excessive episode span clear unfinished evidence. Four high residuals over
at least `1.5 s` are required. The candidate, protocol, and process-pool
execution path were committed before the full run at source commit `06168b8`.

The 2026-08-11 matrix executed 640 replications and 87,744 observations. Its
compact reviewed record is
[`airspeed_wind_mismatch_monitor_v5_development.json`](../validation/public/airspeed_wind_mismatch_monitor_v5_development.json).
The raw scorer reported 71 required failures, which review separates into two
classes:

- 62 were an instrumentation-contract defect in the gap control. All 64
  streams contained the declared gap and all 64 stayed unlatched, but the
  counter recorded a gap only when it cleared a non-empty episode. The scorer
  incorrectly treated an empty-episode gap as an absent gap.
- nine were genuine candidate-behavior failures. One two-impulse and four
  three-impulse streams latched. Two four-impulse and two persistent-bias
  streams latched using only two or three injected samples. Seeds `52038` and
  `52061` repeatedly exposed the same cause: a consecutive nominal high run
  immediately before injection was extended by new injected highs.

Long nominal, high-noise nominal, and single-impulse controls stayed unlatched
in `64/64` replications each. Persistent TAS bias latched in `64/64`, but the
two contaminated onset windows make v5 non-promotable. The descriptive
horizontal-wind stress latched `64/64`; vertical wind latched `54/64`, with
P50/P95 delay `3.0/8.0 s`. These stress results demonstrate residual response,
not source attribution or vertical-wind observability.

V5 is rejected. Its structural improvement removes the v4 arbitrary rolling
window, but “consecutive” alone does not define a causal episode onset. A next
opened protocol must separate observed gaps from actual episode clears,
preserve the latch-trigger trace after terminal latch, and test an explicit
onset-boundary mechanism. No new holdout is allowed until that structure is
stable, and no synthetic result removes the need for physical calibrated
TAS/GNSS/regime evidence.

## v6 source-time persistence characterization

V6 was opened as a new development protocol rather than a repair of v4 or v5.
It changed the monitor to source-time persistence with explicit warmup and
quiet confirmation, separate source/arrival gap telemetry, immutable trigger
snapshots, and statistical family-level scoring. The production ESKF, Mahony
backup, public API, and wind-state boundary were unchanged.

The train run used seeds `62001--62128`, 14 cases, and 1,792 complete
replications. It processed 290,304 generated observations and 194,688
observations delivered to the residual monitor. The deterministic contracts
(common-random pairing, gap/coverage accounting, pulse monotonicity, and
immutable trigger snapshots) all passed. The compact result is retained at
[`airspeed_wind_residual_persistence_v6_train.json`](../validation/public/airspeed_wind_residual_persistence_v6_train.json),
from commit `b83f30c` (runner/monitor candidate commit `2789bab`).

The primary statistical results were mixed:

- The nuisance family had zero false latches in 128 seed families. Its 97.5%
  one-sided Clopper–Pearson upper bound was `2.8408%`, below the registered
  `3%` development limit.
- Both signs of the persistent `±2 m/s` TAS offset latched in all 128 seed
  families, but the family P95 source-time detection delay was `3.5 s`, above
  the registered `3.0 s` limit. Therefore the persistent endpoint failed and
  the v6 train status is `failed_train_development_checks`.

V6 tune was deliberately not run. Review after the train identified four
additional audit blockers that must be fixed in a new protocol, not retrofitted
into this result: an episode can begin before the injection and latch after it
while being scored as a clean detection; the one-shot tune gate was not
enforced by the runner; the delay P95 flattened two correlated signs instead
of aggregating one worst delay per seed family; and the residual-provider
override was not fully declared in the protocol. The campaign also used equal
source and arrival timestamps, so its statistical result does not validate
arrival jitter, burst delivery, or arrival-only gaps.

V6 is consequently a retained synthetic diagnostic, not a source supervisor,
TAS fault classifier, wind estimator qualification, or flight-control gate.
The next iteration must be named v7, use disjoint seeds, record
`episode_start_source_timestamp_us`, classify pre-existing episodes as
ambiguous rather than successful detections, aggregate delay at the seed-family
level, declare the residual-provider configuration, and enforce a committed
single-use tune receipt. No result from v6 authorizes a 17-state branch or
hardware flight.

## v7 implementation preflight (before train)

V7 is a new, unexecuted development protocol. It does not modify or overwrite
any v4/v5/v6 file or result. Its preflight implementation now has 26 focused
tests and a full host-suite compatibility check (`274` tests, with only the
pre-existing optional PX4-source checks reported as blocked).

The v7 candidate adds three audit protections before any large run:

- the monitor records source/arrival episode onset and requires a fresh quiet
  boundary; a mid-band residual or aborted high episode cannot inherit old
  evidence;
- the evaluator configuration, override, event ordering, and geometry-gate
  semantics are explicit and hashed;
- persistent delay is scored per complete seed family with a fixed ceil-based
  order statistic and a failure sentinel, while a durable, atomic tune-start
  receipt prevents concurrent or ordinary repeated tune execution.
- every injected performance case must resolve exactly one delivery-matched
  null during protocol loading. The one structural source-gap contract case is
  explicitly marked `paired_null_required: false`, because it validates
  fail-closed transport behavior rather than differential detection latency.

The primary preflight matrix includes aligned and bounded-jitter 2 Hz delivery
profiles with source-noise pairing preserved. A true 20 Hz burst/arrival-only
gap campaign is intentionally not claimed yet: it requires a separate source
rate and delivery generator and remains a v8/future robustness item rather than
being represented by a misleading 2 Hz approximation.

V7 train seeds are `71101--71612` (512 families); tune seeds are
`72101--73124` (1,024 families). Seed `71001` was used only for a preflight
smoke and is permanently retired in the protocol registry.

## v7 train result (failed development evidence)

The v7 train was executed once from clean commit `023a0a4` with eight workers.
It completed all 512 seed families and 17 registered cases: 8,704 complete
replays, 1,410,048 generated observations, and 958,464 NIS-fed observations.
The immutable result is retained at
[`airspeed_wind_residual_persistence_v7_train.json`](../validation/public/airspeed_wind_residual_persistence_v7_train.json)
(SHA-256 `42e933461b9fdae3f38ec6787133193fc79ccb3a02be4e8714799760b5471dfc`).

The result cleanly separates the two primary outcomes:

- Nuisance false latches: `3/512 = 0.586%`; the 97.5% one-sided
  Clopper--Pearson upper bound is `1.703%`, below the registered `3%` limit.
- Persistent `±2 m/s` TAS-offset family: `292/512 = 57.031%` failures; the
  97.5% upper bound is `61.365%` and the family P95 is the registered failure
  sentinel `6.001 s`, not the `3.0 s` target. Only 220 complete seed families
  satisfied all four sign/delivery members.

The deterministic common-random pairing, gap/coverage trace, and pulse
monotonicity contracts all passed. Aligned and bounded-jitter variants had the
same persistent latch counts (positive `264/512`, negative `277/512` clean
post-injection latches), so this limited 2 Hz jitter model did not explain the
failure. The retained records instead show that the currently conservative
quiet/high persistence rule often never forms a qualifying high episode for a
`±2 m/s` step; 11--14 cases per sign were also correctly marked as
pre-existing/ambiguous rather than credited after the injection. This is a
monitor-characterization result, not evidence that the production ESKF or
Mahony estimator failed.

The high-noise descriptive case never became geometry-qualified and fed zero
residuals. It is retained as a coverage observation, not counted as evidence
of high-noise detection performance. The true 20 Hz burst and arrival-only-gap
matrix remains unimplemented and unclaimed.

V7 tune is prohibited because train did not pass. The next iteration must use
a new v8 protocol and disjoint seed ranges; v7 data may be analyzed, but its
thresholds, code, or train artifact must not be rewritten or rerun.

## Boundary

Even a clean v4 holdout pass would mean only that this frozen *synthetic*
screen exercised the stated monitor behavior. The actual v4 result failed, and
even a pass could not identify vertical wind, qualify a pitot, infer source quality, enable TAS during hover or
transition, or prove fixed-wing GNSS-denied navigation. Physical calibrated
TAS/GNSS data and a separately justified private source/regime authority still
precede any 17-error-state wind experiment.

## Reproduction

```bash
python3 -m unittest tests.test_airspeed_wind_mismatch_monitor -v
python3 -m unittest tests.test_airspeed_wind_mismatch_monitor_v5 -v
python3 validation/run_airspeed_wind_mismatch_monitor.py --phase development --jobs 1
python3 validation/run_airspeed_wind_mismatch_monitor_v5.py --jobs 8
# v6 train (retained failed development evidence; do not run v6 tune):
python3 -m unittest tests.test_airspeed_wind_residual_persistence_v6 -v
python3 validation/run_airspeed_wind_residual_persistence_v6.py \
  --phase train --jobs 8
# After the code, tests, v2 development record, and v4 protocol are committed:
python3 validation/run_airspeed_wind_mismatch_monitor.py \
  --protocol validation/airspeed_wind_mismatch_monitor_protocol_v4.json \
  --phase sealed_holdout --jobs 8
```

The currently committed v4 result is expected to return exit status `1`,
because the sealed campaign failed. That status is evidence, not a runner
error.

`--jobs 1` is the strictly sequential reproduction path and is required on
restricted runners that do not allow child-process creation; a normal
workstation may use more jobs without changing individual seed cases.

If a restricted session cannot keep the complete campaign alive, split the
already frozen v4 list only into explicit contiguous shards using
`--seed-start-index`, `--seed-count`, and `--emit-records`, writing each raw
record shard under `build/`. Then use the same frozen runner's `--merge-shards`
mode once. It rejects a dirty source tree, an altered runner/protocol, mixed
commits, overlapping/missing seeds, or any shard that lacks exactly one record
per case/seed; the final compact artifact omits raw records. Sharding is
execution transport, not a second opportunity to inspect a partial outcome or
change v4 parameters.

Only the clean, compact sealed result may be labelled a holdout and enter
`validation/public/`; the v4 default is
`validation/public/airspeed_wind_mismatch_monitor_v4.json`, so it cannot
overwrite an earlier protocol's result. A failed result remains in the public
record with `status: failed`; it must not be replaced by a later tuned run.
A separately named compact v2
development summary may also be retained there only with its `phase` and
non-holdout status intact. Individual streams and other temporary artifacts
remain under `build/` and are removed after review.
