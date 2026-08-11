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

## Boundary

Even a clean v2 holdout pass would mean only that this frozen *synthetic*
screen exercised the stated monitor behavior. It cannot identify vertical
wind, qualify a pitot, infer source quality, enable TAS during hover or
transition, or prove fixed-wing GNSS-denied navigation. Physical calibrated
TAS/GNSS data and a separately justified private source/regime authority still
precede any 17-error-state wind experiment.

## Reproduction

```bash
python3 -m unittest tests.test_airspeed_wind_mismatch_monitor -v
python3 validation/run_airspeed_wind_mismatch_monitor.py --phase development --jobs 1
# After the code, tests, v2 development record, and v4 protocol are committed:
python3 validation/run_airspeed_wind_mismatch_monitor.py \
  --protocol validation/airspeed_wind_mismatch_monitor_protocol_v4.json \
  --phase sealed_holdout --jobs 8
```

`--jobs 1` is the strictly sequential reproduction path and is required on
restricted runners that do not allow child-process creation; a normal
workstation may use more jobs without changing individual seed cases.

Only the clean, compact sealed result may be labelled a holdout and enter
`validation/public/`; the v4 default is
`validation/public/airspeed_wind_mismatch_monitor_v4.json`, so it cannot
overwrite an earlier protocol's result. A separately named compact v2
development summary may also be retained there only with its `phase` and
non-holdout status intact. Individual streams and other temporary artifacts
remain under `build/` and are removed after review.
