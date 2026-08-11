# TAS/wind multi-seed confirmation

## Purpose

The first v1 source-contract oracle is a deterministic boundary check. This
separate campaign increases noise coverage without turning its seed into a
hidden tuning variable. It repeats the unchanged v1 case rules across two
disjoint confirmation seed windows:

- `replication_a`: seeds `17001` through `17016`;
- `replication_b`: seeds `27001` through `27016`.

Each window is split into two independently reproducible eight-seed execution
shards. Sharding changes neither case nor seed coverage; it makes constrained
workstations resumable. A merge refuses missing or overlapping records.

Every seed executes every one of the fifteen frozen cases. The campaign has no
train, tune, or estimator-parameter phase: any failed replication is retained
as a failure; no threshold, source contract, scenario, or seed is widened or
replaced based on the observed result.

## What is measured

The campaign retains compact per-scenario aggregates, total source-observation
count, accepted/rejected event count, terminal-status counts, positive-case
wind-error and known-wind NIS distributions, failed-case records if any, and a
canonical SHA-256 digest of all individual replication records. Full
per-sample streams, records, build outputs, and plots remain disposable under
the [workspace artifact policy](workspace-artifact-policy.md).

Failures are additionally classified without being suppressed: an ending
`qualified` negative control is an `unsafe_false_qualification`; a final latch
without a subsequent `source_latched` event is an event-order evidence gap; a
stale ending that misses a required latch is a latch-completeness gap. These
labels are diagnostic summaries, not a way to waive the frozen case rule.

The base source protocol hash is frozen in
[`validation/airspeed_wind_observability_campaign_v1.json`](../validation/airspeed_wind_observability_campaign_v1.json).
The runner rejects any drift before a campaign starts.

## Reproduction and interpretation

```bash
python3 validation/run_airspeed_wind_observability_campaign.py --jobs 4
python3 -m unittest tests.test_airspeed_wind_observability_campaign -v
```

On a constrained workstation, create four disposable shard files under
`build/`, then merge them into one compact public summary:

```bash
python3 validation/run_airspeed_wind_observability_campaign.py \
  --split replication_a_1 --include-records --out build/airspeed-wind-a1.json
python3 validation/run_airspeed_wind_observability_campaign.py \
  --split replication_a_2 --include-records --out build/airspeed-wind-a2.json
python3 validation/run_airspeed_wind_observability_campaign.py \
  --split replication_b_1 --include-records --out build/airspeed-wind-b1.json
python3 validation/run_airspeed_wind_observability_campaign.py \
  --split replication_b_2 --include-records --out build/airspeed-wind-b2.json
python3 validation/run_airspeed_wind_observability_campaign.py \
  --merge-shard build/airspeed-wind-a1.json \
  --merge-shard build/airspeed-wind-a2.json \
  --merge-shard build/airspeed-wind-b1.json \
  --merge-shard build/airspeed-wind-b2.json
```

Passing this campaign means the same synthetic qualification/rejection behavior
replicated across declared noise seeds. It does not provide physical TAS,
wind, calibration, aerodynamic-model, target-compute, or GNSS-denied
navigation evidence. A physical fixed-wing train/tune/holdout comparison is
still required before a 17-error-state wind branch can be evaluated.
