# G0 bias-observability paired A/B comparison

This document defines how a candidate estimator change is compared with the
frozen baseline for the G0 horizontal accelerometer-bias observability work.
The comparator consumes completed campaign summaries; it does not regenerate
inputs, rerun the estimator, or retune acceptance thresholds after seeing the
candidate results.

## Scope and trial pairing

G0 v1 has two usable development splits:

- `train`: two trajectories, 16 seeds, and nine declared bias vectors;
- `tune`: one trajectory, 32 seeds, and nine declared bias vectors.

That is `2 × 16 × 9 + 1 × 32 × 9 = 576` trials. The comparison key is the
exact tuple `(split, trajectory_id, bias_vector_id, seed)`. Baseline and
candidate must contain the same set of keys; missing, duplicate, extra, or
unexpected keys fail the comparison before acceptance is considered. This
prevents a candidate from appearing better by dropping difficult cases.

The v1 holdout is deliberately not opened here. Earlier smoke execution used
holdout trajectory families and seed `30000`, so that holdout is diagnostic,
not blind release evidence. The replacement sealed-holdout design is described
in `validation/bias_observability_protocol_v2_plan.json`; its concrete manifest
must remain outside the public working tree until the baseline, candidate,
thresholds, and regression policy are frozen.

## Integrity and provenance gates

Before interpreting a metric delta, the comparator checks:

1. equal protocol semantic SHA-256;
2. immutable 40-hex source commit for each campaign;
3. recorded clean source trees;
4. passed campaign execution with no unrecorded trial failures;
5. identical generator and campaign-runner SHA-256 values;
6. identical analyzer SHA-256 values (or a hash derived from the declared
   analyzer command path);
7. equal truth-boundary and campaign-mode declarations;
8. an input SHA-256 for every paired trial, with baseline and candidate values
   equal and, when an `input.csv` is present, matching the file bytes.

An integrity failure is not the same thing as an estimator trial failure. The
report retains all paired diagnostics, but the final status remains failed so
that an unsealed campaign cannot be promoted to release evidence.

## Metrics and signs

For ordinary error or count metrics, lower is better. For `healthy_ratio`,
higher is better. NIS and NEES are consistency metrics: neither a very small
nor a very large value is automatically good. The policy declares theoretical
means of 3 for position/velocity NIS, 6 for navigation NEES, and 5 for the
five-dimensional tilt/accelerometer-bias joint NEES. The comparator scores a
value by its absolute distance from that declared mean; a negative
candidate-minus-baseline score delta is an improvement.

The primary G0 metric is terminal horizontal-bias error P95. It must improve
by the predeclared minimum in both `train` and `tune`. Nonzero bias groups are
also checked for the required improvement fraction and maximum regression
count. Zero-bias pass-to-fail transitions, material regressions, global
regressions, newly failing groups, and worsening signed mirror asymmetry are
explicit failures.

## Right-censored settling times

If a trial does not settle within its declared observation window, its settling
time is right-censored rather than a numeric value. The comparator reports four
categorical transitions:

- `settled_to_settled`;
- `settled_to_censored`;
- `censored_to_settled`;
- `censored_to_censored`.

Settling-time deltas are calculated only for settled-to-settled pairs. A
candidate that changes a trial from censored to settled receives credit in the
categorical transition summary and can then contribute its numeric settling
time; it is not assigned an artificial timeout value.

## Five-dimensional joint NEES evidence

The required candidate metric `tilt_accel_bias_joint_nees_mean` is read from
the sibling per-trial `metrics.json` artifact under
`bias_estimation.tilt_accel_bias_joint_nees`. The summary must also report the
invalid-covariance sample count. Missing metrics are explicit failures rather
than silently treated as zero. This joint NEES is a consistency diagnostic for
the coupled tilt and horizontal accelerometer-bias evidence; it is not an
absolute accuracy score and does not replace independent attitude or motion
truth.

## Current 2026-07-20 campaign boundary

The available baseline and candidate campaigns each contain all 576 train/tune
trials and pair exactly. Their metric deltas are useful diagnostic evidence:

- nonzero-bias terminal horizontal P95 improves by about `0.00063 m/s²` on
  train and `0.00605 m/s²` on tune;
- attitude RMSE improves modestly (about `0.011°` train and `0.043°` tune);
- position RMSE is approximately unchanged;
- candidate tune has fewer right-censored trials;
- the joint NEES values are below the theoretical mean in both campaigns, with
  candidate consistency distance smaller in the sampled comparison;
- invalid joint-NEES covariance samples remain zero.

These numbers must be read with the integrity boundary: both campaigns were
generated from dirty source trees, and their compact trial summaries do not
carry input SHA-256 values or retain `input.csv` in each trial directory. The
comparison therefore reports missing input hashes and dirty provenance and
must fail as release evidence. It is a paired diagnostic result, not a blind
holdout claim, and it does not prove hardware-level flight readiness.

## Reproduction

```bash
python3 validation/compare_bias_observability_campaigns.py \
  --baseline build/g0-ab-20260720-baseline/summary.json \
  --candidate build/g0-ab-20260720-candidate/summary.json \
  --policy validation/bias_observability_ab_policy_v1.json \
  --out-dir build/g0-ab-20260720-comparison
```

The command writes `summary.json`, `report.md`, and `paired_trials.csv`.
For the current campaign, a non-zero exit is expected because the provenance
and input-hash integrity gates are intentionally doing their job.
