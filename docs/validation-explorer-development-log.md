# Validation Explorer Development Log

This log records changes to the local validation workstation. It is separate
from flight qualification evidence and must not be used to claim hardware or
flight readiness.

## 2026-08-07: data catalog and coverage view

### Input

- Algorithm repository: current working tree on `feature/g0-correlated-static-prior`.
- Independent-truth curves: EuRoC `MH_01_easy` and `V1_03_difficult`, each with
  raw-IMU and reference-bias diagnostic tracks from
  `build/public-dataset-suite-final/`.
- Catalog summaries: EuRoC, Blackbird, UrbanNav, electrical UAV, INSANE,
  IDF-DS, the 40,000-trial Monte Carlo campaign, and the input-integrity
  campaign.

### Implementation

- Added `reports/aerakia-optimization-explorer/scripts/build-validation-data.mjs`.
- The generator uses repository-relative paths, checks required result columns,
  finite numeric values, strictly increasing timestamps, before/after sample
  alignment, and source/result SHA-256 hashes.
- Added `public/data/validation-data.json` with evidence grade, reference kind,
  sample volume, coverage highlights, limitations, algorithm commit, and curve
  samples.
- Replaced the starter test with data-integrity and workstation-source tests.
- Added a coverage matrix beside the interactive geodesic/Euler curve view.

### Current observed result

- Catalog: 10 entries and 8,623,072 recorded/attempted samples as represented
  by the source summaries; the number is coverage volume, not an accuracy
  denominator.
- Independent physical-truth entries: 3 (EuRoC MH_01, EuRoC V1_03, Blackbird).
- Physical heading entries: 2 (INSANE indoor local magnetic datum and outdoor
  dual-RTK input). They remain separate evidence classes.
- Synthetic Monte Carlo: 1,000 completed trials in the selected 40,000-seed
  campaign family summary; this is not 40,000 completed trials.
- Curves: 2 EuRoC paired tracks with 20,932 and 36,381 samples respectively.

### Interpretation and limits

- The workstation improves auditability and triage; it does not improve the
  estimator by itself.
- Reference-bias tracks isolate propagation/update behavior and are not online
  bias-observability evidence.
- PX4/IDF-DS entries are engineering-reference and stress evidence, not
  independent attitude truth.
- UrbanNav includes a 131 s position gap and exposes a navigation-consistency
  issue; it must not be pooled with EuRoC attitude RMSE.
- INSANE indoor results retain the negative magnetometer-on A/B result; the
  viewer must not summarize it as a successful absolute-heading qualification.

### Reproduction

From `reports/aerakia-optimization-explorer`:

```bash
npm run data:build
npm test
```

The generated artifact is tied to the current algorithm commit and records the
hashes of every input summary and curve source. Raw datasets remain outside the
frontend and are restored/downloaded separately.

## Next planned workstation increments

1. Add replay-run selection for ULog, UrbanNav, and fault-injection outputs
   using the same schema and evidence labels.
2. Add downloadable per-case JSON/CSV subsets and a Markdown report generated
   from the exact selection.
3. Add navigation/outage and NIS/NEES time-series plots once those result
   channels are included in a compact curve artifact.
4. Add desktop/mobile browser QA and an export path before any hosted release.
