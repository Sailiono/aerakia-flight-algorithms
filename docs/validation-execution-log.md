# Validation execution log

This append-only log records why each hardware-independent validation step was started, what was
changed, which evidence was produced, and what remains unresolved. Generated raw outputs stay under
`build/`; reviewed conclusions and reproduction commands remain in Git.

## 2026-07-18 — input integrity and timing audit started

### Reason

FCOne v2 hardware is not yet available. The highest-value remaining PC-only work is to make every
input failure mode explicit before the public estimator is connected to a private driver/task
graph. Completion is based on coverage, sample count, retained failures, and repeatability rather
than one successful demonstration.

### Baseline inspected

- Branch: `feature/hardware-free-validation-roadmap`.
- Starting commit: `06d34b4`.
- Existing host gate: strict C build, C/Python tests, deterministic scenarios, reviewed thresholds,
  and a 20-seed navigation Monte Carlo campaign.
- Existing data evidence: 24 private PX4 ULogs plus EuRoC `MH_01_easy` and
  `V1_03_difficult` external-reference sequences.

### Findings

1. Mahony and the ESKF adapter reject missing/non-finite required IMU vectors.
2. Duplicate and reversed IMU timestamps are rejected before state propagation.
3. IMU intervals outside configured minimum/maximum bounds return
   `AERAKIA_STATUS_TIMESTAMP_ERROR`; an excessive forward gap deliberately re-anchors the timestamp
   without propagating across the gap.
4. Sparse magnetometer validity is represented in the IMU sample flags.
5. GNSS position/velocity, trusted heading, and barometer calls do not currently carry physical
   observation timestamps. The algorithm layer therefore cannot distinguish fresh aiding from a
   delayed, duplicate, reordered, or future observation.
6. The CSV validation runner counts syntactically malformed rows but does not exercise public-API
   status behavior for every malformed value. API-level deterministic tests and a bulk fault
   campaign are both required.

### Decision

The closure is split into two reviewed parts:

1. Complete IMU timestamp/value/flag fault handling with deterministic state-invariance checks and
   a multi-seed bulk campaign.
2. Add an explicit timestamped aiding-observation contract before FCOne integration, while keeping
   any compatibility wrappers clearly marked as unable to validate freshness.

The detailed matrix and acceptance rules are maintained in
[`input-integrity-validation.md`](input-integrity-validation.md).

## 2026-07-18 — input integrity and aiding freshness implemented

### Changes

- Corrected minimum-interval handling so rejected early IMU samples do not advance estimator time.
- Kept the documented large-forward-gap behavior: reject propagation, re-anchor time, and resume on
  the next valid sample.
- Added timestamped GNSS, trusted-heading, and barometer observations with per-source duplicate,
  reorder, future, and configurable-age rejection. Untimestamped compatibility functions remain,
  but the FCOne integration guide forbids their use.
- Routed the native validation runner through the timestamped APIs, so every synthetic and replay
  scenario exercises the production freshness contract.
- Added deterministic public-API state/covariance invariance checks and a fixed-seed bulk campaign.

### Failed test retained and corrected

The first extended bulk run reported exactly 300 invariant failures. All were the stale-aiding
cases: the test expected a 0.05 s age limit but had left the filter at its documented 0.5 s default.
The implementation correctly accepted those observations. The campaign fixture now explicitly
sets 0.05 s; no library threshold or acceptance criterion was relaxed.

### Final evidence

Command:

```bash
python3 validation/run_host_regression.py \
  --build-dir build/input-contract-regression \
  --out-dir build/input-contract-report
```

- strict C build: passed;
- CTest: 4/4, including the 1,020,000-attempt campaign;
- Python tests: 18/18;
- deterministic scenario thresholds: all passed;
- bulk IMU attempts: 1,020,000 across 100 seeds and 50/100/200/400/1000 Hz;
- aiding attempts: 2,100 across GNSS, heading, and barometer, including separate duplicate and
  reordered cases;
- maximum injected consecutive fault burst: 100;
- invariant failures and unhealthy outputs: zero.

Generated evidence is under `build/input-contract-report/`, including one log per command,
`run-manifest.json`, the machine-readable campaign summary, and deterministic scenario reports.

### Sanitizer evidence

The first AddressSanitizer/UndefinedBehaviorSanitizer CTest invocation could not start
LeakSanitizer because the managed terminal runs under `ptrace`. This was an execution-environment
limitation, not a reported allocation leak. The same Debug binaries were rerun with only leak
detection disabled:

```bash
ASAN_OPTIONS=detect_leaks=0 UBSAN_OPTIONS=halt_on_error=1 \
  ctest --test-dir build/input-contract-sanitized --output-on-failure
```

All 4/4 tests passed. The full 1,020,000-attempt campaign ran under ASan+UBSan in 177.54 seconds;
no address or undefined-behavior finding was reported. Leak detection remains unverified in this
managed environment and must not be described as passing.

After adding separate reordered-aiding counters and exhaustive aiding numeric cases, the final
Release gate was rerun with output in `build/input-contract-final-report/`. It again passed 4/4 C
tests, 18/18 Python tests, all deterministic thresholds, and the complete bulk campaign.

## 2026-07-18 — public EuRoC suite made reproducible

### Reason

The repository contained reviewed EuRoC summaries and a converter, but no single command verified
that the restored raw inputs still matched their hashes, regenerated every track, used the native C
runner, and compared the resulting metrics to the reviewed baseline. This left room for stale
documentation or silent result drift.

### Coverage

The new manifest fixes two sequences and five distinct tracks:

- `MH_01_easy`: raw IMU and reference-bias-corrected diagnostic tracks;
- `V1_03_difficult`: raw Vicon cold-start tilt, raw trusted-attitude initialization, and
  reference-bias-corrected diagnostic tracks;
- unique recorded data: 57,313 IMU samples over 286.555 seconds;
- total replay volume: 135,558 IMU processing attempts over 677.765 aggregate track-seconds;
- each input CSV is SHA-256 checked before conversion;
- every track records converter, native replay, analyzer commands, stdout/stderr, runtime, source
  metadata, selected metrics, and baseline comparisons.

### First run finding retained

Four tracks reproduced every selected metric within the declared tolerance. The V1_03 cold-start
track exposed a stale baseline: post-alignment tilt RMSE improved from 1.230° to 0.862° after the
previously reviewed absolute-attitude covariance reset. Velocity RMSE changed from 0.085612 to
0.085292 m/s and navigation NEES from 5.618810 to 5.621088. The input hashes, conversion parameters,
sample count, 1.0 s alignment time, heading-unobservable result, health, and navigation acceptance
remained consistent. The baseline and capability documentation were updated to the newly reproduced
values; tolerances were not widened.

### Final command and result

```bash
python3 validation/run_public_dataset_suite.py \
  --data-root /path/to/euroc-minimal-inputs \
  --runner build/aerakia_validation_runner \
  --out-dir build/public-dataset-suite
```

The reviewed rerun passed 5/5 tracks, 5/5 immutable input checks, and 42/42 selected metric
comparisons. The generated directory contains `run-manifest.json`, `report.md`, per-track source
metadata, replay/results files, plots, metrics, reports, and one log for each of 15 subprocesses.
The ordinary host gate was then rerun: 4/4 CTest, 20/20 Python tests, the full 1,020,000-attempt
input campaign, and every deterministic threshold passed.
