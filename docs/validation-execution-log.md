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

## 2026-07-18 — real-motion trusted-heading geometry and recovery gate

### Reason

The synthetic trusted-heading scenario covered source dropout and outliers, but did not exercise
the production heading update through recorded high-dynamic motion. Public dataset review also
showed that motion-capture attitude truth and a recorded dual-antenna heading observation are
different evidence classes. EuRoC provides the former; RELLIS-3D's VN-300 is selected for the
latter intake.

### First run finding retained

A 10 Hz, 1°-noise heading stream derived from direct Vicon yaw was replayed through raw V1_03 IMU,
with a four-second outage and two 90° outliers. The unqualified run reported 81.5% normal update
acceptance and false yaw excursions near 180°. Inspection localized them to repeated pitch angles
of approximately 75–82°, where body-forward Euler heading is poorly conditioned and wrap changes
do not represent a comparable physical yaw error. The failed interpretation was not used as a
baseline.

The input contract and analyzer now require a minimum body-forward horizontal projection of 0.25
for this scalar heading evidence. The core also refuses the mathematically singular near-vertical
case. This preserves the existing navigation-frame yaw-only correction, which intentionally does
not use one scalar heading observation to alter independently observed tilt.

### Final evidence

- recorded V1_03 IMU samples: 20,932 over 104.655 s;
- Vicon-derived 10 Hz heading attempts: 782;
- geometry-unobservable samples: 4,582;
- cold-start full alignment: 1.000 s;
- post-alignment quaternion-geodesic attitude RMSE: 1.498°;
- observable-heading yaw RMSE: 1.130°;
- normal heading acceptance: 96.282%;
- injected 90° outlier rejection: 2/2;
- observable outage maximum yaw error: 3.943°;
- recovery time: 0.995 s;
- post-recovery yaw RMSE: 1.224°;
- navigation NEES mean: 5.646 for expected mean 6;
- unhealthy samples and navigation recoveries: zero.

The expanded public suite passed 6/6 tracks, 5/5 immutable input checks, 56/56 selected metric
comparisons, and 156,490 replayed IMU attempts from 57,313 unique recorded samples. This closes
real-motion software-path behavior only. Because the heading observations are derived from the
same Vicon truth used for scoring, it does not claim dual-antenna, vision, or rate-table sensor
accuracy.

The complete host gate was then rebuilt from a fresh Release directory and passed 4/4 CTest,
22/22 Python tests, the 1,020,000-attempt input-integrity campaign, every deterministic scenario,
and all reviewed thresholds. Generated evidence is retained under `build/heading-host-report/` and
`build/public-dataset-suite-heading/`.

The Debug sanitizer build was rebuilt and rerun with `ASAN_OPTIONS=detect_leaks=0` and
`UBSAN_OPTIONS=halt_on_error=1`. All 4/4 tests passed; the complete 1,020,000-attempt campaign ran
under ASan+UBSan in 183.81 seconds with no address or undefined-behavior report. Leak detection
remains excluded because of the previously documented managed-terminal `ptrace` limitation.

## 2026-07-18 — estimator roles fixed before FCOne integration

### Reason

Running ESKF and Mahony in parallel is not by itself a safe fallback architecture. Their product
roles, valid outputs, transition evidence, and ownership boundary must be explicit before an FCOne
adapter or controller can consume them.

### Decision

- The 15-error-state ESKF is the primary navigation estimator.
- Robust Mahony is an independent attitude-only fallback and cross-monitor.
- Standard Mahony remains a validation baseline, not a deployed fallback.
- Automatic source selection and control continuity belong to a private FCOne supervisor.
- Mahony-only degradation invalidates position, velocity, and navigation integrity outputs.
- One rejected aiding update or estimator disagreement is diagnostic evidence, not an immediate
  switch condition.

The complete hardware-independent contract, required transition log fields, hysteresis rule, and
remaining executable mock test are recorded in
[`estimator-supervision.md`](estimator-supervision.md) and the roadmap.

### Health-audit finding and correction

Reviewing the signals available to the future FCOne supervisor found that
`AerakiaNavigationEstimate.healthy` checked only covariance diagonal finiteness and sign. A
corrupted nominal position, velocity, bias, or quaternion could therefore remain labelled healthy
if the covariance was still finite. The health calculation now also requires every nominal-state
component to be finite and the quaternion squared norm to remain within `1e-3` of unity.
Deterministic public-API tests inject a non-finite state, a non-unit quaternion, and a non-finite
covariance separately and require all three to report unhealthy.

The first strict build of this check used the generic name `ESKF_State`; the public core names the
type `ESKF_NominalState`, so compilation correctly failed before tests ran. The helper signature was
corrected to the existing public type; no estimator behavior or acceptance threshold was changed to
resolve the build failure.

### Regression evidence

Command:

```bash
python3 validation/run_host_regression.py \
  --build-dir build/supervisor-health-host \
  --out-dir build/supervisor-health-report
```

- strict Release build: passed;
- CTest: 4/4, including the 1,020,000-attempt input campaign;
- Python tests: 22/22;
- all deterministic accuracy, recovery, consistency, bias, and magnetic-robustness thresholds:
  passed;
- generated logs and environment/commit manifest: `build/supervisor-health-report/`.

## 2026-07-18 — INSANE outdoor UAV heading intake started

### Reason

RELLIS-3D records a physical VN-300 dual-GNSS/INS, but its published ROS topics expose the unit's
fused quaternion without raw dual-baseline validity. The INSANE `outdoor_1` package instead retains
two synchronized RTK receiver streams, PX4 IMU/magnetometer data, calibration, time offsets, and a
published 80 Hz six-degree-of-freedom reference. It is therefore the stronger public physical-yaw
intake while FCOne v2 hardware is unavailable.

### Source audit

- official sensor ZIP: 18,586,950 bytes, SHA-256
  `02ea94047ccb7d887c34f90f0c868f8430bdc448883bf82aeadd2a616d79bb79`;
- official calibration ZIP: 9,458 bytes, SHA-256
  `cff4fbd099051cf0ff29838f7ab4bc67ff35d70cc6555029e537ca1036c215f2`;
- dataset-tools commit: `9a1c8c0fdd195f2d869fff292f2ce5b273c5a03d`;
- package contents: 51,034 PX4 IMU records, 22,786 PX4 magnetometer records, two RTK streams,
  and 17,503 published 80 Hz pose rows;
- official baseline length: 1.16 m; virtual-baseline azimuth in the PX4 IMU frame: -135 degrees;
- declared license: BSD-2-Clause with additional no-sale/non-commercial and citation conditions.

The published pose is generated from the dual-RTK baseline plus calibrated magnetometer, so direct
baseline heading and reference yaw share measurements. This is strong evidence that the production
heading path handles a physical dual-antenna observation, but not an independent sensor-versus-truth
accuracy proof.

### First replay and retained failure

The first heading-isolation conversion produced 39,174 overlapping recorded IMU samples over
199.735 s and 1,378 geometry-qualified dual-RTK heading updates. Native C replay completed with no
malformed row and no navigation recovery. Markdown report generation then failed because the
analyzer assumed every trusted-heading track contained declared injected faults and formatted a
missing fault-rejection ratio as a number. Natural physical data intentionally contains no injected
fault flag. The report writer now prints this field as not applicable; no heading acceptance or
accuracy threshold was altered.

The first report rerun also refused the new evidence label `shared_sensor_reference` because the CLI
previously allowed only synthetic, independent-truth, and PX4-estimate categories. A dedicated
shared-sensor category was added so this track cannot be accidentally presented as independent
truth. NIS/NEES logic continues to exclude shared-sensor references.

The next report exposed a converter wiring error: reference quaternions were populated, but the
legacy Euler reference columns were left at zero. The native runner correctly preserved the
quaternion fields while the per-axis report read the explicit zero Euler columns, producing a false
123-degree yaw error. The converter now derives those diagnostic columns from the transformed
reference quaternion; this correction changes reference serialization, not estimator output.

### Reviewed INSANE result

After correcting reference serialization, the reference-seeded track reports 2.018-degree ESKF yaw
RMSE, 97.10% normal heading acceptance, and 100% estimator health. The independent cold-start track
completes tilt alignment at 1.000 s and heading alignment at 11.492 s; post-heading-alignment yaw
RMSE is 1.534 degrees, normal heading acceptance is 91.15%, reported recovery is 2.000 s, and health
remains 100%. Both tracks replay all 39,174 rows with zero malformed inputs and zero navigation
recoveries.

The full-attitude result is deliberately rejected as capability evidence. At the start of the
sequence, the published pose roll/pitch is inconsistent with the measured accelerometer gravity
direction and produces roughly 18–20 degrees of tilt disagreement. The dual-RTK heading itself is
geometrically consistent: 1,378 input candidates have a median 1.1591 m baseline against the
declared 1.16 m installation. The committed summary therefore contains yaw-path metrics and the
tilt limitation, not a misleading full-attitude score.

Two converter unit tests were added for the ENU/FLU to NED/FRD transform and RTK-fixed pairing. The
first test expectation incorrectly assumed a 180-degree roll for the identity source transform; the
matrix and quaternion both showed the correct result is 0-degree roll and 90-degree NED yaw. The
expectation was corrected without changing production code, and the retained transform round-trip
assertion now guards the actual invariant.

## 2026-07-18 — RELLIS-3D full-stack audit

### Reason and source

RELLIS was evaluated as a possible second physical-heading dataset because the platform uses a
VectorNav VN-300. The official download is a 4.0 GiB ZIP with SHA-256
`95c0eacef45b28c832ec7a3bdb60042891230ea98c81f1a052e50b00bbb941da`; its single entry is an
8,887,258,280-byte `example_filtered.bag`. The ZIP integrity test passes.

### Retained failure and evidence decision

The extracted ROS bag has a valid `#ROSBAG V2.0` header but no readable index for the standard
`rosbags` reader. Raw topic scanning confirms `/vectornav/IMU`, `/vectornav/Odom`, `/vectornav/GPS`,
`/vectornav/Mag`, `/vectornav/Pres`, and `/vectornav/Temp`. The reviewed VectorNav driver publishes
the device-fused quaternion and does not expose raw dual-antenna baseline validity/status through
these topics. Its orientation covariance conversion also multiplies squared degree uncertainty by
`pi/180` rather than squaring the radians conversion, so the published covariance is not accepted
without correction.

RELLIS is consequently not counted as independent physical heading truth. Reindexing may make it
useful for fused-device compatibility, but it does not improve the evidence class enough to block
the higher-value INSANE integration or broader independent datasets.

## 2026-07-18 — Estimator roles, health semantics, and final gate

### Role decision

The 15-error-state ESKF is the primary navigation estimator. Robust Mahony is an independent
attitude-only fallback and cross-monitor; standard Mahony remains a comparison baseline. A private
FCOne supervisor owns mode transitions. On Mahony degradation, attitude may remain valid but ESKF
position, velocity, covariance, and navigation-ready flags must be invalidated. The public repository
now records the mode contract and handover constraints without embedding private flight policy.

### Health defect closed

`AerakiaNavigationEstimate.healthy` previously checked only covariance diagonals. A corrupted nominal
position, velocity, bias, or quaternion could therefore be reported healthy while covariance stayed
finite. Health now requires every nominal state component to be finite, quaternion norm squared to
remain within 0.001 of unity, and covariance diagonals to be finite/non-negative. Public API tests
inject NaN state, non-unit quaternion, and NaN covariance independently; all must report unhealthy.

### Final verification

```bash
python validation/run_host_regression.py \
  --build-dir build/final-heading-host \
  --out-dir build/final-heading-report
```

- strict Release C99 build: passed;
- CTest: 4/4, including the 1,020,000-attempt input-integrity campaign;
- Python tools: 24/24;
- all deterministic accuracy, recovery, consistency, bias, and magnetic gates: passed;
- generated logs and manifest: `build/final-heading-report/`.

The same C targets were rebuilt with AddressSanitizer and UndefinedBehaviorSanitizer. ESKF core,
magnetic gate, public API, and the full 1,020,000-attempt campaign all passed. The campaign reported
zero invariant failures and zero unhealthy outputs after 790,917 accepted IMU samples, 229,083
intentional IMU rejections, timing/burst faults, and all declared aiding freshness/numeric cases.
