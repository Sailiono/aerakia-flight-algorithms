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

## 2026-07-18 — Executable FCOne-neutral integration contracts

### Reason

The previous milestone documented ESKF-primary/Mahony-fallback roles but did not execute the
application-level invariants. Two host-only oracles were added without importing private FCOne
headers or moving product policy into the public algorithm core:

- `validation/estimator_supervisor_contract.c` models required mode/output behavior;
- `validation/fcone_adapter_contract.c` models the final calibrated-publication boundary and drives
  the real ESKF adapter through timing and aiding cases.

### Supervisor coverage and safety finding

The supervisor contract covers five-sample startup qualification, three-sample soft-observability
confirmation, Mahony attitude-only degradation, position/velocity invalidation, 15-degree handover
continuity, five-sample recovery dwell, both-estimators-invalid behavior, transition reason/count,
and transition timestamp. The later Blackbird milestone below supersedes the entry gate with 10°
and adds continuity on fallback entry plus a finite degraded-mode budget.

The first design applied hysteresis to every ESKF health loss. Review found that this would allow a
numerically invalid ESKF state to remain selected during the confirmation window. The contract now
separates hard invalidity from soft degradation: non-finite state, invalid quaternion, or invalid
covariance causes immediate Mahony/invalid transition; only finite but temporarily unobservable or
aid-degraded navigation uses failure confirmation. This is a contract correction found before any
private FCOne implementation existed.

### Adapter coverage and retained test failures

The mock FCOne publication uses physical timestamps, FRD engineering units, and independent source
validity. Sentinel values verify `g` to m/s², degrees/s to rad/s, and gauss to µT without silent axis
changes. The real adapter is then exercised for missing required gyro, corrected same-timestamp
retry, duplicate rejection, forward-gap reanchor and recovery, delayed GNSS, future/stale/duplicate
GNSS, and timestamped trusted heading.

The first run failed two test expectations:

1. the magnetic conversion comparison used a 1e-6 float tolerance even though decimal gauss values
   multiplied by 100 incur slightly larger binary-float rounding; the sentinel tolerance was changed
   to 1e-5 without changing conversion or algorithm code;
2. a stale GNSS assertion was issued after a newer GNSS update, so the API correctly classified it
   as reordered before evaluating age. The sequence now tests future and stale observations before
   accepting the first fresh observation, then tests duplicate handling.

After correction, both new contracts pass under strict `-Werror -pedantic` compilation. They are
test oracles, not claims that the private FCOne v2 adapter, supervisor, scheduler, or flight response
has already been implemented.

### Complete gate

```bash
python validation/run_host_regression.py \
  --build-dir build/integration-contract-final \
  --out-dir build/integration-contract-report
```

- Release CTest: 6/6;
- Python tools: 24/24;
- deterministic accuracy/recovery/NIS/NEES/bias/magnetic thresholds: all passed;
- release input-integrity campaign: 1,020,000 attempts, zero invariant failures and zero unhealthy
  outputs.

All five non-campaign C targets passed under ASan+UBSan. The full 1,020,000-attempt campaign was then
run directly with the same sanitizers and also completed with zero invariant failures, zero unhealthy
outputs, 790,917 accepted samples, 139,059 missing-measurement rejections, 90,024 timestamp
rejections, 20,355 forward-gap reanchors, and every declared aiding fault class exercised.

## 2026-07-18 — Blackbird independent-motion intake and bounded fallback

### Reason

The existing public corpus covered two EuRoC sequences but did not independently score a longer
aggressive UAV trajectory. It also documented robust Mahony as an attitude fallback without a
data-derived bound on how long a parallel, unaided Mahony state could remain control-qualified.

The official Blackbird sensor host returned TLS EOF/empty HTTP responses. The intake therefore used
MathWorks' MIT-licensed `BlackbirdVIOData.tar` redistribution of `NYC Subway Winter`, retained the
raw TAR/MAT outside Git, and committed only the converter, input hash, manifest, and metric summary.

### Retained frame/time failure and correction

The first audit independently subtracted the first IMU and first motion-capture timestamps. This
silently discarded a `-1.762237184 s` stream-start offset; x/y truth-vs-IMU angular-rate correlation
then appeared near zero. No algorithm score from that attempt was accepted.

Using a common absolute time origin and the official Blackbird body-to-IMU quaternion produced
axis correlations `0.99345 / 0.98947 / 0.99984`, total angular-rate RMSE `0.03627 rad/s`, and total
specific-force RMSE `0.36386 m/s²`. These checks are executable preconditions in
`convert_blackbird_to_replay.py`; ambiguous time/frame data now fail closed.

### Coverage and result

- 26,995 unique recorded IMU samples, 270.076 s, 99.978 Hz;
- independent 19.9999 Hz motion-capture pose;
- speed P95/max `2.80/3.03 m/s`;
- gyro norm P95/max `1.95/4.22 rad/s`;
- acceleration norm P99/max `11.43/59.78 m/s²`;
- three retained tracks and 80,985 replay attempts.

Raw ESKF tracking reaches `1.573°` geodesic attitude, `1.131°` tilt, and `1.069°` Euler-yaw RMSE.
It remains healthy for every sample with zero navigation recovery. Tilt-only cold start completes in
`1.009 s` and reaches `1.131°` post-alignment tilt RMSE; heading alignment correctly remains false.
Synthetic GNSS produces `0.123 m / 0.082 m/s` position/velocity RMSE and navigation NEES `4.76`, but
is explicitly not recorded-receiver evidence.

Raw robust Mahony reaches `51.478°` geodesic and `7.814°` tilt RMSE with no magnetometer/heading.
Static reference-bias subtraction improves full attitude only to `10.129°`, leaving the aggressive
acceleration limitation visible. The offline truth envelope shows first 10° error at `14.588 s`;
an entry at or below 10° can reach `14.959°` within one second, while the older 15°/five-second
example can reach `20.307°`.

### Supervisor correction

The earlier oracle checked continuity only when returning from Mahony to ESKF. It could therefore
select a parallel Mahony instance that had already drifted before the ESKF fault. The executable
contract now:

- compares Mahony with the last qualified output before fallback entry;
- rejects a discontinuous fallback instead of publishing an attitude step;
- invalidates position, velocity, and navigation throughout Mahony-only operation;
- expires Mahony-only operation after a finite budget with an explicit transition reason;
- uses a conservative 10°/one-second host-test gate derived from the Blackbird envelope.

The numerical values are test-oracle defaults, not flight-qualified FCOne policy. Final limits must
be chosen with vehicle dynamics, controller tolerance, failsafe timing, HIL, and hardware evidence.

### Reproduction

```bash
python validation/run_public_dataset_suite.py \
  --data-root /path/to/blackbird-mathworks \
  --manifest validation/public/blackbird_manifest.json \
  --runner build/aerakia_validation_runner \
  --out-dir build/blackbird-suite
```

The suite verifies the MAT SHA-256 before conversion, runs and logs all nine subprocesses, checks
selected metrics against the committed baseline, and records the host environment and Git state.

The complete Release host gate passes CTest `6/6`, Python `29/29`, all deterministic
accuracy/recovery/NIS/NEES/bias/magnetic thresholds, and the 1,020,000-attempt input-integrity
campaign with zero invariant or unhealthy-output failures. The five non-campaign C targets also
pass ASan+UBSan. LeakSanitizer itself cannot run inside the desktop sandbox's ptrace environment;
that infrastructure failure is retained, and the ASan+UBSan run is repeated with leak detection
disabled rather than misreported as a code defect. The full sanitized 1,020,000-attempt campaign
then completes with 790,917 accepted samples, 229,083 intentional rejections, 20,355 forward-gap
reanchors, zero invariant failures, and zero unhealthy outputs.

## 2026-07-18 — UrbanNav recorded-position outage and independent aiding APIs

### Reason and complementary evidence choice

EuRoC and Blackbird provide external motion references, but their GNSS observations are generated
from truth. Private PX4 ULogs contain physical GNSS but compare primarily with another onboard
estimator rather than independent truth. UrbanNav HK `Medium-Urban-1` was selected to bridge those
evidence classes with recorded Xsens IMU, recorded u-blox F9P positions, SPAN-CPT postprocessed
truth, and an actual long receiver outage. It remains a ground-vehicle dataset and cannot substitute
for aircraft dynamics, physical heading, or an independent raw-sensor truth chain.

Only the selected official IMU/GNSS/truth files were retained under ignored `build/` storage. The
official tools repository was pinned at `075f96b6a6d9252b37486ecb175b4ae690c56f54`. Immutable input
hashes are recorded in `validation/public/urbannav_manifest.json` and
`docs/public-datasets.md`; raw archives and generated replay/result CSVs are not committed.

### Interface defect exposed and corrected

The previous timestamped public contract required GNSS position and velocity in one observation.
The selected F9P NMEA publishes valid positions and GST uncertainty but no receiver velocity or
course. Differentiating position would create a derived signal and falsely label it physical
receiver velocity. The public API therefore gained independent timestamped position and velocity
observations while preserving the paired GPS API. Freshness, duplicate rejection, and component-
specific recovery are tested separately; position recovery preserves velocity and velocity
recovery preserves position.

A post-implementation review found that the first independent-API draft still shared one rejection
counter. Interleaved accepted positions could therefore erase persistent velocity rejection history.
The counters are now source-specific, the legacy aggregate exposes their maximum for diagnostics,
and an alternating accepted-position/rejected-velocity test proves velocity recovery still fires.

The replay schema now has independent `gps_position_update` and `gps_velocity_update` flags. The
analyzer applies NIS masks to the actual update source rather than assuming every position epoch
also fused velocity.

### Intake audit and coverage

- 314,185 unique IMU samples over 785.451 s, approximately 400.33 Hz;
- 655 accepted physical F9P position epochs with same-epoch GST uncertainty;
- 37 corrupt/non-NMEA fragment lines rejected by checksum/format checks;
- one recorded 131 s position-aiding gap, about 130 missing nominal 1 Hz epochs;
- speed P95/max `10.20/11.50 m/s` and gyro-norm P95/max `0.225/0.675 rad/s`.

The official source body frame is right/forward/up. The converter maps it to FRD and converts ENU
navigation to NED. A fail-closed audit differentiates SPAN position and compares it with the
separately published SPAN body velocity. North/east correlations are `0.99992/0.99989` and
horizontal velocity RMSE is `0.0745 m/s`; scoring is refused if this audit fails.

Raw F9P horizontal position error against SPAN is `2.890 m` RMSE, `5.036 m` P95, and `6.183 m`
maximum. No antenna lever arm is applied because the published direction is not sufficiently clear
for an unchecked assumption. A scalar worst-axis GST variance is used and recorded as a limitation.

### Result and corrected interpretation

With reference attitude initialization, ESKF attitude RMSE is `1.534°` geodesic, `0.772°` tilt,
and `1.328°` yaw. Nominal position-aided RMSE is `6.518 m`. During the 131 s outage, position error
peaks at `1471.5 m` and velocity error at `27.40 m/s`; the first resumed physical position update
returns posterior position error to `5.395 m`, with 100% finite/healthy output and no forced reset.

The first review incorrectly suspected the scalar GST variance was the main explanation for the
approximately `247.99 m` whole-run position RMSE. Interval decomposition disproved that: the value
is dominated by the real 131 s unaided segment, while pre-gap and normal aided errors remain around
the receiver-level single-digit-metre range. The analyzer now reports aided, unaided, pre-gap,
peak, first-resumed, and sustained-recovery metrics separately so this interpretation cannot recur.

The cold-start track completes tilt alignment in about one second and reaches `2.211°` tilt RMSE,
but heading alignment correctly remains false and yaw RMSE is `36.65°`. The sequence has neither
magnetometer nor physical heading, so this is retained evidence of yaw unobservability. Robust
Mahony likewise never reaches the configured fallback entry-error gates; this exposed an empty-set
crash in the offline fallback analyzer, which now returns `null` metrics and has a regression test.

Position NIS means `0.011/0.339` are strongly conservative while six-state navigation NEES means
`12.91/16.27` exceed the expected mean of 6. These are not tuned away on the scored sequence. They
show that position-only aiding leaves the velocity subspace insufficiently modeled and that
real-receiver covariance tuning is not complete.

### Reproduction and remaining limitations

```bash
python validation/run_public_dataset_suite.py \
  --data-root build/dataset-intake/urbannav-public \
  --manifest validation/public/urbannav_manifest.json \
  --runner build/urbannav-dev/aerakia_validation_runner \
  --out-dir build/public-dataset-suite-urbannav
```

The reviewed suite passes two tracks and 628,370 replay attempts with hash/baseline checks. Combined
with EuRoC and Blackbird, the independent/external-reference corpus contains 398,493 unique physical
IMU samples and 865,845 replay attempts. INSANE remains a separate shared-source physical-heading
class rather than being mixed into the independent-truth count.

Remaining limitations are explicit: ground vehicle rather than aircraft; no receiver velocity,
magnetometer, or heading; SPAN is a postprocessed GNSS/INS reference; no antenna lever-arm or aiding
delay compensation; and scalar rather than per-axis receiver variance. The next complementary
dataset should provide recorded receiver Doppler velocity plus independent truth, preferably on an
aerial platform. A second UrbanNav tunnel track adds environmental diversity but repeats the same
core limitations.

### Validation closure

- strict Release CTest: `6/6`, including the 1,020,000-attempt input-integrity campaign;
- Python tools: `34/34`, including NMEA integrity, all rotation-to-quaternion branches, frame
  mapping, gap decomposition, and no-eligible-fallback handling;
- deterministic accuracy, magnetic, cold-start, outage, heading, bias, NIS, and NEES thresholds:
  all passed through `run_host_regression.py`;
- UrbanNav baseline: `2/2` tracks and 628,370 replay attempts;
- complementary Blackbird baseline: `3/3` tracks and 80,985 replay attempts;
- complementary EuRoC baseline: `6/6` tracks and 156,490 replay attempts;
- ASan+UBSan: all six C targets passed; the sanitized 1,020,000-attempt campaign completed in
  182.69 s after the final source-specific counter correction, with leak detection disabled for the
  previously documented managed-terminal limitation.

The first Blackbird rerun failed before conversion because system Python lacked SciPy. No score was
accepted from that attempt. An ignored repository-local virtual environment was created from
`requirements.txt`; the complete Blackbird and EuRoC suites then passed under that isolated runtime.

## 2026-07-18 — no-aiding validity and pinned PX4 comparison boundary

### Reason

The UrbanNav outage showed a dangerous qualification ambiguity: `healthy` described finite state
and covariance only, so it remained true throughout 131 seconds without horizontal aiding even
after position error became operationally unusable. A finite filter is not necessarily a valid
navigation solution. The public contract and FCOne supervisor oracle needed to express that
difference directly before integration.

PX4 `main` was reviewed at commit
`de8158101c96ad6b04170dc91f087148104c58eb`. Its generated state definition confirms that EKF2
also uses three covariance dimensions for quaternion attitude, so comparing the labels `EKF` and
`ESKF` is not meaningful. The reviewed PX4 system has a substantially broader 24-dimensional error
state, delayed-fusion/output-prediction machinery, more aiding models, GSF yaw backup, and mature
fusion/failsafe state machines. This is an operational feature gap, not evidence for an invented
accuracy ratio. A same-input A/B protocol is retained in `docs/px4-ekf2-comparison.md`.

### Contract correction

- The original checkpoint used one five-second horizontal timeout. The later independent G0 review
  split it into `maximum_horizontal_position_dead_reckoning_s` and
  `maximum_horizontal_velocity_dead_reckoning_s`, both defaulting to five seconds.
- Numerical `healthy` is retained separately from horizontal position, velocity, and combined
  navigation validity.
- Accepted position and velocity constraints refresh only their own timestamps; ZUPT refreshes
  velocity only, and recovery probation keeps both invalid until its paired dwell completes.
- A rejected observation cannot refresh validity, and velocity-only aiding cannot invent an
  uninitialized position origin.
- The estimator-supervisor executable contract now consumes the production validity flag instead
  of a standalone test-only observability boolean.

The five-second default mirrors the reviewed PX4 `EKF2_NOAID_TOUT` default but remains configurable;
private FCOne policy must still decide vehicle mode and failsafe behavior.

### Recorded-outage result

Both immutable UrbanNav tracks pass their complete baseline after adding validity output. Numerical
health remains `100%`, while horizontal navigation is valid for `263,605 / 314,185` samples, or
`83.9012%`. Maximum finite aiding age reaches `130.9959 s`. Thus the recorded long outage remains
available for drift/recovery analysis, but the public output is no longer qualified as valid for
most of that interval. First-update reacquisition and all previously retained accuracy and
consistency metrics remain unchanged.

### Validation at this checkpoint

- strict Release CTest: `6/6`;
- Python tools: `34/34`;
- one-command host regression: passed all reviewed deterministic gates;
- UrbanNav baseline: `2/2`, 628,370 replay attempts;
- ASan+UBSan: `6/6`; the complete 1,020,000-attempt campaign took `184.10 s`, and the
  final expanded public-API timeout test was rebuilt and rerun separately under the same flags;
- `git diff --check`: clean at the checkpoint.

This closes output qualification, not long-duration pure INS. The latter is not a supported
capability for either Aerakia or PX4-class MEMS inputs without another velocity/position constraint.

## 2026-07-18 — IDF-DS volume expansion and current-PX4 schema repair

### Reason and source quality

The preceding public corpus had strong independent/external references but limited total flight
time. IDF-DS was selected for volume, fixed-wing duration, aggressive dynamics, PX4 schema
coverage, reset events, clipping, airspeed, GNSS, and estimator diagnostics. It is not an
independent-truth dataset: PX4 attitude/local position are onboard comparison references.

The official 2,121,943,653-byte Pixhawk archive matched publisher MD5
`8b990cc4c7ec1225a16e9a28225e5162`. The published package contains 120 synchronized flight CSVs
and 13 original ULogs. Only ignored local extraction/replay artifacts were retained; code, audit
logic, checksums, aggregate coverage, and reviewed metrics enter Git.

### Full raw-ULog intake

All 13 original ULogs parsed without error:

- 7,128,090 IMU samples and 310,799 GNSS samples;
- 35,697.16 s (9.92 h) total duration;
- maximum 7.268 rad/s gyro, 240.62 m/s² acceleration, and 27.59 m/s GNSS speed;
- 12 logs with PX4 attitude resets, one with reported clipping, and zero with direct GNSS heading.

The intake found that current PX4 `vehicle_gps_position` can contain a constantly zero
`timestamp_sample` while its publication `timestamp` remains valid. The old field-presence rule
would collapse such GNSS data to one epoch. Timestamp selection now requires a usable varying
stream and otherwise falls back to publication time. Current `latitude_deg/longitude_deg` and SI
altitude fields are also supported alongside legacy integer geodetic fields. Both repairs have
unit tests.

### Selected native-C replays

Three tracks were chosen from the full audit rather than by favorable score: maximum rotation,
maximum speed, and the only log with reported clipping. They add 1,608,985 native-C replay samples
over 8,054.42 s. Numerical health is 100% on all three, but the retained results expose unresolved
real-receiver modeling:

| Track | Full / tilt / yaw RMSE | Position / velocity RMSE | Position / velocity NIS mean | Recoveries |
| --- | ---: | ---: | ---: | ---: |
| rotation | 3.241° / 2.449° / 2.105° | 2.073 m / 0.335 m/s | 35.68 / 13.73 | 34 |
| speed | 5.823° / 2.472° / 5.257° | 3.619 m / 0.632 m/s | 95.83 / 20.52 | 80 |
| clipping | 3.781° / 2.589° / 2.740° | 2.244 m / 0.373 m/s | 36.59 / 15.60 | 26 |

For three-dimensional measurements the expected NIS mean is 3. These large values, 54--86%
component acceptance, and 26--80 recoveries block any claim that GNSS delay/noise tuning is
complete. They are not tuned away on these evaluation tracks. Mahony robust full-attitude RMSE is
96--98° over the long runs, supporting its bounded degraded-attitude role and rejecting a
long-duration parallel-navigation interpretation.

## 2026-07-18 — aerial physical-GPS and RTK-reference intake

### Reason and evidence boundary

UrbanNav closed real position/outage coverage but did not publish receiver velocity, and the
long-duration IDF tracks use PX4 estimates as references. The Zenodo electrical-infrastructure UAV
survey was selected to add aircraft motion, physical DJI GPS position, paired RTK position/velocity,
and multiple physical IMUs. The smallest 2,199,738,167-byte bag matched publisher MD5
`ccb69193138b5b7f5ae1e44bde81228d`.

The bag does not contain a separate drone-GPS velocity topic. RTK velocity is used only as a scored
reference, never silently relabeled as receiver aiding. DJI onboard orientation shares onboard
measurements and is likewise an engineering attitude reference, not independent truth.

### Intake repairs and audit

The first timing summary incorrectly used the inverse median interval as the reported rate. Bursty
header timestamps made that number physically impossible. The auditor now reports effective rate
as `(samples - 1) / duration`, retains median interval separately, and fails non-monotonic streams.
The first RTK frame decision compared only horizontal velocity and could not distinguish vertical
sign. Selection now minimizes full three-dimensional differentiated-position RMSE and records all
candidate correlations/errors.

Accepted intake evidence:

- common overlap `41.393 s`;
- DJI IMU/GPS effective rates `400.04/50.00 Hz`; RTK position/velocity `5.001 Hz`;
- selected native NED RTK velocity has north/east/down correlations
  `0.9794/0.9717/0.9834` and `0.197 m/s` 3D RMSE against differentiated RTK position;
- DJI GPS versus RTK relative horizontal RMSE/P95/max is `0.109/0.149/0.193 m`;
- header-to-bag timestamp P95 is at most `4.98 ms` across required streams, although isolated
  maximum outliers reach `98 ms` and are retained in metadata.

### Native replay result

The hardware-neutral replay contains 16,560 physical IMU samples, 2,070 physical GPS position
updates, and no invented GPS velocity. All samples remain numerically healthy and all position
updates are accepted. Against the separately recorded RTK reference, position RMSE/P95 is
`0.179/0.277 m` and velocity RMSE is `0.263 m/s`. Against DJI onboard orientation, ESKF geodesic,
tilt, and yaw RMSE are `2.766/0.966/2.601°`.

Position NIS mean is only `0.00107` for expected mean 3. This is retained as a covariance/source-
independence limitation: the bag has no position variance, conversion declares fixed `4 m²`, and
the DJI GPS/RTK streams may share receiver or correction sources. It is not an accuracy claim.
Robust Mahony reaches `136.77°` full-attitude RMSE but `1.629°` tilt RMSE because this replay has no
magnetometer or trusted heading; it remains a finite-duration attitude fallback, not navigation.

### Reproduction and checkpoint validation

```bash
python validation/audit_uav_electrical_bag.py voo_3_electrical.bag \
  --expected-md5 ccb69193138b5b7f5ae1e44bde81228d --out intake.json
python simulation/tools/convert_uav_electrical_to_replay.py voo_3_electrical.bag \
  --out replay.csv --metadata source.json
build/aerakia_validation_runner replay.csv results.csv
python validation/analyze_results.py results.csv --out-dir report \
  --scenario uav_electrical_voo3 --reference-kind external_reference
```

The raw bag, replay, result CSV, and plots remain under ignored `build/`. Reusable intake/conversion
code, source checksum, reason, corrections, metrics, and limitations enter Git. Python unit tests
now include current-PX4 timestamp/schema regressions, NED geodetic axes, and the UAV ENU/FLU to
NED/FRD proper-rotation round trip.

Final checkpoint after the new evidence classes and converters: strict Release CTest `6/6`, Python
tools `39/39`, all deterministic scenario thresholds, the 1,020,000-attempt input-integrity gate,
Python byte-compilation, and `git diff --check` pass through the one-command host regression.

## 2026-07-18 — scoped PX4-class goal and parallel evidence audit

### Reason

The project goal was clarified as reaching at least PX4-class estimation capability while later
using stronger FCOne v2 hardware. This required deciding whether to prioritize independent-truth
datasets or a direct PX4 comparison, and whether repeatedly fitting to truth would be valid.

Three read-only subagent audits ran in parallel with non-overlapping scopes:

1. official pinned PX4 `ecl_EKF` same-input harness design;
2. primary-source public dataset and truth-independence search;
3. current capability, evidence-grade, tests, and acceptance-gap audit.

The primary agent verified the high-risk source findings locally and integrated the final plan.

### Decision

Independent truth and PX4 A/B are complementary rather than alternatives. Truth measures absolute
accuracy/consistency; same-input A/B measures relative non-inferiority inside a declared common
sensor/task envelope. Both estimators can agree and both be wrong, so PX4 output never becomes
truth. Parameters are tuned only on whole-flight calibration/development partitions, then frozen
for validation and locked blind flights.

"PX4-class" is initially scoped to local-NED, GNSS-aided small multirotor/fixed-wing operation with
physical IMU, magnetometer, barometer, GNSS position/velocity, optional trusted heading, bounded
aiding outages, and explicit validity/recovery. It does not claim every PX4 vehicle, sensor, or
deployment-history capability.

### Source-verified G0 blockers

- `src/eskf.c` labels three-dimensional position/velocity gates as 3-sigma but applies the scalar
  rule `NIS <= gate²`; vector thresholds must instead be tied to measurement degrees of freedom and
  a declared false-rejection probability.
- `src/eskf_adapter.c` defaults `navigation_recovery_rejection_limit` to ten and directly resets
  position or velocity to a repeatedly rejected observation, then refreshes horizontal aiding.
  This can turn a persistent bad source into a trusted re-anchor. Recovery needs source-quality,
  consistency, correction-size, supervisor-authorization, and probation gates.
- Current tests check covariance and process-noise behavior but contain no executable complete
  F/Q/H finite-difference suite, despite earlier documentation/history referring to one.
- Startup heading completion is retained, but no continuous heading-aiding-age/validity output
  exists. Vertical validity is also not independently qualified.
- `CMakeLists.txt` enables `-Wall -Wextra`, not repository-level `-Werror`; GitHub CI has no
  sanitizer job. Local strict/sanitized results remain valid checkpoints, but the remote gate is
  incomplete.
- Public dataset baselines primarily detect metric change. They must be complemented by one-way
  capability thresholds so a reproducibly poor result remains a capability failure.

### PX4 M0 design

The official PX4 commit remains
`de8158101c96ad6b04170dc91f087148104c58eb`. M0 will compile the official host `ecl_EKF` and drive
`EstimatorInterface` directly. The unversioned full PX4 snapshot under FCOne v1 may help prototype
an adapter but cannot be a benchmark; `ekf2_integration/src/ekf2_lite.cpp` is still an explicit
stub with initialization, input, update, and output TODOs.

M0 consumes the same immutable synthetic outage event stream, records source/config/input hashes,
checks units/frames/event counts, separates zero/recorded/scanned delay, retains PX4 resets and
fault flags, and scores each output at its own physical timestamp. It must be deterministic and
generate 5/10/30/60/120 s cuts before any performance ratio is accepted.

### Dataset search outcome

No reviewed public source simultaneously provides physical UAV IMU, physical receiver
position/Doppler velocity, independent continuous 6DoF/yaw truth, deliberate GNSS outage, and broad
flight dynamics. The evidence set therefore remains a matrix.

Immediate small downloads are INSANE `indoor_1` and `transition_1` (about 53.6 MB total) to reuse
the existing converter with independent indoor OptiTrack yaw/full pose and transition behavior.
RTK-SLAM follows for deliberate long GNSS degradation and surveyed Leica checkpoints, while MILUV
is retained for selected multi-UAV Vicon/bias experiments rather than a full 173 GB download.
INSANE outdoor/reference channels and license wording require their existing shared-source and
usage caveats; no channel is promoted without a fresh frame/time/license audit.

### Recorded plan

The resulting scope, evidence grades, anti-overfitting split, G0--G4 gates, PX4 M0 fairness rules,
provisional blind-test non-inferiority method, dataset order, FCOne/HIL requirements, and subagent
ownership are now centralized in `docs/px4-class-validation-plan.md`.

## 2026-07-18 — G0 mathematical, recovery, validity, and statistical closure

### Reason and source audit

The PX4-class gap audit identified four issues that could invalidate additional volume claims:
three-dimensional observations used the scalar 3-sigma NIS limit, repeatedly rejected navigation
observations could eventually force an automatic re-anchor, heading/vertical observability was not
continuously qualified, and the current tree no longer contained the executable full F/Q/H audit
described by older history.

The history audit recovered the earlier internal-model test boundary and also found that the active
tree had lost the position transition's attitude and accelerometer-bias `dt²/2` couplings. The
first restored 10,000-case prediction finite-difference run reached maximum absolute error
`0.00337988`, above the proposed `0.0025` limit. The failure was not waived: the attitude block was
changed from first-order `I-[omega]dt` to exact discrete SO(3), and gyro-bias coupling now uses the
SO(3) right Jacobian.

### Mathematical and measurement changes

- `eskf_models.c` is the single internal implementation of F, Q, trusted-heading geometry, and
  magnetic-heading geometry used by both production code and numerical tests.
- Position propagation includes velocity, attitude, and accelerometer-bias `dt²/2` couplings.
- Continuous IMU white-noise densities retain the reviewed `sigma² dt` mapping with integrated
  acceleration velocity-position cross covariance.
- Scalar observations use the one-degree-of-freedom 99.7300204% NIS limit `9.0`; vector
  position/velocity/ZUPT observations use the three-degree-of-freedom limit
  `14.1564136091267`. Reported test ratio is NIS divided by the applicable threshold.
- Heading and magnetic geometry explicitly report unobservable near-vertical body-forward or
  horizontal-field geometry rather than emitting a misleading correction.

The final 10,000-case-per-family executable campaign reports:

- transition F maximum absolute finite-difference error: `1.61851333e-07` across all 15 columns;
- trusted-heading H maximum absolute finite-difference error: `2.16205933e-08`;
- magnetic-heading H maximum absolute finite-difference error: `1.64858118e-08`;
- process-noise blocks remain finite, symmetric, positive, and positive semidefinite across the
  randomized range.

### Recovery and continuous validity

Automatic state reset after a rejection count was removed. Standalone position or velocity
observations can never force re-anchoring. A paired GNSS recovery candidate now requires bounded
variance and multiple kinematically consistent samples; the exact candidate timestamp then requires
one-shot application authorization with independently verified source quality and correction-size
bounds. Successful recovery preserves attitude and IMU bias, enters probation, and leaves
navigation invalid until subsequent accepted paired updates complete the dwell.

Heading, vertical-position, and vertical-velocity aiding now have separate age and validity outputs.
Startup alignment completion is retained as historical state but can no longer be mistaken for
continued observability. Tests cover initial invalidity, independent qualification, timeout, failed
authorization, excessive correction, accepted bounded recovery, and probation exit.

### Public capability and multi-rate test correction

Public dataset reproduction baselines remain change detectors, while declared one-way minimum or
maximum capability gates can independently fail a stable result. The V1_03 difficult derived-
heading track must meet full-attitude, observable-yaw, fault-rejection, and recovery-time gates; its
reference-bias track separately gates attitude, tilt, and navigation NEES.

The first multi-rate gate was intentionally rejected because it compared one seed while keeping
per-sample IMU noise variance constant and increasing magnetometer updates with IMU rate. That made
the physical noise process and aiding bandwidth differ between 100 and 1000 Hz. The corrected gate:

1. uses interval-average gyro samples whose delta angles exactly reconstruct the reference pose;
2. checks deterministic discretization independently;
3. fixes magnetometer and GNSS publications at 100 Hz and 10 Hz;
4. scales IMU sample noise with square root of rate to preserve continuous white-noise density;
5. compares paired multi-seed statistical aggregates for attitude, navigation, NIS, and NEES.

The first 1,000-trial campaign then exposed a validation-runner orchestration failure at seed 778.
The exact seed replayed successfully when isolated, so this was not recorded as an estimator hang.
The campaign tool was corrected to create a per-seed checkpoint, use an isolated configuration
directory, enforce a timeout, retry a bounded number of times, retain execution-failure text, and
resume without discarding hundreds of completed independent trials. Final campaign and total
regression results are recorded in the continuation below after the corrected rerun completes.

### Corrected 1,000-trial result and frozen statistical gates

The corrected run completed seeds 0--999 with zero execution failures. The earlier 20-seed
thresholds were also found to be incorrectly interpreted as absolute maxima: when sampling 1,000
independent Gaussian-noise/bias trials, expected distribution tails occasionally exceeded those
small-sample maxima without numerical or consistency failure. Before future blind use, the G0 gate
was frozen as two levels:

- distribution P95: position `<=0.75 m`, velocity `<=0.40 m/s`, attitude `<=2.0 deg`;
- per-trial hard envelope: position `<=1.25 m`, velocity `<=0.60 m/s`, attitude `<=3.5 deg`;
- numerical health must remain 100% and unexpected navigation recovery remains zero-tolerance;
- aggregate NIS/NEES means must remain within the declared consistency bands.

Final results:

| Metric | Mean | P05 | P95 | Maximum |
| --- | ---: | ---: | ---: | ---: |
| position RMSE | 0.3921 m | 0.2178 m | 0.6495 m | 1.0266 m |
| velocity RMSE | 0.1731 m/s | 0.0887 m/s | 0.2984 m/s | 0.4571 m/s |
| post-alignment attitude RMSE | 0.9798 deg | 0.4992 deg | 1.7272 deg | 2.6924 deg |
| position NIS mean | 2.9628 | 2.6550 | 3.3075 | 3.6510 |
| velocity NIS mean | 2.5173 | 2.2371 | 2.8153 | 3.0887 |
| six-state navigation NEES mean | 4.9828 | 3.4932 | 7.0346 | 10.2682 |

All 1,000 states/covariances remained healthy and no navigation recovery was invoked. The expected
means are 3 for each three-dimensional NIS and 6 for navigation NEES; the result is mildly
conservative, not divergent or overconfident. These remain synthetic evidence and do not replace
physical truth, temperature, vibration, or target execution tests.

### Final multi-rate result

The formal gate uses the same 32 fixed seeds at 100/200/400/1000 Hz. A small seed count's median
was rejected as the cross-rate statistic because the RMSE distribution is skewed and its median
varied 6.3% even while per-rate means differed only 1.1%. The frozen gate uses the paired seed-set
mean and retains a 5% maximum cross-rate spread plus independent absolute capability limits.

- attitude RMSE means: `0.6694 / 0.6766 / 0.6720 / 0.6751 deg`;
- position RMSE means: `0.3775 / 0.3774 / 0.3770 / 0.3777 m`;
- velocity RMSE means: `0.1661 / 0.1658 / 0.1662 / 0.1667 m/s`;
- position NIS means: `2.9247 / 2.9247 / 2.9246 / 2.9247`;
- velocity NIS means: `2.5270 / 2.5267 / 2.5271 / 2.5271`;
- navigation NEES means: `4.8620 / 4.8628 / 4.8604 / 4.8601`.

All deterministic integration, statistical spread, and per-rate one-way gates pass. This closes
the earlier ambiguity between input-integrity rate coverage and actual estimator-accuracy
invariance.

### Final G0 verification checkpoint

The closure candidate was rebuilt and rerun after the model, generator, gate, and documentation
changes were complete:

- seven native CTest targets passed under repository-wide warnings-as-errors; the 10,000-case
  model campaign was part of that run;
- the same seven targets passed under AddressSanitizer and UndefinedBehaviorSanitizer, including
  the 1,020,000-attempt input-integrity campaign;
- 42 Python converter, analyzer, manifest, gate, and orchestration tests passed;
- seven deterministic synthetic scenarios passed every reviewed threshold, including cold start,
  magnetic spike/bias, five-second GNSS outage/reacquisition, trusted-heading faults/recovery, and
  online bias convergence;
- the 32-seed 100/200/400/1000 Hz gate passed both deterministic and statistical limits;
- the corrected 1,000-seed campaign completed with zero execution, hard-envelope, distribution,
  consistency, health, or unexpected-recovery failures;
- all six EuRoC tracks passed baseline reproducibility and declared one-way capability gates,
  replaying 156,490 IMU samples in the final run.

The final deterministic outage result is `0.507 m` position RMSE, `0.231 m/s` velocity RMSE, and
`0.686 deg` post-alignment attitude RMSE. The online-bias scenario ends at `0.030 m/s²`
accelerometer-bias error and `0.000522 rad/s` gyroscope-bias error while retaining navigation NEES
`4.998`. These are host/synthetic acceptance figures, not target-hardware or flight claims.

The first remote G0 candidate exposed a Windows-only warnings-as-errors failure: nine innovation
diagnostic assignments narrowed the internal `eskf_float_t` values into the public float result
structure implicitly. The Windows log reported MSVC `C4244`; Ubuntu and macOS had already passed.
The API boundary now uses explicit float conversions while all NIS and innovation calculations
remain in `eskf_float_t`. A local clang-cl `/W4 /WX` compile of all C sources and the focused native
core/model/public-API tests passed before the correction was republished.

The next remote Windows build advanced past the library but still failed later in the all-target
build. A stricter local `-Wconversion -Werror` audit of every source, test, and validation C file
then exposed four enum-complement signedness conversions and one integer-to-double timestamp
conversion in the public-API/input-integrity tests. Those test-harness boundaries now use explicit
`uint32_t`/`double` conversions, and the complete conversion audit passes with zero diagnostics.

## 2026-07-18 — independent G0 review, failed confirmation, and corrected confirmation

### Why G0 was reopened

The first closure checkpoint was given to an independent validation-gap review. It identified four
ways a green suite could still overstate capability: recovery authorization was not bound to one
physical source generation and one quality decision window; position and velocity reused one
horizontal-validity timestamp; Q/H claims exceeded the actual executable oracle; and the first
1,000 Monte Carlo seeds were development-visible.

### Recovery and validity correction

Recovery observations now carry `source_id`, `source_generation`, and `quality_sequence`.
`source_id == 0` may still use ordinary fusion but can never form a recovery candidate. The default
5--10 Hz recovery profile requires three consistent paired samples spanning at least 0.20 s, no
sample gap above 0.30 s, authorization no more than 0.25 s after the exact candidate, bounded
position/velocity correction, and three same-source probation accepts spanning at least 0.20 s.
Any source/generation/quality change, rejection, reorder, or excessive gap restarts the applicable
window. Tests include stale authorization, source/generation/quality mismatch, candidate gap,
probation source switch, burst-without-duration, probation gap, and exact dwell completion.

Horizontal position and velocity now carry independent accepted-aiding timestamps, ages, timeouts,
and validity. Position-only, velocity-only, paired, rejected, and ZUPT cases are tested separately.
The old summary age remains the maximum of the two required ages for conservative telemetry; flight
qualification must use the individual validity booleans.

### Q scope and heading-model audit

The randomized process-noise gate evaluates all 225 entries against a complete structure oracle,
including the integrated acceleration velocity-position block. It checks every entry finite, full
symmetry, zero-noise output, and full-matrix positive semidefiniteness. A separate RK4 integration
of `dQ/dt = A Q + Q A^T + W` matches to `5.2042e-18`, but that continuous oracle deliberately covers
only the declared reduced within-step model: direct IMU/bias white noise plus velocity-position
integration. It does not close every higher-order transition/bias coupling. The production Q remains
a tested high-rate approximation, not an exact full-model discretization claim.

The first correction replaced the old projected heading H with the complete raw body-X heading
Jacobian for both trusted and magnetic heading. Its 10,000-case finite-difference error was below
`1.0e-8`, but the deterministic clean/magnetic ESKF tracks regressed to `4.594°`, `2.631°`, and
`9.580°`: scalar magnetic heading began injecting model error into unobserved tilt. That failed
track is retained; the implementation was not accepted merely because the Jacobian test was green.

The final split uses the complete raw Jacobian for trusted dual-GNSS/vision-style heading. The
magnetometer path is explicitly a tilt-conditioned local NED-yaw pseudo correction with
`R_nb^T e_D`; its tuning variance and pseudo-NIS are not presented as a general physical-heading
measurement at arbitrary tilt. The raw-heading finite-difference maximum is `1.1054e-08`, the
magnetic pure-yaw directional error is `1.2102e-08`, and finite yaw corrections preserve gravity
direction within `8.8818e-16`. The seven deterministic scenarios again pass all reviewed gates.

### Consumed failure: unbounded startup-bias confirmation

After freezing the existing limits, seeds 10000--10999 ran the complete 20 s cold-start,
translation, 5 s GNSS outage, and reacquisition scenario. All 1,000 executions completed, but seed
10988 reached `3.715137°` attitude RMSE against the `3.5°` per-trial hard limit. It had drawn
`0.199252 m/s²` residual y-axis accelerometer bias from the unbounded Gaussian. Health remained
100%, position/velocity RMSE were `0.598 m / 0.300 m/s`, navigation NEES was `4.451`, and no recovery
occurred. The failed range is retained and marked consumed.

The diagnosis is an observability boundary, not a reason to widen the accuracy gate. The scenario
keeps attitude nearly fixed; one static gravity direction cannot distinguish horizontal bias from
tilt. FCOne's public algorithm input is already specified as calibrated SI data, so the statistical
profile was revised to declare rather than imply a calibration envelope.

### Revised profile and new confirmation

The provisional pre-hardware input profile now truncates each constant startup residual-bias
component at three standard deviations: `0.15 m/s²` accelerometer and `0.6 deg/s` gyroscope under
the existing priors. Rejection sampling preserves the Gaussian shape inside the declared envelope;
metadata records the limit and actual vector. All development seeds whose old draws exceeded the
new envelope, plus seed 10988, passed before opening a new range.

Seeds 20000--20999 then produced:

| Metric | Mean | P95 | Maximum | 99% bootstrap P95 upper |
| --- | ---: | ---: | ---: | ---: |
| position RMSE | 0.3930 m | 0.6392 m | 0.9156 m | 0.6830 m |
| velocity RMSE | 0.1739 m/s | 0.2973 m/s | 0.4318 m/s | 0.3110 m/s |
| attitude RMSE | 0.9624 deg | 1.7120 deg | 2.9810 deg | 1.8594 deg |

Position NIS mean was `2.9750`, velocity NIS mean `2.5269`, and six-state navigation NEES mean
`5.0243`. Their 99% bootstrap mean intervals were `[2.9575, 2.9917]`, `[2.5118, 2.5411]`, and
`[4.9328, 5.1202]`. All 1,000 states were healthy, no navigation recovery occurred, and there were
zero execution, hard, distribution, aggregate-consistency, or bootstrap-bound failures. With zero
failures, the exact 95% binomial upper failure probability is `0.00299125`.

The bound remains a pre-hardware hypothesis. FCOne v2 static multi-orientation and thermal tests
must verify or replace it; a hardware violation fails the profile instead of silently expanding the
host gate.

### Clean post-audit confirmation

After the audit implementation was fixed at commit `5109568`, seeds 40000--40999 were opened as a
new confirmation range. The protocol fingerprint recorded `git.dirty=false` and bound the exact
runner, generator, analyzer, thresholds, gate policy, timing, noise, bias profile, and commit.
All 1,000 trials completed with zero execution, hard-envelope, distribution, aggregate-consistency,
health, recovery, or 99% bootstrap failures. Position/velocity/attitude RMSE P95 were
`0.6361 m / 0.2915 m/s / 1.7041°`; mean position NIS, velocity NIS, and six-state navigation NEES
were `2.9560 / 2.5318 / 4.9758`. The protocol semantic SHA-256 is
`4b335a794546866e8dbc8e5787723ad0b23cc78e088cc966d6af4a0bc8e10789`, and the committed summary
records the protocol-file and result-file hashes.

This is a navigation-outage Monte Carlo confirmation, not a substitute for blind VTOL
bias-observability holdout evidence. A later protocol audit found that the historical v1 smoke
selector had already opened seed `30000` in both v1 holdout trajectory families. V1 therefore
remains diagnostic evidence only. Final evidence requires the replacement sealed v2 holdout after
a candidate and its gates are frozen.

### Post-confirmation audit closures

The probation source/generation/quality check was found to occur after the core GPS update. A wrong
source could therefore modify state and covariance even though it did not advance probation. The
check now runs before fusion. Three mismatch cases assert byte-identical adapter state, nominal
state, full covariance, innovations, counters, and aiding timestamps.

Monte Carlo resume checkpoints now carry one canonical SHA-256 protocol fingerprint covering the
scenario, duration/rate/jitter, all sensor noise, bias distribution and bound, generator,
orchestrator, analyzer, runner binary, deterministic thresholds, threshold checker, gate policy,
Git commit, and dirty state. Resume rejects missing or mismatched directory and per-seed
fingerprints instead of silently mixing the old unbounded and new bounded protocols.

The breaking adapter changes are versioned as public API `0.3.0`. The migration note records split
horizontal position/velocity validity, source-stamped recovery observations, exact authorization
binding, and fail-closed probation. A compile-time version test is part of CTest.

### Deterministic residual-bias boundary campaign

Random truncation alone did not prove the six-dimensional three-sigma box. A new deterministic
campaign therefore covers zero bias, all six axes at both signs, all 60 signed isolated pairs, and
all 64 six-axis corners: 137 cases total. The first run passes 120/137 strict gates. All cases remain
100% numerically healthy with zero navigation recovery; worst attitude/position/velocity RMSE are
`1.178° / 0.179 m / 0.079 m/s`. The 17 failures share accelerometer `+X/-Y`: settling below
`0.05 m/s²` takes 36.4 s versus the frozen 35 s budget.

A paired 32-seed follow-up retains the same physical bias vectors and matched noise streams. The
`+X/-Y` direction misses in 22/32 trials, the mirrored direction in 8/32, and zero injected bias in
3/32. Fifteen target-direction trials remain above `0.05 m/s²` at 40 s; final target-direction
error has median/P95/maximum `0.0486 / 0.0650 / 0.0840 m/s²`. All 96 trials remain healthy with
zero recovery. This establishes a real direction-sensitive convergence/observability gap; the
35 s gate is retained rather than widened after seeing the data.

### Trusted-heading model and EuRoC rebaseline

Trusted heading now uses the complete raw right-error Jacobian while the magnetic path remains the
explicit yaw-only pseudo correction described above. Six EuRoC tracks replay 156,490 IMU samples.
Five unaffected baselines reproduce; the derived-heading track changes as expected and still passes
all four one-way capability gates. After review, its new baseline is `1.549°` post-alignment
geodesic attitude, `0.920°` observable yaw, `99.7436%` normal heading acceptance, `5.100°` maximum
four-second-outage yaw error, `0.995 s` recovery, and `0.890°` post-recovery yaw RMSE. The source is
still derived Vicon yaw, not a recorded heading sensor.

## 2026-07-18 — G0 horizontal-bias causal audit and independent VTOL cross-validation

The 137-case boundary failure was not treated as a reason to increase the 35 s gate. A paired
32-seed causal audit showed that one-pose alignment maps the `0.15 m/s²` horizontal residual into
about `0.876°` of tilt, as predicted by `bias/g`. Replaying the same 96 inputs with correct initial
attitude while retaining sensor-derived bias initialization reduced the three groups to nearly the
same final distribution: median bias error about `0.0184 m/s²`, P95 about `0.0374 m/s²`. Together
with near-odd-symmetric sign response, this rejects an axis-sign bug as the leading cause.

The runner first exported both complete 3x3 bias covariance blocks. The analyzer added bias NEES,
per-axis/terminal-five-second error, continuous-five-second convergence, and right-censoring. The
focused `+X/-Y` replay gave accelerometer/gyro bias NEES means `0.782 / 0.113`; the accelerometer
bias remained right-censored at `38.99 s` while the gyro reached its continuous window at `19.10 s`.
The subsequent joint-consistency extension exports the exact 5x5 marginal covariance for
`[δθx, δθy, δba_x, δba_y, δba_z]` and evaluates `Log(q_estimate^-1 q_truth)` in the same right-error
tangent frame. Yaw is excluded rather than falsely scored when no trusted heading is observable.
A seed-7, 4000-sample review with explicit `+0.15/-0.15/0 m/s²` accelerometer bias completed with
zero malformed samples or recovery and produced joint NEES mean `0.768`, terminal-five-second mean
`0.278`, and zero invalid covariance samples. The low score confirms conservative covariance but
does not remove the retained convergence failure.

A new v1 protocol then replaced reliance on the old multisine with five minimum-jerk VTOL
profiles: hover axis pulses, takeoff-box-land, yaw-quadrant hover, early-transition S-curve, and
landing gust recovery. It freezes nine horizontal bias vectors, train seeds 0--15, tune seeds
1000--1031, and designated holdout seeds 30000--30063, totaling 1,728 release trials. It
uses interval-start ZOH acceleration/truth, sensor-only cold start, protocol/source/binary hashes,
and per-trajectory/vector gates.

The 15-trial smoke completed with zero execution failures. Capability failed on all ten boundary-
bias trials and passed on all five zero-bias trials. Boundary terminal horizontal-bias P95 ranged
from `0.0595` to `0.2154 m/s²`; all ten were right-censored. The report status was changed from a
potentially misleading generic pass to separate `execution=passed` and `capability=failed` fields.
No `release` campaign was run. However, the original smoke selector sampled seed `30000` from every
trajectory, including both v1 holdout trajectory families. Six holdout-family trials were therefore
inspected during smoke. This contamination was discovered later; it invalidates v1 as blind final
holdout evidence even though the explicit release mode remained unused.

The high-level adapter also gained one explicit, validated `process_noise` profile so private
FCOne VTOL/transition/fixed-wing configurations can be named and pinned without forking algorithm
source. Invalid profile fields fall back atomically to the public default; zero remains an explicit
valid value. PX4 numerical values are not copied because the two implementations use different
discrete process-noise semantics.

## 2026-07-18 — G0 train/tune accelerometer-bias candidate study

The estimator source was fixed at commit `5109568`, and three temporary wrapper-built runners were
used only to study the current post-alignment accelerometer-bias covariance (`0.04`), a broader
covariance (`0.09`), and that broader covariance with `sigma_acc_bias` increased from `0.001` to
`0.003 m/s3/sqrt(Hz)`. The wrapper applied the covariance immediately after static alignment; no
candidate changed the estimator core or became a product configuration.

All 324 scheduled runs completed: three train/tune trajectories, four frozen seeds per split, nine
bias vectors, and three candidates. This candidate study did not schedule or access holdout trials,
but seed `30000` in both v1 holdout trajectory families had already been exposed by the historical
smoke. Baseline passed 23/108 with 81 right-censored trials. The broader covariance passed 33/108
with 73 right-censored trials, but all 64 non-zero train trials still failed and remained censored;
its terminal horizontal-error P95 tail and maximum worsened from `0.21698 / 0.22511` to
`0.22070 / 0.23479 m/s²`, and maximum attitude RMSE rose from `2.336°` to `2.396°`. The larger
random-walk noise produced the same pass/censor counts and only a `-0.000015 m/s²` mean terminal-P95
change relative to the broader-prior candidate.

Both candidates were rejected. The result points to unresolved tilt--horizontal-bias observability
coupling rather than a scalar covariance/noise setting that can safely close G0. The compact,
reviewable evidence is committed as
[`validation/public/g0_bias_candidate_study.json`](../validation/public/g0_bias_candidate_study.json);
the temporary wrappers and generated trial files remain uncommitted research artifacts.

## 2026-07-18 — PX4 M0 fail-closed comparison skeleton

The M0 protocol freezes PX4 commit `de8158101c96ad6b04170dc91f087148104c58eb`, five audited
source-file hashes, stock estimator profiles, one canonical physical-input schema, one delayed-core
fusion-horizon output schema, and mandatory provenance sidecars. The runner has no download or
fallback path and always emits a status manifest. Bias-validity flags now participate in the score:
invalid samples cannot satisfy the continuous settling gate, and coverage/NEES require positive
variance as well as a valid bias estimate.

The local FCOne v1 PX4 tree was inspected read-only at `08bb3acfd004aa9f6d26924a0e9f632c4877b304`,
which does not match the frozen M0 commit. The preflight therefore failed closed before comparing
outputs. Missing canonical exporters remain an explicit blocker, and no PX4 parity number was
generated or inferred.

## 2026-07-18 — GCC 14 strict-C99 CI compatibility

The first remote CI attempt at commit `18e5968` failed before sanitizer execution. Reproduction on
GCC 14 identified a strict-C99 diagnostic in two test-only helpers: converting writable 15x15
arrays to pointers-to-arrays with added `const` qualifiers is not permitted before C23 under
`-pedantic -Werror`. GCC 15 and Clang 21 accepted the code and had hidden the portability failure.

The two helper signatures were made C99-compatible without changing estimator code. Non-MSVC CMake
builds now require C99 with extensions disabled, and the sanitizer job pins GCC 14. The reviewed
GCC 14 Release build shows explicit `-std=c99 -pedantic -Werror` and passes all nine CTest targets;
the current-tree GCC 14 ASan/UBSan build also passes all nine targets. The 1,020,000-attempt input
integrity campaign took `183.84 s`, and the complete sanitizer suite took `196.02 s` with zero
failures.

## 2026-07-18 — integrated G0 regression confirmation

The integrated candidate tree was rebuilt and exercised after the joint-consistency, M0 protocol,
and GCC 14 portability changes. The hardware-free host regression passed all nine CTest targets,
all 69 Python tests, the 1,020,000-attempt malformed/input-integrity campaign, and all seven frozen
deterministic scenarios. Representative ESKF geodesic attitude RMSE was `0.311°` for the clean
scenario, `0.331°` under a magnetic spike, `0.327°` under sustained magnetic bias, and `0.686°`
after alignment in the navigation-outage scenario; the latter also reported `0.507 m` position and
`0.231 m/s` velocity RMSE. Trusted-heading recovery reported `0.228°` post-recovery yaw RMSE.

The six-track EuRoC suite replayed 156,490 IMU samples and passed every frozen baseline and one-way
capability gate. The difficult Vicon track retained `2.572°` reference-bias-corrected ESKF
geodesic attitude RMSE, `0.829°` tilt RMSE, and navigation NEES `4.950`. The derived-heading track
retained `1.549°` post-alignment geodesic attitude and `0.920°` observable-yaw RMSE, with `0.995 s`
recovery. This remains Vicon-derived heading evidence, not a physical heading-sensor dataset.

A final focused seed-7 bias replay after rebuilding the runner completed all 4,000 samples with
zero malformed samples and zero navigation recovery. Its five-dimensional tilt/accelerometer-bias
joint NEES mean was `0.768` (expected 5), with terminal-five-second mean `0.278` and no invalid
covariance. The track converged in `28.5 s`, but this individual pass does not override the failed
multi-trajectory train/tune study or authorize a release claim. Any future release claim must use
the sealed v2 holdout plan after the candidate, thresholds, and immutable inputs are fixed.

## 2026-07-20 — G0 static-prior paired A/B study and provenance hardening

### Research question

The retained G0 failure is direction-sensitive convergence of horizontal accelerometer bias after
cold-start tilt alignment. The question was whether a physically motivated static-prior change could
reduce the coupled tilt/bias error without changing estimator equations, widening gates, or using
truth-assisted initialization.

### Frozen protocol and scale

The paired comparison used the frozen v1 train/tune trajectory and bias-vector definitions, identical
inputs and seeds per pair, and no holdout access. Each arm executed `576` trials (`1,152` total):
three train/tune trajectories, disjoint split seeds, and the complete declared residual-bias set.
Right-censored non-convergence remained a capability failure. Execution success and capability gates
were reported separately.

### Results

| Arm | Executed | Passed | Failed | Right-censored | Execution failures |
| --- | ---: | ---: | ---: | ---: | ---: |
| Frozen baseline | 576 | 175 | 401 | 390 | 0 |
| Static-prior candidate | 576 | 231 | 345 | 341 | 0 |

The candidate moved `49` paired trials from right-censored to settled and produced zero
baseline-pass-to-candidate-fail regressions. The improvement was tune-only: non-zero-bias train
passes remained `0/16` in both arms, while zero-bias train passes remained `16/16` in both arms.
The candidate was rejected and no scalar static-prior or process-noise adjustment was promoted to
the estimator or FCOne configuration.

### Interpretation and negative conclusion

The evidence is consistent with unresolved tilt--horizontal-accelerometer-bias observability
coupling under the current excitation and covariance model. It does not prove universal
mathematical unobservability, and it does not justify widening accuracy/settling gates. The next
experiment must be a materially different, pre-registered excitation-aware estimator hypothesis
under a clean v2 protocol; repeated scalar P/Q tuning is not an accepted closure path.

### Publication and sealing limits

This A/B study is diagnostic rather than publication-grade blind evidence. The historical runs used
compact mode from a dirty tree; their summaries predated per-trial input SHA-256, byte count, and
row-count provenance; and the v1 smoke had exposed seed `30000` from both holdout trajectory
families. No missing historical SHA values were fabricated. The public aggregate record is
[`validation/public/g0_static_prior_ab_study.json`](../validation/public/g0_static_prior_ab_study.json).

The runner now captures input SHA-256, byte count, and data rows before compact mode deletes
`input.csv`/`results.csv`, retains historical `input_sha256` compatibility, and fingerprints the
protocol plus information-marker manifest. The marker manifest is still `draft` and analyzer-only;
it is not an observability proof, rank test, estimator input, or truth-assisted initializer.

A sealed release holdout without a public marker continues to execute ordinary metrics and gates and
is classified as `unavailable_sealed_holdout`. It is not an execution failure and is not assigned a
public information-state label. Any future v2 protected marker manifest must live in a separate
protected-CI path.

## 2026-07-20 — heading, excitation, and degraded-navigation design audit

### Research questions

The audit asked whether the existing public corpus contains independent absolute-yaw evidence,
whether the G0 information marker is a product-usable excitation detector, and whether FCOne v2
barometer/airspeed inputs can improve GNSS-denied navigation.

### Findings

The corpus contains independent motion-capture yaw truth and a physical dual-RTK heading input, but
not yet a completed end-to-end track where a physical heading observation is scored against an
independent yaw truth. EuRoC/Blackbird are Class B propagation evidence, Vicon-derived heading is
Class C fusion-path evidence, and INSANE is Class D shared-source physical-input evidence. PX4
attitude and course over ground remain diagnostic references only. The evidence classes and Class A
intake requirements are recorded in `docs/absolute-heading-evidence.md`.

The G0 `information_ready_time_s` marker is a predeclared trajectory milestone, not a rank or
observability test. A product candidate must use a causal sliding-window local information matrix
for the tilt/accelerometer-bias subspace, pass negative controls, and report unresolved status when
directional excitation or accepted aiding is insufficient. The input audit also retained the
interval-semantics, truth-derived-stationarity, idealized-GNSS, sample-noise, single-rate, and
missing-physical-error limitations. No estimator candidate was promoted.

The A/B evidence pipeline was hardened before further candidate work. New campaigns explicitly
record the analyzer SHA-256. The comparator now requires identical information-marker semantic
hashes and fails closed when zero-bias or global regression evidence is missing, instead of treating
missing values as no regression. Historical missing provenance is not fabricated.

Barometer support currently provides only timestamped scalar relative-height fusion. Airspeed and
wind states do not exist in the core or runner, so no existing outage result includes their benefit.
The planned paired study covers IMU-only, barometer, TAS with known wind, TAS with estimated wind,
and the combined configuration across 5/10/30/60/120-second outages and declared VTOL/fixed-wing
regimes. The plan is recorded in `docs/airspeed-barometer-plan.md` and
`validation/airspeed_barometer_validation_plan_v1.json`. It does not claim long-duration
GNSS-independent position.

### Verification

`python3 -m unittest tests.test_bias_observability_comparison` completed 11 tests successfully,
including new information-marker mismatch and missing-metric fail-closed cases. The complete
`python3 validation/run_host_regression.py` gate then passed: all 9 CTest targets, all 89 Python
tests, the 1,020,000-attempt input-integrity campaign, all deterministic scenarios, and every frozen
threshold completed without failure. The generated manifest records GCC 15.2.0, CMake 4.2.3,
Python 3.14.4, commands, timings, current commit, and the intentionally dirty documentation branch.

## 2026-07-20 — causal G0 information analyzer and barometer fault study

### G0 information analyzer

The fixed trajectory-time information marker was not promoted. A new analyzer reconstructs the
current 15-state local transition using estimated attitude/bias, IMU samples, and only actually
accepted position/velocity observations. After whitening, it projects out ten nuisance-state
directions and scores `[tilt_x, tilt_y, accel_bias_x, accel_bias_y, accel_bias_z]`.

Sixteen focused controls initially passed. Static/fixed-attitude constant velocity, yaw-only, and
single-direction controls remain below full rank; attitude/rate-consistent multi-direction motion
reaches `5/5`; rejected aiding contributes rank `0/5`. Static and constant velocity deliberately
have identical observable IMU channels; this records inertial indistinguishability rather than two
independent measured tracks.
stale aiding, too few unique epochs, and a less-than-five-second window cannot report structural
readiness; the post-update observation at the start of a window is excluded; and future-sample
changes do not alter a closed past window. A subsequent provenance audit found that exact
replay/result hashes alone could not prove the producer's causal semantics. Schema v2 therefore
also binds the replay generator, runner, command, and Git commit and rejects known truth-init
options. The remaining causal fields are explicitly producer assertions, not attested proof. The existing
bias-convergence replay first met the diagnostic checks at `5.0 s`, but its historical
truth-derived static hint prevents promotion under the new causal provenance contract.

The report schema is now `2`; generic `information_ready` names were removed in favor of explicit
`*_analyzer_only` fields. This is analyzer-only evidence: thresholds are not frozen, process
noise, bias random walk, preintegration covariance, aiding correlation, and error-reset Jacobians
are not included, and no estimator correction is enabled.

### Barometer instrumentation and paired study

The replay runner now preserves optional physical `baro_timestamp_us` and exports adapter status,
age, innovation, variance, NIS, test ratio, supervisor decisions, fault flags, and latch state. A
rejected timestamp/value call clears per-call acceptance. With correct timestamps, all `0.60 s`
delayed samples were rejected before fusion by the existing age gate.

The first 2-seed, 5/30-second smoke contained 20 paired fault/outage trials. At 30 seconds, nominal
barometer aiding reduced vertical-position RMSE from `0.4695 m` to `0.1004 m`. Constant datum bias,
weather step, and freeze worsened it to `1.0155 m`, `1.3552 m`, and `2.9543 m`, while numerical
health remained 100%. Core NIS alone therefore does not establish source trust.

A causal source supervisor was added as an opt-in module, leaving the production ESKF default
unchanged. It detects exact frozen output in about `0.15 s`, but the three pre-latch updates already
alter position, vertical velocity, accelerometer bias, and covariance. A single supervised lane
therefore remained worse than IMU-only. An offline hot-shadow output mux restored the 30-second
freeze RMSE to `0.4600 m` and weather-step RMSE to `0.4348 m`, close to the `0.4695 m` IMU-only
baseline. This is an architecture upper bound, not a real-time FCOne implementation.

An exploratory `2 sigma` jump threshold then ran 60 train trials: 10 seeds, 5/30-second outages,
and nominal/weather-step/freeze groups. It switched on all 20 weather-step trials but also all 20
nominal trials; 30-second nominal supervised acceptance fell to about 25%. The candidate was
rejected and the source restored to `3 sigma`. Persistent datum bias remains unobservable without
an independent height reference. Full 5--120-second, 100-seed train/tune campaigns are deferred
until a sequential detector and real multi-lane state policy are implemented.

### Verification at this checkpoint

The strict GCC 15.2/C99 build passes all 10 CTest targets, including the new barometer-supervisor
unit target and the 1,020,000-attempt input-integrity campaign. After the causal-provenance audit,
focused G0 tests pass `16/16`; the full Python suite passes `121/121`; `py_compile` and
`git diff --check` pass. Generated reports and raw dataset files remain outside Git, while methods,
negative results, code, tests, and capability limits are retained.

## 2026-07-20 — independent magnetic-yaw intake and post-review hardening

### Purpose and anti-leakage protocol

INSANE `indoor_1` was selected because it records a physical PX4 magnetometer and an independent
raw OptiTrack vehicle pose. Generated INSANE `ground_truth` products were excluded. The common raw
overlap was frozen into calibration `[10,110) s`, development `[110,210) s`, and holdout
`[210,310) s`. Calibration estimated time offset, rigid IMU/mocap rotation, and a local magnetic yaw
datum. Development and holdout never updated those values.

An independent review found that reference-attitude tracking in the generic runner still generated
the ESKF magnetic reference from the first scored truth attitude and physical magnetometer sample.
That truth-coupled holdout self-calibration was removed. The runner now uses only the replay's
declared magnetic datum in both cold-start and reference-attitude-init modes. All retained metrics
below were regenerated after the correction.

### Calibration and data coverage

The calibration uses 6,732 excited angular samples. Its robust angular-rate residual is
`0.0629 rad/s`, norm correlation `0.951`, three-axis excitation singular-value ratio `0.388`, and
time-offset one-percent cost interval `[-6,+2] ms`. The protected `indoor_1` holdout contains 19,642
IMU samples and 8,776 physical magnetometer updates over 99.995 s. Frozen frame/time audit gives
angular-rate correlation/RMSE `0.911/0.0733 rad/s` and gravity-direction median/P95
`6.13°/12.57°`. The split begins in motion, so only one-time reference-attitude initialization is
scored; no cold-start claim is made.

| Frozen `indoor_1` holdout | Geodesic RMSE | Tilt RMSE | Yaw RMSE / P95 / max |
| --- | ---: | ---: | ---: |
| Robust Mahony | `7.653°` | `0.912°` | `7.599° / 11.243° / 11.835°` |
| Standard Mahony | `8.045°` | `1.237°` | `7.951° / 12.139° / 12.603°` |
| ESKF | `14.092°` | `6.737°` | `12.528° / 23.922° / 28.113°` |

ESKF health remains 100%. The outer magnetic-field gate passes 99.989%, and the innovation stage
accepts all 8,776 updates. A paired input changes only `mag_valid` and `mag_update`: ESKF
geodesic/tilt/yaw RMSE is `0.718/0.666/0.270°` with magnetometer disabled versus
`14.092/6.737/12.528°` with it enabled. Robust Mahony yaw similarly changes from `0.316°` to
`7.599°`. Missing position/velocity aiding remains a navigation boundary, but the A/B demonstrates
that physical magnetic fusion is the main observed degradation on this track. Field/datum quality,
the 3D observation model, and attitude/bias coupling remain under investigation.

The frozen calibration was opened once on raw `transition_1`. Relative angular-rate and gravity
audits passed over 7,520 IMU and 3,358 magnetometer updates in 38.131 s, but the first physical field
direction differed from the declared datum by `-49.298°` (`49.298°` magnitude). The separately
reported `49.359°` value is rejected-heading innovation P95, not the initial field residual.
Relative-motion and gravity audits cannot detect a constant mocap-world yaw rotation. This sequence
is therefore rejected as cross-sequence absolute-heading accuracy evidence. Its metrics and a paired magnetometer on/off diagnostic are
retained only for root-cause analysis; generated stitched truth remains unused.

### G0 analyzer post-review changes

The causal excitation tool now emits schema 2 fields explicitly named
`structural_information_ready_analyzer_only`. It excludes the window-start post-update observation,
counts paired position/velocity at one timestamp as one aiding epoch, and adds fixed-attitude
constant-velocity inertial-indistinguishability, stale-aiding, attitude/rate-consistent structural,
rejected-aiding, and future-isolation controls. Schema v2 binds exact replay/results, generator,
runner, command, and Git commit and rejects known truth-init command options. It still cannot attest
the semantics of an arbitrary producer binary, so the causal fields remain explicit assertions.
Process noise, preintegration covariance, aiding correlation, and ESKF reset Jacobians remain
omitted, so no estimator gate or coupled bias correction is enabled.

### Barometer post-review changes and final smoke

The supervisor now separates evaluation from commit: the fused residual baseline advances only
after ESKF core acceptance, while timestamp/freeze/fault classification history advances during
evaluation. Freeze recovery is sequential, while jump-latch return-to-baseline recovery requires an
application authorization bound to source, generation, quality, and time. The current contract
cannot adopt a persistent new datum. Quantized slow climb/descent negative
controls, persistent non-baro-row latch logging, stale acceptance clearing, strict JSON, and
fail-closed shadow preconditions were added.

The final two-seed, 30-second, six-fault smoke completed 12 matched four-arm trials:

| Fault | IMU only | Raw baro | Supervised | Shadow mux |
| --- | ---: | ---: | ---: | ---: |
| Nominal | `0.4695 m` | `0.1004 m` | `0.0997 m` | `0.0997 m` |
| Random walk | `0.4695 m` | `0.1517 m` | `0.1514 m` | `0.1514 m` |
| Constant datum bias | `0.4695 m` | `1.0155 m` | `1.0147 m` | `1.0147 m` |
| Weather step | `0.4695 m` | `1.3552 m` | `0.4522 m` | `0.4348 m` |
| Frozen output | `0.4695 m` | `2.9543 m` | `1.8075 m` | `0.4600 m` |
| `0.60 s` delay | `0.4695 m` | `0.4695 m` | `0.4695 m` | `0.4695 m` |

The shadow input audit now compares every CSV field and permits only the declared `baro_update`
difference; the command audit derives common runner/mode configuration instead of hard-coding two
truth values. The shadow result is explicitly a partial-output offline upper bound: full nominal
state, covariance, source generation, reset counter, and controller handoff are not verified. Constant
datum bias remains unobservable. The earlier `2 sigma` threshold candidate remains rejected because
it switched all 20 nominal training trials. Source-generation reset and authorized recovery remain
unit-test-only; the four-arm campaign does not exercise them end to end.

### Verification at this checkpoint

G0 focused tests pass `16/16`; barometer focused Python tests pass `10/10`; the new INSANE
calibration/converter/A-B tests pass `14/14`. The complete Python discovery passes `129/129` and a
fresh strict C99 CTest passes `10/10`. The complete host regression also passes all 129 Python tests,
all 10 CTest targets, the 1,020,000-attempt input-integrity campaign, every deterministic scenario,
and every frozen threshold. A fresh ASan/UBSan build passes `10/10`; its input campaign takes
`145.85 s` and the complete sanitizer suite `155.41 s`, with no sanitizer findings. Raw archives,
replays, and detailed result CSVs remain ignored build artifacts; aggregate JSON, code, tests,
methods, and negative decisions are committed together at this checkpoint.
