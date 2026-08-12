# Validation execution log

This append-only log records why each hardware-independent validation step was started, what was
changed, which evidence was produced, and what remains unresolved. Generated raw outputs stay under
`build/`; reviewed conclusions and reproduction commands remain in Git.

## 2026-08-12 — G0 multi-pose static calibration candidate

### Reason

The existing one-pose startup cannot distinguish a horizontal accelerometer
bias from a small roll/pitch error. Earlier scalar-prior, process-noise, and
host replay-correction candidates were rejected for failing to generalize. The
next bounded hypothesis is an explicit preflight procedure with multiple
stationary gravity directions, before any ESKF stream exists.

### Changes

- Added a hardware-neutral C99 gravity-sphere solver for six or more stationary
  pose means, with gyro-bias averaging and fail-closed geometry, residual,
  stationarity, and gyro-scatter gates.
- Added a leave-one-pose-out influence gate so a materially corrupted pose is
  not silently absorbed by the all-pairs least-squares fit.
- Added a one-shot ESKF adapter API. It seeds biases before streaming, removes
  the accepted accelerometer seed before tilt alignment, preserves the seed
  against one-pose overwrite, and rejects repeated/late application without
  changing the ESKF image.
- Made both calibration CLI and replay runner enforce the exact CSV schema and
  integer `sample_count` contract. The campaign uses past-only causal IMU
  stationarity rather than a trajectory-derived static hint.
- Added C, Python, CLI malformed-input, influence, and provenance tests plus
  documentation of the per-IMU FCOne collection boundary.

### Evidence and decision

The opened-development campaign ran `548` paired trials: 137 boundary cases at
each of four seeds (`41001, 41003, 41009, 41021`), 100 Hz, 40 s, six pose means,
and 400 samples per pose. The baseline passed `246/548`; the multi-pose
candidate passed `548/548`, with `302` fail-to-pass improvements and `0`
pass-to-fail regressions. Estimated accelerometer-bias error norm was
`0.000685/0.001305/0.001305 m/s^2` mean/P95/maximum. The candidate therefore
shows a material **synthetic cold-start** improvement for the known constant
bias boundary.

This remains opened-development evidence. It does not close hardware accuracy,
temperature, scale/misalignment, vibration, three-IMU independence, physical
heading, GNSS-denied navigation, or flight authority. A new sealed synthetic
holdout and per-IMU FCOne six-pose/thermal/vibration collection are required
before changing public `main` or enabling the path in FCOne control code.

### Verification

The feature branch strict build passed all `18/18` CTest targets and `319/319`
Python tests. A fresh ASan/UBSan build passed all `18/18` CTest targets with
leak detection disabled for this environment. The full campaign output remains
outside Git under `/tmp/aerakia-multipose-548-causal/`; its compact protocol and
source manifest are recorded when the candidate is committed.

### Sealed-holdout preparation

The opened result is deliberately not promoted by re-running it with a new
label. `validation/multipose_static_calibration_protocol_v1.json` freezes a
new eight-seed `51001..51071` holdout, the complete 137-case residual-bias
matrix, six-pose/400-sample collection, causal quantized-IMU stationarity,
100 Hz/40 s replay, and every C/Python source input that can change the
result. The campaign runner compares its live source manifest against the
protocol and rejects a dirty worktree before it reads a sealed seed.

The holdout result is intentionally not written here yet. It must be executed
once from the clean commit that introduces the protocol; either outcome will
be retained with its source/binary/environment hashes. This prevents the
synthetic G0 result from becoming a moving target while FCOne v2 hardware is
still unavailable.

### Follow-up audit — end-to-end contract and outlier boundary

The public API test was tightened so it now constructs six raw stationary pose
means, calls `aerakia_static_imu_calibrate()`, and passes that actual result to
`aerakia_eskf_apply_static_imu_calibration()`. A hand-filled accepted result is
no longer sufficient for the integration test. The full native CTest suite
remains `18/18` passing.

The new diagnostic scan
`validation/scan_static_imu_calibration_outlier_boundary.py` was run with the
frozen default configuration. It found an asymmetric boundary: radial
contamination of `0.03 m/s²` is accepted with approximately `0.0150 m/s²`
fitted-bias error, while `0.04 m/s²` is rejected; tangential contamination of
`0.8 m/s²` is accepted with approximately `0.0163 m/s²` error, while `1.0
m/s²` is rejected. This confirms that the leave-one-out gate is not a general
outlier detector. The result is retained as diagnostic evidence and does not
justify threshold tuning or physical-calibration claims.

## 2026-08-11 — TAS/wind causal observability prerequisite

### Reason

FCOne v2 will have an airspeed source, but adding horizontal wind states before
proving source semantics and flight-regime observability would turn invalid
pitot/transition data into a covariance-confidence failure. The first task is
therefore not a 17-state implementation: it is an executable boundary for when
TAS plus GNSS velocity can, and cannot, claim two-dimensional wind information.

### Changes

- Added a frozen hardware-neutral TAS source contract with physical source and
  delivery timestamps, fresh GNSS Doppler-velocity timestamp/variance, regime,
  block/stall/wash, and sideslip qualification fields.
- Added separate known-wind upper-bound and truth-free causal two-state
  information/least-squares lanes. Truth is held outside the causal oracle and
  is read only by the final scorer.
- Required three direction clusters plus a 12-sample bootstrap after the third
  cluster, because two scalar range centres retain a mirror ambiguity despite
  local rank two.
- Added fail-closed source/NIS latch behavior and controls for no excitation,
  invalid regime, source faults, delay/reorder, GNSS loss, wind shear, TAS
  scale/bias, and vertical-wind model mismatch.

### Evidence and decision

At commit `127349485d4de3869bb1e8dfae904a0ebe7067ee`, all 15/15 frozen
deterministic cases passed over 736 source observations. The qualified
48-observation multi-heading case had hidden-truth terminal wind error
`0.22656 m/s`, information minimum eigenvalue `55.7742`, condition number
`1.9416`, and known-wind NIS mean `1.0413`; the TAS Jacobian finite-difference
maximum error was `2.87e-9`.

Straight flight remained unobservable. Hover, low TAS, transition, rotor
wash, sideslip, blocked/stalled pitot, and excessive delay rejected their
source data. GNSS outage aged to `stale_no_fresh_gnss_tas`; wind shear, TAS
scale/bias, and vertical-wind mismatch each reached the no-auto-recovery source
latch. A reordered source sample was rejected without state/information
mutation while the preceding valid estimate remained qualified.

This closes an A2.0 validation prerequisite only. The production ESKF/API is
unchanged, no airspeed or wind state is promoted, and the result does not prove
physical air-data quality or GNSS-denied navigation. The next gate is a
separate pre-registered physical fixed-wing train/tune/holdout comparison
against the frozen 15-error-state baseline.

### Verification

`python3 validation/run_airspeed_wind_observability.py` produced the compact
artifact [`airspeed_wind_observability_v1.json`](../validation/public/airspeed_wind_observability_v1.json)
with a clean source state and runner SHA-256. Focused Python tests passed 9/9.
A subsequent fresh host regression passed 17/17 CTests, 206/206 Python tests,
the 1,020,000-attempt input-integrity campaign, deterministic-suite generation,
and reviewed threshold checks. Its disposable manifest correctly records that
the evidence/documentation files were being prepared for commit; no executable
source changed between the compact oracle artifact and the regression.

## 2026-08-11 — multi-seed TAS/wind confirmation blocks vertical-wind promotion

### Reason

The single frozen source-contract fixture passed, but a single noise seed is
not enough to claim that residual NIS reliably catches air-data model mismatch.
The next step repeated every frozen case in two disjoint confirmation windows
without using the outcome to retune any rule.

### Method

The campaign fixed 32 seeds (`17001--17016`, `27001--27016`) and every one of
the 15 v1 cases. Four eight-seed build shards retained compact individual
records, and a merge checked exact non-overlap/completeness before producing a
public aggregate. The source protocol's SHA-256 is part of the campaign
contract; the runner rejects source-protocol drift.

### Evidence and decision

At commit `6cd1812b66f2548ed8c8e2e3b668ff4dd4b2efff`, the merged campaign
covered 480 replication cases / 23,552 source observations. It returned
**failed**, with 477 cases passing and three retained failures, all in the
unflagged vertical-wind case:

- 30/32 ended `source_latched`;
- 1/32 ended stale without a completed required latch;
- 1/32 ended `qualified`, an unsafe false qualification.

The remaining source controls and the 32 qualified multi-heading replications
were stable: terminal wind error mean/P95/maximum
`0.13742/0.28459/0.52372 m/s`, with known-wind NIS-mean P05/P95
`0.63245/1.31173` around `0.96845`.

The result rejects the v1 proposition that residual NIS alone can guarantee
detection of a vertical-wind model violation. The data cannot honestly support
a threshold tweak: with only TAS magnitude and GNSS ground velocity, unflagged
vertical air mass is not independently observable in all draws. The 15-state
ESKF remains unchanged, and a 17-state wind branch remains blocked pending an
explicit, independently justified fixed-wing source/regime envelope and
physical evidence.

### Verification

The compact failed evidence is retained as
[`airspeed_wind_observability_campaign_v1.json`](../validation/public/airspeed_wind_observability_campaign_v1.json),
including all failed-case records, canonical digest, clean Git provenance, and
shard completeness checks. The new shard/merge unit tests pass 5/5. A fresh
post-review host regression passed 17/17 CTests, 211/211 Python tests, the
1,020,000-attempt input-integrity campaign, deterministic-suite generation,
and reviewed threshold checks. Its disposable manifest records the pending
evidence/documentation commit; no executable source changed after the
clean-provenance campaign artifact was generated.

## 2026-08-11 — TAS residual-pattern monitor review and sealed holdout design

### Reason

The 480-case TAS/wind confirmation retained one unflagged vertical-wind false
qualification. A proposed residual-history monitor was reviewed before being
committed as evidence. The review found that its nominal control never formed
a full four-observation window, while its first mismatch could latch from one
large residual plus three nominal samples. Its reported `128/128` result was
therefore development-only and invalid as a persistent-mismatch claim.

### Changes

- Retained the v1 review failure in `airspeed-wind-mismatch-monitor.md`; it
  never enters `validation/public/` and changes no production ESKF code/API.
- Added a v2 opened-development protocol that begins with the frozen
  multi-heading source stream and requires at least 40 post-qualification
  clean observations / 37 complete windows before any injection.
- Added long nominal, high-noise nominal, single impulse, gap-plus-impulse,
  persistent TAS-scale, horizontal-wind-step, and vertical-wind controls.
- Added source/arrival timing to the host monitor, a `1.0 s` inter-sample gap
  reset, `1.75 s` maximum four-sample span, clipped individual NIS, and a
  multiple-contribution latch condition. This is a validation instrument, not
  a private FCOne supervisor.
- Added a development-only v2 protocol plus a separately fingerprinted sealed
  protocol. The sealed protocol locks the base source protocol, base
  oracle/generator, candidate runner, cases, parameters, and holdout seeds; it
  refuses a dirty worktree.

### Opened-development evidence and decision

The first v2 parameter setting was correctly rejected in its opened 32-seed
matrix: `170/224` cases passed, with nominal/impulse false latches and a
high-noise bootstrap-boundary defect. Those failures selected the *development*
revision only; no sealed seed was read.

The second v2 setting passes `224/224` replications, covering 30,528 source
observations and 17,723 complete monitor windows. All four non-latch controls
remain unlatched across 32 seeds. Each of the three persistent mismatches
latches across 32 seeds within its declared deadline; every trigger contains
at least two injection-phase samples. The compact v2 development record
[`airspeed_wind_mismatch_monitor_v2_development.json`](../validation/public/airspeed_wind_mismatch_monitor_v2_development.json)
was generated from clean commit `e672e5d` with the then-frozen monitor runner;
its `phase: development` remains part of the artifact, so it is selection
evidence rather than a holdout result. Individual streams remain disposable
under `build/`.

This result is intentionally narrow. It demonstrates that the reviewed host
screen no longer obtains a green nominal result from zero executable windows
and does not latch on the declared one-off impulse controls. It does **not**
make vertical wind observable, qualify physical air data, enable TAS in VTOL
regimes, or unblock a 17-state wind branch. The source-policy blocker from the
480-case wind confirmation remains in force.

### Protocol-integrity incident and corrected next verification

Before this source was committed, an independent implementation smoke ran all
seven cases for intended-v3 seed `42001` in a temporary clean checkout. It
reported `7/7`, but that number is discarded: opening even one originally
sealed seed means v3 cannot honestly be called an unopened holdout. No parameter
was adjusted from that result, the original worktree did not run v3, and no
public artifact was written. The event is retained here rather than hidden.

The v2 protocol is now explicitly development-only. The contaminated v3
manifest is discarded, and a fresh v4 manifest uses only `43001--43128`, none
of which were run during development or review. Commit the runner, tests,
v1-review record, v2 development record, and v4 protocol. Then execute only:

```bash
python3 validation/run_airspeed_wind_mismatch_monitor.py \
  --protocol validation/airspeed_wind_mismatch_monitor_protocol_v4.json \
  --phase sealed_holdout --jobs 8
```

The compact clean result may be committed only after review. Any holdout
failure remains evidence and may not be corrected by changing this v4 protocol.

### Reproduction environment note

The managed desktop sandbox terminates a forked `ProcessPoolExecutor` without
diagnostic output after the campaign begins. The runner therefore has an
explicit in-process `--jobs 1` path; it evaluates the same isolated case
function in canonical task order and does not change any random seed, monitor
parameter, result schema, or ESKF code. Normal workstations may still use the
existing process pool. This is execution portability only, not additional
algorithm evidence.

The same runner also supports explicit contiguous raw-record shards followed
by one strict merge. This is only a transport fallback for a session-duration
limit: the merger requires every frozen seed exactly once, every case record,
one clean source commit, and matching runner/protocol fingerprints. It emits a
compact final summary and does not permit partial results to replace the sealed
campaign.

A direct long sequential rerun completed after later source edits and recorded
a dirty worktree in its own provenance. That output was discarded and did not
replace the clean `e672e5d` v2 development record. This demonstrates why the
shard merger requires clean, matching source provenance rather than trusting a
late file write.

## 2026-08-11 — v4 sealed holdout executed and failed

The fresh v4 seed set `43001--43128` was executed once after the runner,
protocol, tests, and v2 development artifact were frozen. Four contiguous raw
shards were merged with the strict provenance checker from clean commit
`f085488`. The compact result is
[`airspeed_wind_mismatch_monitor_v4.json`](../validation/public/airspeed_wind_mismatch_monitor_v4.json).

The campaign processed `896` case/seed replications, `122,112` input
observations, and `70,917` complete windows. It passed `889` and failed `7`.
The failed records are retained verbatim in the JSON artifact:

| Case | Seed(s) | Failure pattern |
| --- | --- | --- |
| `single_tas_impulse` | `43033`, `43108` | false latch; one seed latched before injection |
| `persistent_tas_scale_bias` | `43061` | early latch before injection / insufficient injected evidence |
| `persistent_horizontal_wind_step` | `43067` | latch window contained only one injected sample |
| `persistent_vertical_wind` | `43065`, `43102` | detection exceeded the 8 s deadline |
| `persistent_vertical_wind` | `43074` | early latch before injection / insufficient injected evidence |

This is a valid failed holdout, not a partial run and not an algorithm
promotion. The v4 parameters and seeds are frozen permanently. The result
blocks promotion of this residual-pattern monitor to a runtime source
supervisor; further work must begin with a new development protocol that
explicitly addresses single-point dominance, minimum post-injection coverage,
and the detection deadline. The production ESKF and public API were unchanged.

## 2026-08-11 — full-state fixed-lag replay candidate rejected

### Reason

The earlier 5D no-injection MAP proposal was rejected because it fit a closed
window without mutating or replaying the ESKF. A second experiment was needed
to determine whether the missing result came from the lack of a complete state
transaction or from the correction hypothesis itself.

### Changes

- Added a private host-only helper that reuses the core nominal-state injection
  and attitude covariance-reset Jacobian.
- Moved validation injection to an explicit pre-IMU/pre-aiding boundary.
- Recorded raw pre-update GNSS position/velocity innovations and diagonal
  innovation variances in the native replay output.
- Added symmetric finite-difference replays, a prior-regularized five-state
  correction, full replay, and an innovation-only second-half holdout.
- Added exact zero-injection, transaction symmetry, input validation, schedule,
  and candidate-campaign tests.

### Evidence

The frozen development protocol ran 216 paired train/tune trials over three
causal v2 motions, nine signed accelerometer-bias vectors, and eight seeds per
split. The compact report is
[`fixed_lag_replay_candidate_v1.json`](../validation/public/fixed_lag_replay_candidate_v1.json);
the focused interpretation is in
[`fixed-lag-replay-candidate.md`](fixed-lag-replay-candidate.md).

- 91 candidates passed the innovation-only holdout and were externally scored;
  125 were rejected by the holdout, schedule, or health checks.
- Accepted train/tune terminal horizontal-bias P95 deltas were
  `-0.00571/-0.00360 m/s2`.
- Four zero-bias material regressions and twelve attitude-RMSE material
  regressions remained; signed nonzero group improvement was `17/24`.
- Every accepted replay remained numerically healthy with zero navigation
  recoveries and an unchanged accepted P/V schedule.

### Decision

The candidate is rejected for promotion. The production ESKF, public API, and
FCOne path remain unchanged. The experiment establishes an executable,
auditable correction transaction and shows that the current local innovation
fit still overfits residual/model effects. A future candidate must add full
15x15 lag covariance, process/preintegration covariance, physical source/arrival
timing, and a new sealed protocol before another correction is considered.

## 2026-08-11 — full-state nuisance screen rejected

### Reason

The 5D replay candidate could not distinguish a desired tilt/bias correction
from trajectory residual fitting. The full-state screen tested whether carrying
all 15 error states and a complete boundary covariance would remove that
failure mode.

### Evidence and decision

The frozen 60-trial screen recorded 24 trust-region accepted candidates, 34
truth-free validation rejections, and two perturbation-schedule rejections.
All accepted cases improved position RMSE, but the intended horizontal-bias
metric improved in only 12 of 19 nonzero cases. One zero-bias and one
attitude-RMSE material regression remained. The candidate is rejected; do not
expand its matrix or promote it to FCOne. Full detail and the compact report are
in [full-state sensitivity screen](full-state-sensitivity-screen.md).

### Verification

After recording the result, the release CTest suite passed `14/14`, the Python
suite passed `197/197`, and the ASan/UBSan CTest suite passed `14/14`. The
sandbox runs processes under ptrace, so LeakSanitizer cannot execute here; the
sanitizer run used `ASAN_OPTIONS=detect_leaks=0` and therefore does not make a
leak-detection claim. It still exercises AddressSanitizer and UndefinedBehavior
Sanitizer for the covered tests.

## 2026-08-11 — lag covariance composition prerequisite closed

### Reason

The rejected replay candidates used only a boundary covariance. A real
fixed-lag formulation must carry the complete state transition and accumulated
process covariance through every retained IMU interval before its measurement
correlations or a backward pass can be trusted.

### Evidence and decision

A new validation-only oracle accumulated the production pre-integration
transition and process covariance over `384` deterministic variable-rate
chains (`4,592` IMU intervals) and compared the result with repeated
`eskf_predict()` covariance propagation. The maximum endpoint difference was
`7.81597009e-14` in the reviewed double build. A host float evaluation also
passed its predeclared `5e-4` bound with a maximum difference of
`6.10351562e-05`.

This closes only the no-measurement `Phi/Q` composition identity. It does not
promote a smoother or a correction candidate, and it does not relax the
requirements for measurement-update cross covariance, ESKF relinearization,
physical source/arrival timestamps, a clean protocol, or a sealed holdout.
Full scope and reproduction commands are in
[fixed-lag covariance composition](fixed-lag-covariance-composition.md).

### Verification

The release CTest suite passed `15/15`, including the new composition oracle,
and the Python suite passed `197/197`. The host-float build passed the oracle's
separate tolerance. The ASan/UBSan CTest suite also passed `15/15` with
`ASAN_OPTIONS=detect_leaks=0`. LeakSanitizer cannot run under this desktop
sandbox's ptrace model, so that sanitizer execution does not make a
leak-detection claim; AddressSanitizer and UndefinedBehaviorSanitizer remained
active for the covered tests.

## 2026-08-11 — P/V lag cross-covariance float limitation found

### Reason

The composed `Phi/Q` prerequisite still lacked cross-covariance transport
through accepted measurements and attitude-reset injection. This test used a
joint live/boundary covariance rather than treating the boundary covariance as
an isolated prior.

### Evidence and decision

The 30x30 augmented oracle completed 192 deterministic chains, 2,202 IMU
intervals, and 1,150 accepted P/V updates. In the reviewed double build, its
live 15x15 block matches production prediction/update covariance to
`1.0658141e-14` / `7.10542736e-15`, exercised a maximum attitude-reset
correction of `6.08450099e-04 rad`, and the joint covariance remained finite,
symmetric, and PSD.

The host float build is intentionally **not** promoted: its live block still
matches the float ESKF to `3.81469727e-06` / `1.90734863e-06`, but 130 joint
PSD checks failed with a worst Cholesky pivot of `-1.79939767e-05` after a
maximum reset correction of `6.08450061e-04 rad`. The current naive covariance
form is therefore disallowed for a float fixed-lag buffer.
Future target work needs a square-root/UD representation or separately
validated bounded PSD-repair policy. Full scope and commands are in
[fixed-lag measurement cross covariance](fixed-lag-measurement-cross-covariance.md).

### Verification

The reviewed default release gate passed `16/16` CTest targets, including the
new augmented-covariance oracle, and `197/197` Python tests. A fresh Debug
AddressSanitizer/UndefinedBehaviorSanitizer build also passed `16/16` CTest
targets; the exhaustive input-integrity campaign required `134.35 s` and the
complete sanitizer CTest run required `157.80 s`. The managed desktop sandbox
cannot run LeakSanitizer under its `ptrace` model, so the sanitizer command used
`ASAN_OPTIONS=detect_leaks=0`; this result covers ASan and UBSan only and makes
no leak-detection claim.

## 2026-08-11 — complete barometer shadow-lane handoff contract

### Reason

The complete barometer campaign showed a source-latch boundary but retained an
offline diagnostic mux that selected only vertical position, vertical velocity,
and health. That cannot be a flight-estimator transaction: earlier accepted
barometer data can alter bias and all correlated covariance blocks, and a
partial splice would leave the output internally inconsistent.

### Changes

- Added a host-only two-lane contract driven by the same audited IMU, position,
  velocity, and trusted-heading event stream; the active lane alone receives
  barometer observations.
- Used the existing causal barometer supervisor to reach a real repeated-value
  freeze latch, then permit one active-to-shadow image transfer.
- Required an all-or-nothing transfer of the pointer-free `AerakiaEskf` image,
  rather than a component or covariance blend, and recorded attitude, position,
  velocity, accelerometer-bias, gyro-bias, and covariance reset deltas.
- Added fail-closed cases for missing latch, automatic return, timestamp skew,
  common-input provenance mismatch, configuration mismatch, source-generation
  mismatch, incomplete alignment, finite-but-unhealthy attitude, non-finite
  covariance, and duplicate switching.

### Evidence and decision

The success matrix passed at 100, 200, and 400 Hz. Every path completed static
alignment, triggered the actual supervisor latch, transferred the complete
shadow image, and remained byte-equivalent to the shadow after another common
IMU event. The representative 400 Hz path carried 158 IMU events plus two each
of position, velocity, and heading observations. Its logged reset deltas were
`0 rad` attitude, `0.10191299 m` position norm, `0.0106834192 m/s` velocity
norm, and `0.0982111301` maximum absolute covariance entry.

The zero attitude reset is expected for the level vertical fixture. The nonzero
position, velocity, and covariance values demonstrate why the earlier vertical
output mux is not a valid handoff. The precondition mutations all blocked the
transaction and preserved the preexisting output image.

This closes a host transaction prerequisite only. It does not promote the
public barometer supervisor to FCOne control authority, implement a private
atomic/scheduler handoff, authorize recovery to a barometer lane, or replace
physical pressure, temperature, vibration, source-reset, redundant-voting, and
controller-reset testing.

### Verification

The reviewed release gate passed `17/17` CTest targets and `197/197` Python
tests. A separate host-float build passed the handoff contract at all three
rates; its representative 400 Hz position/velocity/covariance reset diagnostics
were `0.101912946 m`, `0.0106832981 m/s`, and `0.0982105508`.

The one-command `validation/run_host_regression.py` workflow was also rerun
from a clean release build directory. It configured, strictly built, passed
`17/17` CTest and `197/197` Python tests, re-executed the 1,020,000-attempt
input campaign, regenerated the deterministic suite, and passed the reviewed
threshold checker. Its disposable run manifest records all seven command lines,
durations, tool versions, and the uncommitted source state used for the review.

A fresh ASan/UBSan Debug build passed all `17/17` CTest targets in `157.27 s`;
the 1,020,000-attempt input-integrity campaign consumed `133.88 s`. The
managed desktop sandbox cannot run LeakSanitizer under its `ptrace` model, so
the command used `ASAN_OPTIONS=detect_leaks=0`. This is an ASan/UBSan result,
not a leak-detection claim.

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

After the three implementation/evidence commits, the same 12 trials were regenerated from clean
commit `dcc002c1317a9eb3fac4a1f1f60126dfc14797b0`. The report records an empty Git status and all
12 machine-derived shadow input/command audits pass. Summary SHA-256 is
`94b7f9e20a37787e074e8057f1c3470a4302a2f80df24fe7dbc1ad5c0777cbce`; report SHA-256 is
`1b141fab857dbd67e263a217c31f65c2d8216c273ee0816eaca021e8de9d3a8b`.

### Verification at this checkpoint

G0 focused tests pass `16/16`; barometer focused Python tests pass `10/10`; the new INSANE
calibration/converter/A-B tests pass `14/14`. The complete Python discovery passes `129/129` and a
fresh strict C99 CTest passes `10/10`. The complete host regression also passes all 129 Python tests,
all 10 CTest targets, the 1,020,000-attempt input-integrity campaign, every deterministic scenario,
and every frozen threshold. A fresh ASan/UBSan build passes `10/10`; its input campaign takes
`145.85 s` and the complete sanitizer suite `155.41 s`, with no sanitizer findings. Raw archives,
replays, and detailed result CSVs remain ignored build artifacts; aggregate JSON, code, tests,
methods, and negative decisions are committed together at this checkpoint.

## 2026-07-20 — physical magnetic source diagnostic and supervision pre-registration

### Correction to the prior hypothesis

An implementation audit corrected an earlier hypothesis that missing magnetic inclination or the
reference Down component caused the `INSANE indoor_1` degradation. The current ESKF magnetic model
is deliberately a horizontal NED-yaw pseudo observation: both static alignment and the online
update use only horizontal `atan2(E, N)` terms. Reference Down is therefore absent from the residual
and Jacobian. Supplying a full three-dimensional reference while keeping this model unchanged would
produce the same update. No full-3D model was enabled or implied by this investigation.

### Rebuilt split provenance

The earlier calibration replay had been produced with an outdated source-window manifest. No raw
input was changed; the ignored build manifest was repaired by adding the missing
`calibration.source_sequence`, then current converter tooling rebuilt the three disjoint replays:
calibration `[10,110) s`, development `[110,210) s`, and historical holdout `[210,310) s`.
Their replay SHA-256 values are respectively
`114cb0fe95c904fea6ed7e4e03f8835cb8e4eacc29ffcb4a0df5d5d8a5aaf2cf`,
`31e230f431a3f354843a937e89b7b3ce4f8863a709d3f0521f8f8fe12bd0a59f`, and
`35364c37cb0c92a66ec2f9eceb809ee044a4b39c9ebffd1d5ba8af6839055309`.
The rebuilt development and holdout hashes match the prior valid replays; the calibration now has a
correctly isolated 100-second window.

### Paired source result

A new offline-only diagnostic pairs the exact same replay with magnetometer-on and magnetometer-off
ESKF results. It fails closed on timestamp/row mismatch, changing declared magnetic datum, invalid
quaternion or field data, a magnetometer-on flag that differs from the replay, or any magnetic input
in the off arm. Reference attitude is used only offline to rotate raw physical magnetometer samples
to the reference NED frame and score the source; it is not an estimator input and is forbidden from
future runtime source decisions.

| Rebuilt `indoor_1` window | Physical updates | ESKF accepted | Mag on yaw / tilt RMSE | Mag off yaw / tilt RMSE | Datum residual P95 abs |
| --- | ---: | ---: | ---: | ---: | ---: |
| calibration `[10,110) s` | 8,788 | 100% | `6.248 / 12.279 deg` | `0.298 / 0.608 deg` | `16.234 deg` |
| development `[110,210) s` | 8,782 | 100% | `10.879 / 6.317 deg` | `0.250 / 0.615 deg` | `18.636 deg` |

In development, the predeclared `>=15 deg` physical-datum-residual bin contains 977 samples. It
still has 100% ESKF magnetic acceptance while the magnetometer-on yaw/tilt RMSE is
`15.532/8.645 deg`, against `0.218/0.630 deg` in the paired off arm. The absolute physical residual
to absolute on-minus-off yaw correlation is `0.569` on calibration and `0.572` on development.
This supports the narrow conclusion that accepted physical magnetic updates can be harmful and that
the existing magnitude plus pseudo-NIS protection is inadequate. It does not prove whether the
underlying cause is environment/current, calibration/install residual, time/frame error, or later
ESKF covariance coupling.

### Frozen next step

`validation/magnetic_supervision_protocol_v1.json` now reserves calibration and development for
design only. The historical `indoor_1` holdout and `transition_1` cannot select thresholds. Before
downloading `indoor_2`, a candidate must be implemented from causal physical inputs only, tested
against predeclared synthetic disturbance families, and frozen with its configuration fingerprint.
`indoor_2` then becomes first sealed validation and `indoor_3` is unchanged replication. A candidate
cannot pass by rejecting all magnetic updates. The protocol explicitly records the fundamental
limit: slow persistent magnetic heading error with nearly constant norm cannot be disambiguated from
yaw drift using IMU plus magnetometer alone; independent heading or a stronger source constraint is
required.

### Causal feature screen

The diagnostic was extended with two features that do not consume reference attitude, estimator
error, future data, or sequence identity in their calculation. The first gyro-propagates the previous
valid body-frame magnetic direction using the preceding physical gyro samples, then measures its
angle to the next valid magnetic direction. The calibration/development residual P95 is
`1.172/1.188 deg`, and its correlation with absolute magnetometer-on-minus-off yaw impact is
`-0.014/-0.106`. It therefore does not identify the slow failure, although it remains appropriate
for a future abrupt direction-step synthetic fault family.

The second feature is an inclination proxy from the angle between raw magnetic field and raw specific
force, evaluated only under the already public static-alignment contract: acceleration norm within
`0.20 g` of gravity and gyro norm at most `0.05 rad/s`. It yields 3,352 calibration and 1,653
development samples. Its offline physical-inclination correlation is `0.779/0.480`, but its
correlation with absolute yaw impact reverses from `-0.339` to `+0.332`. That cross-window
instability rejects it as a candidate source-quality policy.

No runtime C path, magnetic tuning, or magnetic fusion default changed after either negative result.
The public default remains `fuse_magnetometer = false`. The retained conclusion is that these
causal features can detect only subsets of physical faults; they cannot resolve a slow persistent
magnetic datum error without independent heading, calibration, or environmental evidence.

## 2026-07-21 — Full process-noise discretization audit

The former Q evidence proved the declared reduced continuous model exactly, but did not quantify
the real higher-order terms omitted when new gyro noise, bias random walk, specific force, and
angular rate interact during a single IMU interval. The production Q was therefore correctly
described as a high-rate approximation, but the remaining `full Qd` item had not been measured.

The C model campaign now constructs the independent frozen-coefficient continuous first-order
matrix `A` and white-noise spectral-density matrix `W` for the complete 15-error-state model, then
integrates `dQ/dt = A Q + Q A^T + W` with RK4. It verifies the independently constructed `A`
against the production transition derivative, requires the omitted coupling to be nonzero, checks
the oracle for finiteness/symmetry/PSD, and checks 20 versus 40 RK4 steps. The production runtime
path is unchanged.

In a fixed-seed 1,000-case 100--1000 Hz synthetic stress campaign (1--10 ms IMU periods, each
specific-force component bounded by 25 m/s^2, each angular-rate component bounded by 6 rad/s, and
the documented high-noise profile `0.37/0.018/0.004/0.0007` in SI density units), the maximum
relative Frobenius defect was `5.01673511e-04` (`0.0502%`). RK4 step refinement differed by at
most `1.35308431e-16`; the transition-derivative comparison differed by at most
`3.51718654e-06`. The predeclared one-percent high-rate bound passes.

This closes the bounded host-model question: a production full coupled Qd is not mathematically
required for the declared high-rate stress envelope. It does **not** authorize an unrestricted Qd
claim. Scheduler gaps over 10 ms, motion outside the envelope, colored/correlated sensor noise,
STM32H7 precision behavior, or target NIS/NEES inconsistency reopen the work. The exact model,
omitted terms, executable limits, and re-open conditions are in
[process-noise-discretization.md](process-noise-discretization.md).

### Verification at this checkpoint

The magnetic-source diagnostic unit suite passes `5/5`, including paired-flag and off-arm acceptance,
gyro-propagation sign, and gravity-proxy controls. The complete Python discovery passes `134/134`, strict C99 CTest passes
`10/10`, and `validation/run_host_regression.py` passes. The Python public-dataset suite reports its
expected `blocked` status for the optional PX4 comparison because `--px4-source` was not supplied;
it is not a test failure and no network download was attempted. `git diff --check` and strict JSON
validation of the protocol pass. No estimator parameter, runtime C path, or threshold was changed.

## 2026-07-21 — official PX4 same-input M0 transport baseline

The first executable same-input comparison now builds a read-only out-of-tree host library from
official PX4 commit `de8158101c96ad6b04170dc91f087148104c58eb`. The standalone project compiles
only the required official `ecl_EKF` and support translation units, uses PX4's unmodified uORB
header generator (`empy 3.3.4`), and records the pinned source hashes. It neither modifies the PX4
checkout nor uses the FCOne v1 snapshot.

The deterministic no-delay, 65 s / 100 Hz hover-first input contains 6,501 IMU events. PX4 exports
6,489 strictly advancing delayed-fusion horizons; Aerakia emits only at those exact horizons. Both
canonical sidecars, input/output SHA-256 values, source commits, profiles, and protocol fingerprints
are validated by the fail-closed scorer.

| Metric | Aerakia | PX4 |
| --- | ---: | ---: |
| Bias RMSE | `0.113687 m/s²` | `0.122453 m/s²` |
| Terminal bias error | `0.092103 m/s²` | `0.122432 m/s²` |
| Numerical-health ratio | `100%` | `100%` |
| 35 s absolute convergence gate | Fail | Fail |

The result is intentionally not interpreted as a performance win, parity result, or flight-readiness
claim: this is one deterministic fixed-bias case with no measurement delay. Its value is that it
closes the event/frame/time/provenance path needed for future fair tests. The broader matched-Q,
multi-bias, delay, fault, independent-truth, CPU/memory, and blind G2 tracks remain open.

## 2026-07-21 — G0 correlated static-prior rejection and host-only closure

The last remaining generic host-only G0 tuning hypothesis was whether a one-pose stationary
alignment should retain the physical local correlation between right tilt error and accelerometer
bias, rather than initialize them as independent. The candidate changed only startup covariance; it
did not add state, consume truth/commands/future samples in the filter, or alter the public default
path.

A frozen paired campaign ran the same 576 train/tune trial keys for baseline and candidate, for
1,152 completed executions with zero execution failures. The candidate improved the aggregate pass
count from `175/576` to `231/576` and converted 49 right-censored trials to settled trials. That is
not sufficient to promote it: it produced 22 material zero-bias regressions and five
mirror-symmetry regressions. The candidate was therefore rejected and its runtime initializer
removed.

The study is diagnostic rather than release evidence: the v1 generator still uses a trajectory-truth
static hint, both arms were executed from a dirty source tree, and the historical v1 holdout was
already exposed and was not reopened. The durable public result is
[`g0_correlated_static_prior_rejection.json`](../validation/public/g0_correlated_static_prior_rejection.json);
the method and decision are in
[G0 correlated static-prior rejection](g0-correlated-static-prior.md).

Because the full campaign can exceed short execution-session limits, the cross-validation runner now
has an identity-checked `--resume` mode. It reuses only records whose trajectory, motion, split,
bias vector, seed, input SHA-256, result shape, and normal runner mode all match; stale records or
records containing the removed experiment switch are rerun. The resume tests exercise exact reuse
and every rejection path.

The complete current host regression passed after candidate removal: strict C99 CTest `10/10`,
Python discovery `145/145`, the input-integrity campaign, the deterministic synthetic suite, and all
frozen thresholds. This confirms no default regression; it is not a flight-readiness or physical
sensor validation claim.

The correct next estimator hypothesis is not another static prior or scalar P/Q sweep. It is a
causal fixed-lag, excitation-aware joint tilt/accelerometer-bias correction using actual accepted
GNSS position/velocity observations, correct covariance/repropagation, and a physical IMU interval,
timestamp, stationarity, and source-quality contract. That contract belongs to the private FCOne v2
adapter and is deferred until the hardware integration inputs exist.

## 2026-08-07 — G0 v2 input-contract smoke

The first pre-candidate G0 step replaced the ambiguous v1 synthetic input assumptions with a
versioned validation contract. The generator now integrates body specific force over each physical
IMU interval, emits auditable delta-angle/delta-velocity fields alongside the existing public rate
fields, defines white noise by density rather than per-sample standard deviation, and computes the
static indication from a one-second past-only window over the quantized IMU stream. The causal
detector is explicitly not a complete stationarity policy because constant-velocity translation is
inertially indistinguishable from rest.

The initial four-rate smoke is implemented by
`validation/run_bias_observability_input_contract_v2.py` for 50/100/200/400 Hz. It checks timestamp
and delta-field identity, rejects truth-derived stationarity metadata, and runs the unchanged native
ESKF host runner at every rate. This smoke is an input/integrity gate only; it does not promote an
estimator correction or close the 35-second horizontal-bias convergence defect. Timestamp delay,
jitter, quantization/saturation stress, random walk, thermal drift, lever arm, and the protected
fixed-lag holdout remain deferred.

The same run now binds the analyzer-only causal provenance manifest. Structural readiness was first
declared at `5.0 s` at 100 Hz and `7.5 s` at 400 Hz, but not within 32 s at 50 or 200 Hz. This
sampling-rate dependence is retained as a new diagnostic finding. The readiness was transient: the
final 20-second window had effective rank 3 and failed the condition gate at all four rates. This
blocks using the analyzer as a fixed-lag trigger until its transition, aiding covariance,
resampling, and persistence semantics are made rate-consistent.

The repository was then reorganized without changing source paths: directory-level READMEs now identify
the test, simulation, validation, and workstation entry points. The two G0 build trees contained only
compiler products, replay CSVs, analyzer reports, and command logs; the compact hashed result and this
execution record are retained, while those build trees and Python bytecode caches are disposable and
scheduled for removal after this commit.

## 2026-08-07 — G0 analyzer rate sensitivity

A 12-case study compared 50/100/200/400 Hz IMU input under continuous-density,
zero-noise, and fixed-per-sample-noise profiles while holding the aiding stream
at 10 Hz. Every case had `100%` ESKF health and zero navigation recoveries.
Effective full-rank analyzer windows appeared only during the early excited
portion of the trajectory; the final 20-second windows were rank `3` in all
cases. Structural-ready counts varied from `0` to `35` by rate/profile.

The zero-noise control preserves the qualitative result, so lowering sensor
noise or changing the density conversion is not a sufficient fix. The finding
is attributed to finite excitation leaving a trailing window, plus analyzer
limitations around process/preintegration covariance, aiding correlation, and
error-reset Jacobians. The compact result is
[`g0_rate_sensitivity.json`](../validation/public/g0_rate_sensitivity.json).
No estimator code or thresholds were changed; a rate-invariant information
model and explicit history/persistence semantics are required before a causal
fixed-lag correction is implemented.

## 2026-08-07 — G0 rate-study null-control correction

The first rate-study implementation labeled a case `zero_sensor_noise` while
only setting IMU noise to zero; magnetometer and GNSS noise were still enabled.
That was not a valid all-measurement-zero control. The profile was renamed to
`zero_all_measurement_noise`, regenerated, and its evidence re-read before any
conclusion was retained.

In the corrected control, all 50/100/200/400 Hz cases stayed at effective rank
3 with zero full-rank and zero structural-ready windows. The noisy profiles can
temporarily reach rank 5, including where the ideal control does not. This
supports the narrower conclusion that noisy filtered-state linearization can
manufacture apparent rank; it does not identify a safe estimator correction.

The follow-up stationary-null/structural-control campaign is implemented by
`validation/run_bias_observability_gate_characterization.py`. Its result must be
interpreted as analyzer error-rate evidence only. No ESKF source, threshold, or
flight policy was changed.

The completed 16-seed, four-rate run produced 26/64 stationary
structural-ready false positives and 60/64 stationary full-rank detections.
Per-rate ready counts were 6/16 at 50 Hz, 5/16 at 100 Hz, 5/16 at 200 Hz, and
10/16 at 400 Hz. All 64 ESKF executions remained healthy with zero navigation
recoveries. Both zero-noise positive-control trajectories were structural-ready
at all four rates. The current analyzer gate is therefore rejected; only the
ideal trajectory candidates are retained for a future corrected model.

## 2026-08-10 — G0 score calibration split

The binary analyzer gate was retained as a rejected diagnostic after its
stationary false-positive result. A continuous score, the maximum minimum
eigenvalue over causal windows, was calibrated only on stationary seeds 0--7.
The candidate threshold was frozen as three times the calibration P99:
`0.004936251852866821`.

On disjoint stationary seeds 100--115 at 50/100/200/400 Hz, the candidate score
passed `0/64`, while the old structural-ready flag passed `31/64`. On the same
new seeds and rates, noisy `takeoff_box_land` and `yaw_quadrant_hover` each
passed `64/64`; their minimum scores were `0.09024` and `139.1852`. These are
analyzer screening results only. The score remains disconnected from ESKF
correction, supervisor qualification, and flight authority.

## 2026-08-10 — no-injection fixed-lag bias proposal rejected

### Reason

The corrected continuous score removed the binary analyzer's known stationary
false-positive path, but it did not establish that a score-qualified
tilt/accelerometer-bias correction would improve the filter. Before modifying
the ESKF, a host-only solver was built to make that distinction explicit.

### Method

Each trial runs the unmodified native ESKF first. A separate Python solver then
uses only the closed 20 s window of IMU, accepted GNSS position/velocity, and
baseline filter outputs. It nuisance-projects the whitened design and forms a
prior-regularized MAP proposal for right tilt x/y and three accelerometer-bias
components. Truth is deliberately unavailable to the solver and is read only
afterward by the evaluator.

The frozen development matrix covered two causal v2 maneuvers, nine signed
residual-bias groups, and eight seeds: `144` baseline replays. Four closed
windows were evaluated per replay where score-qualified. Three prior-information
scales (`0.1`, `0.3`, `1.0`) were compared. No proposal was injected, no state
or covariance was replayed, and no flight/control authority was exercised.

### Result and decision

All `144` baseline executions had `100%` numerical health and zero navigation
recoveries. None of the three scales met a non-regression criterion. At the
least harmful scale (`1.0`), all `475` score-qualified windows changed mean/P95
bias error from `0.09508/0.22055` to `0.09729/0.22663 m/s2`; only `163/475`
windows improved. At the terminal window, the result was `0.10189/0.22257` to
`0.10291/0.23550 m/s2`, with `61/144` improved.

The candidate is rejected. No public estimator core, default parameter,
threshold, or private FCOne product setting was changed. The durable compact
artifact records the protocol, code identities, per-trial input/result manifest
hashes, aggregate statistics, and motion stratification:
[`fixed_lag_bias_proposal_campaign.json`](../validation/public/fixed_lag_bias_proposal_campaign.json).
The campaign runner now omits large per-window details by default; they are
available only with `--include-trials` for local debugging and stay outside Git.
It also defaults to a single worker so the official evidence path does not rely
on uncontrolled generator/native-runner process fanout; `--jobs` remains an
explicit workstation-specific opt-in.

### Next bounded experiment

Do not turn this rejected 5x5 marginal proposal into an injected correction.
First construct a host-only delayed-GNSS rewind/replay oracle. It must restore a
full pre-aiding ESKF snapshot at an exact IMU boundary, apply the existing GPS
update, and replay immutable later IMU/aiding events in canonical order. Its
isolated delayed-epoch output must match the zero-delay reference in state and
covariance before any new full-lag correction candidate can be specified. This
is prerequisite infrastructure, not a claim of product delayed fusion or flight
readiness.

## 2026-08-10 — delayed-GNSS rewind/replay prerequisite

### Reason

The rejected MAP proposal exposed a concrete missing prerequisite: it had no
full lag covariance or state/covariance repropagation path. The existing public
adapter has a timestamp/freshness contract but no historical state buffer.
Before designing another estimator correction, we needed to test the existing
ESKF update/reset mathematics in a complete replay transaction rather than
implement a partial correction shortcut.

### Method

The new host-only native oracle retains complete `AerakiaEskf` snapshots at
IMU boundaries and immutable IMU/GNSS P/V records in a bounded ring. It runs a
zero-delay reference and a lane with exactly one withheld GNSS P/V epoch. At
delivery, the delayed lane restores the source-time pre-aiding snapshot,
performs the ordinary timestamped GPS update, and replays later IMU plus
already-committed GPS events in canonical order.

The executed matrix has twelve exact-boundary cases over one deterministic,
exact-timestamp synthetic P/V stream: 100/200/400 Hz and
20/50/100/150 ms delivery delay. The Python wrapper validates status/scope,
pre-delivery sensitivity, post-delivery state/covariance/metadata equivalence,
finite state, PSD covariance, and the native pass result. It does not use truth
inside the replay path.

### Result

All `12/12` cases passed. Each delayed lane visibly diverged before delivery
(maximum state-component difference approximately `8.5e-4`--`9.0e-4`), then
matched the zero-delay reference after replay with zero recorded state and
covariance difference and matching metadata under a `1e-12` double-precision
tolerance. Every case remained finite and PSD.

This proves only deterministic replay equivalence for an isolated synthetic
GNSS P/V event. It does not add a production rewind API, validate multi-event
delays, delayed heading/barometer, source-arrival timing, multi-IMU switching,
target CPU/RAM, or safety behavior. The compact evidence and full boundary are
in [`delayed_gnss_repropagation_oracle_v1.json`](../validation/public/delayed_gnss_repropagation_oracle_v1.json)
and [Delayed GNSS rewind/replay oracle](delayed-gnss-repropagation-oracle.md).

## 2026-08-10 — final post-change host verification

After the compact evidence and delayed-oracle boundary changes were committed,
the tree was rebuilt from scratch for the final check. Release C99 CTest passed
`11/11`; the Python discovery suite passed `174/174`. The float evaluation
profile built with the same warnings-as-errors policy and passed its `10/10`
CTest targets, including the 400 Hz / 150 ms replay boundary.

The Debug sanitizer profile passed the ten non-long-running CTest targets under
`ASAN_OPTIONS=detect_leaks=0` and `UBSAN_OPTIONS=halt_on_error=1`. The excluded
input-integrity executable was then run separately under the same ASan/UBSan
settings: `1,020,000` attempts, `0` invariant failures, and `0` unhealthy
outputs. Leak detection remains unverified because LeakSanitizer cannot start in
the managed traced desktop environment; no allocation-leak claim is made.

These checks validate the current source and contracts only. They do not add
physical sensor, thermal, target-MCU timing, generic overlapping multi-event
transport, or flight evidence.

## 2026-08-10 — sequential delayed-GNSS replay boundary

### Reason

The first rewind/replay oracle proved one isolated late P/V update, but a
single success does not show that a later historical snapshot remains coherent
after a prior replay transaction. The smallest useful extension is two
non-overlapping delayed sources; an overlapping schedule needs a different
ordered-event-buffer design and must not be implied by this test.

### Method

The native oracle adds a separately named `sequential` mode with source epochs
at 3.0 s and 4.0 s. The second source is after the first delivery for every
rate/delay cell. At each delivery, it restores the complete pre-aiding source
snapshot, applies the normal GPS update, replays later IMU and already
committed P/V events, and compares the result to the zero-delay lane. The
wrapper rejects a missing event, a source that is not strictly after the prior
delivery, a missing replay, a nonzero post-replay state/covariance difference,
or metadata mismatch.

### Result

The 100/200/400 Hz by 20/50/100/150 ms matrix completed `12/12` sequential
cases. Both source updates were accepted and replayed in every case. First
event pre-delivery state differences ranged from `8.53e-4` to `8.96e-4`; the
second ranged from `9.95e-4` to `1.06e-3`, confirming that the test would
detect a missing late update. After each event and at final time, every state
and full covariance comparison was exactly zero, metadata matched, and all
filters remained finite/PSD.

The new CTest target, Python negative schedule test, and compact evidence are
retained under
[`delayed_gnss_repropagation_sequential_v1.json`](../validation/public/delayed_gnss_repropagation_sequential_v1.json).
This remains host-only replay correctness evidence. Delayed heading/barometer,
mixed sensor types, interpolation, physical source-arrival timestamps, target
resource limits, and flight policy are still open.

## 2026-08-10 — overlapping/reordered delayed-GNSS replay boundary

### Reason

The sequential oracle deliberately avoided a harder condition: a newer source
can arrive while an earlier source is still pending. Reusing the isolated
replay helper in that case would be wrong because it replays every historical
P/V update and would silently fuse the older source before its declared
delivery. The next bounded evidence step is therefore not a product OOSM
implementation; it is a test that detects exactly that premature fusion error.

### Method

The new `overlap` scenario retains two exact 20 ms-cadence P/V source epochs at
`3.000 s` and `3.020 s`. The newer epoch is delivered one IMU interval after
its source. The older one arrives after `50`, `100`, or `150 ms`, giving a
strictly overlapping source/delivery window and reverse delivery order.

At every delivery, the delayed lane rebuilds from the earliest affected full
pre-aiding `AerakiaEskf` snapshot. It processes immutable IMU records and
applies an aiding record only if that source has already been delivered. The
new wrapper and negative tests require exactly two chronological sources,
overlap, reverse delivery, one pending earlier source at newer delivery, a
nonzero newer-delivery difference while that source is pending, and final
state/covariance/metadata equivalence only after the older delivery. A delay
below `50 ms` is rejected because it cannot express that pending condition.

### Result

The `100/200/400 Hz x 50/100/150 ms` matrix passed `9/9` cases. The older and
newer pre-delivery state differences range from `2.47945e-4` to `2.48558e-4`.
After the newer first delivery, the delayed lane correctly remains different
from the zero-delay baseline: state difference is `1.18867e-4`--`1.19160e-4`
and covariance difference is `1.24941e-3`--`1.26297e-3`. After the older
delivery, every final state and covariance difference is exactly zero and
metadata matches under the `1e-12` double-precision tolerance. Both lanes stay
finite and PSD. The 400 Hz / 150 ms CTest exercises a 60-sample delay inside
the 64-sample host ring.

The evidence is retained under
[`delayed_gnss_repropagation_overlap_v1.json`](../validation/public/delayed_gnss_repropagation_overlap_v1.json).
This closes a small replay-correctness uncertainty only. It does not establish
arbitrary event-count OOSM support, loss behavior for pending observations,
interpolation, delayed heading/barometer, multi-IMU replay, physical
source/arrival timing, target resource cost, controller policy, or flight
readiness.

## 2026-08-10 — GCC static-analysis hardening

An additional GCC `-fanalyzer -Werror` build reported a potential read of an
uninitialized entry in the local 15x3 Kalman-gain array inside the three-axis
measurement update. The existing nested loops assign all entries before use,
and ordinary CTest already exercised the path, but the warning is a useful
maintenance hazard: a future loop-bound change could make the warning real.

The local gain array is now explicitly zero-initialized at declaration. This
does not change the normal computed gain or ESKF mathematics; it makes the
array's fallback state defined and allows the analyzer to verify the function
without a suppression. The analyzer build completed with warnings-as-errors,
then its complete `12/12` CTest suite passed. This is static host evidence only
and does not replace target compiler or target-MCU verification.

## 2026-08-10 — Cortex-M7 preflight and genuine float candidate

### Reason

FCOne v2 hardware is not present, but the project can still reject a false
embedded-readiness assumption before integration. The old `float` candidate
reduced covariance storage while the core still used double transcendental
functions and double-promoting literals. On an STM32H7's single-precision FPU,
that makes host results an unreliable proxy for its target timing advantage.

### Method

`validation/run_cortex_m7_cross_compile.py` now compiles all seven portable C
units using `arm-none-eabi-gcc 14.2.1` for Cortex-M7 Thumb hard-float
`fpv5-d16`, archives them, performs a relocatable internal link, probes the
three public struct layouts, and classifies only final external dependencies.
The float core is separately compiled with
`-Wdouble-promotion -Werror=double-promotion`.

The type-specific core math change was checked through fresh Release CTests in
both profiles (`12/12` double, `11/11` float) and the byte-identical 20-second,
400 Hz host differential pair. The public script helper has four toolchain-free
unit tests, including path-safety checks that prohibit recursive cleanup outside
its dedicated `build/` directory.

A fresh whole-tree GCC analyzer run also found that the neutral FCOne navigation
mock omitted `source_id`, `source_generation`, and `quality_sequence` when it
constructed `AerakiaGpsObservation`. The mock now initializes and propagates
those recovery-contract fields explicitly, and asserts their preservation.
After that correction, the strict analyzer build and its `12/12` CTests passed.

### Result and decision

The strict float-core promotion check passes. The double/float portable text
section sums are `24,954/23,580 B`; `AerakiaEskf` is `2,912/1,824 B` and
`ESKF_Handle` is `2,136/1,068 B`. The clean-motion float-minus-double maxima
are `0.0011745 deg`, `0.0039432 m`, and `0.0006704 m/s`, with both evaluated
tracks 100% healthy.

Float is now a valid *measurement candidate*, not a promoted FCOne profile.
The default remains double until a complete target shadow image measures WCET,
stack, Flash/RAM, numerical health, scheduling, and logging load. The compact
artifact is [`cortex_m7_cross_compile_v1.json`](../validation/public/cortex_m7_cross_compile_v1.json);
the focused method and limitations are in
[Cortex-M7 cross-compile preflight](cortex-m7-cross-compile.md).

## 2026-08-10 — final reordered-replay verification

The reordered-replay source, fail-closed wrapper, negative payload tests, and
documentation were committed before evidence generation. The retained artifact
therefore records source commit `c0e941f`, uses only repository-relative command
paths, and passes all `9/9` `100/200/400 Hz x 50/100/150 ms` overlap cells.

A fresh Release host regression then passed `13/13` CTests and `189/189`
Python tests. It includes the unchanged full input-integrity campaign:
`1,020,000` attempted IMU samples with zero invariant failures and zero
unhealthy outputs. A fresh float evaluation profile also passed `12/12` CTests.

Fresh ASan/UBSan Debug CTests passed `12/12` when the unchanged long
input-integrity campaign was excluded. The managed desktop runner terminates
that otherwise silent million-sample sanitizer executable before it can report
completion, so a full sanitizer result for that one unchanged campaign is not
claimed in this checkpoint. The new reordered oracle is included in the
passing sanitizer set. Leak detection remains disabled because the managed
traced environment cannot start LeakSanitizer.

## 2026-08-11 — TAS residual monitor v5 opened diagnostic

### Reason

The frozen v4 holdout failed `7/896` cases because a rolling window could mix
old residuals with a new impulse, latch before injection, or meet a deadline
that had no physical justification. V4 remains frozen and was not rerun. V5
was opened only to test a structural alternative: evidence must be consecutive
after a low-residual, timing-gap, or maximum-span reset.

### Method

The v5 protocol fixes 64 development seeds (`52001--52064`), ten cases, and
common-random-number prefixes for paired streams. It requires 40 eligible
warmup observations, four consecutive NIS contributions `>=4`, per-sample cap
`6`, cumulative score `>=16`, and `1.5--2.0 s` episode duration. Long nominal,
high-noise nominal, one/two/three/four TAS impulses, gap plus impulse, and
persistent TAS bias are required checks; horizontal and vertical wind are
descriptive stresses only.

The first serial execution was stopped before producing output after a measured
single-case runtime of `3.37 s` implied roughly 40 minutes for 640 cases. No
result was discarded. A deterministic process-pool path was added; a small
fixed matrix produced identical sequential/parallel canonical hashes. The full
matrix then ran once from clean commit `06168b8` with eight workers.

### Result and cause

The matrix processed 640 replications and 87,744 observations. Common-prefix
pairing passed. Raw status is failed with 71 required failures:

- 62 are a scorer/instrumentation contract error in the gap control. Every
  stream had the declared gap and remained unlatched, but the counter increments
  only when a gap clears a non-empty episode. Only seeds `52038` and `52061`
  had evidence to clear; the other 62 were wrongly reported as missing a gap.
- nine are real candidate failures. Seed `52038` latched on two adjacent
  impulses; seeds `52028`, `52038`, `52050`, and `52061` latched on three.
  Four-impulse and persistent-bias cases at `52038`/`52061` latched with only
  two or three injected observations. A nominal high run immediately before
  injection was still able to extend into the new event.

Long nominal, high-noise nominal, and one-impulse controls stayed unlatched in
all 64 seeds. Persistent TAS bias latched in all 64, with P50/P95 delay
`1.5/3.0 s`, but its two contaminated onset windows prevent promotion.
Horizontal-wind stress latched `64/64`; vertical-wind stress latched `54/64`
with P50/P95 delay `3.0/8.0 s`. Those cases remain descriptive because the
residual cannot distinguish wind from source, timing, sideslip, or model error.

### Decision

V5 is rejected as a runtime monitor. Consecutive evidence fixes the arbitrary
v4 rolling window but not causal onset. The next opened diagnostic must record
gap observations separately from episode clears, preserve the exact latch
episode after terminal latch, and evaluate an explicit onset-boundary rule.
No new sealed holdout will be created until that structure is stable. The
production 16-nominal/15-error-state ESKF, Mahony backup, public API, and TAS/
wind state remain unchanged. The compact reviewed evidence is
[`airspeed_wind_mismatch_monitor_v5_development.json`](../validation/public/airspeed_wind_mismatch_monitor_v5_development.json).

## 2026-08-11 — TAS residual persistence v6 train

### Reason and freeze boundary

V6 was opened as a separately named source-time persistence
characterization. It was not allowed to modify the production estimator,
public sensor contract, Mahony backup, or wind-state API. The protocol fixed
train seeds `62001--62128`, 14 cases, common-random paired streams, a 97.5%
one-sided Clopper–Pearson score, a nuisance false-latch limit, a persistent
`±2 m/s` TAS-offset deadline, and deterministic gap/trace/monotonicity
contracts before execution.

### Execution and result

The run completed once from clean commit `2789bab` with eight workers. It
processed 1,792 replications, 290,304 generated observations, and 194,688
NIS-fed observations. The compact artifact was committed as `b83f30c`:
[`airspeed_wind_residual_persistence_v6_train.json`](../validation/public/airspeed_wind_residual_persistence_v6_train.json).

The nuisance family produced zero false latches in 128 seed families; its
97.5% upper bound was `2.8408%` against the registered `3%` limit. Both signs
of the persistent TAS offset latched in all 128 families, but family P95
source-time detection delay was `3.5 s`, exceeding the registered `3.0 s`
limit. The status is therefore `failed_train_development_checks`; no v6 tune
was executed.

### Audit findings after the run

The result is retained without reinterpretation. A code/protocol audit found
that v6 scoring still allows an episode that starts before the injection and
latches after it to be counted as a clean post-injection detection. The tune
runner also did not enforce committed-train provenance or single-use output,
the delay P95 pooled two correlated signs instead of one worst delay per seed,
and the runner's non-latching residual-provider override was not declared in
the protocol. Finally, the full campaign used identical source and arrival
timestamps; unit tests cover their separation, but this train does not
validate arrival jitter, burst delivery, or arrival-only gaps.

These findings change the evaluation contract, so v6 is closed as failed
development evidence. They are not patched into v6 and no v6 tune/holdout is
permitted. A new v7 protocol with disjoint seeds must fix the attribution,
family aggregation, evaluator provenance, one-shot tune gate, and at least a
minimal arrival-timing integration matrix before any later promotion review.

## 2026-08-11 — v7 residual-persistence preflight implementation

### Scope and seed hygiene

V7 was created as a new protocol after the v6 train was closed. No v4/v5/v6
source, protocol, or result was rewritten. The preflight smoke used seed
`71001`; it is recorded as retired, and v7 train/tune use disjoint ranges
`71101--71612` and `72101--73124`.

### Implementation and checks

The v7 monitor now records immutable source/arrival episode onset and quiet
boundary timestamps. Mid-band residuals, low-residual episode aborts, gaps,
invalid samples, and epoch changes require fresh qualification. Gap facts are
preserved in the returned decision even when the gap-ending event has no NIS.
The residual evaluator override and geometry-gate ordering are declared in the
protocol and represented by a canonical hash. Bounded-jitter delivery is
generated separately from source noise and is exercised through the evaluator
to monitor pipeline. Persistent delay scoring uses one family maximum per seed
and includes a fixed failure sentinel in the primary P95. An exclusive
`tune_started` receipt is created before any tune worker starts.

Focused v7 tests pass `26/26`; the full Python suite passes `274/274`. A fresh
Release CMake configuration also builds and runs `17/17` CTest targets. The
optional PX4 comparison tests remain blocked only because their external PX4
source tree is not present; they are not part of this v7 monitor result.

The train entry point also rejects a dirty worktree, so its provenance commit
cannot silently describe uncommitted protocol or runner changes.

### Late aggregate-scoring audit fix

Before the full v7 train, an aggregate-scoring inspection found that the sole
`structural_gap` case intentionally has no nominal counterpart but the old
loop still attempted paired-null attribution for every injected case. The v7
protocol now marks that exception explicitly with `paired_null_required: false`.
All other injected cases are required to resolve exactly one matched null during
protocol loading; a focused aggregate test proves the structural-gap record is
still scored for its own fail-closed contract without being mislabeled as a
performance detection pair. This is a pre-train correction to a new v7
protocol, not a change to any historical v4/v5/v6 evidence.

### Current status

The preflight was committed as `023a0a4` and pushed before execution. The
single v7 train then ran once from that clean commit using eight workers. It
completed 8,704 replays / 1,410,048 generated observations / 958,464
NIS-fed observations and wrote
[`airspeed_wind_residual_persistence_v7_train.json`](../validation/public/airspeed_wind_residual_persistence_v7_train.json)
with an initially clean worktree provenance record.

The status is `failed_train_development_checks`. Nuisance false latches were
`3/512` (point `0.5859%`, 97.5% upper `1.7027%`) and met the registered
`3%` upper-bound limit. The persistent `±2 m/s` family failed `292/512`
(point `57.0313%`, 97.5% upper `61.3654%`), with only 220 fully successful
families and a failure-sentinel family P95 of `6.001 s`. All deterministic
contracts passed. The failure is dominated by no post-injection latch: 247
positive and 234 negative aligned cases (with equal counts for bounded
jitter); 11 positive and 14 negative cases per delivery profile were correctly
classified as pre-existing/ambiguous.

No v7 tune claim or tune run is permitted. A true 20 Hz burst and
arrival-only-gap campaign is still deliberately not claimed. Any attempted
improvement must begin as v8 with disjoint seeds, preserving the v7 protocol,
code, result, and failure records exactly as executed.

### Post-run root-cause replay

A read-only same-stream replay distinguishes a monitor-policy limitation from
a scorer defect. In failed positive seed `71101`, injection begins at source
`69.0 s`. A pre-injection NIS of `2.075` at `67.5 s` is mid-band, so v7 clears
its quiet boundary. The quiet samples at `68.0` and `68.5 s` span only `0.5 s`,
below the required `1.0 s`; the following high residuals therefore arrive
while `UNQUALIFIED` and cannot form an episode. This behavior is deliberate:
at 2 Hz a `1.5 s` episode requires four high samples, and v7 cannot recreate a
quiet boundary after a persistent fault starts. The historical v6 monitor
latches the identical stream only because it preserves its old quiet boundary
across mid-band evidence; that is not an acceptable v7 attribution shortcut.

The v8 design question is therefore not “how to make v7 pass.” It is whether a
bounded recent quiet boundary or graded causal evidence can improve sensitivity
without restoring pre-onset episode inheritance. That hypothesis requires a
new protocol, candidate code, disjoint seed ranges, and fresh pre-registered
tests.

### 2026-08-11 — v8 policy-shape diagnostics and fresh screen

An independent host-only probe was added without changing v7, the production
ESKF/Mahony code, or the public API. It compares `recent_boundary`,
`graded_evidence`, and `bounded_retry` shapes using only source/arrival time,
epoch, validity, and NIS. Focused semantics cover mid-band onset, expired
boundaries, source/arrival gaps, invalid input, retry exhaustion, and same-epoch
requalification after a gap. The first high sample in graded mode now starts
with zero accumulated high evidence; a focused test locks that rule.

Focused validation completed `7/7` tests and `45` hand-authored policy/case/
age comparisons. The retained v7 seed `71101` was replayed read-only: v7 did
not latch, while the unselected candidates latched at source `70.0--70.5 s`.

A separate fresh development screen then ran once on seeds `74101--74132`
(`32` families, `16` cases per family, `512` replays, `8` workers). It produced
zero nominal false latches and zero structural-gap latches for all candidates.
Persistent clean-family passes at recent-boundary ages `1/2/3 s` were:

```text
recent_boundary: 19/32, 25/32, 27/32
graded_evidence: 20/32, 26/32, 27/32
bounded_retry:   19/32, 25/32, 27/32
```

This is a clear sensitivity improvement over the v7 failure pattern but does
not meet the provisional `>=95%` clean-attribution screen target. Seed `74123`
contains a high episode that starts before the offline injection boundary and
continues through it; it remains an attribution failure by design. At age `3 s`
the 1.0 s pulse latched `0/32`, the 1.5 s pulse `1/32`, and the 2.0 s pulse
`28/32`. The screen is therefore retained as non-promoting evidence; no policy
or age is selected, and no v8 train/tune/holdout has been opened.

The large trace remains under `build/` only. A compact summary with the full
trace SHA-256, source hashes, seed manifest, and aggregate counts is retained
in `validation/public/airspeed_wind_residual_persistence_v8_screen_74101_32_summary.json`.

### 2026-08-11 — partial-quiet probation candidate opened

The first screen showed that many misses occur after a short post-mid-band
quiet run that is too short for v7's full one-second boundary. A separate
diagnostic shape was therefore added without changing the prior screen or v7:
two quiet observations spanning `0.5 s` may create a probationary boundary, but
an episode started from that boundary must contain at least five high samples
over `2.0 s`. Full boundaries keep the ordinary four-sample/`1.5 s` rule.
Boundary quality and episode-onset quality are now exported explicitly.

The focused contract remains deterministic and now covers four policy shapes.
Before the fresh screen, the partial candidate was tightened so it cannot
bootstrap initial qualification, cannot reuse a full boundary after a
post-degradation interruption, and has an independent finite retry budget.

### 2026-08-11 — v8 partial-policy follow-up screen

A new, disjoint screen ran on seeds `74201--74216` with the same 16 cases per
family (`256` stream replays, ages `1/2/3 s`). The full trace is retained only
under `build/airspeed_wind_residual_persistence_v8_screen_74201_16.json`; the
compact summary is committed at
`validation/public/airspeed_wind_residual_persistence_v8_screen_74201_16_summary.json`.

All four candidates had zero nominal false latches and zero structural-gap
latches at every age. Clean persistent-family passes were:

```text
age 1 s: recent 8/16, graded 9/16, bounded 8/16, partial 5/16
age 2 s: recent 13/16, graded 15/16, bounded 13/16, partial 5/16
age 3 s: recent 15/16, graded 16/16, bounded 15/16, partial 5/16
```

The partial candidate's sensitivity loss is retained as a negative result;
its safety constraints were not relaxed to improve the score. Graded evidence
at age 3 s is only a provisional comparator from a small non-holdout screen;
no v8 policy, age, threshold, estimator state, or FCOne authority is selected.

The remaining misses were reviewed separately in
`docs/airspeed-wind-residual-persistence-v8-failure-analysis.md`. The failure
is an information/attribution boundary after the quiet boundary expires, not a
numerical or transport defect. The next candidate must therefore separate a
causal diagnostic anomaly from a control-qualified fault and add an independent
regime/source transition before any supervisor authority is considered.

### 2026-08-11 — v8 boundary and rate semantics locked

Focused coverage was extended to lock the exact edge conditions identified by
the failure review. At the configured 3 s boundary age, an episode starting at
exactly 3.000000 s is admitted; an onset at 3.000001 s is rejected; and an
episode admitted before expiry may finish after expiry because authorization is
snapshotted at onset. Graded evidence was also exercised under irregular source
spacing and bounded arrival jitter, confirming that the evidence threshold uses
source time rather than sample count or arrival cadence. The v8 focused suite
now passes `15/15`; these are contract tests, not new selection evidence.

### 2026-08-11 — v8 diagnostic/control-qualification boundary preflight

The v8 screens showed that NIS and timing alone cannot causally identify the
start of a persistent air-data/model mismatch once the quiet boundary has
expired. Extending that stale boundary would import pre-onset ambiguity into
control authority. The next step was therefore implemented as a host-only,
orthogonal diagnostic lane rather than another permissive policy candidate.

`CausalPolicyProbe` now records an immutable
`UNQUALIFIED_PERSISTENT_RESIDUAL` snapshot only when the authorized epoch has
previously completed a full quiet boundary, the boundary is no longer recent,
the source/arrival stream remains continuous and valid, and strict high NIS
count/span requirements are met. The normal `ProbeState` and `latched` result
are unchanged; the new status cannot switch a source, reset the estimator, or
affect a controller. Low/mid-band interruptions and transport/epoch faults
clear unfinished diagnostic evidence fail-closed.

The contract is frozen in
`validation/airspeed_wind_residual_persistence_v8_diagnostic_lane_protocol.json`
and documented in
`docs/airspeed-wind-residual-persistence-v8-diagnostic-lane.md`. Ten new
focused tests cover stale-boundary diagnosis, normal fresh-boundary latching,
startup prohibition, interruption, gaps, exact/just-past age, source-time
count/span, snapshot immutability, and the non-authority invariant. The v8
comparator suite plus the new lane suite pass `25/25`; repository CTest remains
`17/17`. No production C, ESKF, Mahony, adapter, or FCOne code changed.

The next execution is a disjoint, non-promoting diagnostic screen. Its full
trace stays under `build/`; only a hash-linked compact summary is eligible for
`validation/public/`. Control qualification remains unimplemented until an
independently audited regime/source transition and corroborating source are
defined and separately validated.

### 2026-08-11 — v8 diagnostic-lane 64-family screen and attribution audit

The frozen diagnostic implementation at `2bdb109` ran once on seeds
`74301--74364`, `17` cases per seed, and three recent-boundary ages with eight
workers. The campaign completed `1,088` replays. Its full `1.3 GiB` trace is
kept only under `build/`; SHA-256 is
`1d5efa87fe67f6152775f1e7018922c639ffbafe15fc4b0f01c7627ceb77259d`.
The compact summary was retained in commit `f14bd76`.

A separately committed streaming audit (`785d63d`) then verified the full
trace hash and all `1,088` records without loading the artifact wholesale. It
separates raw control latches, clean post-injection control attribution,
pre-existing ambiguity, diagnostic-only persistence, and a descriptive union.
For `graded_evidence @ 3 s`, raw control latches were `234/256`, but ten began
before injection and cannot be credited. Clean control coverage is `224/256`
members and `55/64` complete families. The diagnostic lane adds `14` clean
members and rescues three complete families, producing a descriptive union of
`238/256` members and `58/64` families. Six families remain unresolved.

No nominal or bounded-jitter nominal case produced a diagnostic event; the
calibrated-high-noise nominal and structural-gap cases also remained at zero.
The result is retained as non-promoting development evidence. The descriptive
union improves logging/triage coverage only and is explicitly not a control
qualification, source selector, estimator reset, or FCOne authority.

### 2026-08-11 — v8 diagnostic provenance correction and paired replay

The first attribution audit exposed a diagnostic-only provenance loss. If a
mid-band point expired the normal quiet boundary before a later strict-high
episode, the normal lane correctly removed authority but the diagnostic lane
also removed the timestamp proving that a full quiet baseline had existed in
the current continuous epoch. This caused long high sequences such as seed
`74354` to remain invisible to the diagnostic output.

The timestamp is now retained in a separate non-authoritative field. Focused
tests prove that it cannot reactivate a normal boundary, cannot bridge a
mid-band interruption, and still resets on source/arrival gaps, invalid input,
epoch changes, and reauthorization.

The exact `1,088` opened streams were paired-replayed from clean commit
`5568b8e`. All `13,056` primary policy comparisons matched the pre-fix result;
only diagnostic snapshots changed. At `graded_evidence @ 3 s`, diagnostic-
clean members increased from `14` to `22`; the descriptive union increased to
`246/256` members and `61/64` families. The remaining ten members are exactly
the pre-existing ambiguous episodes in families `74306`, `74349`, and `74352`.

Normal, jittered-normal, calibrated-high-noise normal, structural-gap, and
`1.5 s` pulse diagnostic counts remain zero. Five `2.0 s` pulses produce a
diagnostic-only record at age `3 s`. The replay is regression/root-cause
evidence on an opened screen, not a fresh error-rate confirmation or control
promotion.

### 2026-08-11 — v8 fresh diagnostic confirmation frozen

Before opening more data, the diagnostic confirmation was pre-registered on
seeds `74401--74528` (`128` families, `17` cases each, `2,176` replays). This
volume is intentional: with zero nuisance events, its 97.5% one-sided
Clopper–Pearson upper bound is `2.8408%`; a 64-family run cannot meet the
registered `3%` confidence limit.

The selected development comparator is `graded_evidence @ 3 s`. The frozen
checks require nuisance-family point rate `<=1%`, confidence upper bound
`<=3%`, persistent descriptive-union family coverage `>=95%`, zero structural-
gap diagnostic events, and zero `1.5 s` pulse diagnostic events. The `2.0 s`
pulse remains descriptive. Other policies and ages remain comparators. The run
is single-use opened development confirmation, not a holdout, and cannot
promote control authority even if every check passes.

### 2026-08-12 — TAS residual persistence experiment R8 fresh confirmation failed

The pre-registered confirmation ran once from frozen commit `5213c53` on seeds
`74401--74528`: `128` independent families, `17` cases per family, and `2,176`
replays. The complete trace remains outside Git at
`build/airspeed_wind_residual_persistence_v8_diagnostic_lane_confirmation_74401_128.json`
with SHA-256
`c89cf80e62033b6d4c116b815024ffdf67efb3440bee9d80e4b95fa477f055b2`.
The compact screen summary and independent streamed attribution audit are
retained under `validation/public/`.

The selected `graded_evidence @ 3 s` comparator did **not** pass the registered
development gate. Nominal diagnostic events were `0/128`, giving a 97.5%
one-sided Clopper-Pearson upper bound of `2.8408%`, and structural-gap
diagnostic events were `0`; both checks pass. Persistent descriptive-union
coverage was `117/128 = 91.40625%`, below the required `95%`, and the `1.5 s`
pulse produced two diagnostic events, violating the zero-event check. The
descriptive union contains `438/512` clean normal-control members and `32/512`
additional diagnostic-clean members, for `470/512` members total. Eleven
families therefore remain unresolved.

This failure is preserved without retuning against these seeds. It authorizes
no source switch, wind state, estimator reset, private FCOne supervisor action,
or flight-control decision. `R8` is an experiment revision only; it is not an
FCOne hardware or firmware version. The next hypothesis must introduce
independent physical corroboration rather than another stale-boundary policy
tune.
