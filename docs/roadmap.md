# Development roadmap

This roadmap separates work that can be completed on a PC from work that requires the FCOne v2
hardware. The public algorithm repository remains the only editable source for Mahony, ESKF, and
hardware-independent validation code. FCOne repositories consume a reviewed commit through an
adapter; they do not maintain a second algorithm copy.

Public branch, history, and tagging rules are defined in the [release policy](release-policy.md).
The scoped definition of PX4-class capability, evidence grades, anti-overfitting rules, and G0--G4
acceptance gates are defined in the [PX4-class validation plan](px4-class-validation-plan.md).
The eVTOL flight-regime profiles and public/private source-of-truth loop are defined in
[FCOne/eVTOL target profiles and repository flow](vehicle-target-and-repository-flow.md).
The consolidated pre-hardware entry decision, state-dimension policy, and shadow-integration
boundary are defined in [FCOne v2 algorithm closure](fcone-v2-algorithm-closure.md).

## Current baseline

- Public `main` recovery point: `ef03c348e4d73d7e5461c9f4aec0b112760cffda`.
- The recovered private checkpoint has the same source tree as public `main`.
- Strict host build plus the C and Python test suites pass on the recovered local machine.
- Twenty-four private PX4 ULogs and the minimal EuRoC inputs are retained outside the public Git
  history and have verified SHA-256 manifests.
- Flight qualification, target timing, thermal behavior, and physical heading truth remain open.

## P0 — hardware-independent closure

The validation-only fixed-lag covariance composition prerequisite is now
closed for the declared host scope: 384 variable-rate chains and 4,592 IMU
intervals match repeated production `F/Q` propagation in double and host
float. This does not close measurement cross covariance, relinearization,
source/arrival timing, or a product smoother.

The double-precision P/V cross-covariance transaction is also now verified by
a 30x30 augmented oracle. Host float loses PSD in the deliberately near-
singular joint covariance despite matching the live 15-state block, so a naive
covariance-form fixed-lag implementation is not an acceptable STM32H7 path.
Square-root/UD or bounded PSD-repair evidence is a prerequisite for that
future target design.

| Work item | Acceptance evidence | Status |
| --- | --- | --- |
| One-command host regression | Configure, build, C tests, Python tests, deterministic scenarios, threshold checks, logs, and environment manifest complete from one command | Implemented by `validation/run_host_regression.py`; keep it passing |
| Monte Carlo consistency | Reviewed seed set covers IMU bias/noise, timestamp jitter, aiding loss, and recovery; aggregate NIS/NEES confidence bounds and failure seeds are reported | Constant three-axis bias, monotonic timestamp jitter, measurement noise, and five-second aiding loss implemented; thermal drift and transport faults pending |
| Trusted-heading behavior | Controlled heading covers cold-start yaw, geometry validity, normal updates, dropout, outliers, rejection, and recovery without treating GNSS course as body yaw | Synthetic/Vicon-derived fault gates and INSANE physical dual-RTK input implemented; independent physical yaw truth remains P2 evidence |
| Independent absolute heading | A physical magnetometer or dual-antenna heading input is scored against an independent six-degree-of-freedom truth source with audited time/frame/source lineage | INSANE `indoor_1` provides an adverse same-sequence local-datum A candidate: magnetometer on/off ESKF yaw RMSE `12.53°/0.27°`; rebuilt calibration/development A/B also fail with all physical updates accepted; `transition_1` failed cross-sequence datum transfer at `49.30°` initial residual and is diagnostic only. A causal source-supervision protocol is frozen before opening `indoor_2`; surveyed North, causal cold start, and broader regimes remain open |
| Bias convergence | Gyroscope and accelerometer bias error, convergence time, and steady-state uncertainty are reported separately; batch reference-bias correction is not counted as online convergence | Multi-axis synthetic gate implemented; thermal and physical characterization remain P2 evidence |
| G0 v2 IMU input contract | Physical timestamped interval semantics, delta-angle/delta-velocity audit fields, causal quantized-IMU stationarity, and rate-invariant noise density | Implemented as a four-rate integrity smoke; this is not estimator-capability evidence. Timestamp transport faults, thermal/random-walk profiles, and private FCOne selector policy remain open. See [G0 v2 input contract](bias-observability-v2-input-contract.md). |
| Airspeed/barometer degraded navigation | Paired IMU-only, raw barometer, supervised barometer, hot-shadow failover, known-wind TAS, estimated-wind TAS, and combined arms cover fixed-wing/VTOL regimes and 5-120 s GNSS outages | The v1 `600/600` synthetic matrix is complete and hashed: nominal 120 s vertical RMSE is `0.1008 m` raw / `0.1014 m` supervised versus `41.6243 m` IMU-only; timestamp delay is rejected. Persistent datum bias, source-reset behavior, real full-state failover, sealed physical pressure tests, and airspeed/wind remain open. See [barometer study](barometer-supervision-study.md). |
| Timing and malformed input | Duplicate, stale, out-of-order, delayed, missing, non-finite, and implausible samples have explicit deterministic behavior and tests | Implemented deterministic matrix plus 1,020,000-attempt, 100-seed campaign; retain as a host gate |
| Delayed-GNSS replay prerequisite | Exact source-timestamp rewind, full snapshot restore, existing GPS update, deterministic IMU/aiding replay, state/covariance equivalence, and fail-closed scope | Host-only isolated P/V oracle passes 12/12 and sequential non-overlapping two-event P/V oracle passes 12/12 at 100/200/400 Hz and 20/50/100/150 ms. A distinct overlapping/reordered two-event P/V oracle passes 9/9 at 100/200/400 Hz and 50/100/150 ms, requiring the newer delivery to remain non-equivalent while the older source is pending. Arbitrary multi-event schedules, product API, and FCOne arrival-time path remain open |
| Public dataset runner | Dataset manifest records source, hash, frame transform, time offset, command, code commit, and output summary; selected datasets reproduce with one command | Implemented for 6 EuRoC, 3 Blackbird, and 2 UrbanNav tracks; 398,493 unique external-reference IMU samples and 865,845 replay attempts; keep all reviewed baseline gates passing |
| High-volume PX4 compatibility | Audit current PX4 schemas at corpus scale, select stress tracks before scoring, and retain resets/clipping/innovation failures without treating PX4 estimates as truth | IDF-DS audit complete: 13 raw ULogs, 7.13 million IMU samples over 9.92 h; three selected native replays total 1.61 million samples and expose high NIS/recovery counts |
| Aerial physical position/reference | Exercise physical aircraft IMU and GPS position input against a separately recorded RTK position/velocity path without synthesizing absent receiver fields | Electrical-survey `voo_3` complete: 16,560 replay samples, 2,070 GPS position updates, RTK reference path; physical drone-GPS velocity remains absent |
| Recorded GNSS degradation | Recorded receiver data and independent navigation truth cover nominal aiding, a real outage, drift, and reacquisition without inventing missing receiver fields | UrbanNav Medium Urban 1 complete: 314,185 IMU samples, 655 valid F9P position epochs, one 131 s outage; physical receiver velocity and aircraft dynamics remain complementary gaps |
| No-aiding output qualification | Numerical health remains separate from horizontal navigation validity; accepted constraints refresh validity, rejected data do not, and a configurable timeout is enforced | Default five-second timeout implemented and exercised over the recorded 131 s UrbanNav outage; exact FCOne failsafe policy remains private P1 work |
| Same-input PX4 comparison | A pinned PX4 EKF2 and Aerakia consume identical immutable IMU/aiding data, initialization track, masks, delays, and outage windows; validity, drift, recovery, consistency, CPU, and memory are reported | Official `ecl_EKF` M0 host harness completed: 65 s / 100 Hz synthetic same-input replay produced 6,489 aligned horizons with 100% numerical health in both estimators. Both missed the 35 s bias-convergence gate, so this closes transport/provenance only; no parity, non-inferiority, or flight-readiness ratio is claimed. G2 breadth remains open. |
| FCOne-neutral contract tests | A mock publisher verifies timestamp, FRD/NED frames, SI units, validity flags, update freshness, dropout, and stale-aiding behavior without including private FCOne headers | Executable oracle implemented; exact private FCOne v2 adapter remains P1 |
| Estimator-supervisor contract | Mock modes prove ESKF-primary, Mahony attitude-only degradation, output invalidation, transition logging, hard-vs-soft failure handling, hysteresis, continuity on fallback/recovery, finite fallback duration, and recovery without implementing private flight policy in the public core | Executable host contract implemented; private FCOne policy remains P1 |

Run the complete current host gate with:

```bash
python validation/run_host_regression.py
```

Generated logs, reports, plots, and `run-manifest.json` stay under `build/` and are not committed.

Run the first reviewed 20-seed navigation consistency baseline after building the native runner:

```bash
python validation/run_monte_carlo.py \
  --runner build/host-regression/aerakia_validation_runner
```

This phase deliberately reports empirical P05/P95 ranges across seeds. It covers drawn constant
three-axis IMU biases and monotonic timestamp jitter. Transport gaps, reordering, malformed values,
burst loss, and aiding freshness are now covered by the separate input-integrity campaign; thermal
drift remains hardware-required evidence.

Absolute-heading source independence and the airspeed/barometer model boundaries are defined in
[Absolute-heading evidence](absolute-heading-evidence.md) and
[Airspeed and barometer aiding plan](airspeed-barometer-plan.md). The machine-readable paired plan
is [`validation/airspeed_barometer_validation_plan_v1.json`](../validation/airspeed_barometer_validation_plan_v1.json).

### Newly audited G0 blockers

Before additional dataset volume is promoted as capability evidence:

1. restore executable F/Q/H finite-difference gates;
2. make NIS thresholds measurement-dimension-aware;
3. replace unconditional rejection-count re-anchoring with supervisor-authorized, source-quality-
   checked, bounded and probationary recovery;
4. add continuous heading and vertical validity;
5. separate reproducibility baselines from one-way capability limits;
6. add warnings-as-errors and sanitizer CI.

These items precede numerical PX4 parity claims. PX4 M0 scaffolding and small independent-truth
dataset intake may proceed in parallel because they do not require changing the shared estimator
source.

### G0 closure candidate and retained failures

The audited G0 blockers are now implemented and executable:

- 10,000-case transition and full trusted-heading Jacobian campaigns, a 225-element Q structure
  oracle/PSD check, a reduced-continuous-model Q integration oracle, and magnetometer yaw-only
  boundary tests;
- degree-of-freedom-aware NIS gates;
- source/generation/quality-snapshot and time-window-bound supervisor-authorized recovery with
  bounded correction and probation;
- independent horizontal-position/horizontal-velocity, heading, vertical-position, and
  vertical-velocity validity;
- separate public reproducibility and one-way capability gates;
- repository-wide warnings-as-errors plus sanitizer CI;
- 32-seed accuracy/consistency invariance at 100/200/400/1000 Hz;
- a consumed 1,000-seed unbounded-prior confirmation that retained one 4-sigma startup-bias tail
  failure, followed by a newly frozen 1,000-seed calibrated-bias confirmation with 10,000-resample
  bootstrap bounds, zero hard failures, and zero health/recovery failures.
- protocol-fingerprinted Monte Carlo checkpoints that reject resume across code, binary, threshold,
  noise, timing, or bias-profile changes;
- a 137-case deterministic three-sigma bias-box campaign covering every axis/sign, all signed
  pairs, and all 64 six-dimensional corners.

G0 is not declared fully closed yet. The 137-case campaign passes every numerical-health and
navigation-consistency gate, but only 120 cases meet every accuracy/convergence gate. All 17 first-
run misses share the `+X/-Y` acceleration-bias direction and exceeded the reviewed 35 s settling
budget. A follow-up 32-seed paired diagnostic confirmed direction-sensitive convergence: the target
direction missed in 22/32 trials, its mirror in 8/32, and zero injected bias in 3/32. The threshold
is not widened after observing that result; the observability/covariance cause must be corrected or
the product budget reviewed independently.

The causal analysis and frozen non-multisine cross-validation design are recorded in
[Horizontal accelerometer-bias observability](bias-observability.md). New bias reports include
three-axis covariance/NEES, per-axis and terminal-window error, continuous-five-second convergence,
and explicit right-censoring. The runner now also exports the complete 5x5 marginal covariance for
right-error tilt x/y plus three-axis accelerometer bias, so their joint NEES retains the cross terms
without adding unobservable yaw.

The historical protocol smoke is deliberately not green at the capability layer: 15/15 trials
executed, all five zero-bias tracks passed, and all ten boundary-bias tracks failed stable
convergence. Audit found that this smoke also opened seed `30000` in both v1 holdout trajectories,
so v1 remains diagnostic evidence but cannot provide a blind release result. A 324-run train/tune
study subsequently rejected both a broader scalar bias prior and a larger bias random walk:
neither fixed the non-zero train cases and both left worse tail behavior. A subsequent frozen
576+576 paired train/tune A/B study tested a static-prior candidate and rejected it: the candidate
improved tune-only counts but left both non-zero-bias train directions at `0/16` and did not
generalize. The final generic host-only variant, a physically correlated static
tilt/accelerometer-bias startup prior, was also rejected after a paired 576+576 train/tune campaign:
aggregate improvement came with 22 zero-bias and five mirror-symmetry regressions. Its complete
result is [`g0_correlated_static_prior_rejection.json`](../validation/public/g0_correlated_static_prior_rejection.json).
The first materially different candidate, a no-injection causal 20 s fixed-lag MAP proposal, was
rejected over 144 causal v2 replays. Two later replay candidates are now also retained as negative
evidence: a 216-trial 5D complete-injection/replay campaign still had zero-bias and attitude
regressions, and a 60-trial full-15D nuisance/covariance screen improved navigation fit but not the
tilt/bias boundary reliably. Their focused records are [fixed-lag replay candidate](fixed-lag-replay-candidate.md)
and [full-state sensitivity screen](full-state-sensitivity-screen.md). Do not repeat static-prior,
scalar P/Q, or trust-region sweeps. The next G0 work requires a genuine lag/smoothing formulation
with full transition and process/preintegration covariance, innovation cross covariance, and
physical source/arrival timing. FCOne v2 physical IMU interval, causal stationarity, and source
quality contracts remain required before that work; a new candidate also needs a clean protocol and
sealed holdout.

The statistical result assumes each calibrated startup residual-bias component lies within three sigma
(`0.15 m/s²` accelerometer and `0.6 deg/s` gyroscope under the current provisional prior). FCOne
hardware characterization must confirm or replace that input contract. G0 does not close G1
physical-truth volume, G2 same-input PX4 non-inferiority, or G3 FCOne target evidence.

The current Q discretization remains explicitly a high-rate reduced-model approximation. Its
100--1000 Hz accuracy/consistency invariance and full frozen-coefficient Qd oracle now pass within
a declared stress envelope; this closes the bounded host claim, not unrestricted
transition/high-dynamic/target statistical consistency. The remaining boundaries and re-open
conditions are recorded in [process-noise discretization](process-noise-discretization.md).

### 2026-07-20 — G0 static-prior A/B decision

The paired study recorded in
[`validation/public/g0_static_prior_ab_study.json`](../validation/public/g0_static_prior_ab_study.json)
completed `576` train/tune trials for the frozen baseline and `576` for the static-prior candidate
(`1,152/1,152` execution success, zero execution failures). The baseline passed `175/576` with
`390` right-censored trials; the candidate passed `231/576` with `341` right-censored trials.

The candidate moved `49` paired trials from right-censored to settled and produced zero
baseline-pass-to-candidate-fail regressions, but the improvement was tune-only. Both arms retained
`0/16` passes for the non-zero-bias train directions while both retained `16/16` zero-bias train
passes. The candidate was rejected and no estimator-core or product configuration change was
accepted. The evidence supports unresolved tilt--horizontal-accelerometer-bias observability
coupling, not a safely solvable scalar prior/process-noise setting.

This historical study is diagnostic rather than publication-grade blind evidence: compact mode
deleted raw per-trial CSVs, summaries predated per-trial input SHA/byte/row provenance, the runs
started from a dirty tree, and the v1 smoke had exposed seed `30000` from both holdout families.
No old SHA values were reconstructed or fabricated. New compact campaigns now seal input SHA-256,
byte count, and data-row count before deleting CSVs, and record protocol and marker-manifest
fingerprints.

The public information-marker manifest remains `draft` and analyzer-only. A protected release
holdout without a public marker continues to run ordinary metrics and gates but is classified as
`unavailable_sealed_holdout`; it must not be treated as an execution failure or given a public
information-state label. A future v2 protected marker path must be separate from the public
manifest.

## P1 — FCOne v2 integration before hardware arrival

1. Freeze the FCOne v1 and BSP-validation source commits used by each migrated module.
2. Keep FCOne v2 as a private, independent build; remove build-time references to sibling v1
   directories as modules are migrated.
3. Pin this repository as a submodule or equivalent immutable dependency.
4. Implement a private `FCOneAerakiaAdapter` for calibrated sensor publications and estimator
   outputs. Board registers, CubeMX, HAL, RTOS, and product logic stay private.
5. Run the adapter against synthetic and recovered replay streams in shadow mode before it can
   affect a control path.

## P2 — hardware-required evidence

- Board bring-up and per-peripheral acceptance using the BSP-validation evidence as the reference.
- Static multi-orientation and thermal IMU characterization.
- Motor-off, throttle-sweep, and recovery magnetic-disturbance tests.
- Target-MCU execution time, stack, precision, watchdog, and long-run numerical checks.
- Rate-table, motion-capture, or dual-antenna GNSS heading truth.
- HIL and bounded flight-envelope expansion with logged abort criteria.

## Data still worth collecting

Additional ordinary ULogs are lower priority. The highest-value new evidence is:

1. Valid dual-antenna GNSS heading during static rotation and dynamic flight.
2. Independent yaw truth from motion capture or a rate table with synchronized IMU/magnetometer.
3. Controlled GNSS interruption and recovery with detailed aiding-source innovations.
4. Controlled motor-current magnetic-disturbance data at fixed vehicle attitude.
5. Recorded receiver position and Doppler velocity with independent navigation truth, preferably on
   an aerial platform; UrbanNav closes position-outage coverage and the electrical survey adds an
   aerial RTK velocity reference, but neither publishes physical receiver velocity as filter input.
6. INSANE `indoor_1` and `transition_1` for independent OptiTrack attitude/yaw and indoor-to-outdoor
   transition coverage, followed by RTK-SLAM for surveyed long GNSS-degradation checkpoints.

No physical dataset should be described as ground truth unless its independent measurement chain,
frame transform, and synchronization uncertainty are recorded.
