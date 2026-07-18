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

## Current baseline

- Public `main` recovery point: `ef03c348e4d73d7e5461c9f4aec0b112760cffda`.
- The recovered private checkpoint has the same source tree as public `main`.
- Strict host build plus the C and Python test suites pass on the recovered local machine.
- Twenty-four private PX4 ULogs and the minimal EuRoC inputs are retained outside the public Git
  history and have verified SHA-256 manifests.
- Flight qualification, target timing, thermal behavior, and physical heading truth remain open.

## P0 — hardware-independent closure

| Work item | Acceptance evidence | Status |
| --- | --- | --- |
| One-command host regression | Configure, build, C tests, Python tests, deterministic scenarios, threshold checks, logs, and environment manifest complete from one command | Implemented by `validation/run_host_regression.py`; keep it passing |
| Monte Carlo consistency | Reviewed seed set covers IMU bias/noise, timestamp jitter, aiding loss, and recovery; aggregate NIS/NEES confidence bounds and failure seeds are reported | Constant three-axis bias, monotonic timestamp jitter, measurement noise, and five-second aiding loss implemented; thermal drift and transport faults pending |
| Trusted-heading behavior | Controlled heading covers cold-start yaw, geometry validity, normal updates, dropout, outliers, rejection, and recovery without treating GNSS course as body yaw | Synthetic/Vicon-derived fault gates and INSANE physical dual-RTK input implemented; independent physical yaw truth remains P2 evidence |
| Bias convergence | Gyroscope and accelerometer bias error, convergence time, and steady-state uncertainty are reported separately; batch reference-bias correction is not counted as online convergence | Multi-axis synthetic gate implemented; thermal and physical characterization remain P2 evidence |
| Timing and malformed input | Duplicate, stale, out-of-order, delayed, missing, non-finite, and implausible samples have explicit deterministic behavior and tests | Implemented deterministic matrix plus 1,020,000-attempt, 100-seed campaign; retain as a host gate |
| Public dataset runner | Dataset manifest records source, hash, frame transform, time offset, command, code commit, and output summary; selected datasets reproduce with one command | Implemented for 6 EuRoC, 3 Blackbird, and 2 UrbanNav tracks; 398,493 unique external-reference IMU samples and 865,845 replay attempts; keep all reviewed baseline gates passing |
| High-volume PX4 compatibility | Audit current PX4 schemas at corpus scale, select stress tracks before scoring, and retain resets/clipping/innovation failures without treating PX4 estimates as truth | IDF-DS audit complete: 13 raw ULogs, 7.13 million IMU samples over 9.92 h; three selected native replays total 1.61 million samples and expose high NIS/recovery counts |
| Aerial physical position/reference | Exercise physical aircraft IMU and GPS position input against a separately recorded RTK position/velocity path without synthesizing absent receiver fields | Electrical-survey `voo_3` complete: 16,560 replay samples, 2,070 GPS position updates, RTK reference path; physical drone-GPS velocity remains absent |
| Recorded GNSS degradation | Recorded receiver data and independent navigation truth cover nominal aiding, a real outage, drift, and reacquisition without inventing missing receiver fields | UrbanNav Medium Urban 1 complete: 314,185 IMU samples, 655 valid F9P position epochs, one 131 s outage; physical receiver velocity and aircraft dynamics remain complementary gaps |
| No-aiding output qualification | Numerical health remains separate from horizontal navigation validity; accepted constraints refresh validity, rejected data do not, and a configurable timeout is enforced | Default five-second timeout implemented and exercised over the recorded 131 s UrbanNav outage; exact FCOne failsafe policy remains private P1 work |
| Same-input PX4 comparison | A pinned PX4 EKF2 and Aerakia consume identical immutable IMU/aiding data, initialization track, masks, delays, and outage windows; validity, drift, recovery, consistency, CPU, and memory are reported | Official `ecl_EKF` M0 architecture and fairness contract reviewed; harness/results pending, so no numerical PX4/Aerakia accuracy ratio is claimed |
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

The first protocol smoke is deliberately not green at the capability layer: 15/15 trials execute,
all five zero-bias tracks pass, and all ten boundary-bias tracks fail stable convergence. A 324-run
train/tune study subsequently rejected both a broader scalar bias prior and a larger bias random
walk: neither fixed the non-zero train cases and both left worse tail behavior. The next G0 work is
therefore an excitation-aware tilt/bias candidate, followed by one opening of the 1,152-trial
holdout only if train/tune selects it. The release holdout is not run merely to produce a larger
known failure set before a candidate exists.

The statistical result assumes each calibrated startup residual-bias component lies within three sigma
(`0.15 m/s²` accelerometer and `0.6 deg/s` gyroscope under the current provisional prior). FCOne
hardware characterization must confirm or replace that input contract. G0 does not close G1
physical-truth volume, G2 same-input PX4 non-inferiority, or G3 FCOne target evidence.

The current Q discretization is also explicitly a high-rate reduced-model approximation. Its
100--1000 Hz accuracy/consistency invariance passes, but a full coupled continuous-model Qd oracle
remains a mathematical hardening item before claiming unrestricted transition/high-dynamic
statistical consistency.

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
