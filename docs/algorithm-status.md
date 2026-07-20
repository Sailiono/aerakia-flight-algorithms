# Algorithm status

## Current verdict

The portable estimator core is suitable for FCOne integration work and further bench testing,
but it is not yet justified to claim that the complete flight-estimation system is verified.
The current evidence establishes deterministic host behavior, measurement integrity handling,
long-run covariance health, synthetic cold start, private PX4-referenced replay tracking, and two
EuRoC external-reference sequences. Blackbird adds 270.1 s of independent motion-capture truth with
aggressive angular motion. UrbanNav adds 785.5 s of recorded 400 Hz IMU, physical F9P position
aiding, independent SPAN-CPT postprocessed truth, and a real 131 s GNSS outage. Vicon-derived
heading tests yaw alignment and faults under recorded
motion, and INSANE exercises the same production path with 1,378 physical dual-RTK observations.
IDF-DS adds a 9.92-hour, 13-ULog PX4 schema/coverage audit and 1,608,985 selected fixed-wing native
replay samples. The electrical-infrastructure UAV set adds physical DJI position aiding and a
separately recorded RTK position/velocity reference over 16,560 replay samples.
Neither derived/shared-source heading track establishes independent absolute heading accuracy or
flight safety.

## Evidence completed

| Area | Evidence | Status |
| --- | --- | --- |
| Error-state math | 15-dimensional error state, exact discrete SO(3) attitude transition, position `dt²/2` coupling, quaternion injection/reset Jacobian, and Joseph-form scalar/vector updates | 10,000-case F and complete raw-heading Jacobian campaigns pass below `1.7e-7` / `1.2e-8`; all 225 Q entries, PSD, and the declared reduced continuous model pass, while full coupled Qd remains open |
| Covariance health | Long mixed predict/update sequence checked for finite, symmetric, positive-semidefinite covariance | Passing |
| Measurement integrity | Dimension-aware NIS gates, latched magnetic rejection, and source/generation/quality-snapshot/time-window-bound, multi-sample, bounded, application-authorized probationary navigation recovery | Passing deterministic regressions; stale, source-switched, and standalone observations cannot force re-anchor |
| Heading semantics | Magnetometer, trusted heading, GNSS course, and PX4 GSF are separate paths; course is never silently treated as body yaw | Implemented |
| Trusted-heading fault behavior | Cold-start completion, normal fusion, four-second dropout, two 90° outliers, rejection, geometry validity, and recovery with magnetometer disabled | Passing deterministic synthetic and Vicon-derived fault gates; INSANE physical dual-RTK path passes 1,378 updates, but shares its yaw reference |
| Online IMU bias behavior | Static gyro initialization plus motion/GNSS-aided accelerometer-bias convergence, with truth error and settling time reported separately | Passing deterministic multi-axis synthetic gate; single-pose accelerometer observability limit and hardware thermal behavior remain explicit |
| Cold-start alignment | Static accelerometer tilt, magnetic heading with explicit declination, IMU-bias initialization, and covariance reset at the new linearization point; no PX4 attitude seed | Passing unit/noisy synthetic checks and direct-Vicon tilt (0.862° post-alignment RMSE); external yaw truth still pending |
| Navigation consistency | GNSS position/velocity NIS and posterior 6-state navigation NEES through a five-second outage and reacquisition | The 1,000-seed calibrated-bias distribution passes hard, bootstrap, consistency, health, and recovery gates; deterministic bias-box coverage retains a direction-sensitive 35 s convergence failure, and the preceding unbounded-prior confirmation retains one 4-sigma tail failure |
| EuRoC public replay | 36,381-sample Leica/IMU `MH_01_easy` and 20,932-sample direct-pose `V1_03_difficult`; raw, cold-start, derived-heading, and reference-bias tracks retained | Navigation NIS/NEES consistent; direct Vicon pose passes high-dynamic replay; derived heading is not a recorded heading sensor |
| Blackbird public replay | 26,995 recorded IMU samples over 270.1 s against independent motion capture; common time base and published body/IMU extrinsic are checked before conversion | ESKF 1.573° geodesic / 1.131° tilt RMSE, 100% health, zero recovery; retained Mahony drift proves fallback must be bounded |
| UrbanNav public replay | 314,185 recorded IMU samples, 655 checksum/quality-screened F9P positions, SPAN truth, and one 131 s recorded outage | Position-only API and recovery pass with 100% numerical health; horizontal navigation is valid for 83.90% of samples and explicitly expires after five unaided seconds; 6.52 m nominal aided RMSE and first-update reacquisition are retained alongside severe unaided drift and inconsistent NIS/NEES |
| IDF-DS fixed-wing volume | 13 raw PX4 ULogs, 7,128,090 audited IMU samples over 9.92 h; rotation, speed, and clipping tracks selected before scoring | 1,608,985 native replay samples remain healthy; 3.24–5.82° agreement with PX4 attitude and 2.07–3.62 m with PX4 position, but high NIS and 26–80 recoveries retain real delay/noise-model gaps |
| Aerial GPS/RTK reference | 16,560 physical DJI IMU samples, 2,070 physical GPS position updates, and separately recorded 5 Hz RTK position/velocity | 100% health; 0.179 m position and 0.263 m/s velocity RMSE against RTK reference; no receiver-velocity aiding or independent attitude/absolute truth claim |
| Host regression | Strict C99 warnings-as-errors build, public API tests, deterministic synthetic fault suite | Passing reviewed thresholds |
| Input/transport integrity | Exhaustive required-IMU non-finite checks; timestamp order/gap behavior; optional-mag isolation; timestamped GNSS/heading/barometer freshness and recovery | Passing 100 seeds, 1,020,000 IMU attempts, 2,100 aiding attempts, and burst lengths through 100 with zero invariant/health failures |
| Estimator supervision | ESKF-primary startup, hard-invalid immediate response, soft observability hysteresis, continuity-gated and time-bounded Mahony attitude-only degradation, navigation invalidation, continuity-gated recovery, and transition evidence | Passing executable public contract; private FCOne policy and actuator interaction remain open |
| Output qualification | Numerical health is distinct from independently aged horizontal position/velocity, heading, vertical-position, and vertical-velocity validity; only accepted applicable constraints refresh each age | Passing public API and UrbanNav outage gates; position-only, velocity-only, and ZUPT semantics are explicit, and startup alignment is not treated as continuing observability |
| FCOne-neutral adapter | Physical timestamp preservation, FRD sentinel axes, g/deg/s/gauss conversion, independent validity bits, missing data, duplicate/gap recovery, and future/stale aiding | Passing executable mock-publication contract; exact v2 message and scheduler remain open |
| Private replay | Sanitized relative GNSS, reset events, GSF diagnostics, and native C replay across the selected ULog suite | Operational; PX4 remains an engineering reference |

## P0 work before hardware flight tests

1. Add a second physical heading track with independent yaw truth, such as dual-antenna GNSS plus
   motion capture/rate table. INSANE closes the recorded dual-RTK software path but shares its yaw
   reference and cannot establish absolute sensor accuracy.
2. Keep heading geometry, source validity, variance, baseline quality, and freshness explicit.
   Validate the exact FCOne receiver status semantics before flight use.
3. Keep the implemented transport delay, reordering, sample loss, malformed-value, and aiding-age
   campaign passing; add hardware thermal drift only when temperature data are available.
4. Apply the passing neutral adapter oracle to the exact private FCOne message and scheduler when
   its v2 interfaces are available; keep the same timestamp, frame, unit, validity, and stale-data
   checks.
5. Add a complementary physical receiver track with recorded Doppler velocity and independent
   truth, preferably under aircraft dynamics. UrbanNav closes recorded position/outage coverage but
   has no receiver velocity, heading, magnetometer, or aircraft motion. The UAV electrical survey
   adds aircraft position aiding and an RTK velocity reference, but still does not expose physical
   drone-GPS velocity as estimator input.

## P1 work when the new hardware is available

- Static bench and thermal bias characterization.
- Rate-table or motion-capture attitude tests with independent truth.
- GNSS outage/reacquisition, magnetic disturbance, and location-change tests.
- Target-MCU timing, stack, precision, and numerical-stability measurements.
- HIL followed by bounded envelope-expansion flights with reviewed abort criteria.

This document is an engineering maturity statement, not an airworthiness claim.

## Current quantitative assessment

### Accuracy

- Controlled synthetic scenarios: ESKF full-attitude RMSE is 0.33–0.89° across clean motion,
  magnetic spikes/bias, navigation outage, trusted-heading recovery, and online-bias excitation.
- In the frozen 1,000-seed calibrated-input confirmation, attitude RMSE is `0.962°` mean,
  `1.712°` P95, and `2.981°` maximum; position is `0.393 m` mean / `0.639 m` P95, and velocity is
  `0.174 m/s` mean / `0.297 m/s` P95. The 99% bootstrap P95 upper bounds are `1.859°`, `0.683 m`,
  and `0.311 m/s` respectively. This result assumes per-axis startup residual biases within the
  provisional calibrated-input contract.
- Independent-reference EuRoC raw IMU: tilt RMSE is 0.691° on `MH_01_easy`, 1.041° on
  `V1_03_difficult`, and 0.860° after Vicon-declared cold-start tilt alignment.
- With a 10 Hz, 1°-noise heading stream derived from Vicon truth, the V1_03 cold start completes in
  1.0 s and post-alignment geodesic attitude RMSE is 1.549°. On geometry-observable samples yaw RMSE
  is 0.920°; two injected 90° observations are both rejected. These are real-motion software-path
  results, not recorded heading-sensor accuracy.
- On INSANE `outdoor_1`, 39,174 recorded UAV IMU samples and 1,378 physical dual-RTK heading
  observations produce 2.018° yaw RMSE with reference initialization. Independent cold start reaches
  1.534° post-alignment yaw RMSE, with 91.15% normal update acceptance and 100% estimator health.
  The scored yaw shares the RTK baseline, so this validates integration behavior rather than
  independent absolute accuracy. Full-attitude scores are excluded because the published tilt and
  measured gravity direction are inconsistent.
- On Blackbird `NYC Subway Winter`, 26,995 recorded IMU samples over 270.1 s reach 4.22 rad/s and an
  isolated 59.78 m/s² acceleration spike. Against independent motion capture, raw ESKF geodesic
  attitude RMSE is 1.573°, tilt RMSE 1.131°, and Euler-yaw RMSE 1.069°. Tilt-only cold start
  completes in 1.009 s with 1.131° post-alignment tilt RMSE. No heading alignment is claimed because
  the package has no magnetometer or trusted-heading observation.
- EuRoC full-attitude raw-IMU RMSE is 8.406° and 4.088°. This is dominated by yaw drift from the
  recorded approximately 0.08 rad/s z-gyro bias with no magnetometer or trusted heading; it is an
  observed limitation, not an acceptable heading-accuracy claim.
- Reference-bias-corrected EuRoC diagnostic tracks reach 2.974° and 2.572° full-attitude RMSE, but
  batch truth-bias subtraction is not online estimation and is never reported as flight accuracy.
- EuRoC position RMSE is 0.123–0.128 m and velocity RMSE 0.082–0.089 m/s with deterministic
  synthetic 10 Hz GNSS generated from external reference truth. These values validate fusion math,
  covariance, and replay determinism—not a physical GNSS receiver.
- Blackbird position/velocity RMSE is 0.123 m / 0.082 m/s with the same declared synthetic-GNSS
  evidence boundary; navigation NEES is 4.76 for expected mean 6.
- UrbanNav reference-initialized attitude reaches 1.534° geodesic, 0.772° tilt, and 1.328° yaw
  RMSE. Nominal position-aided RMSE is 6.518 m. The whole-run 247.99 m value is dominated by a real
  131 s GNSS outage and is not reported as nominal position accuracy.
- During that UrbanNav outage, position-only inertial error peaks at 1471.5 m and velocity error at
  27.40 m/s; the first resumed physical position update returns posterior position error to 5.395 m.
  With the default five-second limit, horizontal navigation is valid for 263,605 of 314,185 samples
  (83.90%) while numerical health remains 100%; rejected aiding cannot refresh that validity.
  Cold-start tilt reaches 2.211° RMSE, while yaw reaches 36.65° because the sequence provides no
  magnetometer or heading observation. This is a retained observability failure, not a yaw claim.
- Three selected long IDF fixed-wing replays remain healthy and agree with PX4 references at
  3.241–5.823° full attitude, 2.449–2.589° tilt, 2.073–3.619 m position, and 0.335–0.632 m/s
  velocity RMSE. PX4 is not truth; high NIS and 26–80 recoveries make these compatibility/stress
  results rather than absolute-accuracy evidence.
- The 41.4 s electrical-survey replay reaches 2.766° geodesic / 0.966° tilt RMSE against shared
  DJI onboard attitude and 0.179 m position / 0.263 m/s velocity RMSE against the separately
  recorded RTK reference. The unusually close DJI GPS/RTK trajectories and declared fixed `4 m²`
  aiding variance prevent an independent absolute-accuracy or consistency claim.

### Robustness

- Software/input robustness is strong for the tested contract: 1,020,000 IMU attempts, 2,100
  timestamped aiding attempts, five sample rates, bursts through 100 samples, exhaustive required
  non-finite axes, and zero state/covariance invariant or health failures.
- Numerical robustness is strong on the host: strict C build, long covariance checks, Joseph-form
  updates, finite-difference transition/raw-heading geometry, full Q structure and reduced-
  continuous-model process-noise oracles,
  and the full million-attempt campaign under ASan+UBSan.
- The new 1,000-seed confirmation has 100% healthy output, zero unexpected navigation recovery,
  zero hard/distribution/consistency/bootstrap failures, and a 95% zero-failure probability upper
  bound of `0.2991%`. Its navigation NIS means are `2.975/2.527` and six-state NEES mean is `5.024`.
- A deterministic 137-case residual-bias box covers all six axes/signs, all 60 signed isolated
  pairs, and all 64 six-dimensional corners at the provisional three-sigma bounds. Numerical health
  and navigation consistency pass in every case, but only 120/137 meet every convergence/accuracy
  gate. A 32-seed paired follow-up shows the `+X/-Y` accelerometer-bias direction misses the 35 s
  settling budget in 22/32 runs versus 8/32 for its mirror and 3/32 at zero injected bias. This is
  an open direction-sensitive observability/convergence defect, not a reason to widen the gate.
- Aggressive-motion robustness now includes Blackbird's independent motion capture: ESKF remains
  healthy for all 26,995 samples with zero navigation recovery. The same run retains a 51.48° raw
  Mahony full-attitude RMSE without magnetometer/heading; even static reference-bias subtraction
  leaves 10.13°. Therefore Mahony is classified as continuity-gated, finite-duration degraded
  attitude—not a second long-duration navigation solution.
- Estimation consistency is currently reasonable: EuRoC navigation NEES means are 4.94–5.19 for a
  six-dimensional expected mean of 6; position NIS is 3.06–3.08 for expected mean 3. Velocity NIS
  at 2.43–2.57 is mildly conservative rather than overconfident.
- UrbanNav exposes a different consistency failure that synthetic paired GNSS did not: position
  NIS means 0.011/0.339 are very conservative while six-state navigation NEES means 12.91/16.27 are
  overconfident. The position-only stream leaves velocity weakly observed, so this evidence blocks
  any claim that real-receiver covariance tuning is complete.
- Recorded-GNSS robustness now covers 314,185 IMU samples, 655 accepted physical position epochs,
  a 131 s outage, and immediate first-update position reacquisition with 100% finite/healthy output.
  It does not prove acceptable inertial navigation during long GNSS loss.
- Physical robustness remains only partly proven. The private ULogs add real fixed-wing/multirotor,
  clipping, magnetic disturbance, and PX4-reset coverage, but they lack independent truth. Thermal
  drift, vibration, installation error, motor current, real GNSS loss, and target timing await
  FCOne v2 or controlled public data.
- The Vicon heading stress track explicitly excludes 4,582 of 20,932 samples where the body-forward
  horizontal projection is below 0.25. During the controlled observable outage, maximum yaw error is
  5.100°; recovery takes 0.995 s and post-recovery yaw RMSE is 0.890°.
- The INSANE physical dual-RTK track adds 199.7 s of recorded UAV motion and maintains 100% health;
  97.10% of normal updates are accepted with reference initialization and 91.15% after cold start.
  Natural data has no declared heading faults, so outlier rejection remains proven by the separate
  injected-fault tracks.
- The IDF volume audit expands physical PX4 compatibility to 7.13 million IMU samples over 9.92 h;
  the selected 1.61 million-sample replays expose rather than conceal high innovation inconsistency.
  The aerial DJI/RTK replay adds a second physical aircraft position path but remains too short and
  too source-coupled to close receiver-delay or independent-navigation-truth requirements.

### Maturity judgment

| Scope | Current judgment | Reason |
| --- | ---: | --- |
| Portable algorithm/math implementation | 88–91% | Exact SO(3) F, raw trusted-heading Jacobian, scoped magnetic yaw correction, dimension-aware NIS, source/time-bound recovery, independent validity, covariance handling, and cold start pass; full coupled Qd, delayed fusion, and physical heading evidence remain open |
| Host-side software robustness | 95–97% | Million-attempt malformed/timing campaign, two disclosed 1,000-seed confirmations, bootstrap bounds, sanitizers, 7.13 million audited PX4 IMU samples, and 1.63 million native-C physical replays are retained |
| Heading robustness | 70–74% | Continuous validity, real-motion fault/recovery, and physical dual-RTK input pass; independent physical yaw truth and GNSS-velocity GSF fallback remain open |
| Navigation robustness | 76–80% | Source/time-bound recovery, probation, independent position/velocity validity, recorded outage/reacquisition, fixed-wing replay, and aerial position reference pass; real delay/receiver-velocity modeling remain open |
| FCOne integration readiness | 78–82% | Neutral adapter, independent aiding/validity, and bounded supervisor contracts are executable; exact redundant-sensor messages, scheduling, target precision, and resources are unverified |
| Flight-main-estimator readiness | 55–65% | Suitable for shadow mode and bench/HIL preparation; independent heading, target timing, sensor switching, delay compensation, and bounded flight evidence still block sole-estimator authority |

The highest-value non-hardware work remaining is a recorded receiver position-plus-Doppler-velocity
track with independent truth, a second independently scored physical-heading track, delay/time-
offset sensitivity, and applying the executable neutral contracts to private FCOne v2 interfaces.
The highest-value physical evidence remains synchronized independent yaw truth, thermal/vibration
characterization, motor magnetic disturbance, and target-MCU timing/stack measurements.

## 2026-07-18 Monte Carlo finding

The first formal 20-seed run found one reproducible cold-start transient: seed 17 reached 1.837°
post-alignment attitude RMSE because a GNSS position update used attitude-position covariance from
the pre-alignment linearization point and briefly pulled attitude by approximately 8–10°. Static
alignment had changed the nominal quaternion but had not reset its covariance relationships.

The correction adds an explicit attitude-covariance reset after absolute static alignment, clears
the stale attitude cross-covariances, and applies conservative configurable uncertainty floors of
2° for tilt and 10° for magnetic heading. After the correction all 20 seeds pass:

- position RMSE: mean 0.420 m, maximum 0.638 m;
- velocity RMSE: mean 0.187 m/s, maximum 0.300 m/s;
- post-alignment attitude RMSE: mean 0.712°, maximum 1.239°;
- position NIS mean across seeds: 2.943 (three degrees of freedom expected mean 3);
- velocity NIS mean across seeds: 2.495 (three degrees of freedom expected mean 3);
- navigation NEES mean across seeds: 5.047 (six degrees of freedom expected mean 6);
- navigation recoveries and unhealthy samples: zero.

These are synthetic measurement-noise results with a fixed five-second GNSS outage. They are not
physical truth, and their empirical seed percentiles are not theoretical independent-sample
confidence intervals.

## 2026-07-18 bias and timestamp extension

The same 20-seed navigation scenario now draws one constant three-axis accelerometer and gyroscope
bias per run (`σ=0.05 m/s²` and `σ=0.20°/s`) and applies monotonic per-interval timestamp jitter
(`σ=250 µs` at 100 Hz). Each generated input records the actual bias vector and interval extrema.

Three seeds exceeded the earlier noise-only 1.5° attitude gate, reaching 1.596–1.880°. One reached
0.3507 m/s against the earlier 0.35 m/s velocity gate. These runs remained finite, accepted normal
aiding, required no navigation reset, and retained reasonable NIS/NEES. The attitude increase is
consistent with the static observability limit between horizontal accelerometer bias and tilt; the
test does not claim that one stationary pose can identify both exactly.

The initial declared stress gate was therefore 2.0° post-alignment attitude RMSE and 0.40 m/s
velocity RMSE, while retaining the tighter deterministic noise-only CI thresholds. The subsequent
bias-observability audit below supersedes the attitude limit for this specific under-excited
scenario; it does not relax the deterministic or physical-test requirements.

## 2026-07-18 trusted heading and initial bias-observability evidence

The hardware-independent suite now contains two explicit scenarios that were previously only API
claims:

- `trusted_heading_recovery` disables magnetometer input, completes yaw alignment from a declared
  10 Hz heading source, rejects two injected 90° outliers, coasts through a 3.99 s outage, and
  resumes fusion. At seed 7 it accepts 148/148 normal updates, rejects 2/2 faults, limits outage yaw
  error to 0.187°, and reports 0.226° post-recovery yaw RMSE.
- `bias_convergence` performs one second of static cold start followed by 39 seconds of independent
  roll/pitch/yaw and three-axis specific-force excitation with continuous 10 Hz GNSS aiding. The
  injected accelerometer-bias error falls from 0.122 to 0.0349 m/s² and stays below 0.05 m/s² after
  29.4 s; final gyroscope-bias error is 0.000499 rad/s.

This work also corrected a covariance inconsistency: one stationary gravity direction cannot fully
separate tilt from horizontal accelerometer bias, so static alignment no longer marks accelerometer
bias with the same high confidence as directly observed gyro bias. Accelerometer-bias covariance is
kept broad for later motion/GNSS updates. These are deterministic synthetic results and do not
replace temperature, vibration, multi-orientation, or independent physical-truth testing.

Repeating the 20-seed outage Monte Carlo with that honest accelerometer-bias prior produced one
2.253° case under a trajectory with translation but no attitude excitation. The run stayed healthy,
required no navigation recovery, and retained consistent NIS/NEES; its accelerometer-bias error did
not become observable in 20 seconds. The scenario-specific bound is therefore 2.5° (observed maximum
2.253°, empirical P95 1.849°), while the separate multi-axis bias scenario must still meet 1.25° and
explicit bias-convergence gates. This separation prevents an unobservable one-pose case from being
misrepresented as either successful bias convergence or general flight accuracy.

## 2026-07-18 G0 independent-audit correction and confirmation

An independent post-closure review reopened four claims instead of accepting the earlier green
suite at face value:

- navigation recovery lacked physical-source/generation/quality-snapshot binding and explicit
  evidence-age/gap/dwell limits;
- horizontal position and velocity shared one validity timestamp;
- Q was only spot-checked and the heading model was described too loosely as a complete raw
  `atan2` Jacobian;
- seeds 0--999 had influenced the published thresholds and were not an unseen confirmation set.

The implementation now binds recovery to one source generation and one unchanged quality snapshot,
requires a count plus physical duration, rejects stale authorization, and keeps outputs invalid
through same-source probation. Position, velocity, and ZUPT refresh only their applicable validity
ages. The Q gate compares all 225 entries against a complete structure oracle, independently
integrates the declared reduced continuous model, and checks full-matrix finiteness, symmetry,
zero-noise behavior, and positive semidefiniteness. It does not yet validate every higher-order
within-step coupling implied by the complete transition model. Trusted heading now uses the complete
raw body-X heading Jacobian. Magnetometer yaw correction remains explicitly scoped as a tilt-
conditioned pseudo observation and is not represented as a general physical-heading NIS model.

The first unseen seed range, 10000--10999, was run with the prior unbounded Gaussian startup-bias
model and is retained as a failed confirmation. Seed 10988 drew `0.19925 m/s²` y-axis residual
accelerometer bias and reached `3.715°` attitude RMSE against the `3.5°` hard limit. The estimator
remained 100% healthy with reasonable NIS/NEES and zero recovery; the failure exposed the known
single-pose tilt/bias unobservability rather than numerical divergence. That seed range was not
reused as a blind pass.

The input contract was then made explicit: the current provisional FCOne pre-calibration profile
limits each startup residual-bias component to three standard deviations (`0.15 m/s²` accelerometer,
`0.6 deg/s` gyroscope). All development seeds affected by truncation were rerun before a new range
was opened. Seeds 20000--20999 then completed with zero execution, hard-envelope, distribution,
consistency, health, recovery, or 99% bootstrap failures. The exact 95% zero-failure probability
upper bound is `0.2991%`. The sanitized result is committed as
`validation/public/g0_monte_carlo_confirmation.json`.

After the independent audit changes were committed, a separate previously unused range,
40000--40999, was run from clean commit `5109568` with a protocol fingerprint covering the Git
state, runner, generator, analyzer, thresholds, gate policy, timing, noise, and bias profile. All
1,000 trials completed with zero execution, hard-envelope, distribution, consistency, recovery,
health, or 99% bootstrap failures. Position/velocity/attitude RMSE P95 were `0.6361 m`,
`0.2915 m/s`, and `1.7041°`; mean position NIS, velocity NIS, and six-state navigation NEES were
`2.9560`, `2.5318`, and `4.9758`. This confirms the calibrated navigation-outage regression but does
not close the independent VTOL bias-observability failure described below.

This calibrated-input bound is provisional, not a specification invented for the final hardware.
FCOne v2 multi-orientation and thermal characterization must measure it. If hardware exceeds it,
the profile fails and requires better calibration, a longer/multi-pose alignment procedure, or a
new validated estimator design; the host threshold is not silently widened.

### Independent VTOL bias cross-validation

The legacy `bias_convergence` multisine is no longer the only bias evidence. A frozen v1 protocol
defines five independent minimum-jerk VTOL trajectories, nine exact horizontal residual-bias
vectors, disjoint train/tune/holdout seeds, and 1,728 release trials. The protocol uses explicit
interval-start ZOH acceleration semantics, sensor-only cold start, per-trajectory/vector gates, and
right-censored non-convergence.

The first 15-trial smoke completed every execution but failed capability on all 10 boundary-bias
trials; all five zero-bias trials passed. Terminal horizontal-bias P95 ranged from `0.0595` to
`0.2154 m/s²` for the tested boundary cases, and every boundary case was right-censored without a
stable five-second pass. This is a genuine retained G0 failure, not a smoke-infrastructure failure.
It shows that the previous single-trajectory convergence result does not generalize yet. The
historical smoke unintentionally included seed `30000` from both v1 holdout trajectories, so v1 is
no longer eligible as blind final evidence. It remains useful diagnostic evidence only.

Bias reporting now exports each 3x3 bias covariance block and the complete 5x5 marginal covariance
for right-error tilt x/y plus accelerometer bias. It calculates their joint NEES with all cross
terms while excluding unobservable yaw, as well as per-axis error, terminal-five-second statistics,
continuous-five-second convergence, and explicit right-censoring. In the paired `+X/-Y` diagnostic,
the earlier accelerometer/gyro bias-only NEES means were `0.782 / 0.113`; a reproducible seed-7
review with explicit `+0.15/-0.15 m/s²` horizontal bias gives 5D joint NEES mean `0.768` versus the
expected `5`, with zero invalid covariance samples. The covariance is clearly conservative, yet
that fact does not by itself make the physical bias converge.

A source-fixed train/tune study at commit `5109568` completed 324/324 runs without further opening
v1 holdout trials. Broadening post-alignment accelerometer-bias covariance from `0.04` to `0.09`
increased passes from 23/108 to 33/108 and reduced right-censoring from 81/108 to 73/108, but all
64 non-zero train trials still failed and remained censored; terminal and attitude tails also
worsened. Raising `sigma_acc_bias` from `0.001` to `0.003 m/s3/sqrt(Hz)` added no material benefit.
Both temporary-wrapper candidates were rejected and were not implemented. G0 therefore remains
open on tilt--horizontal-bias observability. The runner now keeps smoke/train-tune away from
holdout and fails train-tune/release automation on capability failures even when the deprecated
`--report-only` flag is supplied. A v2 plan requires a new sealed holdout manifest in protected CI;
any final v1 release claim is prohibited. The compact candidate evidence is retained in
[`validation/public/g0_bias_candidate_study.json`](../validation/public/g0_bias_candidate_study.json).

The adapter now exposes one explicit `process_noise` profile and applies it atomically to the core.
Named VTOL/transition/fixed-wing profiles belong to the private FCOne product configuration and
must be linked to evidence; the public library does not silently auto-tune by vehicle or copy PX4
parameter values with different discrete-Q semantics.

## 2026-07-20 — G0 static-prior A/B decision

The latest frozen paired study is recorded in
[`validation/public/g0_static_prior_ab_study.json`](../validation/public/g0_static_prior_ab_study.json).
It executed `576` train/tune trials for the frozen baseline and `576` for the static-prior candidate:
`1,152/1,152` trials completed with zero execution failures. Baseline passed `175/576` and had
`390` right-censored trials; the candidate passed `231/576` and had `341` right-censored trials.

The candidate improved tune-only outcomes (`49` paired trials moved from right-censored to settled,
with zero baseline-pass-to-candidate-fail regressions) but did not generalize. Non-zero-bias train
passes remained `0/16` in both arms, while zero-bias train passes remained `16/16` in both arms.
The candidate was rejected and no static-prior or scalar process-noise change was promoted into the
estimator or FCOne product configuration.

This leaves accelerometer-bias observability as an active G0 blocker. The evidence points to
unresolved tilt--horizontal-bias coupling under the current excitation and covariance model; it does
not prove universal unobservability, justify widening gates, establish PX4 non-inferiority, or imply
FCOne v2 flight readiness. The next experiment must be a materially different, pre-registered
excitation-aware estimator hypothesis under a clean v2 protocol, not another scalar tuning sweep.

The A/B evidence is diagnostic only: the historical runs used a dirty tree and compact output that
predated per-trial input provenance, and the v1 smoke exposed seed `30000` from both holdout
trajectory families. No historical SHA values were fabricated. New compact campaigns now retain
input SHA-256, byte count, and data-row count before deleting CSVs and record protocol/marker
fingerprints. The public marker manifest remains `draft`; sealed holdouts without a public marker
continue to run metrics/gates and are classified as `unavailable_sealed_holdout` rather than as
execution failures or public information states.
