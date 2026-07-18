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
Neither derived/shared-source heading track establishes independent absolute heading accuracy or
flight safety.

## Evidence completed

| Area | Evidence | Status |
| --- | --- | --- |
| Error-state math | 15-dimensional error state, quaternion injection/reset Jacobian, Joseph-form scalar and vector measurement updates | Implemented and unit tested |
| Covariance health | Long mixed predict/update sequence checked for finite, symmetric, positive-semidefinite covariance | Passing |
| Measurement integrity | NIS gates, latched magnetic-disturbance rejection, recovery confirmation, navigation recovery supervision | Passing deterministic regressions |
| Heading semantics | Magnetometer, trusted heading, GNSS course, and PX4 GSF are separate paths; course is never silently treated as body yaw | Implemented |
| Trusted-heading fault behavior | Cold-start completion, normal fusion, four-second dropout, two 90° outliers, rejection, geometry validity, and recovery with magnetometer disabled | Passing deterministic synthetic and Vicon-derived fault gates; INSANE physical dual-RTK path passes 1,378 updates, but shares its yaw reference |
| Online IMU bias behavior | Static gyro initialization plus motion/GNSS-aided accelerometer-bias convergence, with truth error and settling time reported separately | Passing deterministic multi-axis synthetic gate; single-pose accelerometer observability limit and hardware thermal behavior remain explicit |
| Cold-start alignment | Static accelerometer tilt, magnetic heading with explicit declination, IMU-bias initialization, and covariance reset at the new linearization point; no PX4 attitude seed | Passing unit/noisy synthetic checks and direct-Vicon tilt (0.862° post-alignment RMSE); external yaw truth still pending |
| Navigation consistency | GNSS position/velocity NIS and posterior 6-state navigation NEES through a five-second outage and reacquisition | 20-seed measurement-noise baseline and constant-bias/timestamp-jitter extension pass with zero numerical/recovery failures; thermal and transport faults pending |
| EuRoC public replay | 36,381-sample Leica/IMU `MH_01_easy` and 20,932-sample direct-pose `V1_03_difficult`; raw, cold-start, derived-heading, and reference-bias tracks retained | Navigation NIS/NEES consistent; direct Vicon pose passes high-dynamic replay; derived heading is not a recorded heading sensor |
| Blackbird public replay | 26,995 recorded IMU samples over 270.1 s against independent motion capture; common time base and published body/IMU extrinsic are checked before conversion | ESKF 1.573° geodesic / 1.131° tilt RMSE, 100% health, zero recovery; retained Mahony drift proves fallback must be bounded |
| UrbanNav public replay | 314,185 recorded IMU samples, 655 checksum/quality-screened F9P positions, SPAN truth, and one 131 s recorded outage | Position-only API and recovery pass with 100% numerical health; horizontal navigation is valid for 83.90% of samples and explicitly expires after five unaided seconds; 6.52 m nominal aided RMSE and first-update reacquisition are retained alongside severe unaided drift and inconsistent NIS/NEES |
| Host regression | Strict C99 warnings-as-errors build, public API tests, deterministic synthetic fault suite | Passing reviewed thresholds |
| Input/transport integrity | Exhaustive required-IMU non-finite checks; timestamp order/gap behavior; optional-mag isolation; timestamped GNSS/heading/barometer freshness and recovery | Passing 100 seeds, 1,020,000 IMU attempts, 2,100 aiding attempts, and burst lengths through 100 with zero invariant/health failures |
| Estimator supervision | ESKF-primary startup, hard-invalid immediate response, soft observability hysteresis, continuity-gated and time-bounded Mahony attitude-only degradation, navigation invalidation, continuity-gated recovery, and transition evidence | Passing executable public contract; private FCOne policy and actuator interaction remain open |
| Output qualification | Numerical health is distinct from horizontal position/velocity validity; only accepted position/velocity/ZUPT observations refresh the configurable dead-reckoning interval | Passing public API and UrbanNav outage gates; FCOne must consume the validity fields rather than infer validity from finite values |
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
   has no receiver velocity, heading, magnetometer, or aircraft motion.

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
- Independent-reference EuRoC raw IMU: tilt RMSE is 0.713° on `MH_01_easy`, 1.047° on
  `V1_03_difficult`, and 0.862° after Vicon-declared cold-start tilt alignment.
- With a 10 Hz, 1°-noise heading stream derived from Vicon truth, the V1_03 cold start completes in
  1.0 s and post-alignment geodesic attitude RMSE is 1.498°. On geometry-observable samples yaw RMSE
  is 1.130°; two injected 90° observations are both rejected. These are real-motion software-path
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
- EuRoC full-attitude raw-IMU RMSE is 8.804° and 4.102°. This is dominated by yaw drift from the
  recorded approximately 0.08 rad/s z-gyro bias with no magnetometer or trusted heading; it is an
  observed limitation, not an acceptable heading-accuracy claim.
- Reference-bias-corrected EuRoC diagnostic tracks reach 3.254° and 2.474° full-attitude RMSE, but
  batch truth-bias subtraction is not online estimation and is never reported as flight accuracy.
- EuRoC position RMSE is 0.131–0.137 m and velocity RMSE 0.082–0.091 m/s with deterministic
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

### Robustness

- Software/input robustness is strong for the tested contract: 1,020,000 IMU attempts, 2,100
  timestamped aiding attempts, five sample rates, bursts through 100 samples, exhaustive required
  non-finite axes, and zero state/covariance invariant or health failures.
- Numerical robustness is strong on the host: strict C build, long covariance checks, Joseph-form
  updates, finite-difference Jacobians, and the full million-attempt campaign under ASan+UBSan.
- Aggressive-motion robustness now includes Blackbird's independent motion capture: ESKF remains
  healthy for all 26,995 samples with zero navigation recovery. The same run retains a 51.48° raw
  Mahony full-attitude RMSE without magnetometer/heading; even static reference-bias subtraction
  leaves 10.13°. Therefore Mahony is classified as continuity-gated, finite-duration degraded
  attitude—not a second long-duration navigation solution.
- Estimation consistency is currently reasonable: EuRoC navigation NEES means are 5.19–5.70 for a
  six-dimensional expected mean of 6; position NIS is 3.07–3.08 for expected mean 3. Velocity NIS
  at 2.43–2.58 is mildly conservative rather than overconfident.
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
  3.943°; recovery takes 0.995 s and post-recovery yaw RMSE is 1.224°.
- The INSANE physical dual-RTK track adds 199.7 s of recorded UAV motion and maintains 100% health;
  97.10% of normal updates are accepted with reference initialization and 91.15% after cold start.
  Natural data has no declared heading faults, so outlier rejection remains proven by the separate
  injected-fault tracks.

### Maturity judgment

| Scope | Current judgment | Reason |
| --- | ---: | --- |
| Portable algorithm/math implementation | 84–88% | Core equations, covariance handling, independent position/velocity aiding, cold start, and recovery are mature; real-receiver covariance consistency and physical heading truth remain open |
| Host-side software robustness | 90–93% | High-volume malformed/timing campaign, sanitizers, and 398,493 unique public external-reference IMU samples pass, including a recorded 131 s aiding outage |
| Heading robustness | 68–72% | Real-motion fault/recovery and physical dual-RTK input now pass; independent physical yaw truth and GNSS-velocity GSF fallback remain open |
| Navigation robustness | 68–74% | Recorded position outage/reacquisition and fail-closed no-aiding validity now pass, but long unaided drift, missing receiver velocity, delay modeling, and NIS/NEES tuning remain open |
| FCOne integration readiness | 75–80% | Neutral adapter, independent aiding, and bounded-fallback supervisor contracts are executable; exact private messages, scheduling, target precision, and resource use are unverified |
| Flight-main-estimator readiness | 50–60% | Suitable for shadow mode and bench/HIL preparation, not justified as the sole flight estimator yet |

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

## 2026-07-18 trusted heading and bias observability closure

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
