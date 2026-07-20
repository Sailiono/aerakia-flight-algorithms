# PX4-class validation plan for the FCOne target envelope

This plan turns the long-term goal of reaching PX4-class estimation capability into testable
claims. It does not use PX4 output as truth, equate one benchmark score with product maturity, or
claim parity across every vehicle and sensor supported by PX4.

## Scoped goal

The first parity target is the FCOne v2 operating envelope:

- small multirotor and fixed-wing aircraft;
- local NED navigation with calibrated FRD IMU at 100--400 Hz;
- physical magnetometer, barometer, GNSS position and GNSS Doppler velocity;
- optional trusted heading from dual-antenna GNSS or another independently qualified source;
- bounded aiding interruptions, explicit invalidation after the allowed dead-reckoning interval,
  and controlled reacquisition;
- ESKF as the primary navigation estimator and robust Mahony as a finite-duration attitude-only
  fallback/cross-monitor.

Long-duration IMU-only horizontal navigation, global ECEF navigation, optical flow, VIO, terrain,
and full PX4 feature parity are not part of the first claim. They can become later capability
increments only after their sensors, use cases, and acceptance evidence are defined.

Within the eVTOL family, hover/VTOL, transition, and fixed-wing cruise are separate validation and
configuration profiles. The public state model and source remain common; profile differences must
be justified by sensor/vibration/dynamics evidence and frozen before their validation set. See
[FCOne/eVTOL target profiles and repository flow](vehicle-target-and-repository-flow.md).

## Why two comparison tracks are mandatory

| Track | Question answered | Required reference | What it cannot prove |
| --- | --- | --- | --- |
| Independent-truth validation | Is Aerakia physically accurate, consistent, and robust? | Motion capture, laser tracker, surveyed total station, or another independently audited chain | Relative parity with PX4 and missing PX4 product features |
| Same-input PX4 A/B | Is Aerakia non-inferior to a mature benchmark inside the declared sensor/task envelope? | Both estimators scored against the same independent truth | Absolute correctness if both estimators share an error or the reference is not independent |
| FCOne hardware/HIL/flight | Does the complete self-developed hardware/software product work in real time and remain safe? | Board measurements, HIL, shadow logs, bounded flight tests | Generalization beyond the tested hardware and flight envelope |

The tracks are complementary. PX4 ULog outputs and IDF-DS are engineering references, not truth.
Conversely, a truth dataset without a PX4 run cannot establish relative parity.

## Anti-overfitting and parameter policy

ESKF tuning is system identification and uncertainty modeling, not unrestricted truth fitting.
Datasets are divided by whole flight, never by randomly splitting samples from one flight:

1. **Calibration:** time offsets, sensor extrinsics, scale, Allan/noise parameters, and receiver
   variance interpretation. Calibration outputs and uncertainty are recorded.
2. **Development:** one global parameter set may be tuned against declared objectives.
3. **Validation:** verifies the frozen parameter set and can reject it, but does not retune it.
4. **Locked blind test:** opened only after code, parameters, thresholds, and comparison protocol
   are frozen.

Reference-derived batch bias correction, synthetic GNSS generated from truth, or a truth-derived
heading may test a software path, but is never reported as physical sensor accuracy. A different
parameter set per log is forbidden for parity claims.

## Evidence grades

| Grade | Meaning | Current examples |
| --- | --- | --- |
| A | Independent physical truth with audited timing/frames | Blackbird motion capture, EuRoC direct Vicon |
| B | External reference with possible source/environment coupling | UrbanNav SPAN, electrical-survey RTK |
| C | Shared physical source between input and scoring reference | INSANE baseline heading, derived Vicon heading |
| D | Mature estimator or onboard engineering reference | private PX4 ULogs, IDF-DS PX4 attitude/local position |
| E | Synthetic truth | deterministic scenarios and Monte Carlo |

Coverage volume and truth grade are reported separately. Millions of grade-D samples improve
compatibility and stress evidence but do not become grade-A accuracy evidence.

## G0 — close mathematical and safety blockers

These gates precede claims based on additional datasets or PX4 ratios:

1. Restore executable finite-difference validation for prediction, an explicitly scoped
   process-noise oracle, and every declared nonlinear measurement geometry. A projected
   pseudo-measurement must be named
   and tested as such rather than presented as the full raw measurement Jacobian. Run across
   ordinary and near-singular geometries, and make unobservable geometry an explicit result.
2. Replace the generic `gate²` interpretation with measurement-dimension-aware chi-square/NIS
   thresholds. A one-dimensional 3-sigma gate and a three-dimensional 99.73% gate do not share the
   same NIS limit.
3. Remove unconditional trusted re-anchoring after a count of rejected observations. Recovery must
   require audited source quality, multi-sample consistency, bounded correction, explicit
   supervisor authorization, and a probationary validity state.
4. Add continuous heading-aiding age/validity and vertical-position/velocity validity. A completed
   startup heading alignment does not prove that yaw remains observable.
5. Separate public-dataset reproducibility baselines from one-way capability gates. A stable but
   poor result must remain reproducible while still failing capability acceptance.
6. Add repository-level warnings-as-errors and sanitizer CI rather than relying only on local
   invocation.
7. Verify accuracy invariance on the same physical trajectory at 100/200/400/1000 Hz; current
   multi-rate input-integrity testing does not prove estimator accuracy invariance.

Initial G0 scale is 10,000 randomized derivative points and at least 1,000 independent Monte Carlo
trials for consistency intervals. Exact numerical derivative tolerances are frozen only after the
finite-difference step size and conditioning audit are documented.

Current G0 evidence includes a 225-element structure oracle/PSD test plus independent numerical
integration of the declared reduced continuous Q model and a bounded full frozen-coefficient Qd
oracle, source/generation/quality-
snapshot and time-window-bound recovery, independent horizontal position/velocity validity, and a
new 1,000-seed calibrated-input confirmation with 10,000-resample 99% bootstrap bounds. The prior
unbounded-bias confirmation is retained with its one hard failure. G0 therefore applies only to the
declared calibrated-input host profile; FCOne hardware must verify the provisional residual-bias
envelope before the result can become a target profile.

## G1 — independent-truth capability

The initial evidence target is at least two independent sources, ten complete trajectories, and
60 minutes of grade-A physical attitude truth spanning static, nominal maneuver, high angular rate,
high specific force, near-vertical geometry, and declared sensor faults.

Provisional engineering gates, to be frozen before the blind set is opened:

- tilt RMSE at or below 1.5 degrees;
- yaw RMSE at or below 2.5 degrees only where independent physical yaw is observable;
- attitude P95 at or below 3 degrees and yaw P95 at or below 5 degrees;
- no forced navigation recovery on nominal-quality data;
- position/velocity recovery within two valid observation periods, with a correction bounded by
  both declared observation uncertainty and controller continuity requirements;
- physical GNSS evaluation must include receiver position, receiver Doppler velocity, and an
  independent navigation reference. Missing receiver fields are not synthesized.

These are project gates, not airworthiness standards. Control-system error budgets may require
tighter limits and take precedence.

## G2 — same-input PX4 non-inferiority

The first narrow implementation is the
[accelerometer-bias A/B protocol](px4-bias-ab-protocol.md), which fixes the PX4 source revision,
module defaults, identical-input contract, stock and matched-Q tracks, VTOL hover-first motion, and
per-direction scoring before a broader feature comparison is attempted.

PX4 is pinned to official commit
`de8158101c96ad6b04170dc91f087148104c58eb`. The unversioned PX4 snapshot under FCOne v1 may help
prototype interfaces but is not an accepted benchmark; the old `ekf2_lite.cpp` remains a stub.

The first runner links the official host `ecl_EKF` library directly rather than starting the uORB
module. It records the PX4 SHA, configuration, runner patch hash, compiler, input hash, event counts,
and output-time semantics. No PX4 estimator source file may be modified.

The M0 transport baseline has completed one 65 s / 100 Hz fixed-bias synthetic event stream. It
generated 6,489 exact delayed-horizon pairs with 100% numerical health in both estimators, while
both missed the protocol's 35 s absolute bias-convergence gate. This establishes the executable
same-input/export path only; it does not satisfy the full G2 conditions below and cannot be used as
a PX4 superiority, parity, or non-inferiority result.

Two comparisons remain separate:

- **Common-sensor math/product core:** both receive only IMU, GNSS position/velocity, heading-only
  magnetometer, barometer, and valid GNSS heading available in the common contract.
- **PX4 full available system:** PX4 may use its additional declared sources and fault machinery.
  This measures the feature gap and is not combined with common-sensor accuracy ratios.

Initialization tracks include production cold start, common warm-up, and a same-cut propagation
track. Delay tracks include zero delay, recorded delay, and 0/20/40/80/120/200 ms scans. PX4
delayed-horizon state and latest output-predictor state are scored against truth at their own
physical timestamps.

The M0 harness is accepted only when:

- both estimators consume exactly the same immutable event sequence and `dt` convention;
- no truth field reaches either filter;
- static, constant-rate, gravity sign, unit, and frame sentinels pass;
- event counts and timestamps match exactly;
- repeated runs are deterministic;
- 5/10/30/60/120 s aiding cuts are generated from one manifest;
- outputs include attitude, velocity, position, bias, validity, resets, CPU, and state memory;
- one command generates `comparison.json`, logs, and `report.md`.

After a pilot run establishes reference uncertainty and repeatability, a non-inferiority margin is
predeclared before blind evaluation. A provisional starting rule is a median Aerakia/PX4 RMSE ratio
at or below 1.10, 95% bootstrap upper bound at or below 1.20, P95-error ratio at or below 1.25, and
no locked sequence over 1.5 times PX4. Every track must also pass the absolute G1 gate; beating a bad
PX4 result is not sufficient.

## G3 — FCOne target and HIL

- Pin the reviewed algorithm commit in the private FCOne v2 integration.
- Run Aerakia and the existing estimator in shadow mode before any control-path authority.
- Measure execution time, WCET, stack high-water mark, Flash/RAM, deadline misses, and precision on
  the actual target. Host timing and `sizeof` do not replace target evidence.
- Complete multi-orientation, temperature, vibration, power-supply, installation, and motor-current
  magnetic tests on at least three boards.
- Complete a 24-hour HIL run with zero NaN, unexpected reset, deadline miss, or silent-validity
  failure before bounded flight tests.

Stronger hardware can reduce noise and provide more compute headroom, but it cannot repair wrong
frames, bad timestamps, missing observability, unsafe reset policy, or overfitting. Hardware quality
and estimator correctness are separate acceptance axes.

## G4 — bounded flight expansion

Multirotor and fixed-wing gates are maintained separately. Each platform first completes at least
ten representative shadow flights across multiple locations and known GNSS/magnetic stress before
limited estimator authority. Comparison with PX4's deployment maturity ultimately requires tens to
hundreds of accumulated hours, not one successful flight or one dataset score.

## Dataset acquisition order

| Priority | Dataset | Immediate purpose | Boundary |
| --- | --- | --- | --- |
| P0 | [INSANE indoor/transition](https://www.aau.at/en/smart-systems-technologies/control-of-networked-systems/datasets/insane-dataset/) | Independent OptiTrack 6DoF/yaw, cold start, magnetic/GNSS transition using the existing INSANE converter | Outdoor published attitude can share magnetometer/RTK sources; license wording needs clarification before commercial reuse claims |
| P0 | [RTK-SLAM](https://www.isprs.org/resources/datasets/benchmarks/RTK-SLAM/Default.aspx) | Deliberate long GNSS degradation and surveyed Leica checkpoints | Ground/handheld; sparse position checkpoints, no continuous yaw or physical Doppler velocity |
| P1 | [MILUV](https://decargroup.github.io/miluv/) | Independent Vicon yaw, three UAVs, bias/Allan and multi-agent motion | Indoor and no GNSS; select experiments rather than downloading the 173 GB corpus |
| P1 | [MUN-FRL](https://mun-frl-vil-dataset.readthedocs.io/en/latest/) | Aircraft/rotorcraft vibration and long-flight position reference | No independent yaw; position reference is RTK/PPK |
| P2 | [MARS-LVIG](https://mars.hku.hk/dataset.html) | Large-scale UAV raw GNSS/IMU and flight dynamics | Reference construction may share GNSS/IMU; non-commercial/share-alike license |

No reviewed public dataset currently closes all requirements simultaneously. Physical receiver
Doppler velocity plus independent aerial truth and an independent heading source may still require
a controlled FCOne/PX4 collection campaign.

## Immediate execution order

1. Implement and review G0 dimension-aware gates, supervised recovery, continuous validity, and
   executable Jacobian/capability tests.
2. In parallel, scaffold PX4 M0 against the official pinned `ecl_EKF` and intake INSANE
   `indoor_1` plus `transition_1`.
3. Run M0 on synthetic outage, then UrbanNav, EuRoC/Blackbird, IDF, delay scans, and physical
   heading tracks in that order.
4. Intake RTK-SLAM after the small INSANE tracks; keep its sparse checkpoint claim separate.
5. Freeze development parameters and thresholds, then open the locked blind set.
6. Only after G0--G2 pass, apply the exact private FCOne v2 message adapter and proceed to G3.

Subagents may work in parallel on isolated scopes: mathematical/consistency tests, dataset
independence/conversion, and PX4 runner/resource work. The primary agent owns shared thresholds,
code integration, total regression, Git history, and the final evidence log.
