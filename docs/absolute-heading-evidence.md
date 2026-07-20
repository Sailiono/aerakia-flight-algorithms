# Absolute-heading evidence

Absolute yaw is not one claim. Aerakia separates estimator-output accuracy, heading-sensor
accuracy, and heading-fusion behavior so that a source is never scored against itself.

## Evidence classes

| Class | Heading observation used by Aerakia | Scoring reference | Claim allowed |
| --- | --- | --- | --- |
| A | physical magnetometer or dual-antenna GNSS | independent motion capture, surveyed multi-antenna reference, or independently processed metrology-grade INS | end-to-end absolute-yaw accuracy |
| B | no recorded heading sensor | independent motion-capture pose | inertial propagation after a declared yaw initialization; not cold-start absolute heading |
| C | heading derived from independent pose truth | the same pose truth | heading API, geometry gate, dropout, outlier, and recovery behavior only |
| D | physical dual-antenna heading | orientation built from the same antenna baseline | physical-input compatibility and internal consistency only |
| E | GNSS course over ground or another estimator's yaw | none | diagnostic comparison only; never body-heading truth |

An acceptable Class A sequence must preserve the raw heading observation and a reference that does
not consume that observation. Shared clocks are acceptable and preferred; shared heading sensors
are not independent. The intake must record lever arms, antenna baseline, body/sensor extrinsics,
true-versus-magnetic North, magnetic declination, latency, validity flags, and uncertainty.

## Current evidence

| Dataset/track | Class | Current result | Boundary |
| --- | --- | --- | --- |
| EuRoC direct Vicon pose | B | independent full-pose truth supports attitude propagation scoring | no recorded magnetometer or physical heading observation |
| Blackbird motion capture | B | ESKF geodesic RMSE `1.573 deg`, yaw RMSE `1.069 deg` with reference attitude initialization | no recorded magnetometer, GNSS, or heading observation |
| EuRoC Vicon-derived heading fault track | C | observable yaw RMSE `0.920 deg`; outliers, outage, and recovery pass | heading input and scoring yaw come from the same Vicon pose |
| INSANE `indoor_1` raw PX4 magnetometer + OptiTrack | A candidate | frozen-calibration holdout: robust Mahony yaw RMSE/P95/max `7.599/11.243/11.835 deg`; ESKF `12.528/23.922/28.113 deg` | reference-attitude initialization only; no GNSS/position aiding, no surveyed North, and the yaw datum is frozen from a separate magnetic calibration split |
| INSANE `transition_1` cross-sequence transfer diagnostic | rejected A candidate | 38.131 s, 7,520 IMU and 3,358 physical-mag updates; the first physical field disagrees with the frozen declared datum by `49.298 deg` | angular-rate/gravity audits cannot observe a constant mocap-world yaw rotation; starts in motion and has no surveyed datum, so its `4.037 deg` robust-Mahony yaw RMSE is diagnostic and is not accepted as absolute-heading accuracy |
| INSANE `outdoor_1_sensors` | D | physical dual-RTK cold-start yaw RMSE `1.534 deg` after alignment | published yaw and direct heading share the same RTK baseline |
| PX4/IDF-DS ULogs | E | large-scale compatibility and fault coverage | `vehicle_attitude`, GSF yaw, and course over ground are not independent truth |

The repository now has one physical-magnetometer track scored against external OptiTrack attitude,
with the local yaw datum established from the separate calibration-window magnetometer. It remains
an adverse **A candidate**, not a completed true-North Class A qualification: the datum was not
independently surveyed, cold start is not covered, and the `transition_1` transfer attempt failed
because the mocap-world yaw datum was not portable. On the valid `indoor_1` holdout the ESKF is also
materially worse than Mahony.

The paired `indoor_1` A/B changes only `mag_valid` and `mag_update`. With magnetometer disabled,
ESKF geodesic/tilt/yaw RMSE is `0.718/0.666/0.270 deg`; with 8,776 physical updates enabled it is
`14.092/6.737/12.528 deg`. The ESKF innovation stage accepts all 8,776 updates and the outer field
gate passes 99.989%. This proves that the current magnitude gate and innovation test do not protect
this track; it does not yet distinguish field contamination, datum error, the 3D observation model,
or attitude/bias coupling as the root cause.

## Class A intake protocol

1. Freeze the estimator configuration before scoring.
2. Verify independent source lineage from dataset documentation and raw topics.
3. Estimate clock offset and rigid extrinsics on a calibration split only.
4. Exclude heading geometry singularities, invalid dual fixes, low baseline quality, and magnetic
   saturation using predeclared source-quality rules.
5. Run cold start, normal tracking, natural dropout, injected outlier, and recovery tracks.
6. Report geodesic attitude error and wrapped yaw error only where body forward has a sufficiently
   large horizontal projection.
7. Report accuracy by stationary, hover, transition, fixed-wing straight, turn, climb, and descent
   regimes where the dataset supports them.
8. Keep calibration/tuning sequences separate from the final holdout sequence.

Minimum report fields are yaw RMSE/P95/maximum, convergence time, outage drift, outlier false-
acceptance rate, valid-sample coverage, recovery time, innovation NIS, source-quality rejection
counts, frame/time audit residuals, dataset hashes, and the exact code commit.

## Evidence still needed

The preferred next dataset is a recorded magnetometer or dual-antenna GNSS heading stream on a
platform with an independent six-degree-of-freedom truth system. A ground or handheld platform is
acceptable for validating the generic heading model; a UAV/VTOL sequence is still required before
flight-regime qualification. If no public Class A UAV sequence passes the source-lineage audit, the
FCOne data-collection plan must include an independently tracked yaw reference rather than treating
PX4 output as truth.

### Next audited intake order

1. INSANE `indoor_1` and `transition_1` are converted and retained. `indoor_1` uses disjoint
   calibration/development/holdout windows. `transition_1` was opened only after calibration was
   frozen, but failed the cross-sequence yaw-datum transfer and is diagnostic only. The first
   tracking implementation also incorrectly derived the magnetic reference from the holdout's first
   truth attitude and magnetometer sample. That self-calibration was removed; final scoring uses
   only the declared datum. The remaining work is an independently surveyed true-North datum,
   causal static cold start, natural/injected magnetic faults, and broader UAV regimes. Dataset
   terms add non-commercial use and no-patent-license restrictions.
2. UrbanNav HK already provides independent SPAN-CPT+IE roll/pitch/azimuth truth against the
   low-cost Xsens/F9P path. The recorded Xsens quaternion can be evaluated as a physical trusted-
   heading input against SPAN, but that would validate Aerakia's generic heading-aiding path rather
   than its raw magnetometer model. The platform is a ground vehicle and the source page has no
   explicit redistributable license, so raw data remains local research input.
3. MILUV is a later indoor multi-UAV Vicon/bias diversity source. Only selected experiments should
   be downloaded from the approximately 173 GB corpus.

NTU VIRAL is not an absolute-yaw source: its Leica channel supplies position and the official
dataset explicitly does not provide orientation ground truth. It remains useful for other aerial
motion claims but cannot close this gate.
