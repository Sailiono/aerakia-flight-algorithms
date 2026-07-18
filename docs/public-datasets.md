# Public dataset intake plan

No single public dataset covers all Aerakia validation needs. The evidence set is split by claim:
independent attitude/navigation truth, aggressive UAV motion, GNSS degradation, and PX4-format
compatibility. Dataset files remain outside this repository; only converters, manifests, checksums,
and derived reports belong in version control.

| Priority | Dataset | What it can prove | Important limitation | Intake unit |
| --- | --- | --- | --- | --- |
| P0 | [EuRoC MAV](https://www.research-collection.ethz.ch/entities/researchdata/bcaf173e-5dac-484b-bc37-faf97a594f1f) | IMU propagation and attitude/position accuracy against Vicon or Leica truth | No GNSS or magnetometer; Machine Hall orientation is IMU-aided | `MH_01_easy` and direct-pose `V1_03_difficult` complete |
| P0 | [INSANE](https://www.aau.at/en/smart-systems-technologies/control-of-networked-systems/datasets/insane-dataset/) | Recorded UAV IMU plus physical 1.16 m dual-RTK body-heading observations | Published yaw and direct heading share the RTK baseline; published tilt currently fails gravity consistency | `outdoor_1_sensors` yaw-path intake complete |
| P0 | [Blackbird](https://github.com/mit-aera/Blackbird-Dataset) | Aggressive UAV dynamics against motion-capture truth | No GNSS or magnetometer; the reviewed redistributed sequence reaches about 3 m/s rather than the full corpus maximum | MathWorks `NYC Subway Winter` sensor/pose package complete |
| P1 | [RELLIS-3D](https://github.com/unmannedlab/RELLIS-3D) | Recorded VectorNav VN-300 fused attitude and IMU under outdoor off-road motion | Ground vehicle; public topics do not expose raw dual-baseline validity and the reviewed bag lacks a readable index | Retain as a secondary device-output audit, not heading truth |
| P1 | [UrbanNav](https://github.com/IPNL-POLYU/UrbanNavDataset) | GNSS/IMU behavior in urban canyons and tunnels against SPAN-CPT truth | Ground vehicle; selected F9P NMEA contains positions but no receiver velocity or heading | `UrbanNav-HK-Medium-Urban-1` complete; tunnel remains optional diversity |
| P1 | [GVINS](https://github.com/HKUST-Aerial-Robotics/GVINS) | Raw multi-constellation GNSS, IMU, and intermittent-GNSS comparison | ROS bag and ENU/ECEF conventions need an explicit adapter | Sports-field bag after license and checksum review |
| P1 | [PX4 Flight Review v2](https://github.com/PX4/flight-review-rs) | Real ULog schema coverage, estimator resets, and unusual sensor combinations | Public PX4 estimates are not independent truth; publication terms and privacy must be checked | Metadata-screened logs only; never bulk-commit raw logs |

## Acceptance checklist

Before a sequence is scored:

1. Record the source URL, dataset version, sequence name, download date, license or terms, and
   SHA-256 checksum.
2. Keep raw files immutable and outside Git.
3. Declare source frames, body axes, timestamp units, sensor-to-body extrinsics, gravity sign, and
   truth semantics before conversion to Aerakia NED/FRD.
4. Verify monotonic timestamps, coverage overlap, sample rates, gaps, quaternion norm, and transform
   round trips.
5. Export only required IMU, aiding, and truth channels into the hardware-neutral replay contract.
6. Freeze parameters before scoring and retain failed runs in the report.

## Scoring policy

- EuRoC and Blackbird support independent-truth RMSE and NEES for the observable state subset.
- UrbanNav supports recorded-position RMSE/NIS, outage drift/reacquisition, attitude error, and
  navigation NEES after time/frame alignment is independently checked. Its selected receiver stream
  does not support measured-velocity claims.
- PX4 public ULogs support compatibility and fault discovery, not absolute accuracy claims.
- Absence of magnetometer or dual-antenna heading must remain explicit; course over ground is never
  relabeled as body heading.

## Completed physical-heading intake: INSANE `outdoor_1_sensors`

The official sensor-only archive and calibration package were downloaded without the image data.
Raw files remain outside Git; the converter and the reviewed metric summary are committed. The
package contains two RTK-fixed position streams, PX4 IMU/magnetometer data, a declared IMU/truth
time offset, and a 1.16 m calibrated antenna baseline.

| Item | Recorded value |
| --- | --- |
| Sensor ZIP SHA-256 | `02ea94047ccb7d887c34f90f0c868f8430bdc448883bf82aeadd2a616d79bb79` |
| Calibration ZIP SHA-256 | `cff4fbd099051cf0ff29838f7ab4bc67ff35d70cc6555029e537ca1036c215f2` |
| Dataset tools commit | `9a1c8c0fdd195f2d869fff292f2ce5b273c5a03d` |
| Retained replay | 39,174 recorded IMU samples, 199.735 s |
| Physical heading input | 1,378 synchronized RTK-fixed baseline observations |
| Baseline geometry | median 1.1591 m, standard deviation 0.0393 m after the geometry gate |
| Frames | source ENU/FLU converted to Aerakia NED/FRD |
| Evidence class | shared-sensor physical reference, not independent truth |

```bash
python simulation/tools/convert_insane_to_replay.py outdoor_1_sensors \
  --out replay.csv --metadata source.json

build/aerakia_validation_runner --cold-start replay.csv results.csv
python validation/analyze_results.py results.csv \
  --out-dir report --scenario insane-outdoor-1-dual-rtk-cold-start \
  --reference-kind shared_sensor_reference
```

With reference attitude initialization, ESKF yaw RMSE is 2.018 degrees, normal heading acceptance
is 97.10%, and estimator health is 100%. In the independent cold-start track, heading alignment
completes at 11.492 s; post-alignment yaw RMSE is 1.534 degrees, normal heading acceptance is
91.15%, the reported recovery interval is 2.000 s, and health remains 100%. There are no declared
fault observations in this natural sequence, so it does not replace the existing outlier gate.

The dataset's published orientation is constructed from the dual-RTK baseline and calibrated
magnetometer. Direct baseline heading and scored yaw therefore share a physical source. In addition,
the published initial roll/pitch disagrees with the measured gravity direction by roughly 18–20
degrees under replay. Until that frame/export discrepancy is resolved, only the yaw input path is
accepted from this sequence; full-attitude metrics are deliberately excluded from capability claims.

## Completed independent-motion intake: Blackbird `NYC Subway Winter`

The official Blackbird download host did not complete either HTTPS or HTTP sensor-file requests on
2026-07-18. Rather than bypassing source validation, the intake uses MathWorks' documented
`BlackbirdVIOData.tar` redistribution of the Blackbird `NYC Subway Winter` sequence. The package is
MIT licensed and contains recorded IMU, motion-capture pose, image timestamps, and images in one
MAT file; only IMU, timestamps, and pose are read. The raw TAR and MAT stay outside Git.

| Item | Recorded value |
| --- | --- |
| MathWorks TAR SHA-256 | `5a225275874bf4d129a7c9649524e9baf43d401bb2ad452f85709f514a39c0ec` |
| `data.mat` SHA-256 | `58b852a823d13afa0bcc5064c28b9de599a4adc5e191e4979bd10fef66b154b9` |
| Included license SHA-256 | `c9fe8f7fb586becf1335fa515f173bfda1803db4a49826f03aba09f5443208cf` |
| Official Blackbird tools commit | `8f02a207a85bea415e32375b67b56bf8b0e1c02d` |
| Retained overlap | 26,995 recorded IMU samples, 270.076 s, 99.978 Hz |
| Independent truth | 19.9999 Hz motion-capture body pose |
| Motion | speed P95/max 2.80/3.03 m/s; gyro norm P95/max 1.95/4.22 rad/s; acceleration norm P99/max 11.43/59.78 m/s² |
| Aiding | deterministic synthetic 10 Hz GNSS from motion-capture position and differentiated velocity |

The first frame audit incorrectly zeroed the IMU and motion-capture timestamps independently. That
discarded the real `-1.762237184 s` motion-capture-start offset and made x/y angular rates appear
uncorrelated. The converter now retains one common absolute time base and applies the published
body-to-IMU quaternion
`[0.70747936, 0.00202983, -0.00774523, 0.70668865]` (wxyz). It refuses conversion unless the
motion-capture-derived and recorded angular rates correlate by at least 0.95 on all axes and the
frame/gravity residual stays bounded. The accepted axis correlations are
`0.99345 / 0.98947 / 0.99984`; total angular-rate RMSE is `0.03627 rad/s` and total specific-force
RMSE is `0.36386 m/s²`.

| Track | Mahony robust geodesic / tilt RMSE | ESKF geodesic / tilt / yaw RMSE | Position / velocity RMSE | Position / velocity NIS mean | 6D nav NEES mean |
| --- | ---: | ---: | ---: | ---: | ---: |
| Raw IMU, reference attitude init | 51.478° / 7.814° | 1.573° / 1.131° / 1.069° | 0.123 m / 0.082 m/s | 3.057 / 2.568 | 4.763 |
| Static-reference bias diagnostic | 10.129° / 7.744° | 1.573° / 1.131° / 1.069° | 0.123 m / 0.082 m/s | 3.057 / 2.568 | 4.763 |
| Raw IMU, independent tilt cold start | not scored as product fallback | post-alignment tilt 1.131° | 0.123 m / 0.082 m/s | 3.057 / 2.568 | 4.762 |

The cold start completes tilt alignment in `1.009 s` and correctly leaves heading alignment false
because there is no magnetometer or trusted-heading observation. ESKF remains healthy for 100% of
the replay with zero navigation recoveries. Synthetic GNSS means the navigation result validates
fusion and covariance consistency, not a physical receiver.

The retained Mahony failure is operationally important. With no magnetic/heading reference it is
not an indefinite backup: raw yaw drifts severely and aggressive translational acceleration also
degrades tilt. The public supervisor oracle now requires Mahony to be continuous with the last
qualified output before fallback, invalidates all navigation fields, and applies a finite degraded
time budget. With a 10° admissible entry error, the worst one-second Blackbird truth envelope reaches
14.959°; the older 15°/five-second example reached 20.307°. The oracle therefore uses a conservative
10°/one-second host-test policy. It is not a flight-qualified FCOne constant.

```bash
python simulation/tools/convert_blackbird_to_replay.py BlackbirdVIOData \
  --out replay.csv --metadata source.json \
  --synthetic-gnss-rate-hz 10 --seed 7 --static-hint-duration-s 5

python validation/run_public_dataset_suite.py \
  --data-root /path/to/blackbird-mathworks \
  --manifest validation/public/blackbird_manifest.json \
  --runner build/aerakia_validation_runner \
  --out-dir build/blackbird-suite
```

The three declared tracks replay 80,985 samples but contain 26,995 unique physical IMU samples.
Together with EuRoC and the completed UrbanNav intake, the external-reference corpus now contains
398,493 unique recorded IMU samples and 865,845 replay attempts. INSANE adds 39,174 shared-source
physical-heading samples but is kept in a separate evidence class.

## RELLIS-3D audit result

The official full-stack download is a 4.0 GiB ZIP (SHA-256
`95c0eacef45b28c832ec7a3bdb60042891230ea98c81f1a052e50b00bbb941da`) containing an 8.89 GB
`example_filtered.bag`. The ZIP passes its integrity test, but the ROS bag lacks a readable index for
the standard `rosbags` reader. A raw topic scan confirms `/vectornav/IMU`, `/vectornav/Odom`,
`/vectornav/GPS`, `/vectornav/Mag`, `/vectornav/Pres`, and `/vectornav/Temp`.

The reviewed VectorNav ROS driver publishes the unit's fused quaternion but does not preserve a raw
dual-baseline validity/status stream in these messages. RELLIS is therefore retained as possible
fused-device compatibility evidence after reindexing, not as an independent dual-GNSS heading truth
source. INSANE is the higher-priority physical-heading track.

## Completed EuRoC intake: `MH_01_easy`

The official Machine Hall bundle was range-downloaded from the ETH Research Collection. The
12.7 GB outer bundle was not duplicated; the nested `MH_01_easy.zip` archive is retained outside
Git together with the minimal IMU and reference CSVs. Temporary range-download partials were
deleted after the nested ZIP passed an integrity check.

| Item | Recorded value |
| --- | --- |
| DOI | `10.3929/ethz-b-000690084` |
| Source license | In Copyright - Non-Commercial Use Permitted |
| Nested ZIP SHA-256 | `5f4ecbc563e7bf04950efab76fc3b14e126d4c91a1dd1134c33124fe18a4bfc6` |
| IMU CSV SHA-256 | `226470999dc8ba758c838982fd7eb2bc9f3f2b489e85a38d0ecf8b9d40785a5b` |
| Reference CSV SHA-256 | `aae2d7c5684724c8afe364d2fd8499a6ed77a9169adf03736531f67d8e7b2117` |
| Scored overlap | 36,381 samples, 181.9 s, nominal 200 Hz |
| Frame mapping | EuRoC FLU/+Z-up to Aerakia FRD/NED: `diag(1,-1,-1)`; quaternion `[w,x,-y,-z]` |
| Aiding used | Deterministic synthetic GNSS at 10 Hz, 0.5 m / 0.1 m/s standard deviations, seed 7 |

The converter refuses non-identity IMU/body extrinsics until the general transform has its own
round-trip tests. It checks timestamp order, quaternion norms, stream overlap, and the expected
gravity direction before writing replay data.

```bash
python simulation/tools/convert_euroc_to_replay.py MH_01_easy \
  --out replay.csv --metadata source.json \
  --synthetic-gnss-rate-hz 10 --seed 7

# Diagnostic math-validation track; not an online-estimation result.
python simulation/tools/convert_euroc_to_replay.py MH_01_easy \
  --out replay_bias_corrected.csv --metadata source_bias_corrected.json \
  --synthetic-gnss-rate-hz 10 --seed 7 --apply-reference-bias
```

### Current result

Both tracks use the recorded first attitude as a trusted initial condition. `MH_01_easy` starts in
motion, and there is no magnetometer or other absolute heading source.

| Track | Mahony robust geodesic attitude / Euler-yaw RMSE | ESKF geodesic attitude / Euler-yaw RMSE | Position / velocity RMSE | Position / velocity NIS mean | 6D nav NEES mean |
| --- | ---: | ---: | ---: | ---: | ---: |
| Raw IMU | 105.520° / 107.077° | 8.804° / 8.146° | 0.131 m / 0.086 m/s | 3.080 / 2.585 | 5.294 |
| Reference-bias corrected | 2.983° / 3.596° | 3.254° / 3.280° | 0.131 m / 0.082 m/s | 3.078 / 2.537 | 5.192 |

The raw Mahony yaw failure is retained: EuRoC's median z-gyro bias is about 0.0784 rad/s, so yaw is
not observable without magnetometer/heading aiding and drifts through wrap boundaries. Subtracting
the batch reference bias demonstrates propagation behavior but does not prove online bias
observability. Machine Hall position is externally constrained by Leica, while orientation is an
IMU-aided batch estimate; the attitude column is therefore an external reference with shared-sensor
limitations, not fully independent motion-capture truth.

The 6D NEES result exposed and then verified a process-noise bug: continuous noise densities had
been discretized with `dt²` instead of `dt`. After correcting gyro/accelerometer terms to `dt` and
adding integrated velocity-position covariance, EuRoC NEES moved from 14.34/12.60 to 5.29/5.19
against an expected mean of 6. The deterministic synthetic suite remains within reviewed bounds.

## Completed recorded-GNSS intake: UrbanNav HK `Medium-Urban-1`

The official UrbanNav repository and Dropbox sensor folders supplied a minimal intake containing
only the selected Xsens IMU log, u-blox F9P NMEA, and SPAN-CPT postprocessed truth. Raw downloads
remain under ignored `build/` storage. The converter rejects bad NMEA checksums and invalid fixes,
uses same-epoch GST uncertainty, converts WGS84 coordinates to local NED, and never manufactures a
receiver velocity from differentiated positions.

| Item | Recorded value |
| --- | --- |
| Official tools commit | `075f96b6a6d9252b37486ecb175b4ae690c56f54` |
| Full IMU ZIP SHA-256 | `0114fb60053fff6cb7ae16a68c09fbd7840cf83557af4ffa127bcbbe8aff4a28` |
| GNSS ZIP SHA-256 | `b265d9983533b2c038ba2ec46c90a4c078bef5119fef128e6b843af3a1bc5fe2` |
| SPAN truth SHA-256 | `9d48bb497878aafbd17290789560394c72ecafec20c9e0eaff448418695d92bf` |
| Selected IMU SHA-256 | `5c329bcafd781e9d00aca41b42d7e5822a8861f3b9d439666ea005f8613e802d` |
| Selected NMEA SHA-256 | `ff2f53a63ddc13d81bebbeba4808f468bddf07ce6941ec2ed11ec2876c812876` |
| Replay coverage | 314,185 unique IMU samples, 785.451 s, about 400.33 Hz |
| Receiver aiding | 655 valid position epochs; 37 bad/non-NMEA fragments rejected |
| Recorded outage | one 131 s gap, equivalent to about 130 missing nominal 1 Hz epochs |
| Motion envelope | speed P95/max 10.20/11.50 m/s; gyro norm P95/max 0.225/0.675 rad/s |

The official source body axes are right/forward/up. Conversion to Aerakia FRD uses
`[[0,1,0],[1,0,0],[0,0,-1]]`; source ENU navigation is converted to NED. Before scoring, the
converter differentiates SPAN position and compares it with the separately published SPAN body
velocity. North/east correlations are `0.99992/0.99989` and horizontal velocity RMSE is
`0.0745 m/s`. This executable audit prevents a plausible but wrong axis or sign mapping from being
accepted.

Two retained tracks intentionally answer different questions:

| Track | Attitude geodesic / tilt / yaw RMSE | Nominal aided position RMSE | 131 s outage peak | First resumed posterior error |
| --- | ---: | ---: | ---: | ---: |
| Reference-attitude initialization | 1.534° / 0.772° / 1.328° | 6.518 m | 1471.5 m | 5.395 m |
| Independent tilt-only cold start | 36.722° / 2.211° / 36.647° | 6.794 m | 1283.5 m | 5.374 m |

The whole-run position RMSE of about 248 m in the reference-initialized track is not nominal GNSS
accuracy: it is dominated by the real 131 s unaided interval. The report therefore scores aided,
unaided, pre-gap, peak, first-resumed, and sustained-recovery intervals separately. Reacquisition
returns below 10 m on the first accepted position update without a forced navigation reset, while
the position-only inertial drift during the outage remains intentionally visible.

Raw F9P horizontal error against SPAN is 2.890 m RMSE and 5.036 m P95. Aerakia's 6.518 m aided
position RMSE is therefore not an accuracy improvement over the receiver on this track. It is a
stress result for a position-only estimator path with no measured velocity, no magnetometer, no
delay compensation, and a scalar worst-axis GST variance. Position NIS means `0.011/0.339` are
strongly conservative, while six-state navigation NEES means `12.91/16.27` are overconfident,
primarily exposing the unaided velocity subspace. These consistency failures are recorded work,
not tuned away on the evaluation sequence.

The cold-start result is equally important: tilt completes in about one second, but heading cannot
complete because this sequence has neither magnetometer nor a physical heading observation. The
36.65° yaw RMSE must not be cited as general ESKF yaw accuracy; it demonstrates the expected
unobservability and the need for a valid heading source or a separately validated motion-based yaw
observer.

```bash
python validation/run_public_dataset_suite.py \
  --data-root /path/to/urbannav-minimal-inputs \
  --manifest validation/public/urbannav_manifest.json \
  --runner build/aerakia_validation_runner \
  --out-dir build/urbannav-suite
```

The next complementary navigation source should contain recorded receiver Doppler velocity and
independent truth, preferably on an aircraft. A second UrbanNav tunnel sequence increases urban
environment diversity but repeats the same missing-velocity and ground-vehicle limitations; it is
therefore useful after, not instead of, that evidence.

## Completed EuRoC direct-Vicon intake: `V1_03_difficult`

Only the 801 MB nested sequence ZIP was range-downloaded from the 6.04 GB Vicon Room 1 bundle; the
ROS bag and outer archive were not stored. The direct `vicon0` pose uses the published non-identity
tracking-body extrinsic. The converter applies `T_RB = T_RS * inverse(T_BS)` and has a synthetic
non-identity-extrinsic regression test.

| Item | Recorded value |
| --- | --- |
| Nested ZIP SHA-256 | `3a29beda3c9467dcab08bfa181ed924c3397c2bdb3a780baaef6eb12955d82a4` |
| IMU CSV SHA-256 | `543835f7b7dd9d1287c9052955e030ade113c51dc97ec2c15d13a318521ad0d7` |
| Direct Vicon CSV SHA-256 | `766cd2841d5f2e4106c5b7e00b804cb6705f59b054625cbd74dd1b40e2c0c3fa` |
| Batch reference CSV SHA-256 | `703ba51a854daba80f20df93e76605c2278b3f227afe2bcbe9eb4255186fc3d9` |
| Scored overlap | 20,932 samples, 104.655 s, nominal 200 Hz |
| Pose time alignment | subtract 40,000 µs from logged Vicon timestamps |
| Velocity reference | EuRoC batch velocity; raw `vicon0` contains pose only |

The fixed 40 ms latency was estimated once by aligning transformed raw Vicon pose to EuRoC's
published batch trajectory; it is recorded as a conversion parameter and is never re-fitted during
scoring. At zero offset the raw/batch median attitude difference was about 1.4°; the fixed alignment
reduces the high-dynamic timing mismatch to the sub-degree range.

```bash
python simulation/tools/convert_euroc_to_replay.py V1_03_difficult \
  --pose-source vicon --pose-time-offset-us 40000 \
  --out replay.csv --metadata source.json \
  --synthetic-gnss-rate-hz 10 --seed 7
```

| Track | Mahony robust geodesic / tilt RMSE | ESKF geodesic / tilt RMSE | Position / velocity RMSE | Position / velocity NIS mean | 6D nav NEES mean |
| --- | ---: | ---: | ---: | ---: | ---: |
| Raw IMU | 95.508° / 3.360° | 4.102° / 1.047° | 0.136 m / 0.091 m/s | 3.066 / 2.506 | 5.702 |
| Reference-bias corrected | 2.875° / 2.845° | 2.474° / 0.829° | 0.137 m / 0.086 m/s | 3.065 / 2.427 | 5.628 |

A separate raw-IMU cold-start run marks only the first 1.2 s as stationary from external Vicon
evidence and raises the validation runner's pre-alignment gyro threshold to 0.15 rad/s to admit the
observed 0.08 rad/s startup bias. Tilt alignment completes at 1.0 s; post-alignment tilt RMSE is
0.862°, navigation NEES is 5.621, and covariance remains healthy. The earlier 1.230° result was
superseded by the reviewed absolute-attitude covariance reset added after static alignment; the
one-command suite retains this change as an explicit baseline update. Heading alignment correctly
does not complete in the sensor-free track because the dataset contains no magnetometer or recorded
trusted-heading observation.

A separate evidence track derives a 10 Hz heading observation from direct Vicon yaw, adds 1°
deterministic noise, a declared four-second outage, and two 90° outliers, then cold-starts without a
reference quaternion. It excludes scalar heading whenever the body-forward horizontal projection
falls below 0.25; V1_03 repeatedly approaches vertical and contributes 4,582 such samples. Alignment
completes at 1.0 s, post-alignment geodesic attitude RMSE is 1.498°, geometry-observable yaw RMSE is
1.130°, both outliers are rejected, observable-outage maximum yaw error is 3.943°, and recovery is
0.995 s with 1.224° post-recovery yaw RMSE. This validates the production heading path under real
motion, but the observation is derived from the same Vicon truth and is not physical sensor accuracy.

This sequence repeatedly approaches ±90° pitch. The older combined Euler metric reported false
14.5°/4.3° errors because equivalent roll/yaw representations jump by 180° at the singularity.
The retained per-axis plot exposed that validation bug; primary scoring now uses quaternion
geodesic and gravity-direction errors. Euler-yaw remains diagnostic only on this sequence.

Direct Vicon supplies independent pose, while velocity and optional bias correction still come
from EuRoC's batch estimate. Consequently the attitude/position evidence is stronger than the
velocity-bias evidence, and the corrected track still does not prove online bias observability.

Run all six reviewed tracks from restored minimal inputs with:

```bash
python validation/run_public_dataset_suite.py \
  --data-root /path/to/euroc-minimal-inputs \
  --runner build/aerakia_validation_runner \
  --out-dir build/public-dataset-suite
```

The suite verifies every retained input hash before conversion, logs all 18 subprocesses, compares
selected metrics against the committed baselines, and writes an environment/commit manifest plus an
aggregate report. The two sequences contain 57,313 unique recorded IMU samples; six declared tracks
produce 156,490 replay attempts. Multiple tracks increase algorithm-path coverage but are not counted
as additional physical data.

UrbanNav now closes recorded GNSS-position degradation and outage behavior. Recorded receiver
Doppler velocity with independent truth remains the next navigation evidence gap. A second
physical-heading source is still desirable, but it must expose either raw
antenna-baseline validity or an independently synchronized yaw reference. None is counted until raw
topics, timing, frames, license, and hashes pass the same intake checklist.
