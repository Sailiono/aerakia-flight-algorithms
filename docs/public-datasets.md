# Public dataset intake plan

No single public dataset covers all Aerakia validation needs. The evidence set is split by claim:
independent attitude/navigation truth, aggressive UAV motion, GNSS degradation, and PX4-format
compatibility. Dataset files remain outside this repository; only converters, manifests, checksums,
and derived reports belong in version control.

| Priority | Dataset | What it can prove | Important limitation | Intake unit |
| --- | --- | --- | --- | --- |
| P0 | [EuRoC MAV](https://www.research-collection.ethz.ch/entities/researchdata/bcaf173e-5dac-484b-bc37-faf97a594f1f) | IMU propagation and attitude/position accuracy against Vicon or Leica truth | No GNSS or magnetometer; Machine Hall orientation is IMU-aided | `MH_01_easy` and direct-pose `V1_03_difficult` complete |
| P0 | [Blackbird](https://github.com/mit-aera/Blackbird-Dataset) | Aggressive UAV dynamics against high-rate motion-capture truth | No GNSS or magnetometer; the full image dataset is multi-terabyte | Two sensor-only flight chunks at moderate and high speed |
| P0 | [RELLIS-3D](https://github.com/unmannedlab/RELLIS-3D) | Recorded VectorNav VN-300 dual-antenna GNSS/INS heading and IMU under outdoor off-road motion | Ground vehicle; VN-300 output is a sensor observation rather than independent heading truth | One ROS-bag sequence after topic, frame, timing, and license audit |
| P1 | [UrbanNav](https://github.com/IPNL-POLYU/UrbanNavDataset) | GNSS/IMU behavior in urban canyons and tunnels against SPAN-CPT truth | Ground vehicle rather than aircraft; download IMU, GNSS, and truth separately | Medium-urban and tunnel sensor subsets |
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
- UrbanNav supports navigation RMSE, NIS, outage/reacquisition behavior, and position/velocity NEES
  after time/frame alignment is independently checked.
- PX4 public ULogs support compatibility and fault discovery, not absolute accuracy claims.
- Absence of magnetometer or dual-antenna heading must remain explicit; course over ground is never
  relabeled as body heading.

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

## Next downloads

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

RELLIS-3D is the next heading-specific intake because its platform records a VectorNav VN-300
dual-antenna GNSS/INS. Blackbird sensor-only chunks remain desirable for aggressive aerial motion,
and UrbanNav sensor subsets remain the next navigation intake for recorded GNSS degradation and
outage behavior. None should be counted until raw topics, timing, frames, license, and hashes pass
the same intake checklist.
