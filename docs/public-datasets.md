# Public dataset intake plan

No single public dataset covers all Aerakia validation needs. The evidence set is split by claim:
independent attitude/navigation truth, aggressive UAV motion, GNSS degradation, and PX4-format
compatibility. Dataset files remain outside this repository; only converters, manifests, checksums,
and derived reports belong in version control.

| Priority | Dataset | What it can prove | Important limitation | Intake unit |
| --- | --- | --- | --- | --- |
| P0 | [EuRoC MAV](https://www.research-collection.ethz.ch/entities/researchdata/bcaf173e-5dac-484b-bc37-faf97a594f1f) | IMU propagation and attitude/position accuracy against Vicon or Leica truth | No GNSS or magnetometer; Machine Hall orientation is IMU-aided | `MH_01_easy` complete; add one difficult Vicon sequence |
| P0 | [Blackbird](https://github.com/mit-aera/Blackbird-Dataset) | Aggressive UAV dynamics against high-rate motion-capture truth | No GNSS or magnetometer; the full image dataset is multi-terabyte | Two sensor-only flight chunks at moderate and high speed |
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

| Track | Mahony robust attitude / yaw RMSE | ESKF attitude / yaw RMSE | Position / velocity RMSE | Position / velocity NIS mean | 6D nav NEES mean |
| --- | ---: | ---: | ---: | ---: | ---: |
| Raw IMU | 61.890° / 107.077° | 4.821° / 8.146° | 0.131 m / 0.086 m/s | 3.080 / 2.585 | 5.294 |
| Reference-bias corrected | 2.585° / 3.596° | 1.974° / 3.280° | 0.131 m / 0.082 m/s | 3.078 / 2.537 | 5.192 |

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

Next add one EuRoC Vicon-room difficult sequence for genuinely independent 6D motion-capture truth.
Blackbird sensor-only chunks remain desirable for aggressive motion, but its official download
endpoint was unavailable during this intake. UrbanNav sensor subsets follow after the second EuRoC
sequence so navigation/outage comparisons use the same audited conversion contract.
