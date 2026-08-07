# Aerakia Algorithm Optimization Showcase

This collection summarizes the July 17-21, 2026 Aerakia optimization campaign. It is organized around comparisons that preserve the same dataset, reference truth, aiding setup, and initialization family.

## What improved

| Dataset and condition | ESKF attitude RMSE before | ESKF attitude RMSE after | Reduction | Position RMSE before -> after |
| --- | ---: | ---: | ---: | ---: |
| EuRoC MH_01_easy, raw IMU seeded | 8.80 deg | 3.25 deg | 63.04% | 0.131 m -> 0.131 m |
| EuRoC V1_03_difficult, raw IMU seeded | 4.04 deg | 2.32 deg | 42.66% | 0.139 m -> 0.136 m |
| EuRoC V1_03_difficult, Vicon body-pose seeded | 4.10 deg | 2.47 deg | 39.69% | 0.136 m -> 0.137 m |

Attitude RMSE is the quaternion geodesic error in degrees. Lower is better. Position values are shown as a guardrail rather than a primary optimization target; they remain stable across the paired runs.

## Optimization Timeline

| Date | Campaign step | Evidence |
| --- | --- | --- |
| Jul 17 | Established independent EuRoC attitude and navigation validation. | MH_01_easy baseline: 3.72 deg attitude RMSE, 0.224 m position RMSE. |
| Jul 17 | Added Vicon tilt-only cold-start coverage. | V1_03 post-alignment tilt RMSE: 0.860 deg; position RMSE: 0.123 m. |
| Jul 18 | Validated trusted-heading recovery and bias-corrected propagation. | Observable yaw RMSE: 0.920 deg; recovery: 0.995 s. |
| Jul 18 | Expanded to aggressive UAV, GNSS-outage, and physical-flight conditions. | Blackbird: 1.57 deg attitude RMSE; INSANE cold start: 1.53 deg yaw RMSE; UrbanNav health ratio: 1.0. |
| Jul 20 | Added magnetic-source supervision evidence. | Negative indoor result was retained; no unsupported runtime change was promoted. |
| Jul 21 | Rejected a correlated static-prior candidate. | Production default and runtime code remained unchanged. |

## Image provenance

The slide deck embeds the original comparison figures. The separately rendered
PNG files and their temporary `cloud-restored-*` source paths are intentionally
not retained here. Recreate them with the public dataset suite when the
EuRoC inputs are restored; the metric values below remain the reviewed,
machine-readable record.

## Condition Coverage

The following records demonstrate breadth, not a shared absolute-accuracy ranking:

| Condition | Evidence boundary | Result |
| --- | --- | --- |
| Blackbird aggressive UAV motion | Independent motion-capture truth; synthetic GNSS | 1.57 deg attitude RMSE; 1.0 healthy ratio |
| INSANE outdoor dual-RTK heading cold start | Shared sensor/reference yaw evidence | 1.53 deg post-alignment yaw RMSE; 11.49 s alignment |
| UrbanNav Hong Kong 131 s GNSS outage | Independent SPAN-CPT navigation reference | 1.0 healthy ratio; 6.52 m aided position RMSE |
| IDF-DS fixed-wing maximum angular motion | PX4 compatibility and stress coverage | 3.24 deg attitude RMSE; 34 navigation recoveries |

Read `showcase-data.json` for the complete, machine-readable metric snapshot and image index.
