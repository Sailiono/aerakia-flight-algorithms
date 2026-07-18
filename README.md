# Aerakia Flight Algorithms

[![CI](https://github.com/Sailiono/aerakia-flight-algorithms/actions/workflows/ci.yml/badge.svg)](https://github.com/Sailiono/aerakia-flight-algorithms/actions/workflows/ci.yml)

Portable C99 flight-estimation algorithms with a reproducible PC validation platform.

> 中文简介：这是 Aerakia 的公开算法层和验证工具，不包含飞控硬件、CubeMX、HAL/RTOS、传感器驱动、板级 HIL 协议或产品控制逻辑。

## What this repository demonstrates

- One hardware-neutral measurement contract shared by embedded targets and PC replay
- A paper-derived standard Mahony baseline and an Aerakia fault-tolerant configuration
- A 15-dimensional error-state Kalman filter (16-component nominal state) with paired or independent GNSS position/velocity, barometer, heading, magnetometer, and zero-velocity updates
- Independent static cold-start tilt and magnetic-heading alignment with an explicit true-North declination reference
- Deterministic scenario generation, native C replay, metric calculation, plots, and CI regression gates
- GNSS NIS and independent-truth navigation NEES diagnostics, including outage/reacquisition replay
- Separate numerical-health and horizontal-navigation-validity outputs with a configurable no-aiding timeout
- No heap allocation, operating-system calls, MCU headers, or device drivers in the algorithm library

## Algorithms

| Algorithm | Role | Notable behavior |
| --- | --- | --- |
| Mahony standard | Reference baseline | Full accelerometer and magnetometer correction |
| Mahony robust | Short-duration degraded-attitude fallback / cross-monitor | Adaptive accelerometer trust, yaw-only magnetic correction, magnitude anomaly gate |
| 15-error-state ESKF | Aerakia navigation estimator | IMU propagation, bias states, covariance reset Jacobian, aiding gates, navigation recovery |

The Mahony implementation was written against the published nonlinear complementary-filter formulation rather than copied from the previous hardware project. The ESKF follows the quaternion/error-state conventions documented by Joan Solà. See [References](docs/references.md).

In the intended FCOne architecture, the ESKF is the primary navigation estimator and robust
Mahony is an independent, continuity-gated, time-bounded attitude-only fallback/cross-monitor.
Mahony does not replace ESKF
position, velocity, covariance, or aiding integrity. Standard Mahony remains a validation baseline.
The application-level transition contract is documented in
[Estimator supervision](docs/estimator-supervision.md).

## Stable driver-to-algorithm boundary

The private driver layer publishes calibrated samples in FRD body axes and explicit units. Both estimators consume the same structure:

```c
#include <aerakia/eskf_adapter.h>
#include <aerakia/mahony.h>

AerakiaImuSample sample = {
    .timestamp_us = sensor_timestamp_us,
    .acceleration_m_s2 = {ax, ay, az},
    .angular_rate_rad_s = {gx, gy, gz},
    .magnetic_field_ut = {mx, my, mz},
    .flags = AERAKIA_SAMPLE_ACCEL_VALID
           | AERAKIA_SAMPLE_GYRO_VALID
           | AERAKIA_SAMPLE_MAG_VALID,
};

AerakiaAttitudeEstimate attitude;
aerakia_mahony_update(&mahony, &sample, &attitude);

AerakiaNavigationEstimate navigation;
aerakia_eskf_process_imu(&eskf, &sample, &navigation);

/* `healthy` is numerical integrity; use this qualification before publishing position/velocity. */
if (!navigation.horizontal_navigation_valid) {
    invalidate_horizontal_navigation_output();
}

/* Lower-rate aiding stays hardware-neutral too. */
AerakiaPositionObservation position = {
    gps_sample_time_us, gps_position_ned, gps_position_variance
};
aerakia_eskf_update_position_observation(&eskf, &position);

/* Publish velocity separately only when the receiver actually provides it. */
AerakiaVelocityObservation velocity = {
    gps_sample_time_us, gps_velocity_ned, gps_velocity_variance
};
aerakia_eskf_update_velocity_observation(&eskf, &velocity);
```

Sensor register access, axis remapping, calibration, and unit conversion stay in the private adapter. Algorithms validate the monotonic sample timestamp and derive `dt` internally. See [Integration guide](docs/integration.md).

The complete dependency direction is shown in [Architecture](docs/architecture.md).
The mathematical and product-level comparison with PX4 EKF2, including the IMU-only boundary and
the same-input A/B gate, is documented in [PX4 EKF2 comparison](docs/px4-ekf2-comparison.md).
The scoped meaning of "PX4-class", truth/benchmark separation, anti-overfitting protocol, and
G0--G4 acceptance gates are defined in the
[PX4-class validation plan](docs/px4-class-validation-plan.md).

## Build and test

Requirements: CMake 3.16+, a C99 compiler, and Python 3.10+ for validation.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
```

## Run the PC validation platform

```bash
python -m venv .venv
source .venv/bin/activate              # Windows: .venv\Scripts\activate
python -m pip install -r requirements.txt

python validation/run_suite.py \
  --runner build/aerakia_validation_runner \
  --out-dir build/validation
```

The suite generates timestamped IMU/magnetic data with ground truth, replays the exact native C code, produces per-axis errors and plots, and checks reviewed regression limits.

Example results from the deterministic 20 s / 100 Hz / seed 7 suite:

| Scenario | Mahony standard | Mahony robust | ESKF |
| --- | ---: | ---: | ---: |
| Clean motion | 0.1476° | 0.1547° | 0.3515° |
| Magnetic spike | 0.2612° | 0.1540° | 0.3741° |
| Persistent magnetic bias | 27.1650° | 0.2854° | 0.3271° |
| Tilted cold start (post-alignment) | — | — | 0.0369° |
| GNSS outage/reacquisition (post-alignment) | — | — | 0.6821° |
| Trusted heading dropout/recovery (post-alignment) | — | — | 0.2755° |
| Multi-axis bias convergence (post-alignment) | — | — | 0.8863° |

Attitude values use quaternion geodesic RMSE, which is singularity-free and is not numerically
comparable to the older RMS of three Euler components. In the outage scenario, position RMSE is 0.497 m and velocity
RMSE is 0.226 m/s; NIS and NEES are checked against reviewed deterministic bounds. These synthetic
results demonstrate repeatability and fault response, not flight safety or airworthiness. The
[algorithm status](docs/algorithm-status.md) records the current maturity and P0 blockers, while the
[validation methodology](docs/validation.md) defines the evidence still required from public
datasets, motion-capture/rate-table tests, HIL, and flight logs.

For the ordered pre-hardware work, acceptance evidence, FCOne v2 integration boundary, and data
still worth collecting, see the [development roadmap](docs/roadmap.md). The complete current host
gate can be reproduced and recorded with:

```bash
python validation/run_host_regression.py
```

Private PX4 ULogs can be normalized without exporting absolute coordinates or hardware identifiers:

```bash
python simulation/tools/convert_ulog_to_replay.py flight.ulg \
  --out build/flight/replay.csv --metadata build/flight/source.json

python validation/run_ulog_suite.py private_manifest.json \
  --runner build/aerakia_validation_runner --out-dir build/ulog-validation
```

Raw ULogs and private manifests stay outside Git. Only the converter, replay contract, example manifest, and analysis code are public.

ULog reports keep three yaw views separate: raw agreement with PX4, agreement after applying
the logged PX4 reset deltas, and per-reset-segment drift. Direct dual-antenna GNSS heading is
fused only when PX4 marks it finite; ordinary GNSS course and PX4's GSF yaw remain diagnostics.

The reviewed public evidence combines EuRoC and Blackbird motion-capture/reference tracks,
UrbanNav recorded Xsens/F9P data with a 131 s outage, a 9.92-hour IDF-DS PX4 compatibility audit,
and an aerial DJI-GPS/RTK-reference replay. Independent truth, shared-source reference, and PX4
estimate results remain separate. Converters, source hashes, frame/time audits, results, and
explicit limitations are documented in [Public datasets](docs/public-datasets.md). Raw archives
remain outside Git.

After restoring the minimal EuRoC inputs, reproduce all five reviewed tracks with:

```bash
python validation/run_public_dataset_suite.py \
  --data-root /path/to/euroc-minimal-inputs \
  --runner build/aerakia_validation_runner
```

The command checks immutable input hashes, runs conversion/native replay/scoring, retains one log
per subprocess, and fails when a selected metric drifts from the committed baseline.

## Repository layout

```text
include/aerakia/    Public measurement and algorithm APIs
src/                Portable C99 implementations
simulation/tools/   Dataset generation, capture conversion, and analysis
validation/         Native replay runner, reports, and regression thresholds
tests/              Host-side unit and interface tests
docs/               Contracts, methodology, and references
```

## Public boundary

This repository intentionally excludes board support, generated configuration, vendor middleware, private transport protocols, and aircraft/product logic. See [Public repository boundary](docs/public-boundary.md).

## License

Licensed under the [Apache License 2.0](LICENSE).
