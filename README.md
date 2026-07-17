# Aerakia Flight Algorithms

[![CI](https://github.com/Sailiono/aerakia-flight-algorithms/actions/workflows/ci.yml/badge.svg)](https://github.com/Sailiono/aerakia-flight-algorithms/actions/workflows/ci.yml)

Portable C99 flight-estimation algorithms with a reproducible PC validation platform.

> 中文简介：这是 Aerakia 的公开算法层和验证工具，不包含飞控硬件、CubeMX、HAL/RTOS、传感器驱动、板级 HIL 协议或产品控制逻辑。

## What this repository demonstrates

- One hardware-neutral measurement contract shared by embedded targets and PC replay
- A paper-derived standard Mahony baseline and an Aerakia fault-tolerant configuration
- A 15-state error-state Kalman filter (ESKF) with position, barometer, magnetometer, and zero-velocity updates
- Deterministic scenario generation, native C replay, metric calculation, plots, and CI regression gates
- No heap allocation, operating-system calls, MCU headers, or device drivers in the algorithm library

## Algorithms

| Algorithm | Role | Notable behavior |
| --- | --- | --- |
| Mahony standard | Reference baseline | Full accelerometer and magnetometer correction |
| Mahony robust | Aerakia attitude estimator | Adaptive accelerometer trust, yaw-only magnetic correction, magnitude anomaly gate |
| 15-state ESKF | Aerakia navigation estimator | IMU propagation, bias states, covariance, aiding updates, innovation gating |

The Mahony implementation was written against the published nonlinear complementary-filter formulation rather than copied from the previous hardware project. The ESKF follows the quaternion/error-state conventions documented by Joan Solà. See [References](docs/references.md).

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
```

Sensor register access, axis remapping, calibration, and unit conversion stay in the private adapter. Algorithms validate the monotonic sample timestamp and derive `dt` internally. See [Integration guide](docs/integration.md).

The complete dependency direction is shown in [Architecture](docs/architecture.md).

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

Example results from the deterministic 8 s / 100 Hz / seed 7 suite:

| Scenario | Mahony standard | Mahony robust | ESKF |
| --- | ---: | ---: | ---: |
| Clean motion | 0.2090° | 0.2230° | 0.0368° |
| Magnetic spike | 0.3044° | 0.1862° | 0.1859° |
| Persistent magnetic bias | 14.0675° | 2.3891° | 0.0164° |

Values are wrapped attitude RMSE. They demonstrate deterministic behavior and fault response under the stated synthetic model; they do not prove flight safety or airworthiness. The [validation methodology](docs/validation.md) defines the evidence still required from public datasets, motion-capture/rate-table tests, HIL, and flight logs.

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
