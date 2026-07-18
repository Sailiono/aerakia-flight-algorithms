# Same-input PX4/Aerakia accelerometer-bias A/B protocol

This protocol answers a narrow question before broader estimator parity claims: when FCOne/eVTOL
hover-first motion excites horizontal accelerometer bias weakly, do Aerakia and a pinned PX4 EKF2
build converge under the same physical inputs, and do their covariances remain honest?

## Frozen PX4 reference

The first implementation target is PX4 commit
`de8158101c96ad6b04170dc91f087148104c58eb`. The runner must set module defaults explicitly.
Instantiating the core EKF class alone is not equivalent to the standard module configuration.

| Quantity | PX4 module value for the frozen comparison |
| --- | ---: |
| initial accelerometer-bias standard deviation | `0.2 m/s²` |
| accelerometer-bias process parameter | `0.003 m/s³` |
| accelerometer-bias state limit | `0.4 m/s²` |
| bias-learning acceleration limit | `25 m/s²` |
| bias-learning angular-rate limit | `3 rad/s` |
| learning-inhibit decay time | `0.5 s` |
| IMU control mask | `7` |

Primary source pointers are PX4's
[`params_accel_bias.yaml`](https://github.com/PX4/PX4-Autopilot/blob/de8158101c96ad6b04170dc91f087148104c58eb/src/modules/ekf2/params_accel_bias.yaml),
[`module.yaml`](https://github.com/PX4/PX4-Autopilot/blob/de8158101c96ad6b04170dc91f087148104c58eb/src/modules/ekf2/module.yaml), and
[`covariance.cpp`](https://github.com/PX4/PX4-Autopilot/blob/de8158101c96ad6b04170dc91f087148104c58eb/src/modules/ekf2/EKF/covariance.cpp).

PX4 and Aerakia do not assign identical discrete meaning to a numerically equal bias process-noise
parameter. Aerakia uses a continuous density with `Q_bias = sigma² dt`; PX4's frozen implementation
uses a per-step term proportional to `(dt * parameter)²`. Values therefore must not be copied by
name alone.

## Two comparison tracks

1. **Stock profile:** the frozen PX4 module profile versus the frozen Aerakia VTOL-hover profile.
   This measures the behavior an integrator actually receives.
2. **Matched discrete Q:** match initial bias covariance and each step's discrete bias process
   variance. This isolates model/update behavior from parameter-semantics differences.

Both tracks record every parameter, source commit, runner binary hash, input hash, threshold hash,
and protocol hash. A result without those facts is diagnostic only.

## Identical input contract

Both estimators consume the same calibrated FRD delta-angle/delta-velocity or equivalently audited
interval samples, integration durations, per-axis clipping, GNSS position and Doppler velocity,
barometer, magnetic observations, measurement delays, `at_rest`, `in_air`, and transition mode.
Core filter states are compared at the common delayed fusion horizon; PX4 output prediction is
reported separately rather than mixed into core-filter accuracy.

## Hover-first trajectory

The first vehicle target is FCOne eVTOL hover/takeoff/landing:

1. 10 s ground static initialization;
2. 20 s low-excitation hover, reported as an observability diagnostic rather than a guaranteed
   horizontal-bias convergence window;
3. 40--60 s bounded roll/pitch, north/east translation, climb/descent, and yaw excitation;
4. 20 s post-excitation hover to test retention;
5. separate clipping, high-rate, aiding-outage, and IMU-generation-change fault tracks.

The deterministic boundary starts with zero bias, all six signed single axes, all signed pairs, and
all six-dimensional corners. The known `+X/-Y`, mirrored `-X/+Y`, and zero-bias cases retain paired
noise seeds. Each track runs at least 60 s even though the product convergence budget remains 35 s;
late convergence must not disappear through truncation.

## Metrics and interpretation

- three-axis and vector bias error, terminal P50/P95/P99, and right-censored settling time;
- bias NEES and truth coverage by covariance;
- false-confidence cases where covariance is small while truth error exceeds the limit;
- attitude, velocity, position, NIS, and navigation NEES cost;
- learning-inhibit fraction, clipping response, recovery time, and erroneous bias jump;
- every metric grouped by trajectory and exact bias vector, never only pooled.

The current absolute Aerakia gates remain `0.05 m/s²` bias-vector error and a 35 s convergence
budget. They are not changed after observing the `+X/-Y` failure.

Interpret results as follows:

- PX4 passes and Aerakia fails: investigate Aerakia model, covariance, update, or initialization.
- both fail in hover but pass after reviewed excitation: the dominant boundary is observability.
- both converge after 35 s: the mathematical state may be stable, but the product budget is unmet.
- Aerakia converges faster while NEES or attitude degrades: do not count it as superior.

This A/B does not replace independent truth. It establishes engineering non-inferiority only for
the declared vehicle profile and input contract.

## M0 executable skeleton

M0 freezes the comparison before either exporter is accepted:

- `validation/px4_bias_ab_m0_manifest.json` freezes the PX4 commit, audited source-file hashes,
  stock configurations, time semantics, and absolute convergence gates;
- `validation/px4_bias_ab_schema_v1.json` defines one dense physical-input CSV, one estimator-output
  CSV, and the mandatory provenance sidecars;
- `validation/run_px4_bias_ab_m0.py` verifies the local PX4 Git tree and every artifact hash, then
  compares only outputs aligned at the same delayed fusion horizon;
- `tests/test_px4_bias_ab_m0.py` locks the fail-closed behavior and the first comparison metrics.

The runner has no clone, fetch, download, or fallback-estimator path. A normal invocation is:

```bash
python3 validation/run_px4_bias_ab_m0.py \
  --px4-source /absolute/path/to/PX4-Autopilot \
  --input-csv /absolute/path/to/input.csv \
  --input-metadata /absolute/path/to/input.metadata.json \
  --aerakia-output /absolute/path/to/aerakia.csv \
  --aerakia-metadata /absolute/path/to/aerakia.metadata.json \
  --px4-output /absolute/path/to/px4.csv \
  --px4-metadata /absolute/path/to/px4.metadata.json \
  --out-dir build/px4-bias-ab-m0
```

It always writes `run-manifest.json`. Exit status `0` means the supplied frozen artifacts were
compared, `1` means a supplied source or artifact violated the protocol, and `2` means a required
local dependency or exporter artifact is absent. Only status `completed` may contain comparison
metrics. Even then, the manifest keeps `claim_status: no_px4_parity_claim` because M0 is a narrow
accelerometer-bias diagnostic, not flight-readiness evidence.

Bias validity is part of the score, not decorative metadata. Three-sigma coverage and bias NEES
use only samples with `accel_bias_valid=1` and strictly positive reported variances. A continuous
settling interval fails if any sample marks the bias invalid, even when the numerical bias value is
inside the error threshold.

At this milestone the canonical Aerakia and PX4 exporters are still an explicit blocked dependency.
Until both produce the declared CSV and provenance sidecar from the same input, no numerical PX4
result is reported.
