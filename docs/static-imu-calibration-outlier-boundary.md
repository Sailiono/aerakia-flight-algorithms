# Static IMU calibration outlier boundary

The companion scan in `validation/scan_static_imu_calibration_outlier_boundary.py`
characterizes the frozen default six-pose calculator without changing any
threshold. One `+Y` pose is contaminated while the other five pose means are
exact, and the report records the fitted-bias error and leave-one-out gate.

Run:

```bash
python3 validation/scan_static_imu_calibration_outlier_boundary.py \
  --calibrator build/aerakia_static_imu_calibration_cli
```

The reviewed result is
[`static_imu_calibration_outlier_boundary_v1.json`](../validation/public/static_imu_calibration_outlier_boundary_v1.json).

## Result

The radial contamination boundary is conservative: a `0.03 m/s²` radial
perturbation is accepted with about `0.0150 m/s²` fitted-bias error, while
`0.04 m/s²` is rejected by the leave-one-out influence gate. Tangential
contamination is much less observable in this six-axis geometry: `0.8 m/s²`
is still accepted with `0.0163 m/s²` bias error, while `1.0 m/s²` is rejected.

This is an important limitation, not a defect hidden by the report. The
calculator is a constant-bias seed estimator with geometry and influence
guards; it is not a general corrupted-window detector. The private FCOne
pre-arm collector must therefore reject windows using variance, settling,
clipping, vibration, temperature, and selector provenance before calling the
public calculator. If the product requires a tighter accepted seed-error
contract, the contract must be set from physical IMU data and the public
calculator's gate must be revisited in a separately registered study.

The scan is diagnostic evidence only. It does not establish physical
calibration accuracy, temperature behavior, or flight readiness.
