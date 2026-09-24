#!/usr/bin/env python3
"""Convert the MathWorks Blackbird VIO package into Aerakia replay CSV.

The package contains recorded 100 Hz IMU data and a motion-capture pose stream.  The
converter preserves their common absolute time base and applies the body/IMU rotation
published by the official Blackbird conversion tools.  Images are deliberately ignored.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np


GRAVITY_M_S2 = 9.80665
BODY_TO_IMU_QUATERNION_WXYZ = np.asarray(
    [0.707479362748676, 0.002029830683065, -0.007745228390866, 0.706688646087723],
    dtype=np.float64,
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _normalize_quaternions(quaternions: np.ndarray) -> np.ndarray:
    result = np.atleast_2d(np.asarray(quaternions, dtype=np.float64)).copy()
    norms = np.linalg.norm(result, axis=1)
    if np.any(~np.isfinite(norms) | (norms <= 1.0e-12)):
        raise ValueError("ground-truth quaternion stream contains invalid values")
    result /= norms[:, None]
    for index in range(1, len(result)):
        if np.dot(result[index - 1], result[index]) < 0.0:
            result[index] *= -1.0
    return result


def _quaternion_multiply(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    left = np.atleast_2d(np.asarray(left, dtype=np.float64))
    right = np.atleast_2d(np.asarray(right, dtype=np.float64))
    if len(left) == 1 and len(right) != 1:
        left = np.repeat(left, len(right), axis=0)
    if len(right) == 1 and len(left) != 1:
        right = np.repeat(right, len(left), axis=0)
    if len(left) != len(right):
        raise ValueError("quaternion arrays are not broadcast-compatible")
    w1, x1, y1, z1 = left.T
    w2, x2, y2, z2 = right.T
    return np.column_stack(
        (
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        )
    )


def _rotation_from_quaternion(quaternion: np.ndarray) -> np.ndarray:
    quaternion = _normalize_quaternions(quaternion)
    w, x, y, z = quaternion.T
    result = np.empty((len(quaternion), 3, 3), dtype=np.float64)
    result[:, 0, 0] = 1.0 - 2.0 * (y * y + z * z)
    result[:, 0, 1] = 2.0 * (x * y - w * z)
    result[:, 0, 2] = 2.0 * (x * z + w * y)
    result[:, 1, 0] = 2.0 * (x * y + w * z)
    result[:, 1, 1] = 1.0 - 2.0 * (x * x + z * z)
    result[:, 1, 2] = 2.0 * (y * z - w * x)
    result[:, 2, 0] = 2.0 * (x * z - w * y)
    result[:, 2, 1] = 2.0 * (y * z + w * x)
    result[:, 2, 2] = 1.0 - 2.0 * (x * x + y * y)
    return result


def _interpolate_quaternions(
    source_timestamp_ns: np.ndarray,
    source_quaternion: np.ndarray,
    query_timestamp_ns: np.ndarray,
) -> np.ndarray:
    source = _normalize_quaternions(source_quaternion)
    right = np.searchsorted(source_timestamp_ns, query_timestamp_ns, side="right")
    left = np.clip(right - 1, 0, len(source_timestamp_ns) - 1)
    right = np.clip(right, 0, len(source_timestamp_ns) - 1)
    result = np.empty((len(query_timestamp_ns), 4), dtype=np.float64)
    for index, query in enumerate(query_timestamp_ns):
        low, high = int(left[index]), int(right[index])
        if low == high:
            result[index] = source[low]
            continue
        alpha = float(query - source_timestamp_ns[low]) / float(
            source_timestamp_ns[high] - source_timestamp_ns[low]
        )
        candidate = (1.0 - alpha) * source[low] + alpha * source[high]
        result[index] = candidate / np.linalg.norm(candidate)
    return _normalize_quaternions(result)


def _interpolate_vectors(
    source_timestamp_ns: np.ndarray,
    source_values: np.ndarray,
    query_timestamp_ns: np.ndarray,
) -> np.ndarray:
    origin = int(min(source_timestamp_ns[0], query_timestamp_ns[0]))
    source_time = (source_timestamp_ns - origin).astype(np.float64) * 1.0e-9
    query_time = (query_timestamp_ns - origin).astype(np.float64) * 1.0e-9
    return np.column_stack(
        [np.interp(query_time, source_time, source_values[:, axis]) for axis in range(3)]
    )


def _quaternion_to_euler_deg(quaternion: np.ndarray) -> np.ndarray:
    w, x, y, z = _normalize_quaternions(quaternion).T
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return np.degrees(np.column_stack((roll, pitch, yaw)))


def _relative_angular_rate(
    timestamp_ns: np.ndarray, quaternion_ned_body: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    quaternion = _normalize_quaternions(quaternion_ned_body)
    conjugate = quaternion[:-1].copy()
    conjugate[:, 1:] *= -1.0
    relative = _quaternion_multiply(conjugate, quaternion[1:])
    relative[relative[:, 0] < 0.0] *= -1.0
    vector_norm = np.linalg.norm(relative[:, 1:], axis=1)
    angle = 2.0 * np.arctan2(vector_norm, np.clip(relative[:, 0], -1.0, 1.0))
    rotation_vector = np.zeros((len(relative), 3), dtype=np.float64)
    nonzero = vector_norm > 1.0e-12
    rotation_vector[nonzero] = (
        relative[nonzero, 1:] / vector_norm[nonzero, None] * angle[nonzero, None]
    )
    delta_s = np.diff(timestamp_ns).astype(np.float64) * 1.0e-9
    if np.any(delta_s <= 0.0):
        raise ValueError("ground-truth timestamps must be strictly increasing")
    midpoint_ns = timestamp_ns[:-1] + np.rint(0.5 * np.diff(timestamp_ns)).astype(np.int64)
    return midpoint_ns, rotation_vector / delta_s[:, None]


def _load_package(data_path: Path) -> tuple[np.ndarray, ...]:
    try:
        from scipy.io import loadmat
    except ImportError as error:
        raise RuntimeError(
            "Blackbird MAT conversion requires scipy; install the validation extra first"
        ) from error
    document = loadmat(data_path, squeeze_me=True, struct_as_record=False)
    required = ("accelReadings", "gyroReadings", "gTruth", "timeStamps")
    missing = [name for name in required if name not in document]
    if missing:
        raise ValueError(f"Blackbird MAT is missing fields: {', '.join(missing)}")
    acceleration = np.asarray(document["accelReadings"], dtype=np.float64)
    angular_rate = np.asarray(document["gyroReadings"], dtype=np.float64)
    truth = np.asarray(document["gTruth"], dtype=np.float64)
    timestamps = document["timeStamps"]
    imu_timestamp_ns = np.rint(np.asarray(timestamps.imuTimeStamps).reshape(-1)).astype(np.int64)
    truth_timestamp_ns = np.rint(
        np.asarray(timestamps.imageTimeStamps).reshape(-1)
    ).astype(np.int64)
    if acceleration.shape != angular_rate.shape or acceleration.shape[1] != 3:
        raise ValueError("Blackbird IMU arrays must be matching Nx3 arrays")
    if len(acceleration) != len(imu_timestamp_ns) or truth.shape != (len(truth_timestamp_ns), 7):
        raise ValueError("Blackbird sample counts do not match their timestamp streams")
    for values, name in (
        (acceleration, "acceleration"), (angular_rate, "angular rate"), (truth, "truth")
    ):
        if np.any(~np.isfinite(values)):
            raise ValueError(f"Blackbird {name} contains non-finite values")
    if np.any(np.diff(imu_timestamp_ns) <= 0) or np.any(np.diff(truth_timestamp_ns) <= 0):
        raise ValueError("Blackbird timestamps must be strictly increasing")
    return acceleration, angular_rate, truth, imu_timestamp_ns, truth_timestamp_ns


def convert_blackbird(
    sequence_dir: Path,
    output_path: Path,
    metadata_path: Path | None = None,
    synthetic_gnss_rate_hz: float = 10.0,
    synthetic_position_sigma_m: float = 0.5,
    synthetic_velocity_sigma_m_s: float = 0.1,
    seed: int = 7,
    static_hint_duration_s: float = 5.0,
    apply_static_reference_bias: bool = False,
) -> dict[str, object]:
    if synthetic_gnss_rate_hz < 0.0:
        raise ValueError("synthetic GNSS rate must be non-negative")
    if synthetic_position_sigma_m <= 0.0 or synthetic_velocity_sigma_m_s <= 0.0:
        raise ValueError("synthetic GNSS standard deviations must be positive")
    if not math.isfinite(static_hint_duration_s) or static_hint_duration_s < 0.0:
        raise ValueError("static hint duration must be finite and non-negative")
    sequence_dir = Path(sequence_dir)
    data_path = sequence_dir / "data.mat"
    acceleration_sensor, angular_rate_sensor, truth, imu_timestamp_ns, truth_timestamp_ns = (
        _load_package(data_path)
    )

    overlap = (imu_timestamp_ns >= truth_timestamp_ns[0]) & (
        imu_timestamp_ns <= truth_timestamp_ns[-1]
    )
    if np.count_nonzero(overlap) < 2:
        raise ValueError("Blackbird IMU and motion-capture streams do not overlap")
    acceleration_sensor = acceleration_sensor[overlap]
    angular_rate_sensor = angular_rate_sensor[overlap]
    imu_timestamp_ns = imu_timestamp_ns[overlap]

    rotation_body_imu = _rotation_from_quaternion(BODY_TO_IMU_QUATERNION_WXYZ)[0]
    acceleration_frd = acceleration_sensor @ rotation_body_imu.T
    angular_rate_frd = angular_rate_sensor @ rotation_body_imu.T
    truth_quaternion = _normalize_quaternions(truth[:, 3:7])
    reference_quaternion = _interpolate_quaternions(
        truth_timestamp_ns, truth_quaternion, imu_timestamp_ns
    )
    reference_position = _interpolate_vectors(
        truth_timestamp_ns, truth[:, :3], imu_timestamp_ns
    )
    reference_position -= reference_position[0]

    try:
        from scipy.signal import savgol_filter
    except ImportError as error:
        raise RuntimeError(
            "Blackbird truth differentiation requires scipy; install the validation extra first"
        ) from error
    truth_delta_s = float(np.median(np.diff(truth_timestamp_ns))) * 1.0e-9
    truth_velocity = savgol_filter(
        truth[:, :3], 21, 3, deriv=1, delta=truth_delta_s, axis=0, mode="interp"
    )
    truth_acceleration = savgol_filter(
        truth[:, :3], 21, 3, deriv=2, delta=truth_delta_s, axis=0, mode="interp"
    )
    reference_velocity = _interpolate_vectors(
        truth_timestamp_ns, truth_velocity, imu_timestamp_ns
    )

    timestamp_us = np.rint((imu_timestamp_ns - imu_timestamp_ns[0]) * 1.0e-3).astype(np.int64)
    static_hint = timestamp_us <= int(round(static_hint_duration_s * 1.0e6))
    if static_hint_duration_s == 0.0:
        static_hint[:] = False
    if np.count_nonzero(static_hint) > 0:
        static_gyro_bias = np.median(angular_rate_frd[static_hint], axis=0)
        static_rotation = _rotation_from_quaternion(reference_quaternion[static_hint])
        expected_static_specific_force = np.einsum(
            "nji,j->ni", static_rotation, np.asarray([0.0, 0.0, -GRAVITY_M_S2])
        )
        static_accel_bias = np.median(
            acceleration_frd[static_hint] - expected_static_specific_force, axis=0
        )
    else:
        static_gyro_bias = np.zeros(3)
        static_accel_bias = np.zeros(3)
    if apply_static_reference_bias:
        if np.count_nonzero(static_hint) < 20:
            raise ValueError("static reference-bias correction requires a verified static prefix")
        angular_rate_frd -= static_gyro_bias
        acceleration_frd -= static_accel_bias

    truth_midpoint_ns, truth_angular_rate_body = _relative_angular_rate(
        truth_timestamp_ns, truth_quaternion
    )
    measured_at_midpoint = _interpolate_vectors(
        imu_timestamp_ns, angular_rate_frd, truth_midpoint_ns
    )
    angular_overlap = (truth_midpoint_ns >= imu_timestamp_ns[0]) & (
        truth_midpoint_ns <= imu_timestamp_ns[-1]
    )
    angular_overlap &= np.linalg.norm(truth_angular_rate_body, axis=1) < 6.0
    angular_overlap &= np.linalg.norm(measured_at_midpoint, axis=1) < 6.0
    angular_error = truth_angular_rate_body[angular_overlap] - measured_at_midpoint[angular_overlap]
    angular_rmse = np.sqrt(np.mean(angular_error * angular_error, axis=0))
    angular_correlation = np.asarray(
        [
            np.corrcoef(
                truth_angular_rate_body[angular_overlap, axis],
                measured_at_midpoint[angular_overlap, axis],
            )[0, 1]
            for axis in range(3)
        ]
    )
    if np.any(angular_correlation < 0.95) or float(np.sqrt(np.mean(angular_error**2))) > 0.2:
        raise ValueError(
            "Blackbird time/frame validation failed; refusing to serialize ambiguous IMU axes"
        )

    truth_rotation = _rotation_from_quaternion(truth_quaternion)
    predicted_specific_force = np.einsum(
        "nji,nj->ni",
        truth_rotation,
        truth_acceleration - np.asarray([0.0, 0.0, GRAVITY_M_S2]),
    )
    measured_at_truth = _interpolate_vectors(
        imu_timestamp_ns, acceleration_frd, truth_timestamp_ns
    )
    force_overlap = (truth_timestamp_ns >= imu_timestamp_ns[0] + 1_000_000_000) & (
        truth_timestamp_ns <= imu_timestamp_ns[-1] - 1_000_000_000
    )
    force_overlap &= np.linalg.norm(measured_at_truth, axis=1) < 30.0
    force_error = predicted_specific_force[force_overlap] - measured_at_truth[force_overlap]
    force_rmse = np.sqrt(np.mean(force_error * force_error, axis=0))
    if float(np.sqrt(np.mean(force_error**2))) > 1.0:
        raise ValueError("Blackbird gravity/frame validation failed")

    euler_deg = _quaternion_to_euler_deg(reference_quaternion)
    rng = np.random.default_rng(seed)
    gps_position = reference_position + rng.normal(
        0.0, synthetic_position_sigma_m, reference_position.shape
    )
    gps_velocity = reference_velocity + rng.normal(
        0.0, synthetic_velocity_sigma_m_s, reference_velocity.shape
    )
    position_update = np.zeros(len(timestamp_us), dtype=np.int64)
    if synthetic_gnss_rate_hz > 0.0:
        period_us = max(1, int(round(1.0e6 / synthetic_gnss_rate_hz)))
        buckets = timestamp_us // period_us
        position_update[0] = 1
        position_update[1:] = buckets[1:] != buckets[:-1]

    output_path.parent.mkdir(parents=True, exist_ok=True)
    header = [
        "seq", "ts_us", "raw_acc_mg_x", "raw_acc_mg_y", "raw_acc_mg_z",
        "raw_gyro_mdps_x", "raw_gyro_mdps_y", "raw_gyro_mdps_z",
        "raw_mag_cuT_x", "raw_mag_cuT_y", "raw_mag_cuT_z",
        "mag_valid", "mag_update", "magnetic_declination_rad",
        "roll_mdeg", "pitch_mdeg", "yaw_mdeg",
        "ref_q_w", "ref_q_x", "ref_q_y", "ref_q_z", "position_ref_valid",
        "ref_position_n_m", "ref_position_e_m", "ref_position_d_m",
        "ref_velocity_n_m_s", "ref_velocity_e_m_s", "ref_velocity_d_m_s",
        "position_update", "gps_position_n_m", "gps_position_e_m", "gps_position_d_m",
        "gps_velocity_n_m_s", "gps_velocity_e_m_s", "gps_velocity_d_m_s",
        "gps_position_variance_m2", "gps_velocity_variance_m2_s2", "static_hint",
        "gnss_heading_valid", "gnss_heading_update", "gnss_heading_rad",
        "gnss_heading_variance_rad2", "gnss_heading_fault",
    ]
    with output_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        for index in range(len(timestamp_us)):
            writer.writerow(
                [
                    index, int(timestamp_us[index]),
                    *np.rint(acceleration_frd[index] / GRAVITY_M_S2 * 1000.0).astype(int),
                    *np.rint(np.degrees(angular_rate_frd[index]) * 1000.0).astype(int),
                    0, 0, 0, 0, 0, 0.0,
                    *np.rint(euler_deg[index] * 1000.0).astype(int),
                    *reference_quaternion[index], 1,
                    *reference_position[index], *reference_velocity[index],
                    int(position_update[index]), *gps_position[index], *gps_velocity[index],
                    synthetic_position_sigma_m**2, synthetic_velocity_sigma_m_s**2,
                    int(static_hint[index]), 0, 0, 0.0, math.radians(1.0) ** 2, 0,
                ]
            )

    speed = np.linalg.norm(reference_velocity, axis=1)
    gyro_norm = np.linalg.norm(angular_rate_frd, axis=1)
    accel_norm = np.linalg.norm(acceleration_frd, axis=1)
    metadata: dict[str, object] = {
        "dataset": "Blackbird UAV",
        "sequence": "NYC Subway Winter (MathWorks VIO package)",
        "source_paper": "arXiv:1810.01987",
        "source_license": "MIT (license included with redistributed package)",
        "reference_kind": "independent_truth",
        "samples": int(len(timestamp_us)),
        "duration_s": float(timestamp_us[-1]) * 1.0e-6,
        "input_sha256": {"data.mat": _sha256(data_path)},
        "streams": {
            "imu_rate_hz": float(1.0e9 / np.median(np.diff(imu_timestamp_ns))),
            "motion_capture_rate_hz": float(1.0e9 / np.median(np.diff(truth_timestamp_ns))),
            "motion_capture_start_minus_imu_start_s": float(
                (truth_timestamp_ns[0] - imu_timestamp_ns[0]) * 1.0e-9
            ),
        },
        "frame_conversion": {
            "source_navigation": "mocap NED",
            "source_body": "Blackbird body frame",
            "target": "Aerakia FRD body and local NED navigation",
            "body_to_imu_quaternion_wxyz": BODY_TO_IMU_QUATERNION_WXYZ.tolist(),
            "application": "recorded IMU vectors rotated from IMU sensor axes into body FRD",
            "angular_rate_truth_vs_imu_axis_correlation": angular_correlation.tolist(),
            "angular_rate_truth_vs_imu_axis_rmse_rad_s": angular_rmse.tolist(),
            "angular_rate_truth_vs_imu_total_rmse_rad_s": float(
                np.sqrt(np.mean(angular_error**2))
            ),
            "specific_force_truth_vs_imu_axis_rmse_m_s2": force_rmse.tolist(),
            "specific_force_truth_vs_imu_total_rmse_m_s2": float(
                np.sqrt(np.mean(force_error**2))
            ),
        },
        "motion_coverage": {
            "speed_p95_m_s": float(np.percentile(speed, 95.0)),
            "speed_max_m_s": float(np.max(speed)),
            "gyro_norm_p95_rad_s": float(np.percentile(gyro_norm, 95.0)),
            "gyro_norm_p99_rad_s": float(np.percentile(gyro_norm, 99.0)),
            "gyro_norm_max_rad_s": float(np.max(gyro_norm)),
            "accel_norm_p95_m_s2": float(np.percentile(accel_norm, 95.0)),
            "accel_norm_p99_m_s2": float(np.percentile(accel_norm, 99.0)),
            "accel_norm_max_m_s2": float(np.max(accel_norm)),
        },
        "synthetic_gnss": {
            "enabled": bool(synthetic_gnss_rate_hz > 0.0),
            "source": "motion-capture position plus smoothed differentiated velocity",
            "rate_hz": synthetic_gnss_rate_hz,
            "position_sigma_m": synthetic_position_sigma_m,
            "velocity_sigma_m_s": synthetic_velocity_sigma_m_s,
            "seed": seed,
            "updates": int(np.count_nonzero(position_update)),
        },
        "application_static_hint": {
            "duration_s": static_hint_duration_s,
            "samples": int(np.count_nonzero(static_hint)),
            "source": "externally verified near-zero motion-capture displacement and speed",
        },
        "reference_bias_correction": {
            "applied": apply_static_reference_bias,
            "source": "median recorded IMU residual during the externally verified static prefix",
            "gyro_bias_rad_s": static_gyro_bias.tolist(),
            "accel_bias_m_s2": static_accel_bias.tolist(),
        },
        "limitations": [
            "The package contains no recorded magnetometer, GNSS, or trusted-heading observation.",
            "Synthetic GNSS is generated from motion-capture truth and is not receiver evidence.",
            "Static reference-bias correction, when enabled, isolates propagation math and is not online observability evidence.",
            "This redistributed sequence reaches aggressive angular motion but not the 5-7 m/s maximum speed of the full Blackbird corpus.",
        ],
    }
    if metadata_path is not None:
        metadata_path.parent.mkdir(parents=True, exist_ok=True)
        metadata_path.write_text(
            json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
    return metadata


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sequence_dir", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--metadata", type=Path)
    parser.add_argument("--synthetic-gnss-rate-hz", type=float, default=10.0)
    parser.add_argument("--synthetic-position-sigma-m", type=float, default=0.5)
    parser.add_argument("--synthetic-velocity-sigma-m-s", type=float, default=0.1)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--static-hint-duration-s", type=float, default=5.0)
    parser.add_argument("--apply-static-reference-bias", action="store_true")
    args = parser.parse_args()
    metadata = convert_blackbird(
        args.sequence_dir,
        args.out,
        args.metadata,
        args.synthetic_gnss_rate_hz,
        args.synthetic_position_sigma_m,
        args.synthetic_velocity_sigma_m_s,
        args.seed,
        args.static_hint_duration_s,
        args.apply_static_reference_bias,
    )
    print(json.dumps(metadata, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
