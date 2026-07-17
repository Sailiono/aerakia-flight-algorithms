#!/usr/bin/env python3
"""Convert a EuRoC ASL-format sequence into Aerakia's NED/FRD replay contract."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import yaml


GRAVITY_M_S2 = 9.80665
SOURCE_TO_AERAKIA = np.diag([1.0, -1.0, -1.0])


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _load_csv(path: Path, expected_columns: int) -> np.ndarray:
    values = np.atleast_2d(np.loadtxt(path, delimiter=",", comments="#", dtype=np.float64))
    if values.shape[1] != expected_columns:
        raise ValueError(f"{path} has {values.shape[1]} columns, expected {expected_columns}")
    timestamps = values[:, 0].astype(np.int64)
    if np.any(np.diff(timestamps) <= 0):
        raise ValueError(f"timestamps must be strictly increasing: {path}")
    return values


def _validate_identity_extrinsics(path: Path) -> dict[str, object]:
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    transform = document.get("T_BS")
    if not isinstance(transform, dict):
        raise ValueError(f"missing T_BS in {path}")
    matrix = np.asarray(transform["data"], dtype=np.float64).reshape(
        int(transform["rows"]), int(transform["cols"])
    )
    if matrix.shape != (4, 4) or not np.allclose(matrix, np.eye(4), atol=1.0e-12):
        raise ValueError(
            "this converter currently requires identity EuRoC IMU/body extrinsics; "
            "implement and test the general transform before using this sequence"
        )
    return document


def _normalize_quaternions(quaternions: np.ndarray) -> np.ndarray:
    result = np.asarray(quaternions, dtype=np.float64).copy()
    norms = np.linalg.norm(result, axis=1)
    if np.any(~np.isfinite(norms) | (norms <= 1.0e-12)):
        raise ValueError("ground-truth quaternion stream contains invalid values")
    result /= norms[:, None]
    for index in range(1, len(result)):
        if np.dot(result[index - 1], result[index]) < 0.0:
            result[index] *= -1.0
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
        low = int(left[index])
        high = int(right[index])
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
    origin = int(source_timestamp_ns[0])
    source_time = (source_timestamp_ns - origin).astype(np.float64) * 1.0e-9
    query_time = (query_timestamp_ns - origin).astype(np.float64) * 1.0e-9
    return np.column_stack(
        [np.interp(query_time, source_time, source_values[:, axis]) for axis in range(3)]
    )


def _quaternion_to_euler_deg(quaternion: np.ndarray) -> np.ndarray:
    w, x, y, z = quaternion.T
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return np.degrees(np.column_stack((roll, pitch, yaw)))


def _rotation_from_quaternion(quaternion: np.ndarray) -> np.ndarray:
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


def convert_euroc(
    sequence_dir: Path,
    output_path: Path,
    metadata_path: Path | None = None,
    synthetic_gnss_rate_hz: float = 0.0,
    synthetic_position_sigma_m: float = 0.5,
    synthetic_velocity_sigma_m_s: float = 0.1,
    seed: int = 7,
    apply_reference_bias: bool = False,
) -> dict[str, object]:
    mav0 = sequence_dir / "mav0"
    imu_path = mav0 / "imu0" / "data.csv"
    imu_yaml_path = mav0 / "imu0" / "sensor.yaml"
    truth_path = mav0 / "state_groundtruth_estimate0" / "data.csv"
    truth_yaml_path = mav0 / "state_groundtruth_estimate0" / "sensor.yaml"
    imu_yaml = _validate_identity_extrinsics(imu_yaml_path)
    _validate_identity_extrinsics(truth_yaml_path)
    imu = _load_csv(imu_path, 7)
    truth = _load_csv(truth_path, 17)
    imu_timestamp_ns = imu[:, 0].astype(np.int64)
    truth_timestamp_ns = truth[:, 0].astype(np.int64)
    overlap = (imu_timestamp_ns >= truth_timestamp_ns[0]) & (imu_timestamp_ns <= truth_timestamp_ns[-1])
    if np.count_nonzero(overlap) < 2:
        raise ValueError("IMU and ground-truth streams do not overlap")
    imu = imu[overlap]
    imu_timestamp_ns = imu_timestamp_ns[overlap]

    source_quaternion = _interpolate_quaternions(
        truth_timestamp_ns, truth[:, 4:8], imu_timestamp_ns
    )
    source_position = _interpolate_vectors(
        truth_timestamp_ns, truth[:, 1:4], imu_timestamp_ns
    )
    source_velocity = _interpolate_vectors(
        truth_timestamp_ns, truth[:, 8:11], imu_timestamp_ns
    )
    source_acceleration = imu[:, 4:7].copy()
    source_angular_rate = imu[:, 1:4].copy()
    reference_gyro_bias = _interpolate_vectors(
        truth_timestamp_ns, truth[:, 11:14], imu_timestamp_ns
    )
    reference_accel_bias = _interpolate_vectors(
        truth_timestamp_ns, truth[:, 14:17], imu_timestamp_ns
    )
    if apply_reference_bias:
        source_angular_rate -= reference_gyro_bias
        source_acceleration -= reference_accel_bias

    world_specific_force = np.einsum(
        "nij,nj->ni", _rotation_from_quaternion(source_quaternion), source_acceleration
    )
    median_world_specific_force = np.median(world_specific_force, axis=0)
    if median_world_specific_force[2] < 0.5 * GRAVITY_M_S2:
        raise ValueError(
            "EuRoC quaternion/gravity convention check failed; refusing an ambiguous frame conversion"
        )

    acceleration_frd = source_acceleration @ SOURCE_TO_AERAKIA
    angular_rate_frd = source_angular_rate @ SOURCE_TO_AERAKIA
    position_ned = source_position @ SOURCE_TO_AERAKIA
    position_ned -= position_ned[0]
    velocity_ned = source_velocity @ SOURCE_TO_AERAKIA
    quaternion_ned_frd = source_quaternion.copy()
    quaternion_ned_frd[:, 2:] *= -1.0
    quaternion_ned_frd = _normalize_quaternions(quaternion_ned_frd)
    euler_deg = _quaternion_to_euler_deg(quaternion_ned_frd)

    rng = np.random.default_rng(seed)
    gps_position = position_ned + rng.normal(0.0, synthetic_position_sigma_m, position_ned.shape)
    gps_velocity = velocity_ned + rng.normal(0.0, synthetic_velocity_sigma_m_s, velocity_ned.shape)
    position_update = np.zeros(len(imu), dtype=np.int64)
    if synthetic_gnss_rate_hz > 0.0:
        period_ns = max(1, int(round(1.0e9 / synthetic_gnss_rate_hz)))
        buckets = (imu_timestamp_ns - imu_timestamp_ns[0]) // period_ns
        position_update[0] = 1
        position_update[1:] = buckets[1:] != buckets[:-1]

    output_path.parent.mkdir(parents=True, exist_ok=True)
    header = [
        "seq", "ts_us",
        "raw_acc_mg_x", "raw_acc_mg_y", "raw_acc_mg_z",
        "raw_gyro_mdps_x", "raw_gyro_mdps_y", "raw_gyro_mdps_z",
        "raw_mag_cuT_x", "raw_mag_cuT_y", "raw_mag_cuT_z",
        "mag_valid", "mag_update", "magnetic_declination_rad",
        "roll_mdeg", "pitch_mdeg", "yaw_mdeg",
        "ref_q_w", "ref_q_x", "ref_q_y", "ref_q_z",
        "position_ref_valid",
        "ref_position_n_m", "ref_position_e_m", "ref_position_d_m",
        "ref_velocity_n_m_s", "ref_velocity_e_m_s", "ref_velocity_d_m_s",
        "position_update",
        "gps_position_n_m", "gps_position_e_m", "gps_position_d_m",
        "gps_velocity_n_m_s", "gps_velocity_e_m_s", "gps_velocity_d_m_s",
        "gps_position_variance_m2", "gps_velocity_variance_m2_s2", "static_hint",
    ]
    with output_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        timestamp_us = np.rint((imu_timestamp_ns - imu_timestamp_ns[0]) * 1.0e-3).astype(np.int64)
        for index in range(len(imu)):
            writer.writerow(
                [
                    index, int(timestamp_us[index]),
                    *np.rint(acceleration_frd[index] / GRAVITY_M_S2 * 1000.0).astype(int),
                    *np.rint(np.degrees(angular_rate_frd[index]) * 1000.0).astype(int),
                    0, 0, 0, 0, 0, 0.0,
                    *np.rint(euler_deg[index] * 1000.0).astype(int),
                    *quaternion_ned_frd[index], 1,
                    *position_ned[index], *velocity_ned[index], int(position_update[index]),
                    *gps_position[index], *gps_velocity[index],
                    synthetic_position_sigma_m**2, synthetic_velocity_sigma_m_s**2, 0,
                ]
            )

    metadata: dict[str, object] = {
        "dataset": "EuRoC MAV",
        "sequence": sequence_dir.name,
        "source_doi": "10.3929/ethz-b-000690084",
        "source_license": "In Copyright - Non-Commercial Use Permitted",
        "reference_kind": "independent_truth",
        "samples": int(len(imu)),
        "duration_s": float((imu_timestamp_ns[-1] - imu_timestamp_ns[0]) * 1.0e-9),
        "nominal_imu_rate_hz": float(imu_yaml.get("rate_hz", math.nan)),
        "input_sha256": {
            "imu0/data.csv": _sha256(imu_path),
            "state_groundtruth_estimate0/data.csv": _sha256(truth_path),
        },
        "frame_conversion": {
            "source": "EuRoC right-handed body/reference frames with world +Z up",
            "target": "Aerakia FRD body and local NED-like navigation frame",
            "vector_matrix_diagonal": [1.0, -1.0, -1.0],
            "quaternion_mapping_wxyz": ["w", "x", "-y", "-z"],
            "position_origin": "first interpolated truth sample",
            "median_source_world_specific_force_m_s2": median_world_specific_force.tolist(),
        },
        "synthetic_gnss": {
            "enabled": bool(synthetic_gnss_rate_hz > 0.0),
            "rate_hz": synthetic_gnss_rate_hz,
            "position_sigma_m": synthetic_position_sigma_m,
            "velocity_sigma_m_s": synthetic_velocity_sigma_m_s,
            "seed": seed,
            "updates": int(np.count_nonzero(position_update)),
        },
        "reference_bias_correction": {
            "applied": apply_reference_bias,
            "source": "interpolated EuRoC batch-estimated b_w_RS_S and b_a_RS_S",
            "median_gyro_bias_rad_s": np.median(reference_gyro_bias, axis=0).tolist(),
            "median_accel_bias_m_s2": np.median(reference_accel_bias, axis=0).tolist(),
        },
        "limitations": [
            "EuRoC contains no magnetometer or GNSS measurements used by this replay.",
            "Machine Hall orientation is an IMU-aided batch estimate; position uses Leica and IMU.",
            "Synthetic GNSS, when enabled, is generated from truth and is not a recorded sensor.",
            "Reference-bias correction, when enabled, validates propagation/update math but not online bias observability.",
        ],
    }
    if metadata_path is not None:
        metadata_path.parent.mkdir(parents=True, exist_ok=True)
        metadata_path.write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return metadata


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sequence_dir", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--metadata", type=Path)
    parser.add_argument("--synthetic-gnss-rate-hz", type=float, default=0.0)
    parser.add_argument("--synthetic-position-sigma-m", type=float, default=0.5)
    parser.add_argument("--synthetic-velocity-sigma-m-s", type=float, default=0.1)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument(
        "--apply-reference-bias",
        action="store_true",
        help="subtract EuRoC batch-estimated IMU biases (diagnostic math-validation track)",
    )
    args = parser.parse_args()
    if args.synthetic_gnss_rate_hz < 0.0:
        parser.error("--synthetic-gnss-rate-hz must be non-negative")
    if args.synthetic_position_sigma_m <= 0.0 or args.synthetic_velocity_sigma_m_s <= 0.0:
        parser.error("synthetic GNSS standard deviations must be positive")
    metadata = convert_euroc(
        args.sequence_dir,
        args.out,
        args.metadata,
        args.synthetic_gnss_rate_hz,
        args.synthetic_position_sigma_m,
        args.synthetic_velocity_sigma_m_s,
        args.seed,
        args.apply_reference_bias,
    )
    print(json.dumps(metadata, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
