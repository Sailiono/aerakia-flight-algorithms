#!/usr/bin/env python3
"""Convert raw INSANE IMU, magnetometer, and OptiTrack streams to replay CSV.

The magnetometer is an estimator input. The raw OptiTrack pose is used only as an independent
scoring reference. Generated INSANE ground-truth products are deliberately unsupported because
their outdoor attitude can consume the same magnetometer used by the estimator.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
from typing import Any

import numpy as np


GRAVITY_M_S2 = 9.80665
FLU_TO_FRD = np.diag([1.0, -1.0, -1.0])
ALLOWED_SPLITS = {"calibration", "development", "holdout"}
ALLOWED_CALIBRATION_SOURCES = {"external", "calibration", "development"}
REPLAY_HEADER = [
    "seq", "ts_us", "raw_acc_mg_x", "raw_acc_mg_y", "raw_acc_mg_z",
    "raw_gyro_mdps_x", "raw_gyro_mdps_y", "raw_gyro_mdps_z",
    "raw_mag_cuT_x", "raw_mag_cuT_y", "raw_mag_cuT_z", "mag_valid", "mag_update",
    "magnetic_declination_rad", "roll_mdeg", "pitch_mdeg", "yaw_mdeg",
    "ref_q_w", "ref_q_x", "ref_q_y", "ref_q_z", "position_ref_valid",
    "ref_position_n_m", "ref_position_e_m", "ref_position_d_m",
    "ref_velocity_n_m_s", "ref_velocity_e_m_s", "ref_velocity_d_m_s",
    "position_update", "gps_position_n_m", "gps_position_e_m", "gps_position_d_m",
    "gps_velocity_n_m_s", "gps_velocity_e_m_s", "gps_velocity_d_m_s",
    "gps_position_variance_m2", "gps_velocity_variance_m2_s2", "static_hint",
    "gnss_heading_valid", "gnss_heading_update", "gnss_heading_rad",
    "gnss_heading_variance_rad2", "gnss_heading_fault",
]


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _load_named(path: Path, required: tuple[str, ...]) -> np.ndarray:
    if not path.is_file():
        raise ValueError(f"missing source CSV: {path}")
    data = np.genfromtxt(path, delimiter=",", names=True, dtype=np.float64)
    data = np.atleast_1d(data)
    if data.size == 0 or data.dtype.names is None:
        raise ValueError(f"empty or unlabelled CSV: {path}")
    missing = [name for name in required if name not in data.dtype.names]
    if missing:
        raise ValueError(f"{path} is missing columns: {', '.join(missing)}")
    values = np.column_stack([data[name] for name in required])
    if np.any(~np.isfinite(values)):
        raise ValueError(f"{path} contains non-finite values in required columns")
    if len(data) < 2 or np.any(np.diff(data["t"]) <= 0.0):
        raise ValueError(f"timestamps must be strictly increasing: {path}")
    return data


def _proper_rotation(value: Any, name: str) -> np.ndarray:
    matrix = np.asarray(value, dtype=np.float64)
    if matrix.shape != (3, 3) or np.any(~np.isfinite(matrix)):
        raise ValueError(f"{name} must be a finite 3x3 matrix")
    if not np.allclose(matrix.T @ matrix, np.eye(3), atol=2.0e-5):
        raise ValueError(f"{name} is not orthonormal")
    if not np.isclose(np.linalg.det(matrix), 1.0, atol=2.0e-5):
        raise ValueError(f"{name} is not a proper rotation")
    return matrix


def _finite_vector(value: Any, size: int, name: str) -> np.ndarray:
    vector = np.asarray(value, dtype=np.float64)
    if vector.shape != (size,) or np.any(~np.isfinite(vector)):
        raise ValueError(f"{name} must be a finite {size}-vector")
    return vector


def _finite_scalar(document: dict[str, Any], name: str) -> float:
    if name not in document:
        raise ValueError(f"manifest is missing {name}")
    value = float(document[name])
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


def _window(value: Any, name: str) -> tuple[float, float]:
    numbers = np.asarray(value, dtype=np.float64)
    if numbers.shape != (2,) or np.any(~np.isfinite(numbers)):
        raise ValueError(f"{name} must contain two finite seconds")
    start, stop = map(float, numbers)
    if start < 0.0 or stop <= start:
        raise ValueError(f"{name} must satisfy 0 <= start < stop")
    return start, stop


def _overlap(first: tuple[float, float], second: tuple[float, float]) -> bool:
    return max(first[0], second[0]) < min(first[1], second[1])


def _source_path(sequence_dir: Path, value: Any, expected: str) -> Path:
    name = str(value)
    path = Path(name)
    if path.is_absolute() or len(path.parts) != 1 or path.name != expected:
        raise ValueError(
            f"{expected} must be read directly from the sequence directory; found {name!r}"
        )
    resolved = sequence_dir / path
    if "ground_truth" in {part.lower() for part in resolved.parts}:
        raise ValueError("generated ground_truth paths are forbidden for this converter")
    return resolved


def _load_manifest(path: Path) -> dict[str, Any]:
    document = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict) or document.get("schema_version") != 1:
        raise ValueError("manifest schema_version must be 1")
    sequence = document.get("sequence")
    if not isinstance(sequence, str) or not sequence.strip():
        raise ValueError("manifest sequence must be a non-empty string")
    split = document.get("split")
    if not isinstance(split, dict) or split.get("name") not in ALLOWED_SPLITS:
        raise ValueError("split.name must be calibration, development, or holdout")
    output_window = _window(split.get("window_s"), "split.window_s")
    calibration = document.get("calibration")
    if not isinstance(calibration, dict):
        raise ValueError("manifest calibration object is required")
    source_split = calibration.get("source_split")
    if source_split not in ALLOWED_CALIBRATION_SOURCES:
        raise ValueError("calibration.source_split must be external, calibration, or development")
    source_sequence = calibration.get("source_sequence")
    if not isinstance(source_sequence, str) or not source_sequence.strip():
        raise ValueError("calibration.source_sequence must be a non-empty string")
    source_window_value = calibration.get("source_window_s")
    if source_split == "external":
        if source_window_value is not None:
            _window(source_window_value, "calibration.source_window_s")
    else:
        source_window = _window(source_window_value, "calibration.source_window_s")
        if (
            source_sequence == sequence
            and split["name"] != source_split
            and _overlap(output_window, source_window)
        ):
            raise ValueError("output and calibration source windows must be disjoint")
    if split["name"] == "holdout" and source_split == "holdout":
        raise ValueError("holdout data cannot provide its own calibration")
    if not str(calibration.get("provenance", "")).strip():
        raise ValueError("calibration.provenance must describe where calibration came from")
    streams = document.get("streams")
    if not isinstance(streams, dict):
        raise ValueError("manifest streams object is required")
    audit = document.get("audit")
    if not isinstance(audit, dict):
        raise ValueError("manifest audit object is required")
    return document


def _normalize_quaternions(value: np.ndarray) -> np.ndarray:
    quaternions = np.asarray(value, dtype=np.float64).copy()
    norms = np.linalg.norm(quaternions, axis=1)
    if np.any(~np.isfinite(norms)) or np.any(norms < 1.0e-9):
        raise ValueError("OptiTrack quaternion stream contains invalid values")
    quaternions /= norms[:, None]
    for index in range(1, len(quaternions)):
        if np.dot(quaternions[index - 1], quaternions[index]) < 0.0:
            quaternions[index] *= -1.0
    return quaternions


def _rotation_from_quaternion(quaternion: np.ndarray) -> np.ndarray:
    w, x, y, z = _normalize_quaternions(quaternion).T
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


def _quaternion_from_rotation(matrix: np.ndarray) -> np.ndarray:
    result = np.empty((len(matrix), 4), dtype=np.float64)
    for index, rotation in enumerate(matrix):
        trace = float(np.trace(rotation))
        if trace > 0.0:
            scale = math.sqrt(trace + 1.0) * 2.0
            quaternion = np.asarray(
                [0.25 * scale, (rotation[2, 1] - rotation[1, 2]) / scale,
                 (rotation[0, 2] - rotation[2, 0]) / scale,
                 (rotation[1, 0] - rotation[0, 1]) / scale]
            )
        else:
            axis = int(np.argmax(np.diag(rotation)))
            if axis == 0:
                scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
                quaternion = np.asarray(
                    [(rotation[2, 1] - rotation[1, 2]) / scale, 0.25 * scale,
                     (rotation[0, 1] + rotation[1, 0]) / scale,
                     (rotation[0, 2] + rotation[2, 0]) / scale]
                )
            elif axis == 1:
                scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
                quaternion = np.asarray(
                    [(rotation[0, 2] - rotation[2, 0]) / scale,
                     (rotation[0, 1] + rotation[1, 0]) / scale, 0.25 * scale,
                     (rotation[1, 2] + rotation[2, 1]) / scale]
                )
            else:
                scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
                quaternion = np.asarray(
                    [(rotation[1, 0] - rotation[0, 1]) / scale,
                     (rotation[0, 2] + rotation[2, 0]) / scale,
                     (rotation[1, 2] + rotation[2, 1]) / scale, 0.25 * scale]
                )
        result[index] = quaternion / np.linalg.norm(quaternion)
    return _normalize_quaternions(result)


def _quaternion_multiply(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = left.T
    w2, x2, y2, z2 = right.T
    return np.column_stack(
        (w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
         w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
         w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
         w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2)
    )


def _interpolate_quaternions(
    source_time_s: np.ndarray, source: np.ndarray, query_time_s: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    source = _normalize_quaternions(source)
    right = np.searchsorted(source_time_s, query_time_s, side="right")
    left = np.clip(right - 1, 0, len(source_time_s) - 1)
    right = np.clip(right, 0, len(source_time_s) - 1)
    result = np.empty((len(query_time_s), 4), dtype=np.float64)
    bracket_gap = source_time_s[right] - source_time_s[left]
    for index, query in enumerate(query_time_s):
        low, high = int(left[index]), int(right[index])
        if low == high:
            result[index] = source[low]
            continue
        alpha = float(query - source_time_s[low]) / float(
            source_time_s[high] - source_time_s[low]
        )
        candidate = (1.0 - alpha) * source[low] + alpha * source[high]
        result[index] = candidate / np.linalg.norm(candidate)
    return _normalize_quaternions(result), bracket_gap


def _interpolate_vectors(
    source_time_s: np.ndarray, source: np.ndarray, query_time_s: np.ndarray
) -> np.ndarray:
    return np.column_stack(
        [np.interp(query_time_s, source_time_s, source[:, axis]) for axis in range(3)]
    )


def _quaternion_to_euler_deg(quaternion: np.ndarray) -> np.ndarray:
    w, x, y, z = _normalize_quaternions(quaternion).T
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return np.degrees(np.column_stack((roll, pitch, yaw)))


def _map_magnetometer(
    imu_time_s: np.ndarray,
    mag_time_s: np.ndarray,
    magnetometer_frd_t: np.ndarray,
    maximum_residual_s: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    update = np.zeros(len(imu_time_s), dtype=np.int64)
    valid = np.zeros(len(imu_time_s), dtype=np.int64)
    values = np.zeros((len(imu_time_s), 3), dtype=np.float64)
    residual = np.full(len(imu_time_s), np.nan, dtype=np.float64)
    right = np.searchsorted(imu_time_s, mag_time_s)
    mapped: dict[int, tuple[float, int]] = {}
    for source_index, candidate in enumerate(right):
        choices = [index for index in (candidate - 1, candidate) if 0 <= index < len(imu_time_s)]
        if not choices:
            continue
        row = min(choices, key=lambda index: abs(imu_time_s[index] - mag_time_s[source_index]))
        delta = abs(float(imu_time_s[row] - mag_time_s[source_index]))
        if delta > maximum_residual_s:
            raise ValueError(
                f"magnetometer-to-IMU time residual {delta:.6f}s exceeds "
                f"{maximum_residual_s:.6f}s"
            )
        previous = mapped.get(row)
        if previous is None or delta < previous[0]:
            mapped[row] = (delta, source_index)
    if not mapped:
        raise ValueError("no magnetometer samples map to the selected IMU split")
    for row, (delta, source_index) in mapped.items():
        update[row] = 1
        valid[row] = 1
        values[row] = magnetometer_frd_t[source_index]
        residual[row] = delta
    return values, valid, update, residual


def _angular_rate_audit(
    timestamp_s: np.ndarray,
    quaternion_ned_frd: np.ndarray,
    angular_rate_frd: np.ndarray,
    window_s: float,
) -> dict[str, object]:
    median_dt = float(np.median(np.diff(timestamp_s)))
    lag = max(1, int(round(window_s / median_dt)))
    if lag >= len(timestamp_s) - 1:
        raise ValueError("selected split is too short for angular-rate frame audit")
    first = quaternion_ned_frd[:-lag].copy()
    first[:, 1:] *= -1.0
    relative = _quaternion_multiply(first, quaternion_ned_frd[lag:])
    relative[relative[:, 0] < 0.0] *= -1.0
    vector_norm = np.linalg.norm(relative[:, 1:], axis=1)
    angle = 2.0 * np.arctan2(vector_norm, np.clip(relative[:, 0], -1.0, 1.0))
    truth_rate = np.zeros((len(relative), 3), dtype=np.float64)
    nonzero = vector_norm > 1.0e-12
    truth_rate[nonzero] = (
        relative[nonzero, 1:] / vector_norm[nonzero, None] * angle[nonzero, None]
    )
    interval_s = timestamp_s[lag:] - timestamp_s[:-lag]
    truth_rate /= interval_s[:, None]

    segment_dt = np.diff(timestamp_s)
    trapezoid = 0.5 * (angular_rate_frd[:-1] + angular_rate_frd[1:]) * segment_dt[:, None]
    integral = np.vstack((np.zeros((1, 3)), np.cumsum(trapezoid, axis=0)))
    measured_rate = (integral[lag:] - integral[:-lag]) / interval_s[:, None]
    usable = (np.linalg.norm(truth_rate, axis=1) < 6.0) & (
        np.linalg.norm(measured_rate, axis=1) < 6.0
    )
    truth_rate = truth_rate[usable]
    measured_rate = measured_rate[usable]
    if len(truth_rate) < 2:
        raise ValueError("insufficient finite angular-rate samples for frame audit")
    error = truth_rate - measured_rate
    truth_norm = np.linalg.norm(truth_rate, axis=1)
    measured_norm = np.linalg.norm(measured_rate, axis=1)
    if np.std(truth_norm) < 1.0e-9 or np.std(measured_norm) < 1.0e-9:
        norm_correlation = 0.0
    else:
        norm_correlation = float(np.corrcoef(truth_norm, measured_norm)[0, 1])
    axis_correlation = []
    for axis in range(3):
        if np.std(truth_rate[:, axis]) < 1.0e-9 or np.std(measured_rate[:, axis]) < 1.0e-9:
            axis_correlation.append(0.0)
        else:
            axis_correlation.append(
                float(np.corrcoef(truth_rate[:, axis], measured_rate[:, axis])[0, 1])
            )
    return {
        "samples": int(len(truth_rate)),
        "window_s": float(lag * median_dt),
        "norm_correlation": norm_correlation,
        "axis_correlation": axis_correlation,
        "axis_rmse_rad_s": np.sqrt(np.mean(error * error, axis=0)).tolist(),
        "total_rmse_rad_s": float(np.sqrt(np.mean(error * error))),
        "error_p95_rad_s": float(np.percentile(np.linalg.norm(error, axis=1), 95.0)),
    }


def _gravity_audit(
    acceleration_frd: np.ndarray,
    angular_rate_frd: np.ndarray,
    quaternion_ned_frd: np.ndarray,
) -> dict[str, object]:
    rotation = _rotation_from_quaternion(quaternion_ned_frd)
    expected = np.einsum(
        "nji,j->ni", rotation, np.asarray([0.0, 0.0, -GRAVITY_M_S2])
    )
    acceleration_norm = np.linalg.norm(acceleration_frd, axis=1)
    usable = (np.linalg.norm(angular_rate_frd, axis=1) < 0.08) & (
        np.abs(acceleration_norm - GRAVITY_M_S2) < 0.75
    )
    measured = acceleration_frd[usable]
    expected = expected[usable]
    if len(measured) == 0:
        return {"samples": 0, "median_direction_error_deg": None, "p95_direction_error_deg": None}
    cosine = np.sum(measured * expected, axis=1) / (
        np.linalg.norm(measured, axis=1) * np.linalg.norm(expected, axis=1)
    )
    error_deg = np.degrees(np.arccos(np.clip(cosine, -1.0, 1.0)))
    return {
        "samples": int(len(error_deg)),
        "median_direction_error_deg": float(np.median(error_deg)),
        "p95_direction_error_deg": float(np.percentile(error_deg, 95.0)),
    }


def convert_insane_mocap(
    sequence_dir: Path,
    manifest_path: Path,
    output_path: Path,
    metadata_path: Path | None = None,
) -> dict[str, object]:
    sequence_dir = Path(sequence_dir)
    manifest_path = Path(manifest_path)
    manifest = _load_manifest(manifest_path)
    streams = manifest["streams"]
    imu_path = _source_path(sequence_dir, streams.get("imu"), "px4_imu.csv")
    mag_path = _source_path(sequence_dir, streams.get("magnetometer"), "px4_mag.csv")
    mocap_path = _source_path(
        sequence_dir, streams.get("reference"), "mocap_vehicle_data.csv"
    )

    imu = _load_named(imu_path, ("t", "a_x", "a_y", "a_z", "w_x", "w_y", "w_z"))
    mag = _load_named(mag_path, ("t", "cart_x", "cart_y", "cart_z"))
    mocap = _load_named(
        mocap_path, ("t", "p_x", "p_y", "p_z", "q_w", "q_x", "q_y", "q_z")
    )

    calibration = manifest["calibration"]
    mocap_time_offset_s = _finite_scalar(calibration, "mocap_time_offset_s")
    if abs(mocap_time_offset_s) > 1.0:
        raise ValueError("mocap_time_offset_s exceeds the reviewed +/-1s range")
    mocap_body_rotation = _proper_rotation(
        calibration.get("mocap_body_rotation"), "calibration.mocap_body_rotation"
    )
    mocap_world_to_ned = _proper_rotation(
        calibration.get("mocap_world_to_ned_rotation"),
        "calibration.mocap_world_to_ned_rotation",
    )
    mag_sensor_to_flu = _proper_rotation(
        calibration.get("mag_sensor_to_flu_rotation"),
        "calibration.mag_sensor_to_flu_rotation",
    )
    mag_intrinsic_transform = np.asarray(
        calibration.get("mag_intrinsic_transform"), dtype=np.float64
    )
    if mag_intrinsic_transform.shape != (3, 3) or np.any(~np.isfinite(mag_intrinsic_transform)):
        raise ValueError("calibration.mag_intrinsic_transform must be a finite 3x3 matrix")
    if np.linalg.cond(mag_intrinsic_transform) > 10.0:
        raise ValueError("calibration.mag_intrinsic_transform is ill-conditioned")
    mag_intrinsic_offset_t = _finite_vector(
        calibration.get("mag_intrinsic_offset_t"), 3,
        "calibration.mag_intrinsic_offset_t",
    )
    magnetic_declination_rad = _finite_scalar(calibration, "magnetic_declination_rad")
    if abs(magnetic_declination_rad) > math.pi:
        raise ValueError("magnetic_declination_rad must be in [-pi, pi]")
    yaw_datum = str(calibration.get("yaw_datum", ""))
    if yaw_datum not in {
        "local_optitrack", "calibrated_magnetic_reference", "surveyed_true_north"
    }:
        raise ValueError(
            "calibration.yaw_datum must be local_optitrack, "
            "calibrated_magnetic_reference, or surveyed_true_north"
        )

    audit_config = manifest["audit"]
    maximum_mocap_gap_s = _finite_scalar(audit_config, "max_mocap_gap_s")
    maximum_mag_residual_s = _finite_scalar(audit_config, "max_mag_time_residual_s")
    angular_rate_window_s = _finite_scalar(audit_config, "angular_rate_window_s")
    maximum_angular_rmse = _finite_scalar(
        audit_config, "max_angular_rate_rmse_rad_s"
    )
    minimum_norm_correlation = _finite_scalar(
        audit_config, "min_angular_rate_norm_correlation"
    )
    maximum_gravity_error_deg = _finite_scalar(
        audit_config, "max_gravity_direction_error_deg"
    )
    minimum_audit_samples = int(audit_config.get("min_audit_samples", 0))
    minimum_static_samples = int(audit_config.get("min_static_samples", 0))
    if not 0.0 < maximum_mocap_gap_s <= 0.5:
        raise ValueError("max_mocap_gap_s must be in (0, 0.5]")
    if not 0.0 < maximum_mag_residual_s <= 0.1:
        raise ValueError("max_mag_time_residual_s must be in (0, 0.1]")
    if not 0.005 <= angular_rate_window_s <= 0.25:
        raise ValueError("angular_rate_window_s must be in [0.005, 0.25]")
    if not 0.0 < maximum_angular_rmse <= 1.0:
        raise ValueError("max_angular_rate_rmse_rad_s must be in (0, 1]")
    if not 0.0 <= minimum_norm_correlation <= 1.0:
        raise ValueError("min_angular_rate_norm_correlation must be in [0, 1]")
    if not 0.0 < maximum_gravity_error_deg <= 45.0:
        raise ValueError("max_gravity_direction_error_deg must be in (0, 45]")
    if minimum_audit_samples < 10 or minimum_static_samples < 1:
        raise ValueError("audit sample minima are too small")

    imu_time_s = np.asarray(imu["t"], dtype=np.float64)
    mag_time_s = np.asarray(mag["t"], dtype=np.float64)
    mocap_time_s = np.asarray(mocap["t"], dtype=np.float64) + mocap_time_offset_s
    overlap_start_s = max(imu_time_s[0], mag_time_s[0], mocap_time_s[0])
    overlap_stop_s = min(imu_time_s[-1], mag_time_s[-1], mocap_time_s[-1])
    if overlap_stop_s <= overlap_start_s:
        raise ValueError("IMU, magnetometer, and OptiTrack streams do not overlap")
    split_name = str(manifest["split"]["name"])
    split_start_s, split_stop_s = _window(manifest["split"]["window_s"], "split.window_s")
    absolute_start_s = overlap_start_s + split_start_s
    absolute_stop_s = overlap_start_s + split_stop_s
    if absolute_stop_s > overlap_stop_s + 1.0e-9:
        raise ValueError("selected split exceeds the common source overlap")
    selected = (imu_time_s >= absolute_start_s) & (imu_time_s < absolute_stop_s)
    if np.count_nonzero(selected) < minimum_audit_samples + 2:
        raise ValueError("selected split has too few IMU samples")
    imu_time_s = imu_time_s[selected]

    acceleration_flu = np.column_stack((imu["a_x"], imu["a_y"], imu["a_z"]))[selected]
    angular_rate_flu = np.column_stack((imu["w_x"], imu["w_y"], imu["w_z"]))[selected]
    acceleration_frd = acceleration_flu @ FLU_TO_FRD.T
    angular_rate_frd = angular_rate_flu @ FLU_TO_FRD.T

    raw_mocap_quaternion = np.column_stack(
        (mocap["q_w"], mocap["q_x"], mocap["q_y"], mocap["q_z"])
    )
    raw_quaternion_norm = np.linalg.norm(raw_mocap_quaternion, axis=1)
    mocap_quaternion, mocap_bracket_gap_s = _interpolate_quaternions(
        mocap_time_s, raw_mocap_quaternion, imu_time_s
    )
    if float(np.max(mocap_bracket_gap_s)) > maximum_mocap_gap_s:
        raise ValueError(
            f"OptiTrack bracket gap {np.max(mocap_bracket_gap_s):.6f}s exceeds "
            f"{maximum_mocap_gap_s:.6f}s"
        )
    mocap_rotation = _rotation_from_quaternion(mocap_quaternion)
    reference_rotation = np.asarray(
        [mocap_world_to_ned @ rotation @ mocap_body_rotation @ FLU_TO_FRD
         for rotation in mocap_rotation]
    )
    reference_quaternion = _quaternion_from_rotation(reference_rotation)
    reference_euler_deg = _quaternion_to_euler_deg(reference_quaternion)
    mocap_position = np.column_stack((mocap["p_x"], mocap["p_y"], mocap["p_z"]))
    reference_position = _interpolate_vectors(mocap_time_s, mocap_position, imu_time_s)
    reference_position = (reference_position - reference_position[0]) @ mocap_world_to_ned.T
    reference_velocity = np.gradient(reference_position, imu_time_s, axis=0)

    selected_mag = (mag_time_s >= imu_time_s[0]) & (mag_time_s <= imu_time_s[-1])
    if np.count_nonzero(selected_mag) < 2:
        raise ValueError("selected split has too few raw magnetometer samples")
    mag_time_selected_s = mag_time_s[selected_mag]
    magnetometer_sensor_t = np.column_stack(
        (mag["cart_x"], mag["cart_y"], mag["cart_z"])
    )[selected_mag]
    magnetometer_sensor_t = (
        magnetometer_sensor_t - mag_intrinsic_offset_t
    ) @ mag_intrinsic_transform.T
    magnetometer_flu_t = magnetometer_sensor_t @ mag_sensor_to_flu.T
    magnetometer_frd_t = magnetometer_flu_t @ FLU_TO_FRD.T
    mapped_magnetometer_t, mag_valid, mag_update, mag_residual_s = _map_magnetometer(
        imu_time_s, mag_time_selected_s, magnetometer_frd_t, maximum_mag_residual_s
    )

    angular_audit = _angular_rate_audit(
        imu_time_s, reference_quaternion, angular_rate_frd, angular_rate_window_s
    )
    if int(angular_audit["samples"]) < minimum_audit_samples:
        raise ValueError("insufficient angular-rate audit coverage")
    if float(angular_audit["total_rmse_rad_s"]) > maximum_angular_rmse:
        raise ValueError("OptiTrack/IMU angular-rate frame audit exceeded RMSE limit")
    if float(angular_audit["norm_correlation"]) < minimum_norm_correlation:
        raise ValueError("OptiTrack/IMU angular-rate frame audit correlation is too low")
    gravity_audit = _gravity_audit(
        acceleration_frd, angular_rate_frd, reference_quaternion
    )
    if int(gravity_audit["samples"]) < minimum_static_samples:
        raise ValueError("insufficient quasi-static samples for gravity/frame audit")
    if float(gravity_audit["median_direction_error_deg"]) > maximum_gravity_error_deg:
        raise ValueError("OptiTrack/IMU gravity direction audit exceeded limit")

    timestamp_us = np.rint((imu_time_s - imu_time_s[0]) * 1.0e6).astype(np.int64)
    if np.any(np.diff(timestamp_us) <= 0):
        raise ValueError("microsecond replay timestamps are not strictly increasing")
    static_hint_duration_s = float(manifest.get("static_hint_duration_s", 0.0))
    if not math.isfinite(static_hint_duration_s) or static_hint_duration_s < 0.0:
        raise ValueError("static_hint_duration_s must be finite and non-negative")
    static_hint = (timestamp_us * 1.0e-6 < static_hint_duration_s).astype(np.int64)

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(REPLAY_HEADER)
        for index in range(len(timestamp_us)):
            writer.writerow(
                [
                    index, int(timestamp_us[index]),
                    *np.rint(acceleration_frd[index] / GRAVITY_M_S2 * 1000.0).astype(int),
                    *np.rint(np.degrees(angular_rate_frd[index]) * 1000.0).astype(int),
                    *np.rint(mapped_magnetometer_t[index] * 1.0e8).astype(int),
                    int(mag_valid[index]), int(mag_update[index]), magnetic_declination_rad,
                    *np.rint(reference_euler_deg[index] * 1000.0).astype(int),
                    *reference_quaternion[index], 0,
                    *reference_position[index], *reference_velocity[index],
                    0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0,
                    int(static_hint[index]), 0, 0, 0.0, math.radians(1.0) ** 2, 0,
                ]
            )

    evidence_class = "A" if yaw_datum == "surveyed_true_north" else (
        "A_candidate_calibrated_magnetic_yaw"
        if yaw_datum == "calibrated_magnetic_reference"
        else "A_candidate_local_yaw"
    )
    metadata: dict[str, object] = {
        "schema_version": "aerakia.insane_mocap_replay.v1",
        "dataset": "INSANE",
        "sequence": str(manifest.get("sequence", sequence_dir.name)),
        "split": {
            "name": split_name,
            "window_s": [split_start_s, split_stop_s],
            "origin": "seconds from the beginning of common IMU/magnetometer/OptiTrack overlap",
        },
        "reference_kind": "independent_optitrack_attitude",
        "evidence_class": evidence_class,
        "source_independence": {
            "estimator_heading_input": "recorded raw physical magnetometer",
            "scoring_reference": "recorded external OptiTrack/VRPN vehicle pose",
            "generated_ground_truth_used": False,
            "gnss_or_reference_derived_heading_used": False,
        },
        "input_sha256": {
            "px4_imu.csv": _sha256(imu_path),
            "px4_mag.csv": _sha256(mag_path),
            "mocap_vehicle_data.csv": _sha256(mocap_path),
            "manifest": _sha256(manifest_path),
        },
        "output_sha256": _sha256(output_path),
        "samples": {
            "raw_imu": int(len(imu)),
            "raw_magnetometer": int(len(mag)),
            "raw_optitrack": int(len(mocap)),
            "replay_imu": int(len(timestamp_us)),
            "mapped_magnetometer_updates": int(np.count_nonzero(mag_update)),
        },
        "duration_s": float(timestamp_us[-1]) * 1.0e-6,
        "source_rates_hz": {
            "imu": float(1.0 / np.median(np.diff(imu["t"]))),
            "magnetometer": float(1.0 / np.median(np.diff(mag["t"]))),
            "optitrack": float(1.0 / np.median(np.diff(mocap["t"]))),
        },
        "calibration": {
            "source_sequence": calibration["source_sequence"],
            "source_split": calibration["source_split"],
            "source_window_s": calibration.get("source_window_s"),
            "provenance": calibration["provenance"],
            "mocap_time_offset_s": mocap_time_offset_s,
            "mocap_body_rotation": mocap_body_rotation.tolist(),
            "mocap_world_to_ned_rotation": mocap_world_to_ned.tolist(),
            "mag_sensor_to_flu_rotation": mag_sensor_to_flu.tolist(),
            "mag_intrinsic_transform": mag_intrinsic_transform.tolist(),
            "mag_intrinsic_offset_t": mag_intrinsic_offset_t.tolist(),
            "magnetic_declination_rad": magnetic_declination_rad,
            "yaw_datum": yaw_datum,
        },
        "frame_time_audit": {
            "raw_optitrack_quaternion_norm_min": float(np.min(raw_quaternion_norm)),
            "raw_optitrack_quaternion_norm_max": float(np.max(raw_quaternion_norm)),
            "maximum_optitrack_bracket_gap_s": float(np.max(mocap_bracket_gap_s)),
            "maximum_magnetometer_mapping_residual_s": float(
                np.nanmax(mag_residual_s)
            ),
            "angular_rate": angular_audit,
            "gravity_direction": gravity_audit,
        },
        "limitations": [
            "OptiTrack is an independent local optical yaw reference, but local_optitrack is not surveyed true North.",
            "The public preprocessed package does not supply a reviewed PX4-IMU-to-mocap rigid-body calibration; the manifest calibration must be audited separately.",
            "OptiTrack position is exported for inspection but position_ref_valid is false because the rigid-body lever arm is not established.",
            "The replay contains no GNSS heading; absolute-yaw behavior is driven only by the recorded magnetometer.",
            "INSANE data terms add a no-commercial-use condition and do not grant a patent license.",
        ],
    }
    if metadata_path is not None:
        metadata_path = Path(metadata_path)
        metadata_path.parent.mkdir(parents=True, exist_ok=True)
        metadata_path.write_text(
            json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
    return metadata


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sequence_dir", type=Path)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--metadata", type=Path)
    args = parser.parse_args()
    metadata = convert_insane_mocap(
        args.sequence_dir, args.manifest, args.out, args.metadata
    )
    print(json.dumps(metadata, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
