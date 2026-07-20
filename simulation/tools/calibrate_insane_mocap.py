#!/usr/bin/env python3
"""Estimate INSANE OptiTrack/IMU calibration using only a declared calibration split."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Any

import numpy as np

import convert_insane_mocap_to_replay as replay


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _mean_interval_rate(
    timestamp_s: np.ndarray, angular_rate: np.ndarray, lag: int
) -> np.ndarray:
    delta_s = np.diff(timestamp_s)
    trapezoid = 0.5 * (angular_rate[:-1] + angular_rate[1:]) * delta_s[:, None]
    integral = np.vstack((np.zeros((1, 3)), np.cumsum(trapezoid, axis=0)))
    interval_s = timestamp_s[lag:] - timestamp_s[:-lag]
    return (integral[lag:] - integral[:-lag]) / interval_s[:, None]


def _quaternion_interval_rate(
    timestamp_s: np.ndarray, quaternion: np.ndarray, lag: int
) -> np.ndarray:
    first = quaternion[:-lag].copy()
    first[:, 1:] *= -1.0
    relative = replay._quaternion_multiply(first, quaternion[lag:])
    relative[relative[:, 0] < 0.0] *= -1.0
    vector_norm = np.linalg.norm(relative[:, 1:], axis=1)
    angle = 2.0 * np.arctan2(vector_norm, np.clip(relative[:, 0], -1.0, 1.0))
    rotation_vector = np.zeros((len(relative), 3), dtype=np.float64)
    nonzero = vector_norm > 1.0e-12
    rotation_vector[nonzero] = (
        relative[nonzero, 1:] / vector_norm[nonzero, None] * angle[nonzero, None]
    )
    return rotation_vector / (timestamp_s[lag:] - timestamp_s[:-lag])[:, None]


def _rotation_angle_deg(rotation: np.ndarray) -> float:
    cosine = np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0)
    return math.degrees(math.acos(float(cosine)))


def _robust_wahba(
    source: np.ndarray, target: np.ndarray, iterations: int = 8
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    if source.shape != target.shape or source.ndim != 2 or source.shape[1] != 3:
        raise ValueError("Wahba inputs must be matching Nx3 arrays")
    if len(source) < 20:
        raise ValueError("insufficient samples for robust Wahba calibration")
    source_center = np.median(source, axis=0)
    target_center = np.median(target, axis=0)
    source_zero = source - source_center
    target_zero = target - target_center
    weights = np.ones(len(source), dtype=np.float64)
    singular_values = np.zeros(3)
    rotation = np.eye(3)
    for _ in range(iterations):
        covariance = source_zero.T @ (weights[:, None] * target_zero)
        left, singular_values, right = np.linalg.svd(covariance)
        rotation = right.T @ left.T
        if np.linalg.det(rotation) < 0.0:
            right[-1] *= -1.0
            rotation = right.T @ left.T
        residual = np.linalg.norm(target_zero - source_zero @ rotation.T, axis=1)
        scale = 1.4826 * np.median(np.abs(residual - np.median(residual))) + 1.0e-9
        cutoff = 1.5 * scale
        weights = np.minimum(1.0, cutoff / np.maximum(residual, 1.0e-12))
    return rotation, weights, singular_values


def _fit_offset(
    offset_s: float,
    imu_time_s: np.ndarray,
    angular_rate_flu: np.ndarray,
    mocap_time_s: np.ndarray,
    mocap_quaternion: np.ndarray,
    angular_window_s: float,
    minimum_rate_rad_s: float,
    maximum_rate_rad_s: float,
) -> dict[str, Any]:
    interpolated, bracket_gap = replay._interpolate_quaternions(
        mocap_time_s + offset_s, mocap_quaternion, imu_time_s
    )
    lag = max(1, int(round(angular_window_s / np.median(np.diff(imu_time_s)))))
    mocap_rate = _quaternion_interval_rate(imu_time_s, interpolated, lag)
    imu_rate = _mean_interval_rate(imu_time_s, angular_rate_flu, lag)
    imu_norm = np.linalg.norm(imu_rate, axis=1)
    mocap_norm = np.linalg.norm(mocap_rate, axis=1)
    usable = (
        (imu_norm >= minimum_rate_rad_s)
        & (imu_norm <= maximum_rate_rad_s)
        & (mocap_norm <= maximum_rate_rad_s)
    )
    imu_rate = imu_rate[usable]
    mocap_rate = mocap_rate[usable]
    if len(imu_rate) < 500:
        raise ValueError("insufficient angular excitation samples")
    rotation, weights, singular_values = _robust_wahba(imu_rate, mocap_rate)
    imu_center = imu_rate - np.median(imu_rate, axis=0)
    mocap_center = mocap_rate - np.median(mocap_rate, axis=0)
    predicted = imu_center @ rotation.T
    error = mocap_center - predicted
    weighted_rmse = math.sqrt(
        float(np.sum(weights * np.sum(error * error, axis=1)) / (3.0 * np.sum(weights)))
    )
    norm_correlation = float(
        np.corrcoef(np.linalg.norm(mocap_center, axis=1), np.linalg.norm(predicted, axis=1))[0, 1]
    )
    return {
        "offset_s": offset_s,
        "rotation": rotation,
        "weights": weights,
        "singular_values": singular_values,
        "samples": len(error),
        "weighted_rmse_rad_s": weighted_rmse,
        "residual_median_rad_s": float(np.median(np.linalg.norm(error, axis=1))),
        "residual_p95_rad_s": float(np.percentile(np.linalg.norm(error, axis=1), 95.0)),
        "norm_correlation": norm_correlation,
        "maximum_mocap_bracket_gap_s": float(np.max(bracket_gap)),
        "source_rate": imu_rate,
        "target_rate": mocap_rate,
    }


def _robust_unit_vector(vectors: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    normalized = vectors / np.linalg.norm(vectors, axis=1)[:, None]
    estimate = np.median(normalized, axis=0)
    estimate /= np.linalg.norm(estimate)
    for _ in range(8):
        error = np.degrees(
            np.arccos(np.clip(normalized @ estimate, -1.0, 1.0))
        )
        limit = max(2.0, float(np.percentile(error, 80.0)))
        weights = np.minimum(1.0, limit / np.maximum(error, 1.0e-9))
        estimate = np.sum(weights[:, None] * normalized, axis=0)
        estimate /= np.linalg.norm(estimate)
    error = np.degrees(np.arccos(np.clip(normalized @ estimate, -1.0, 1.0)))
    return estimate, error


def _triad(up: np.ndarray, magnetic: np.ndarray) -> np.ndarray:
    third = up / np.linalg.norm(up)
    first = magnetic - third * np.dot(third, magnetic)
    if np.linalg.norm(first) < 0.15:
        raise ValueError("gravity and magnetic reference are nearly collinear")
    first /= np.linalg.norm(first)
    second = np.cross(third, first)
    return np.column_stack((first, second, third))


def _estimate_world_datum(
    imu: np.ndarray,
    mag: np.ndarray,
    mocap: np.ndarray,
    calibration: dict[str, Any],
    calibration_absolute_window_s: tuple[float, float],
    mocap_time_offset_s: float,
    mocap_body_rotation: np.ndarray,
) -> tuple[np.ndarray, dict[str, Any]]:
    start_s, stop_s = calibration_absolute_window_s
    imu_time_s = np.asarray(imu["t"])
    mag_time_s = np.asarray(mag["t"])
    mocap_time_s = np.asarray(mocap["t"]) + mocap_time_offset_s
    mocap_quaternion = np.column_stack(
        (mocap["q_w"], mocap["q_x"], mocap["q_y"], mocap["q_z"])
    )

    imu_selected = (imu_time_s >= start_s) & (imu_time_s < stop_s)
    selected_imu_time = imu_time_s[imu_selected]
    imu_quaternion, _ = replay._interpolate_quaternions(
        mocap_time_s, mocap_quaternion, selected_imu_time
    )
    rotation_world_mocap = replay._rotation_from_quaternion(imu_quaternion)
    rotation_world_flu = rotation_world_mocap @ mocap_body_rotation
    acceleration_flu = np.column_stack((imu["a_x"], imu["a_y"], imu["a_z"]))[
        imu_selected
    ]
    angular_rate_flu = np.column_stack((imu["w_x"], imu["w_y"], imu["w_z"]))[
        imu_selected
    ]
    quasi_static = (np.linalg.norm(angular_rate_flu, axis=1) < 0.08) & (
        np.abs(np.linalg.norm(acceleration_flu, axis=1) - replay.GRAVITY_M_S2) < 0.75
    )
    if np.count_nonzero(quasi_static) < 100:
        raise ValueError("insufficient quasi-static gravity samples in calibration split")
    gravity_world = np.einsum(
        "nij,nj->ni", rotation_world_flu[quasi_static], acceleration_flu[quasi_static]
    )
    gravity_direction, gravity_error_deg = _robust_unit_vector(gravity_world)

    mag_selected = (mag_time_s >= start_s) & (mag_time_s < stop_s)
    selected_mag_time = mag_time_s[mag_selected]
    mag_quaternion, _ = replay._interpolate_quaternions(
        mocap_time_s, mocap_quaternion, selected_mag_time
    )
    rotation_world_mocap = replay._rotation_from_quaternion(mag_quaternion)
    rotation_world_flu = rotation_world_mocap @ mocap_body_rotation
    raw_magnetometer_t = np.column_stack(
        (mag["cart_x"], mag["cart_y"], mag["cart_z"])
    )[mag_selected]
    intrinsic = np.asarray(calibration["mag_intrinsic_transform"], dtype=np.float64)
    offset_t = np.asarray(calibration["mag_intrinsic_offset_t"], dtype=np.float64)
    sensor_to_flu = np.asarray(calibration["mag_sensor_to_flu_rotation"], dtype=np.float64)
    magnetometer_flu_t = (raw_magnetometer_t - offset_t) @ intrinsic.T @ sensor_to_flu.T
    magnetic_world = np.einsum("nij,nj->ni", rotation_world_flu, magnetometer_flu_t)
    magnetic_direction, magnetic_error_deg = _robust_unit_vector(magnetic_world)

    declination = float(calibration["magnetic_declination_rad"])
    inclination = float(calibration["magnetic_inclination_rad"])
    magnetic_ned = np.asarray(
        [
            math.cos(inclination) * math.cos(declination),
            math.cos(inclination) * math.sin(declination),
            -math.sin(inclination),
        ]
    )
    up_ned = np.asarray([0.0, 0.0, -1.0])
    rotation_ned_world = _triad(up_ned, magnetic_ned) @ _triad(
        gravity_direction, magnetic_direction
    ).T
    mapped_magnetic = rotation_ned_world @ magnetic_direction
    horizontal_target = magnetic_ned - up_ned * np.dot(up_ned, magnetic_ned)
    horizontal_mapped = mapped_magnetic - up_ned * np.dot(up_ned, mapped_magnetic)
    yaw_residual_deg = math.degrees(
        math.acos(
            float(
                np.clip(
                    np.dot(horizontal_target, horizontal_mapped)
                    / (np.linalg.norm(horizontal_target) * np.linalg.norm(horizontal_mapped)),
                    -1.0,
                    1.0,
                )
            )
        )
    )
    metrics = {
        "gravity_samples": int(len(gravity_error_deg)),
        "gravity_direction_mocap_world": gravity_direction.tolist(),
        "gravity_direction_error_median_deg": float(np.median(gravity_error_deg)),
        "gravity_direction_error_p95_deg": float(np.percentile(gravity_error_deg, 95.0)),
        "magnetometer_samples": int(len(magnetic_error_deg)),
        "magnetic_direction_mocap_world": magnetic_direction.tolist(),
        "magnetic_direction_error_median_deg": float(np.median(magnetic_error_deg)),
        "magnetic_direction_error_p95_deg": float(np.percentile(magnetic_error_deg, 95.0)),
        "mapped_magnetic_direction_ned": mapped_magnetic.tolist(),
        "expected_magnetic_direction_ned": (magnetic_ned / np.linalg.norm(magnetic_ned)).tolist(),
        "horizontal_yaw_residual_deg": yaw_residual_deg,
    }
    if metrics["gravity_direction_error_p95_deg"] > 20.0:
        raise ValueError("calibration gravity direction dispersion is too large")
    if metrics["magnetic_direction_error_p95_deg"] > 30.0:
        raise ValueError("calibration magnetic direction dispersion is too large")
    if yaw_residual_deg > 0.1:
        raise ValueError("failed to align the local magnetic yaw datum")
    return replay._proper_rotation(rotation_ned_world, "estimated world datum"), metrics


def _block_rotation_uncertainty(
    source: np.ndarray, target: np.ndarray, reference: np.ndarray, blocks: int = 10
) -> dict[str, float]:
    angles = []
    for indices in np.array_split(np.arange(len(source)), blocks):
        if len(indices) < 20:
            continue
        rotation, _, _ = _robust_wahba(source[indices], target[indices])
        angles.append(_rotation_angle_deg(rotation @ reference.T))
    if len(angles) < 5:
        raise ValueError("insufficient blocks for rotation uncertainty estimate")
    return {
        "blocks": len(angles),
        "median_deg": float(np.median(angles)),
        "p95_deg": float(np.percentile(angles, 95.0)),
        "maximum_deg": float(np.max(angles)),
    }


def _write_manifest(path: Path, document: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(document, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def calibrate_insane_mocap(
    sequence_dir: Path,
    base_manifest_path: Path,
    calibration_window_s: tuple[float, float],
    output_calibration_manifest: Path,
    development_window_s: tuple[float, float] | None = None,
    output_development_manifest: Path | None = None,
    holdout_window_s: tuple[float, float] | None = None,
    output_holdout_manifest: Path | None = None,
    search_min_s: float = -0.1,
    search_max_s: float = 0.1,
    search_step_s: float = 0.002,
    angular_window_s: float = 0.05,
    minimum_rate_rad_s: float = 0.1,
    maximum_rate_rad_s: float = 4.0,
) -> dict[str, Any]:
    sequence_dir = Path(sequence_dir)
    base_manifest_path = Path(base_manifest_path)
    base_manifest = replay._load_manifest(base_manifest_path)
    calibration_window_s = replay._window(calibration_window_s, "calibration window")
    if development_window_s is not None:
        development_window_s = replay._window(development_window_s, "development window")
    if holdout_window_s is not None:
        holdout_window_s = replay._window(holdout_window_s, "holdout window")
    windows = [calibration_window_s]
    if development_window_s is not None:
        windows.append(development_window_s)
    if holdout_window_s is not None:
        windows.append(holdout_window_s)
    for index, first in enumerate(windows):
        for second in windows[index + 1:]:
            if replay._overlap(first, second):
                raise ValueError("calibration, development, and holdout windows must be disjoint")
    if (development_window_s is None) != (output_development_manifest is None):
        raise ValueError("development window and output path must be provided together")
    if (holdout_window_s is None) != (output_holdout_manifest is None):
        raise ValueError("holdout window and output path must be provided together")
    if not search_min_s < search_max_s or not 0.0001 <= search_step_s <= 0.01:
        raise ValueError("invalid time-offset search range or step")

    streams = base_manifest["streams"]
    imu_path = replay._source_path(sequence_dir, streams.get("imu"), "px4_imu.csv")
    mag_path = replay._source_path(sequence_dir, streams.get("magnetometer"), "px4_mag.csv")
    mocap_path = replay._source_path(
        sequence_dir, streams.get("reference"), "mocap_vehicle_data.csv"
    )
    imu = replay._load_named(
        imu_path, ("t", "a_x", "a_y", "a_z", "w_x", "w_y", "w_z")
    )
    mag = replay._load_named(mag_path, ("t", "cart_x", "cart_y", "cart_z"))
    mocap = replay._load_named(
        mocap_path, ("t", "p_x", "p_y", "p_z", "q_w", "q_x", "q_y", "q_z")
    )
    calibration = base_manifest["calibration"]
    if "magnetic_inclination_rad" not in calibration:
        raise ValueError("base manifest requires calibration.magnetic_inclination_rad")

    imu_time = np.asarray(imu["t"])
    mag_time = np.asarray(mag["t"])
    mocap_time = np.asarray(mocap["t"])
    overlap_origin = max(imu_time[0], mag_time[0], mocap_time[0])
    overlap_stop = min(imu_time[-1], mag_time[-1], mocap_time[-1])
    if max(window[1] for window in windows) > overlap_stop - overlap_origin:
        raise ValueError("declared split exceeds common raw-stream overlap")
    absolute_calibration_window = (
        overlap_origin + calibration_window_s[0],
        overlap_origin + calibration_window_s[1],
    )
    selected = (imu_time >= absolute_calibration_window[0]) & (
        imu_time < absolute_calibration_window[1]
    )
    calibration_imu_time = imu_time[selected]
    angular_rate_flu = np.column_stack((imu["w_x"], imu["w_y"], imu["w_z"]))[
        selected
    ]
    mocap_quaternion = np.column_stack(
        (mocap["q_w"], mocap["q_x"], mocap["q_y"], mocap["q_z"])
    )
    offsets = np.arange(
        search_min_s, search_max_s + 0.5 * search_step_s, search_step_s
    )
    candidates = [
        _fit_offset(
            float(offset), calibration_imu_time, angular_rate_flu, mocap_time,
            mocap_quaternion, angular_window_s, minimum_rate_rad_s, maximum_rate_rad_s,
        )
        for offset in offsets
    ]
    candidates.sort(key=lambda candidate: candidate["weighted_rmse_rad_s"])
    best = candidates[0]
    if abs(best["offset_s"] - search_min_s) < 0.5 * search_step_s or abs(
        best["offset_s"] - search_max_s
    ) < 0.5 * search_step_s:
        raise ValueError("time-offset optimum lies on the search boundary")
    if best["samples"] < 500:
        raise ValueError("insufficient angular excitation samples")
    singular_values = np.asarray(best["singular_values"])
    excitation_ratio = float(singular_values[-1] / singular_values[0])
    if excitation_ratio < 0.05:
        raise ValueError("calibration angular excitation is rank-deficient")
    if best["weighted_rmse_rad_s"] > 0.2 or best["norm_correlation"] < 0.7:
        raise ValueError("calibration angular-rate residual is not acceptable")
    one_percent = sorted(
        float(candidate["offset_s"])
        for candidate in candidates
        if candidate["weighted_rmse_rad_s"] <= 1.01 * best["weighted_rmse_rad_s"]
    )
    if one_percent[-1] - one_percent[0] > 0.05:
        raise ValueError("time-offset minimum is insufficiently identifiable")
    rotation_uncertainty = _block_rotation_uncertainty(
        np.asarray(best["source_rate"]), np.asarray(best["target_rate"]),
        np.asarray(best["rotation"]),
    )
    if rotation_uncertainty["p95_deg"] > 5.0:
        raise ValueError("rigid rotation uncertainty is too large")

    rotation_ned_world, datum_metrics = _estimate_world_datum(
        imu, mag, mocap, calibration, absolute_calibration_window,
        float(best["offset_s"]), np.asarray(best["rotation"]),
    )
    calibrated = json.loads(json.dumps(base_manifest))
    calibrated["split"] = {
        "name": "calibration", "window_s": list(calibration_window_s)
    }
    calibrated["calibration"].update(
        {
            "source_sequence": calibrated["sequence"],
            "source_split": "calibration",
            "source_window_s": list(calibration_window_s),
            "provenance": (
                "calibrate_insane_mocap.py; estimates use only the declared calibration "
                "window and never development or holdout samples"
            ),
            "mocap_time_offset_s": float(best["offset_s"]),
            "mocap_body_rotation": np.asarray(best["rotation"]).tolist(),
            "mocap_world_to_ned_rotation": rotation_ned_world.tolist(),
            "yaw_datum": "calibrated_magnetic_reference",
        }
    )
    calibration_report = {
        "tool_sha256": _sha256(Path(__file__)),
        "base_manifest_sha256": _sha256(base_manifest_path),
        "input_sha256": {
            "px4_imu.csv": _sha256(imu_path),
            "px4_mag.csv": _sha256(mag_path),
            "mocap_vehicle_data.csv": _sha256(mocap_path),
        },
        "calibration_window_s": list(calibration_window_s),
        "source_sequence": calibrated["sequence"],
        "holdout_samples_read_for_estimation": 0,
        "angular_rate": {
            "samples": int(best["samples"]),
            "weighted_rmse_rad_s": float(best["weighted_rmse_rad_s"]),
            "residual_median_rad_s": float(best["residual_median_rad_s"]),
            "residual_p95_rad_s": float(best["residual_p95_rad_s"]),
            "norm_correlation": float(best["norm_correlation"]),
            "singular_values": singular_values.tolist(),
            "excitation_minimum_to_maximum_ratio": excitation_ratio,
        },
        "time_offset": {
            "estimate_s": float(best["offset_s"]),
            "search_range_s": [search_min_s, search_max_s],
            "search_step_s": search_step_s,
            "one_percent_cost_interval_s": [one_percent[0], one_percent[-1]],
            "best_weighted_rmse_rad_s": float(best["weighted_rmse_rad_s"]),
            "second_best_weighted_rmse_rad_s": float(candidates[1]["weighted_rmse_rad_s"]),
        },
        "rigid_rotation": {
            "mocap_body_rotation": np.asarray(best["rotation"]).tolist(),
            "block_uncertainty": rotation_uncertainty,
        },
        "world_datum": datum_metrics,
        "negative_claims": [
            "This calibration split is not an accuracy holdout.",
            "The yaw datum is calibrated to the recorded local magnetic field, not independently surveyed true North.",
            "No generated INSANE ground-truth product is used.",
        ],
    }
    calibrated["calibration_estimation"] = calibration_report
    _write_manifest(Path(output_calibration_manifest), calibrated)
    if development_window_s is not None and output_development_manifest is not None:
        development = json.loads(json.dumps(calibrated))
        development["split"] = {
            "name": "development", "window_s": list(development_window_s)
        }
        _write_manifest(Path(output_development_manifest), development)
    if holdout_window_s is not None and output_holdout_manifest is not None:
        holdout = json.loads(json.dumps(calibrated))
        holdout["split"] = {"name": "holdout", "window_s": list(holdout_window_s)}
        _write_manifest(Path(output_holdout_manifest), holdout)
    return calibration_report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sequence_dir", type=Path)
    parser.add_argument("--base-manifest", type=Path, required=True)
    parser.add_argument("--calibration-window", type=float, nargs=2, required=True)
    parser.add_argument("--development-window", type=float, nargs=2)
    parser.add_argument("--holdout-window", type=float, nargs=2)
    parser.add_argument("--out-calibration-manifest", type=Path, required=True)
    parser.add_argument("--out-development-manifest", type=Path)
    parser.add_argument("--out-holdout-manifest", type=Path)
    parser.add_argument("--search-min-s", type=float, default=-0.1)
    parser.add_argument("--search-max-s", type=float, default=0.1)
    parser.add_argument("--search-step-s", type=float, default=0.002)
    args = parser.parse_args()
    report = calibrate_insane_mocap(
        args.sequence_dir,
        args.base_manifest,
        tuple(args.calibration_window),
        args.out_calibration_manifest,
        tuple(args.development_window) if args.development_window else None,
        args.out_development_manifest,
        tuple(args.holdout_window) if args.holdout_window else None,
        args.out_holdout_manifest,
        args.search_min_s,
        args.search_max_s,
        args.search_step_s,
    )
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
