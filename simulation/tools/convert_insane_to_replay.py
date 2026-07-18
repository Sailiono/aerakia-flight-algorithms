#!/usr/bin/env python3
"""Convert an INSANE outdoor sensor package into Aerakia's replay contract.

The converter keeps the evidence classes separate: PX4 IMU samples are recorded inputs,
dual-RTK baseline heading is a recorded physical observation, and the published 80 Hz pose is the
reference. The published pose and heading share the RTK baseline, so the resulting yaw score is not
an independent sensor-vs-truth accuracy claim.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
from pathlib import Path

import numpy as np

GRAVITY_M_S2 = 9.80665
ENU_TO_NED = np.asarray([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
FLU_TO_FRD = np.diag([1.0, -1.0, -1.0])


def _load_named(path: Path) -> np.ndarray:
    data = np.genfromtxt(path, delimiter=",", names=True, dtype=np.float64)
    if data.size == 0:
        raise ValueError(f"empty CSV: {path}")
    return np.atleast_1d(data)


def _normalize_quaternions(quaternions: np.ndarray) -> np.ndarray:
    result = np.asarray(quaternions, dtype=np.float64).copy()
    norms = np.linalg.norm(result, axis=1)
    if np.any(~np.isfinite(norms)) or np.any(norms < 1.0e-9):
        raise ValueError("reference quaternion stream contains invalid values")
    result /= norms[:, None]
    for index in range(1, len(result)):
        if np.dot(result[index - 1], result[index]) < 0.0:
            result[index] *= -1.0
    return result


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


def _quaternion_to_euler_deg(quaternion: np.ndarray) -> np.ndarray:
    w, x, y, z = quaternion.T
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return np.degrees(np.column_stack((roll, pitch, yaw)))


def _quaternion_from_rotation_matrix(matrix: np.ndarray) -> np.ndarray:
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        result = np.asarray(
            [0.25 * scale, (matrix[2, 1] - matrix[1, 2]) / scale,
             (matrix[0, 2] - matrix[2, 0]) / scale,
             (matrix[1, 0] - matrix[0, 1]) / scale]
        )
    else:
        axis = int(np.argmax(np.diag(matrix)))
        if axis == 0:
            scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            result = np.asarray([(matrix[2, 1] - matrix[1, 2]) / scale, 0.25 * scale,
                                 (matrix[0, 1] + matrix[1, 0]) / scale,
                                 (matrix[0, 2] + matrix[2, 0]) / scale])
        elif axis == 1:
            scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            result = np.asarray([(matrix[0, 2] - matrix[2, 0]) / scale,
                                 (matrix[0, 1] + matrix[1, 0]) / scale, 0.25 * scale,
                                 (matrix[1, 2] + matrix[2, 1]) / scale])
        else:
            scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            result = np.asarray([(matrix[1, 0] - matrix[0, 1]) / scale,
                                 (matrix[0, 2] + matrix[2, 0]) / scale,
                                 (matrix[1, 2] + matrix[2, 1]) / scale, 0.25 * scale])
    return result / np.linalg.norm(result)


def _interpolate_quaternions(t_source: np.ndarray, source: np.ndarray, query: np.ndarray) -> np.ndarray:
    source = _normalize_quaternions(source)
    right = np.searchsorted(t_source, query, side="right")
    left = np.clip(right - 1, 0, len(source) - 1)
    right = np.clip(right, 0, len(source) - 1)
    result = np.empty((len(query), 4), dtype=np.float64)
    for index, timestamp in enumerate(query):
        low, high = int(left[index]), int(right[index])
        if low == high:
            result[index] = source[low]
        else:
            alpha = float(timestamp - t_source[low]) / float(t_source[high] - t_source[low])
            candidate = (1.0 - alpha) * source[low] + alpha * source[high]
            result[index] = candidate / np.linalg.norm(candidate)
    return _normalize_quaternions(result)


def _read_imu_truth_offset(path: Path) -> float:
    match = re.search(r"^t_pximu_imugt:\s*([-+0-9.eE]+)", path.read_text(), re.MULTILINE)
    if match is None:
        raise ValueError(f"missing t_pximu_imugt in {path}")
    return float(match.group(1))


def _pair_dual_rtk(first: np.ndarray, second: np.ndarray, tolerance_s: float) -> tuple[np.ndarray, np.ndarray]:
    right = np.searchsorted(second["t_gps"], first["t_gps"])
    pairs: list[tuple[int, int]] = []
    for first_index, candidate in enumerate(right):
        choices = [index for index in (candidate - 1, candidate) if 0 <= index < len(second)]
        if not choices:
            continue
        second_index = min(
            choices, key=lambda index: abs(second["t_gps"][index] - first["t_gps"][first_index])
        )
        if abs(second["t_gps"][second_index] - first["t_gps"][first_index]) <= tolerance_s:
            if first["gps_fix_type"][first_index] == 3 and second["gps_fix_type"][second_index] == 3:
                pairs.append((first_index, second_index))
    if not pairs:
        raise ValueError("no synchronized dual-RTK fixed pairs")
    return np.asarray([pair[0] for pair in pairs]), np.asarray([pair[1] for pair in pairs])


def convert_insane(
    sequence_dir: Path,
    output_path: Path,
    metadata_path: Path | None = None,
    baseline_body_azimuth_deg: float = -135.0,
    heading_sigma_deg: float = 1.0,
    stationary_prefix_s: float = 5.0,
) -> dict:
    sequence_dir = Path(sequence_dir)
    imu = _load_named(sequence_dir / "px4_imu.csv")
    truth = _load_named(sequence_dir / "ground_truth" / "ground_truth_80hz.csv")
    rtk1 = _load_named(sequence_dir / "rtk_gps1.csv")
    rtk2 = _load_named(sequence_dir / "rtk_gps2.csv")
    imu_truth_offset_s = _read_imu_truth_offset(sequence_dir / "time_info.yaml")

    for stream, name in ((imu, "IMU"), (truth, "truth"), (rtk1, "RTK1"), (rtk2, "RTK2")):
        if np.any(np.diff(stream["t"]) <= 0.0):
            raise ValueError(f"{name} timestamps are not strictly increasing")

    overlap = (imu["t"] >= truth["t"][0]) & (imu["t"] <= truth["t"][-1])
    imu = imu[overlap]
    if len(imu) < 2:
        raise ValueError("IMU and reference pose do not overlap")
    timestamp_s = imu["t"]
    timestamp_us = np.rint((timestamp_s - timestamp_s[0]) * 1.0e6).astype(np.int64)

    acceleration_frd = np.column_stack((imu["a_x"], -imu["a_y"], -imu["a_z"]))
    angular_rate_frd = np.column_stack((imu["w_x"], -imu["w_y"], -imu["w_z"]))
    source_quaternion = np.column_stack(
        (truth["q_w"], truth["q_x"], truth["q_y"], truth["q_z"])
    )
    source_quaternion = _interpolate_quaternions(truth["t"], source_quaternion, timestamp_s)
    source_rotation = _rotation_from_quaternion(source_quaternion)
    reference_quaternion = np.asarray(
        [_quaternion_from_rotation_matrix(ENU_TO_NED @ rotation @ FLU_TO_FRD)
         for rotation in source_rotation]
    )
    reference_euler_deg = _quaternion_to_euler_deg(reference_quaternion)
    reference_position = np.column_stack((truth["p_y"], truth["p_x"], -truth["p_z"]))
    position_ned = np.column_stack(
        [np.interp(timestamp_s, truth["t"], reference_position[:, axis]) for axis in range(3)]
    )
    velocity_ned = np.gradient(position_ned, timestamp_s, axis=0)

    first_index, second_index = _pair_dual_rtk(rtk1, rtk2, 0.002)
    heading_time_s = 0.5 * (rtk1["t"][first_index] + rtk2["t"][second_index]) \
        - imu_truth_offset_s
    baseline_enu = np.column_stack(
        [rtk2[name][second_index] - rtk1[name][first_index] for name in ("p_x", "p_y", "p_z")]
    )
    baseline_norm = np.linalg.norm(baseline_enu, axis=1)
    horizontal_norm = np.linalg.norm(baseline_enu[:, :2], axis=1)
    valid = np.isfinite(baseline_norm) & (baseline_norm > 0.9) & (baseline_norm < 1.4) \
        & (horizontal_norm / baseline_norm > 0.8)
    heading_time_s = heading_time_s[valid]
    baseline_enu = baseline_enu[valid]
    baseline_norm = baseline_norm[valid]
    baseline_world_azimuth = np.arctan2(baseline_enu[:, 1], baseline_enu[:, 0])
    body_yaw_enu = baseline_world_azimuth - math.radians(baseline_body_azimuth_deg)
    heading_ned = (0.5 * math.pi - body_yaw_enu + math.pi) % (2.0 * math.pi) - math.pi

    heading_valid = np.zeros(len(imu), dtype=np.int64)
    heading_update = np.zeros(len(imu), dtype=np.int64)
    heading_value = np.zeros(len(imu), dtype=np.float64)
    heading_indices = np.searchsorted(timestamp_s, heading_time_s)
    for source_index, row_index in enumerate(heading_indices):
        if row_index >= len(imu) or heading_time_s[source_index] < timestamp_s[0]:
            continue
        if heading_update[row_index] != 0:
            continue
        heading_valid[row_index] = 1
        heading_update[row_index] = 1
        heading_value[row_index] = heading_ned[source_index]

    static_hint = ((timestamp_s - timestamp_s[0]) < stationary_prefix_s).astype(np.int64)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    header = [
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
    with output_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        for index in range(len(imu)):
            writer.writerow([
                index, int(timestamp_us[index]),
                *np.rint(acceleration_frd[index] / GRAVITY_M_S2 * 1000.0).astype(int),
                *np.rint(np.degrees(angular_rate_frd[index]) * 1000.0).astype(int),
                0, 0, 0, 0, 0, 0.0,
                *np.rint(reference_euler_deg[index] * 1000.0).astype(int),
                *reference_quaternion[index], 0,
                *position_ned[index], *velocity_ned[index], 0,
                *position_ned[index], *velocity_ned[index], 1.0, 0.04,
                int(static_hint[index]), int(heading_valid[index]), int(heading_update[index]),
                heading_value[index], math.radians(heading_sigma_deg) ** 2, 0,
            ])

    metadata = {
        "dataset": "INSANE",
        "sequence": sequence_dir.name,
        "reference_kind": "dual_rtk_and_magnetometer_fused_pose",
        "recorded_imu_samples": int(len(imu)),
        "duration_s": float(timestamp_s[-1] - timestamp_s[0]),
        "recorded_dual_rtk_heading_updates": int(np.sum(heading_update)),
        "dual_rtk_pairs_before_geometry_gate": int(len(first_index)),
        "baseline_norm_median_m": float(np.median(baseline_norm)),
        "baseline_norm_std_m": float(np.std(baseline_norm)),
        "imu_truth_time_offset_s": imu_truth_offset_s,
        "baseline_body_azimuth_deg": baseline_body_azimuth_deg,
        "heading_sigma_deg": heading_sigma_deg,
        "stationary_prefix_s": stationary_prefix_s,
        "frame_mapping": {
            "source_navigation": "ENU",
            "target_navigation": "NED",
            "source_body": "FLU/PX4 MAVROS body convention",
            "target_body": "FRD",
        },
        "limitations": [
            "The published pose and direct heading both use the same dual-RTK baseline; yaw scoring is not independent sensor-vs-truth accuracy.",
            "The published full pose failed the initial gravity/accelerometer consistency check; use this intake for yaw evidence until the tilt-frame discrepancy is resolved.",
            "No GNSS position or velocity is fused in this heading-isolation replay.",
        ],
    }
    if metadata_path is not None:
        metadata_path.parent.mkdir(parents=True, exist_ok=True)
        metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    return metadata


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("sequence_dir", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--metadata", type=Path)
    parser.add_argument("--baseline-body-azimuth-deg", type=float, default=-135.0)
    parser.add_argument("--heading-sigma-deg", type=float, default=1.0)
    parser.add_argument("--stationary-prefix-s", type=float, default=5.0)
    args = parser.parse_args()
    metadata = convert_insane(
        args.sequence_dir, args.out, args.metadata, args.baseline_body_azimuth_deg,
        args.heading_sigma_deg, args.stationary_prefix_s,
    )
    print(json.dumps(metadata, indent=2))


if __name__ == "__main__":
    main()
