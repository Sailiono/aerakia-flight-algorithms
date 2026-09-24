#!/usr/bin/env python3
"""Convert UrbanNav Hong Kong text/NMEA subsets into Aerakia replay CSV.

The converter uses recorded Xsens IMU samples, u-blox NMEA position fixes, and
SPAN-CPT post-processed navigation truth on their common UTC time base.  NMEA
velocity is deliberately not invented when the receiver leaves RMC/VTG course
blank; the replay exercises the position-only public observation path.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from datetime import datetime, timezone
from pathlib import Path

import numpy as np


GRAVITY_M_S2 = 9.80665
SOURCE_TO_FRD = np.asarray(
    [[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]], dtype=np.float64
)
ENU_TO_NED = SOURCE_TO_FRD.copy()
EXPECTED_FILENAMES = {
    "imu": "xsense_imu_medium_urban1_update.txt",
    "nmea": "UrbanNav-HK-Medium-Urban-1.ublox.f9p.nmea",
    "truth": "UrbanNav_TST_GT_raw.txt",
}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _load_whitespace_numeric(path: Path, skiprows: int, columns: int) -> np.ndarray:
    rows: list[list[float]] = []
    with path.open("r", encoding="utf-8", errors="strict") as stream:
        for line_number, line in enumerate(stream, start=1):
            if line_number <= skiprows or not line.strip():
                continue
            fields = line.split()
            if len(fields) != columns:
                raise ValueError(
                    f"{path.name}:{line_number}: expected {columns} columns, got {len(fields)}"
                )
            try:
                rows.append([float(field) for field in fields])
            except ValueError as error:
                raise ValueError(f"{path.name}:{line_number}: non-numeric field") from error
    result = np.asarray(rows, dtype=np.float64)
    if result.ndim != 2 or result.shape[1] != columns or len(result) < 2:
        raise ValueError(f"{path.name}: insufficient numeric data")
    if np.any(~np.isfinite(result)):
        raise ValueError(f"{path.name}: contains non-finite values")
    if np.any(np.diff(result[:, 0]) <= 0.0):
        raise ValueError(f"{path.name}: UTC timestamps are not strictly increasing")
    return result


def _nmea_checksum_valid(line: str) -> bool:
    if not line.startswith("$") or "*" not in line:
        return False
    payload, checksum = line[1:].rsplit("*", 1)
    value = 0
    for character in payload:
        value ^= ord(character)
    try:
        return value == int(checksum[:2], 16)
    except ValueError:
        return False


def _nmea_degrees(value: str, hemisphere: str, latitude: bool) -> float:
    width = 2 if latitude else 3
    if len(value) <= width:
        raise ValueError("invalid NMEA latitude/longitude")
    degrees = float(value[:width]) + float(value[width:]) / 60.0
    if hemisphere in ("S", "W"):
        degrees = -degrees
    elif hemisphere not in ("N", "E"):
        raise ValueError("invalid NMEA hemisphere")
    return degrees


def _utc_from_nmea(date_ddmmyy: str, time_hhmmss: str) -> float:
    if len(date_ddmmyy) != 6 or len(time_hhmmss) < 6:
        raise ValueError("invalid NMEA date/time")
    whole = time_hhmmss[:6]
    fraction = float("0" + time_hhmmss[6:]) if len(time_hhmmss) > 6 else 0.0
    parsed = datetime.strptime(date_ddmmyy + whole, "%d%m%y%H%M%S").replace(
        tzinfo=timezone.utc
    )
    return parsed.timestamp() + fraction


def _parse_nmea(path: Path) -> dict[str, np.ndarray]:
    epochs: dict[str, dict[str, object]] = {}
    current_date: str | None = None
    invalid_checksums = 0
    # The distributed receiver log contains a small number of non-ASCII/binary fragments between
    # otherwise valid NMEA lines. Latin-1 preserves bytes one-to-one so checksum validation can
    # reject those fragments explicitly instead of aborting or silently replacing characters.
    with path.open("r", encoding="latin-1", errors="strict") as stream:
        for line_number, raw_line in enumerate(stream, start=1):
            line = raw_line.strip()
            if not line:
                continue
            if not _nmea_checksum_valid(line):
                invalid_checksums += 1
                continue
            fields = line[1 : line.index("*")].split(",")
            message = fields[0][2:]
            if message == "RMC" and len(fields) >= 10 and fields[9]:
                current_date = fields[9]
            if message not in ("GGA", "GST") or len(fields) < 2 or not fields[1]:
                continue
            epoch = epochs.setdefault(fields[1], {})
            if message == "GGA":
                if len(fields) < 12 or not all(fields[index] for index in (2, 3, 4, 5, 6)):
                    continue
                epoch["gga"] = {
                    "latitude_deg": _nmea_degrees(fields[2], fields[3], True),
                    "longitude_deg": _nmea_degrees(fields[4], fields[5], False),
                    "quality": int(fields[6]),
                    "satellites": int(fields[7]) if fields[7] else 0,
                    "hdop": float(fields[8]) if fields[8] else math.nan,
                    "ellipsoidal_height_m": float(fields[9]) + float(fields[11]),
                    "line": line_number,
                    "date": current_date,
                }
            else:
                if len(fields) >= 9 and all(fields[index] for index in (6, 7, 8)):
                    epoch["gst"] = {
                        "latitude_sigma_m": float(fields[6]),
                        "longitude_sigma_m": float(fields[7]),
                        "altitude_sigma_m": float(fields[8]),
                    }

    records: list[list[float]] = []
    missing_uncertainty = 0
    for time_text, epoch in epochs.items():
        gga = epoch.get("gga")
        gst = epoch.get("gst")
        if not isinstance(gga, dict) or int(gga["quality"]) <= 0 or int(gga["satellites"]) < 4:
            continue
        if not isinstance(gst, dict):
            missing_uncertainty += 1
            continue
        date = gga.get("date")
        if not isinstance(date, str):
            raise ValueError(f"NMEA GGA at line {gga['line']} has no preceding RMC date")
        sigma = max(
            float(gst["latitude_sigma_m"]),
            float(gst["longitude_sigma_m"]),
            float(gst["altitude_sigma_m"]),
            0.25,
        )
        records.append(
            [
                _utc_from_nmea(date, time_text),
                float(gga["latitude_deg"]),
                float(gga["longitude_deg"]),
                float(gga["ellipsoidal_height_m"]),
                sigma * sigma,
                float(gga["quality"]),
                float(gga["satellites"]),
                float(gga["hdop"]),
            ]
        )
    records.sort(key=lambda row: row[0])
    result = np.asarray(records, dtype=np.float64)
    if len(result) < 2 or np.any(np.diff(result[:, 0]) <= 0.0):
        raise ValueError("NMEA position epochs are insufficient or non-monotonic")
    return {
        "data": result,
        "invalid_checksums": np.asarray([invalid_checksums], dtype=np.int64),
        "missing_uncertainty": np.asarray([missing_uncertainty], dtype=np.int64),
    }


def _geodetic_to_ecef(geodetic: np.ndarray) -> np.ndarray:
    latitude = np.radians(geodetic[:, 0])
    longitude = np.radians(geodetic[:, 1])
    height = geodetic[:, 2]
    semi_major = 6378137.0
    eccentricity_squared = 6.69437999014e-3
    prime_vertical = semi_major / np.sqrt(1.0 - eccentricity_squared * np.sin(latitude) ** 2)
    return np.column_stack(
        (
            (prime_vertical + height) * np.cos(latitude) * np.cos(longitude),
            (prime_vertical + height) * np.cos(latitude) * np.sin(longitude),
            (prime_vertical * (1.0 - eccentricity_squared) + height) * np.sin(latitude),
        )
    )


def _geodetic_to_ned(geodetic: np.ndarray, origin: np.ndarray) -> np.ndarray:
    origin_ecef = _geodetic_to_ecef(origin.reshape(1, 3))[0]
    delta = _geodetic_to_ecef(geodetic) - origin_ecef
    latitude = math.radians(float(origin[0]))
    longitude = math.radians(float(origin[1]))
    rotation = np.asarray(
        [
            [-math.sin(latitude) * math.cos(longitude),
             -math.sin(latitude) * math.sin(longitude), math.cos(latitude)],
            [-math.sin(longitude), math.cos(longitude), 0.0],
            [-math.cos(latitude) * math.cos(longitude),
             -math.cos(latitude) * math.sin(longitude), -math.sin(latitude)],
        ],
        dtype=np.float64,
    )
    return delta @ rotation.T


def _rotation_enu_source(roll: np.ndarray, pitch: np.ndarray, heading: np.ndarray) -> np.ndarray:
    yaw = -heading
    cp, sp = np.cos(roll), np.sin(roll)
    ct, st = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    result = np.empty((len(roll), 3, 3), dtype=np.float64)
    result[:, 0, 0] = cy * cp - sy * st * sp
    result[:, 0, 1] = -sy * ct
    result[:, 0, 2] = cy * sp + sy * st * cp
    result[:, 1, 0] = sy * cp + cy * st * sp
    result[:, 1, 1] = cy * ct
    result[:, 1, 2] = sy * sp - cy * st * cp
    result[:, 2, 0] = -ct * sp
    result[:, 2, 1] = st
    result[:, 2, 2] = ct * cp
    return result


def _quaternion_from_rotation(rotation: np.ndarray) -> np.ndarray:
    wxyz = np.empty((len(rotation), 4), dtype=np.float64)
    for index, matrix in enumerate(rotation):
        trace = float(np.trace(matrix))
        if trace > 0.0:
            scale = math.sqrt(trace + 1.0) * 2.0
            quaternion = np.asarray(
                [
                    0.25 * scale,
                    (matrix[2, 1] - matrix[1, 2]) / scale,
                    (matrix[0, 2] - matrix[2, 0]) / scale,
                    (matrix[1, 0] - matrix[0, 1]) / scale,
                ]
            )
        else:
            axis = int(np.argmax(np.diag(matrix)))
            if axis == 0:
                scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
                quaternion = np.asarray(
                    [
                        (matrix[2, 1] - matrix[1, 2]) / scale,
                        0.25 * scale,
                        (matrix[0, 1] + matrix[1, 0]) / scale,
                        (matrix[0, 2] + matrix[2, 0]) / scale,
                    ]
                )
            elif axis == 1:
                scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
                quaternion = np.asarray(
                    [
                        (matrix[0, 2] - matrix[2, 0]) / scale,
                        (matrix[0, 1] + matrix[1, 0]) / scale,
                        0.25 * scale,
                        (matrix[1, 2] + matrix[2, 1]) / scale,
                    ]
                )
            else:
                scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
                quaternion = np.asarray(
                    [
                        (matrix[1, 0] - matrix[0, 1]) / scale,
                        (matrix[0, 2] + matrix[2, 0]) / scale,
                        (matrix[1, 2] + matrix[2, 1]) / scale,
                        0.25 * scale,
                    ]
                )
        wxyz[index] = quaternion / np.linalg.norm(quaternion)
    for index in range(1, len(wxyz)):
        if np.dot(wxyz[index - 1], wxyz[index]) < 0.0:
            wxyz[index] *= -1.0
    return wxyz


def _interpolate_quaternions(
    source_time: np.ndarray, source_wxyz: np.ndarray, query_time: np.ndarray
) -> np.ndarray:
    right = np.searchsorted(source_time, query_time, side="right")
    left = np.clip(right - 1, 0, len(source_time) - 1)
    right = np.clip(right, 0, len(source_time) - 1)
    interpolated = np.empty((len(query_time), 4), dtype=np.float64)
    for index, query in enumerate(query_time):
        low = int(left[index])
        high = int(right[index])
        if low == high:
            interpolated[index] = source_wxyz[low]
            continue
        alpha = float((query - source_time[low]) / (source_time[high] - source_time[low]))
        first = source_wxyz[low]
        second = source_wxyz[high].copy()
        dot = float(np.dot(first, second))
        if dot < 0.0:
            second *= -1.0
            dot = -dot
        if dot > 0.9995:
            candidate = (1.0 - alpha) * first + alpha * second
        else:
            angle = math.acos(max(-1.0, min(1.0, dot)))
            candidate = (
                math.sin((1.0 - alpha) * angle) * first
                + math.sin(alpha * angle) * second
            ) / math.sin(angle)
        interpolated[index] = candidate / np.linalg.norm(candidate)
    for index in range(1, len(interpolated)):
        if np.dot(interpolated[index - 1], interpolated[index]) < 0.0:
            interpolated[index] *= -1.0
    return interpolated


def _interpolate_vectors(
    source_time: np.ndarray, source_values: np.ndarray, query_time: np.ndarray
) -> np.ndarray:
    return np.column_stack(
        [np.interp(query_time, source_time, source_values[:, axis]) for axis in range(3)]
    )


def _quaternion_to_euler_deg(quaternion: np.ndarray) -> np.ndarray:
    w, x, y, z = quaternion.T
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return np.degrees(np.column_stack((roll, pitch, yaw)))


def _truth_stream(truth: np.ndarray) -> dict[str, np.ndarray | list[float]]:
    geodetic = np.column_stack(
        (
            truth[:, 3] + truth[:, 4] / 60.0 + truth[:, 5] / 3600.0,
            truth[:, 6] + truth[:, 7] / 60.0 + truth[:, 8] / 3600.0,
            truth[:, 9],
        )
    )
    origin = geodetic[0]
    position_ned = _geodetic_to_ned(geodetic, origin)
    rotation_enu_source = _rotation_enu_source(
        np.radians(truth[:, 16]), np.radians(truth[:, 17]), np.radians(truth[:, 18])
    )
    rotation_ned_frd = np.einsum(
        "ij,njk,kl->nil", ENU_TO_NED, rotation_enu_source, SOURCE_TO_FRD
    )
    quaternion = _quaternion_from_rotation(rotation_ned_frd)
    velocity_enu = np.einsum("nij,nj->ni", rotation_enu_source, truth[:, 10:13])
    velocity_ned = velocity_enu @ ENU_TO_NED.T

    differentiated_velocity = np.gradient(position_ned, truth[:, 0], axis=0)
    moving = np.linalg.norm(velocity_ned[:, :2], axis=1) > 1.0
    if np.count_nonzero(moving) < 10:
        raise ValueError("SPAN truth does not contain enough moving samples for a frame audit")
    correlations = np.asarray(
        [
            np.corrcoef(velocity_ned[moving, axis], differentiated_velocity[moving, axis])[0, 1]
            for axis in range(2)
        ]
    )
    horizontal_rmse = float(
        np.sqrt(np.mean((velocity_ned[moving, :2] - differentiated_velocity[moving, :2]) ** 2))
    )
    if np.any(~np.isfinite(correlations)) or np.min(correlations) < 0.90 or horizontal_rmse > 1.5:
        raise ValueError(
            "SPAN body/navigation frame audit failed: "
            f"correlation={correlations.tolist()} rmse={horizontal_rmse:.3f} m/s"
        )
    return {
        "time": truth[:, 0],
        "origin": origin.tolist(),
        "position": position_ned,
        "velocity": velocity_ned,
        "quaternion": quaternion,
        "velocity_correlations": correlations,
        "velocity_horizontal_rmse_m_s": horizontal_rmse,
    }


def convert_urbannav(
    sequence_dir: Path,
    output_path: Path,
    metadata_path: Path | None = None,
    static_hint_duration_s: float = 0.0,
) -> dict[str, object]:
    sequence_dir = Path(sequence_dir)
    paths = {key: sequence_dir / value for key, value in EXPECTED_FILENAMES.items()}
    missing = [str(path) for path in paths.values() if not path.is_file()]
    if missing:
        raise FileNotFoundError("missing UrbanNav input: " + ", ".join(missing))
    if not math.isfinite(static_hint_duration_s) or static_hint_duration_s < 0.0:
        raise ValueError("static hint duration must be finite and non-negative")

    imu = _load_whitespace_numeric(paths["imu"], 2, 13)
    truth_raw = _load_whitespace_numeric(paths["truth"], 2, 20)
    nmea_document = _parse_nmea(paths["nmea"])
    nmea = nmea_document["data"]
    truth = _truth_stream(truth_raw)

    truth_time = np.asarray(truth["time"])
    overlap_start = max(imu[0, 0], truth_time[0])
    overlap_end = min(imu[-1, 0], truth_time[-1])
    overlap = (imu[:, 0] >= overlap_start) & (imu[:, 0] <= overlap_end)
    imu = imu[overlap]
    if len(imu) < 2:
        raise ValueError("UrbanNav IMU and SPAN truth do not overlap")

    reference_position = _interpolate_vectors(
        truth_time, np.asarray(truth["position"]), imu[:, 0]
    )
    reference_velocity = _interpolate_vectors(
        truth_time, np.asarray(truth["velocity"]), imu[:, 0]
    )
    reference_quaternion = _interpolate_quaternions(
        truth_time, np.asarray(truth["quaternion"]), imu[:, 0]
    )
    euler_deg = _quaternion_to_euler_deg(reference_quaternion)
    acceleration_frd = imu[:, 6:9] @ SOURCE_TO_FRD.T
    angular_rate_frd = imu[:, 3:6] @ SOURCE_TO_FRD.T

    nmea_overlap = (nmea[:, 0] >= imu[0, 0]) & (nmea[:, 0] <= imu[-1, 0])
    nmea = nmea[nmea_overlap]
    if len(nmea) < 2:
        raise ValueError("UrbanNav NMEA and IMU do not overlap")
    origin = np.asarray(truth["origin"], dtype=np.float64)
    nmea_position = _geodetic_to_ned(nmea[:, 1:4], origin)
    nmea_reference = _interpolate_vectors(
        truth_time, np.asarray(truth["position"]), nmea[:, 0]
    )
    nmea_error = nmea_position - nmea_reference
    nmea_horizontal_error = np.linalg.norm(nmea_error[:, :2], axis=1)
    nmea_three_dimensional_error = np.linalg.norm(nmea_error, axis=1)

    update_index = np.searchsorted(imu[:, 0], nmea[:, 0], side="left")
    in_range = update_index < len(imu)
    update_index = update_index[in_range]
    nmea = nmea[in_range]
    nmea_position = nmea_position[in_range]
    if len(np.unique(update_index)) != len(update_index):
        raise ValueError("multiple UrbanNav NMEA epochs map to one IMU sample")

    position_update = np.zeros(len(imu), dtype=np.int64)
    gps_position = np.zeros((len(imu), 3), dtype=np.float64)
    gps_variance = np.ones(len(imu), dtype=np.float64)
    position_update[update_index] = 1
    gps_position[update_index] = nmea_position
    gps_variance[update_index] = nmea[:, 4]
    timestamp_us = np.rint((imu[:, 0] - imu[0, 0]) * 1.0e6).astype(np.int64)
    static_hint = timestamp_us <= int(round(static_hint_duration_s * 1.0e6))
    if static_hint_duration_s == 0.0:
        static_hint[:] = False
    static_validation: dict[str, object] | None = None
    if np.any(static_hint):
        static_speed = np.linalg.norm(reference_velocity[static_hint], axis=1)
        static_gyro = np.linalg.norm(angular_rate_frd[static_hint], axis=1)
        static_acceleration_deviation = np.abs(
            np.linalg.norm(acceleration_frd[static_hint], axis=1) - GRAVITY_M_S2
        )
        static_validation = {
            "samples": int(np.count_nonzero(static_hint)),
            "speed_max_m_s": float(np.max(static_speed)),
            "gyro_norm_max_rad_s": float(np.max(static_gyro)),
            "acceleration_norm_deviation_max_m_s2": float(
                np.max(static_acceleration_deviation)
            ),
        }
        if (static_validation["speed_max_m_s"] > 0.25
                or static_validation["gyro_norm_max_rad_s"] > 0.15
                or static_validation["acceleration_norm_deviation_max_m_s2"] > 1.0):
            raise ValueError(
                "requested UrbanNav static hint is contradicted by truth/IMU motion: "
                f"{static_validation}"
            )

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
        "position_update", "gps_position_update", "gps_velocity_update",
        "gps_position_n_m", "gps_position_e_m", "gps_position_d_m",
        "gps_velocity_n_m_s", "gps_velocity_e_m_s", "gps_velocity_d_m_s",
        "gps_position_variance_m2", "gps_velocity_variance_m2_s2", "static_hint",
    ]
    with output_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        for index in range(len(imu)):
            writer.writerow(
                [
                    index, int(timestamp_us[index]),
                    *np.rint(acceleration_frd[index] / GRAVITY_M_S2 * 1000.0).astype(int),
                    *np.rint(np.degrees(angular_rate_frd[index]) * 1000.0).astype(int),
                    0, 0, 0, 0, 0, 0.0,
                    *np.rint(euler_deg[index] * 1000.0).astype(int),
                    *reference_quaternion[index], 1,
                    *reference_position[index], *reference_velocity[index],
                    0, int(position_update[index]), 0,
                    *gps_position[index], 0.0, 0.0, 0.0,
                    gps_variance[index], 1.0, int(static_hint[index]),
                ]
            )

    gaps = np.diff(nmea[:, 0])
    metadata: dict[str, object] = {
        "dataset": "UrbanNav Hong Kong",
        "sequence": "UrbanNav-HK-Medium-Urban-1",
        "source_doi": "10.33012/navi.602",
        "source_license": "dataset page provides attribution/contact terms; no SPDX license",
        "reference_kind": "independent_truth",
        "samples": int(len(imu)),
        "duration_s": float(timestamp_us[-1]) * 1.0e-6,
        "input_sha256": {key: _sha256(path) for key, path in paths.items()},
        "streams": {
            "imu_rate_hz": float(1.0 / np.median(np.diff(imu[:, 0]))),
            "truth_rate_hz": float(1.0 / np.median(np.diff(truth_time))),
            "nmea_position_updates": int(len(nmea)),
            "nmea_position_rate_hz_when_present": float(1.0 / np.median(gaps)),
            "nmea_maximum_gap_s": float(np.max(gaps)),
            "nmea_gaps_over_2_s": int(np.count_nonzero(gaps > 2.0)),
            "nmea_invalid_checksum_lines": int(nmea_document["invalid_checksums"][0]),
            "nmea_epochs_missing_gst": int(nmea_document["missing_uncertainty"][0]),
        },
        "frame_conversion": {
            "source_body": "UrbanNav body fixed at Xsens: x right, y forward, z up",
            "target_body": "FRD",
            "source_to_frd_matrix": SOURCE_TO_FRD.tolist(),
            "source_navigation": "geodetic/SPAN local ENU convention",
            "target_navigation": "local NED at first SPAN point",
            "truth_velocity_vs_position_difference_correlation_ne": np.asarray(
                truth["velocity_correlations"]
            ).tolist(),
            "truth_velocity_vs_position_difference_horizontal_rmse_m_s": float(
                truth["velocity_horizontal_rmse_m_s"]
            ),
        },
        "recorded_gnss_error": {
            "horizontal_rmse_m": float(np.sqrt(np.mean(nmea_horizontal_error**2))),
            "horizontal_p50_m": float(np.percentile(nmea_horizontal_error, 50.0)),
            "horizontal_p95_m": float(np.percentile(nmea_horizontal_error, 95.0)),
            "horizontal_max_m": float(np.max(nmea_horizontal_error)),
            "three_dimensional_rmse_m": float(
                np.sqrt(np.mean(nmea_three_dimensional_error**2))
            ),
            "three_dimensional_p95_m": float(
                np.percentile(nmea_three_dimensional_error, 95.0)
            ),
        },
        "coverage": {
            "speed_p95_m_s": float(np.percentile(np.linalg.norm(reference_velocity, axis=1), 95)),
            "speed_max_m_s": float(np.max(np.linalg.norm(reference_velocity, axis=1))),
            "gyro_norm_p95_rad_s": float(
                np.percentile(np.linalg.norm(angular_rate_frd, axis=1), 95)
            ),
            "gyro_norm_max_rad_s": float(np.max(np.linalg.norm(angular_rate_frd, axis=1))),
        },
        "static_hint_validation": static_validation,
        "aiding": {
            "position": "recorded u-blox F9P NMEA GGA, variance from same-epoch GST",
            "velocity": "not fused; recorded RMC/VTG course is blank",
            "antenna_lever_arm": "not compensated; published transform semantics need independent review",
        },
        "limitations": [
            "The platform is a ground vehicle, so this validates GNSS/INS navigation rather than aircraft dynamics.",
            "SPAN-CPT truth is independent of the low-cost Xsens/u-blox path but is itself a post-processed GNSS/INS solution.",
            "Only recorded position is fused; receiver velocity/course is not present in this NMEA stream.",
            "A single scalar GST-derived variance conservatively applies the worst reported axis sigma to all NED axes.",
            "The published GNSS antenna lever arm is not applied because transform direction is ambiguous in the source metadata.",
            "No magnetometer is used, so yaw behavior depends on initialization and inertial/navigation observability.",
        ],
    }
    if metadata_path is not None:
        metadata_path.parent.mkdir(parents=True, exist_ok=True)
        metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    return metadata


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sequence_dir", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--metadata", type=Path)
    parser.add_argument("--static-hint-duration-s", type=float, default=0.0)
    arguments = parser.parse_args()
    metadata = convert_urbannav(
        arguments.sequence_dir, arguments.out, arguments.metadata,
        arguments.static_hint_duration_s,
    )
    print(json.dumps(metadata, indent=2))


if __name__ == "__main__":
    main()
