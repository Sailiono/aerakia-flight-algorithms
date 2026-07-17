#!/usr/bin/env python3
"""Convert a PX4 ULog into Aerakia's hardware-neutral replay CSV contract."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Callable

import numpy as np


GRAVITY_M_S2 = 9.80665
EARTH_RADIUS_M = 6_378_137.0
TOPICS = (
    "sensor_combined",
    "vehicle_magnetometer",
    "vehicle_attitude",
    "vehicle_local_position",
    "vehicle_gps_position",
    "vehicle_air_data",
    "yaw_estimator_status",
)


def normalize_quaternions(quaternions: np.ndarray) -> np.ndarray:
    values = np.asarray(quaternions, dtype=np.float64).copy()
    norms = np.linalg.norm(values, axis=1)
    valid = np.isfinite(norms) & (norms > 1.0e-12)
    values[valid] /= norms[valid, None]
    values[~valid] = np.array([1.0, 0.0, 0.0, 0.0])
    for index in range(1, len(values)):
        if np.dot(values[index - 1], values[index]) < 0.0:
            values[index] *= -1.0
    return values


def interpolate_quaternions(
    source_timestamps_us: np.ndarray,
    source_quaternions: np.ndarray,
    query_timestamps_us: np.ndarray,
    source_groups: np.ndarray | None = None,
) -> np.ndarray:
    """Normalized linear interpolation, never crossing a reset-group boundary."""
    timestamps = np.asarray(source_timestamps_us, dtype=np.int64)
    quaternions = normalize_quaternions(source_quaternions)
    query = np.asarray(query_timestamps_us, dtype=np.int64)
    if len(timestamps) == 0:
        raise ValueError("cannot interpolate an empty quaternion stream")
    if source_groups is None:
        groups = np.zeros(len(timestamps), dtype=np.int64)
    else:
        groups = np.asarray(source_groups)

    right = np.searchsorted(timestamps, query, side="right")
    left = np.clip(right - 1, 0, len(timestamps) - 1)
    right = np.clip(right, 0, len(timestamps) - 1)
    result = np.empty((len(query), 4), dtype=np.float64)
    for index, query_timestamp in enumerate(query):
        low = int(left[index])
        high = int(right[index])
        if low == high or timestamps[high] == timestamps[low] or groups[low] != groups[high]:
            result[index] = quaternions[low]
            continue
        alpha = float(query_timestamp - timestamps[low]) / float(timestamps[high] - timestamps[low])
        candidate = (1.0 - alpha) * quaternions[low] + alpha * quaternions[high]
        norm = np.linalg.norm(candidate)
        result[index] = candidate / norm if norm > 1.0e-12 else quaternions[low]
    return result


def held_samples(
    source_timestamps_us: np.ndarray,
    source_values: np.ndarray,
    query_timestamps_us: np.ndarray,
    fill_value: float = math.nan,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Zero-order hold plus source-valid and fresh-sample masks."""
    timestamps = np.asarray(source_timestamps_us, dtype=np.int64)
    values = np.asarray(source_values)
    query = np.asarray(query_timestamps_us, dtype=np.int64)
    output_shape = (len(query),) + values.shape[1:]
    output = np.full(output_shape, fill_value, dtype=np.float64)
    valid = np.zeros(len(query), dtype=bool)
    fresh = np.zeros(len(query), dtype=bool)
    if len(timestamps) == 0:
        return output, valid, fresh
    indices = np.searchsorted(timestamps, query, side="right") - 1
    valid = indices >= 0
    safe = np.clip(indices, 0, len(timestamps) - 1)
    output[valid] = values[safe[valid]]
    fresh[0] = valid[0]
    if len(query) > 1:
        fresh[1:] = valid[1:] & (~valid[:-1] | (safe[1:] != safe[:-1]))
    return output, valid, fresh


def _topic(ulog: Any, name: str) -> dict[str, np.ndarray] | None:
    for dataset in getattr(ulog, "data_list", []):
        if dataset.name == name and getattr(dataset, "multi_id", 0) == 0:
            return dataset.data
    try:
        return ulog.get_dataset(name).data
    except (KeyError, IndexError, ValueError, StopIteration):
        return None


def _field(data: dict[str, np.ndarray], *names: str, default: Any = None) -> np.ndarray:
    for name in names:
        if name in data:
            return np.asarray(data[name])
    if default is not None:
        return np.asarray(default)
    raise KeyError(f"none of the ULog fields exist: {names}")


def _vector(data: dict[str, np.ndarray], *bases: str) -> np.ndarray:
    for base in bases:
        names = [f"{base}[{axis}]" for axis in range(3)]
        if all(name in data for name in names):
            return np.column_stack([data[name] for name in names]).astype(np.float64)
    raise KeyError(f"none of the ULog vectors exist: {bases}")


def _quaternion(data: dict[str, np.ndarray], base: str) -> np.ndarray:
    names = [f"{base}[{axis}]" for axis in range(4)]
    if not all(name in data for name in names):
        raise KeyError(f"missing quaternion field: {base}")
    return np.column_stack([data[name] for name in names]).astype(np.float64)


def _timestamps(data: dict[str, np.ndarray]) -> np.ndarray:
    return _field(data, "timestamp_sample", "timestamp").astype(np.int64)


def _sort_unique(timestamp: np.ndarray, *values: np.ndarray) -> tuple[np.ndarray, ...]:
    order = np.argsort(timestamp, kind="stable")
    sorted_timestamp = timestamp[order]
    keep = np.r_[True, sorted_timestamp[1:] > sorted_timestamp[:-1]]
    return (sorted_timestamp[keep], *(value[order][keep] for value in values))


def _quaternion_to_euler_deg(quaternion: np.ndarray) -> np.ndarray:
    w, x, y, z = quaternion.T
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return np.degrees(np.column_stack((roll, pitch, yaw)))


def _gravity_body_mg(quaternion: np.ndarray) -> np.ndarray:
    w, x, y, z = quaternion.T
    return 1000.0 * np.column_stack(
        (
            -2.0 * (x * z - w * y),
            -2.0 * (y * z + w * x),
            -(1.0 - 2.0 * (x * x + y * y)),
        )
    )


def _relative_gps_ned(gps: dict[str, np.ndarray]) -> tuple[np.ndarray, np.ndarray]:
    latitude = _field(gps, "lat").astype(np.float64) * 1.0e-7
    longitude = _field(gps, "lon").astype(np.float64) * 1.0e-7
    altitude = _field(gps, "alt", "alt_ellipsoid").astype(np.float64) * 1.0e-3
    fix_type = _field(gps, "fix_type", default=np.full(len(latitude), 3))
    valid = (fix_type >= 3) & np.isfinite(latitude) & np.isfinite(longitude) & np.isfinite(altitude)
    valid &= (np.abs(latitude) > 1.0e-9) | (np.abs(longitude) > 1.0e-9)
    ned = np.full((len(latitude), 3), np.nan, dtype=np.float64)
    if np.any(valid):
        first = int(np.flatnonzero(valid)[0])
        latitude0 = math.radians(latitude[first])
        ned[valid, 0] = np.radians(latitude[valid] - latitude[first]) * EARTH_RADIUS_M
        ned[valid, 1] = (
            np.radians(longitude[valid] - longitude[first]) * EARTH_RADIUS_M * math.cos(latitude0)
        )
        ned[valid, 2] = altitude[first] - altitude[valid]
    return ned, valid


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _load_ulog(path: Path) -> Any:
    try:
        from pyulog import ULog
    except ImportError as error:
        raise RuntimeError("pyulog is required; install the repository requirements") from error
    return ULog(str(path))


def convert_ulog(
    ulog_path: Path,
    output_csv: Path,
    metadata_path: Path | None = None,
    assume_stationary: bool = False,
    ulog_factory: Callable[[Path], Any] = _load_ulog,
) -> dict[str, Any]:
    ulog_path = Path(ulog_path)
    ulog = ulog_factory(ulog_path)
    sensor = _topic(ulog, "sensor_combined")
    attitude = _topic(ulog, "vehicle_attitude")
    if sensor is None or attitude is None:
        raise ValueError("ULog must contain sensor_combined and vehicle_attitude")

    imu_t = _timestamps(sensor)
    acceleration = _vector(sensor, "accelerometer_m_s2")
    angular_rate = _vector(sensor, "gyro_rad", "gyroscope_rad_s")
    imu_valid = np.isfinite(imu_t) & np.all(np.isfinite(acceleration), axis=1)
    imu_valid &= np.all(np.isfinite(angular_rate), axis=1)
    imu_t, acceleration, angular_rate = _sort_unique(
        imu_t[imu_valid], acceleration[imu_valid], angular_rate[imu_valid]
    )
    if len(imu_t) < 2:
        raise ValueError("ULog contains fewer than two valid IMU samples")

    attitude_t = _timestamps(attitude)
    attitude_q = _quaternion(attitude, "q")
    reset_counter = _field(attitude, "quat_reset_counter", default=np.zeros(len(attitude_t))).astype(int)
    delta_q_reset = np.column_stack(
        [
            _field(
                attitude,
                f"delta_q_reset[{axis}]",
                default=np.full(len(attitude_t), 1.0 if axis == 0 else 0.0),
            )
            for axis in range(4)
        ]
    ).astype(np.float64)
    attitude_valid = np.isfinite(attitude_t) & np.all(np.isfinite(attitude_q), axis=1)
    attitude_t, attitude_q, reset_counter, delta_q_reset = _sort_unique(
        attitude_t[attitude_valid],
        attitude_q[attitude_valid],
        reset_counter[attitude_valid],
        delta_q_reset[attitude_valid],
    )
    reference_q = interpolate_quaternions(attitude_t, attitude_q, imu_t, reset_counter)
    reference_euler = _quaternion_to_euler_deg(reference_q)
    gravity_mg = _gravity_body_mg(reference_q)
    held_reset_counter, reset_valid, _ = held_samples(attitude_t, reset_counter, imu_t, 0.0)
    held_delta_q, _, _ = held_samples(attitude_t, delta_q_reset, imu_t, 0.0)
    held_reset_counter = held_reset_counter.astype(int)
    reset_event = np.zeros(len(imu_t), dtype=int)
    if len(reset_event) > 1:
        reset_event[1:] = (held_reset_counter[1:] != held_reset_counter[:-1]).astype(int)

    magnetometer = _topic(ulog, "vehicle_magnetometer")
    magnetic_ut = np.zeros((len(imu_t), 3), dtype=np.float64)
    magnetic_valid = np.zeros(len(imu_t), dtype=bool)
    magnetic_update = np.zeros(len(imu_t), dtype=bool)
    if magnetometer is not None:
        mag_t = _timestamps(magnetometer)
        mag_gauss = _vector(magnetometer, "magnetometer_ga", "magnetometer_gauss")
        mag_t, mag_gauss = _sort_unique(mag_t, mag_gauss)
        held_mag, held_valid, held_fresh = held_samples(mag_t, mag_gauss * 100.0, imu_t, 0.0)
        magnetic_ut = held_mag
        magnetic_valid = held_valid & np.all(np.isfinite(held_mag), axis=1)
        magnetic_valid &= np.linalg.norm(held_mag, axis=1) > 1.0e-6
        magnetic_update = magnetic_valid & held_fresh

    local_position = _topic(ulog, "vehicle_local_position")
    reference_position = np.zeros((len(imu_t), 3), dtype=np.float64)
    reference_velocity = np.zeros((len(imu_t), 3), dtype=np.float64)
    reference_position_valid = np.zeros(len(imu_t), dtype=bool)
    if local_position is not None:
        local_t = _timestamps(local_position)
        local_p = np.column_stack([_field(local_position, name) for name in ("x", "y", "z")])
        local_v = np.column_stack([_field(local_position, name) for name in ("vx", "vy", "vz")])
        validity = _field(local_position, "xy_valid", default=np.ones(len(local_t))).astype(bool)
        validity &= _field(local_position, "z_valid", default=np.ones(len(local_t))).astype(bool)
        validity &= _field(local_position, "v_xy_valid", default=np.ones(len(local_t))).astype(bool)
        validity &= _field(local_position, "v_z_valid", default=np.ones(len(local_t))).astype(bool)
        local_t, local_p, local_v, validity = _sort_unique(local_t, local_p, local_v, validity)
        held_p, p_valid, _ = held_samples(local_t, local_p, imu_t, 0.0)
        held_v, v_valid, _ = held_samples(local_t, local_v, imu_t, 0.0)
        held_validity, _, _ = held_samples(local_t, validity.astype(float), imu_t, 0.0)
        reference_position_valid = p_valid & v_valid & (held_validity > 0.5)
        reference_position_valid &= np.all(np.isfinite(held_p), axis=1) & np.all(np.isfinite(held_v), axis=1)
        if np.any(reference_position_valid):
            first = int(np.flatnonzero(reference_position_valid)[0])
            reference_position = held_p - held_p[first]
            reference_velocity = held_v

    gps = _topic(ulog, "vehicle_gps_position")
    gps_position = np.zeros((len(imu_t), 3), dtype=np.float64)
    gps_velocity = np.zeros((len(imu_t), 3), dtype=np.float64)
    gps_position_variance = np.ones(len(imu_t), dtype=np.float64)
    gps_velocity_variance = np.ones(len(imu_t), dtype=np.float64)
    gps_update = np.zeros(len(imu_t), dtype=bool)
    if gps is not None:
        gps_t = _timestamps(gps)
        gps_p, gps_valid = _relative_gps_ned(gps)
        gps_v = np.column_stack(
            [_field(gps, name) for name in ("vel_n_m_s", "vel_e_m_s", "vel_d_m_s")]
        ).astype(np.float64)
        eph = _field(gps, "eph", default=np.ones(len(gps_t))).astype(np.float64)
        epv = _field(gps, "epv", default=eph).astype(np.float64)
        speed_sigma = _field(gps, "s_variance_m_s", default=np.ones(len(gps_t))).astype(np.float64)
        position_variance = np.maximum(np.maximum(eph, epv), 0.25) ** 2
        velocity_variance = np.maximum(speed_sigma, 0.05) ** 2
        gps_valid &= np.all(np.isfinite(gps_v), axis=1)
        gps_t, gps_p, gps_v, position_variance, velocity_variance, gps_valid = _sort_unique(
            gps_t, gps_p, gps_v, position_variance, velocity_variance, gps_valid
        )
        held_p, held_valid, held_fresh = held_samples(gps_t, gps_p, imu_t, 0.0)
        held_v, _, _ = held_samples(gps_t, gps_v, imu_t, 0.0)
        held_p_var, _, _ = held_samples(gps_t, position_variance, imu_t, 1.0)
        held_v_var, _, _ = held_samples(gps_t, velocity_variance, imu_t, 1.0)
        held_fix, _, _ = held_samples(gps_t, gps_valid.astype(float), imu_t, 0.0)
        gps_position = held_p
        gps_velocity = held_v
        gps_position_variance = held_p_var
        gps_velocity_variance = held_v_var
        gps_update = held_valid & held_fresh & (held_fix > 0.5)

    air_data = _topic(ulog, "vehicle_air_data")
    barometer_height = np.zeros(len(imu_t), dtype=np.float64)
    barometer_variance = np.ones(len(imu_t), dtype=np.float64)
    barometer_update = np.zeros(len(imu_t), dtype=bool)
    if air_data is not None:
        baro_t = _timestamps(air_data)
        altitude = _field(air_data, "baro_alt_meter").astype(np.float64)
        baro_t, altitude = _sort_unique(baro_t, altitude)
        held_altitude, held_valid, held_fresh = held_samples(baro_t, altitude, imu_t, 0.0)
        finite = held_valid & np.isfinite(held_altitude)
        if np.any(finite):
            first = int(np.flatnonzero(finite)[0])
            barometer_height = held_altitude - held_altitude[first]
        barometer_update = finite & held_fresh

    yaw_estimator = _topic(ulog, "yaw_estimator_status")
    gsf_yaw_rad = np.zeros(len(imu_t), dtype=np.float64)
    gsf_yaw_variance_rad2 = np.zeros(len(imu_t), dtype=np.float64)
    gsf_yaw_valid = np.zeros(len(imu_t), dtype=bool)
    if yaw_estimator is not None:
        gsf_t = _timestamps(yaw_estimator)
        gsf_yaw = _field(yaw_estimator, "yaw_composite").astype(np.float64)
        gsf_variance = _field(yaw_estimator, "yaw_variance").astype(np.float64)
        source_valid = np.isfinite(gsf_yaw) & np.isfinite(gsf_variance)
        # A 0.2-rad one-sigma ceiling excludes the multi-hypothesis convergence
        # period without pretending GSF is ground truth.
        source_valid &= (gsf_variance > 0.0) & (gsf_variance <= 0.04)
        gsf_t, gsf_yaw, gsf_variance, source_valid = _sort_unique(
            gsf_t, gsf_yaw, gsf_variance, source_valid
        )
        held_yaw, held_valid, _ = held_samples(gsf_t, gsf_yaw, imu_t, 0.0)
        held_variance, _, _ = held_samples(gsf_t, gsf_variance, imu_t, 0.0)
        held_source_valid, _, _ = held_samples(gsf_t, source_valid.astype(float), imu_t, 0.0)
        gsf_yaw_rad = held_yaw
        gsf_yaw_variance_rad2 = held_variance
        gsf_yaw_valid = held_valid & (held_source_valid > 0.5)

    dt_us = np.r_[int(np.median(np.diff(imu_t))), np.diff(imu_t)]
    header = [
        "seq", "host_ts_us", "ts_us", "dt_us",
        "raw_acc_mg_x", "raw_acc_mg_y", "raw_acc_mg_z",
        "raw_gyro_mdps_x", "raw_gyro_mdps_y", "raw_gyro_mdps_z",
        "raw_mag_cuT_x", "raw_mag_cuT_y", "raw_mag_cuT_z", "mag_valid", "mag_update",
        "g_est_mg_x", "g_est_mg_y", "g_est_mg_z",
        "roll_mdeg", "pitch_mdeg", "yaw_mdeg",
        "ref_q_w", "ref_q_x", "ref_q_y", "ref_q_z",
        "ref_attitude_reset_counter", "ref_attitude_reset_event",
        "ref_delta_q_reset_w", "ref_delta_q_reset_x", "ref_delta_q_reset_y", "ref_delta_q_reset_z",
        "ref_gsf_yaw_rad", "ref_gsf_yaw_variance_rad2", "ref_gsf_yaw_valid",
        "position_ref_valid", "ref_position_n_m", "ref_position_e_m", "ref_position_d_m",
        "ref_velocity_n_m_s", "ref_velocity_e_m_s", "ref_velocity_d_m_s",
        "position_update", "gps_position_n_m", "gps_position_e_m", "gps_position_d_m",
        "gps_velocity_n_m_s", "gps_velocity_e_m_s", "gps_velocity_d_m_s",
        "gps_position_variance_m2", "gps_velocity_variance_m2_s2",
        "baro_update", "baro_height_up_m", "baro_variance_m2", "static_hint",
    ]
    output_csv.parent.mkdir(parents=True, exist_ok=True)
    with output_csv.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        for index in range(len(imu_t)):
            writer.writerow(
                [
                    index, int(imu_t[index]), int(imu_t[index]), int(dt_us[index]),
                    *np.rint(acceleration[index] / GRAVITY_M_S2 * 1000.0).astype(int),
                    *np.rint(np.degrees(angular_rate[index]) * 1000.0).astype(int),
                    *np.rint(magnetic_ut[index] * 100.0).astype(int),
                    int(magnetic_valid[index]), int(magnetic_update[index]),
                    *np.rint(gravity_mg[index]).astype(int),
                    *np.rint(reference_euler[index] * 1000.0).astype(int),
                    *reference_q[index],
                    int(held_reset_counter[index]) if reset_valid[index] else 0,
                    int(reset_event[index]),
                    *held_delta_q[index],
                    gsf_yaw_rad[index], gsf_yaw_variance_rad2[index], int(gsf_yaw_valid[index]),
                    int(reference_position_valid[index]),
                    *reference_position[index], *reference_velocity[index],
                    int(gps_update[index]), *gps_position[index], *gps_velocity[index],
                    gps_position_variance[index], gps_velocity_variance[index],
                    int(barometer_update[index]), barometer_height[index], barometer_variance[index],
                    int(assume_stationary),
                ]
            )

    metadata = {
        "source": ulog_path.name,
        "source_sha256": _sha256(ulog_path) if ulog_path.exists() else None,
        "samples": int(len(imu_t)),
        "duration_s": float((imu_t[-1] - imu_t[0]) * 1.0e-6),
        "topics_present": [topic for topic in TOPICS if _topic(ulog, topic) is not None],
        "magnetometer_updates": int(np.count_nonzero(magnetic_update)),
        "gps_updates": int(np.count_nonzero(gps_update)),
        "barometer_updates": int(np.count_nonzero(barometer_update)),
        "attitude_reset_events": int(np.count_nonzero(reset_event)),
        "gsf_yaw_reference_samples": int(np.count_nonzero(gsf_yaw_valid)),
        "assume_stationary": bool(assume_stationary),
        "privacy": "Hardware identifiers and absolute GPS coordinates are not exported.",
    }
    if metadata_path is not None:
        metadata_path.parent.mkdir(parents=True, exist_ok=True)
        metadata_path.write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return metadata


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ulog", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--metadata", type=Path)
    parser.add_argument(
        "--assume-stationary",
        action="store_true",
        help="mark every row as application-declared stationary for alignment/ZUPT tests",
    )
    args = parser.parse_args()
    metadata = convert_ulog(args.ulog, args.out, args.metadata, args.assume_stationary)
    print(json.dumps(metadata, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
