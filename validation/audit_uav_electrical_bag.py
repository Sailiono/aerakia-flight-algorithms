#!/usr/bin/env python3
"""Fail-closed intake audit for Zenodo 8092105 UAV/RTK ROS bags."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Any

import numpy as np


TOPICS = (
    "/dji_osdk_ros/imu",
    "/imu/data",
    "/dji_osdk_ros/gps_position",
    "/dji_osdk_ros/rtk_position",
    "/dji_osdk_ros/rtk_velocity",
)


def _md5(path: Path) -> str:
    digest = hashlib.md5()  # noqa: S324 - verifies the publisher's immutable checksum
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _stamp_s(message: Any) -> float:
    stamp = message.header.stamp
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def _load(path: Path) -> dict[str, list[tuple[float, float, Any]]]:
    try:
        from rosbags.rosbag1 import Reader
        from rosbags.typesys import Stores, get_typestore
    except ImportError as error:
        raise RuntimeError("rosbags>=0.11 is required for this optional dataset intake") from error
    typestore = get_typestore(Stores.ROS1_NOETIC)
    result: dict[str, list[tuple[float, float, Any]]] = {topic: [] for topic in TOPICS}
    with Reader(path) as reader:
        connections = [connection for connection in reader.connections if connection.topic in result]
        missing = sorted(set(TOPICS) - {connection.topic for connection in connections})
        if missing:
            raise ValueError(f"required ROS topics are missing: {missing}")
        for connection, bag_timestamp_ns, raw in reader.messages(connections=connections):
            message = typestore.deserialize_ros1(raw, connection.msgtype)
            result[connection.topic].append(
                (_stamp_s(message), float(bag_timestamp_ns) * 1.0e-9, message)
            )
    return result


def _geodetic_to_ned(geodetic: np.ndarray, origin: np.ndarray) -> np.ndarray:
    latitude = np.radians(geodetic[:, 0])
    longitude = np.radians(geodetic[:, 1])
    height = geodetic[:, 2]
    semi_major = 6378137.0
    eccentricity_squared = 6.69437999014e-3
    prime_vertical = semi_major / np.sqrt(1.0 - eccentricity_squared * np.sin(latitude) ** 2)
    ecef = np.column_stack(
        (
            (prime_vertical + height) * np.cos(latitude) * np.cos(longitude),
            (prime_vertical + height) * np.cos(latitude) * np.sin(longitude),
            (prime_vertical * (1.0 - eccentricity_squared) + height) * np.sin(latitude),
        )
    )
    origin_latitude = math.radians(float(origin[0]))
    origin_longitude = math.radians(float(origin[1]))
    origin_prime_vertical = semi_major / math.sqrt(
        1.0 - eccentricity_squared * math.sin(origin_latitude) ** 2
    )
    origin_ecef = np.asarray(
        [
            (origin_prime_vertical + origin[2]) * math.cos(origin_latitude)
            * math.cos(origin_longitude),
            (origin_prime_vertical + origin[2]) * math.cos(origin_latitude)
            * math.sin(origin_longitude),
            (origin_prime_vertical * (1.0 - eccentricity_squared) + origin[2])
            * math.sin(origin_latitude),
        ]
    )
    rotation = np.asarray(
        [
            [-math.sin(origin_latitude) * math.cos(origin_longitude),
             -math.sin(origin_latitude) * math.sin(origin_longitude),
             math.cos(origin_latitude)],
            [-math.sin(origin_longitude), math.cos(origin_longitude), 0.0],
            [-math.cos(origin_latitude) * math.cos(origin_longitude),
             -math.cos(origin_latitude) * math.sin(origin_longitude),
             -math.sin(origin_latitude)],
        ]
    )
    return (ecef - origin_ecef) @ rotation.T


def _correlation(left: np.ndarray, right: np.ndarray) -> float:
    if len(left) < 3 or np.std(left) < 1.0e-8 or np.std(right) < 1.0e-8:
        return math.nan
    return float(np.corrcoef(left, right)[0, 1])


def _stream_timing(records: list[tuple[float, float, Any]]) -> dict[str, float | int]:
    header = np.asarray([record[0] for record in records])
    bag = np.asarray([record[1] for record in records])
    if len(header) < 2 or np.any(np.diff(header) <= 0.0):
        raise ValueError("header timestamps are insufficient or non-monotonic")
    latency = bag - header
    return {
        "samples": len(records),
        "duration_s": float(header[-1] - header[0]),
        "rate_hz": float((len(header) - 1) / (header[-1] - header[0])),
        "median_header_interval_s": float(np.median(np.diff(header))),
        "maximum_gap_s": float(np.max(np.diff(header))),
        "bag_minus_header_p50_ms": float(np.percentile(latency, 50.0) * 1000.0),
        "bag_minus_header_p95_ms": float(np.percentile(latency, 95.0) * 1000.0),
        "bag_minus_header_max_abs_ms": float(np.max(np.abs(latency)) * 1000.0),
    }


def audit(
    path: Path,
    expected_md5: str | None = None,
    streams: dict[str, list[tuple[float, float, Any]]] | None = None,
) -> dict[str, Any]:
    path = Path(path)
    actual_md5 = _md5(path)
    if expected_md5 is not None and actual_md5.lower() != expected_md5.lower():
        raise ValueError(f"MD5 mismatch: expected {expected_md5}, got {actual_md5}")
    streams = _load(path) if streams is None else streams
    timing = {topic: _stream_timing(records) for topic, records in streams.items()}

    imu = streams["/dji_osdk_ros/imu"]
    gyro = np.asarray(
        [[record[2].angular_velocity.x, record[2].angular_velocity.y,
          record[2].angular_velocity.z] for record in imu]
    )
    acceleration = np.asarray(
        [[record[2].linear_acceleration.x, record[2].linear_acceleration.y,
          record[2].linear_acceleration.z] for record in imu]
    )

    gps = streams["/dji_osdk_ros/gps_position"]
    rtk = streams["/dji_osdk_ros/rtk_position"]
    rtk_velocity_records = streams["/dji_osdk_ros/rtk_velocity"]
    gps_t = np.asarray([record[0] for record in gps])
    rtk_t = np.asarray([record[0] for record in rtk])
    rtk_velocity_t = np.asarray([record[0] for record in rtk_velocity_records])
    gps_geodetic = np.asarray(
        [[record[2].latitude, record[2].longitude, record[2].altitude] for record in gps]
    )
    rtk_geodetic = np.asarray(
        [[record[2].latitude, record[2].longitude, record[2].altitude] for record in rtk]
    )
    gps_ned = _geodetic_to_ned(gps_geodetic, gps_geodetic[0])
    rtk_ned = _geodetic_to_ned(rtk_geodetic, rtk_geodetic[0])
    rtk_velocity_raw = np.asarray(
        [[record[2].vector.x, record[2].vector.y, record[2].vector.z]
         for record in rtk_velocity_records]
    )
    if len(rtk_t) != len(rtk_velocity_t) or np.max(np.abs(rtk_t - rtk_velocity_t)) > 1.0e-3:
        raise ValueError("RTK position and velocity timestamps are not paired within 1 ms")

    differentiated_time = 0.5 * (rtk_t[1:] + rtk_t[:-1])
    differentiated_velocity = np.diff(rtk_ned, axis=0) / np.diff(rtk_t)[:, None]
    candidates = {
        "ned_xyz": rtk_velocity_raw,
        "ned_xyz_z_flipped": rtk_velocity_raw * np.asarray([1.0, 1.0, -1.0]),
        "enu_xyz_to_ned": rtk_velocity_raw[:, [1, 0, 2]] * np.asarray([1.0, 1.0, -1.0]),
        "enu_xyz_to_ned_keep_z": rtk_velocity_raw[:, [1, 0, 2]],
    }
    velocity_audits: dict[str, dict[str, float]] = {}
    for name, candidate in candidates.items():
        interpolated = np.column_stack(
            [np.interp(differentiated_time, rtk_velocity_t, candidate[:, axis]) for axis in range(3)]
        )
        horizontal_error = np.linalg.norm(interpolated[:, :2] - differentiated_velocity[:, :2], axis=1)
        three_dimensional_error = np.linalg.norm(interpolated - differentiated_velocity, axis=1)
        velocity_audits[name] = {
            "north_correlation": _correlation(interpolated[:, 0], differentiated_velocity[:, 0]),
            "east_correlation": _correlation(interpolated[:, 1], differentiated_velocity[:, 1]),
            "down_correlation": _correlation(interpolated[:, 2], differentiated_velocity[:, 2]),
            "horizontal_rmse_m_s": float(np.sqrt(np.mean(horizontal_error**2))),
            "three_dimensional_rmse_m_s": float(
                np.sqrt(np.mean(three_dimensional_error**2))
            ),
        }
    selected_velocity_frame = min(
        velocity_audits, key=lambda name: velocity_audits[name]["three_dimensional_rmse_m_s"]
    )
    selected_audit = velocity_audits[selected_velocity_frame]
    if (selected_audit["north_correlation"] < 0.9
            or selected_audit["east_correlation"] < 0.9
            or selected_audit["horizontal_rmse_m_s"] > 2.0
            or selected_audit["three_dimensional_rmse_m_s"] > 2.0):
        raise ValueError(f"RTK velocity frame audit failed: {velocity_audits}")

    gps_at_rtk = np.column_stack(
        [np.interp(rtk_t, gps_t, gps_ned[:, axis]) for axis in range(3)]
    )
    relative_position_error = gps_at_rtk - rtk_ned
    horizontal_error = np.linalg.norm(relative_position_error[:, :2], axis=1)
    if float(np.percentile(horizontal_error, 95.0)) > 10.0:
        raise ValueError("drone GPS and RTK relative horizontal trajectories disagree by over 10 m P95")

    overlap_start = max(records[0][0] for records in streams.values())
    overlap_end = min(records[-1][0] for records in streams.values())
    overlap_s = float(overlap_end - overlap_start)
    if overlap_s < 10.0:
        raise ValueError(f"common required-stream overlap is only {overlap_s:.3f} s")

    return {
        "schema_version": 1,
        "dataset": "UAV Surveying - Electrical Transmission Infrastructure",
        "source_doi": "10.5281/zenodo.8092105",
        "source_license": "CC-BY-4.0",
        "file": path.name,
        "file_md5": actual_md5,
        "evidence_boundary": (
            "Recorded DJI IMU/GPS plus a separate RTK stream and Xsens-class AHRS topic. "
            "The smallest bag has no separate drone-GPS velocity topic; RTK is an external "
            "reference, not an independent metrology-grade ground truth claim."
        ),
        "common_overlap_s": overlap_s,
        "streams": timing,
        "coverage": {
            "gyro_norm_p95_rad_s": float(np.percentile(np.linalg.norm(gyro, axis=1), 95.0)),
            "gyro_norm_max_rad_s": float(np.max(np.linalg.norm(gyro, axis=1))),
            "acceleration_norm_p95_m_s2": float(
                np.percentile(np.linalg.norm(acceleration, axis=1), 95.0)
            ),
            "acceleration_norm_max_m_s2": float(np.max(np.linalg.norm(acceleration, axis=1))),
        },
        "rtk_velocity_frame_audit": {
            "selected": selected_velocity_frame,
            "candidates": velocity_audits,
        },
        "drone_gps_vs_rtk_relative_position": {
            "horizontal_rmse_m": float(np.sqrt(np.mean(horizontal_error**2))),
            "horizontal_p95_m": float(np.percentile(horizontal_error, 95.0)),
            "horizontal_max_m": float(np.max(horizontal_error)),
            "vertical_rmse_after_independent_origin_alignment_m": float(
                np.sqrt(np.mean(relative_position_error[:, 2] ** 2))
            ),
        },
        "accepted_for": [
            "aerial physical-IMU and physical-position compatibility",
            "external RTK relative-position and velocity reference",
            "timestamp/frame intake testing",
        ],
        "not_accepted_for": [
            "physical drone-GPS velocity aiding",
            "independent absolute attitude accuracy",
            "metrology-grade absolute position truth",
        ],
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--expected-md5")
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    result = audit(args.bag, args.expected_md5)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
