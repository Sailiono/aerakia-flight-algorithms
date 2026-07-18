#!/usr/bin/env python3
"""Convert Zenodo 8092105 DJI/RTK ROS bag data into Aerakia replay CSV."""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "validation"))

import audit_uav_electrical_bag as intake  # noqa: E402
from convert_urbannav_to_replay import _quaternion_from_rotation  # noqa: E402
from convert_ulog_to_replay import _quaternion_to_euler_deg  # noqa: E402


GRAVITY_M_S2 = 9.80665
FLU_FROM_FRD = np.diag([1.0, -1.0, -1.0])
NED_FROM_ENU = np.asarray([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])


def _rotation_from_quaternion_wxyz(quaternion: np.ndarray) -> np.ndarray:
    quaternion = np.asarray(quaternion, dtype=np.float64)
    quaternion /= np.linalg.norm(quaternion, axis=1)[:, None]
    w, x, y, z = quaternion.T
    result = np.empty((len(quaternion), 3, 3), dtype=np.float64)
    result[:, 0, 0] = 1.0 - 2.0 * (y * y + z * z)
    result[:, 0, 1] = 2.0 * (x * y - z * w)
    result[:, 0, 2] = 2.0 * (x * z + y * w)
    result[:, 1, 0] = 2.0 * (x * y + z * w)
    result[:, 1, 1] = 1.0 - 2.0 * (x * x + z * z)
    result[:, 1, 2] = 2.0 * (y * z - x * w)
    result[:, 2, 0] = 2.0 * (x * z - y * w)
    result[:, 2, 1] = 2.0 * (y * z + x * w)
    result[:, 2, 2] = 1.0 - 2.0 * (x * x + y * y)
    return result


def _records_to_geodetic(records: list[tuple[float, float, object]]) -> np.ndarray:
    return np.asarray(
        [[record[2].latitude, record[2].longitude, record[2].altitude] for record in records]
    )


def _interpolate(time: np.ndarray, values: np.ndarray, query: np.ndarray) -> np.ndarray:
    return np.column_stack([np.interp(query, time, values[:, axis]) for axis in range(values.shape[1])])


def convert(path: Path, output: Path, metadata_path: Path | None = None) -> dict[str, object]:
    path = Path(path)
    streams = intake._load(path)
    audit = intake.audit(path, streams=streams)
    imu_records = streams["/dji_osdk_ros/imu"]
    gps_records = streams["/dji_osdk_ros/gps_position"]
    rtk_records = streams["/dji_osdk_ros/rtk_position"]
    rtk_velocity_records = streams["/dji_osdk_ros/rtk_velocity"]

    overlap_start = max(imu_records[0][0], gps_records[0][0], rtk_records[0][0])
    overlap_end = min(imu_records[-1][0], gps_records[-1][0], rtk_records[-1][0])
    imu_records = [record for record in imu_records if overlap_start <= record[0] <= overlap_end]
    imu_t = np.asarray([record[0] for record in imu_records])
    acceleration_flu = np.asarray(
        [[record[2].linear_acceleration.x, record[2].linear_acceleration.y,
          record[2].linear_acceleration.z] for record in imu_records]
    )
    angular_rate_flu = np.asarray(
        [[record[2].angular_velocity.x, record[2].angular_velocity.y,
          record[2].angular_velocity.z] for record in imu_records]
    )
    source_quaternion = np.asarray(
        [[record[2].orientation.w, record[2].orientation.x,
          record[2].orientation.y, record[2].orientation.z] for record in imu_records]
    )
    source_rotation = _rotation_from_quaternion_wxyz(source_quaternion)
    reference_rotation = np.einsum(
        "ij,njk,kl->nil", NED_FROM_ENU, source_rotation, FLU_FROM_FRD
    )
    reference_quaternion = _quaternion_from_rotation(reference_rotation)
    euler_deg = _quaternion_to_euler_deg(reference_quaternion)
    acceleration_frd = acceleration_flu @ FLU_FROM_FRD.T
    angular_rate_frd = angular_rate_flu @ FLU_FROM_FRD.T

    gps_t = np.asarray([record[0] for record in gps_records])
    rtk_t = np.asarray([record[0] for record in rtk_records])
    rtk_velocity_t = np.asarray([record[0] for record in rtk_velocity_records])
    gps_geodetic = _records_to_geodetic(gps_records)
    rtk_geodetic = _records_to_geodetic(rtk_records)
    gps_ned = intake._geodetic_to_ned(gps_geodetic, gps_geodetic[0])
    rtk_ned = intake._geodetic_to_ned(rtk_geodetic, rtk_geodetic[0])
    rtk_velocity_raw = np.asarray(
        [[record[2].vector.x, record[2].vector.y, record[2].vector.z]
         for record in rtk_velocity_records]
    )
    velocity_frame = audit["rtk_velocity_frame_audit"]["selected"]
    if velocity_frame == "ned_xyz":
        rtk_velocity_ned = rtk_velocity_raw
    elif velocity_frame == "ned_xyz_z_flipped":
        rtk_velocity_ned = rtk_velocity_raw * np.asarray([1.0, 1.0, -1.0])
    elif velocity_frame == "enu_xyz_to_ned":
        rtk_velocity_ned = rtk_velocity_raw[:, [1, 0, 2]] * np.asarray([1.0, 1.0, -1.0])
    elif velocity_frame == "enu_xyz_to_ned_keep_z":
        rtk_velocity_ned = rtk_velocity_raw[:, [1, 0, 2]]
    else:
        raise ValueError(f"unsupported audited velocity frame: {velocity_frame}")
    reference_position = _interpolate(rtk_t, rtk_ned, imu_t)
    reference_velocity = _interpolate(rtk_velocity_t, rtk_velocity_ned, imu_t)

    position_update = np.zeros(len(imu_t), dtype=bool)
    gps_position = np.zeros((len(imu_t), 3), dtype=np.float64)
    gps_indices = np.searchsorted(imu_t, gps_t, side="left")
    valid = (gps_indices >= 0) & (gps_indices < len(imu_t))
    gps_indices = gps_indices[valid]
    gps_values = gps_ned[valid]
    keep = np.r_[True, gps_indices[1:] != gps_indices[:-1]] if len(gps_indices) else np.asarray([])
    gps_indices = gps_indices[keep]
    gps_values = gps_values[keep]
    position_update[gps_indices] = True
    gps_position[gps_indices] = gps_values

    timestamp_us = np.rint((imu_t - imu_t[0]) * 1.0e6).astype(np.int64)
    output.parent.mkdir(parents=True, exist_ok=True)
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
    with output.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        for index in range(len(imu_t)):
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
                    4.0, 1.0, 0,
                ]
            )
    metadata: dict[str, object] = {
        **audit,
        "replay": {
            "samples": len(imu_t),
            "duration_s": float(timestamp_us[-1]) * 1.0e-6,
            "gps_position_updates": int(np.count_nonzero(position_update)),
            "gps_position_variance_m2": 4.0,
            "input_imu": "DJI OSDK IMU, FLU converted to FRD",
            "attitude_reference": "DJI OSDK onboard orientation converted ENU/FLU to NED/FRD",
            "navigation_reference": "separate RTK relative position and audited RTK NED velocity",
            "reference_kind": "external_reference_with_shared_onboard_attitude",
        },
    }
    if metadata_path is not None:
        metadata_path.parent.mkdir(parents=True, exist_ok=True)
        metadata_path.write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return metadata


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--metadata", type=Path)
    args = parser.parse_args()
    result = convert(args.bag, args.out, args.metadata)
    print(json.dumps(result["replay"], indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
