#!/usr/bin/env python3
"""Generate a deterministic, analyzer-compatible synthetic IMU dataset."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import numpy as np


GRAVITY_M_S2 = 9.80665


def generate_motion(
    duration_s: float,
    rate_hz: float,
    motion: str,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    sample_count = max(2, int(round(duration_s * rate_hz)))
    time_s = np.arange(sample_count, dtype=np.float64) / rate_hz
    roll_deg = np.zeros_like(time_s)
    pitch_deg = np.zeros_like(time_s)
    yaw_deg = np.zeros_like(time_s)

    if motion == "yaw_spin":
        yaw_deg = time_s * 30.0
    elif motion == "slow_sin":
        roll_deg = 10.0 * np.sin(2.0 * math.pi * 0.20 * time_s)
        pitch_deg = 6.0 * np.sin(2.0 * math.pi * 0.15 * time_s)
        yaw_deg = time_s * 10.0
    elif motion == "yaw_jump":
        yaw_deg = time_s * 5.0
        yaw_deg[sample_count // 2 :] += 30.0
    elif motion == "roll_flip":
        roll_deg = 170.0 * np.sin(2.0 * math.pi * 0.10 * time_s)
    else:
        raise ValueError(f"unsupported motion: {motion}")

    return time_s, roll_deg, pitch_deg, yaw_deg


def body_to_ned_matrix(roll_rad: float, pitch_rad: float, yaw_rad: float) -> np.ndarray:
    cr, sr = math.cos(roll_rad), math.sin(roll_rad)
    cp, sp = math.cos(pitch_rad), math.sin(pitch_rad)
    cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )


def synthesize_measurements(
    time_s: np.ndarray,
    roll_deg: np.ndarray,
    pitch_deg: np.ndarray,
    yaw_deg: np.ndarray,
    rng: np.random.Generator,
    accel_noise_m_s2: float,
    gyro_noise_deg_s: float,
    mag_noise_ut: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    roll_rad = np.radians(roll_deg)
    pitch_rad = np.radians(pitch_deg)
    yaw_rad = np.radians(yaw_deg)

    roll_rate = np.gradient(roll_rad, time_s)
    pitch_rate = np.gradient(pitch_rad, time_s)
    yaw_rate = np.gradient(yaw_rad, time_s)

    gyro_rad_s = np.column_stack(
        (
            roll_rate - np.sin(pitch_rad) * yaw_rate,
            np.cos(roll_rad) * pitch_rate
            + np.sin(roll_rad) * np.cos(pitch_rad) * yaw_rate,
            -np.sin(roll_rad) * pitch_rate
            + np.cos(roll_rad) * np.cos(pitch_rad) * yaw_rate,
        )
    )

    specific_force_ned = np.array([0.0, 0.0, -GRAVITY_M_S2])
    magnetic_field_ned_ut = np.array([22.0, 0.0, 44.0])
    ideal_accel = np.empty((len(time_s), 3), dtype=np.float64)
    ideal_mag = np.empty((len(time_s), 3), dtype=np.float64)
    for index in range(len(time_s)):
        rotation = body_to_ned_matrix(roll_rad[index], pitch_rad[index], yaw_rad[index])
        ideal_accel[index] = rotation.T @ specific_force_ned
        ideal_mag[index] = rotation.T @ magnetic_field_ned_ut

    measured_accel = ideal_accel + rng.normal(0.0, accel_noise_m_s2, ideal_accel.shape)
    measured_gyro = np.degrees(gyro_rad_s) + rng.normal(0.0, gyro_noise_deg_s, gyro_rad_s.shape)
    measured_mag = ideal_mag + rng.normal(0.0, mag_noise_ut, ideal_mag.shape)
    return measured_accel, measured_gyro, measured_mag, ideal_accel


def inject_magnetic_anomaly(
    magnetic_ut: np.ndarray,
    mode: str,
    start: int,
    length: int,
    rng: np.random.Generator,
) -> np.ndarray:
    flags = np.zeros(len(magnetic_ut), dtype=np.int64)
    end = min(len(magnetic_ut), start + length)
    if mode == "none":
        return flags

    if mode == "spike":
        magnetic_ut[start:end] += rng.uniform(-25.0, 25.0, (end - start, 3))
        flags[start:end] = 1
    elif mode == "bias":
        magnetic_ut[start:, 0] += 50.0
        magnetic_ut[start:, 1] += 25.0
        flags[start:] = 1
    elif mode == "drift":
        ramp = np.linspace(0.0, 50.0, len(magnetic_ut) - start)
        magnetic_ut[start:, 0] += ramp
        flags[start:] = 1
    else:
        raise ValueError(f"unsupported anomaly: {mode}")
    return flags


def write_golden_csv(
    path: Path,
    time_s: np.ndarray,
    roll_deg: np.ndarray,
    pitch_deg: np.ndarray,
    yaw_deg: np.ndarray,
    accel_m_s2: np.ndarray,
    gyro_deg_s: np.ndarray,
    magnetic_ut: np.ndarray,
    ideal_accel_m_s2: np.ndarray,
    anomaly_flags: np.ndarray,
    rate_hz: float,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    dt_us = int(round(1_000_000.0 / rate_hz))
    header = [
        "seq", "host_ts_us", "ts_us", "dt_us",
        "raw_acc_mg_x", "raw_acc_mg_y", "raw_acc_mg_z",
        "raw_gyro_mdps_x", "raw_gyro_mdps_y", "raw_gyro_mdps_z",
        "raw_mag_cuT_x", "raw_mag_cuT_y", "raw_mag_cuT_z",
        "g_est_mg_x", "g_est_mg_y", "g_est_mg_z",
        "innov_acc_milli", "innov_mag_milli", "acc_w_milli", "acc_n_milli",
        "roll_mdeg", "pitch_mdeg", "yaw_mdeg", "anomaly",
    ]

    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        for index, time_value in enumerate(time_s):
            ts_us = int(round(time_value * 1_000_000.0))
            wrapped_yaw = (yaw_deg[index] + 180.0) % 360.0 - 180.0
            writer.writerow(
                [
                    index, ts_us, ts_us, dt_us,
                    *np.rint(accel_m_s2[index] / GRAVITY_M_S2 * 1000.0).astype(int),
                    *np.rint(gyro_deg_s[index] * 1000.0).astype(int),
                    *np.rint(magnetic_ut[index] * 100.0).astype(int),
                    *np.rint(ideal_accel_m_s2[index] / GRAVITY_M_S2 * 1000.0).astype(int),
                    0, 0, 1000, 1000,
                    int(round(roll_deg[index] * 1000.0)),
                    int(round(pitch_deg[index] * 1000.0)),
                    int(round(wrapped_yaw * 1000.0)),
                    int(anomaly_flags[index]),
                ]
            )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True, help="output golden CSV path")
    parser.add_argument("--duration", type=float, default=20.0, help="duration in seconds")
    parser.add_argument("--rate", type=float, default=100.0, help="sample rate in Hz")
    parser.add_argument(
        "--motion",
        choices=["yaw_spin", "slow_sin", "yaw_jump", "roll_flip"],
        default="yaw_spin",
    )
    parser.add_argument("--anomaly", choices=["none", "spike", "bias", "drift"], default="none")
    parser.add_argument("--seed", type=int, default=0, help="random seed")
    args = parser.parse_args()

    if args.duration <= 0.0 or args.rate <= 0.0:
        parser.error("--duration and --rate must be positive")

    rng = np.random.default_rng(args.seed)
    time_s, roll_deg, pitch_deg, yaw_deg = generate_motion(args.duration, args.rate, args.motion)
    accel, gyro, magnetic, ideal_accel = synthesize_measurements(
        time_s,
        roll_deg,
        pitch_deg,
        yaw_deg,
        rng,
        accel_noise_m_s2=0.02,
        gyro_noise_deg_s=0.05,
        mag_noise_ut=0.20,
    )
    anomaly_flags = inject_magnetic_anomaly(
        magnetic,
        args.anomaly,
        start=len(time_s) // 3,
        length=max(1, int(round(args.rate * 0.5))),
        rng=rng,
    )
    write_golden_csv(
        args.out,
        time_s,
        roll_deg,
        pitch_deg,
        yaw_deg,
        accel,
        gyro,
        magnetic,
        ideal_accel,
        anomaly_flags,
        args.rate,
    )
    print(f"Wrote {len(time_s)} samples to {args.out}")


if __name__ == "__main__":
    main()
