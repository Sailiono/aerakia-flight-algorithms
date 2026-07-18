#!/usr/bin/env python3
"""Generate a deterministic, analyzer-compatible synthetic IMU dataset."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np


GRAVITY_M_S2 = 9.80665


def jittered_timestamps_us(
    sample_count: int,
    rate_hz: float,
    jitter_std_us: float,
    rng: np.random.Generator,
) -> np.ndarray:
    nominal_dt_us = 1_000_000.0 / rate_hz
    if jitter_std_us <= 0.0:
        return np.rint(np.arange(sample_count, dtype=np.float64) * nominal_dt_us).astype(np.int64)
    increments = nominal_dt_us + rng.normal(0.0, jitter_std_us, max(0, sample_count - 1))
    increments = np.maximum(increments, 0.25 * nominal_dt_us)
    timestamps = np.zeros(sample_count, dtype=np.float64)
    if sample_count > 1:
        timestamps[1:] = np.cumsum(increments)
    return np.rint(timestamps).astype(np.int64)


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
    elif motion == "static_tilted":
        roll_deg.fill(25.0)
        pitch_deg.fill(-18.0)
        yaw_deg.fill(35.0)
    elif motion == "navigation_outage":
        pass
    elif motion == "bias_excitation":
        active_time = np.maximum(time_s - 2.0, 0.0)
        ramp = np.clip(active_time / 2.0, 0.0, 1.0)
        roll_deg = ramp * 15.0 * np.sin(2.0 * math.pi * 0.17 * active_time)
        pitch_deg = ramp * 12.0 * np.sin(2.0 * math.pi * 0.13 * active_time)
        yaw_deg = ramp * 25.0 * np.sin(2.0 * math.pi * 0.09 * active_time)
    elif motion == "heading_recovery":
        yaw_deg = np.where(
            time_s < 2.0,
            0.0,
            np.where(
                time_s < 6.0,
                20.0 * (time_s - 2.0),
                np.where(
                    time_s < 10.0,
                    80.0,
                    np.where(time_s < 14.0, 80.0 - 15.0 * (time_s - 10.0), 20.0),
                ),
            ),
        )
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


def euler_to_quaternion(
    roll_rad: np.ndarray, pitch_rad: np.ndarray, yaw_rad: np.ndarray
) -> np.ndarray:
    """Return scalar-first body-to-NED quaternions for the requested samples."""
    half_roll = 0.5 * roll_rad
    half_pitch = 0.5 * pitch_rad
    half_yaw = 0.5 * yaw_rad
    cr, sr = np.cos(half_roll), np.sin(half_roll)
    cp, sp = np.cos(half_pitch), np.sin(half_pitch)
    cy, sy = np.cos(half_yaw), np.sin(half_yaw)
    return np.column_stack((
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ))


def quaternion_multiply(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return np.array((
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    ))


def interval_average_body_rate(
    time_s: np.ndarray, roll_rad: np.ndarray, pitch_rad: np.ndarray, yaw_rad: np.ndarray
) -> np.ndarray:
    """Generate rate samples whose delta angles exactly match the reference poses.

    Row zero carries no preceding interval and is therefore zero. Each later row
    is the constant body rate over `(t[i-1], t[i]]`, matching the timestamped IMU
    sample contract consumed by the native runner.
    """
    quaternions = euler_to_quaternion(roll_rad, pitch_rad, yaw_rad)
    rates = np.zeros((len(time_s), 3), dtype=np.float64)
    for index in range(1, len(time_s)):
        previous_conjugate = quaternions[index - 1].copy()
        previous_conjugate[1:] *= -1.0
        delta = quaternion_multiply(previous_conjugate, quaternions[index])
        if delta[0] < 0.0:
            delta *= -1.0
        vector_norm = float(np.linalg.norm(delta[1:]))
        if vector_norm <= 1.0e-15:
            continue
        angle = 2.0 * math.atan2(vector_norm, float(delta[0]))
        rates[index] = delta[1:] * (angle / vector_norm) / (
            time_s[index] - time_s[index - 1]
        )
    return rates


def synthesize_measurements(
    time_s: np.ndarray,
    roll_deg: np.ndarray,
    pitch_deg: np.ndarray,
    yaw_deg: np.ndarray,
    rng: np.random.Generator,
    accel_noise_m_s2: float,
    gyro_noise_deg_s: float,
    mag_noise_ut: float,
    acceleration_ned_m_s2: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    roll_rad = np.radians(roll_deg)
    pitch_rad = np.radians(pitch_deg)
    yaw_rad = np.radians(yaw_deg)

    gyro_rad_s = interval_average_body_rate(
        time_s, roll_rad, pitch_rad, yaw_rad
    )

    if acceleration_ned_m_s2 is None:
        acceleration_ned_m_s2 = np.zeros((len(time_s), 3), dtype=np.float64)
    gravity_ned = np.array([0.0, 0.0, GRAVITY_M_S2])
    magnetic_field_ned_ut = np.array([22.0, 0.0, 44.0])
    ideal_accel = np.empty((len(time_s), 3), dtype=np.float64)
    ideal_mag = np.empty((len(time_s), 3), dtype=np.float64)
    for index in range(len(time_s)):
        rotation = body_to_ned_matrix(roll_rad[index], pitch_rad[index], yaw_rad[index])
        ideal_accel[index] = rotation.T @ (acceleration_ned_m_s2[index] - gravity_ned)
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
    static_flags: np.ndarray,
    position_ned_m: np.ndarray,
    velocity_ned_m_s: np.ndarray,
    gps_position_ned_m: np.ndarray,
    gps_velocity_ned_m_s: np.ndarray,
    gps_updates: np.ndarray,
    position_reference_valid: np.ndarray,
    timestamp_us: np.ndarray,
    magnetometer_valid: np.ndarray,
    magnetometer_updates: np.ndarray,
    heading_valid: np.ndarray,
    heading_updates: np.ndarray,
    heading_rad: np.ndarray,
    heading_variance_rad2: np.ndarray,
    heading_fault: np.ndarray,
    acceleration_bias_m_s2: np.ndarray,
    gyroscope_bias_deg_s: np.ndarray,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    nominal_dt_us = int(round(1_000_000.0 / rate_hz))
    header = [
        "seq", "host_ts_us", "ts_us", "dt_us",
        "raw_acc_mg_x", "raw_acc_mg_y", "raw_acc_mg_z",
        "raw_gyro_mdps_x", "raw_gyro_mdps_y", "raw_gyro_mdps_z",
        "raw_mag_cuT_x", "raw_mag_cuT_y", "raw_mag_cuT_z",
        "g_est_mg_x", "g_est_mg_y", "g_est_mg_z",
        "innov_acc_milli", "innov_mag_milli", "acc_w_milli", "acc_n_milli",
        "roll_mdeg", "pitch_mdeg", "yaw_mdeg", "anomaly", "static_hint",
        "position_update", "position_ref_valid",
        "ref_position_n_m", "ref_position_e_m", "ref_position_d_m",
        "ref_velocity_n_m_s", "ref_velocity_e_m_s", "ref_velocity_d_m_s",
        "gps_position_n_m", "gps_position_e_m", "gps_position_d_m",
        "gps_velocity_n_m_s", "gps_velocity_e_m_s", "gps_velocity_d_m_s",
        "gps_position_variance_m2", "gps_velocity_variance_m2_s2",
        "mag_valid", "mag_update",
        "gnss_heading_valid", "gnss_heading_update", "gnss_heading_rad",
        "gnss_heading_variance_rad2", "gnss_heading_fault",
        "truth_accel_bias_x_m_s2", "truth_accel_bias_y_m_s2", "truth_accel_bias_z_m_s2",
        "truth_gyro_bias_x_rad_s", "truth_gyro_bias_y_rad_s", "truth_gyro_bias_z_rad_s",
    ]

    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        for index, time_value in enumerate(time_s):
            ts_us = int(timestamp_us[index])
            dt_us = nominal_dt_us if index == 0 else ts_us - int(timestamp_us[index - 1])
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
                    int(static_flags[index]),
                    int(gps_updates[index]), int(position_reference_valid[index]),
                    *position_ned_m[index], *velocity_ned_m_s[index],
                    *gps_position_ned_m[index], *gps_velocity_ned_m_s[index],
                    0.25, 0.01,
                    int(magnetometer_valid[index]), int(magnetometer_updates[index]),
                    int(heading_valid[index]), int(heading_updates[index]), heading_rad[index],
                    heading_variance_rad2[index], int(heading_fault[index]),
                    *acceleration_bias_m_s2,
                    *np.radians(gyroscope_bias_deg_s),
                ]
            )


def trusted_heading_profile(
    time_s: np.ndarray,
    yaw_deg: np.ndarray,
    rate_hz: float,
    rng: np.random.Generator,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    update_period = max(1, int(round(rate_hz / 10.0)))
    update = (np.arange(len(time_s)) % update_period == 0)
    valid = time_s >= 1.0
    dropout = (time_s >= 8.0) & (time_s < 12.0)
    valid &= ~dropout
    update &= valid
    fault = update & (time_s >= 6.0) & (time_s < 6.2)
    heading = np.radians(yaw_deg) + rng.normal(0.0, math.radians(1.0), len(time_s))
    heading[fault] += math.radians(90.0)
    heading = (heading + math.pi) % (2.0 * math.pi) - math.pi
    variance = np.full(len(time_s), math.radians(2.0) ** 2, dtype=np.float64)
    return valid.astype(np.int64), update.astype(np.int64), heading, variance, fault.astype(np.int64)


def navigation_profile(time_s: np.ndarray, rate_hz: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    acceleration = np.zeros((len(time_s), 3), dtype=np.float64)
    acceleration[(time_s >= 2.0) & (time_s < 4.0), 0] = 0.8
    acceleration[(time_s >= 6.0) & (time_s < 8.0), 1] = 0.5
    acceleration[(time_s >= 10.0) & (time_s < 12.0), 0] = -0.8
    acceleration[(time_s >= 14.0) & (time_s < 16.0), 1] = -0.5
    dt = 1.0 / rate_hz
    velocity = np.zeros_like(acceleration)
    position = np.zeros_like(acceleration)
    for index in range(1, len(time_s)):
        velocity[index] = velocity[index - 1] + acceleration[index - 1] * dt
        position[index] = (
            position[index - 1] + velocity[index - 1] * dt
            + 0.5 * acceleration[index - 1] * dt * dt
        )
    return acceleration, velocity, position


def bias_excitation_profile(
    time_s: np.ndarray, rate_hz: float
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Generate bounded multi-axis specific-force excitation after static alignment."""
    active_time = np.maximum(time_s - 2.0, 0.0)
    ramp = np.clip(active_time / 2.0, 0.0, 1.0)
    acceleration = np.column_stack(
        (
            ramp * 0.8 * np.sin(2.0 * math.pi * 0.11 * active_time),
            ramp * 0.6 * np.sin(2.0 * math.pi * 0.07 * active_time + 0.4),
            ramp * 0.35 * np.sin(2.0 * math.pi * 0.05 * active_time + 0.8),
        )
    )
    acceleration[time_s < 2.0] = 0.0
    dt = 1.0 / rate_hz
    velocity = np.zeros_like(acceleration)
    position = np.zeros_like(acceleration)
    for index in range(1, len(time_s)):
        velocity[index] = velocity[index - 1] + acceleration[index - 1] * dt
        position[index] = (
            position[index - 1] + velocity[index - 1] * dt
            + 0.5 * acceleration[index - 1] * dt * dt
        )
    return acceleration, velocity, position


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True, help="output golden CSV path")
    parser.add_argument("--duration", type=float, default=20.0, help="duration in seconds")
    parser.add_argument("--rate", type=float, default=100.0, help="sample rate in Hz")
    parser.add_argument(
        "--motion",
        choices=[
            "yaw_spin", "slow_sin", "yaw_jump", "roll_flip", "static_tilted",
            "navigation_outage", "bias_excitation", "heading_recovery",
        ],
        default="yaw_spin",
    )
    parser.add_argument("--anomaly", choices=["none", "spike", "bias", "drift"], default="none")
    parser.add_argument(
        "--static-hint", action="store_true",
        help="mark samples as application-confirmed stationary for alignment and ZUPT",
    )
    parser.add_argument("--seed", type=int, default=0, help="random seed")
    parser.add_argument(
        "--accel-bias-std-m-s2", type=float, default=0.0,
        help="standard deviation used to draw one constant three-axis accelerometer bias",
    )
    parser.add_argument(
        "--gyro-bias-std-deg-s", type=float, default=0.0,
        help="standard deviation used to draw one constant three-axis gyroscope bias",
    )
    parser.add_argument(
        "--timestamp-jitter-std-us", type=float, default=0.0,
        help="standard deviation of per-interval timestamp jitter; intervals remain monotonic",
    )
    parser.add_argument(
        "--accel-noise-m-s2", type=float, default=0.02,
        help="per-sample accelerometer noise standard deviation",
    )
    parser.add_argument(
        "--gyro-noise-deg-s", type=float, default=0.05,
        help="per-sample gyroscope noise standard deviation",
    )
    parser.add_argument(
        "--mag-noise-ut", type=float, default=0.20,
        help="per-update magnetometer noise standard deviation",
    )
    parser.add_argument(
        "--mag-rate-hz", type=float,
        help="magnetometer publication rate; defaults to the IMU rate",
    )
    parser.add_argument(
        "--gps-position-noise-m", type=float, default=0.5,
        help="GNSS position noise standard deviation",
    )
    parser.add_argument(
        "--gps-velocity-noise-m-s", type=float, default=0.1,
        help="GNSS velocity noise standard deviation",
    )
    parser.add_argument(
        "--rate-invariant-streams", action="store_true",
        help=(
            "draw GNSS noise only at its fixed-rate update epochs so the same seed "
            "represents the same aiding stream at different IMU rates"
        ),
    )
    parser.add_argument("--metadata", type=Path, help="optional JSON generation manifest")
    parser.add_argument(
        "--trusted-heading", action="store_true",
        help="generate a 10 Hz trusted-heading stream with dropout and outlier events",
    )
    parser.add_argument(
        "--disable-magnetometer", action="store_true",
        help="mark magnetometer samples invalid to isolate other heading sources",
    )
    args = parser.parse_args()

    if args.duration <= 0.0 or args.rate <= 0.0:
        parser.error("--duration and --rate must be positive")
    non_negative = (
        args.accel_bias_std_m_s2,
        args.gyro_bias_std_deg_s,
        args.timestamp_jitter_std_us,
        args.accel_noise_m_s2,
        args.gyro_noise_deg_s,
        args.mag_noise_ut,
        args.gps_position_noise_m,
        args.gps_velocity_noise_m_s,
    )
    if any(value < 0.0 for value in non_negative):
        parser.error("noise, bias, and timestamp-jitter values must be non-negative")
    if args.mag_rate_hz is not None and (
        args.mag_rate_hz <= 0.0 or args.mag_rate_hz > args.rate
    ):
        parser.error("--mag-rate-hz must be positive and no greater than --rate")

    rng = np.random.default_rng(args.seed)
    timing_rng = np.random.default_rng(args.seed ^ 0xA34A91)
    heading_rng = np.random.default_rng(args.seed ^ 0x5EAD1A6)
    gps_rng = np.random.default_rng(args.seed ^ 0x6A5A1D)
    time_s, roll_deg, pitch_deg, yaw_deg = generate_motion(args.duration, args.rate, args.motion)
    acceleration_ned = np.zeros((len(time_s), 3), dtype=np.float64)
    velocity_ned = np.zeros_like(acceleration_ned)
    position_ned = np.zeros_like(acceleration_ned)
    if args.motion == "navigation_outage":
        acceleration_ned, velocity_ned, position_ned = navigation_profile(time_s, args.rate)
    elif args.motion == "bias_excitation":
        acceleration_ned, velocity_ned, position_ned = bias_excitation_profile(time_s, args.rate)
    accel, gyro, magnetic, ideal_accel = synthesize_measurements(
        time_s,
        roll_deg,
        pitch_deg,
        yaw_deg,
        rng,
        accel_noise_m_s2=args.accel_noise_m_s2,
        gyro_noise_deg_s=args.gyro_noise_deg_s,
        mag_noise_ut=args.mag_noise_ut,
        acceleration_ned_m_s2=acceleration_ned,
    )
    # Preserve the historical deterministic random stream when injection is disabled.
    acceleration_bias = (
        rng.normal(0.0, args.accel_bias_std_m_s2, 3)
        if args.accel_bias_std_m_s2 > 0.0 else np.zeros(3, dtype=np.float64)
    )
    gyroscope_bias = (
        rng.normal(0.0, args.gyro_bias_std_deg_s, 3)
        if args.gyro_bias_std_deg_s > 0.0 else np.zeros(3, dtype=np.float64)
    )
    accel += acceleration_bias
    gyro += gyroscope_bias
    timestamp_us = jittered_timestamps_us(
        len(time_s), args.rate, args.timestamp_jitter_std_us, timing_rng
    )
    anomaly_flags = inject_magnetic_anomaly(
        magnetic,
        args.anomaly,
        start=len(time_s) // 3,
        length=max(1, int(round(args.rate * 0.5))),
        rng=rng,
    )
    navigation_enabled = args.motion in ("navigation_outage", "bias_excitation")
    static_flags = np.full(len(time_s), int(args.static_hint), dtype=np.int64)
    if args.motion in ("navigation_outage", "bias_excitation", "heading_recovery") \
            and args.static_hint:
        static_flags = (time_s < 1.5).astype(np.int64)
    gps_updates = np.zeros(len(time_s), dtype=np.int64)
    position_reference_valid = np.full(len(time_s), int(navigation_enabled), dtype=np.int64)
    gps_position = np.zeros_like(position_ned)
    gps_velocity = np.zeros_like(velocity_ned)
    if navigation_enabled:
        gps_period = max(1, int(round(args.rate / 10.0)))
        available = np.ones(len(time_s), dtype=bool)
        if args.motion == "navigation_outage":
            available = (time_s < 6.0) | (time_s >= 11.0)
        gps_updates = ((np.arange(len(time_s)) % gps_period == 0) & available).astype(np.int64)
        if args.rate_invariant_streams:
            update_indices = np.flatnonzero(gps_updates)
            gps_position = position_ned.copy()
            gps_velocity = velocity_ned.copy()
            gps_position[update_indices] += gps_rng.normal(
                0.0, args.gps_position_noise_m, (len(update_indices), 3)
            )
            gps_velocity[update_indices] += gps_rng.normal(
                0.0, args.gps_velocity_noise_m_s, (len(update_indices), 3)
            )
        else:
            gps_position = position_ned + rng.normal(
                0.0, args.gps_position_noise_m, position_ned.shape
            )
            gps_velocity = velocity_ned + rng.normal(
                0.0, args.gps_velocity_noise_m_s, velocity_ned.shape
            )
    magnetometer_valid = np.full(len(time_s), int(not args.disable_magnetometer), dtype=np.int64)
    mag_rate_hz = args.rate if args.mag_rate_hz is None else args.mag_rate_hz
    magnetometer_period = max(1, int(round(args.rate / mag_rate_hz)))
    magnetometer_updates = (
        np.arange(len(time_s)) % magnetometer_period == 0
    ).astype(np.int64)
    heading_valid = np.zeros(len(time_s), dtype=np.int64)
    heading_updates = np.zeros(len(time_s), dtype=np.int64)
    heading_rad = np.zeros(len(time_s), dtype=np.float64)
    heading_variance_rad2 = np.ones(len(time_s), dtype=np.float64)
    heading_fault = np.zeros(len(time_s), dtype=np.int64)
    if args.trusted_heading:
        heading_valid, heading_updates, heading_rad, heading_variance_rad2, heading_fault = (
            trusted_heading_profile(time_s, yaw_deg, args.rate, heading_rng)
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
        static_flags,
        position_ned,
        velocity_ned,
        gps_position,
        gps_velocity,
        gps_updates,
        position_reference_valid,
        timestamp_us,
        magnetometer_valid,
        magnetometer_updates,
        heading_valid,
        heading_updates,
        heading_rad,
        heading_variance_rad2,
        heading_fault,
        acceleration_bias,
        gyroscope_bias,
    )
    if args.metadata is not None:
        args.metadata.parent.mkdir(parents=True, exist_ok=True)
        args.metadata.write_text(
            json.dumps(
                {
                    "schema_version": 1,
                    "seed": args.seed,
                    "sample_count": len(time_s),
                    "rate_hz": args.rate,
                    "acceleration_bias_m_s2": acceleration_bias.tolist(),
                    "gyroscope_bias_deg_s": gyroscope_bias.tolist(),
                    "accel_bias_std_m_s2": args.accel_bias_std_m_s2,
                    "gyro_bias_std_deg_s": args.gyro_bias_std_deg_s,
                    "timestamp_jitter_std_us": args.timestamp_jitter_std_us,
                    "accel_noise_m_s2": args.accel_noise_m_s2,
                    "gyro_noise_deg_s": args.gyro_noise_deg_s,
                    "mag_noise_ut": args.mag_noise_ut,
                    "mag_rate_hz": mag_rate_hz,
                    "gps_position_noise_m": args.gps_position_noise_m,
                    "gps_velocity_noise_m_s": args.gps_velocity_noise_m_s,
                    "rate_invariant_streams": args.rate_invariant_streams,
                    "timestamp_interval_min_us": int(np.min(np.diff(timestamp_us))),
                    "timestamp_interval_max_us": int(np.max(np.diff(timestamp_us))),
                    "trusted_heading_updates": int(np.count_nonzero(heading_updates)),
                    "trusted_heading_fault_updates": int(np.count_nonzero(heading_fault)),
                    "trusted_heading_dropout_samples": int(np.count_nonzero(
                        args.trusted_heading and (heading_valid == 0)
                    )),
                },
                indent=2,
                sort_keys=True,
            ) + "\n",
            encoding="utf-8",
        )
    print(f"Wrote {len(time_s)} samples to {args.out}")


if __name__ == "__main__":
    main()
