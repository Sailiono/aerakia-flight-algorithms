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

# The public replay CSV continues to publish calibrated rate/specific-force
# samples for the current ESKF API.  The v2 validation contract additionally
# records the equivalent interval deltas, so a private driver adapter can be
# audited without changing the portable public API prematurely.
MEASUREMENT_CONTRACT_LEGACY = "legacy_rate_sample_v1"
MEASUREMENT_CONTRACT_DELTA_INTERVAL = "delta_interval_v2"
STATIONARITY_SOURCE_LEGACY_PROFILE = "legacy_profile_truth"
STATIONARITY_SOURCE_CAUSAL_IMU = "causal_imu_window"
STATIONARITY_SOURCE_NONE = "none"

BIAS_OBSERVABILITY_TRAJECTORY_DURATIONS_S = {
    "bias_cv_hover_axis_pulses": 32.0,
    "bias_cv_takeoff_box_land": 38.0,
    "bias_cv_yaw_quadrant_hover": 40.0,
    "bias_cv_early_transition_s_curve": 42.0,
    "bias_cv_landing_gust_recovery": 36.0,
}


def _quintic_segment(
    time_s: np.ndarray,
    start_s: float,
    stop_s: float,
    start_position: np.ndarray,
    start_velocity: np.ndarray,
    start_acceleration: np.ndarray,
    stop_position: np.ndarray,
    stop_velocity: np.ndarray,
    stop_acceleration: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Evaluate a fifth-order Hermite segment with continuous p/v/a endpoints."""
    duration = stop_s - start_s
    if duration <= 0.0:
        raise ValueError("quintic segment duration must be positive")
    u = np.clip((time_s - start_s) / duration, 0.0, 1.0)
    c0 = np.asarray(start_position, dtype=np.float64)
    c1 = np.asarray(start_velocity, dtype=np.float64) * duration
    c2 = 0.5 * np.asarray(start_acceleration, dtype=np.float64) * duration * duration
    right_hand_side = np.vstack((
        np.asarray(stop_position, dtype=np.float64) - c0 - c1 - c2,
        np.asarray(stop_velocity, dtype=np.float64) * duration - c1 - 2.0 * c2,
        np.asarray(stop_acceleration, dtype=np.float64) * duration * duration - 2.0 * c2,
    ))
    coefficients = np.linalg.solve(
        np.array(((1.0, 1.0, 1.0), (3.0, 4.0, 5.0), (6.0, 12.0, 20.0))),
        right_hand_side,
    )
    c3, c4, c5 = coefficients
    position = (
        c0 + u[:, None] * (c1 + u[:, None] * (
            c2 + u[:, None] * (c3 + u[:, None] * (c4 + u[:, None] * c5))
        ))
    )
    velocity = (
        c1 + u[:, None] * (
            2.0 * c2 + u[:, None] * (3.0 * c3 + u[:, None] * (4.0 * c4 + u[:, None] * 5.0 * c5))
        )
    ) / duration
    acceleration = (
        2.0 * c2 + u[:, None] * (
            6.0 * c3 + u[:, None] * (12.0 * c4 + u[:, None] * 20.0 * c5)
        )
    ) / (duration * duration)
    return position, velocity, acceleration


def _piecewise_quintic(
    time_s: np.ndarray,
    waypoints: list[tuple[float, list[float], list[float]]],
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Evaluate vector waypoints whose acceleration is zero at every boundary."""
    dimension = len(waypoints[0][1])
    position = np.zeros((len(time_s), dimension), dtype=np.float64)
    velocity = np.zeros_like(position)
    acceleration = np.zeros_like(position)
    zero_acceleration = np.zeros(dimension, dtype=np.float64)
    for index in range(len(waypoints) - 1):
        start_time, start_position, start_velocity = waypoints[index]
        stop_time, stop_position, stop_velocity = waypoints[index + 1]
        is_last = index == len(waypoints) - 2
        selected = (time_s >= start_time) & (
            (time_s <= stop_time) if is_last else (time_s < stop_time)
        )
        segment_position, segment_velocity, segment_acceleration = _quintic_segment(
            time_s[selected],
            start_time,
            stop_time,
            np.asarray(start_position),
            np.asarray(start_velocity),
            zero_acceleration,
            np.asarray(stop_position),
            np.asarray(stop_velocity),
            zero_acceleration,
        )
        position[selected] = segment_position
        velocity[selected] = segment_velocity
        acceleration[selected] = segment_acceleration
    return position, velocity, acceleration


def _vtol_attitude_from_acceleration(
    acceleration_ned_m_s2: np.ndarray,
    yaw_deg: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Align body down with the ideal VTOL thrust direction for the requested yaw."""
    gravity = np.array((0.0, 0.0, GRAVITY_M_S2), dtype=np.float64)
    body_down_ned = gravity - acceleration_ned_m_s2
    body_down_ned /= np.linalg.norm(body_down_ned, axis=1)[:, None]
    yaw_rad = np.radians(yaw_deg)
    yaw_frame_x = np.cos(yaw_rad) * body_down_ned[:, 0] + np.sin(yaw_rad) * body_down_ned[:, 1]
    yaw_frame_y = -np.sin(yaw_rad) * body_down_ned[:, 0] + np.cos(yaw_rad) * body_down_ned[:, 1]
    roll_rad = np.arcsin(np.clip(-yaw_frame_y, -1.0, 1.0))
    pitch_rad = np.arctan2(yaw_frame_x, body_down_ned[:, 2])
    return np.degrees(roll_rad), np.degrees(pitch_rad)


def generate_bias_observability_trajectory(
    name: str,
    duration_s: float,
    rate_hz: float,
) -> tuple[
    np.ndarray, np.ndarray, np.ndarray, np.ndarray,
    np.ndarray, np.ndarray, np.ndarray, np.ndarray,
]:
    """Generate frozen, non-multisine VTOL trajectories for bias cross-validation."""
    expected_duration = BIAS_OBSERVABILITY_TRAJECTORY_DURATIONS_S.get(name)
    if expected_duration is None:
        raise ValueError(f"unsupported bias-observability trajectory: {name}")
    if not math.isclose(duration_s, expected_duration, rel_tol=0.0, abs_tol=1.0e-9):
        raise ValueError(
            f"{name} is frozen at {expected_duration:g} s, received {duration_s:g} s"
        )
    sample_count = max(2, int(round(duration_s * rate_hz)))
    time_s = np.arange(sample_count, dtype=np.float64) / rate_hz
    zero_velocity = [0.0, 0.0, 0.0]

    if name == "bias_cv_hover_axis_pulses":
        position_waypoints = [
            (0.0, [0.0, 0.0, 0.0], zero_velocity),
            (3.0, [0.0, 0.0, 0.0], zero_velocity),
            (7.0, [2.0, 0.0, 0.0], zero_velocity),
            (11.0, [0.0, 0.0, 0.0], zero_velocity),
            (15.0, [0.0, 2.0, 0.0], zero_velocity),
            (19.0, [0.0, 0.0, 0.0], zero_velocity),
            (23.0, [1.5, 1.5, 0.0], zero_velocity),
            (27.0, [0.0, 0.0, 0.0], zero_velocity),
            (32.0, [0.0, 0.0, 0.0], zero_velocity),
        ]
        yaw_waypoints = [(0.0, [0.0], [0.0]), (32.0, [0.0], [0.0])]
        static_flags = ((time_s < 3.0) | (time_s >= 27.0)).astype(np.int64)
    elif name == "bias_cv_takeoff_box_land":
        position_waypoints = [
            (0.0, [0.0, 0.0, 0.0], zero_velocity),
            (3.0, [0.0, 0.0, 0.0], zero_velocity),
            (7.0, [0.0, 0.0, -2.0], zero_velocity),
            (11.0, [2.0, 0.0, -2.0], zero_velocity),
            (15.0, [2.0, 2.0, -2.0], zero_velocity),
            (19.0, [0.0, 2.0, -2.0], zero_velocity),
            (23.0, [0.0, 0.0, -2.0], zero_velocity),
            (29.0, [0.0, 0.0, -2.0], zero_velocity),
            (35.0, [0.0, 0.0, 0.0], zero_velocity),
            (38.0, [0.0, 0.0, 0.0], zero_velocity),
        ]
        yaw_waypoints = [(0.0, [0.0], [0.0]), (38.0, [0.0], [0.0])]
        static_flags = ((time_s < 3.0) | (time_s >= 35.0)).astype(np.int64)
    elif name == "bias_cv_yaw_quadrant_hover":
        position_waypoints = [
            (0.0, [0.0, 0.0, 0.0], zero_velocity),
            (3.0, [0.0, 0.0, 0.0], zero_velocity),
            (4.5, [0.5, 0.0, 0.0], zero_velocity),
            (6.0, [0.0, 0.0, 0.0], zero_velocity),
            (10.0, [0.0, 0.0, 0.0], zero_velocity),
            (11.5, [0.5, 0.0, 0.0], zero_velocity),
            (13.0, [0.0, 0.0, 0.0], zero_velocity),
            (17.0, [0.0, 0.0, 0.0], zero_velocity),
            (18.5, [0.5, 0.0, 0.0], zero_velocity),
            (20.0, [0.0, 0.0, 0.0], zero_velocity),
            (24.0, [0.0, 0.0, 0.0], zero_velocity),
            (25.5, [0.5, 0.0, 0.0], zero_velocity),
            (27.0, [0.0, 0.0, 0.0], zero_velocity),
            (31.0, [0.0, 0.0, 0.0], zero_velocity),
            (40.0, [0.0, 0.0, 0.0], zero_velocity),
        ]
        yaw_waypoints = [
            (0.0, [0.0], [0.0]), (6.0, [0.0], [0.0]),
            (10.0, [90.0], [0.0]), (13.0, [90.0], [0.0]),
            (17.0, [180.0], [0.0]), (20.0, [180.0], [0.0]),
            (24.0, [270.0], [0.0]), (27.0, [270.0], [0.0]),
            (31.0, [360.0], [0.0]), (40.0, [360.0], [0.0]),
        ]
        static_flags = ((time_s < 3.0) | (time_s >= 31.0)).astype(np.int64)
    elif name == "bias_cv_early_transition_s_curve":
        position_waypoints = [
            (0.0, [0.0, 0.0, 0.0], zero_velocity),
            (3.0, [0.0, 0.0, 0.0], zero_velocity),
            (7.0, [0.0, 0.0, -2.0], zero_velocity),
            (13.0, [24.0, 0.0, -2.0], [8.0, 0.0, 0.0]),
            (23.0, [104.0, 10.0, -2.0], [7.0, 2.0, 0.0]),
            (31.0, [144.0, 20.0, -2.0], zero_velocity),
            (36.0, [144.0, 20.0, -2.0], zero_velocity),
            (40.0, [144.0, 20.0, 0.0], zero_velocity),
            (42.0, [144.0, 20.0, 0.0], zero_velocity),
        ]
        yaw_waypoints = [
            (0.0, [0.0], [0.0]), (13.0, [0.0], [0.0]),
            (23.0, [18.0], [0.0]), (31.0, [0.0], [0.0]),
            (42.0, [0.0], [0.0]),
        ]
        static_flags = ((time_s < 3.0) | (time_s >= 40.0)).astype(np.int64)
    else:
        position_waypoints = [
            (0.0, [0.0, 0.0, 0.0], zero_velocity),
            (3.0, [0.0, 0.0, 0.0], zero_velocity),
            (7.0, [0.0, 0.0, -3.0], zero_velocity),
            (12.0, [0.0, 0.0, -3.0], zero_velocity),
            (15.0, [0.8, -0.5, -2.5], zero_velocity),
            (18.0, [-0.7, 0.8, -2.0], zero_velocity),
            (21.0, [0.4, -0.6, -1.2], zero_velocity),
            (24.0, [0.0, 0.0, -0.5], zero_velocity),
            (28.0, [0.0, 0.0, 0.0], zero_velocity),
            (36.0, [0.0, 0.0, 0.0], zero_velocity),
        ]
        yaw_waypoints = [
            (0.0, [0.0], [0.0]), (12.0, [0.0], [0.0]),
            (18.0, [45.0], [0.0]), (24.0, [-30.0], [0.0]),
            (28.0, [0.0], [0.0]), (36.0, [0.0], [0.0]),
        ]
        static_flags = ((time_s < 3.0) | (time_s >= 28.0)).astype(np.int64)

    position_ned, velocity_ned, acceleration_ned = _piecewise_quintic(
        time_s, position_waypoints
    )
    yaw_values, _, _ = _piecewise_quintic(time_s, yaw_waypoints)
    yaw_deg = yaw_values[:, 0]
    roll_deg, pitch_deg = _vtol_attitude_from_acceleration(acceleration_ned, yaw_deg)
    return (
        time_s, roll_deg, pitch_deg, yaw_deg,
        acceleration_ned, velocity_ned, position_ned, static_flags,
    )


def interval_start_zoh_kinematics(
    time_s: np.ndarray,
    endpoint_acceleration_ned_m_s2: np.ndarray,
    initial_velocity_ned_m_s: np.ndarray,
    initial_position_ned_m: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Shift acceleration to interval-start ZOH and integrate matching endpoint truth.

    Row ``i`` (for ``i > 0``) contains the acceleration sampled at ``t[i-1]`` and
    held on ``(t[i-1], t[i]]``. Velocity and position at row ``i`` are integrated
    with that same held value. Row zero has no preceding interval and retains the
    first endpoint acceleration sample.
    """
    acceleration = np.asarray(endpoint_acceleration_ned_m_s2, dtype=np.float64).copy()
    if len(acceleration) > 1:
        acceleration[1:] = endpoint_acceleration_ned_m_s2[:-1]
    velocity = np.zeros_like(acceleration)
    position = np.zeros_like(acceleration)
    velocity[0] = np.asarray(initial_velocity_ned_m_s, dtype=np.float64)
    position[0] = np.asarray(initial_position_ned_m, dtype=np.float64)
    for index in range(1, len(time_s)):
        dt = float(time_s[index] - time_s[index - 1])
        velocity[index] = velocity[index - 1] + acceleration[index] * dt
        position[index] = (
            position[index - 1]
            + velocity[index - 1] * dt
            + 0.5 * acceleration[index] * dt * dt
        )
    return acceleration, velocity, position


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


def quaternion_to_body_to_ned_matrix(quaternion_wxyz: np.ndarray) -> np.ndarray:
    """Return the body-to-NED rotation for one scalar-first quaternion."""
    quaternion = np.asarray(quaternion_wxyz, dtype=np.float64)
    norm = float(np.linalg.norm(quaternion))
    if not math.isfinite(norm) or norm <= 1.0e-15:
        raise ValueError("quaternion must be finite and nonzero")
    w_value, x_value, y_value, z_value = quaternion / norm
    return np.array(
        [
            [
                1.0 - 2.0 * (y_value * y_value + z_value * z_value),
                2.0 * (x_value * y_value - z_value * w_value),
                2.0 * (x_value * z_value + y_value * w_value),
            ],
            [
                2.0 * (x_value * y_value + z_value * w_value),
                1.0 - 2.0 * (x_value * x_value + z_value * z_value),
                2.0 * (y_value * z_value - x_value * w_value),
            ],
            [
                2.0 * (x_value * z_value - y_value * w_value),
                2.0 * (y_value * z_value + x_value * w_value),
                1.0 - 2.0 * (x_value * x_value + y_value * y_value),
            ],
        ],
        dtype=np.float64,
    )


def quaternion_slerp(
    first_wxyz: np.ndarray, second_wxyz: np.ndarray, fraction: float
) -> np.ndarray:
    """Interpolate two body-to-NED attitudes along the shortest rotation."""
    first = np.asarray(first_wxyz, dtype=np.float64)
    second = np.asarray(second_wxyz, dtype=np.float64)
    first /= np.linalg.norm(first)
    second /= np.linalg.norm(second)
    dot_product = float(np.dot(first, second))
    if dot_product < 0.0:
        second = -second
        dot_product = -dot_product
    if dot_product > 0.9995:
        result = first + fraction * (second - first)
        return result / np.linalg.norm(result)
    angle = math.acos(np.clip(dot_product, -1.0, 1.0))
    sine = math.sin(angle)
    result = (
        math.sin((1.0 - fraction) * angle) / sine * first
        + math.sin(fraction * angle) / sine * second
    )
    return result / np.linalg.norm(result)


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


def interval_average_body_specific_force(
    time_s: np.ndarray,
    roll_deg: np.ndarray,
    pitch_deg: np.ndarray,
    yaw_deg: np.ndarray,
    interval_acceleration_ned_m_s2: np.ndarray,
) -> np.ndarray:
    """Integrate body specific force over each timestamped IMU interval.

    ``interval_acceleration_ned_m_s2[i]`` is held on ``(t[i-1], t[i]]``.  The
    body attitude is reconstructed from the two endpoint quaternions and
    integrated with five-point Gauss-Legendre quadrature.  This makes the
    generated delta-velocity semantic explicit and avoids mixing an endpoint
    NED acceleration with a different endpoint body attitude.
    """
    time = np.asarray(time_s, dtype=np.float64)
    acceleration_ned = np.asarray(interval_acceleration_ned_m_s2, dtype=np.float64)
    if acceleration_ned.shape != (len(time), 3):
        raise ValueError("interval acceleration must have one NED vector per timestamp")
    quaternions = euler_to_quaternion(
        np.radians(roll_deg), np.radians(pitch_deg), np.radians(yaw_deg)
    )
    result = np.empty_like(acceleration_ned)
    gravity_ned = np.array((0.0, 0.0, GRAVITY_M_S2), dtype=np.float64)
    result[0] = quaternion_to_body_to_ned_matrix(quaternions[0]).T @ (
        acceleration_ned[0] - gravity_ned
    )
    quadrature_x, quadrature_w = np.polynomial.legendre.leggauss(5)
    fractions = 0.5 * (quadrature_x + 1.0)
    weights = 0.5 * quadrature_w
    for index in range(1, len(time)):
        if time[index] <= time[index - 1]:
            raise ValueError("timestamps must be strictly increasing")
        sample = np.zeros(3, dtype=np.float64)
        for fraction, weight in zip(fractions, weights):
            rotation = quaternion_to_body_to_ned_matrix(
                quaternion_slerp(quaternions[index - 1], quaternions[index], float(fraction))
            )
            sample += weight * (rotation.T @ (acceleration_ned[index] - gravity_ned))
        result[index] = sample
    return result


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
    measurement_contract: str = MEASUREMENT_CONTRACT_LEGACY,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    roll_rad = np.radians(roll_deg)
    pitch_rad = np.radians(pitch_deg)
    yaw_rad = np.radians(yaw_deg)

    gyro_rad_s = interval_average_body_rate(
        time_s, roll_rad, pitch_rad, yaw_rad
    )

    if acceleration_ned_m_s2 is None:
        acceleration_ned_m_s2 = np.zeros((len(time_s), 3), dtype=np.float64)
    magnetic_field_ned_ut = np.array([22.0, 0.0, 44.0])
    ideal_mag = np.empty((len(time_s), 3), dtype=np.float64)
    if measurement_contract == MEASUREMENT_CONTRACT_LEGACY:
        gravity_ned = np.array([0.0, 0.0, GRAVITY_M_S2])
        ideal_accel = np.empty((len(time_s), 3), dtype=np.float64)
        for index in range(len(time_s)):
            rotation = body_to_ned_matrix(roll_rad[index], pitch_rad[index], yaw_rad[index])
            ideal_accel[index] = rotation.T @ (acceleration_ned_m_s2[index] - gravity_ned)
    elif measurement_contract == MEASUREMENT_CONTRACT_DELTA_INTERVAL:
        ideal_accel = interval_average_body_specific_force(
            time_s,
            roll_deg,
            pitch_deg,
            yaw_deg,
            acceleration_ned_m_s2,
        )
    else:
        raise ValueError(f"unsupported measurement contract: {measurement_contract}")

    for index in range(len(time_s)):
        rotation = body_to_ned_matrix(roll_rad[index], pitch_rad[index], yaw_rad[index])
        ideal_mag[index] = rotation.T @ magnetic_field_ned_ut

    measured_accel = ideal_accel + rng.normal(0.0, accel_noise_m_s2, ideal_accel.shape)
    measured_gyro = np.degrees(gyro_rad_s) + rng.normal(0.0, gyro_noise_deg_s, gyro_rad_s.shape)
    measured_mag = ideal_mag + rng.normal(0.0, mag_noise_ut, ideal_mag.shape)
    return measured_accel, measured_gyro, measured_mag, ideal_accel


def quantize_replay_measurements(
    acceleration_m_s2: np.ndarray,
    angular_rate_deg_s: np.ndarray,
    magnetic_field_ut: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Quantize exactly as the CSV transport and reconstruct its SI values."""
    acceleration = (
        np.rint(np.asarray(acceleration_m_s2, dtype=np.float64) / GRAVITY_M_S2 * 1000.0)
        * GRAVITY_M_S2 / 1000.0
    )
    angular_rate = np.rint(np.asarray(angular_rate_deg_s, dtype=np.float64) * 1000.0) / 1000.0
    magnetic_field = np.rint(np.asarray(magnetic_field_ut, dtype=np.float64) * 100.0) / 100.0
    return acceleration, angular_rate, magnetic_field


def causal_imu_stationarity_flags(
    timestamp_us: np.ndarray,
    acceleration_m_s2: np.ndarray,
    angular_rate_deg_s: np.ndarray,
    *,
    window_s: float,
    gyro_threshold_rad_s: float,
    acceleration_tolerance_m_s2: float,
) -> np.ndarray:
    """Return a past-only IMU stationarity indication for validation adapters.

    The detector deliberately sees only the same quantized IMU stream published
    to the replay CSV.  It cannot distinguish rest from constant-velocity
    translation; FCOne must combine its private armed/vehicle-state policy with
    this signal before using it for alignment or ZUPT.
    """
    timestamp = np.asarray(timestamp_us, dtype=np.int64)
    acceleration = np.asarray(acceleration_m_s2, dtype=np.float64)
    angular_rate = np.asarray(angular_rate_deg_s, dtype=np.float64)
    if len(timestamp) == 0 or acceleration.shape != (len(timestamp), 3) \
            or angular_rate.shape != (len(timestamp), 3):
        raise ValueError("stationarity inputs must contain equally sized timestamped vectors")
    if window_s <= 0.0 or gyro_threshold_rad_s <= 0.0 or acceleration_tolerance_m_s2 <= 0.0:
        raise ValueError("stationarity thresholds must be positive")
    if len(timestamp) > 1 and np.any(np.diff(timestamp) <= 0):
        raise ValueError("stationarity timestamps must be strictly increasing")

    gyro_norm_rad_s = np.linalg.norm(np.radians(angular_rate), axis=1)
    acceleration_norm = np.linalg.norm(acceleration, axis=1)
    result = np.zeros(len(timestamp), dtype=np.int64)
    start = 0
    for index in range(len(timestamp)):
        minimum_timestamp = int(timestamp[index] - round(window_s * 1.0e6))
        while start < index and timestamp[start] < minimum_timestamp:
            start += 1
        if timestamp[index] - timestamp[start] < int(round(window_s * 1.0e6)):
            continue
        gyro_window = gyro_norm_rad_s[start:index + 1]
        acceleration_window = acceleration_norm[start:index + 1]
        if (np.percentile(gyro_window, 95) <= gyro_threshold_rad_s
                and abs(float(np.mean(acceleration_window)) - GRAVITY_M_S2)
                <= acceleration_tolerance_m_s2):
            result[index] = 1
    return result


def interval_deltas_from_published_rates(
    timestamp_us: np.ndarray,
    acceleration_m_s2: np.ndarray,
    angular_rate_deg_s: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return delta angle/velocity implied by published interval-average rates."""
    timestamp = np.asarray(timestamp_us, dtype=np.int64)
    acceleration = np.asarray(acceleration_m_s2, dtype=np.float64)
    angular_rate = np.asarray(angular_rate_deg_s, dtype=np.float64)
    if len(timestamp) < 1 or acceleration.shape != (len(timestamp), 3) \
            or angular_rate.shape != (len(timestamp), 3):
        raise ValueError("delta inputs must contain equally sized timestamped vectors")
    interval_us = np.zeros(len(timestamp), dtype=np.int64)
    if len(timestamp) > 1:
        interval_us[1:] = np.diff(timestamp)
        if np.any(interval_us[1:] <= 0):
            raise ValueError("delta timestamps must be strictly increasing")
    interval_s = interval_us.astype(np.float64) * 1.0e-6
    delta_velocity = acceleration * interval_s[:, None]
    delta_angle = np.radians(angular_rate) * interval_s[:, None]
    return interval_us, delta_angle, delta_velocity


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
    delta_interval_us: np.ndarray | None = None,
    delta_angle_rad: np.ndarray | None = None,
    delta_velocity_m_s: np.ndarray | None = None,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    nominal_dt_us = int(round(1_000_000.0 / rate_hz))
    header = [
        "seq", "host_ts_us", "ts_us", "dt_us",
        "raw_acc_mg_x", "raw_acc_mg_y", "raw_acc_mg_z",
        "raw_gyro_mdps_x", "raw_gyro_mdps_y", "raw_gyro_mdps_z",
        "delta_interval_us", "delta_angle_rad_x", "delta_angle_rad_y", "delta_angle_rad_z",
        "delta_velocity_m_s_x", "delta_velocity_m_s_y", "delta_velocity_m_s_z",
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

    accel_m_s2, gyro_deg_s, magnetic_ut = quantize_replay_measurements(
        accel_m_s2, gyro_deg_s, magnetic_ut
    )

    if delta_interval_us is None or delta_angle_rad is None or delta_velocity_m_s is None:
        delta_interval_us, delta_angle_rad, delta_velocity_m_s = (
            interval_deltas_from_published_rates(timestamp_us, accel_m_s2, gyro_deg_s)
        )
    if (np.asarray(delta_interval_us).shape != (len(time_s),)
            or np.asarray(delta_angle_rad).shape != (len(time_s), 3)
            or np.asarray(delta_velocity_m_s).shape != (len(time_s), 3)):
        raise ValueError("delta fields must have exactly one interval/vector per timestamp")

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
                    int(delta_interval_us[index]),
                    *delta_angle_rad[index], *delta_velocity_m_s[index],
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


def draw_bounded_normal_vector(
    rng: np.random.Generator,
    standard_deviation: float,
    sigma_limit: float,
) -> np.ndarray:
    """Draw a three-axis Gaussian prior, optionally truncated per axis by rejection sampling."""
    if standard_deviation <= 0.0:
        return np.zeros(3, dtype=np.float64)
    values = rng.normal(0.0, standard_deviation, 3)
    if sigma_limit <= 0.0:
        return values
    bound = standard_deviation * sigma_limit
    outside = np.abs(values) > bound
    while np.any(outside):
        values[outside] = rng.normal(
            0.0, standard_deviation, int(np.count_nonzero(outside))
        )
        outside = np.abs(values) > bound
    return values


def resolve_bias_vector(
    rng: np.random.Generator,
    standard_deviation: float,
    sigma_limit: float,
    explicit_vector: list[float] | tuple[float, float, float] | np.ndarray | None,
) -> tuple[np.ndarray, str]:
    """Return an explicit three-axis bias or draw the configured random prior."""
    if explicit_vector is not None:
        values = np.asarray(explicit_vector, dtype=np.float64)
        if values.shape != (3,) or not np.all(np.isfinite(values)):
            raise ValueError("explicit bias vectors must contain three finite values")
        return values.copy(), "explicit_vector"
    return (
        draw_bounded_normal_vector(rng, standard_deviation, sigma_limit),
        "bounded_random_prior" if standard_deviation > 0.0 else "zero_default",
    )


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
            *BIAS_OBSERVABILITY_TRAJECTORY_DURATIONS_S,
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
        "--accel-bias-vector-m-s2", type=float, nargs=3, metavar=("X", "Y", "Z"),
        help="explicit constant accelerometer bias vector; overrides the random prior",
    )
    parser.add_argument(
        "--gyro-bias-vector-deg-s", type=float, nargs=3, metavar=("X", "Y", "Z"),
        help="explicit constant gyroscope bias vector; overrides the random prior",
    )
    parser.add_argument(
        "--bias-sigma-limit", type=float, default=0.0,
        help="optional per-axis truncation of the startup residual-bias prior; zero is unbounded",
    )
    parser.add_argument(
        "--timestamp-jitter-std-us", type=float, default=0.0,
        help="standard deviation of per-interval timestamp jitter; intervals remain monotonic",
    )
    parser.add_argument(
        "--accel-time-semantics",
        choices=("legacy_endpoint", "interval_start_zoh"),
        default="legacy_endpoint",
        help=(
            "legacy keeps acceleration evaluated at each row timestamp; interval_start_zoh "
            "shifts acceleration to the start of each propagation interval and reintegrates "
            "matching velocity/position truth"
        ),
    )
    parser.add_argument(
        "--measurement-contract",
        choices=(MEASUREMENT_CONTRACT_LEGACY, MEASUREMENT_CONTRACT_DELTA_INTERVAL),
        default=MEASUREMENT_CONTRACT_LEGACY,
        help=(
            "legacy publishes endpoint rate/specific-force semantics; delta_interval_v2 "
            "publishes interval-average rates plus auditable delta-angle/delta-velocity fields"
        ),
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
        "--accel-noise-density-m-s2-sqrt-hz", type=float,
        help="continuous accelerometer white-noise density for delta_interval_v2",
    )
    parser.add_argument(
        "--gyro-noise-density-rad-s-sqrt-hz", type=float,
        help="continuous gyroscope white-noise density for delta_interval_v2",
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
    parser.add_argument(
        "--stationarity-source",
        choices=(
            STATIONARITY_SOURCE_LEGACY_PROFILE,
            STATIONARITY_SOURCE_CAUSAL_IMU,
            STATIONARITY_SOURCE_NONE,
        ),
        default=STATIONARITY_SOURCE_LEGACY_PROFILE,
        help=(
            "source for static_hint: legacy profile truth for historical data, "
            "or a past-only quantized-IMU window for v2 validation"
        ),
    )
    parser.add_argument(
        "--stationarity-window-s", type=float, default=1.0,
        help="past-only IMU window used by causal_imu_window stationarity",
    )
    parser.add_argument(
        "--stationarity-gyro-threshold-rad-s", type=float, default=0.05,
        help="95th-percentile gyroscope norm limit for causal_imu_window stationarity",
    )
    parser.add_argument(
        "--stationarity-acceleration-tolerance-m-s2", type=float,
        default=0.20 * GRAVITY_M_S2,
        help="mean acceleration-norm gravity tolerance for causal_imu_window stationarity",
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
        args.bias_sigma_limit,
    )
    if any(value < 0.0 for value in non_negative):
        parser.error("noise, bias, and timestamp-jitter values must be non-negative")
    if args.mag_rate_hz is not None and (
        args.mag_rate_hz <= 0.0 or args.mag_rate_hz > args.rate
    ):
        parser.error("--mag-rate-hz must be positive and no greater than --rate")
    if args.stationarity_source == STATIONARITY_SOURCE_CAUSAL_IMU and not args.static_hint:
        parser.error("causal_imu_window requires --static-hint")
    if args.stationarity_source == STATIONARITY_SOURCE_NONE and args.static_hint:
        parser.error("--stationarity-source none cannot be combined with --static-hint")
    if args.measurement_contract == MEASUREMENT_CONTRACT_DELTA_INTERVAL:
        if args.accel_time_semantics != "interval_start_zoh":
            parser.error("delta_interval_v2 requires --accel-time-semantics interval_start_zoh")
        if args.timestamp_jitter_std_us != 0.0:
            parser.error("delta_interval_v2 v1 smoke keeps physical intervals exact; timestamp jitter is a later campaign")
        if (args.accel_noise_density_m_s2_sqrt_hz is None
                or args.gyro_noise_density_rad_s_sqrt_hz is None):
            parser.error("delta_interval_v2 requires both IMU noise densities")
    density_values = (
        args.accel_noise_density_m_s2_sqrt_hz,
        args.gyro_noise_density_rad_s_sqrt_hz,
    )
    if any(value is not None and value < 0.0 for value in density_values):
        parser.error("IMU noise densities must be non-negative")
    if (args.measurement_contract == MEASUREMENT_CONTRACT_LEGACY
            and any(value is not None for value in density_values)):
        parser.error("IMU noise densities are only defined for delta_interval_v2")

    rng = np.random.default_rng(args.seed)
    timing_rng = np.random.default_rng(args.seed ^ 0xA34A91)
    heading_rng = np.random.default_rng(args.seed ^ 0x5EAD1A6)
    gps_rng = np.random.default_rng(args.seed ^ 0x6A5A1D)
    bias_observability_motion = args.motion in BIAS_OBSERVABILITY_TRAJECTORY_DURATIONS_S
    if bias_observability_motion:
        (
            time_s, roll_deg, pitch_deg, yaw_deg,
            acceleration_ned, velocity_ned, position_ned, profile_static_flags,
        ) = generate_bias_observability_trajectory(args.motion, args.duration, args.rate)
    else:
        time_s, roll_deg, pitch_deg, yaw_deg = generate_motion(
            args.duration, args.rate, args.motion
        )
        acceleration_ned = np.zeros((len(time_s), 3), dtype=np.float64)
        velocity_ned = np.zeros_like(acceleration_ned)
        position_ned = np.zeros_like(acceleration_ned)
        profile_static_flags = np.zeros(len(time_s), dtype=np.int64)
        if args.motion == "navigation_outage":
            acceleration_ned, velocity_ned, position_ned = navigation_profile(
                time_s, args.rate
            )
        elif args.motion == "bias_excitation":
            acceleration_ned, velocity_ned, position_ned = bias_excitation_profile(
                time_s, args.rate
            )
    if args.accel_time_semantics == "interval_start_zoh":
        acceleration_ned, velocity_ned, position_ned = interval_start_zoh_kinematics(
            time_s,
            acceleration_ned,
            velocity_ned[0],
            position_ned[0],
        )
        if bias_observability_motion:
            roll_deg, pitch_deg = _vtol_attitude_from_acceleration(
                acceleration_ned, yaw_deg
            )
    if args.measurement_contract == MEASUREMENT_CONTRACT_DELTA_INTERVAL:
        accel_noise_m_s2 = args.accel_noise_density_m_s2_sqrt_hz * math.sqrt(args.rate)
        gyro_noise_deg_s = math.degrees(
            args.gyro_noise_density_rad_s_sqrt_hz * math.sqrt(args.rate)
        )
    else:
        accel_noise_m_s2 = args.accel_noise_m_s2
        gyro_noise_deg_s = args.gyro_noise_deg_s
    accel, gyro, magnetic, ideal_accel = synthesize_measurements(
        time_s,
        roll_deg,
        pitch_deg,
        yaw_deg,
        rng,
        accel_noise_m_s2=accel_noise_m_s2,
        gyro_noise_deg_s=gyro_noise_deg_s,
        mag_noise_ut=args.mag_noise_ut,
        acceleration_ned_m_s2=acceleration_ned,
        measurement_contract=args.measurement_contract,
    )
    # Preserve the historical deterministic random stream when injection is disabled.
    acceleration_bias, acceleration_bias_source = resolve_bias_vector(
        rng,
        args.accel_bias_std_m_s2,
        args.bias_sigma_limit,
        args.accel_bias_vector_m_s2,
    )
    gyroscope_bias, gyroscope_bias_source = resolve_bias_vector(
        rng,
        args.gyro_bias_std_deg_s,
        args.bias_sigma_limit,
        args.gyro_bias_vector_deg_s,
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
    accel, gyro, magnetic = quantize_replay_measurements(accel, gyro, magnetic)
    navigation_enabled = args.motion in ("navigation_outage", "bias_excitation") \
        or bias_observability_motion
    static_flags = np.full(len(time_s), int(args.static_hint), dtype=np.int64)
    if args.static_hint and args.stationarity_source == STATIONARITY_SOURCE_CAUSAL_IMU:
        static_flags = causal_imu_stationarity_flags(
            timestamp_us,
            accel,
            gyro,
            window_s=args.stationarity_window_s,
            gyro_threshold_rad_s=args.stationarity_gyro_threshold_rad_s,
            acceleration_tolerance_m_s2=args.stationarity_acceleration_tolerance_m_s2,
        )
    elif bias_observability_motion and args.static_hint:
        static_flags = profile_static_flags
    elif args.motion in ("navigation_outage", "bias_excitation", "heading_recovery") \
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
                    "schema_version": 2,
                    "motion": args.motion,
                    "measurement_contract": args.measurement_contract,
                    "delta_fields": {
                        "interval_column": "delta_interval_us",
                        "angle_columns": [
                            "delta_angle_rad_x", "delta_angle_rad_y", "delta_angle_rad_z"
                        ],
                        "velocity_columns": [
                            "delta_velocity_m_s_x", "delta_velocity_m_s_y",
                            "delta_velocity_m_s_z"
                        ],
                        "row_zero_interval_is_zero": True,
                    },
                    "acceleration_time_semantics": args.accel_time_semantics,
                    "truth_kinematics_time_semantics": (
                        "row_i_acceleration_held_on_previous_to_current_interval_with_matching_zoh_truth"
                        if args.accel_time_semantics == "interval_start_zoh"
                        else "legacy_acceleration_evaluated_at_row_timestamp"
                    ),
                    "seed": args.seed,
                    "sample_count": len(time_s),
                    "rate_hz": args.rate,
                    "acceleration_bias_m_s2": acceleration_bias.tolist(),
                    "gyroscope_bias_deg_s": gyroscope_bias.tolist(),
                    "acceleration_bias_source": acceleration_bias_source,
                    "gyroscope_bias_source": gyroscope_bias_source,
                    "accel_bias_std_m_s2": args.accel_bias_std_m_s2,
                    "gyro_bias_std_deg_s": args.gyro_bias_std_deg_s,
                    "bias_sigma_limit": args.bias_sigma_limit,
                    "timestamp_jitter_std_us": args.timestamp_jitter_std_us,
                    "accel_noise_m_s2": accel_noise_m_s2,
                    "gyro_noise_deg_s": gyro_noise_deg_s,
                    "accel_noise_density_m_s2_sqrt_hz": args.accel_noise_density_m_s2_sqrt_hz,
                    "gyro_noise_density_rad_s_sqrt_hz": args.gyro_noise_density_rad_s_sqrt_hz,
                    "mag_noise_ut": args.mag_noise_ut,
                    "mag_rate_hz": mag_rate_hz,
                    "gps_position_noise_m": args.gps_position_noise_m,
                    "gps_velocity_noise_m_s": args.gps_velocity_noise_m_s,
                    "rate_invariant_streams": args.rate_invariant_streams,
                    "stationarity_source": (
                        args.stationarity_source if args.static_hint
                        else STATIONARITY_SOURCE_NONE
                    ),
                    "truth_derived_stationarity": bool(
                        args.static_hint
                        and args.stationarity_source == STATIONARITY_SOURCE_LEGACY_PROFILE
                    ),
                    "stationarity_window_s": args.stationarity_window_s,
                    "stationarity_gyro_threshold_rad_s": args.stationarity_gyro_threshold_rad_s,
                    "stationarity_acceleration_tolerance_m_s2": (
                        args.stationarity_acceleration_tolerance_m_s2
                    ),
                    "stationarity_flagged_samples": int(np.count_nonzero(static_flags)),
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
