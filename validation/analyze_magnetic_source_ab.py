#!/usr/bin/env python3
"""Audit physical magnetic-source behavior with a paired on/off estimator replay.

This tool is deliberately offline-only. It rotates recorded body magnetometer samples with the
independent reference attitude solely to characterize source behavior and score the paired replay.
Its output is not an estimator input, a source-quality decision, or a flight gate.
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


ROOT = Path(__file__).resolve().parents[1]
REPLAY_REQUIRED = (
    "ts_us", "raw_mag_cuT_x", "raw_mag_cuT_y", "raw_mag_cuT_z", "mag_valid",
    "mag_update", "magnetic_declination_rad", "ref_q_w", "ref_q_x", "ref_q_y",
    "ref_q_z", "raw_acc_mg_x", "raw_acc_mg_y", "raw_acc_mg_z", "raw_gyro_mdps_x",
    "raw_gyro_mdps_y", "raw_gyro_mdps_z",
)
RESULT_REQUIRED = (
    "ts_us", "input_mag_update", "eskf_mag_accepted", "eskf_mag_innovation_rad",
    "eskf_mag_test_ratio", "truth_roll_deg", "truth_pitch_deg", "truth_yaw_deg",
    "eskf_roll_deg", "eskf_pitch_deg", "eskf_yaw_deg", "truth_q_w", "truth_q_x",
    "truth_q_y", "truth_q_z", "eskf_q_w", "eskf_q_x", "eskf_q_y", "eskf_q_z",
)
RESIDUAL_BINS_DEG = (
    ("under_5_deg", 0.0, 5.0),
    ("5_to_15_deg", 5.0, 15.0),
    ("at_least_15_deg", 15.0, math.inf),
)
MDPS_TO_RAD_S = math.pi / 180000.0
GRAVITY_M_S2 = 9.80665
STATIC_ACCELERATION_TOLERANCE_G = 0.20
STATIC_GYRO_THRESHOLD_RAD_S = 0.05


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _read_csv(path: Path, required: tuple[str, ...]) -> list[dict[str, str]]:
    if not path.is_file():
        raise ValueError(f"missing CSV: {path}")
    with path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None:
            raise ValueError(f"missing CSV header: {path}")
        missing = [name for name in required if name not in reader.fieldnames]
        if missing:
            raise ValueError(f"{path} is missing columns: {', '.join(missing)}")
        rows = list(reader)
    if not rows:
        raise ValueError(f"CSV has no data rows: {path}")
    return rows


def _float_column(rows: list[dict[str, str]], name: str) -> np.ndarray:
    try:
        values = np.asarray([float(row[name]) for row in rows], dtype=np.float64)
    except (KeyError, ValueError) as error:
        raise ValueError(f"invalid numeric column {name}") from error
    if np.any(~np.isfinite(values)):
        raise ValueError(f"non-finite values in {name}")
    return values


def _wrap_degrees(values: np.ndarray) -> np.ndarray:
    return (values + 180.0) % 360.0 - 180.0


def _summary(values: np.ndarray) -> dict[str, float]:
    if values.size == 0 or np.any(~np.isfinite(values)):
        raise ValueError("cannot summarize empty or non-finite values")
    return {
        "median": float(np.median(values)),
        "p05": float(np.percentile(values, 5.0)),
        "p95": float(np.percentile(values, 95.0)),
        "minimum": float(np.min(values)),
        "maximum": float(np.max(values)),
        "rmse": float(np.sqrt(np.mean(values * values))),
    }


def _optional_summary(values: np.ndarray) -> dict[str, float] | None:
    return None if values.size == 0 else _summary(values)


def _normalized_quaternions(values: np.ndarray, label: str) -> np.ndarray:
    norms = np.linalg.norm(values, axis=1)
    if np.any(~np.isfinite(norms) | (norms <= 1.0e-9)):
        raise ValueError(f"invalid {label} quaternion")
    return values / norms[:, None]


def _rotate_body_to_ned(quaternion: np.ndarray, vector: np.ndarray) -> np.ndarray:
    w_value, x_value, y_value, z_value = quaternion.T
    vx, vy, vz = vector.T
    return np.column_stack((
        (1.0 - 2.0 * (y_value * y_value + z_value * z_value)) * vx
        + 2.0 * (x_value * y_value - w_value * z_value) * vy
        + 2.0 * (x_value * z_value + w_value * y_value) * vz,
        2.0 * (x_value * y_value + w_value * z_value) * vx
        + (1.0 - 2.0 * (x_value * x_value + z_value * z_value)) * vy
        + 2.0 * (y_value * z_value - w_value * x_value) * vz,
        2.0 * (x_value * z_value - w_value * y_value) * vx
        + 2.0 * (y_value * z_value + w_value * x_value) * vy
        + (1.0 - 2.0 * (x_value * x_value + y_value * y_value)) * vz,
    ))


def _rotate_by_rotation_vector(vector: np.ndarray, rotation_vector: np.ndarray) -> np.ndarray:
    angle = float(np.linalg.norm(rotation_vector))
    if angle <= 1.0e-12:
        return vector.copy()
    axis = rotation_vector / angle
    return (
        vector * math.cos(angle)
        + np.cross(axis, vector) * math.sin(angle)
        + axis * float(np.dot(axis, vector)) * (1.0 - math.cos(angle))
    )


def _gyro_propagated_direction_residual_deg(
    timestamp_us: np.ndarray,
    raw_magnetic_ut: np.ndarray,
    raw_angular_rate_rad_s: np.ndarray,
    magnetometer_update: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Measure causal consecutive magnetic direction mismatch in the body frame.

    A locally constant navigation-frame magnetic vector evolves in body coordinates as
    ``m_b(t + dt) = Exp(-omega_b * dt) m_b(t)``. The comparison propagates every IMU row
    after the prior valid magnetic sample, never using a reference attitude or future sample.
    """

    residual_deg = np.full(len(timestamp_us), np.nan, dtype=np.float64)
    interval_s = np.full(len(timestamp_us), np.nan, dtype=np.float64)
    propagated_direction: np.ndarray | None = None
    last_magnetometer_timestamp_us: float | None = None
    for index in range(len(timestamp_us)):
        if propagated_direction is not None and index > 0:
            dt_s = (timestamp_us[index] - timestamp_us[index - 1]) * 1.0e-6
            if not math.isfinite(float(dt_s)) or dt_s <= 0.0:
                raise ValueError("replay timestamps must be strictly increasing")
            propagated_direction = _rotate_by_rotation_vector(
                propagated_direction, -raw_angular_rate_rad_s[index - 1] * dt_s
            )
            propagated_norm = float(np.linalg.norm(propagated_direction))
            if propagated_norm <= 1.0e-9 or not math.isfinite(propagated_norm):
                raise ValueError("gyro propagation produced an invalid magnetic direction")
            propagated_direction /= propagated_norm
        if not magnetometer_update[index]:
            continue
        measured_norm = float(np.linalg.norm(raw_magnetic_ut[index]))
        if measured_norm <= 1.0e-9 or not math.isfinite(measured_norm):
            raise ValueError("physical magnetometer update has zero norm")
        measured_direction = raw_magnetic_ut[index] / measured_norm
        if propagated_direction is not None and last_magnetometer_timestamp_us is not None:
            cosine = float(np.clip(np.dot(propagated_direction, measured_direction), -1.0, 1.0))
            residual_deg[index] = math.degrees(math.acos(cosine))
            interval_s[index] = (timestamp_us[index] - last_magnetometer_timestamp_us) * 1.0e-6
        propagated_direction = measured_direction
        last_magnetometer_timestamp_us = timestamp_us[index]
    return residual_deg, interval_s


def _gravity_conditioned_inclination_proxy_deg(
    raw_magnetic_ut: np.ndarray,
    raw_acceleration_mg: np.ndarray,
    raw_angular_rate_rad_s: np.ndarray,
    magnetometer_update: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Return a yaw-independent magnetic-inclination proxy on stationary-contract samples.

    Under low linear acceleration, the specific-force direction is opposite gravity. The angle
    between it and the physical field is invariant to yaw and uses only the contemporaneous IMU.
    The condition exactly reuses the public static-alignment acceleration and gyro limits; it is
    not asserted to be valid while the vehicle is maneuvering.
    """

    acceleration_norm_m_s2 = np.linalg.norm(raw_acceleration_mg, axis=1) * GRAVITY_M_S2 / 1000.0
    gyro_norm_rad_s = np.linalg.norm(raw_angular_rate_rad_s, axis=1)
    valid_acceleration = acceleration_norm_m_s2 > 1.0e-9
    valid_magnetic = np.linalg.norm(raw_magnetic_ut, axis=1) > 1.0e-9
    condition = (
        magnetometer_update
        & valid_acceleration
        & valid_magnetic
        & (np.abs(acceleration_norm_m_s2 / GRAVITY_M_S2 - 1.0)
           <= STATIC_ACCELERATION_TOLERANCE_G)
        & (gyro_norm_rad_s <= STATIC_GYRO_THRESHOLD_RAD_S)
    )
    inclination_proxy_deg = np.full(len(raw_magnetic_ut), np.nan, dtype=np.float64)
    if np.any(condition):
        magnetic_unit = raw_magnetic_ut[condition] / np.linalg.norm(
            raw_magnetic_ut[condition], axis=1
        )[:, None]
        acceleration_unit = raw_acceleration_mg[condition] / np.linalg.norm(
            raw_acceleration_mg[condition], axis=1
        )[:, None]
        inclination_proxy_deg[condition] = np.degrees(np.arcsin(np.clip(
            -np.sum(magnetic_unit * acceleration_unit, axis=1), -1.0, 1.0
        )))
    return inclination_proxy_deg, condition, acceleration_norm_m_s2, gyro_norm_rad_s


def _tilt_error_deg(reference_q: np.ndarray, estimate_q: np.ndarray) -> np.ndarray:
    # R_nb maps body to NED. Its third row is the NED-down direction expressed
    # in body coordinates, so its angular difference isolates tilt from yaw.
    reference_body_down = np.column_stack((
        2.0 * (reference_q[:, 1] * reference_q[:, 3]
               - reference_q[:, 0] * reference_q[:, 2]),
        2.0 * (reference_q[:, 2] * reference_q[:, 3]
               + reference_q[:, 0] * reference_q[:, 1]),
        1.0 - 2.0 * (reference_q[:, 1] ** 2 + reference_q[:, 2] ** 2),
    ))
    estimate_body_down = np.column_stack((
        2.0 * (estimate_q[:, 1] * estimate_q[:, 3]
               - estimate_q[:, 0] * estimate_q[:, 2]),
        2.0 * (estimate_q[:, 2] * estimate_q[:, 3]
               + estimate_q[:, 0] * estimate_q[:, 1]),
        1.0 - 2.0 * (estimate_q[:, 1] ** 2 + estimate_q[:, 2] ** 2),
    ))
    cosine = np.sum(reference_body_down * estimate_body_down, axis=1)
    return np.degrees(np.arccos(np.clip(cosine, -1.0, 1.0)))


def _pearson(first: np.ndarray, second: np.ndarray) -> float | None:
    if len(first) < 3 or np.std(first) <= 1.0e-12 or np.std(second) <= 1.0e-12:
        return None
    return float(np.corrcoef(first, second)[0, 1])


def _check_timestamp_pairing(
    replay_rows: list[dict[str, str]], on_rows: list[dict[str, str]], off_rows: list[dict[str, str]]
) -> np.ndarray:
    if len(replay_rows) != len(on_rows) or len(replay_rows) != len(off_rows):
        raise ValueError("replay and paired result row counts differ")
    replay_time = _float_column(replay_rows, "ts_us")
    on_time = _float_column(on_rows, "ts_us")
    off_time = _float_column(off_rows, "ts_us")
    if np.any(np.diff(replay_time) <= 0.0):
        raise ValueError("replay timestamps must be strictly increasing")
    if not np.array_equal(replay_time, on_time) or not np.array_equal(replay_time, off_time):
        raise ValueError("paired replay/result timestamps differ")
    return replay_time


def analyze_magnetic_source_ab(
    replay_path: Path,
    magnetometer_on_results_path: Path,
    magnetometer_off_results_path: Path,
) -> dict[str, Any]:
    """Return offline physical-source and paired-estimator diagnostics."""

    replay_rows = _read_csv(replay_path, REPLAY_REQUIRED)
    on_rows = _read_csv(magnetometer_on_results_path, RESULT_REQUIRED)
    off_rows = _read_csv(magnetometer_off_results_path, RESULT_REQUIRED)
    timestamp_us = _check_timestamp_pairing(replay_rows, on_rows, off_rows)
    timestamp_s = (timestamp_us - timestamp_us[0]) * 1.0e-6

    magnetometer_update = (
        _float_column(replay_rows, "mag_valid") != 0.0
    ) & (_float_column(replay_rows, "mag_update") != 0.0)
    if not np.any(magnetometer_update):
        raise ValueError("replay contains no valid physical magnetometer updates")
    on_input_update = _float_column(on_rows, "input_mag_update") != 0.0
    off_input_update = _float_column(off_rows, "input_mag_update") != 0.0
    if not np.array_equal(on_input_update, magnetometer_update):
        raise ValueError("magnetometer-on results do not match replay update flags")
    if np.any(off_input_update):
        raise ValueError("magnetometer-off results contain input magnetometer updates")
    on_accepted = _float_column(on_rows, "eskf_mag_accepted") != 0.0
    off_accepted = _float_column(off_rows, "eskf_mag_accepted") != 0.0
    if np.any(on_accepted & ~on_input_update):
        raise ValueError("magnetometer-on results accept an update not present in the replay")
    if np.any(off_accepted):
        raise ValueError("magnetometer-off results accept a magnetometer update")

    datum_rad = _float_column(replay_rows, "magnetic_declination_rad")
    if np.max(np.abs(datum_rad - datum_rad[0])) > 1.0e-9:
        raise ValueError("declared magnetic datum changes within replay")
    reference_q = _normalized_quaternions(
        np.column_stack([_float_column(replay_rows, f"ref_q_{axis}") for axis in "wxyz"]),
        "reference",
    )
    raw_magnetic_ut = 0.01 * np.column_stack([
        _float_column(replay_rows, f"raw_mag_cuT_{axis}") for axis in "xyz"
    ])
    raw_acceleration_mg = np.column_stack([
        _float_column(replay_rows, f"raw_acc_mg_{axis}") for axis in "xyz"
    ])
    raw_angular_rate_rad_s = (MDPS_TO_RAD_S * np.column_stack([
        _float_column(replay_rows, f"raw_gyro_mdps_{axis}") for axis in "xyz"
    ]))
    magnetic_norm_ut = np.linalg.norm(raw_magnetic_ut, axis=1)
    if np.any(magnetic_norm_ut[magnetometer_update] <= 1.0e-9):
        raise ValueError("physical magnetometer update has zero norm")
    magnetic_ned_ut = _rotate_body_to_ned(reference_q, raw_magnetic_ut)
    horizontal_norm_ut = np.linalg.norm(magnetic_ned_ut[:, :2], axis=1)
    if np.any(horizontal_norm_ut[magnetometer_update] <= 1.0e-9):
        raise ValueError("physical magnetometer update has no horizontal component")
    physical_heading_deg = np.degrees(np.arctan2(magnetic_ned_ut[:, 1], magnetic_ned_ut[:, 0]))
    datum_residual_deg = _wrap_degrees(physical_heading_deg - np.degrees(datum_rad[0]))
    inclination_deg = np.degrees(np.arctan2(magnetic_ned_ut[:, 2], horizontal_norm_ut))
    direction_residual_deg, direction_interval_s = _gyro_propagated_direction_residual_deg(
        timestamp_us, raw_magnetic_ut, raw_angular_rate_rad_s, magnetometer_update
    )
    gravity_inclination_proxy_deg, gravity_conditioned, acceleration_norm_m_s2, gyro_norm_rad_s = (
        _gravity_conditioned_inclination_proxy_deg(
            raw_magnetic_ut,
            raw_acceleration_mg,
            raw_angular_rate_rad_s,
            magnetometer_update,
        )
    )

    truth_yaw = _float_column(on_rows, "truth_yaw_deg")
    on_yaw_error = _wrap_degrees(_float_column(on_rows, "eskf_yaw_deg") - truth_yaw)
    off_yaw_error = _wrap_degrees(_float_column(off_rows, "eskf_yaw_deg") - truth_yaw)
    on_q = _normalized_quaternions(
        np.column_stack([_float_column(on_rows, f"eskf_q_{axis}") for axis in "wxyz"]),
        "magnetometer-on ESKF",
    )
    off_q = _normalized_quaternions(
        np.column_stack([_float_column(off_rows, f"eskf_q_{axis}") for axis in "wxyz"]),
        "magnetometer-off ESKF",
    )
    on_tilt_error = _tilt_error_deg(reference_q, on_q)
    off_tilt_error = _tilt_error_deg(reference_q, off_q)
    innovation_deg = np.degrees(_float_column(on_rows, "eskf_mag_innovation_rad"))
    innovation_ratio = _float_column(on_rows, "eskf_mag_test_ratio")
    accepted = on_accepted

    selected = magnetometer_update
    causal_direction_selected = selected & np.isfinite(direction_residual_deg)
    gravity_selected = selected & gravity_conditioned
    absolute_residual = np.abs(datum_residual_deg)
    bins: dict[str, Any] = {}
    for label, lower, upper in RESIDUAL_BINS_DEG:
        mask = selected & (absolute_residual >= lower) & (absolute_residual < upper)
        bins[label] = {
            "lower_inclusive_deg": lower,
            "upper_exclusive_deg": None if math.isinf(upper) else upper,
            "samples": int(np.count_nonzero(mask)),
            "eskf_on_yaw_rmse_deg": (
                float(np.sqrt(np.mean(on_yaw_error[mask] ** 2))) if np.any(mask) else None
            ),
            "eskf_off_yaw_rmse_deg": (
                float(np.sqrt(np.mean(off_yaw_error[mask] ** 2))) if np.any(mask) else None
            ),
            "eskf_on_tilt_rmse_deg": (
                float(np.sqrt(np.mean(on_tilt_error[mask] ** 2))) if np.any(mask) else None
            ),
            "eskf_off_tilt_rmse_deg": (
                float(np.sqrt(np.mean(off_tilt_error[mask] ** 2))) if np.any(mask) else None
            ),
            "magnetometer_acceptance_ratio": (
                float(np.mean(accepted[mask])) if np.any(mask) else None
            ),
        }

    report: dict[str, Any] = {
        "schema_version": "aerakia.magnetic_source_ab_diagnostic.v1",
        "status": "completed_offline_truth_scored_diagnostic",
        "scope": (
            "Physical magnetometer source characterization and paired ESKF on/off comparison; "
            "not a runtime source-quality gate."
        ),
        "truth_boundary": (
            "Reference quaternions are used only to rotate recorded magnetometer samples into NED "
            "and score estimator output offline. Runtime quality candidates must not consume them."
        ),
        "provenance": {
            "replay_sha256": sha256_file(replay_path),
            "magnetometer_on_results_sha256": sha256_file(magnetometer_on_results_path),
            "magnetometer_off_results_sha256": sha256_file(magnetometer_off_results_path),
            "analyzer_sha256": sha256_file(Path(__file__).resolve()),
            "rows": len(replay_rows),
            "duration_s": float(timestamp_s[-1]),
        },
        "physical_magnetic_source": {
            "updates": int(np.count_nonzero(selected)),
            "declared_datum_deg": float(np.degrees(datum_rad[0])),
            "heading_minus_declared_datum_deg": _summary(datum_residual_deg[selected]),
            "field_norm_ut": _summary(magnetic_norm_ut[selected]),
            "inclination_deg_positive_down": _summary(inclination_deg[selected]),
        },
        "causal_source_features": {
            "gyro_propagated_direction_comparisons": int(np.count_nonzero(causal_direction_selected)),
            "gyro_propagated_direction_interval_s": _optional_summary(
                direction_interval_s[causal_direction_selected]
            ),
            "gyro_propagated_direction_residual_deg": _optional_summary(
                direction_residual_deg[causal_direction_selected]
            ),
            "gravity_conditioned_inclination_samples": int(np.count_nonzero(gravity_selected)),
            "gravity_conditioned_acceleration_norm_error_g": _optional_summary(
                np.abs(acceleration_norm_m_s2[gravity_selected] / GRAVITY_M_S2 - 1.0)
            ),
            "gravity_conditioned_gyro_norm_rad_s": _optional_summary(
                gyro_norm_rad_s[gravity_selected]
            ),
            "gravity_conditioned_inclination_proxy_deg_positive_down": _optional_summary(
                gravity_inclination_proxy_deg[gravity_selected]
            ),
        },
        "paired_estimator_effect": {
            "magnetometer_innovation_deg": _summary(innovation_deg[selected]),
            "magnetometer_test_ratio": _summary(innovation_ratio[selected]),
            "magnetometer_acceptance_ratio": float(np.mean(accepted[selected])),
            "eskf_on_yaw_error_deg": _summary(on_yaw_error[selected]),
            "eskf_off_yaw_error_deg": _summary(off_yaw_error[selected]),
            "eskf_on_tilt_error_deg": _summary(on_tilt_error[selected]),
            "eskf_off_tilt_error_deg": _summary(off_tilt_error[selected]),
            "on_minus_off_yaw_error_deg": _summary(
                _wrap_degrees(on_yaw_error[selected] - off_yaw_error[selected])
            ),
            "on_minus_off_tilt_error_deg": _summary(
                on_tilt_error[selected] - off_tilt_error[selected]
            ),
        },
        "offline_truth_scored_relationships": {
            "physical_heading_residual_vs_on_yaw_error_pearson": _pearson(
                datum_residual_deg[selected], on_yaw_error[selected]
            ),
            "absolute_physical_heading_residual_vs_absolute_on_minus_off_yaw_pearson": _pearson(
                absolute_residual[selected],
                np.abs(_wrap_degrees(on_yaw_error[selected] - off_yaw_error[selected])),
            ),
            "absolute_physical_heading_residual_vs_on_minus_off_tilt_pearson": _pearson(
                absolute_residual[selected], on_tilt_error[selected] - off_tilt_error[selected]
            ),
            "gyro_propagated_direction_residual_vs_absolute_physical_heading_residual_pearson": _pearson(
                direction_residual_deg[causal_direction_selected],
                absolute_residual[causal_direction_selected],
            ),
            "gyro_propagated_direction_residual_vs_absolute_on_minus_off_yaw_pearson": _pearson(
                direction_residual_deg[causal_direction_selected],
                np.abs(_wrap_degrees(
                    on_yaw_error[causal_direction_selected]
                    - off_yaw_error[causal_direction_selected]
                )),
            ),
            "gravity_conditioned_inclination_proxy_vs_physical_inclination_pearson": _pearson(
                gravity_inclination_proxy_deg[gravity_selected], inclination_deg[gravity_selected]
            ),
            "gravity_conditioned_inclination_proxy_vs_absolute_on_minus_off_yaw_pearson": _pearson(
                gravity_inclination_proxy_deg[gravity_selected],
                np.abs(_wrap_degrees(
                    on_yaw_error[gravity_selected] - off_yaw_error[gravity_selected]
                )),
            ),
        },
        "predeclared_residual_bins": bins,
        "limitations": [
            "A persistent magnetic datum offset is indistinguishable from yaw error without an independent heading source.",
            "Correlation is descriptive, not causal attribution.",
            "This diagnostic does not qualify a runtime threshold or authorize source rejection.",
            "The existing ESKF magnetic update is a horizontal yaw pseudo observation, not a full 3D magnetic-vector update.",
        ],
    }
    return report


def _strict_json(value: object) -> object:
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {str(key): _strict_json(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_strict_json(item) for item in value]
    return value


def write_report(report: dict[str, Any], output_directory: Path) -> None:
    output_directory.mkdir(parents=True, exist_ok=True)
    (output_directory / "summary.json").write_text(
        json.dumps(_strict_json(report), indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    effect = report["paired_estimator_effect"]
    source = report["physical_magnetic_source"]
    causal = report["causal_source_features"]
    direction_summary = causal["gyro_propagated_direction_residual_deg"]
    direction_p95 = "n/a" if direction_summary is None else f"{direction_summary['p95']:.6f}"
    direction_rmse = "n/a" if direction_summary is None else f"{direction_summary['rmse']:.6f}"
    inclination_summary = causal["gravity_conditioned_inclination_proxy_deg_positive_down"]
    inclination_p95 = "n/a" if inclination_summary is None else f"{inclination_summary['p95']:.6f}"
    lines = [
        "# Magnetic source A/B diagnostic", "",
        "This is an offline truth-scored source diagnostic, not a runtime rejection policy.", "",
        f"Physical magnetometer updates: **{source['updates']}**.", "",
        "| Causal physical feature | Value |",
        "| --- | ---: |",
        f"| Gyro-propagated direction comparisons | {causal['gyro_propagated_direction_comparisons']} |",
        f"| Direction residual P95 (deg) | {direction_p95} |",
        f"| Direction residual RMSE (deg) | {direction_rmse} |",
        f"| Gravity-conditioned inclination samples | "
        f"{causal['gravity_conditioned_inclination_samples']} |",
        f"| Gravity-conditioned inclination proxy P95 (deg) | {inclination_p95} |",
        "",
        "| Metric | Magnetometer on | Magnetometer off |",
        "| --- | ---: | ---: |",
        f"| ESKF yaw-error RMSE (deg) | {effect['eskf_on_yaw_error_deg']['rmse']:.6f} | "
        f"{effect['eskf_off_yaw_error_deg']['rmse']:.6f} |",
        f"| ESKF tilt-error RMSE (deg) | {effect['eskf_on_tilt_error_deg']['rmse']:.6f} | "
        f"{effect['eskf_off_tilt_error_deg']['rmse']:.6f} |",
        "",
        "| Absolute physical-heading residual bin | Samples | On yaw RMSE | Off yaw RMSE | On tilt RMSE | Off tilt RMSE | Accepted |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for label, values in report["predeclared_residual_bins"].items():
        def format_value(value: object) -> str:
            return "n/a" if value is None else f"{float(value):.6f}"
        lines.append(
            f"| {label} | {values['samples']} | {format_value(values['eskf_on_yaw_rmse_deg'])} | "
            f"{format_value(values['eskf_off_yaw_rmse_deg'])} | "
            f"{format_value(values['eskf_on_tilt_rmse_deg'])} | "
            f"{format_value(values['eskf_off_tilt_rmse_deg'])} | "
            f"{format_value(values['magnetometer_acceptance_ratio'])} |"
        )
    lines.extend(["", "Complete hashes, distributions, correlations, and limitations are in `summary.json`."])
    (output_directory / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("replay_csv", type=Path)
    parser.add_argument("magnetometer_on_results_csv", type=Path)
    parser.add_argument("magnetometer_off_results_csv", type=Path)
    parser.add_argument("--out-dir", type=Path, required=True)
    args = parser.parse_args()
    report = analyze_magnetic_source_ab(
        args.replay_csv, args.magnetometer_on_results_csv, args.magnetometer_off_results_csv
    )
    write_report(report, args.out_dir)
    print(f"Magnetic source A/B report: {args.out_dir / 'report.md'}")


if __name__ == "__main__":
    main()
