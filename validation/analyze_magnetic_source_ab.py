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
    lines = [
        "# Magnetic source A/B diagnostic", "",
        "This is an offline truth-scored source diagnostic, not a runtime rejection policy.", "",
        f"Physical magnetometer updates: **{source['updates']}**.", "",
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
