#!/usr/bin/env python3
"""Causal fixed-lag tilt/accelerometer-bias proposal solver.

This module deliberately performs no ESKF state injection.  It reconstructs a
five-dimensional, prior-regularized MAP proposal from one closed replay window
using only the baseline filter trajectory, IMU samples, and actually accepted
GNSS position/velocity observations.  Truth is not loaded by the solver.

The proposal is an intermediate validation artifact.  It must demonstrate
stable behavior under a pre-registered campaign before a repropagating C
candidate is considered.
"""

from __future__ import annotations

import csv
import hashlib
import json
import math
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import numpy as np

import analyze_bias_excitation_information as analyzer


TARGET_LABELS = analyzer.TARGET_LABELS
TARGET_INDICES = analyzer.TARGET_INDICES
STATE_SCALES = analyzer.STATE_SCALES


@dataclass(frozen=True)
class ResidualStream:
    """Causal analyzer inputs plus measurements needed to form innovations."""

    samples: analyzer.ExcitationSamples
    position_measurement_ned_m: np.ndarray
    velocity_measurement_ned_m_s: np.ndarray
    estimated_position_ned_m: np.ndarray
    estimated_velocity_ned_m_s: np.ndarray
    result_rows: tuple[dict[str, str], ...]


def _read_rows(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None:
            raise ValueError(f"missing CSV header: {path}")
        return list(reader)


def _vectors(rows: Sequence[dict[str, str]], names: tuple[str, str, str]) -> np.ndarray:
    try:
        values = np.asarray([[float(row[name]) for name in names] for row in rows], dtype=np.float64)
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"missing or invalid vector columns: {names}") from error
    if not np.all(np.isfinite(values)):
        raise ValueError(f"non-finite vector columns: {names}")
    return values


def load_residual_stream(replay_path: Path, results_path: Path) -> ResidualStream:
    """Load a timestamp-paired replay/results pair without reading truth fields."""

    samples = analyzer.load_csv_pair(replay_path, results_path)
    replay_rows = _read_rows(replay_path)
    result_rows = _read_rows(results_path)
    if len(replay_rows) != len(result_rows) or len(replay_rows) != len(samples.timestamp_s):
        raise ValueError("replay, results, and analyzer sample counts must agree")
    stream = ResidualStream(
        samples=samples,
        position_measurement_ned_m=_vectors(
            replay_rows, ("gps_position_n_m", "gps_position_e_m", "gps_position_d_m")
        ),
        velocity_measurement_ned_m_s=_vectors(
            replay_rows, ("gps_velocity_n_m_s", "gps_velocity_e_m_s", "gps_velocity_d_m_s")
        ),
        estimated_position_ned_m=_vectors(
            result_rows, ("eskf_position_n_m", "eskf_position_e_m", "eskf_position_d_m")
        ),
        estimated_velocity_ned_m_s=_vectors(
            result_rows, ("eskf_velocity_n_m_s", "eskf_velocity_e_m_s", "eskf_velocity_d_m_s")
        ),
        result_rows=tuple(result_rows),
    )
    for accepted, measurement, estimate, variance, label in (
        (
            samples.position_accepted,
            stream.position_measurement_ned_m,
            stream.estimated_position_ned_m,
            samples.position_variance_m2,
            "position",
        ),
        (
            samples.velocity_accepted,
            stream.velocity_measurement_ned_m_s,
            stream.estimated_velocity_ned_m_s,
            samples.velocity_variance_m2_s2,
            "velocity",
        ),
    ):
        selected = np.asarray(accepted, dtype=bool)
        if not np.all(np.isfinite(measurement[selected])) or not np.all(np.isfinite(estimate[selected])):
            raise ValueError(f"accepted {label} values must be finite")
        if np.any(~np.isfinite(variance[selected])) or np.any(variance[selected] <= 0.0):
            raise ValueError(f"accepted {label} variances must be finite and positive")
    return stream


def whitened_residual_rows(stream: ResidualStream, start_index: int, stop_index: int) -> np.ndarray:
    """Return residuals in exactly the private analyzer design-row order."""

    samples = stream.samples
    rows: list[np.ndarray] = []
    for index in range(start_index + 1, stop_index + 1):
        if bool(samples.position_accepted[index]):
            rows.extend(
                (stream.position_measurement_ned_m[index] - stream.estimated_position_ned_m[index])
                / math.sqrt(float(samples.position_variance_m2[index]))
            )
        if bool(samples.velocity_accepted[index]):
            rows.extend(
                (stream.velocity_measurement_ned_m_s[index] - stream.estimated_velocity_ned_m_s[index])
                / math.sqrt(float(samples.velocity_variance_m2_s2[index]))
            )
    return np.asarray(rows, dtype=np.float64)


def target_prior_covariance_physical(result_row: dict[str, str]) -> np.ndarray:
    """Extract the reviewed 5x5 right-tilt/accelerometer-bias covariance."""

    covariance = np.zeros((5, 5), dtype=np.float64)
    covariance[0, 0] = float(result_row["eskf_right_error_tilt_cov_xx_rad2"])
    covariance[0, 1] = covariance[1, 0] = float(result_row["eskf_right_error_tilt_cov_xy_rad2"])
    covariance[1, 1] = float(result_row["eskf_right_error_tilt_cov_yy_rad2"])
    bias_names = {
        (0, 0): "eskf_accel_bias_cov_xx_m2_s4",
        (0, 1): "eskf_accel_bias_cov_xy_m2_s4",
        (0, 2): "eskf_accel_bias_cov_xz_m2_s4",
        (1, 1): "eskf_accel_bias_cov_yy_m2_s4",
        (1, 2): "eskf_accel_bias_cov_yz_m2_s4",
        (2, 2): "eskf_accel_bias_cov_zz_m2_s4",
    }
    for (row, column), name in bias_names.items():
        covariance[2 + row, 2 + column] = covariance[2 + column, 2 + row] = float(result_row[name])
    cross_names = (
        (
            "eskf_right_error_tilt_accel_bias_cov_x_x_rad_m_s2",
            "eskf_right_error_tilt_accel_bias_cov_x_y_rad_m_s2",
            "eskf_right_error_tilt_accel_bias_cov_x_z_rad_m_s2",
        ),
        (
            "eskf_right_error_tilt_accel_bias_cov_y_x_rad_m_s2",
            "eskf_right_error_tilt_accel_bias_cov_y_y_rad_m_s2",
            "eskf_right_error_tilt_accel_bias_cov_y_z_rad_m_s2",
        ),
    )
    for row in range(2):
        for column in range(3):
            covariance[row, 2 + column] = covariance[2 + column, row] = float(
                result_row[cross_names[row][column]]
            )
    if not np.all(np.isfinite(covariance)):
        raise ValueError("target covariance must be finite")
    covariance = 0.5 * (covariance + covariance.T)
    if float(np.min(np.linalg.eigvalsh(covariance))) <= 0.0:
        raise ValueError("target covariance must be positive definite")
    return covariance


def _project_target_and_residual(
    design: np.ndarray, residual: np.ndarray, nuisance_relative_tolerance: float
) -> tuple[np.ndarray, np.ndarray, int]:
    if design.ndim != 2 or design.shape[1] != analyzer.STATE_DIMENSION:
        raise ValueError("design must have 15 columns")
    if residual.shape != (design.shape[0],):
        raise ValueError("residual length must match design rows")
    scaled = design * STATE_SCALES[np.newaxis, :]
    target = scaled[:, TARGET_INDICES]
    nuisance = scaled[:, analyzer.NUISANCE_INDICES]
    singular_values = np.linalg.svd(nuisance, compute_uv=False)
    cutoff = (
        nuisance_relative_tolerance * singular_values[0]
        if singular_values.size and singular_values[0] > 0.0
        else nuisance_relative_tolerance
    )
    nuisance_rank = int(np.count_nonzero(singular_values > cutoff))
    if nuisance_rank == 0:
        return target, residual, 0
    basis, _, _ = np.linalg.svd(nuisance, full_matrices=False)
    basis = basis[:, :nuisance_rank]
    return target - basis @ (basis.T @ target), residual - basis @ (basis.T @ residual), nuisance_rank


def solve_window(
    stream: ResidualStream,
    start_index: int,
    stop_index: int,
    *,
    prior_information_scale: float,
    score_threshold: float,
    thresholds: analyzer.InformationThresholds = analyzer.InformationThresholds(),
) -> dict[str, object]:
    """Compute one closed-window proposal, keeping the baseline filter immutable."""

    if prior_information_scale <= 0.0 or not math.isfinite(prior_information_scale):
        raise ValueError("prior_information_scale must be finite and positive")
    design, _, coverage_counts = analyzer._observation_rows(
        stream.samples, start_index, stop_index
    )
    residual = whitened_residual_rows(stream, start_index, stop_index)
    if design.shape[0] != len(residual):
        raise ValueError("design/residual row order or count mismatch")
    projected_design, projected_residual, nuisance_rank = _project_target_and_residual(
        design, residual, thresholds.nuisance_relative_tolerance
    )
    information = projected_design.T @ projected_design
    information = 0.5 * (information + information.T)
    eigenvalues = np.linalg.eigvalsh(information)
    largest = float(max(eigenvalues[-1], 0.0))
    rank_cutoff = max(thresholds.rank_absolute_tolerance, thresholds.rank_relative_tolerance * largest)
    effective_rank = int(np.count_nonzero(eigenvalues > rank_cutoff))
    minimum_eigenvalue = float(max(eigenvalues[0], 0.0))
    score_pass = effective_rank == len(TARGET_LABELS) and minimum_eigenvalue >= score_threshold
    result: dict[str, object] = {
        "start_index": start_index,
        "stop_index": stop_index,
        "start_time_s": float(stream.samples.timestamp_s[start_index]),
        "stop_time_s": float(stream.samples.timestamp_s[stop_index]),
        "measurement_row_count": int(design.shape[0]),
        "nuisance_rank": nuisance_rank,
        "effective_rank": effective_rank,
        "minimum_eigenvalue": minimum_eigenvalue,
        "score_threshold": score_threshold,
        "score_pass": score_pass,
        "prior_information_scale": prior_information_scale,
        "accepted_position_updates": int(coverage_counts["position_accepted"]),
        "accepted_velocity_updates": int(coverage_counts["velocity_accepted"]),
    }
    if not score_pass:
        result["reason"] = "score_or_rank_gate_not_met"
        return result
    physical_prior = target_prior_covariance_physical(stream.result_rows[start_index])
    scales = np.diag(STATE_SCALES[TARGET_INDICES])
    normalized_prior = np.linalg.solve(scales, physical_prior) @ np.linalg.inv(scales)
    normalized_prior = 0.5 * (normalized_prior + normalized_prior.T)
    prior_information = np.linalg.inv(normalized_prior) * prior_information_scale
    posterior_information = information + prior_information
    posterior_information = 0.5 * (posterior_information + posterior_information.T)
    normalized_correction = np.linalg.solve(posterior_information, projected_design.T @ projected_residual)
    normalized_posterior_covariance = np.linalg.inv(posterior_information)
    physical_correction = STATE_SCALES[TARGET_INDICES] * normalized_correction
    physical_posterior_covariance = scales @ normalized_posterior_covariance @ scales
    result.update(
        {
            "reason": "proposal_computed_no_state_injection",
            "target_order": list(TARGET_LABELS),
            "correction_physical": physical_correction.tolist(),
            "correction_normalized": normalized_correction.tolist(),
            "posterior_covariance_diagonal_physical": np.diag(physical_posterior_covariance).tolist(),
            "posterior_covariance_minimum_eigenvalue": float(
                np.min(np.linalg.eigvalsh(physical_posterior_covariance))
            ),
        }
    )
    return result


def evaluate_timeline(
    stream: ResidualStream,
    *,
    window_duration_s: float,
    evaluation_interval_s: float,
    prior_information_scale: float,
    score_threshold: float,
) -> list[dict[str, object]]:
    """Evaluate only trailing windows closed at or before each timestamp."""

    if window_duration_s <= 0.0 or evaluation_interval_s <= 0.0:
        raise ValueError("window duration and evaluation interval must be positive")
    timestamps = stream.samples.timestamp_s
    evaluations: list[dict[str, object]] = []
    next_evaluation = float(timestamps[0]) + evaluation_interval_s
    for stop_index in range(1, len(timestamps)):
        stop_time = float(timestamps[stop_index])
        if stop_time + 1.0e-12 < next_evaluation and stop_index != len(timestamps) - 1:
            continue
        start_index = int(np.searchsorted(timestamps, stop_time - window_duration_s, side="left"))
        if start_index < stop_index:
            evaluations.append(
                solve_window(
                    stream,
                    start_index,
                    stop_index,
                    prior_information_scale=prior_information_scale,
                    score_threshold=score_threshold,
                )
            )
        while next_evaluation <= stop_time + 1.0e-12:
            next_evaluation += evaluation_interval_s
    return evaluations


def fingerprint(path: Path) -> dict[str, object]:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return {"sha256": digest.hexdigest(), "bytes": path.stat().st_size}


def main() -> int:
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--replay", type=Path, required=True)
    parser.add_argument("--results", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--window-duration-s", type=float, default=20.0)
    parser.add_argument("--evaluation-interval-s", type=float, default=0.5)
    parser.add_argument("--prior-information-scale", type=float, default=1.0)
    parser.add_argument("--score-threshold", type=float, default=0.004936251852866821)
    args = parser.parse_args()
    stream = load_residual_stream(args.replay, args.results)
    evaluations = evaluate_timeline(
        stream,
        window_duration_s=args.window_duration_s,
        evaluation_interval_s=args.evaluation_interval_s,
        prior_information_scale=args.prior_information_scale,
        score_threshold=args.score_threshold,
    )
    computed = [item for item in evaluations if item["score_pass"]]
    result = {
        "schema_version": 1,
        "status": "proposal_only_no_estimator_injection",
        "method": "causal_fixed_lag_map_joint_tilt_accelerometer_bias",
        "truth_used_by_solver": False,
        "inputs": {"replay": fingerprint(args.replay), "results": fingerprint(args.results)},
        "window_duration_s": args.window_duration_s,
        "evaluation_interval_s": args.evaluation_interval_s,
        "prior_information_scale": args.prior_information_scale,
        "score_threshold": args.score_threshold,
        "evaluation_count": len(evaluations),
        "proposal_count": len(computed),
        "first_proposal_time_s": computed[0]["stop_time_s"] if computed else None,
        "evaluations": evaluations,
        "limitations": [
            "No correction is injected or repropagated through the baseline ESKF.",
            "This proposal lacks process/preintegration covariance across the full lag window.",
            "Truth is excluded from the solver but may be used by a separate evaluator.",
        ],
        "git_commit": subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=Path(__file__).resolve().parents[1], text=True
        ).strip(),
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
