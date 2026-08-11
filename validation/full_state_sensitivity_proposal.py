#!/usr/bin/env python3
"""Host-only full 15D sensitivity/replay diagnostic.

This is a Stage-B diagnostic following the rejected five-dimensional candidate.
It includes all nuisance states in the local solve and uses the complete 15x15
covariance captured at the window boundary.  It is still not a smoother: the
lag process/preintegration covariance is not reconstructed, and the result is
never promoted automatically.
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import tempfile
from pathlib import Path

import numpy as np

import fixed_lag_sensitivity_proposal as base


ROOT = Path(__file__).resolve().parents[1]
PARAMETER_LABELS = (
    "right_tilt_x_rad", "right_tilt_y_rad", "right_tilt_z_rad",
    "velocity_n_m_s", "velocity_e_m_s", "velocity_d_m_s",
    "position_n_m", "position_e_m", "position_d_m",
    "accel_bias_x_m_s2", "accel_bias_y_m_s2", "accel_bias_z_m_s2",
    "gyro_bias_x_rad_s", "gyro_bias_y_rad_s", "gyro_bias_z_rad_s",
)
# Reviewed physical scales are close to the nominal covariance magnitudes and
# only normalize the solve; they are not a product tuning profile.
PARAMETER_SCALES = np.asarray(
    [0.05, 0.05, 0.10, 1.0, 1.0, 1.0, 10.0, 10.0, 10.0,
     0.15, 0.15, 0.15, 0.01, 0.01, 0.01], dtype=np.float64
)
PARAMETER_EPSILON = np.asarray(
    [1.0e-4, 1.0e-4, 1.0e-4, 1.0e-3, 1.0e-3, 1.0e-3,
     1.0e-3, 1.0e-3, 1.0e-3, 1.0e-3, 1.0e-3, 1.0e-3,
     1.0e-5, 1.0e-5, 1.0e-5], dtype=np.float64
)
TRUST_REGION_SCALES = (1.0, 0.5, 0.25, 0.125, 0.0625)


def load_snapshot(path: Path) -> np.ndarray:
    payload = json.loads(path.read_text(encoding="utf-8"))
    covariance = np.asarray(payload.get("P"), dtype=np.float64)
    if covariance.shape != (15, 15) or not np.all(np.isfinite(covariance)):
        raise ValueError("snapshot P must be a finite 15x15 matrix")
    covariance = 0.5 * (covariance + covariance.T)
    eigenvalues = np.linalg.eigvalsh(covariance)
    if float(np.min(eigenvalues)) <= 0.0:
        raise ValueError("snapshot P must be positive definite")
    return covariance


def _prior_information(covariance: np.ndarray, scale: float) -> np.ndarray:
    if covariance.shape != (15, 15):
        raise ValueError("prior covariance must be 15x15")
    if scale <= 0.0 or not math.isfinite(scale):
        raise ValueError("prior_information_scale must be finite and positive")
    scales = np.diag(PARAMETER_SCALES)
    normalized = np.linalg.solve(scales, covariance) @ np.linalg.inv(scales)
    normalized = 0.5 * (normalized + normalized.T)
    information = np.linalg.inv(normalized) * scale
    return 0.5 * (information + information.T)


def _solve_normalized(
    design_physical: np.ndarray,
    residual: np.ndarray,
    variance: np.ndarray,
    prior_information: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    if design_physical.ndim != 2 or design_physical.shape[1] != 15:
        raise ValueError("design must have 15 columns")
    if residual.shape != (design_physical.shape[0],) or variance.shape != residual.shape:
        raise ValueError("design, residual, and variance lengths differ")
    if np.any(~np.isfinite(design_physical)) or np.any(~np.isfinite(residual)):
        raise ValueError("design and residual must be finite")
    if np.any(~np.isfinite(variance)) or np.any(variance <= 0.0):
        raise ValueError("innovation variance must be finite and positive")
    scales = np.diag(PARAMETER_SCALES)
    normalized_design = design_physical @ scales
    whiten = 1.0 / np.sqrt(variance)
    weighted_design = normalized_design * whiten[:, np.newaxis]
    weighted_residual = residual * whiten
    information = weighted_design.T @ weighted_design + prior_information
    information = 0.5 * (information + information.T)
    eigenvalues = np.linalg.eigvalsh(information)
    if float(np.min(eigenvalues)) <= 0.0 or not np.all(np.isfinite(eigenvalues)):
        raise ValueError("full-state information is not positive definite")
    normalized_correction = -np.linalg.solve(
        information, weighted_design.T @ weighted_residual
    )
    correction = PARAMETER_SCALES * normalized_correction
    post_residual = weighted_residual + weighted_design @ normalized_correction
    return correction, eigenvalues, post_residual


def solve_window(
    runner: Path,
    input_path: Path,
    baseline_path: Path,
    snapshot_path: Path,
    window_start_s: float,
    window_stop_s: float,
    *,
    prior_information_scale: float = 1.0,
    minimum_measurement_rows: int = 10,
    work_dir: Path | None = None,
) -> dict[str, object]:
    """Solve and replay a full 15D local correction without truth."""

    baseline = base.load_trace(input_path, baseline_path)
    start_index = int(np.searchsorted(
        baseline.timestamps_us, window_start_s * 1.0e6, side="left"
    ))
    stop_index = int(np.searchsorted(
        baseline.timestamps_us, window_stop_s * 1.0e6, side="right"
    )) - 1
    if start_index <= 0 or stop_index <= start_index:
        raise ValueError("window is outside the trace")
    covariance = load_snapshot(snapshot_path)
    prior_information = _prior_information(covariance, prior_information_scale)
    fit_stop_index = start_index + max(1, (stop_index - start_index) // 2)
    validation_start_index = fit_stop_index + 1
    baseline_schedule = base._event_schedule(baseline, start_index, stop_index)
    owns_work_dir = work_dir is None
    temporary = tempfile.TemporaryDirectory(prefix="aerakia-full-state-") if owns_work_dir else None
    root = Path(temporary.name) if temporary is not None else work_dir
    assert root is not None
    root.mkdir(parents=True, exist_ok=True)
    try:
        plus: list[base.ReplayTrace] = []
        minus: list[base.ReplayTrace] = []
        for parameter_index, epsilon in enumerate(PARAMETER_EPSILON):
            perturbation = np.zeros(15, dtype=np.float64)
            perturbation[parameter_index] = epsilon
            plus.append(base._run_error_state_replay(
                runner, input_path, root, window_start_s, perturbation
            ))
            perturbation[parameter_index] = -epsilon
            minus.append(base._run_error_state_replay(
                runner, input_path, root, window_start_s, perturbation
            ))
        divergent_parameters = [
            {
                "parameter": PARAMETER_LABELS[index % len(PARAMETER_LABELS)],
                "side": "plus" if index < len(PARAMETER_LABELS) else "minus",
            }
            for index, trace in enumerate((*plus, *minus))
            if base._event_schedule(trace, start_index, stop_index) != baseline_schedule
        ]
        if divergent_parameters:
            return {
                "status": "full_state_rejected_perturbation_schedule_changed",
                "truth_used_by_solver": False,
                "window_start_s": window_start_s,
                "window_stop_s": window_stop_s,
                "divergent_parameters": divergent_parameters,
            }
        design_rows: list[np.ndarray] = []
        residual_rows: list[float] = []
        variance_rows: list[float] = []
        events = 0
        for index in range(start_index, fit_stop_index + 1):
            position_accepted, velocity_accepted = baseline_schedule[index - start_index]
            for kind, accepted in (("position", position_accepted), ("velocity", velocity_accepted)):
                if not accepted:
                    continue
                residual = base._vector(baseline.result_rows[index], base.INNOVATION_COLUMNS[kind])
                variance = base._vector(
                    baseline.result_rows[index], base.INNOVATION_VARIANCE_COLUMNS[kind]
                )
                columns = []
                for parameter_index, epsilon in enumerate(PARAMETER_EPSILON):
                    plus_value = base._vector(plus[parameter_index].result_rows[index], base.INNOVATION_COLUMNS[kind])
                    minus_value = base._vector(minus[parameter_index].result_rows[index], base.INNOVATION_COLUMNS[kind])
                    columns.append((plus_value - minus_value) / (2.0 * epsilon))
                jacobian = np.column_stack(columns)
                design_rows.extend(jacobian)
                residual_rows.extend(residual.tolist())
                variance_rows.extend(variance.tolist())
                events += 1
        if len(residual_rows) < minimum_measurement_rows:
            return {
                "status": "full_state_rejected_insufficient_measurements",
                "truth_used_by_solver": False,
                "measurement_rows": len(residual_rows),
            }
        correction, eigenvalues, post_residual = _solve_normalized(
            np.vstack(design_rows), np.asarray(residual_rows),
            np.asarray(variance_rows), prior_information
        )
        before = base._weighted_innovation_norm(
            baseline, baseline_schedule, start_index, validation_start_index, stop_index
        )
        candidates: list[tuple[float, base.ReplayTrace, float]] = []
        rejected_scales: list[dict[str, object]] = []
        for scale in TRUST_REGION_SCALES:
            applied = correction * scale
            corrected = base._run_error_state_replay(
                runner, input_path, root, window_start_s, applied
            )
            schedule_matches = base._schedule_matches(
                baseline, corrected, start_index, stop_index
            )
            healthy = all(
                float(row["eskf_healthy"]) >= 0.5
                for row in corrected.result_rows[start_index:stop_index + 1]
            )
            after = base._weighted_innovation_norm(
                corrected, baseline_schedule, start_index, validation_start_index, stop_index
            ) if schedule_matches else math.inf
            if schedule_matches and healthy and math.isfinite(after) and after <= before:
                candidates.append((scale, corrected, after))
            else:
                rejected_scales.append({
                    "scale": scale,
                    "schedule_matches": schedule_matches,
                    "healthy": healthy,
                    "validation_weighted_innovation_norm": after,
                })
        if candidates:
            applied_scale, corrected, after = min(candidates, key=lambda item: item[2])
            schedule_matches = True
            healthy = True
            validation_improved = True
            status = "full_state_computed_replayed"
        else:
            applied_scale = None
            after = math.inf
            schedule_matches = False
            healthy = False
            validation_improved = False
            status = "full_state_rejected_trust_region"
        applied_correction = correction * applied_scale if applied_scale is not None else None
        return {
            "status": status,
            "method": "full_15d_preupdate_innovation_finite_difference_replay",
            "truth_used_by_solver": False,
            "window_start_s": window_start_s,
            "window_stop_s": window_stop_s,
            "window_start_index": start_index,
            "fit_stop_index": fit_stop_index,
            "validation_start_index": validation_start_index,
            "window_stop_index": stop_index,
            "event_count": events,
            "measurement_rows": len(residual_rows),
            "parameter_order": list(PARAMETER_LABELS),
            "parameter_scales": PARAMETER_SCALES.tolist(),
            "parameter_epsilon": PARAMETER_EPSILON.tolist(),
            "information_eigenvalues": eigenvalues.tolist(),
            "information_condition_number": float(eigenvalues[-1] / eigenvalues[0]),
            "unconstrained_correction_physical": correction.tolist(),
            "correction_physical": applied_correction.tolist() if applied_correction is not None else None,
            "trust_region_scales": list(TRUST_REGION_SCALES),
            "selected_trust_region_scale": applied_scale,
            "rejected_trust_region_scales": rejected_scales,
            "correction_norm": float(np.linalg.norm(correction)),
            "weighted_residual_norm_before": float(np.linalg.norm(np.asarray(residual_rows) / np.sqrt(np.asarray(variance_rows)))),
            "weighted_residual_norm_after_linear": float(np.linalg.norm(post_residual)),
            "schedule_matches_after_replay": schedule_matches,
            "corrected_replay_healthy": healthy,
            "validation_weighted_innovation_norm_before": before,
            "validation_weighted_innovation_norm_after": after,
            "validation_innovation_improved": validation_improved,
            "limitations": [
                "The full 15D prior is from one boundary snapshot; lag process/preintegration covariance is not reconstructed.",
                "Innovation covariance is diagonal and cross-source aiding correlation is omitted.",
                "This is a host diagnostic and not a public estimator or flight-control feature.",
            ],
        }
    finally:
        if temporary is not None:
            temporary.cleanup()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runner", type=Path, required=True)
    parser.add_argument("--replay", type=Path, required=True)
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument("--snapshot", type=Path, required=True)
    parser.add_argument("--window-start-s", type=float, required=True)
    parser.add_argument("--window-stop-s", type=float, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    result = solve_window(
        args.runner.resolve(), args.replay.resolve(), args.baseline.resolve(),
        args.snapshot.resolve(), args.window_start_s, args.window_stop_s,
    )
    result["git_commit"] = subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True
    ).strip()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["status"] == "full_state_computed_replayed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
