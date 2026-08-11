#!/usr/bin/env python3
"""Host-only fixed-lag joint tilt/bias replay candidate.

This experiment is the first candidate that performs the complete transaction
needed by a lagged ESKF correction:

* run symmetric, causal perturbation replays from the same cold-start input;
* use the *pre-update* GNSS position/velocity innovations recorded by the
  runner, not the already corrected state;
* solve a prior-regularized five-dimensional correction for right tilt x/y and
  accelerometer bias x/y/z;
* inject that correction with the core's nominal-state plus covariance-reset
  transaction and replay the window again.

The candidate remains host-only and shadow-only.  It never reads truth, never
changes the production ESKF path, and never grants controller authority.  The
campaign evaluator may use truth after this module returns to score a proposal.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import subprocess
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
PARAMETER_LABELS = (
    "window_start_right_tilt_x_rad",
    "window_start_right_tilt_y_rad",
    "window_start_accel_bias_x_m_s2",
    "window_start_accel_bias_y_m_s2",
    "window_start_accel_bias_z_m_s2",
)
PARAMETER_SCALES = np.asarray([0.05, 0.05, 0.15, 0.15, 0.15], dtype=np.float64)
PARAMETER_EPSILON = np.asarray([1.0e-4, 1.0e-4, 1.0e-3, 1.0e-3, 1.0e-3], dtype=np.float64)
TRUST_REGION_SCALES = (1.0, 0.5, 0.25, 0.125, 0.0625)
ERROR_STATE_DIMENSION = 15
ERROR_STATE_DTHETA = 0
ERROR_STATE_DAB = 9

STATE_COLUMNS = (
    "eskf_position_n_m", "eskf_position_e_m", "eskf_position_d_m",
    "eskf_velocity_n_m_s", "eskf_velocity_e_m_s", "eskf_velocity_d_m_s",
)
BIAS_COLUMNS = (
    "eskf_accel_bias_x_m_s2", "eskf_accel_bias_y_m_s2", "eskf_accel_bias_z_m_s2",
)
INNOVATION_COLUMNS = {
    "position": (
        "eskf_position_innovation_n_m", "eskf_position_innovation_e_m",
        "eskf_position_innovation_d_m",
    ),
    "velocity": (
        "eskf_velocity_innovation_n_m_s", "eskf_velocity_innovation_e_m_s",
        "eskf_velocity_innovation_d_m_s",
    ),
}
INNOVATION_VARIANCE_COLUMNS = {
    "position": (
        "eskf_position_innovation_var_n_m2", "eskf_position_innovation_var_e_m2",
        "eskf_position_innovation_var_d_m2",
    ),
    "velocity": (
        "eskf_velocity_innovation_var_n_m2_s2", "eskf_velocity_innovation_var_e_m2_s2",
        "eskf_velocity_innovation_var_d_m2_s2",
    ),
}


@dataclass(frozen=True)
class ReplayTrace:
    input_rows: tuple[dict[str, str], ...]
    result_rows: tuple[dict[str, str], ...]
    timestamps_us: np.ndarray


def _read_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None:
            raise ValueError(f"missing CSV header: {path}")
        return list(reader)


def _number(row: dict[str, str], name: str) -> float:
    try:
        value = float(row[name])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"invalid numeric field {name}") from error
    if not math.isfinite(value):
        raise ValueError(f"non-finite numeric field {name}")
    return value


def _integer(row: dict[str, str], name: str) -> int:
    value = _number(row, name)
    if value != int(value):
        raise ValueError(f"field {name} is not integral")
    return int(value)


def load_trace(input_path: Path, result_path: Path) -> ReplayTrace:
    """Load a timestamp-paired replay and fail closed on identity errors."""

    input_rows = _read_rows(input_path)
    result_rows = _read_rows(result_path)
    if not input_rows or len(input_rows) != len(result_rows):
        raise ValueError("input and result traces must have equal nonzero row counts")
    timestamps = np.asarray([_number(row, "ts_us") for row in input_rows], dtype=np.float64)
    result_timestamps = np.asarray(
        [_number(row, "ts_us") for row in result_rows], dtype=np.float64
    )
    if not np.array_equal(timestamps, result_timestamps):
        raise ValueError("input and result timestamps differ")
    if np.any(np.diff(timestamps) <= 0.0):
        raise ValueError("trace timestamps must be strictly increasing")
    required = {
        "input_gps_status", "input_position_update", "input_velocity_update",
        "eskf_position_accepted", "eskf_velocity_accepted", "eskf_healthy",
        *STATE_COLUMNS, *BIAS_COLUMNS,
        *INNOVATION_COLUMNS["position"], *INNOVATION_COLUMNS["velocity"],
        *INNOVATION_VARIANCE_COLUMNS["position"],
        *INNOVATION_VARIANCE_COLUMNS["velocity"],
    }
    missing = sorted(required.difference(result_rows[0]))
    if missing:
        raise ValueError(f"result trace is missing required columns: {', '.join(missing)}")
    return ReplayTrace(tuple(input_rows), tuple(result_rows), timestamps)


def _vector(row: dict[str, str], names: Iterable[str]) -> np.ndarray:
    return np.asarray([_number(row, name) for name in names], dtype=np.float64)


def _state_vector(row: dict[str, str]) -> np.ndarray:
    return _vector(row, STATE_COLUMNS)


def _bias_vector(row: dict[str, str]) -> np.ndarray:
    return _vector(row, BIAS_COLUMNS)


def _prior_covariance(row: dict[str, str]) -> np.ndarray:
    """Read the complete reviewed 5x5 marginal from a result row."""

    covariance = np.zeros((5, 5), dtype=np.float64)
    covariance[0, 0] = _number(row, "eskf_right_error_tilt_cov_xx_rad2")
    covariance[0, 1] = covariance[1, 0] = _number(
        row, "eskf_right_error_tilt_cov_xy_rad2"
    )
    covariance[1, 1] = _number(row, "eskf_right_error_tilt_cov_yy_rad2")
    bias_names = {
        (0, 0): "eskf_accel_bias_cov_xx_m2_s4",
        (0, 1): "eskf_accel_bias_cov_xy_m2_s4",
        (0, 2): "eskf_accel_bias_cov_xz_m2_s4",
        (1, 1): "eskf_accel_bias_cov_yy_m2_s4",
        (1, 2): "eskf_accel_bias_cov_yz_m2_s4",
        (2, 2): "eskf_accel_bias_cov_zz_m2_s4",
    }
    for (row_index, column_index), name in bias_names.items():
        value = _number(row, name)
        covariance[2 + row_index, 2 + column_index] = value
        covariance[2 + column_index, 2 + row_index] = value
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
    for row_index in range(2):
        for column_index in range(3):
            value = _number(row, cross_names[row_index][column_index])
            covariance[row_index, 2 + column_index] = value
            covariance[2 + column_index, row_index] = value
    covariance = 0.5 * (covariance + covariance.T)
    eigenvalues = np.linalg.eigvalsh(covariance)
    if not np.all(np.isfinite(eigenvalues)) or float(np.min(eigenvalues)) <= 0.0:
        raise ValueError("window-start target covariance must be positive definite")
    return covariance


def _accepted_schedule(row: dict[str, str]) -> tuple[bool, bool]:
    """Return accepted P/V events, excluding latched status flags."""

    if _integer(row, "input_gps_status") != 0:
        return False, False
    return (
        _integer(row, "input_position_update") != 0
        and _integer(row, "eskf_position_accepted") != 0,
        _integer(row, "input_velocity_update") != 0
        and _integer(row, "eskf_velocity_accepted") != 0,
    )


def _event_schedule(trace: ReplayTrace, start_index: int, stop_index: int) -> tuple[tuple[bool, bool], ...]:
    return tuple(
        _accepted_schedule(trace.result_rows[index])
        for index in range(start_index, stop_index + 1)
    )


def _error_state_from_correction(correction: np.ndarray) -> np.ndarray:
    if correction.shape != (5,) or not np.all(np.isfinite(correction)):
        raise ValueError("correction must be a finite five-vector")
    error_state = np.zeros(ERROR_STATE_DIMENSION, dtype=np.float64)
    error_state[ERROR_STATE_DTHETA] = correction[0]
    error_state[ERROR_STATE_DTHETA + 1] = correction[1]
    error_state[ERROR_STATE_DAB:ERROR_STATE_DAB + 3] = correction[2:5]
    return error_state


def _run_replay(
    runner: Path,
    input_path: Path,
    work_dir: Path,
    window_start_s: float,
    correction: np.ndarray,
) -> ReplayTrace:
    """Run one cold-start replay with a complete pre-IMU error injection."""

    error_state = _error_state_from_correction(correction)
    return _run_error_state_replay(
        runner, input_path, work_dir, window_start_s, error_state, correction
    )


def _run_error_state_replay(
    runner: Path,
    input_path: Path,
    work_dir: Path,
    window_start_s: float,
    error_state: np.ndarray,
    label_values: np.ndarray | None = None,
) -> ReplayTrace:
    """Run a replay with an arbitrary finite 15D error-state injection."""

    if error_state.shape != (ERROR_STATE_DIMENSION,) or not np.all(np.isfinite(error_state)):
        raise ValueError("error_state must be a finite 15-vector")
    if label_values is None:
        label_values = error_state
    if label_values.ndim != 1 or not np.all(np.isfinite(label_values)):
        raise ValueError("label_values must be a finite vector")
    label = "-".join(f"{value:.12g}" for value in label_values)
    if len(label) > 120:
        label = "state-" + hashlib.sha256(
            np.asarray(label_values, dtype=np.float64).tobytes()
        ).hexdigest()[:20]
    output_path = work_dir / f"replay-{label}.csv"
    command = [
        str(runner.resolve()),
        "--cold-start",
        "--inject-at-s", f"{window_start_s:.12g}",
        "--inject-error-state", *(f"{value:.12g}" for value in error_state),
        str(input_path.resolve()), str(output_path),
    ]
    completed = subprocess.run(
        command,
        cwd=ROOT,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
    )
    if completed.returncode != 0:
        raise RuntimeError(
            f"replay failed ({completed.returncode}): {completed.stdout[-2000:]}"
        )
    return load_trace(input_path, output_path)


def _prior_information(prior_covariance: np.ndarray, scale: float) -> np.ndarray:
    if prior_covariance.shape != (5, 5):
        raise ValueError("prior covariance must be 5x5")
    if scale <= 0.0 or not math.isfinite(scale):
        raise ValueError("prior_information_scale must be finite and positive")
    scales = np.diag(PARAMETER_SCALES)
    normalized = np.linalg.solve(scales, prior_covariance) @ np.linalg.inv(scales)
    normalized = 0.5 * (normalized + normalized.T)
    information = np.linalg.inv(normalized) * scale
    return 0.5 * (information + information.T)


def _solve_normalized(
    design_physical: np.ndarray,
    residual: np.ndarray,
    variance: np.ndarray,
    prior_information: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Solve min ||(r + J d)/sqrt(S)||^2 + d^T P^-1 d."""

    if design_physical.ndim != 2 or design_physical.shape[1] != 5:
        raise ValueError("design must have five columns")
    if residual.ndim != 1 or variance.ndim != 1:
        raise ValueError("residual and variance must be vectors")
    if design_physical.shape[0] != residual.size or residual.size != variance.size:
        raise ValueError("design, residual, and variance row counts differ")
    if np.any(~np.isfinite(design_physical)) or np.any(~np.isfinite(residual)):
        raise ValueError("design and residual must be finite")
    if np.any(~np.isfinite(variance)) or np.any(variance <= 0.0):
        raise ValueError("innovation variance must be finite and positive")
    scales = np.diag(PARAMETER_SCALES)
    design_normalized = design_physical @ scales
    whiten = 1.0 / np.sqrt(variance)
    weighted_design = design_normalized * whiten[:, np.newaxis]
    weighted_residual = residual * whiten
    information = weighted_design.T @ weighted_design + prior_information
    information = 0.5 * (information + information.T)
    eigenvalues = np.linalg.eigvalsh(information)
    if not np.all(np.isfinite(eigenvalues)) or float(np.min(eigenvalues)) <= 0.0:
        raise ValueError("proposal information is not positive definite")
    normalized_correction = -np.linalg.solve(
        information, weighted_design.T @ weighted_residual
    )
    physical_correction = PARAMETER_SCALES * normalized_correction
    return physical_correction, eigenvalues, weighted_residual + weighted_design @ normalized_correction


def _schedule_matches(
    baseline: ReplayTrace, candidate: ReplayTrace, start_index: int, stop_index: int
) -> bool:
    return _event_schedule(baseline, start_index, stop_index) == _event_schedule(
        candidate, start_index, stop_index
    )


def _weighted_innovation_norm(
    trace: ReplayTrace,
    schedule: tuple[tuple[bool, bool], ...],
    schedule_start_index: int,
    start_index: int,
    stop_index: int,
) -> float:
    values: list[float] = []
    for index in range(start_index, stop_index + 1):
        position_accepted, velocity_accepted = schedule[index - schedule_start_index]
        for kind, accepted in (("position", position_accepted), ("velocity", velocity_accepted)):
            if not accepted:
                continue
            innovation = _vector(trace.result_rows[index], INNOVATION_COLUMNS[kind])
            variance = _vector(trace.result_rows[index], INNOVATION_VARIANCE_COLUMNS[kind])
            if np.any(variance <= 0.0) or not np.all(np.isfinite(innovation)):
                return math.inf
            values.extend((innovation / np.sqrt(variance)).tolist())
    return float(np.linalg.norm(np.asarray(values, dtype=np.float64))) if values else math.inf


def solve_window(
    runner: Path,
    input_path: Path,
    baseline_path: Path,
    window_start_s: float,
    window_stop_s: float,
    *,
    prior_information_scale: float = 1.0,
    minimum_measurement_rows: int = 10,
    work_dir: Path | None = None,
) -> dict[str, object]:
    """Compute and replay one causal five-dimensional lag correction."""

    if not math.isfinite(window_start_s) or not math.isfinite(window_stop_s):
        raise ValueError("window bounds must be finite")
    if window_start_s < 0.0 or window_stop_s <= window_start_s:
        raise ValueError("window bounds are invalid")
    baseline = load_trace(input_path, baseline_path)
    start_index = int(np.searchsorted(
        baseline.timestamps_us, window_start_s * 1.0e6, side="left"
    ))
    stop_index = int(np.searchsorted(
        baseline.timestamps_us, window_stop_s * 1.0e6, side="right"
    )) - 1
    if start_index <= 0 or start_index >= len(baseline.result_rows) or stop_index <= start_index:
        raise ValueError("window is outside the trace or has no prior row")
    for row in baseline.result_rows[start_index:stop_index + 1]:
        if _number(row, "eskf_healthy") < 0.5:
            raise ValueError("baseline became unhealthy inside proposal window")

    prior_covariance = _prior_covariance(baseline.result_rows[start_index - 1])
    prior_information = _prior_information(prior_covariance, prior_information_scale)
    fit_stop_index = start_index + max(1, (stop_index - start_index) // 2)
    validation_start_index = fit_stop_index + 1
    owns_work_dir = work_dir is None
    temporary = tempfile.TemporaryDirectory(prefix="aerakia-fixed-lag-") if owns_work_dir else None
    root = Path(temporary.name) if temporary is not None else work_dir
    assert root is not None
    root.mkdir(parents=True, exist_ok=True)
    try:
        plus: list[ReplayTrace] = []
        minus: list[ReplayTrace] = []
        for parameter_index, epsilon in enumerate(PARAMETER_EPSILON):
            perturbation = np.zeros(5, dtype=np.float64)
            perturbation[parameter_index] = epsilon
            plus.append(_run_replay(runner, input_path, root, window_start_s, perturbation))
            perturbation[parameter_index] = -epsilon
            minus.append(_run_replay(runner, input_path, root, window_start_s, perturbation))

        baseline_schedule = _event_schedule(baseline, start_index, stop_index)
        if any(
            _event_schedule(trace, start_index, stop_index) != baseline_schedule
            for trace in (*plus, *minus)
        ):
            return {
                "status": "proposal_rejected_perturbation_schedule_changed",
                "truth_used_by_solver": False,
                "window_start_s": window_start_s,
                "window_stop_s": window_stop_s,
            }

        design_rows: list[np.ndarray] = []
        residual_rows: list[float] = []
        variance_rows: list[float] = []
        event_count = 0
        for index in range(start_index, fit_stop_index + 1):
            position_accepted, velocity_accepted = baseline_schedule[index - start_index]
            for kind, accepted in (("position", position_accepted), ("velocity", velocity_accepted)):
                if not accepted:
                    continue
                residual = _vector(baseline.result_rows[index], INNOVATION_COLUMNS[kind])
                variance = _vector(
                    baseline.result_rows[index], INNOVATION_VARIANCE_COLUMNS[kind]
                )
                columns: list[np.ndarray] = []
                for parameter_index, epsilon in enumerate(PARAMETER_EPSILON):
                    plus_residual = _vector(plus[parameter_index].result_rows[index], INNOVATION_COLUMNS[kind])
                    minus_residual = _vector(minus[parameter_index].result_rows[index], INNOVATION_COLUMNS[kind])
                    columns.append((plus_residual - minus_residual) / (2.0 * epsilon))
                jacobian = np.column_stack(columns)
                design_rows.extend(jacobian)
                residual_rows.extend(residual.tolist())
                variance_rows.extend(variance.tolist())
                event_count += 1

        if len(residual_rows) < minimum_measurement_rows:
            return {
                "status": "proposal_rejected_insufficient_measurements",
                "truth_used_by_solver": False,
                "measurement_rows": len(residual_rows),
                "event_count": event_count,
                "window_start_s": window_start_s,
                "window_stop_s": window_stop_s,
            }
        design = np.vstack(design_rows)
        residual = np.asarray(residual_rows, dtype=np.float64)
        variance = np.asarray(variance_rows, dtype=np.float64)
        correction, eigenvalues, weighted_post_residual = _solve_normalized(
            design, residual, variance, prior_information
        )
        validation_baseline_norm = _weighted_innovation_norm(
            baseline, baseline_schedule, start_index, validation_start_index, stop_index
        )
        candidates: list[tuple[float, ReplayTrace, float]] = []
        rejected_scales: list[dict[str, object]] = []
        for scale in TRUST_REGION_SCALES:
            applied = correction * scale
            corrected = _run_replay(runner, input_path, root, window_start_s, applied)
            schedule_matches = _schedule_matches(baseline, corrected, start_index, stop_index)
            corrected_healthy = all(
                _number(row, "eskf_healthy") >= 0.5
                for row in corrected.result_rows[start_index:stop_index + 1]
            )
            corrected_norm = _weighted_innovation_norm(
                corrected, baseline_schedule, start_index, validation_start_index, stop_index
            ) if schedule_matches else math.inf
            if schedule_matches and corrected_healthy and math.isfinite(corrected_norm) \
                    and corrected_norm <= validation_baseline_norm:
                candidates.append((scale, corrected, corrected_norm))
            else:
                rejected_scales.append({
                    "scale": scale,
                    "schedule_matches": schedule_matches,
                    "healthy": corrected_healthy,
                    "validation_weighted_innovation_norm": corrected_norm,
                })
        if candidates:
            applied_scale, corrected, validation_corrected_norm = min(
                candidates, key=lambda item: item[2]
            )
            schedule_matches = True
            corrected_healthy = True
            validation_improved = True
            status = "proposal_computed_replayed"
        else:
            applied_scale = None
            validation_corrected_norm = math.inf
            schedule_matches = False
            corrected_healthy = False
            validation_improved = False
            status = "proposal_rejected_trust_region"
        applied_correction = correction * applied_scale if applied_scale is not None else None
        return {
            "status": status,
            "method": "causal_preupdate_innovation_finite_difference_replay",
            "truth_used_by_solver": False,
            "window_start_s": window_start_s,
            "window_stop_s": window_stop_s,
            "window_start_index": start_index,
            "fit_stop_index": fit_stop_index,
            "validation_start_index": validation_start_index,
            "window_stop_index": stop_index,
            "measurement_rows": len(residual_rows),
            "event_count": event_count,
            "parameter_order": list(PARAMETER_LABELS),
            "parameter_scales": PARAMETER_SCALES.tolist(),
            "parameter_epsilon": PARAMETER_EPSILON.tolist(),
            "prior_information_scale": prior_information_scale,
            "information_eigenvalues": eigenvalues.tolist(),
            "information_condition_number": float(eigenvalues[-1] / eigenvalues[0]),
            "unconstrained_correction_physical": correction.tolist(),
            "correction_physical": applied_correction.tolist() if applied_correction is not None else None,
            "trust_region_scales": list(TRUST_REGION_SCALES),
            "selected_trust_region_scale": applied_scale,
            "rejected_trust_region_scales": rejected_scales,
            "correction_norm": float(np.linalg.norm(correction)),
            "weighted_residual_norm_before": float(np.linalg.norm(residual / np.sqrt(variance))),
            "weighted_residual_norm_after_linear": float(np.linalg.norm(weighted_post_residual)),
            "schedule_matches_after_replay": schedule_matches,
            "corrected_replay_healthy": corrected_healthy,
            "validation_weighted_innovation_norm_before": validation_baseline_norm,
            "validation_weighted_innovation_norm_after": validation_corrected_norm,
            "validation_innovation_improved": validation_improved,
            "limitations": [
                "The finite-difference model is local and host-only.",
                "The innovation weighting uses diagonal S entries and omits cross-axis covariance.",
                "The first gate is frozen by requiring identical accepted-event schedules.",
                "Process/preintegration covariance and delayed physical source timing remain outside this candidate.",
                "This result is shadow evidence, not a production estimator or controller gate.",
            ],
        }
    finally:
        if temporary is not None:
            temporary.cleanup()


def fingerprint(path: Path) -> dict[str, object]:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return {"sha256": digest.hexdigest(), "bytes": path.stat().st_size}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runner", type=Path, required=True)
    parser.add_argument("--replay", type=Path, required=True)
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument("--window-start-s", type=float, required=True)
    parser.add_argument("--window-stop-s", type=float, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--prior-information-scale", type=float, default=1.0)
    args = parser.parse_args()
    result = solve_window(
        args.runner.resolve(), args.replay.resolve(), args.baseline.resolve(),
        args.window_start_s, args.window_stop_s,
        prior_information_scale=args.prior_information_scale,
    )
    result.update(
        {
            "schema_version": 2,
            "inputs": {
                "replay": fingerprint(args.replay),
                "baseline": fingerprint(args.baseline),
                "runner": fingerprint(args.runner),
            },
            "git_commit": subprocess.check_output(
                ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True
            ).strip(),
        }
    )
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["status"] == "proposal_computed_replayed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
