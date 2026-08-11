#!/usr/bin/env python3
"""Run the frozen host-only fixed-lag replay candidate development matrix."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import shutil
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Iterable

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "validation"))

import fixed_lag_sensitivity_proposal as candidate  # noqa: E402


GENERATOR = ROOT / "simulation/tools/generate_synthetic_imu.py"
DEFAULT_PROTOCOL = ROOT / "validation/fixed_lag_replay_candidate_protocol_v1.json"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"empty CSV: {path}")
    return rows


def run_logged(command: list[str], log_path: Path) -> None:
    completed = subprocess.run(
        command,
        cwd=ROOT,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
    )
    log_path.write_text(completed.stdout, encoding="utf-8")
    if completed.returncode != 0:
        raise RuntimeError(
            f"command failed ({completed.returncode}): {' '.join(command)}\n{completed.stdout[-2000:]}"
        )


def _vectors(protocol: dict[str, object]) -> list[dict[str, object]]:
    contract = protocol["residual_bias_contract"]
    assert isinstance(contract, dict)
    sigma = float(contract["accelerometer_sigma_m_s2"])
    vectors = contract["vectors"]
    assert isinstance(vectors, list)
    return [
        {
            "id": str(item["id"]),
            "accel_bias_m_s2": [
                sigma * float(value) for value in item["accel_sigma_multipliers"]
            ],
        }
        for item in vectors
    ]


def _float_vector(row: dict[str, str], names: Iterable[str]) -> np.ndarray:
    return np.asarray([float(row[name]) for name in names], dtype=np.float64)


def _quaternion_error_deg(row: dict[str, str]) -> float:
    truth = _float_vector(row, ("truth_q_w", "truth_q_x", "truth_q_y", "truth_q_z"))
    estimate = _float_vector(row, ("eskf_q_w", "eskf_q_x", "eskf_q_y", "eskf_q_z"))
    norm = float(np.linalg.norm(truth) * np.linalg.norm(estimate))
    if norm <= 0.0:
        raise ValueError("zero quaternion in scored trace")
    dot = min(1.0, abs(float(truth @ estimate)) / norm)
    return float(math.degrees(2.0 * math.acos(dot)))


def trace_metrics(
    rows: list[dict[str, str]], window_start_s: float, window_stop_s: float, terminal_window_s: float
) -> dict[str, float | int]:
    timestamps = np.asarray([float(row["ts_us"]) * 1.0e-6 for row in rows])
    selected = np.flatnonzero(
        (timestamps >= window_start_s - 1.0e-12) & (timestamps <= window_stop_s + 1.0e-12)
    )
    terminal = selected[timestamps[selected] >= window_stop_s - terminal_window_s]
    if selected.size == 0 or terminal.size == 0:
        raise ValueError("scoring window has no rows")

    bias_error = np.asarray(
        [
            _float_vector(rows[index], candidate.BIAS_COLUMNS)
            - _float_vector(
                rows[index],
                (
                    "truth_accel_bias_x_m_s2", "truth_accel_bias_y_m_s2",
                    "truth_accel_bias_z_m_s2",
                ),
            )
            for index in selected
        ]
    )
    horizontal_bias = np.linalg.norm(bias_error[:, :2], axis=1)
    terminal_offset = selected.searchsorted(terminal[0])
    terminal_horizontal_bias = horizontal_bias[terminal_offset:]
    attitude = np.asarray([_quaternion_error_deg(rows[index]) for index in selected])
    reference_position = np.asarray(
        [
            _float_vector(
                rows[index], ("ref_position_n_m", "ref_position_e_m", "ref_position_d_m")
            )
            for index in selected
        ]
    )
    estimated_position = np.asarray(
        [_float_vector(rows[index], candidate.STATE_COLUMNS[:3]) for index in selected]
    )
    reference_velocity = np.asarray(
        [
            _float_vector(
                rows[index], ("ref_velocity_n_m_s", "ref_velocity_e_m_s", "ref_velocity_d_m_s")
            )
            for index in selected
        ]
    )
    estimated_velocity = np.asarray(
        [_float_vector(rows[index], candidate.STATE_COLUMNS[3:]) for index in selected]
    )
    position_norm = np.linalg.norm(estimated_position - reference_position, axis=1)
    velocity_norm = np.linalg.norm(estimated_velocity - reference_velocity, axis=1)
    position_nis = [
        float(rows[index]["eskf_position_nis"])
        for index in selected
        if int(float(rows[index]["input_position_update"])) != 0
        and math.isfinite(float(rows[index]["eskf_position_nis"]))
    ]
    velocity_nis = [
        float(rows[index]["eskf_velocity_nis"])
        for index in selected
        if int(float(rows[index]["input_velocity_update"])) != 0
        and math.isfinite(float(rows[index]["eskf_velocity_nis"]))
    ]
    return {
        "horizontal_bias_terminal_p95_m_s2": float(np.percentile(terminal_horizontal_bias, 95)),
        "horizontal_bias_terminal_mean_m_s2": float(np.mean(terminal_horizontal_bias)),
        "horizontal_bias_end_m_s2": float(horizontal_bias[-1]),
        "attitude_rmse_deg": float(np.sqrt(np.mean(attitude * attitude))),
        "attitude_p95_deg": float(np.percentile(attitude, 95)),
        "position_rmse_m": float(np.sqrt(np.mean(position_norm * position_norm))),
        "velocity_rmse_m_s": float(np.sqrt(np.mean(velocity_norm * velocity_norm))),
        "position_nis_mean": float(np.mean(position_nis)) if position_nis else math.nan,
        "velocity_nis_mean": float(np.mean(velocity_nis)) if velocity_nis else math.nan,
        "healthy_ratio": float(
            np.mean([float(rows[index]["eskf_healthy"]) > 0.5 for index in selected])
        ),
        "navigation_recoveries": max(
            int(float(rows[index]["eskf_navigation_recovery_count"])) for index in selected
        ),
    }


def _corrected_output_path(work_dir: Path, correction: np.ndarray) -> Path:
    label = "-".join(f"{value:.12g}" for value in correction)
    return work_dir / f"replay-{label}.csv"


def run_trial(
    runner: Path,
    work_root: Path,
    protocol: dict[str, object],
    trajectory: dict[str, object],
    vector: dict[str, object],
    seed: int,
    keep_work: bool,
) -> dict[str, object]:
    trial_id = f"{trajectory['id']}--{vector['id']}--{seed:05d}"
    trial_dir = work_root / trial_id
    if trial_dir.exists():
        shutil.rmtree(trial_dir)
    trial_dir.mkdir(parents=True)
    replay = trial_dir / "input.csv"
    metadata = trial_dir / "input-metadata.json"
    baseline = trial_dir / "baseline.csv"
    input_contract = protocol["input_contract"]
    solver = protocol["solver_boundary"]
    assert isinstance(input_contract, dict)
    assert isinstance(solver, dict)
    bias = vector["accel_bias_m_s2"]
    assert isinstance(bias, list)
    duration_s = float(trajectory["duration_s"])
    window_stop_s = duration_s - float(solver["window_stop_margin_s"])
    window_start_s = window_stop_s - float(solver["window_duration_s"])
    generate = [
        sys.executable, str(GENERATOR),
        "--out", str(replay), "--metadata", str(metadata),
        "--duration", str(duration_s), "--rate", str(input_contract["rate_hz"]),
        "--seed", str(seed), "--motion", str(trajectory["motion"]),
        "--static-hint", "--stationarity-source", str(input_contract["stationarity_source"]),
        "--measurement-contract", str(input_contract["measurement_contract"]),
        "--accel-time-semantics", str(input_contract["acceleration_time_semantics"]),
        "--accel-noise-density-m-s2-sqrt-hz",
        str(input_contract["accel_noise_density_m_s2_sqrt_hz"]),
        "--gyro-noise-density-rad-s-sqrt-hz",
        str(input_contract["gyro_noise_density_rad_s_sqrt_hz"]),
        "--mag-noise-ut", str(input_contract["mag_noise_ut"]),
        "--gps-position-noise-m", str(input_contract["gps_position_noise_m"]),
        "--gps-velocity-noise-m-s", str(input_contract["gps_velocity_noise_m_s"]),
        "--accel-bias-vector-m-s2", *(str(value) for value in bias),
        "--gyro-bias-vector-deg-s", "0", "0", "0", "--rate-invariant-streams",
    ]
    try:
        run_logged(generate, trial_dir / "generator.log")
        run_logged(
            [str(runner), "--cold-start", str(replay), str(baseline)],
            trial_dir / "baseline.log",
        )
        baseline_rows = read_rows(baseline)
        result = candidate.solve_window(
            runner, replay, baseline, window_start_s, window_stop_s,
            prior_information_scale=float(solver["prior_information_scale"]),
            work_dir=trial_dir / "candidate-replays",
        )
        record: dict[str, object] = {
            "trial_id": trial_id,
            "split": trajectory["split"],
            "trajectory_id": trajectory["id"],
            "motion": trajectory["motion"],
            "vector_id": vector["id"],
            "bias_m_s2": bias,
            "seed": seed,
            "input_sha256": sha256(replay),
            "baseline_sha256": sha256(baseline),
            "window_start_s": window_start_s,
            "window_stop_s": window_stop_s,
            "candidate_status": result["status"],
            "candidate": result,
            "baseline_metrics": trace_metrics(
                baseline_rows, window_start_s, window_stop_s,
                float(protocol["metrics"]["terminal_window_s"]),
            ),
        }
        if result["status"] == "proposal_computed_replayed":
            correction = np.asarray(result["correction_physical"], dtype=np.float64)
            corrected_path = _corrected_output_path(trial_dir / "candidate-replays", correction)
            corrected_rows = read_rows(corrected_path)
            corrected_metrics = trace_metrics(
                corrected_rows, window_start_s, window_stop_s,
                float(protocol["metrics"]["terminal_window_s"]),
            )
            record["corrected_sha256"] = sha256(corrected_path)
            record["candidate_metrics"] = corrected_metrics
            baseline_metrics = record["baseline_metrics"]
            assert isinstance(baseline_metrics, dict)
            record["paired_delta"] = {
                key: float(corrected_metrics[key]) - float(baseline_metrics[key])
                for key in (
                    "horizontal_bias_terminal_p95_m_s2", "horizontal_bias_terminal_mean_m_s2",
                    "horizontal_bias_end_m_s2", "attitude_rmse_deg", "attitude_p95_deg",
                    "position_rmse_m", "velocity_rmse_m_s", "position_nis_mean",
                    "velocity_nis_mean",
                )
            }
        return record
    finally:
        if not keep_work:
            shutil.rmtree(trial_dir, ignore_errors=True)


def _mean(values: list[float]) -> float | None:
    return float(np.mean(values)) if values else None


def _p95(values: list[float]) -> float | None:
    return float(np.percentile(values, 95)) if values else None


def summarize(records: list[dict[str, object]], protocol: dict[str, object]) -> dict[str, object]:
    completed = [record for record in records if "candidate_metrics" in record]
    metrics = protocol["metrics"]
    assert isinstance(metrics, dict)
    material_bias = float(metrics["material_horizontal_bias_regression_m_s2"])
    material_attitude = float(metrics["material_attitude_rmse_regression_deg"])
    minimum_improvement = float(metrics["minimum_mean_horizontal_bias_improvement_m_s2"])
    minimum_group_fraction = float(metrics["minimum_signed_nonzero_vector_improvement_fraction"])

    def metric(scope: list[dict[str, object]], lane: str, key: str) -> list[float]:
        return [float(record[lane][key]) for record in scope]  # type: ignore[index]

    scopes: dict[str, list[dict[str, object]]] = {
        "all": completed,
        "train": [record for record in completed if record["split"] == "train"],
        "tune": [record for record in completed if record["split"] == "tune"],
    }
    aggregate: dict[str, object] = {}
    for name, scope in scopes.items():
        baseline_values = metric(scope, "baseline_metrics", "horizontal_bias_terminal_p95_m_s2")
        candidate_values = metric(scope, "candidate_metrics", "horizontal_bias_terminal_p95_m_s2")
        deltas = [candidate_value - baseline_value for baseline_value, candidate_value in zip(baseline_values, candidate_values)]
        aggregate[name] = {
            "trials": len(scope),
            "baseline_terminal_bias_p95_mean_m_s2": _mean(baseline_values),
            "candidate_terminal_bias_p95_mean_m_s2": _mean(candidate_values),
            "paired_delta_mean_m_s2": _mean(deltas),
            "paired_delta_p95_m_s2": _p95(deltas),
            "improved_trials": sum(delta < 0.0 for delta in deltas),
        }

    zero = [record for record in completed if record["vector_id"] == "zero"]
    zero_regressions = [
        record["trial_id"] for record in zero
        if float(record["paired_delta"]["horizontal_bias_terminal_p95_m_s2"]) > material_bias  # type: ignore[index]
    ]
    attitude_regressions = [
        record["trial_id"] for record in completed
        if float(record["paired_delta"]["attitude_rmse_deg"]) > material_attitude  # type: ignore[index]
    ]
    groups: dict[tuple[str, str], list[float]] = {}
    for record in completed:
        if record["vector_id"] == "zero":
            continue
        key = (str(record["trajectory_id"]), str(record["vector_id"]))
        groups.setdefault(key, []).append(
            float(record["paired_delta"]["horizontal_bias_terminal_p95_m_s2"])  # type: ignore[index]
        )
    improved_groups = sum(_mean(values) is not None and float(_mean(values)) < 0.0 for values in groups.values())
    group_fraction = improved_groups / len(groups) if groups else 0.0
    train_delta = aggregate["train"]["paired_delta_mean_m_s2"]  # type: ignore[index]
    tune_delta = aggregate["tune"]["paired_delta_mean_m_s2"]  # type: ignore[index]
    integrity_pass = (
        len(completed) == len(records)
        and all(float(record["candidate_metrics"]["healthy_ratio"]) == 1.0 for record in completed)  # type: ignore[index]
        and all(int(record["candidate_metrics"]["navigation_recoveries"]) == 0 for record in completed)  # type: ignore[index]
        and all(bool(record["candidate"]["schedule_matches_after_replay"]) for record in completed)  # type: ignore[index]
    )
    development_pass = (
        integrity_pass
        and train_delta is not None and float(train_delta) <= -minimum_improvement
        and tune_delta is not None and float(tune_delta) <= -minimum_improvement
        and group_fraction >= minimum_group_fraction
        and not zero_regressions
        and not attitude_regressions
    )
    return {
        "trial_count": len(records),
        "completed_candidate_count": len(completed),
        "rejected_candidate_count": len(records) - len(completed),
        "integrity_pass": integrity_pass,
        "development_acceptance_pass": development_pass,
        "aggregate": aggregate,
        "signed_nonzero_group_count": len(groups),
        "improved_signed_nonzero_groups": improved_groups,
        "improved_signed_nonzero_group_fraction": group_fraction,
        "zero_bias_material_regressions": zero_regressions,
        "attitude_material_regressions": attitude_regressions,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runner", type=Path, required=True)
    parser.add_argument("--protocol", type=Path, default=DEFAULT_PROTOCOL)
    parser.add_argument("--out", type=Path, default=Path("validation/public/fixed_lag_replay_candidate_v1.json"))
    parser.add_argument("--work-dir", type=Path, default=Path("build/fixed-lag-replay-candidate-v1"))
    parser.add_argument("--jobs", type=int, default=1)
    parser.add_argument("--keep-work", action="store_true")
    parser.add_argument("--limit-seeds", type=int, default=None, help="development smoke only")
    args = parser.parse_args()
    if args.jobs <= 0 or (args.limit_seeds is not None and args.limit_seeds <= 0):
        parser.error("jobs and limit-seeds must be positive")
    runner = args.runner.resolve()
    protocol_path = args.protocol if args.protocol.is_absolute() else ROOT / args.protocol
    output = args.out if args.out.is_absolute() else ROOT / args.out
    work_dir = args.work_dir if args.work_dir.is_absolute() else ROOT / args.work_dir
    protocol = json.loads(protocol_path.read_text(encoding="utf-8"))
    if protocol.get("status") != "frozen_development_not_promotion":
        parser.error("protocol status must be frozen_development_not_promotion")
    if not runner.is_file():
        parser.error(f"runner does not exist: {runner}")
    vectors = _vectors(protocol)
    trajectories = protocol["trajectories"]
    assert isinstance(trajectories, list)
    tasks = []
    for trajectory in trajectories:
        seed_start = int(trajectory["seed_start"])
        seed_stop = int(trajectory["seed_stop"])
        if args.limit_seeds is not None:
            seed_stop = min(seed_stop, seed_start + args.limit_seeds)
        tasks.extend(
            (trajectory, vector, seed)
            for vector in vectors
            for seed in range(seed_start, seed_stop)
        )
    work_dir.mkdir(parents=True, exist_ok=True)
    with ThreadPoolExecutor(max_workers=args.jobs) as executor:
        futures = [
            executor.submit(
                run_trial, runner, work_dir, protocol, trajectory, vector, seed, args.keep_work
            )
            for trajectory, vector, seed in tasks
        ]
        records = [future.result() for future in futures]
    records.sort(key=lambda item: str(item["trial_id"]))
    summary = summarize(records, protocol)
    result = {
        "schema_version": 1,
        "status": (
            "development_acceptance_passed_not_promoted"
            if summary["development_acceptance_pass"]
            else "development_candidate_rejected_or_incomplete"
        ),
        "promotion_decision": "not_promoted_development_only",
        "protocol_id": protocol["protocol_id"],
        "protocol_sha256": sha256(protocol_path),
        "candidate_source_sha256": sha256(ROOT / "validation/fixed_lag_sensitivity_proposal.py"),
        "campaign_source_sha256": sha256(Path(__file__)),
        "runner_sha256": sha256(runner),
        "generator_sha256": sha256(GENERATOR),
        "git_commit": subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True
        ).strip(),
        "jobs": args.jobs,
        "limited_seed_smoke": args.limit_seeds is not None,
        "summary": summary,
        "trials": records,
        "limitations": protocol["limits"],
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({key: value for key, value in result.items() if key != "trials"}, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
