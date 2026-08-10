#!/usr/bin/env python3
"""Evaluate the no-injection fixed-lag proposal on causal v2 replay cases.

The native ESKF remains the only estimator in every trial.  This campaign
generates a replay, runs the baseline, computes proposal-only corrections at
several closed windows, and only then uses the generator's truth fields to
score the proposal offline.  It is development evidence, not estimator or
flight qualification.
"""

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

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "validation"))

import fixed_lag_bias_proposal as proposal  # noqa: E402
import run_bias_observability_rate_sensitivity as rate_sensitivity  # noqa: E402


GENERATOR = ROOT / "simulation/tools/generate_synthetic_imu.py"
PROTOCOL = ROOT / "validation/bias_observability_protocol_v1.json"
MOTIONS = (
    ("takeoff_box_land", "bias_cv_takeoff_box_land", 38.0),
    ("yaw_quadrant_hover", "bias_cv_yaw_quadrant_hover", 40.0),
)
DEFAULT_PRIOR_SCALES = (0.1, 0.3, 1.0)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def run_logged(command: list[str], log_path: Path) -> None:
    completed = subprocess.run(
        command,
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )
    log_path.write_text(completed.stdout, encoding="utf-8")
    if completed.returncode != 0:
        raise RuntimeError(f"command failed ({completed.returncode}): {' '.join(command)}")


def vector_cases(protocol: dict[str, object]) -> list[dict[str, object]]:
    contract = protocol["residual_bias_contract"]
    assert isinstance(contract, dict)
    sigma = float(contract["accelerometer_sigma_m_s2"])
    vectors = contract["vectors"]
    assert isinstance(vectors, list)
    return [
        {
            "id": str(vector["id"]),
            "accel_bias_m_s2": [
                sigma * float(value) for value in vector["accel_sigma_multipliers"]
            ],
        }
        for vector in vectors
    ]


def bias_from_row(row: dict[str, str], prefix: str) -> np.ndarray:
    return np.asarray(
        [float(row[f"{prefix}_accel_bias_{axis}_m_s2"]) for axis in "xyz"],
        dtype=np.float64,
    )


def rows(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def trial(
    runner: Path,
    work_root: Path,
    motion_id: str,
    motion: str,
    duration_s: float,
    vector: dict[str, object],
    seed: int,
    prior_scales: tuple[float, ...],
    score_threshold: float,
    keep_work: bool,
) -> dict[str, object]:
    trial_dir = work_root / motion_id / str(vector["id"]) / f"seed-{seed:05d}"
    trial_dir.mkdir(parents=True, exist_ok=True)
    replay = trial_dir / "input.csv"
    metadata = trial_dir / "input-metadata.json"
    results = trial_dir / "results.csv"
    noise = json.loads(PROTOCOL.read_text(encoding="utf-8"))["noise"]
    assert isinstance(noise, dict)
    bias = vector["accel_bias_m_s2"]
    assert isinstance(bias, list)
    generate = [
        sys.executable,
        str(GENERATOR),
        "--out", str(replay),
        "--metadata", str(metadata),
        "--duration", str(duration_s),
        "--rate", str(noise["rate_hz"]),
        "--seed", str(seed),
        "--motion", motion,
        "--static-hint",
        "--stationarity-source", "causal_imu_window",
        "--measurement-contract", "delta_interval_v2",
        "--accel-time-semantics", "interval_start_zoh",
        "--accel-noise-density-m-s2-sqrt-hz", "0.002",
        "--gyro-noise-density-rad-s-sqrt-hz", "0.0000872664626",
        "--mag-noise-ut", str(noise["mag_noise_ut"]),
        "--gps-position-noise-m", str(noise["gps_position_noise_m"]),
        "--gps-velocity-noise-m-s", str(noise["gps_velocity_noise_m_s"]),
        "--accel-bias-vector-m-s2", *(str(value) for value in bias),
        "--gyro-bias-vector-deg-s", "0", "0", "0",
        "--rate-invariant-streams",
    ]
    run_logged(generate, trial_dir / "generator.log")
    run_logged([str(runner), "--cold-start", str(replay), str(results)], trial_dir / "filter.log")
    stream = proposal.load_residual_stream(replay, results)
    result_rows = rows(results)
    timestamps = stream.samples.timestamp_s
    stop_times = (20.0, 25.0, 30.0, duration_s - 0.01)
    stop_indices = sorted(
        set(
            int(np.searchsorted(timestamps, min(time_s, float(timestamps[-1])), side="left"))
            for time_s in stop_times
        )
    )
    trial_result: dict[str, object] = {
        "motion_id": motion_id,
        "motion": motion,
        "vector_id": vector["id"],
        "bias_m_s2": bias,
        "seed": seed,
        "input_sha256": sha256(replay),
        "result_sha256": sha256(results),
        "stop_times_s": [float(timestamps[index]) for index in stop_indices],
        "proposals": {str(scale): [] for scale in prior_scales},
        "estimator_healthy_ratio": float(
            sum(float(row["eskf_healthy"]) > 0.5 for row in result_rows) / len(result_rows)
        ),
        "navigation_recoveries": max(int(float(row["eskf_navigation_recovery_count"])) for row in result_rows),
    }
    for stop_index in stop_indices:
        truth = bias_from_row(result_rows[stop_index], "truth")
        baseline = bias_from_row(result_rows[stop_index], "eskf")
        for scale in prior_scales:
            evaluated = proposal.solve_window(
                stream,
                int(np.searchsorted(timestamps, timestamps[stop_index] - 20.0, side="left")),
                stop_index,
                prior_information_scale=scale,
                score_threshold=score_threshold,
            )
            correction = evaluated.get("correction_physical")
            item = {
                "stop_time_s": float(timestamps[stop_index]),
                "score_pass": bool(evaluated["score_pass"]),
                "minimum_eigenvalue": float(evaluated["minimum_eigenvalue"]),
                "baseline_error_norm_m_s2": float(np.linalg.norm(baseline - truth)),
                "proposal_correction_norm_m_s2": None,
                "corrected_error_norm_m_s2": None,
                "improved": False,
            }
            if correction is not None:
                delta_bias = np.asarray(correction[2:5], dtype=np.float64)
                corrected = baseline + delta_bias
                baseline_error = float(np.linalg.norm(baseline - truth))
                corrected_error = float(np.linalg.norm(corrected - truth))
                item.update(
                    {
                        "proposal_correction_norm_m_s2": float(np.linalg.norm(delta_bias)),
                        "corrected_error_norm_m_s2": corrected_error,
                        "improved": corrected_error < baseline_error,
                    }
                )
            trial_result["proposals"][str(scale)].append(item)
    if not keep_work:
        shutil.rmtree(trial_dir)
    return trial_result


def _observations(
    trials: list[dict[str, object]],
    scale: float,
    *,
    motion_id: str | None = None,
    terminal_only: bool = False,
) -> list[dict[str, object]]:
    selected: list[dict[str, object]] = []
    for trial_result in trials:
        if motion_id is not None and trial_result["motion_id"] != motion_id:
            continue
        items = list(trial_result["proposals"][str(scale)])
        if terminal_only and items:
            terminal_time = max(float(item["stop_time_s"]) for item in items)
            items = [
                item
                for item in items
                if math.isclose(float(item["stop_time_s"]), terminal_time, abs_tol=1.0e-9)
            ]
        selected.extend(
            item
            for item in items
            if item["score_pass"] and item["corrected_error_norm_m_s2"] is not None
        )
    return selected


def _summarize_observations(observations: list[dict[str, object]]) -> dict[str, object]:
    baseline = [float(item["baseline_error_norm_m_s2"]) for item in observations]
    corrected = [float(item["corrected_error_norm_m_s2"]) for item in observations]
    improved = sum(bool(item["improved"]) for item in observations)
    return {
        "proposal_observations": len(observations),
        "improved_observations": improved,
        "improvement_rate": improved / len(observations) if observations else None,
        "baseline_error_mean_m_s2": float(np.mean(baseline)) if baseline else None,
        "corrected_error_mean_m_s2": float(np.mean(corrected)) if corrected else None,
        "delta_error_mean_m_s2": float(np.mean(np.asarray(corrected) - np.asarray(baseline)))
        if observations
        else None,
        "baseline_error_p95_m_s2": float(np.percentile(baseline, 95)) if baseline else None,
        "corrected_error_p95_m_s2": float(np.percentile(corrected, 95)) if corrected else None,
        "delta_error_p95_m_s2": float(
            np.percentile(np.asarray(corrected) - np.asarray(baseline), 95)
        )
        if observations
        else None,
    }


def aggregate(
    trials: list[dict[str, object]],
    scales: tuple[float, ...],
    *,
    motion_id: str | None = None,
    terminal_only: bool = False,
) -> dict[str, object]:
    return {
        str(scale): _summarize_observations(
            _observations(
                trials,
                scale,
                motion_id=motion_id,
                terminal_only=terminal_only,
            )
        )
        for scale in scales
    }


def trial_manifest_sha256(trials: list[dict[str, object]], field: str) -> str:
    manifest = [
        {
            "motion_id": trial["motion_id"],
            "vector_id": trial["vector_id"],
            "seed": trial["seed"],
            field: trial[field],
        }
        for trial in trials
    ]
    encoded = json.dumps(manifest, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runner", type=Path, required=True)
    parser.add_argument("--out", type=Path, default=Path("validation/public/fixed_lag_bias_proposal_campaign.json"))
    parser.add_argument("--work-dir", type=Path, default=Path("build/fixed-lag-bias-proposal"))
    parser.add_argument("--seed-start", type=int, default=0)
    parser.add_argument("--seed-count", type=int, default=8)
    parser.add_argument("--prior-scales", default="0.1,0.3,1.0")
    parser.add_argument("--score-threshold", type=float, default=0.004936251852866821)
    parser.add_argument("--jobs", type=int, default=4)
    parser.add_argument("--keep-work", action="store_true")
    parser.add_argument(
        "--include-trials",
        action="store_true",
        help="include large per-window trial details in the output for local debugging",
    )
    args = parser.parse_args()
    prior_scales = tuple(float(value) for value in args.prior_scales.split(","))
    if not prior_scales or any(value <= 0.0 or not math.isfinite(value) for value in prior_scales):
        parser.error("--prior-scales must contain positive finite values")
    if args.seed_count <= 0 or args.jobs <= 0:
        parser.error("seed-count and jobs must be positive")
    runner = args.runner.resolve()
    if not runner.is_file():
        parser.error(f"runner does not exist: {runner}")
    protocol = json.loads(PROTOCOL.read_text(encoding="utf-8"))
    vectors = vector_cases(protocol)
    work_dir = args.work_dir if args.work_dir.is_absolute() else ROOT / args.work_dir
    tasks = [
        (motion_id, motion, duration_s, vector, seed)
        for motion_id, motion, duration_s in MOTIONS
        for vector in vectors
        for seed in range(args.seed_start, args.seed_start + args.seed_count)
    ]
    with ThreadPoolExecutor(max_workers=args.jobs) as executor:
        futures = [
            executor.submit(
                trial,
                runner,
                work_dir,
                motion_id,
                motion,
                duration_s,
                vector,
                seed,
                prior_scales,
                args.score_threshold,
                args.keep_work,
            )
            for motion_id, motion, duration_s, vector, seed in tasks
        ]
        trials = [future.result() for future in futures]
    trials.sort(key=lambda item: (str(item["motion_id"]), str(item["vector_id"]), int(item["seed"])))
    result = {
        "schema_version": 1,
        "status": "proposal_only_no_estimator_injection",
        "promotion_decision": "rejected_for_promotion",
        "study_id": "aerakia-fixed-lag-bias-proposal-campaign-v1",
        "protocol_path": str(PROTOCOL.relative_to(ROOT)),
        "protocol_sha256": sha256(PROTOCOL),
        "runner_sha256": sha256(runner),
        "generator_sha256": sha256(GENERATOR),
        "proposal_solver_sha256": sha256(Path(__file__).with_name("fixed_lag_bias_proposal.py")),
        "motions": [item[0] for item in MOTIONS],
        "bias_vector_count": len(vectors),
        "seed_range": [args.seed_start, args.seed_start + args.seed_count - 1],
        "prior_information_scales": list(prior_scales),
        "score_threshold": args.score_threshold,
        "trial_count": len(trials),
        "healthy_ratio_min": min(float(item["estimator_healthy_ratio"]) for item in trials),
        "navigation_recoveries_max": max(int(item["navigation_recoveries"]) for item in trials),
        "trial_input_manifest_sha256": trial_manifest_sha256(trials, "input_sha256"),
        "trial_result_manifest_sha256": trial_manifest_sha256(trials, "result_sha256"),
        "aggregate": aggregate(trials, prior_scales),
        "terminal_aggregate": aggregate(trials, prior_scales, terminal_only=True),
        "by_motion": {
            motion_id: aggregate(trials, prior_scales, motion_id=motion_id)
            for motion_id, _, _ in MOTIONS
        },
        "limitations": [
            "The baseline ESKF is never modified; corrected errors are an offline proposal score.",
            "Truth is used only after proposal generation by this evaluator.",
            "No repropagation, covariance reset, timing delay, thermal drift, or physical sensor behavior is included.",
            "Prior-scale comparisons are development evidence and cannot be promoted without a new frozen protocol and holdout.",
        ],
        "git_commit": subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True
        ).strip(),
    }
    if args.include_trials:
        result["trials"] = trials
    output = args.out if args.out.is_absolute() else ROOT / args.out
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
