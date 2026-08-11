#!/usr/bin/env python3
"""Run a compact paired screen for the full-state sensitivity diagnostic."""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "validation"))

import full_state_sensitivity_proposal as full  # noqa: E402
import run_fixed_lag_replay_candidate_campaign as metrics  # noqa: E402


GENERATOR = ROOT / "simulation/tools/generate_synthetic_imu.py"
DEFAULT_PROTOCOL = ROOT / "validation/full_state_sensitivity_screen_protocol_v1.json"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _run(command: list[str], log_path: Path) -> None:
    completed = subprocess.run(
        command, cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, check=False,
    )
    log_path.write_text(completed.stdout, encoding="utf-8")
    if completed.returncode != 0:
        raise RuntimeError(completed.stdout[-2000:])


def _corrected_path(work_dir: Path, correction: np.ndarray) -> Path:
    label = "state-" + hashlib.sha256(correction.astype(np.float64).tobytes()).hexdigest()[:20]
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
    directory = work_root / trial_id
    if directory.exists():
        shutil.rmtree(directory)
    directory.mkdir(parents=True)
    replay = directory / "input.csv"
    metadata = directory / "input-metadata.json"
    baseline = directory / "baseline.csv"
    snapshot = directory / "boundary-snapshot.json"
    inputs = protocol["input_contract"]
    solver = protocol["solver"]
    criteria = protocol["criteria"]
    assert isinstance(inputs, dict) and isinstance(solver, dict) and isinstance(criteria, dict)
    stop_s = float(trajectory["duration_s"]) - float(solver["window_stop_margin_s"])
    start_s = stop_s - float(solver["window_duration_s"])
    command = [
        sys.executable, str(GENERATOR), "--out", str(replay), "--metadata", str(metadata),
        "--duration", str(trajectory["duration_s"]), "--rate", str(inputs["rate_hz"]),
        "--seed", str(seed), "--motion", str(trajectory["motion"]), "--static-hint",
        "--stationarity-source", str(inputs["stationarity_source"]),
        "--measurement-contract", str(inputs["measurement_contract"]),
        "--accel-time-semantics", str(inputs["acceleration_time_semantics"]),
        "--accel-noise-density-m-s2-sqrt-hz", str(inputs["accel_noise_density_m_s2_sqrt_hz"]),
        "--gyro-noise-density-rad-s-sqrt-hz", str(inputs["gyro_noise_density_rad_s_sqrt_hz"]),
        "--mag-noise-ut", str(inputs["mag_noise_ut"]),
        "--gps-position-noise-m", str(inputs["gps_position_noise_m"]),
        "--gps-velocity-noise-m-s", str(inputs["gps_velocity_noise_m_s"]),
        "--accel-bias-vector-m-s2", *(str(value) for value in vector["bias_m_s2"]),
        "--gyro-bias-vector-deg-s", "0", "0", "0", "--rate-invariant-streams",
    ]
    try:
        _run(command, directory / "generator.log")
        _run([
            str(runner), "--cold-start", "--snapshot-at-s", f"{start_s:.12g}",
            "--snapshot-out", str(snapshot), str(replay), str(baseline),
        ], directory / "baseline.log")
        baseline_rows = metrics.read_rows(baseline)
        result = full.solve_window(
            runner, replay, baseline, snapshot, start_s, stop_s,
            prior_information_scale=float(solver["prior_information_scale"]),
            work_dir=directory / "candidate-replays",
        )
        record: dict[str, object] = {
            "trial_id": trial_id,
            "trajectory_id": trajectory["id"],
            "vector_id": vector["id"],
            "bias_m_s2": vector["bias_m_s2"],
            "seed": seed,
            "input_sha256": sha256(replay),
            "baseline_sha256": sha256(baseline),
            "snapshot_sha256": sha256(snapshot),
            "status": result["status"],
            "candidate": result,
            "baseline_metrics": metrics.trace_metrics(
                baseline_rows, start_s, stop_s, float(criteria["terminal_window_s"])
            ),
        }
        if result["status"] == "full_state_computed_replayed":
            correction = np.asarray(result["correction_physical"], dtype=np.float64)
            corrected_rows = metrics.read_rows(_corrected_path(directory / "candidate-replays", correction))
            candidate_metrics = metrics.trace_metrics(
                corrected_rows, start_s, stop_s, float(criteria["terminal_window_s"])
            )
            record["candidate_metrics"] = candidate_metrics
            record["paired_delta"] = {
                key: float(candidate_metrics[key]) - float(record["baseline_metrics"][key])  # type: ignore[index]
                for key in (
                    "horizontal_bias_terminal_p95_m_s2", "attitude_rmse_deg",
                    "position_rmse_m", "velocity_rmse_m_s",
                )
            }
        return record
    finally:
        if not keep_work:
            shutil.rmtree(directory, ignore_errors=True)


def summary(records: list[dict[str, object]], protocol: dict[str, object]) -> dict[str, object]:
    criteria = protocol["criteria"]
    assert isinstance(criteria, dict)
    complete = [record for record in records if "candidate_metrics" in record]
    nonzero = [record for record in complete if record["vector_id"] != "zero"]
    improved = [
        record for record in nonzero
        if float(record["paired_delta"]["horizontal_bias_terminal_p95_m_s2"]) < 0.0  # type: ignore[index]
    ]
    zero_regressions = [
        record["trial_id"] for record in complete if record["vector_id"] == "zero"
        and float(record["paired_delta"]["horizontal_bias_terminal_p95_m_s2"])
        > float(criteria["zero_bias_max_terminal_bias_regression_m_s2"])  # type: ignore[index]
    ]
    attitude_regressions = [
        record["trial_id"] for record in complete
        if float(record["paired_delta"]["attitude_rmse_deg"])
        > float(criteria["max_attitude_rmse_regression_deg"])  # type: ignore[index]
    ]
    fraction = len(improved) / len(nonzero) if nonzero else 0.0
    return {
        "trial_count": len(records),
        "completed_candidate_count": len(complete),
        "rejected_candidate_count": len(records) - len(complete),
        "improved_nonzero_trials": len(improved),
        "improved_nonzero_trial_fraction": fraction,
        "zero_bias_material_regressions": zero_regressions,
        "attitude_material_regressions": attitude_regressions,
        "screen_pass": (
            len(complete) == len(records)
            and fraction >= float(criteria["minimum_improved_nonzero_trial_fraction"])
            and not zero_regressions and not attitude_regressions
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runner", type=Path, required=True)
    parser.add_argument("--protocol", type=Path, default=DEFAULT_PROTOCOL)
    parser.add_argument("--out", type=Path, default=Path("build/full-state-screen.json"))
    parser.add_argument("--work-dir", type=Path, default=Path("build/full-state-screen"))
    parser.add_argument("--jobs", type=int, default=1)
    parser.add_argument("--keep-work", action="store_true")
    args = parser.parse_args()
    if args.jobs <= 0:
        parser.error("jobs must be positive")
    protocol_path = args.protocol if args.protocol.is_absolute() else ROOT / args.protocol
    protocol = json.loads(protocol_path.read_text(encoding="utf-8"))
    if protocol.get("status") != "frozen_development_screen_not_promotion":
        parser.error("protocol is not frozen for this screen")
    runner = args.runner.resolve()
    work = args.work_dir if args.work_dir.is_absolute() else ROOT / args.work_dir
    output = args.out if args.out.is_absolute() else ROOT / args.out
    tasks = [
        (trajectory, vector, seed)
        for trajectory in protocol["trajectories"]
        for vector in protocol["vectors"]
        for seed in range(int(trajectory["seed_start"]), int(trajectory["seed_stop"]))
    ]
    work.mkdir(parents=True, exist_ok=True)
    with ThreadPoolExecutor(max_workers=args.jobs) as executor:
        records = list(executor.map(
            lambda task: run_trial(runner, work, protocol, *task, args.keep_work), tasks
        ))
    records.sort(key=lambda record: str(record["trial_id"]))
    result = {
        "schema_version": 1,
        "status": "screen_passed_not_promoted" if summary(records, protocol)["screen_pass"] else "screen_rejected_or_incomplete",
        "protocol_sha256": sha256(protocol_path),
        "runner_sha256": sha256(runner),
        "solver_sha256": sha256(ROOT / "validation/full_state_sensitivity_proposal.py"),
        "git_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        "summary": summary(records, protocol),
        "trials": records,
        "limitations": protocol["limits"],
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({key: value for key, value in result.items() if key != "trials"}, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
