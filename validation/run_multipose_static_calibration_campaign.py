#!/usr/bin/env python3
"""Compare one-pose cold start with an explicit multi-pose IMU calibration.

The candidate receives only a six-pose stationary calibration CSV.  Its seed
is produced by the public C calibration CLI, never by copying simulator truth
directly into the ESKF.  This is synthetic development evidence: it verifies
the implementation and its declared fault envelope, not a physical IMU
calibration or flight qualification.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import subprocess
import sys
import tempfile
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
from typing import Any

import numpy as np

import run_residual_bias_boundary as boundary


ROOT = Path(__file__).resolve().parents[1]
GENERATOR = ROOT / "simulation" / "tools" / "generate_synthetic_imu.py"
ANALYZER = ROOT / "validation" / "analyze_results.py"

# Six orthogonal gravity directions are easy to collect with a fixture and
# leave no direction unobserved in the static sphere fit.
POSE_DIRECTIONS = np.asarray(
    (
        (1.0, 0.0, 0.0), (-1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0), (0.0, -1.0, 0.0),
        (0.0, 0.0, 1.0), (0.0, 0.0, -1.0),
    ),
    dtype=np.float64,
)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def git_commit(root: Path) -> str | None:
    completed = subprocess.run(
        ["git", "-C", str(root), "rev-parse", "HEAD"],
        capture_output=True,
        text=True,
        check=False,
    )
    return completed.stdout.strip() if completed.returncode == 0 else None


def git_worktree_status(root: Path) -> list[str] | None:
    completed = subprocess.run(
        ["git", "-C", str(root), "status", "--porcelain=v1"],
        capture_output=True,
        text=True,
        check=False,
    )
    if completed.returncode != 0:
        return None
    return completed.stdout.splitlines()


def source_manifest(root: Path) -> dict[str, str]:
    """Hash all source inputs that define a campaign result, including dirty files."""
    paths = (
        GENERATOR,
        ANALYZER,
        root / "validation" / "run_residual_bias_boundary.py",
        root / "validation" / "thresholds.json",
        root / "validation" / "validation_runner.c",
        root / "validation" / "static_imu_calibration_cli.c",
        root / "src" / "static_imu_calibration.c",
        root / "src" / "eskf_adapter.c",
        root / "src" / "eskf.c",
        root / "include" / "aerakia" / "static_imu_calibration.h",
        root / "include" / "aerakia" / "eskf_adapter.h",
        root / "include" / "aerakia" / "eskf.h",
    )
    return {str(path.relative_to(root)): sha256_file(path) for path in paths}


def run_checked(command: list[str], timeout_s: float) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        command,
        capture_output=True,
        text=True,
        timeout=timeout_s,
        check=False,
    )


def calibration_noise_seed(validation_seed: int) -> int:
    """Use a stream distinct from the motion/noise stream but common to every case."""
    return (0x5A17_6C31 ^ validation_seed) & 0xFFFF_FFFF


def write_synthetic_pose_means(
    path: Path,
    *,
    acceleration_bias_m_s2: list[float],
    gyroscope_bias_deg_s: list[float],
    raw_acceleration_noise_m_s2: float,
    raw_gyroscope_noise_deg_s: float,
    samples_per_pose: int,
    seed: int,
) -> dict[str, Any]:
    """Write averaged stationary means without exposing simulator truth to the ESKF.

    The known constant biases are used only by this synthetic sensor generator,
    exactly as physical bias would enter raw pose means.  The candidate reads
    only the resulting CSV through the C calibration executable.
    """
    if samples_per_pose < 1:
        raise ValueError("samples_per_pose must be positive")
    if raw_acceleration_noise_m_s2 < 0.0 or raw_gyroscope_noise_deg_s < 0.0:
        raise ValueError("calibration noise scales must be non-negative")
    acceleration_bias = np.asarray(acceleration_bias_m_s2, dtype=np.float64)
    gyroscope_bias_rad_s = np.radians(
        np.asarray(gyroscope_bias_deg_s, dtype=np.float64)
    )
    if acceleration_bias.shape != (3,) or gyroscope_bias_rad_s.shape != (3,):
        raise ValueError("IMU bias vectors must have three axes")
    rng = np.random.default_rng(seed)
    acceleration_mean_noise = raw_acceleration_noise_m_s2 / math.sqrt(samples_per_pose)
    gyroscope_mean_noise = math.radians(raw_gyroscope_noise_deg_s) / math.sqrt(samples_per_pose)
    means = []
    for direction in POSE_DIRECTIONS:
        acceleration = (
            acceleration_bias
            + direction * float(9.80665)
            + rng.normal(0.0, acceleration_mean_noise, size=3)
        )
        gyroscope = gyroscope_bias_rad_s + rng.normal(
            0.0, gyroscope_mean_noise, size=3
        )
        means.append(np.concatenate((acceleration, gyroscope, [samples_per_pose])))
    matrix = np.asarray(means, dtype=np.float64)
    np.savetxt(
        path,
        matrix,
        delimiter=",",
        header=(
            "acc_x_m_s2,acc_y_m_s2,acc_z_m_s2,"
            "gyro_x_rad_s,gyro_y_rad_s,gyro_z_rad_s,sample_count"
        ),
        comments="",
        fmt="%.12g",
    )
    return {
        "pose_count": int(len(matrix)),
        "samples_per_pose": samples_per_pose,
        "raw_acceleration_noise_m_s2": raw_acceleration_noise_m_s2,
        "raw_gyroscope_noise_deg_s": raw_gyroscope_noise_deg_s,
        "mean_acceleration_noise_m_s2": acceleration_mean_noise,
        "mean_gyroscope_noise_rad_s": gyroscope_mean_noise,
        "calibration_noise_seed": seed,
        "pose_means_sha256": sha256_file(path),
    }


def _runner_metrics(
    *,
    runner: Path,
    replay: Path,
    results: Path,
    report_directory: Path,
    timeout_s: float,
    multipose_seed: dict[str, Any] | None = None,
    multipose_pose_means: Path | None = None,
) -> tuple[dict[str, Any] | None, str | None]:
    command = [str(runner), "--cold-start"]
    if multipose_seed is not None and multipose_pose_means is not None:
        raise ValueError("choose one multi-pose input mode")
    if multipose_pose_means is not None:
        command.extend(("--multipose-pose-means", str(multipose_pose_means)))
    if multipose_seed is not None:
        command.extend([
            "--multipose-bias-seed",
            *(str(value) for value in multipose_seed["accelerometer_bias_m_s2"]),
            *(str(value) for value in multipose_seed["gyroscope_bias_rad_s"]),
        ])
    command.extend((str(replay), str(results)))
    completed = run_checked(command, timeout_s)
    if completed.returncode != 0:
        return None, completed.stdout[-4000:] + completed.stderr[-4000:]
    analyzed = run_checked(
        [
            sys.executable, str(ANALYZER), str(results), "--out-dir", str(report_directory),
            "--scenario", "multipose_static_calibration", "--no-plots",
        ],
        timeout_s,
    )
    if analyzed.returncode != 0:
        return None, analyzed.stdout[-4000:] + analyzed.stderr[-4000:]
    return json.loads((report_directory / "metrics.json").read_text(encoding="utf-8")), None


def evaluate_trial(
    case: dict[str, object],
    *,
    runner: Path,
    calibrator: Path,
    limits: dict[str, dict[str, float]],
    validation_seed: int,
    rate_hz: float,
    duration_s: float,
    raw_acceleration_noise_m_s2: float,
    raw_gyroscope_noise_deg_s: float,
    samples_per_pose: int,
    timeout_s: float,
) -> dict[str, Any]:
    case_id = str(case["id"])
    acceleration_bias, gyroscope_bias = boundary.physical_bias_vectors(
        case, 0.05, 0.20, 3.0
    )
    record: dict[str, Any] = {
        "id": case_id,
        "family": str(case["family"]),
        "validation_seed": validation_seed,
        "acceleration_bias_m_s2": acceleration_bias,
        "gyroscope_bias_deg_s": gyroscope_bias,
        "status": "execution_error",
        "failures": [],
    }
    try:
        with tempfile.TemporaryDirectory(prefix="aerakia-multipose-") as temporary:
            work = Path(temporary)
            replay = work / "replay.csv"
            input_metadata = work / "input-metadata.json"
            generated = run_checked(
                [
                    sys.executable, str(GENERATOR), "--out", str(replay),
                    "--metadata", str(input_metadata), "--duration", str(duration_s),
                    "--rate", str(rate_hz), "--seed", str(validation_seed),
                    "--motion", "bias_excitation", "--anomaly", "none",
                    "--accel-bias-std-m-s2", "0.08", "--gyro-bias-std-deg-s", "0.30",
                    "--accel-bias-vector-m-s2", *(str(value) for value in acceleration_bias),
                    "--gyro-bias-vector-deg-s", *(str(value) for value in gyroscope_bias),
                    "--accel-noise-m-s2", str(raw_acceleration_noise_m_s2),
                    "--gyro-noise-deg-s", str(raw_gyroscope_noise_deg_s),
                    "--static-hint", "--stationarity-source", "causal_imu_window",
                    "--stationarity-window-s", "0.5", "--rate-invariant-streams",
                ],
                timeout_s,
            )
            if generated.returncode != 0:
                record["failures"] = ["generator failed: " + generated.stdout[-4000:]]
                return record
            record["replay_sha256"] = sha256_file(replay)
            record["input_metadata_sha256"] = sha256_file(input_metadata)

            pose_means = work / "pose-means.csv"
            calibration_input = write_synthetic_pose_means(
                pose_means,
                acceleration_bias_m_s2=acceleration_bias,
                gyroscope_bias_deg_s=gyroscope_bias,
                raw_acceleration_noise_m_s2=raw_acceleration_noise_m_s2,
                raw_gyroscope_noise_deg_s=raw_gyroscope_noise_deg_s,
                samples_per_pose=samples_per_pose,
                seed=calibration_noise_seed(validation_seed),
            )
            calibration_result = work / "calibration.json"
            calibrated = run_checked(
                [str(calibrator), str(pose_means), str(calibration_result)], timeout_s
            )
            if not calibration_result.exists():
                record["failures"] = ["calibration CLI produced no result"]
                return record
            calibration = json.loads(calibration_result.read_text(encoding="utf-8"))
            record["calibration"] = {
                **calibration_input,
                "result_sha256": sha256_file(calibration_result),
                "cli_returncode": calibrated.returncode,
                "accepted": bool(calibration.get("accepted", False)),
                "status": calibration.get("status"),
                "accelerometer_bias_m_s2": calibration.get("accelerometer_bias_m_s2"),
                "gyroscope_bias_rad_s": calibration.get("gyroscope_bias_rad_s"),
                "gravity_residual_rms_m_s2": calibration.get("gravity_residual_rms_m_s2"),
                "gravity_residual_max_abs_m_s2": calibration.get("gravity_residual_max_abs_m_s2"),
                "gyroscope_residual_rms_rad_s": calibration.get("gyroscope_residual_rms_rad_s"),
                "maximum_leave_one_out_bias_delta_m_s2": calibration.get(
                    "maximum_leave_one_out_bias_delta_m_s2"
                ),
            }
            if calibrated.returncode != 0 or not calibration.get("accepted", False):
                record["failures"] = ["synthetic multi-pose calibration rejected"]
                return record

            baseline, baseline_error = _runner_metrics(
                runner=runner, replay=replay, results=work / "baseline-results.csv",
                report_directory=work / "baseline-report", timeout_s=timeout_s,
            )
            candidate, candidate_error = _runner_metrics(
                runner=runner, replay=replay, results=work / "candidate-results.csv",
                report_directory=work / "candidate-report", timeout_s=timeout_s,
                multipose_pose_means=pose_means,
            )
            if baseline is None or candidate is None:
                record["failures"] = [
                    value for value in (baseline_error, candidate_error) if value is not None
                ] or ["replay runner failed"]
                return record
            accel_active = any(value != 0.0 for value in acceleration_bias)
            gyro_active = any(value != 0.0 for value in gyroscope_bias)
            baseline_metrics = boundary.extract_metrics(baseline)
            candidate_metrics = boundary.extract_metrics(candidate)
            baseline_failures, baseline_skipped = boundary.metric_failures(
                baseline_metrics, limits, accel_active=accel_active, gyro_active=gyro_active
            )
            candidate_failures, candidate_skipped = boundary.metric_failures(
                candidate_metrics, limits, accel_active=accel_active, gyro_active=gyro_active
            )
            calibration_acceleration = np.asarray(
                calibration["accelerometer_bias_m_s2"], dtype=np.float64
            )
            calibration_gyroscope = np.asarray(
                calibration["gyroscope_bias_rad_s"], dtype=np.float64
            )
            truth_gyroscope = np.radians(np.asarray(gyroscope_bias, dtype=np.float64))
            record.update({
                "status": "completed",
                "baseline": {
                    "metrics": baseline_metrics,
                    "failures": baseline_failures,
                    "skipped": baseline_skipped,
                },
                "multipose": {
                    "metrics": candidate_metrics,
                    "failures": candidate_failures,
                    "skipped": candidate_skipped,
                },
                "calibration_error_norms": {
                    "accelerometer_m_s2": float(
                        np.linalg.norm(calibration_acceleration - np.asarray(acceleration_bias))
                    ),
                    "gyroscope_rad_s": float(
                        np.linalg.norm(calibration_gyroscope - truth_gyroscope)
                    ),
                },
                "outcome": {
                    "baseline_passed": not baseline_failures,
                    "multipose_passed": not candidate_failures,
                    "improved_to_pass": bool(baseline_failures) and not candidate_failures,
                    "regressed_from_pass": not baseline_failures and bool(candidate_failures),
                },
            })
            return record
    except (OSError, ValueError, subprocess.TimeoutExpired) as error:
        record["failures"] = [f"{type(error).__name__}: {error}"]
        return record


def summarize(records: list[dict[str, Any]]) -> dict[str, Any]:
    completed = [record for record in records if record.get("status") == "completed"]
    improvements = [record for record in completed if record["outcome"]["improved_to_pass"]]
    regressions = [record for record in completed if record["outcome"]["regressed_from_pass"]]
    baseline_passed = [record for record in completed if record["outcome"]["baseline_passed"]]
    multipose_passed = [record for record in completed if record["outcome"]["multipose_passed"]]
    by_family: dict[str, dict[str, int]] = {}
    for record in records:
        family = str(record.get("family", "unknown"))
        family_summary = by_family.setdefault(
            family,
            {"total": 0, "completed": 0, "baseline_passed": 0, "multipose_passed": 0,
             "improved_to_pass": 0, "regressed_from_pass": 0},
        )
        family_summary["total"] += 1
        if record.get("status") == "completed":
            family_summary["completed"] += 1
            for key in ("baseline_passed", "multipose_passed", "improved_to_pass", "regressed_from_pass"):
                family_summary[key] += int(bool(record["outcome"][key]))
    calibration_errors = [
        record["calibration_error_norms"]["accelerometer_m_s2"] for record in completed
    ]
    return {
        "total": len(records),
        "completed": len(completed),
        "execution_failures": len(records) - len(completed),
        "baseline_passed": len(baseline_passed),
        "multipose_passed": len(multipose_passed),
        "improved_to_pass": len(improvements),
        "regressed_from_pass": len(regressions),
        "families": by_family,
        "calibration_accelerometer_error_norm_m_s2": {
            "mean": float(np.mean(calibration_errors)) if calibration_errors else None,
            "p95": float(np.percentile(calibration_errors, 95)) if calibration_errors else None,
            "maximum": float(np.max(calibration_errors)) if calibration_errors else None,
        },
    }


def parse_seed_list(text: str) -> list[int]:
    values = [item.strip() for item in text.split(",") if item.strip()]
    if not values:
        raise ValueError("at least one validation seed is required")
    parsed = [int(value, 10) for value in values]
    if len(set(parsed)) != len(parsed):
        raise ValueError("validation seeds must be unique")
    if any(value < 0 or value > 0xFFFF_FFFF for value in parsed):
        raise ValueError("validation seeds must be unsigned 32-bit integers")
    return parsed


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runner", type=Path, required=True)
    parser.add_argument("--calibrator", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, default=Path("build/g0-multipose-static-calibration"))
    parser.add_argument("--seeds", default="41001,41003,41009,41021")
    parser.add_argument("--case-ids", default="", help="optional comma-separated development cases")
    parser.add_argument("--rate", type=float, default=100.0)
    parser.add_argument("--duration", type=float, default=40.0)
    parser.add_argument("--jobs", type=int, default=4)
    parser.add_argument("--timeout-s", type=float, default=120.0)
    parser.add_argument("--samples-per-pose", type=int, default=400)
    parser.add_argument("--accel-noise-m-s2", type=float, default=0.02)
    parser.add_argument("--gyro-noise-deg-s", type=float, default=0.05)
    args = parser.parse_args()
    if args.rate <= 0.0 or args.duration <= 2.0 or args.jobs < 1 or args.timeout_s <= 0.0:
        parser.error("rate, duration, jobs, and timeout must be positive; duration must exceed 2 s")
    if args.samples_per_pose < 1 or args.accel_noise_m_s2 < 0.0 or args.gyro_noise_deg_s < 0.0:
        parser.error("calibration samples must be positive and noise values non-negative")
    try:
        seeds = parse_seed_list(args.seeds)
    except ValueError as error:
        parser.error(str(error))
    runner = args.runner.resolve()
    calibrator = args.calibrator.resolve()
    if not runner.is_file() or not calibrator.is_file():
        parser.error("runner and calibrator must be existing executables")
    cases = boundary.build_boundary_cases()
    if args.case_ids:
        requested = {item.strip() for item in args.case_ids.split(",") if item.strip()}
        known = {str(case["id"]) for case in cases}
        unknown = sorted(requested - known)
        if unknown:
            parser.error("unknown case id(s): " + ", ".join(unknown))
        cases = [case for case in cases if str(case["id"]) in requested]
    if not cases:
        parser.error("no cases selected")
    limits = boundary.reviewed_limits(ROOT / "validation" / "thresholds.json")
    output_directory = args.out_dir if args.out_dir.is_absolute() else ROOT / args.out_dir
    output_directory.mkdir(parents=True, exist_ok=True)
    work: list[tuple[dict[str, object], int]] = [
        (case, seed) for seed in seeds for case in cases
    ]
    started = time.monotonic()
    records: list[dict[str, Any]] = []
    with ThreadPoolExecutor(max_workers=args.jobs) as executor:
        futures = {
            executor.submit(
                evaluate_trial,
                case,
                runner=runner,
                calibrator=calibrator,
                limits=limits,
                validation_seed=seed,
                rate_hz=args.rate,
                duration_s=args.duration,
                raw_acceleration_noise_m_s2=args.accel_noise_m_s2,
                raw_gyroscope_noise_deg_s=args.gyro_noise_deg_s,
                samples_per_pose=args.samples_per_pose,
                timeout_s=args.timeout_s,
            ): (str(case["id"]), seed)
            for case, seed in work
        }
        completed_count = 0
        for future in as_completed(futures):
            case_id, seed = futures[future]
            try:
                records.append(future.result())
            except Exception as error:  # Defensive campaign continuity.
                records.append({
                    "id": case_id, "validation_seed": seed, "status": "execution_error",
                    "failures": [f"{type(error).__name__}: {error}"],
                })
            completed_count += 1
            if completed_count == 1 or completed_count % 25 == 0 or completed_count == len(work):
                print(f"completed {completed_count}/{len(work)} paired trials")
    order = {str(case["id"]): index for index, case in enumerate(cases)}
    records.sort(key=lambda record: (int(record["validation_seed"]), order.get(str(record["id"]), 999999)))
    aggregate = summarize(records)
    elapsed_s = time.monotonic() - started
    all_candidate_pass = (
        aggregate["completed"] == aggregate["total"]
        and aggregate["multipose_passed"] == aggregate["total"]
        and aggregate["regressed_from_pass"] == 0
    )
    summary = {
        "schema_version": 1,
        "status": "passed" if all_candidate_pass else "failed",
        "scope": {
            "kind": "synthetic opened-development A/B",
            "candidate": "explicit six-pose static gravity-sphere calibration before cold start",
            "baseline": "existing one-pose static tilt/bias initialization",
            "candidate_information_boundary": (
                "C calibration CLI receives only synthetic stationary pose means; no trajectory "
                "truth or future replay sample is supplied to the ESKF."
            ),
            "not_claimed": [
                "physical FCOne calibration accuracy", "temperature, scale, or misalignment compensation",
                "sealed holdout closure", "flight readiness or airworthiness",
            ],
        },
        "provenance": {
            "source_git_commit": git_commit(ROOT),
            "source_worktree_status": git_worktree_status(ROOT),
            "source_manifest_sha256": source_manifest(ROOT),
            "runner_sha256": sha256_file(runner),
            "calibrator_sha256": sha256_file(calibrator),
            "generator_sha256": sha256_file(GENERATOR),
            "analyzer_sha256": sha256_file(ANALYZER),
        },
        "protocol": {
            "case_count_per_seed": len(cases), "seed_count": len(seeds), "seeds": seeds,
            "rate_hz": args.rate, "duration_s": args.duration,
            "samples_per_pose": args.samples_per_pose,
            "calibration_pose_directions": POSE_DIRECTIONS.tolist(),
            "calibration_noise_common_per_seed": True,
            "calibration_noise_seed_rule": "0x5A176C31 XOR validation_seed",
            "trajectory_noise_common_per_seed_across_cases": True,
        },
        "reviewed_limits": limits,
        "aggregate": aggregate,
        "runtime_s": elapsed_s,
        "records": records,
    }
    (output_directory / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    lines = [
        "# Multi-pose static calibration A/B campaign", "",
        f"Status: **{summary['status']}**. Paired trials: **{aggregate['total']}**; "
        f"candidate passes: **{aggregate['multipose_passed']}**; baseline passes: "
        f"**{aggregate['baseline_passed']}**; improvements: **{aggregate['improved_to_pass']}**; "
        f"regressions: **{aggregate['regressed_from_pass']}**.", "",
        "This is synthetic opened-development evidence. It validates the explicit C calibration "
        "path and cold-start use of its output, not physical hardware calibration or flight readiness.",
        "",
        "| Family | Trials | Baseline pass | Multi-pose pass | Improved | Regressed |",
        "| --- | ---: | ---: | ---: | ---: | ---: |",
    ]
    for family, counts in sorted(aggregate["families"].items()):
        lines.append(
            f"| {family} | {counts['total']} | {counts['baseline_passed']} | "
            f"{counts['multipose_passed']} | {counts['improved_to_pass']} | "
            f"{counts['regressed_from_pass']} |"
        )
    lines.extend([
        "", "Temporary replay CSVs and full result CSVs were held in per-trial temporary directories "
        "and removed after metrics/provenance extraction. `summary.json` retains commands' code "
        "identity, input hashes, calibration diagnostics, and every per-trial result.",
    ])
    (output_directory / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"Multi-pose calibration report: {output_directory / 'report.md'}")
    if not all_candidate_pass:
        raise SystemExit("multi-pose calibration candidate did not satisfy all selected development gates")


if __name__ == "__main__":
    main()
