#!/usr/bin/env python3
"""Compare causal observability diagnostics across IMU rates and noise models.

This is an analyzer-only study.  It does not change the ESKF, tune thresholds,
or promote a fixed-lag correction.  The study keeps the aiding stream at 10 Hz
and separates estimator numerical health from the local information rank.
Temporary replay files are removed after each case unless ``--keep-work`` is
specified; the compact JSON result is the durable artifact.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import shutil
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
GENERATOR = ROOT / "simulation/tools/generate_synthetic_imu.py"
ANALYZER = ROOT / "validation/analyze_bias_excitation_information.py"
PROTOCOL = ROOT / "validation/bias_observability_input_contract_v2.json"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def fingerprint(path: Path) -> dict[str, object]:
    return {"sha256": sha256(path), "bytes": path.stat().st_size}


def run(command: list[str], *, cwd: Path, log_path: Path) -> None:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("w", encoding="utf-8") as log:
        subprocess.run(command, cwd=cwd, check=True, stdout=log, stderr=subprocess.STDOUT)


def profile_settings(profile: str, rate_hz: float, protocol: dict[str, object]) -> dict[str, float]:
    """Return the complete generated-measurement noise model for one profile."""

    noise = protocol["noise"]
    assert isinstance(noise, dict)
    accel_density = float(noise["accel_density_m_s2_sqrt_hz"])
    gyro_density = float(noise["gyro_density_rad_s_sqrt_hz"])
    baseline = {
        "accel_density": accel_density,
        "gyro_density": gyro_density,
        "mag_noise_ut": 0.20,
        "gps_position_noise_m": 0.50,
        "gps_velocity_noise_m_s": 0.10,
    }
    if profile == "continuous_density":
        return baseline
    if profile == "zero_all_measurement_noise":
        return {key: 0.0 for key in baseline}
    if profile == "fixed_sample_imu_noise":
        # Deliberately non-physical across rates: hold per-sample noise fixed
        # to expose estimator/analyzer sensitivity to this common mistake.
        accel_sample_std = 0.01
        gyro_sample_std_rad_s = math.radians(0.02)
        return {
            **baseline,
            "accel_density": accel_sample_std / math.sqrt(rate_hz),
            "gyro_density": gyro_sample_std_rad_s / math.sqrt(rate_hz),
        }
    raise ValueError(f"unknown profile: {profile}")


def summarize_analyzer(report: dict[str, object], static_prefix_end_s: float) -> dict[str, object]:
    evaluations = report["evaluations"]
    assert isinstance(evaluations, list)
    ready_times = [
        float(item["causal_window"]["stop_time_s"])
        for item in evaluations
        if item["structural_information_ready_analyzer_only"]
    ]
    full_rank_times = [
        float(item["causal_window"]["stop_time_s"])
        for item in evaluations
        if int(item["effective_rank"]) == 5
    ]
    early_full_rank_times = [time for time in full_rank_times if time <= static_prefix_end_s]
    early_ready_times = [time for time in ready_times if time <= static_prefix_end_s]
    ranks = [int(item["effective_rank"]) for item in evaluations]
    final = evaluations[-1]
    assert isinstance(final, dict)
    final_checks = final["structural_readiness_checks_analyzer_only"]
    assert isinstance(final_checks, dict)
    return {
        "evaluation_count": int(report["evaluation_count"]),
        "first_ready_time_s": report["first_structural_information_ready_time_s_analyzer_only"],
        "structural_ready_window_count": len(ready_times),
        "structural_ready_time_span_s": (
            [ready_times[0], ready_times[-1]] if ready_times else None
        ),
        "effective_full_rank_window_count": len(full_rank_times),
        "effective_full_rank_time_span_s": (
            [full_rank_times[0], full_rank_times[-1]] if full_rank_times else None
        ),
        "static_prefix_end_s": static_prefix_end_s,
        "effective_full_rank_before_excitation_count": len(early_full_rank_times),
        "structural_ready_before_excitation_count": len(early_ready_times),
        "maximum_effective_rank": max(ranks, default=0),
        "final_effective_rank": int(final["effective_rank"]),
        "final_minimum_eigenvalue": float(final["minimum_eigenvalue"]),
        "final_condition_number": final["condition_number"],
        "final_window_ready": bool(final["structural_information_ready_analyzer_only"]),
        "final_failed_checks": sorted(name for name, passed in final_checks.items() if not passed),
        "final_unique_aiding_epochs": int(
            final["aiding_coverage"]["accepted_unique_timestamps"]
        ),
    }


def run_case(
    profile: str,
    rate_hz: float,
    runner: Path,
    work_root: Path,
    protocol: dict[str, object],
    seed: int,
    keep_work: bool,
    static_prefix_end_s: float,
) -> dict[str, object]:
    case_dir = work_root / profile / f"rate-{int(rate_hz):03d}hz"
    case_dir.mkdir(parents=True, exist_ok=True)
    replay = case_dir / "input.csv"
    metadata = case_dir / "input-metadata.json"
    results = case_dir / "results.csv"
    provenance = case_dir / "analyzer-provenance.json"
    report_path = case_dir / "analyzer-report.json"
    trajectory = protocol["trajectory"]
    assert isinstance(trajectory, dict)
    settings = profile_settings(profile, rate_hz, protocol)
    generate = [
        sys.executable, str(GENERATOR),
        "--out", str(replay), "--metadata", str(metadata),
        "--duration", str(trajectory["duration_s"]), "--rate", str(rate_hz),
        "--seed", str(seed), "--motion", str(trajectory["motion"]), "--static-hint",
        "--stationarity-source", "causal_imu_window",
        "--measurement-contract", "delta_interval_v2",
        "--accel-time-semantics", "interval_start_zoh",
        "--accel-noise-density-m-s2-sqrt-hz", str(settings["accel_density"]),
        "--gyro-noise-density-rad-s-sqrt-hz", str(settings["gyro_density"]),
        "--mag-noise-ut", str(settings["mag_noise_ut"]),
        "--gps-position-noise-m", str(settings["gps_position_noise_m"]),
        "--gps-velocity-noise-m-s", str(settings["gps_velocity_noise_m_s"]),
        "--rate-invariant-streams",
    ]
    run(generate, cwd=ROOT, log_path=case_dir / "generator.log")
    filter_command = [str(runner), "--cold-start", str(replay), str(results)]
    run(filter_command, cwd=ROOT, log_path=case_dir / "filter.log")
    provenance.write_text(
        json.dumps(
            {
                "schema_version": 2,
                "sources": {"replay": fingerprint(replay), "results": fingerprint(results)},
                "execution": {
                    "artifacts": {
                        "replay_generator": {"path": str(GENERATOR), **fingerprint(GENERATOR)},
                        "estimator_runner": {"path": str(runner), **fingerprint(runner)},
                    },
                    "command": filter_command,
                    "git_commit": subprocess.check_output(
                        ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True
                    ).strip(),
                },
                "causal_estimator_output": {
                    "online_forward_filter": True,
                    "future_samples_used": False,
                    "initialization_source": "causal_sensor_alignment",
                    "truth_seeded_initialization": False,
                    "stationarity_source": "causal_detector",
                    "truth_derived_stationarity": False,
                },
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    analyze = [
        sys.executable, str(ANALYZER),
        "--replay", str(replay), "--results", str(results),
        "--provenance-manifest", str(provenance), "--output", str(report_path),
    ]
    run(analyze, cwd=ROOT, log_path=case_dir / "analyzer.log")
    report = json.loads(report_path.read_text(encoding="utf-8"))
    result_rows = results.read_text(encoding="utf-8").splitlines()
    healthy_index = None
    header = result_rows[0].split(",")
    if "eskf_healthy" in header:
        healthy_index = header.index("eskf_healthy")
    healthy_values = (
        [float(row.split(",")[healthy_index]) for row in result_rows[1:]]
        if healthy_index is not None else []
    )
    recovery_index = header.index("eskf_navigation_recovery_count")
    recovery_values = [int(float(row.split(",")[recovery_index])) for row in result_rows[1:]]
    result = {
        "profile": profile,
        "rate_hz": rate_hz,
        "seed": seed,
        "input_sha256": sha256(replay),
        "estimator_healthy_ratio": sum(value > 0.5 for value in healthy_values) / len(healthy_values),
        "estimator_max_navigation_recovery_count": max(recovery_values, default=0),
        "analyzer": summarize_analyzer(report, static_prefix_end_s),
    }
    if not keep_work:
        shutil.rmtree(case_dir)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runner", type=Path, required=True)
    parser.add_argument("--out", type=Path, default=Path("validation/public/g0_rate_sensitivity.json"))
    parser.add_argument("--work-dir", type=Path, default=Path("build/g0-rate-sensitivity"))
    parser.add_argument("--rates", default="50,100,200,400")
    parser.add_argument("--seed", type=int, default=17)
    parser.add_argument(
        "--static-prefix-end-s", type=float, default=3.0,
        help="known stationary prefix end for this frozen trajectory; diagnostic only",
    )
    parser.add_argument("--keep-work", action="store_true")
    args = parser.parse_args()
    protocol = json.loads(PROTOCOL.read_text(encoding="utf-8"))
    rates = [float(value) for value in args.rates.split(",")]
    if not rates or any(rate <= 0.0 for rate in rates) or len(set(rates)) != len(rates):
        parser.error("--rates must contain unique positive values")
    runner = args.runner.resolve()
    if not runner.is_file():
        parser.error(f"runner does not exist: {runner}")
    work_dir = args.work_dir if args.work_dir.is_absolute() else ROOT / args.work_dir
    output = args.out if args.out.is_absolute() else ROOT / args.out
    if args.static_prefix_end_s < 0.0:
        parser.error("--static-prefix-end-s must be non-negative")
    profiles = ("continuous_density", "zero_all_measurement_noise", "fixed_sample_imu_noise")
    cases = [
        run_case(
            profile, rate, runner, work_dir, protocol, args.seed, args.keep_work,
            args.static_prefix_end_s,
        )
        for profile in profiles
        for rate in rates
    ]
    summary = {
        "schema_version": 1,
        "status": "diagnostic_only_not_estimator_gate",
        "study_id": "g0-bias-observability-rate-sensitivity-v1",
        "dataset": "G0 synthetic bias-observability rate sensitivity",
        "sequence": "bias_cv_hover_axis_pulses; 50/100/200/400 Hz; three noise profiles",
        "samples": len(cases),
        "duration_s": 32.0,
        "healthy_ratio": min(case["estimator_healthy_ratio"] for case in cases),
        "protocol_sha256": sha256(PROTOCOL),
        "generator_sha256": sha256(GENERATOR),
        "analyzer_sha256": sha256(ANALYZER),
        "runner_sha256": sha256(runner),
        "rates_hz": rates,
        "profiles": list(profiles),
        "cases": cases,
        "limitations": [
            "The analyzer omits process noise, preintegration covariance, aiding correlations, and reset Jacobians.",
            "Full-rank windows are structural diagnostics, not confidence or correction triggers.",
            "The synthetic trajectory has finite excitation; a trailing window may correctly lose historical rank.",
            "The estimator still consumes rate/specific-force samples; delta fields are contract evidence only.",
        ],
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
