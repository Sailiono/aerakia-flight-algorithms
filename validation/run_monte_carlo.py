#!/usr/bin/env python3
"""Run a reviewed multi-seed navigation consistency baseline."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

import numpy as np


LIMITS = {
    "position_rmse_m": 1.25,
    "velocity_rmse_m_s": 0.60,
    "attitude_rmse_deg": 3.50,
    "navigation_recoveries": 0.0,
    "minimum_healthy_ratio": 1.0,
}

DISTRIBUTION_P95_LIMITS = {
    "position_rmse_m": 0.75,
    "velocity_rmse_m_s": 0.40,
    "attitude_rmse_deg": 2.00,
}

AGGREGATE_CONSISTENCY_LIMITS = {
    "position_nis_mean": (2.0, 4.0),
    "velocity_nis_mean": (2.0, 4.0),
    "navigation_nees_mean": (3.0, 9.0),
}

FULL_STATISTICAL_GATE_TRIALS = 1000
BOOTSTRAP_RESAMPLES = 10000
BOOTSTRAP_CONFIDENCE = 0.99
MAXIMUM_ZERO_FAILURE_PROBABILITY_95 = 0.003


def _canonical_json(value: object) -> str:
    return json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True, allow_nan=False
    )


def _sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def git_identity(root: Path) -> dict[str, object]:
    commit = subprocess.run(
        ["git", "rev-parse", "HEAD"], cwd=root, check=True,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
    ).stdout.strip()
    dirty_output = subprocess.run(
        ["git", "status", "--porcelain", "--untracked-files=normal"],
        cwd=root, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
    ).stdout
    return {"commit": commit, "dirty": bool(dirty_output.strip())}


def make_protocol_fingerprint(protocol: dict[str, object]) -> dict[str, object]:
    canonical = _canonical_json(protocol)
    return {
        "schema_version": 1,
        "sha256": _sha256_bytes(canonical.encode("utf-8")),
        "protocol": protocol,
    }


def build_protocol_fingerprint(
    *,
    root: Path,
    runner: Path,
    scenario: str,
    duration_s: float,
    rate_hz: float,
    timestamp_jitter_std_us: float,
    accel_noise_m_s2: float,
    gyro_noise_deg_s: float,
    mag_noise_ut: float,
    gps_position_noise_m: float,
    gps_velocity_noise_m_s: float,
    accel_bias_std_m_s2: float,
    gyro_bias_std_deg_s: float,
    bias_sigma_limit: float,
) -> dict[str, object]:
    generator = root / "simulation/tools/generate_synthetic_imu.py"
    orchestrator = root / "validation/run_suite.py"
    analyzer = root / "validation/analyze_results.py"
    deterministic_thresholds = root / "validation/thresholds.json"
    threshold_checker = root / "validation/check_thresholds.py"
    resolved_runner = runner.resolve()
    source_paths = {
        "generator_sha256": generator,
        "orchestrator_sha256": orchestrator,
        "analyzer_sha256": analyzer,
        "runner_sha256": resolved_runner,
        "deterministic_thresholds_sha256": deterministic_thresholds,
        "threshold_checker_sha256": threshold_checker,
    }
    missing = [str(path) for path in source_paths.values() if not path.is_file()]
    if missing:
        raise FileNotFoundError("protocol source missing: " + ", ".join(missing))
    gate_policy = {
        "hard_limits": LIMITS,
        "distribution_p95_limits": DISTRIBUTION_P95_LIMITS,
        "aggregate_consistency_limits": AGGREGATE_CONSISTENCY_LIMITS,
        "full_statistical_gate_trials": FULL_STATISTICAL_GATE_TRIALS,
        "bootstrap_resamples": BOOTSTRAP_RESAMPLES,
        "bootstrap_confidence": BOOTSTRAP_CONFIDENCE,
        "maximum_zero_failure_probability_95": MAXIMUM_ZERO_FAILURE_PROBABILITY_95,
    }
    protocol: dict[str, object] = {
        "scenario": scenario,
        "duration_s": duration_s,
        "rate_hz": rate_hz,
        "timestamp_jitter_std_us": timestamp_jitter_std_us,
        "measurement_noise": {
            "accel_noise_m_s2": accel_noise_m_s2,
            "gyro_noise_deg_s": gyro_noise_deg_s,
            "mag_noise_ut": mag_noise_ut,
            "gps_position_noise_m": gps_position_noise_m,
            "gps_velocity_noise_m_s": gps_velocity_noise_m_s,
        },
        "bias_prior": {
            "accel_bias_std_m_s2": accel_bias_std_m_s2,
            "gyro_bias_std_deg_s": gyro_bias_std_deg_s,
            "bias_sigma_limit": bias_sigma_limit,
        },
        "sources": {
            name: file_sha256(path) for name, path in source_paths.items()
        },
        "gate_policy_sha256": _sha256_bytes(
            _canonical_json(gate_policy).encode("utf-8")
        ),
        "git": git_identity(root),
    }
    return make_protocol_fingerprint(protocol)


def require_protocol_match(
    record: dict[str, object], expected: dict[str, object], *, source: str
) -> None:
    actual = record.get("protocol_fingerprint")
    if actual != expected:
        actual_sha = actual.get("sha256") if isinstance(actual, dict) else None
        raise ValueError(
            f"protocol fingerprint mismatch in {source}: "
            f"expected {expected['sha256']}, found {actual_sha or 'missing'}"
        )


def parse_seeds(expression: str) -> list[int]:
    seeds: list[int] = []
    for item in expression.split(","):
        item = item.strip()
        if not item:
            continue
        if ":" in item:
            start_text, stop_text = item.split(":", 1)
            seeds.extend(range(int(start_text), int(stop_text)))
        else:
            seeds.append(int(item))
    unique = list(dict.fromkeys(seeds))
    if not unique:
        raise ValueError("at least one seed is required")
    return unique


def extract_trial(seed: int, metrics: dict[str, object]) -> dict[str, float | int]:
    navigation = metrics["navigation"]
    integrity = metrics["eskf_integrity"]
    consistency = metrics["eskf_consistency"]
    alignment = metrics["cold_start_alignment"]
    assert isinstance(navigation, dict)
    assert isinstance(integrity, dict)
    assert isinstance(consistency, dict)
    assert isinstance(alignment, dict)
    position_nis = consistency["position_nis"]
    velocity_nis = consistency["velocity_nis"]
    navigation_nees = consistency["navigation_nees"]
    assert isinstance(position_nis, dict)
    assert isinstance(velocity_nis, dict)
    assert isinstance(navigation_nees, dict)
    return {
        "seed": seed,
        "position_rmse_m": float(navigation["position_rmse_m"]),
        "velocity_rmse_m_s": float(navigation["velocity_rmse_m_s"]),
        "attitude_rmse_deg": float(alignment["post_alignment_attitude_rmse_deg"]),
        "position_nis_mean": float(position_nis["mean"]),
        "velocity_nis_mean": float(velocity_nis["mean"]),
        "navigation_nees_mean": float(navigation_nees["mean"]),
        "healthy_ratio": float(integrity["healthy_ratio"]),
        "navigation_recoveries": int(integrity["navigation_recoveries"]),
    }


def summarize_trials(trials: list[dict[str, float | int]]) -> dict[str, object]:
    metric_names = [
        "position_rmse_m",
        "velocity_rmse_m_s",
        "attitude_rmse_deg",
        "position_nis_mean",
        "velocity_nis_mean",
        "navigation_nees_mean",
        "healthy_ratio",
        "navigation_recoveries",
    ]
    aggregates: dict[str, dict[str, float]] = {}
    for name in metric_names:
        values = np.asarray([float(trial[name]) for trial in trials], dtype=np.float64)
        aggregates[name] = {
            "minimum": float(np.min(values)),
            "mean": float(np.mean(values)),
            "p05": float(np.percentile(values, 5)),
            "p95": float(np.percentile(values, 95)),
            "maximum": float(np.max(values)),
        }

    failures: list[dict[str, object]] = []
    for trial in trials:
        reasons: list[str] = []
        for name in ("position_rmse_m", "velocity_rmse_m_s", "attitude_rmse_deg"):
            if float(trial[name]) > LIMITS[name]:
                reasons.append(f"{name}={float(trial[name]):.6g}>{LIMITS[name]:.6g}")
        if float(trial["healthy_ratio"]) < LIMITS["minimum_healthy_ratio"]:
            reasons.append(
                f"healthy_ratio={float(trial['healthy_ratio']):.6g}"
                f"<{LIMITS['minimum_healthy_ratio']:.6g}"
            )
        if float(trial["navigation_recoveries"]) > LIMITS["navigation_recoveries"]:
            reasons.append(
                f"navigation_recoveries={int(trial['navigation_recoveries'])}"
                f">{int(LIMITS['navigation_recoveries'])}"
            )
        if reasons:
            failures.append({"seed": int(trial["seed"]), "reasons": reasons})
    consistency_failures: list[str] = []
    for name, (minimum, maximum) in AGGREGATE_CONSISTENCY_LIMITS.items():
        mean = aggregates[name]["mean"]
        if mean < minimum or mean > maximum:
            consistency_failures.append(
                f"{name}.mean={mean:.6g} outside [{minimum:.6g}, {maximum:.6g}]"
            )
    distribution_failures: list[str] = []
    for name, maximum in DISTRIBUTION_P95_LIMITS.items():
        p95 = aggregates[name]["p95"]
        if p95 > maximum:
            distribution_failures.append(
                f"{name}.p95={p95:.6g}>{maximum:.6g}"
            )
    return {
        "aggregates": aggregates,
        "failures": failures,
        "consistency_failures": consistency_failures,
        "distribution_failures": distribution_failures,
    }


def confidence_summary(
    trials: list[dict[str, float | int]],
    *,
    hard_failure_count: int,
    resamples: int = BOOTSTRAP_RESAMPLES,
    confidence: float = BOOTSTRAP_CONFIDENCE,
) -> dict[str, object]:
    """Return deterministic bootstrap bounds and the exact zero-failure upper bound."""
    if not trials or resamples < 1 or not 0.0 < confidence < 1.0:
        raise ValueError("trials, resamples, and confidence must be valid")
    trial_count = len(trials)
    rng = np.random.default_rng(0xA3EA51A)
    indices = rng.integers(
        0, trial_count, size=(resamples, trial_count), dtype=np.int32
    )
    lower_percentile = 50.0 * (1.0 - confidence)
    upper_percentile = 100.0 - lower_percentile
    p95_bounds: dict[str, dict[str, float]] = {}
    mean_bounds: dict[str, dict[str, float]] = {}
    for name in DISTRIBUTION_P95_LIMITS:
        values = np.asarray([float(trial[name]) for trial in trials], dtype=np.float64)
        bootstrapped = np.percentile(values[indices], 95.0, axis=1)
        p95_bounds[name] = {
            "lower": float(np.percentile(bootstrapped, lower_percentile)),
            "upper": float(np.percentile(bootstrapped, upper_percentile)),
        }
    for name in AGGREGATE_CONSISTENCY_LIMITS:
        values = np.asarray([float(trial[name]) for trial in trials], dtype=np.float64)
        bootstrapped = np.mean(values[indices], axis=1)
        mean_bounds[name] = {
            "lower": float(np.percentile(bootstrapped, lower_percentile)),
            "upper": float(np.percentile(bootstrapped, upper_percentile)),
        }
    zero_failure_upper = (
        1.0 - 0.05 ** (1.0 / trial_count) if hard_failure_count == 0 else None
    )
    failures: list[str] = []
    if trial_count >= FULL_STATISTICAL_GATE_TRIALS:
        for name, limit in DISTRIBUTION_P95_LIMITS.items():
            if p95_bounds[name]["upper"] > limit:
                failures.append(
                    f"{name}.p95_bootstrap_upper={p95_bounds[name]['upper']:.6g}>{limit:.6g}"
                )
        for name, (minimum, maximum) in AGGREGATE_CONSISTENCY_LIMITS.items():
            bounds = mean_bounds[name]
            if bounds["lower"] < minimum or bounds["upper"] > maximum:
                failures.append(
                    f"{name}.mean_bootstrap=[{bounds['lower']:.6g}, {bounds['upper']:.6g}] "
                    f"outside [{minimum:.6g}, {maximum:.6g}]"
                )
        if zero_failure_upper is None or zero_failure_upper > MAXIMUM_ZERO_FAILURE_PROBABILITY_95:
            failures.append(
                "zero-failure 95-percent upper probability does not meet 0.003"
            )
    return {
        "trial_count": trial_count,
        "resamples": resamples,
        "confidence": confidence,
        "p95_bounds": p95_bounds,
        "mean_bounds": mean_bounds,
        "zero_failure_probability_upper_95": zero_failure_upper,
        "full_gate_applied": trial_count >= FULL_STATISTICAL_GATE_TRIALS,
        "failures": failures,
    }


def run_trial(
    seed: int,
    *,
    root: Path,
    runner: Path,
    out_dir: Path,
    duration_s: float,
    rate_hz: float,
    accel_bias_std_m_s2: float,
    gyro_bias_std_deg_s: float,
    bias_sigma_limit: float,
    timestamp_jitter_std_us: float,
    accel_noise_m_s2: float,
    gyro_noise_deg_s: float,
    mag_noise_ut: float,
    gps_position_noise_m: float,
    gps_velocity_noise_m_s: float,
    protocol_fingerprint: dict[str, object],
    environment: dict[str, str],
    compact: bool,
    timeout_s: float,
    retries: int,
) -> dict[str, float | int | dict[str, object]]:
    trial_dir = out_dir / f"seed-{seed:04d}"
    command = [
            sys.executable,
            str(root / "validation/run_suite.py"),
            "--runner", str(runner),
            "--out-dir", str(trial_dir),
            "--scenarios", "navigation_outage",
            "--duration", str(duration_s),
            "--rate", str(rate_hz),
            "--seed", str(seed),
            "--accel-bias-std-m-s2", str(accel_bias_std_m_s2),
            "--gyro-bias-std-deg-s", str(gyro_bias_std_deg_s),
            "--bias-sigma-limit", str(bias_sigma_limit),
            "--timestamp-jitter-std-us", str(timestamp_jitter_std_us),
            "--accel-noise-m-s2", str(accel_noise_m_s2),
            "--gyro-noise-deg-s", str(gyro_noise_deg_s),
            "--mag-noise-ut", str(mag_noise_ut),
            "--gps-position-noise-m", str(gps_position_noise_m),
            "--gps-velocity-noise-m-s", str(gps_velocity_noise_m_s),
            "--no-plots",
    ]
    trial_environment = environment.copy()
    trial_environment["MPLCONFIGDIR"] = str((trial_dir / ".matplotlib").resolve())
    last_error: subprocess.CalledProcessError | subprocess.TimeoutExpired | None = None
    for _attempt in range(retries + 1):
        try:
            subprocess.run(
                command,
                cwd=root,
                env=trial_environment,
                check=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=timeout_s,
            )
            last_error = None
            break
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as error:
            last_error = error
    if last_error is not None:
        output = getattr(last_error, "stdout", None) or getattr(last_error, "output", None) or ""
        raise RuntimeError(
            f"seed {seed} failed after {retries + 1} attempt(s): {last_error}\n{output}"
        )
    metrics = json.loads((trial_dir / "navigation_outage/metrics.json").read_text(
        encoding="utf-8"
    ))
    trial = extract_trial(seed, metrics)
    generation = json.loads((trial_dir / "navigation_outage/input-metadata.json").read_text(
        encoding="utf-8"
    ))
    trial["generation"] = generation
    trial["protocol_fingerprint"] = protocol_fingerprint
    if compact:
        shutil.rmtree(trial_dir)
    return trial


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runner", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, default=Path("build/monte-carlo"))
    parser.add_argument("--seeds", default="0:20", help="comma list and/or half-open ranges")
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--rate", type=float, default=100.0)
    parser.add_argument("--accel-bias-std-m-s2", type=float, default=0.05)
    parser.add_argument("--gyro-bias-std-deg-s", type=float, default=0.20)
    parser.add_argument(
        "--bias-sigma-limit", type=float, default=3.0,
        help="per-axis residual-bias prior bound; verify it against FCOne calibration",
    )
    parser.add_argument("--timestamp-jitter-std-us", type=float, default=250.0)
    parser.add_argument("--accel-noise-m-s2", type=float, default=0.02)
    parser.add_argument("--gyro-noise-deg-s", type=float, default=0.05)
    parser.add_argument("--mag-noise-ut", type=float, default=0.20)
    parser.add_argument("--gps-position-noise-m", type=float, default=0.5)
    parser.add_argument("--gps-velocity-noise-m-s", type=float, default=0.1)
    parser.add_argument("--jobs", type=int, default=4)
    parser.add_argument(
        "--compact", action="store_true",
        help="retain complete per-trial facts in summary.json but remove generated CSV/report trees",
    )
    parser.add_argument("--timeout-s", type=float, default=120.0)
    parser.add_argument("--retries", type=int, default=1)
    parser.add_argument(
        "--resume", action="store_true",
        help="reuse completed per-seed JSON checkpoints in the selected output directory",
    )
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[1]
    seeds = parse_seeds(args.seeds)
    environment = os.environ.copy()
    environment["MPLCONFIGDIR"] = str((args.out_dir / ".matplotlib").resolve())
    Path(environment["MPLCONFIGDIR"]).mkdir(parents=True, exist_ok=True)
    if (args.jobs < 1 or args.timeout_s <= 0.0 or args.retries < 0
            or min(
                args.bias_sigma_limit, args.timestamp_jitter_std_us,
                args.accel_noise_m_s2, args.gyro_noise_deg_s, args.mag_noise_ut,
                args.gps_position_noise_m, args.gps_velocity_noise_m_s,
                args.accel_bias_std_m_s2, args.gyro_bias_std_deg_s,
            ) < 0.0):
        parser.error(
            "jobs and timeout must be positive; retries, noise, jitter, and bias values "
            "must be non-negative"
        )
    args.out_dir.mkdir(parents=True, exist_ok=True)
    trial_records_dir = args.out_dir / "trials"
    trial_records_dir.mkdir(parents=True, exist_ok=True)
    protocol_fingerprint = build_protocol_fingerprint(
        root=root,
        runner=args.runner,
        scenario="navigation_outage",
        duration_s=args.duration,
        rate_hz=args.rate,
        timestamp_jitter_std_us=args.timestamp_jitter_std_us,
        accel_noise_m_s2=args.accel_noise_m_s2,
        gyro_noise_deg_s=args.gyro_noise_deg_s,
        mag_noise_ut=args.mag_noise_ut,
        gps_position_noise_m=args.gps_position_noise_m,
        gps_velocity_noise_m_s=args.gps_velocity_noise_m_s,
        accel_bias_std_m_s2=args.accel_bias_std_m_s2,
        gyro_bias_std_deg_s=args.gyro_bias_std_deg_s,
        bias_sigma_limit=args.bias_sigma_limit,
    )
    protocol_path = args.out_dir / "protocol.json"
    if args.resume:
        if protocol_path.is_file():
            saved_protocol = json.loads(protocol_path.read_text(encoding="utf-8"))
            if saved_protocol != protocol_fingerprint:
                raise SystemExit(
                    "resume refused: output-directory protocol fingerprint does not match "
                    f"current protocol ({saved_protocol.get('sha256', 'missing')} != "
                    f"{protocol_fingerprint['sha256']})"
                )
        elif any(trial_records_dir.glob("seed-*.json")) or (args.out_dir / "summary.json").exists():
            raise SystemExit("resume refused: existing Monte Carlo output has no protocol fingerprint")
    protocol_path.write_text(
        json.dumps(protocol_fingerprint, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    trials: list[dict[str, float | int | dict[str, object]]] = []
    pending_seeds: list[int] = []
    for seed in seeds:
        record_path = trial_records_dir / f"seed-{seed:04d}.json"
        if args.resume and record_path.is_file():
            record = json.loads(record_path.read_text(encoding="utf-8"))
            try:
                require_protocol_match(record, protocol_fingerprint, source=str(record_path))
            except ValueError as error:
                raise SystemExit(f"resume refused: {error}") from error
            if int(record.get("seed", -1)) != seed:
                raise SystemExit(
                    f"resume refused: checkpoint {record_path} contains seed "
                    f"{record.get('seed')}, expected {seed}"
                )
            trials.append(record)
        else:
            pending_seeds.append(seed)
    execution_failures: list[dict[str, object]] = []
    with ThreadPoolExecutor(max_workers=args.jobs) as executor:
        futures = {
            executor.submit(
                run_trial,
                seed,
                root=root,
                runner=args.runner.resolve(),
                out_dir=args.out_dir,
                duration_s=args.duration,
                rate_hz=args.rate,
                accel_bias_std_m_s2=args.accel_bias_std_m_s2,
                gyro_bias_std_deg_s=args.gyro_bias_std_deg_s,
                bias_sigma_limit=args.bias_sigma_limit,
                timestamp_jitter_std_us=args.timestamp_jitter_std_us,
                accel_noise_m_s2=args.accel_noise_m_s2,
                gyro_noise_deg_s=args.gyro_noise_deg_s,
                mag_noise_ut=args.mag_noise_ut,
                gps_position_noise_m=args.gps_position_noise_m,
                gps_velocity_noise_m_s=args.gps_velocity_noise_m_s,
                protocol_fingerprint=protocol_fingerprint,
                environment=environment,
                compact=args.compact,
                timeout_s=args.timeout_s,
                retries=args.retries,
            ): seed
            for seed in pending_seeds
        }
        completed = 0
        for future in as_completed(futures):
            seed = futures[future]
            try:
                trial = future.result()
                trials.append(trial)
                (trial_records_dir / f"seed-{seed:04d}.json").write_text(
                    json.dumps(trial, indent=2, sort_keys=True) + "\n", encoding="utf-8"
                )
            except Exception as error:  # retain all other independent trials and report the seed
                execution_failures.append({"seed": seed, "error": str(error)})
            completed += 1
            if completed == 1 or completed % 25 == 0 or completed == len(pending_seeds):
                print(
                    f"completed {completed}/{len(pending_seeds)} pending Monte Carlo trials "
                    f"({len(execution_failures)} execution failures)"
                )
    trials.sort(key=lambda trial: int(trial["seed"]))

    if not trials:
        raise SystemExit("no Monte Carlo trial completed")

    summary = summarize_trials(trials)
    statistical_confidence = confidence_summary(
        trials, hard_failure_count=len(summary["failures"])
    )
    output = {
        "schema_version": 2,
        "protocol_fingerprint": protocol_fingerprint,
        "scope": {
            "scenario": "navigation_outage",
            "seeds": seeds,
            "completed_trials": len(trials),
            "duration_s": args.duration,
            "rate_hz": args.rate,
            "accel_bias_std_m_s2": args.accel_bias_std_m_s2,
            "gyro_bias_std_deg_s": args.gyro_bias_std_deg_s,
            "bias_sigma_limit": args.bias_sigma_limit,
            "timestamp_jitter_std_us": args.timestamp_jitter_std_us,
            "covered": [
                "independent randomized IMU, magnetometer, GNSS position, and GNSS velocity noise",
                "randomized constant three-axis accelerometer and gyroscope bias",
                "declared per-axis residual-bias operating bound",
                "monotonic per-interval IMU timestamp jitter",
                "cold-start alignment",
                "five-second GNSS aiding outage and recovery",
                "navigation NIS and posterior velocity-position NEES",
            ],
            "not_covered": [
                "temperature-dependent or time-varying IMU bias",
                "transport delay, reordering, or dropped IMU samples",
                "independent physical truth or flight qualification",
            ],
        },
        "limits": LIMITS,
        "distribution_p95_limits": DISTRIBUTION_P95_LIMITS,
        "aggregate_consistency_limits": AGGREGATE_CONSISTENCY_LIMITS,
        "execution_failures": execution_failures,
        "trials": trials,
        "statistical_confidence": statistical_confidence,
        **summary,
    }
    args.out_dir.mkdir(parents=True, exist_ok=True)
    (args.out_dir / "summary.json").write_text(
        json.dumps(output, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )

    aggregates = summary["aggregates"]
    assert isinstance(aggregates, dict)
    lines = [
        "# Navigation Monte Carlo baseline",
        "",
        f"Seeds: `{seeds[0]}` through `{seeds[-1]}` "
        f"({len(trials)}/{len(seeds)} completed trials).",
        "",
        "| Metric | Mean | P05 | P95 | Maximum |",
        "| --- | ---: | ---: | ---: | ---: |",
    ]
    for name in (
        "position_rmse_m", "velocity_rmse_m_s", "attitude_rmse_deg",
        "position_nis_mean", "velocity_nis_mean", "navigation_nees_mean",
    ):
        values = aggregates[name]
        lines.append(
            f"| {name} | {values['mean']:.4f} | {values['p05']:.4f} "
            f"| {values['p95']:.4f} | {values['maximum']:.4f} |"
        )
    failures = summary["failures"]
    consistency_failures = summary["consistency_failures"]
    distribution_failures = summary["distribution_failures"]
    confidence_failures = statistical_confidence["failures"]
    lines.extend([
        "",
        f"Hard-envelope failures: **{len(failures)}**; distribution P95 failures: "
        f"**{len(distribution_failures)}**; aggregate consistency failures: "
        f"**{len(consistency_failures)}**; confidence-bound failures: "
        f"**{len(confidence_failures)}**; execution failures: **{len(execution_failures)}**.",
        "",
        f"The deterministic {BOOTSTRAP_CONFIDENCE:.0%} bootstrap bounds use "
        f"{BOOTSTRAP_RESAMPLES:,} resamples. The full confidence gate is applied only at "
        f"{FULL_STATISTICAL_GATE_TRIALS:,} or more trials. The exact 95% zero-failure upper "
        f"probability is `{statistical_confidence['zero_failure_probability_upper_95']}`. "
        "This baseline randomizes measurement noise, constant IMU bias, and "
        "monotonic timestamp jitter while exercising the fixed five-second aiding outage. "
        "Temperature drift and transport faults remain separate P0 work.",
    ])
    (args.out_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    if execution_failures:
        raise SystemExit(
            f"Monte Carlo execution failed for {len(execution_failures)} seed(s); rerun with --resume"
        )
    if failures:
        raise SystemExit(f"Monte Carlo hard envelope failed for {len(failures)} seed(s)")
    if distribution_failures:
        raise SystemExit(
            "Monte Carlo distribution gate failed: "
            + "; ".join(distribution_failures)
        )
    if consistency_failures:
        raise SystemExit(
            "Monte Carlo aggregate consistency gate failed: "
            + "; ".join(consistency_failures)
        )
    if confidence_failures:
        raise SystemExit(
            "Monte Carlo confidence gate failed: " + "; ".join(confidence_failures)
        )
    print(f"Monte Carlo baseline passed: {args.out_dir / 'report.md'}")


if __name__ == "__main__":
    main()
