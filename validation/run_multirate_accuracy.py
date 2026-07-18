#!/usr/bin/env python3
"""Gate deterministic and statistical estimator behavior across IMU rates."""

from __future__ import annotations

import argparse
import json
import math
import statistics
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
from typing import Any


DETERMINISTIC_METRICS = {
    "clean_attitude_rmse_deg": (
        "clean_motion", "algorithms.eskf.overall_attitude_rmse_deg", 0.001, 0.05
    ),
    "outage_attitude_rmse_deg": (
        "navigation_outage", "cold_start_alignment.post_alignment_attitude_rmse_deg", 0.025, 0.05
    ),
    "outage_position_rmse_m": (
        "navigation_outage", "navigation.position_rmse_m", 0.015, 0.05
    ),
    "outage_velocity_rmse_m_s": (
        "navigation_outage", "navigation.velocity_rmse_m_s", 0.007, 0.05
    ),
}

STATISTICAL_METRICS = {
    # Clean-motion discretization is already gated deterministically above. Its
    # short, heading-dominated stochastic score has high between-seed variance,
    # so a small CI seed count is not a defensible distribution-equivalence
    # test. The navigation-outage track supplies the multi-seed attitude,
    # navigation, NIS, and NEES invariance gates.
    "outage_attitude_rmse_deg": (
        "navigation_outage", "cold_start_alignment.post_alignment_attitude_rmse_deg", 0.01, 0.05
    ),
    "outage_position_rmse_m": (
        "navigation_outage", "navigation.position_rmse_m", 0.01, 0.05
    ),
    "outage_velocity_rmse_m_s": (
        "navigation_outage", "navigation.velocity_rmse_m_s", 0.005, 0.05
    ),
    "outage_position_nis_mean": (
        "navigation_outage", "eskf_consistency.position_nis.mean", 0.10, 0.05
    ),
    "outage_velocity_nis_mean": (
        "navigation_outage", "eskf_consistency.velocity_nis.mean", 0.10, 0.05
    ),
    "outage_navigation_nees_mean": (
        "navigation_outage", "eskf_consistency.navigation_nees.mean", 0.30, 0.05
    ),
}

ONE_WAY_LIMITS = {
    "outage_attitude_rmse_deg": (None, 1.5),
    "outage_position_rmse_m": (None, 1.0),
    "outage_velocity_rmse_m_s": (None, 0.4),
    "outage_position_nis_mean": (1.0, 5.0),
    "outage_velocity_nis_mean": (1.0, 5.0),
    "outage_navigation_nees_mean": (1.0, 12.0),
}


def nested_value(document: dict[str, Any], path: str) -> float:
    value: Any = document
    for component in path.split("."):
        value = value[component]
    return float(value)


def load_metrics(rate_dir: Path, definitions: dict[str, tuple[str, str, float, float]]) -> dict[str, float]:
    summaries = {
        item["scenario"]: item
        for item in json.loads((rate_dir / "summary.json").read_text(encoding="utf-8"))
    }
    return {
        name: nested_value(summaries[scenario], path)
        for name, (scenario, path, _absolute, _relative) in definitions.items()
    }


def run_suite(command: list[str], root: Path) -> None:
    completed = subprocess.run(
        command, cwd=root, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT
    )
    if completed.returncode != 0:
        raise RuntimeError(completed.stdout)


def suite_command(
    suite: Path,
    runner: Path,
    rate_dir: Path,
    duration_s: float,
    rate_hz: int,
    seed: int,
    stochastic: bool,
) -> list[str]:
    command = [
        sys.executable, str(suite), "--runner", str(runner),
        "--out-dir", str(rate_dir),
        "--scenarios", "navigation_outage" if stochastic else "clean_motion,navigation_outage",
        "--duration", str(duration_s), "--rate", str(rate_hz),
        "--seed", str(seed), "--mag-rate-hz", "100",
        "--rate-invariant-streams",
    ]
    if stochastic:
        # Preserve continuous white-noise density while the sample rate changes.
        scale = math.sqrt(rate_hz / 100.0)
        command.extend([
            "--accel-noise-m-s2", str(0.02 * scale),
            "--gyro-noise-deg-s", str(0.05 * scale),
            "--mag-noise-ut", "0.20",
            "--gps-position-noise-m", "0.5",
            "--gps-velocity-noise-m-s", "0.1",
        ])
    else:
        command.extend([
            "--accel-noise-m-s2", "0",
            "--gyro-noise-deg-s", "0",
            "--mag-noise-ut", "0",
            "--gps-position-noise-m", "0",
            "--gps-velocity-noise-m-s", "0",
        ])
    return command


def spread_checks(
    rates: list[int],
    metrics: dict[str, dict[str, float]],
    definitions: dict[str, tuple[str, str, float, float]],
) -> list[dict[str, Any]]:
    checks: list[dict[str, Any]] = []
    for name, (_scenario, _path, absolute_tolerance, relative_tolerance) in definitions.items():
        values = [metrics[str(rate)][name] for rate in rates]
        median = statistics.median(values)
        spread = max(values) - min(values)
        limit = max(absolute_tolerance, relative_tolerance * abs(median))
        checks.append({
            "metric": name,
            "values_by_rate": {str(rate): metrics[str(rate)][name] for rate in rates},
            "median": median,
            "spread": spread,
            "allowed_spread": limit,
            "passed": spread <= limit,
        })
    return checks


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runner", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, default=Path("build/multirate-accuracy"))
    parser.add_argument("--rates", default="100,200,400,1000")
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--seeds", type=int, default=32)
    parser.add_argument("--jobs", type=int, default=4)
    parser.add_argument(
        "--resume", action="store_true",
        help="reuse complete per-rate/per-seed summaries in the output directory",
    )
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[1]
    suite = root / "validation" / "run_suite.py"
    runner = args.runner.resolve()
    out_dir = args.out_dir.resolve()
    rates = [int(item) for item in args.rates.split(",") if item.strip()]
    if rates != sorted(set(rates)) or any(rate < 100 for rate in rates):
        parser.error("rates must be unique increasing integers of at least 100 Hz")
    if args.seeds < 2 or args.jobs < 1:
        parser.error("--seeds must be at least 2 and --jobs must be positive")
    if any(rate < 100 for rate in rates):
        parser.error("rates below the fixed 100 Hz magnetometer stream are unsupported")

    deterministic: dict[str, dict[str, float]] = {}
    for rate in rates:
        rate_dir = out_dir / "deterministic" / f"{rate}hz"
        if not (args.resume and (rate_dir / "summary.json").is_file()):
            run_suite(
                suite_command(suite, runner, rate_dir, args.duration, rate, 0, False), root
            )
        deterministic[str(rate)] = load_metrics(rate_dir, DETERMINISTIC_METRICS)

    tasks: list[tuple[int, int, Path, list[str]]] = []
    raw_stochastic: dict[str, list[dict[str, float]]] = {
        str(rate): [] for rate in rates
    }
    for rate in rates:
        for seed in range(args.seeds):
            rate_dir = out_dir / "stochastic" / f"{rate}hz" / f"seed-{seed:04d}"
            if args.resume and (rate_dir / "summary.json").is_file():
                raw_stochastic.setdefault(str(rate), []).append(
                    load_metrics(rate_dir, STATISTICAL_METRICS)
                )
            else:
                tasks.append((
                    rate, seed, rate_dir,
                    suite_command(suite, runner, rate_dir, args.duration, rate, seed, True),
                ))
    for rate in rates:
        raw_stochastic.setdefault(str(rate), [])
    with ThreadPoolExecutor(max_workers=args.jobs) as executor:
        futures = {
            executor.submit(run_suite, command, root): (rate, seed, rate_dir)
            for rate, seed, rate_dir, command in tasks
        }
        for future in as_completed(futures):
            rate, _seed, rate_dir = futures[future]
            future.result()
            raw_stochastic[str(rate)].append(load_metrics(rate_dir, STATISTICAL_METRICS))

    stochastic_aggregates: dict[str, dict[str, float]] = {}
    for rate in rates:
        trials = raw_stochastic[str(rate)]
        stochastic_aggregates[str(rate)] = {
            name: statistics.mean(trial[name] for trial in trials)
            for name in STATISTICAL_METRICS
        }

    deterministic_checks = spread_checks(
        rates, deterministic, DETERMINISTIC_METRICS
    )
    statistical_checks = spread_checks(
        rates, stochastic_aggregates, STATISTICAL_METRICS
    )
    one_way_checks: list[dict[str, Any]] = []
    for rate in rates:
        for metric, value in stochastic_aggregates[str(rate)].items():
            minimum, maximum = ONE_WAY_LIMITS[metric]
            passed = (minimum is None or value >= minimum) and (
                maximum is None or value <= maximum
            )
            one_way_checks.append({
                "rate_hz": rate,
                "metric": metric,
                "value": value,
                "minimum": minimum,
                "maximum": maximum,
                "passed": passed,
            })

    passed = all(
        item["passed"]
        for item in deterministic_checks + statistical_checks + one_way_checks
    )
    document = {
        "schema_version": 3,
        "rates_hz": rates,
        "duration_s": args.duration,
        "stochastic_seeds": args.seeds,
        "noise_model": (
            "fixed 100 Hz magnetometer/GNSS streams; IMU per-sample noise scales "
            "with sqrt(rate) to preserve continuous white-noise density"
        ),
        "deterministic_metrics": deterministic,
        "stochastic_aggregate": "mean across the fixed independent seed set",
        "stochastic_mean_metrics": stochastic_aggregates,
        "deterministic_checks": deterministic_checks,
        "statistical_checks": statistical_checks,
        "one_way_checks": one_way_checks,
        "passed": passed,
    }
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "summary.json").write_text(
        json.dumps(document, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    lines = [
        "# Multi-rate accuracy invariance", "",
        f"Rates: {', '.join(str(rate) for rate in rates)} Hz. Statistical gates use "
        f"{args.seeds} fixed independent seeds and report their mean.", "",
        "## Deterministic integration", "",
        "| Metric | Spread | Allowed | Result |", "| --- | ---: | ---: | --- |",
    ]
    for item in deterministic_checks:
        lines.append(
            f"| `{item['metric']}` | {item['spread']:.6g} | "
            f"{item['allowed_spread']:.6g} | {'PASS' if item['passed'] else 'FAIL'} |"
        )
    lines.extend([
        "", "## Stochastic mean", "",
        "| Metric | Spread | Allowed | Result |", "| --- | ---: | ---: | --- |",
    ])
    for item in statistical_checks:
        lines.append(
            f"| `{item['metric']}` | {item['spread']:.6g} | "
            f"{item['allowed_spread']:.6g} | {'PASS' if item['passed'] else 'FAIL'} |"
        )
    lines.extend(["", f"Overall: **{'PASS' if passed else 'FAIL'}**."])
    (out_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    if not passed:
        failed = [
            item for item in deterministic_checks + statistical_checks + one_way_checks
            if not item["passed"]
        ]
        raise SystemExit(
            "multi-rate accuracy gate failed: "
            + ", ".join(f"{item.get('rate_hz', 'spread')}:{item['metric']}" for item in failed)
        )
    print(f"Multi-rate accuracy gate passed at {rates} Hz -> {out_dir}")


if __name__ == "__main__":
    main()
