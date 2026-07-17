#!/usr/bin/env python3
"""Run reproducible PC validation scenarios through the native C algorithms."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path


SCENARIOS = {
    "clean_motion": {"motion": "slow_sin", "anomaly": "none"},
    "mag_spike": {"motion": "slow_sin", "anomaly": "spike"},
    "mag_bias": {"motion": "yaw_spin", "anomaly": "bias"},
    "yaw_jump": {"motion": "yaw_jump", "anomaly": "none"},
}


def run(command: list[str], environment: dict[str, str] | None = None) -> None:
    subprocess.run(command, check=True, env=environment)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runner", type=Path, required=True, help="native aerakia_validation_runner")
    parser.add_argument("--out-dir", type=Path, default=Path("build/validation"))
    parser.add_argument("--scenarios", default=",".join(SCENARIOS), help="comma-separated scenario names")
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--rate", type=float, default=100.0)
    parser.add_argument("--seed", type=int, default=7)
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[1]
    generator = root / "simulation" / "tools" / "generate_synthetic_imu.py"
    analyzer = root / "validation" / "analyze_results.py"
    selected = [name.strip() for name in args.scenarios.split(",") if name.strip()]
    unknown = [name for name in selected if name not in SCENARIOS]
    if unknown:
        parser.error(f"unknown scenarios: {', '.join(unknown)}")

    matplotlib_config = (args.out_dir / ".matplotlib").resolve()
    matplotlib_config.mkdir(parents=True, exist_ok=True)
    plot_environment = os.environ.copy()
    plot_environment["MPLCONFIGDIR"] = str(matplotlib_config)

    summaries: list[dict[str, object]] = []
    for scenario_name in selected:
        scenario = SCENARIOS[scenario_name]
        scenario_dir = args.out_dir / scenario_name
        scenario_dir.mkdir(parents=True, exist_ok=True)
        input_csv = scenario_dir / "input.csv"
        results_csv = scenario_dir / "results.csv"

        run(
            [
                sys.executable,
                str(generator),
                "--out", str(input_csv),
                "--duration", str(args.duration),
                "--rate", str(args.rate),
                "--seed", str(args.seed),
                "--motion", str(scenario["motion"]),
                "--anomaly", str(scenario["anomaly"]),
            ]
        )
        run([str(args.runner), str(input_csv), str(results_csv)])
        run(
            [
                sys.executable,
                str(analyzer),
                str(results_csv),
                "--out-dir", str(scenario_dir),
                "--scenario", scenario_name,
            ],
            plot_environment,
        )
        summaries.append(json.loads((scenario_dir / "metrics.json").read_text(encoding="utf-8")))

    report_lines = [
        "# Aerakia validation suite",
        "",
        "| Scenario | Mahony standard | Mahony robust | ESKF |",
        "| --- | ---: | ---: | ---: |",
    ]
    for summary in summaries:
        algorithms = summary["algorithms"]
        report_lines.append(
            f"| [{summary['scenario']}]({summary['scenario']}/report.md) "
            f"| {algorithms['mahony_standard']['overall_attitude_rmse_deg']:.4f}° "
            f"| {algorithms['mahony_robust']['overall_attitude_rmse_deg']:.4f}° "
            f"| {algorithms['eskf']['overall_attitude_rmse_deg']:.4f}° |"
        )
    report_lines.extend(
        [
            "",
            "All values are wrapped attitude RMSE over deterministic synthetic inputs.",
            "Use real motion-capture or rate-table data before making reliability claims.",
        ]
    )
    args.out_dir.mkdir(parents=True, exist_ok=True)
    (args.out_dir / "report.md").write_text("\n".join(report_lines) + "\n", encoding="utf-8")
    (args.out_dir / "summary.json").write_text(
        json.dumps(summaries, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(f"Validation report: {args.out_dir / 'report.md'}")


if __name__ == "__main__":
    main()
