#!/usr/bin/env python3
"""Compute repeatable accuracy metrics and plots from validation-runner output."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


ALGORITHMS = ("mahony_standard", "mahony_robust", "eskf")
AXES = ("roll", "pitch", "yaw")


def wrapped_error_deg(estimate: np.ndarray, truth: np.ndarray) -> np.ndarray:
    return (estimate - truth + 180.0) % 360.0 - 180.0


def load_columns(path: Path) -> dict[str, np.ndarray]:
    columns: dict[str, list[float]] = {}
    with path.open("r", encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            for name, value in row.items():
                columns.setdefault(name, []).append(float(value))
    if not columns:
        raise ValueError(f"empty results file: {path}")
    return {name: np.asarray(values, dtype=np.float64) for name, values in columns.items()}


def metrics_for(columns: dict[str, np.ndarray], algorithm: str) -> dict[str, object]:
    axes: dict[str, dict[str, float]] = {}
    squared_errors: list[np.ndarray] = []
    for axis in AXES:
        truth = columns[f"truth_{axis}_deg"]
        estimate = columns[f"{algorithm}_{axis}_deg"]
        error = wrapped_error_deg(estimate, truth)
        absolute = np.abs(error)
        squared_errors.append(error * error)
        axes[axis] = {
            "rmse_deg": float(np.sqrt(np.mean(error * error))),
            "p95_abs_deg": float(np.percentile(absolute, 95)),
            "max_abs_deg": float(np.max(absolute)),
            "final_error_deg": float(error[-1]),
        }

    overall = np.sqrt(np.mean(np.column_stack(squared_errors)))
    result: dict[str, object] = {
        "overall_attitude_rmse_deg": float(overall),
        "axes": axes,
    }
    if algorithm == "mahony_robust":
        result["minimum_accelerometer_weight"] = float(np.min(columns["mahony_robust_acc_weight"]))
        result["mean_magnetometer_weight"] = float(np.mean(columns["mahony_robust_mag_weight"]))
    if algorithm == "eskf":
        result["magnetometer_acceptance_ratio"] = float(np.mean(columns["eskf_mag_accepted"]))
        position = np.column_stack(
            (
                columns["eskf_position_n_m"],
                columns["eskf_position_e_m"],
                columns["eskf_position_d_m"],
            )
        )
        result["final_position_norm_m"] = float(np.linalg.norm(position[-1]))
    return result


def create_plot(columns: dict[str, np.ndarray], output_path: Path, title: str) -> None:
    time_s = (columns["ts_us"] - columns["ts_us"][0]) * 1.0e-6
    figure, axes = plt.subplots(3, 2, figsize=(14, 10), sharex=True)
    colors = {
        "mahony_standard": "#8b8b8b",
        "mahony_robust": "#0072b2",
        "eskf": "#d55e00",
    }

    for row, axis_name in enumerate(AXES):
        truth = columns[f"truth_{axis_name}_deg"]
        axes[row, 0].plot(time_s, truth, color="black", linewidth=1.5, label="truth")
        for algorithm in ALGORITHMS:
            estimate = columns[f"{algorithm}_{axis_name}_deg"]
            axes[row, 0].plot(time_s, estimate, color=colors[algorithm], linewidth=1.0, label=algorithm)
            axes[row, 1].plot(
                time_s,
                wrapped_error_deg(estimate, truth),
                color=colors[algorithm],
                linewidth=1.0,
                label=algorithm,
            )
        axes[row, 0].set_ylabel(f"{axis_name} (deg)")
        axes[row, 1].set_ylabel(f"{axis_name} error (deg)")
        axes[row, 0].grid(alpha=0.25)
        axes[row, 1].grid(alpha=0.25)

    axes[0, 0].legend(ncol=2, fontsize=8)
    axes[0, 1].legend(ncol=2, fontsize=8)
    axes[2, 0].set_xlabel("time (s)")
    axes[2, 1].set_xlabel("time (s)")
    figure.suptitle(title)
    figure.tight_layout()
    figure.savefig(output_path, dpi=150)
    plt.close(figure)


def write_markdown(path: Path, scenario: str, metrics: dict[str, object], plot_name: str) -> None:
    lines = [
        f"# Validation report: {scenario}",
        "",
        "| Algorithm | Attitude RMSE | Roll RMSE | Pitch RMSE | Yaw RMSE |",
        "| --- | ---: | ---: | ---: | ---: |",
    ]
    algorithms = metrics["algorithms"]
    assert isinstance(algorithms, dict)
    for algorithm in ALGORITHMS:
        values = algorithms[algorithm]
        assert isinstance(values, dict)
        axes = values["axes"]
        assert isinstance(axes, dict)
        lines.append(
            f"| `{algorithm}` | {values['overall_attitude_rmse_deg']:.4f}° "
            f"| {axes['roll']['rmse_deg']:.4f}° | {axes['pitch']['rmse_deg']:.4f}° "
            f"| {axes['yaw']['rmse_deg']:.4f}° |"
        )
    lines.extend(
        [
            "",
            f"![Attitude estimates and wrapped errors]({plot_name})",
            "",
            "> Synthetic results verify deterministic behavior and fault response; they are not an airworthiness claim.",
        ]
    )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("results_csv", type=Path)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--scenario", default="validation")
    args = parser.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    columns = load_columns(args.results_csv)
    metrics = {
        "scenario": args.scenario,
        "samples": int(len(columns["seq"])),
        "duration_s": float((columns["ts_us"][-1] - columns["ts_us"][0]) * 1.0e-6),
        "algorithms": {algorithm: metrics_for(columns, algorithm) for algorithm in ALGORITHMS},
    }

    plot_path = args.out_dir / "attitude_comparison.png"
    create_plot(columns, plot_path, f"Aerakia validation — {args.scenario}")
    (args.out_dir / "metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    write_markdown(args.out_dir / "report.md", args.scenario, metrics, plot_path.name)
    print(json.dumps(metrics, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
