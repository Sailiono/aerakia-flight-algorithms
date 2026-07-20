#!/usr/bin/env python3
"""Compare the optional float ESKF build against the reviewed double replay.

This is a host differential test for an STM32H7-oriented candidate profile.  It does not measure
target WCET, FPU ABI, cache/DMA interaction, stack high-water mark, or long-duration target
stability; those remain FCOne v2 evidence.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import subprocess
import sys
from pathlib import Path
from typing import Any


SCENARIOS = {
    "clean_motion": [],
    "navigation_outage": ["--cold-start"],
}


def run(command: list[str], root: Path) -> None:
    completed = subprocess.run(
        command,
        cwd=root,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    if completed.returncode != 0:
        raise RuntimeError(f"{' '.join(command)}\n{completed.stdout}")


def load_rows(path: Path) -> list[dict[str, float]]:
    with path.open("r", encoding="utf-8", newline="") as stream:
        rows = [
            {name: float(value) for name, value in row.items() if name is not None and value}
            for row in csv.DictReader(stream)
        ]
    if not rows:
        raise ValueError(f"empty results file: {path}")
    return rows


def percentile(values: list[float], percentile_value: float) -> float:
    ordered = sorted(values)
    if not ordered:
        raise ValueError("cannot compute a percentile of no values")
    index = (len(ordered) - 1) * percentile_value / 100.0
    lower = int(math.floor(index))
    upper = int(math.ceil(index))
    if lower == upper:
        return ordered[lower]
    fraction = index - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def quaternion_distance_deg(first: dict[str, float], second: dict[str, float]) -> float:
    axes = ("w", "x", "y", "z")
    first_q = [first[f"eskf_q_{axis}"] for axis in axes]
    second_q = [second[f"eskf_q_{axis}"] for axis in axes]
    first_norm = math.sqrt(sum(value * value for value in first_q))
    second_norm = math.sqrt(sum(value * value for value in second_q))
    if first_norm <= 0.0 or second_norm <= 0.0:
        raise ValueError("candidate emitted a non-normalizable quaternion")
    dot = sum(
        first_q[index] * second_q[index] / (first_norm * second_norm)
        for index in range(4)
    )
    return math.degrees(2.0 * math.acos(min(1.0, max(0.0, abs(dot)))))


def vector_distance(first: dict[str, float], second: dict[str, float], prefix: str, unit: str) -> float:
    components = ("n", "e", "d")
    if unit == "m":
        names = [f"{prefix}_{axis}_m" for axis in components]
    elif unit == "m_s":
        names = [f"{prefix}_{axis}_m_s" for axis in components]
    else:
        raise ValueError(f"unknown unit {unit}")
    return math.sqrt(sum((first[name] - second[name]) ** 2 for name in names))


def difference_summary(reference: list[dict[str, float]], candidate: list[dict[str, float]]) -> dict[str, Any]:
    if len(reference) != len(candidate):
        raise ValueError("double and float replay row counts differ")
    attitude: list[float] = []
    position: list[float] = []
    velocity: list[float] = []
    for reference_row, candidate_row in zip(reference, candidate, strict=True):
        if reference_row["seq"] != candidate_row["seq"] or reference_row["ts_us"] != candidate_row["ts_us"]:
            raise ValueError("double and float replay sequence/timestamp differs")
        attitude.append(quaternion_distance_deg(reference_row, candidate_row))
        position.append(vector_distance(reference_row, candidate_row, "eskf_position", "m"))
        velocity.append(vector_distance(reference_row, candidate_row, "eskf_velocity", "m_s"))

    def summarize(values: list[float]) -> dict[str, float]:
        return {
            "rmse": math.sqrt(sum(value * value for value in values) / len(values)),
            "p95": percentile(values, 95.0),
            "maximum": max(values),
        }

    return {
        "attitude_geodesic_deg": summarize(attitude),
        "position_m": summarize(position),
        "velocity_m_s": summarize(velocity),
        "float_healthy_ratio": sum(row["eskf_healthy"] > 0.5 for row in candidate) / len(candidate),
    }


def load_metrics(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def metric_delta(reference: dict[str, Any], candidate: dict[str, Any]) -> dict[str, float | None]:
    reference_navigation = reference.get("navigation") or {}
    candidate_navigation = candidate.get("navigation") or {}
    return {
        "attitude_rmse_deg": (
            float(candidate["algorithms"]["eskf"]["overall_attitude_rmse_deg"])
            - float(reference["algorithms"]["eskf"]["overall_attitude_rmse_deg"])
        ),
        "position_rmse_m": (
            float(candidate_navigation["position_rmse_m"])
            - float(reference_navigation["position_rmse_m"])
            if reference_navigation and candidate_navigation else None
        ),
        "velocity_rmse_m_s": (
            float(candidate_navigation["velocity_rmse_m_s"])
            - float(reference_navigation["velocity_rmse_m_s"])
            if reference_navigation and candidate_navigation else None
        ),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--double-runner", type=Path, required=True)
    parser.add_argument("--float-runner", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, default=Path("build/precision-comparison"))
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--rate", type=int, default=400)
    parser.add_argument("--seed", type=int, default=41)
    args = parser.parse_args()
    if args.duration <= 2.0 or args.rate < 100:
        parser.error("--duration must exceed two seconds and --rate must be at least 100 Hz")
    root = Path(__file__).resolve().parents[1]
    double_runner = args.double_runner.resolve()
    float_runner = args.float_runner.resolve()
    for label, runner in (("double", double_runner), ("float", float_runner)):
        if not runner.is_file():
            parser.error(f"{label} runner not found: {runner}")
    out_dir = args.out_dir.resolve()
    double_dir = out_dir / "double"
    float_dir = out_dir / "float"
    run(
        [
            sys.executable,
            str(root / "validation" / "run_suite.py"),
            "--runner", str(double_runner),
            "--out-dir", str(double_dir),
            "--scenarios", ",".join(SCENARIOS),
            "--duration", str(args.duration),
            "--rate", str(args.rate),
            "--seed", str(args.seed),
            "--mag-rate-hz", "100",
            "--rate-invariant-streams",
            "--no-plots",
        ],
        root,
    )

    records: dict[str, Any] = {}
    for scenario, runner_args in SCENARIOS.items():
        source_dir = double_dir / scenario
        destination_dir = float_dir / scenario
        destination_dir.mkdir(parents=True, exist_ok=True)
        input_csv = source_dir / "input.csv"
        float_results = destination_dir / "results.csv"
        run([str(float_runner), *runner_args, str(input_csv), str(float_results)], root)
        run(
            [
                sys.executable,
                str(root / "validation" / "analyze_results.py"),
                str(float_results),
                "--out-dir", str(destination_dir),
                "--scenario", scenario,
                "--no-plots",
            ],
            root,
        )
        reference_rows = load_rows(source_dir / "results.csv")
        candidate_rows = load_rows(float_results)
        reference_metrics = load_metrics(source_dir / "metrics.json")
        candidate_metrics = load_metrics(destination_dir / "metrics.json")
        records[scenario] = {
            "state_difference": difference_summary(reference_rows, candidate_rows),
            "accuracy_delta_float_minus_double": metric_delta(reference_metrics, candidate_metrics),
        }

    # These are differential-host gates only. They intentionally leave target timing and long-horizon
    # numerical qualification open, while stopping a build that silently changes a 20 s replay.
    checks: list[dict[str, Any]] = []
    for scenario, record in records.items():
        difference = record["state_difference"]
        deltas = record["accuracy_delta_float_minus_double"]
        checks.extend([
            {
                "scenario": scenario,
                "name": "float_output_is_healthy",
                "passed": difference["float_healthy_ratio"] == 1.0,
            },
            {
                "scenario": scenario,
                "name": "maximum_state_divergence_is_bounded",
                "passed": (
                    difference["attitude_geodesic_deg"]["maximum"] <= 0.10
                    and difference["position_m"]["maximum"] <= 0.05
                    and difference["velocity_m_s"]["maximum"] <= 0.02
                ),
            },
            {
                "scenario": scenario,
                "name": "accuracy_delta_is_bounded",
                "passed": (
                    abs(deltas["attitude_rmse_deg"]) <= 0.05
                    and (deltas["position_rmse_m"] is None
                         or abs(deltas["position_rmse_m"]) <= 0.02)
                    and (deltas["velocity_rmse_m_s"] is None
                         or abs(deltas["velocity_rmse_m_s"]) <= 0.01)
                ),
            },
        ])
    passed = all(check["passed"] for check in checks)
    document = {
        "schema_version": 1,
        "scope": (
            "Host differential comparison between the reviewed double core and the optional "
            "single-precision candidate on identical generated inputs."
        ),
        "duration_s": args.duration,
        "rate_hz": args.rate,
        "seed": args.seed,
        "records": records,
        "checks": checks,
        "passed": passed,
        "not_evidence": [
            "STM32H7 execution time or worst-case scheduling",
            "target FPU ABI, DMA/cache effects, Flash/RAM, or stack high-water mark",
            "target long-duration numerical stability",
        ],
    }
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "summary.json").write_text(
        json.dumps(document, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    lines = [
        "# Float candidate versus double reference",
        "",
        "Both runners consume byte-identical generated inputs. This host comparison is not a "
        "target-MCU performance qualification.",
        "",
        "| Scenario | Max attitude difference | Max position difference | Max velocity difference | Float health | Result |",
        "| --- | ---: | ---: | ---: | ---: | --- |",
    ]
    for scenario, record in records.items():
        difference = record["state_difference"]
        scenario_passed = all(
            check["passed"] for check in checks if check["scenario"] == scenario
        )
        lines.append(
            f"| `{scenario}` | {difference['attitude_geodesic_deg']['maximum']:.6g} deg | "
            f"{difference['position_m']['maximum']:.6g} m | "
            f"{difference['velocity_m_s']['maximum']:.6g} m/s | "
            f"{difference['float_healthy_ratio']:.3f} | "
            f"{'PASS' if scenario_passed else 'FAIL'} |"
        )
    lines.extend([
        "",
        f"Overall host differential gate: **{'PASS' if passed else 'FAIL'}**.",
        "",
        "The public default remains double precision. A float build is an FCOne v2 candidate only "
        "after target-specific WCET, stack, memory, FPU and long-run evidence is collected.",
    ])
    (out_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    if not passed:
        raise SystemExit("float candidate differential gate failed")
    print(f"Float precision comparison passed -> {out_dir}")


if __name__ == "__main__":
    main()
