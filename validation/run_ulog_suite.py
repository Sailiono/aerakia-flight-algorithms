#!/usr/bin/env python3
"""Convert, replay, analyze, and summarize a manifest of private PX4 ULogs."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
CONVERTER = ROOT / "simulation" / "tools" / "convert_ulog_to_replay.py"
ANALYZER = ROOT / "validation" / "analyze_results.py"


def run(command: list[str]) -> None:
    print("+", " ".join(command), flush=True)
    subprocess.run(command, check=True)


def load_manifest(path: Path) -> list[dict[str, Any]]:
    data = json.loads(path.read_text(encoding="utf-8"))
    scenarios = data.get("scenarios") if isinstance(data, dict) else data
    if not isinstance(scenarios, list) or not scenarios:
        raise ValueError("manifest must contain a non-empty scenarios list")
    aliases: set[str] = set()
    result: list[dict[str, Any]] = []
    for entry in scenarios:
        if not isinstance(entry, dict) or not isinstance(entry.get("alias"), str):
            raise ValueError("each scenario requires a string alias")
        alias = entry["alias"]
        if not alias.replace("_", "").replace("-", "").isalnum() or alias in aliases:
            raise ValueError(f"invalid or duplicate alias: {alias}")
        source = Path(entry.get("path", ""))
        if not source.is_absolute():
            source = (path.parent / source).resolve()
        if not source.is_file():
            raise FileNotFoundError(source)
        aliases.add(alias)
        result.append(
            {"alias": alias, "path": source, "assume_stationary": bool(entry.get("assume_stationary"))}
        )
    return result


def percentage(value: Any) -> str:
    return "—" if value is None else f"{100.0 * float(value):.1f}%"


def summarize(results: list[dict[str, Any]], output_dir: Path) -> None:
    rows = []
    for result in results:
        metrics = result["metrics"]
        algorithms = metrics["algorithms"]
        eskf = algorithms["eskf"]
        robust = algorithms["mahony_robust"]
        navigation = metrics.get("navigation")
        integrity = metrics["eskf_integrity"]
        reset_aware = metrics.get("eskf_reset_aware_yaw", {})
        rows.append(
            {
                "alias": result["alias"],
                "samples": metrics["samples"],
                "duration_s": metrics["duration_s"],
                "mahony_robust_tilt_rmse_deg": robust["tilt_rmse_deg"],
                "mahony_robust_attitude_rmse_deg": robust["overall_attitude_rmse_deg"],
                "eskf_tilt_rmse_deg": eskf["tilt_rmse_deg"],
                "eskf_attitude_rmse_deg": eskf["overall_attitude_rmse_deg"],
                "eskf_yaw_rmse_deg": eskf["axes"]["yaw"]["rmse_deg"],
                "eskf_reset_aware_yaw_rmse_deg": reset_aware.get("rmse_deg"),
                "position_rmse_m": navigation.get("position_rmse_m") if navigation else None,
                "gps_position_acceptance_ratio": (
                    navigation.get("position_acceptance_ratio") if navigation else None
                ),
                "magnetometer_outer_gate_pass_ratio": integrity["magnetometer_outer_gate_pass_ratio"],
                "magnetometer_innovation_acceptance_ratio": (
                    integrity["magnetometer_innovation_acceptance_ratio"]
                ),
                "navigation_recoveries": integrity["navigation_recoveries"],
                "zero_velocity_updates": integrity["zero_velocity_updates"],
                "reference_reset_events": metrics["reference_resets"]["events"],
            }
        )
    (output_dir / "summary.json").write_text(
        json.dumps({"scenarios": rows}, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    lines = [
        "# Aerakia private ULog validation summary", "",
        "| Scenario | Samples | Robust tilt/full | ESKF tilt/full | ESKF yaw raw/reset-aware | Position | GPS accepted | Recovery/ZUPT |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in rows:
        position = "—" if row["position_rmse_m"] is None else f"{row['position_rmse_m']:.3f} m"
        reset_yaw = "—" if row["eskf_reset_aware_yaw_rmse_deg"] is None else f"{row['eskf_reset_aware_yaw_rmse_deg']:.3f}°"
        lines.append(
            f"| `{row['alias']}` | {row['samples']} | "
            f"{row['mahony_robust_tilt_rmse_deg']:.3f}° / {row['mahony_robust_attitude_rmse_deg']:.3f}° | "
            f"{row['eskf_tilt_rmse_deg']:.3f}° / {row['eskf_attitude_rmse_deg']:.3f}° | "
            f"{row['eskf_yaw_rmse_deg']:.3f}° / {reset_yaw} | {position} | "
            f"{percentage(row['gps_position_acceptance_ratio'])} | "
            f"{row['navigation_recoveries']} / {row['zero_velocity_updates']} |"
        )
    lines.extend(
        [
            "", "Notes:", "",
            "- Raw ULogs, absolute coordinates, and vehicle identifiers remain outside the public repository.",
            "- PX4 attitude/local-position values are engineering references, not independent ground truth.",
            "- Static scenarios use an explicit manifest assertion; the converter never infers stationarity silently.",
            "- Reset-aware yaw is a per-reference-segment diagnostic and can differ materially from globally aligned yaw.",
            "- A position result without GPS updates is ZUPT-aided inertial drift against the PX4 reference.",
        ]
    )
    (output_dir / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest", type=Path)
    parser.add_argument("--runner", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    args = parser.parse_args()
    scenarios = load_manifest(args.manifest.resolve())
    if not args.runner.is_file():
        parser.error(f"runner does not exist: {args.runner}")
    args.out_dir.mkdir(parents=True, exist_ok=True)

    results: list[dict[str, Any]] = []
    for scenario in scenarios:
        scenario_dir = args.out_dir / scenario["alias"]
        scenario_dir.mkdir(parents=True, exist_ok=True)
        replay = scenario_dir / "replay.csv"
        runner_results = scenario_dir / "results.csv"
        command = [
            sys.executable, str(CONVERTER), str(scenario["path"]), "--out", str(replay),
            "--metadata", str(scenario_dir / "source.json"),
        ]
        if scenario["assume_stationary"]:
            command.append("--assume-stationary")
        run(command)
        run([str(args.runner), str(replay), str(runner_results)])
        run(
            [
                sys.executable, str(ANALYZER), str(runner_results), "--out-dir", str(scenario_dir),
                "--scenario", scenario["alias"], "--reference-kind", "px4_estimate",
            ]
        )
        metrics = json.loads((scenario_dir / "metrics.json").read_text(encoding="utf-8"))
        results.append({"alias": scenario["alias"], "metrics": metrics})
    summarize(results, args.out_dir)
    print(f"Wrote suite summary to {args.out_dir / 'summary.md'}")


if __name__ == "__main__":
    main()
