#!/usr/bin/env python3
"""Fail CI when deterministic validation metrics exceed reviewed limits."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("summary_json", type=Path)
    parser.add_argument("--thresholds", type=Path, required=True)
    args = parser.parse_args()

    summaries = json.loads(args.summary_json.read_text(encoding="utf-8"))
    thresholds = json.loads(args.thresholds.read_text(encoding="utf-8"))
    by_scenario = {summary["scenario"]: summary for summary in summaries}
    failures: list[str] = []

    for scenario, algorithm_limits in thresholds.items():
        if scenario not in by_scenario:
            failures.append(f"missing scenario: {scenario}")
            continue
        algorithms = by_scenario[scenario]["algorithms"]
        for algorithm, limit in algorithm_limits.items():
            value = float(algorithms[algorithm]["overall_attitude_rmse_deg"])
            if value > float(limit):
                failures.append(
                    f"{scenario}/{algorithm}: {value:.6f} deg exceeds {float(limit):.6f} deg"
                )
            else:
                print(f"PASS {scenario}/{algorithm}: {value:.6f} <= {float(limit):.6f} deg")

    # The fault-tolerant configuration must outperform the un-gated baseline.
    for scenario in ("mag_spike", "mag_bias"):
        if scenario not in by_scenario:
            continue
        algorithms = by_scenario[scenario]["algorithms"]
        robust = float(algorithms["mahony_robust"]["overall_attitude_rmse_deg"])
        standard = float(algorithms["mahony_standard"]["overall_attitude_rmse_deg"])
        if robust >= standard:
            failures.append(
                f"{scenario}: robust Mahony {robust:.6f} deg did not beat standard {standard:.6f} deg"
            )
        else:
            print(f"PASS {scenario}: robust Mahony {robust:.6f} < standard {standard:.6f} deg")

    if failures:
        raise SystemExit("\n".join(failures))


if __name__ == "__main__":
    main()
