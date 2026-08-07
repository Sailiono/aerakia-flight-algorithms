#!/usr/bin/env python3
"""Fail-closed merger for resumable barometer-outage campaign shards."""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
from typing import Any

from run_baro_outage_ab import aggregate, strict_json_value


ROOT = Path(__file__).resolve().parents[1]


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_summary(path: Path) -> tuple[dict[str, Any], str]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"cannot read {path}: {error}") from error
    required = ("schema_version", "status", "faults", "outages_s", "rate_hz", "provenance", "trials")
    if any(key not in value for key in required):
        raise ValueError(f"{path} is not a complete barometer campaign summary")
    if value["schema_version"] != 5 or value["status"] != "completed":
        raise ValueError(f"{path} has unsupported schema or incomplete status")
    if not isinstance(value["trials"], list) or not value["trials"]:
        raise ValueError(f"{path} has no trials")
    return value, sha256(path)


def restore_metric_nans(trials: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Restore JSON's null representation of undefined per-arm scalar metrics.

    The one-shot runner computes aggregates before strict JSON conversion, where
    undefined NIS values are NaN. Shard summaries must encode them as null, so
    a resumed merge restores NaN only in its private aggregation copy.
    """

    restored = copy.deepcopy(trials)
    for trial in restored:
        for arm in ("imu_only", "imu_baro_raw", "imu_baro_supervised", "imu_baro_shadow_failover"):
            values = trial.get(arm)
            if not isinstance(values, dict):
                continue
            for key, value in values.items():
                # ``strict_json_value`` maps every non-finite top-level arm
                # metric to null.  Examples include unavailable NIS percentiles
                # and the detection delay when no switch took place.  Those are
                # intentionally NaN to the aggregation code, not missing JSON
                # fields. Nested metadata remains untouched.
                if value is None:
                    values[key] = math.nan
    return restored


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, action="append", default=[], help="shard summary.json")
    parser.add_argument(
        "--input-dir", type=Path, action="append", default=[],
        help="recursively discover completed shard summary.json files",
    )
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--expected-seeds", type=int, default=None)
    parser.add_argument(
        "--expected-faults", default=None,
        help="comma-separated requested fault matrix; required with --expected-seeds",
    )
    parser.add_argument(
        "--expected-outages", default=None,
        help="comma-separated requested outage durations; required with --expected-seeds",
    )
    args = parser.parse_args()
    if args.expected_seeds is not None and args.expected_seeds <= 0:
        parser.error("--expected-seeds must be positive")
    if args.expected_seeds is not None and (args.expected_faults is None or args.expected_outages is None):
        parser.error("--expected-seeds requires --expected-faults and --expected-outages")

    expected_faults = None
    expected_outages = None
    if args.expected_faults is not None:
        expected_faults = [value.strip() for value in args.expected_faults.split(",") if value.strip()]
        if not expected_faults or len(expected_faults) != len(set(expected_faults)):
            parser.error("--expected-faults must be a non-empty list with no duplicates")
    if args.expected_outages is not None:
        try:
            expected_outages = [float(value) for value in args.expected_outages.split(",") if value.strip()]
        except ValueError as error:
            parser.error(f"invalid --expected-outages: {error}")
        if (not expected_outages or any(value <= 0.0 for value in expected_outages)
                or len(expected_outages) != len(set(expected_outages))):
            parser.error("--expected-outages must contain positive values with no duplicates")

    input_paths = list(args.input)
    for directory in args.input_dir:
        if not directory.is_dir():
            parser.error(f"--input-dir is not a directory: {directory}")
        input_paths.extend(sorted(directory.rglob("summary.json")))
    if not input_paths:
        parser.error("provide at least one --input or --input-dir")
    if len({path.resolve() for path in input_paths}) != len(input_paths):
        parser.error("the same shard was supplied more than once")
    loaded = [(path.resolve(), *load_summary(path.resolve())) for path in input_paths]
    first = loaded[0][1]
    # Every shard deliberately contains only one fault and one outage group, so
    # its local matrix definition must not be compared to another shard's.
    consistency_keys = ("schema_version", "scope", "rate_hz")
    provenance_keys = (
        "runner_sha256", "script_sha256", "generator_sha256", "barometer_supervisor_sha256",
    )
    trials: list[dict[str, Any]] = []
    seen: set[tuple[str, float, int]] = set()
    sources: list[dict[str, Any]] = []
    for path, summary, digest in loaded:
        for key in consistency_keys:
            if summary.get(key) != first.get(key):
                raise ValueError(f"{path}: {key} differs from the first shard")
        for key in provenance_keys:
            if summary["provenance"].get(key) != first["provenance"].get(key):
                raise ValueError(f"{path}: provenance {key} differs from the first shard")
        for trial in summary["trials"]:
            key = (str(trial["fault"]), float(trial["outage_duration_s"]), int(trial["seed"]))
            if key in seen:
                raise ValueError(f"duplicate trial {key} found in {path}")
            seen.add(key)
            trials.append(trial)
        sources.append({"path": str(path), "sha256": digest, "trials": len(summary["trials"])})

    observed_faults = sorted({str(trial["fault"]) for trial in trials})
    observed_outages = sorted({float(trial["outage_duration_s"]) for trial in trials})
    expected: set[tuple[str, float, int]] | None = None
    if args.expected_seeds is not None:
        expected = {
            (fault, float(outage), seed)
            for fault in expected_faults or []
            for outage in expected_outages or []
            for seed in range(args.expected_seeds)
        }
        missing = sorted(expected - seen)
        extra = sorted(seen - expected)
        if missing or extra:
            raise ValueError(
                f"campaign completeness failure: missing={len(missing)}, extra={len(extra)}; "
                f"first missing={missing[:3]}, first extra={extra[:3]}"
            )

    trials.sort(key=lambda value: (str(value["fault"]), float(value["outage_duration_s"]), int(value["seed"])))
    output = {
        "schema_version": 1,
        "status": "completed",
        "scope": first["scope"],
        "outages_s": expected_outages if expected_outages is not None else observed_outages,
        "faults": expected_faults if expected_faults is not None else observed_faults,
        "seeds": args.expected_seeds,
        "rate_hz": first["rate_hz"],
        "trial_count": len(trials),
        "aggregate": aggregate(restore_metric_nans(trials)),
        "trials": trials,
        "provenance": {
            "merged_by": "validation/merge_baro_outage_campaign.py",
            "source_shards": sources,
            "shared_runner_sha256": first["provenance"]["runner_sha256"],
            "shared_campaign_script_sha256": first["provenance"]["script_sha256"],
            "shared_generator_sha256": first["provenance"]["generator_sha256"],
            "shared_barometer_supervisor_sha256": first["provenance"]["barometer_supervisor_sha256"],
            "completeness_checked": expected is not None,
        },
        "limitations": first.get("limitations", []),
    }
    out_dir = args.out_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "summary.json").write_text(
        json.dumps(strict_json_value(output), indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    rows = [
        "# Merged four-arm barometer outage campaign", "",
        f"Merged **{len(trials)}** exact, non-duplicated trials from **{len(sources)}** shards.",
        "Completeness was checked against the requested Cartesian matrix.", "",
        "| Fault | Outage | IMU-only RMSE | Raw baro RMSE | Supervised RMSE | Shadow failover RMSE |",
        "| --- | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in output["aggregate"]:
        metric = row["outage_vertical_position_rmse_m"]
        rows.append(
            f"| {row['fault']} | {row['outage_duration_s']:g} s | "
            f"{metric['imu_only_mean']:.4f} m | {metric['imu_baro_raw_mean']:.4f} m | "
            f"{metric['imu_baro_supervised_mean']:.4f} m | "
            f"{metric['imu_baro_shadow_failover_mean']:.4f} m |"
        )
    rows.extend(["", "This is a synthetic diagnostic campaign, not a flight-promotion gate."])
    (out_dir / "report.md").write_text("\n".join(rows) + "\n", encoding="utf-8")
    print(f"Merged barometer campaign: {out_dir / 'report.md'}")


if __name__ == "__main__":
    main()
