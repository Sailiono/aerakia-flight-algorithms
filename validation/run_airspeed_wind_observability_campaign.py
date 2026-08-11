#!/usr/bin/env python3
"""Run frozen multi-seed confirmation for the TAS/wind source-contract oracle.

This is deliberately a replication campaign, not a tuning loop and not a
17-state ESKF implementation.  It reuses the frozen v1 case rules for disjoint
synthetic seed windows, then commits only compact aggregates and a digest of
the individual records.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import subprocess
from collections import Counter, defaultdict
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Any

import run_airspeed_wind_observability as oracle


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CAMPAIGN_PROTOCOL = ROOT / "validation" / "airspeed_wind_observability_campaign_v1.json"


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def capture(command: list[str]) -> str | None:
    completed = subprocess.run(
        command,
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    return completed.stdout.strip() if completed.returncode == 0 else None


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    if not (0.0 <= fraction <= 1.0):
        raise ValueError("percentile fraction must be within [0, 1]")
    ordered = sorted(values)
    location = fraction * (len(ordered) - 1)
    low = int(math.floor(location))
    high = int(math.ceil(location))
    return ordered[low] + (ordered[high] - ordered[low]) * (location - low)


def load_campaign_protocol(path: Path = DEFAULT_CAMPAIGN_PROTOCOL) -> dict[str, Any]:
    campaign = json.loads(path.read_text(encoding="utf-8"))
    if campaign.get("schema_version") != 1:
        raise ValueError("unsupported airspeed/wind campaign schema")
    if campaign.get("status") != "frozen-confirmation-only":
        raise ValueError("campaign must remain frozen-confirmation-only")
    if not isinstance(campaign.get("scenario_names"), list) or not campaign["scenario_names"]:
        raise ValueError("campaign must declare one or more scenarios")
    seed_splits = campaign.get("confirmation_seed_splits")
    if not isinstance(seed_splits, dict) or len(seed_splits) < 2:
        raise ValueError("campaign needs at least two named confirmation seed splits")
    seen: set[int] = set()
    for name, seeds in seed_splits.items():
        if not isinstance(name, str) or not name or not isinstance(seeds, list) or not seeds:
            raise ValueError("every confirmation split needs a name and non-empty seed list")
        for seed in seeds:
            if not isinstance(seed, int):
                raise ValueError("confirmation seeds must be integers")
            if seed in seen:
                raise ValueError(f"confirmation seed {seed} appears in multiple splits")
            seen.add(seed)
    required = ("base_protocol_path", "base_protocol_file_sha256", "acceptance_rules", "retention")
    if any(name not in campaign for name in required):
        raise ValueError("campaign protocol is incomplete")
    return campaign


def resolve_base_protocol(campaign: dict[str, Any]) -> tuple[Path, dict[str, Any]]:
    relative = Path(str(campaign["base_protocol_path"]))
    if relative.is_absolute() or ".." in relative.parts:
        raise ValueError("base protocol must be a repository-relative path")
    path = ROOT / relative
    if not path.is_file():
        raise FileNotFoundError(f"base protocol does not exist: {path}")
    expected_hash = str(campaign["base_protocol_file_sha256"])
    actual_hash = file_sha256(path)
    if actual_hash != expected_hash:
        raise ValueError("base protocol SHA-256 differs from the frozen campaign contract")
    return path, oracle.load_protocol(path)


def selected_cases(campaign: dict[str, Any], base_protocol: dict[str, Any]) -> list[dict[str, Any]]:
    by_name = {str(case["name"]): case for case in base_protocol["scenario_matrix"]}
    requested = [str(name) for name in campaign["scenario_names"]]
    if len(set(requested)) != len(requested):
        raise ValueError("campaign scenario names must be unique")
    missing = [name for name in requested if name not in by_name]
    if missing:
        raise ValueError(f"campaign names absent from base protocol: {', '.join(missing)}")
    return [by_name[name] for name in requested]


def compact_record(split: str, seed: int, result: dict[str, object]) -> dict[str, object]:
    causal = result["causal_two_state_wind_oracle"]
    score = result["offline_truth_score"]
    known = result["known_wind_upper_bound"]
    assert isinstance(causal, dict) and isinstance(score, dict) and isinstance(known, dict)
    return {
        "split": split,
        "seed": seed,
        "scenario": result["name"],
        "role": result["role"],
        "passed": result["passed"],
        "errors": result["errors"],
        "input_observations": result["input_observations"],
        "accepted_events": result["accepted_events"],
        "rejected_events": result["rejected_events"],
        "terminal_status": causal["terminal_status"],
        "rejection_counts": causal["rejection_counts"],
        "terminal_wind_error_norm_m_s": score["terminal_wind_error_norm_m_s"],
        "terminal_wind_nees_2d_offline_truth_scored": score[
            "terminal_wind_nees_2d_offline_truth_scored"
        ],
        "known_wind_nis_mean": known["nis_mean"],
        "known_wind_jacobian_finite_difference_max_error": known[
            "maximum_jacobian_finite_difference_error"
        ],
    }


def run_campaign(
    campaign: dict[str, Any],
    base_protocol: dict[str, Any],
    *,
    jobs: int,
) -> dict[str, object]:
    if jobs <= 0:
        raise ValueError("jobs must be positive")
    cases = selected_cases(campaign, base_protocol)
    trials = [
        (split, seed, case)
        for split, seeds in campaign["confirmation_seed_splits"].items()
        for seed in seeds
        for case in cases
    ]

    def run_one(item: tuple[str, int, dict[str, Any]]) -> dict[str, object]:
        split, seed, case = item
        return compact_record(split, seed, oracle.evaluate_case(case, base_protocol, synthetic_seed=seed))

    with ThreadPoolExecutor(max_workers=jobs) as executor:
        records = list(executor.map(run_one, trials))
    records.sort(key=lambda item: (str(item["split"]), int(item["seed"]), str(item["scenario"])))
    return summarize_records(campaign, records)


def summarize_records(campaign: dict[str, Any], records: list[dict[str, object]]) -> dict[str, object]:
    if not records:
        raise ValueError("campaign cannot summarize zero records")
    by_scenario: dict[str, list[dict[str, object]]] = defaultdict(list)
    for record in records:
        by_scenario[str(record["scenario"])].append(record)
    scenarios: list[dict[str, object]] = []
    for name in campaign["scenario_names"]:
        grouped = by_scenario[str(name)]
        status_counts = Counter(str(record["terminal_status"]) for record in grouped)
        rejections: Counter[str] = Counter()
        for record in grouped:
            values = record["rejection_counts"]
            assert isinstance(values, dict)
            rejections.update({str(key): int(value) for key, value in values.items()})
        positive_errors = [
            float(record["terminal_wind_error_norm_m_s"])
            for record in grouped if record["role"] == "positive"
        ]
        positive_nis = [
            float(record["known_wind_nis_mean"])
            for record in grouped
            if record["role"] == "positive" and record["known_wind_nis_mean"] is not None
        ]
        scenarios.append({
            "name": name,
            "role": grouped[0]["role"],
            "replications": len(grouped),
            "passed_replications": sum(bool(record["passed"]) for record in grouped),
            "input_observations": sum(int(record["input_observations"]) for record in grouped),
            "accepted_events": sum(int(record["accepted_events"]) for record in grouped),
            "rejected_events": sum(int(record["rejected_events"]) for record in grouped),
            "terminal_status_counts": dict(sorted(status_counts.items())),
            "aggregate_rejection_counts": dict(sorted(rejections.items())),
            "positive_terminal_wind_error_norm_m_s": {
                "mean": None if not positive_errors else sum(positive_errors) / len(positive_errors),
                "p95": percentile(positive_errors, 0.95),
                "maximum": None if not positive_errors else max(positive_errors),
            },
            "positive_known_wind_nis_mean": {
                "mean": None if not positive_nis else sum(positive_nis) / len(positive_nis),
                "p05": percentile(positive_nis, 0.05),
                "p95": percentile(positive_nis, 0.95),
            },
        })
    failed = [record for record in records if not bool(record["passed"])]
    failure_categories: Counter[str] = Counter()
    for record in failed:
        terminal = str(record["terminal_status"])
        if str(record["role"]) == "negative" and terminal == "qualified":
            failure_categories["unsafe_false_qualification"] += 1
        elif terminal == "source_latched":
            failure_categories["latch_event_order_evidence_gap"] += 1
        elif terminal == "stale_no_fresh_gnss_tas":
            failure_categories["latch_completion_gap_but_not_qualified"] += 1
        else:
            failure_categories["other_frozen_rule_failure"] += 1
    record_digest = oracle.canonical_sha256(records)
    return {
        "schema_version": 1,
        "study_id": campaign["campaign_id"],
        "status": "passed" if not failed else "failed",
        "confirmation_seed_splits": campaign["confirmation_seed_splits"],
        "scenario_summaries": scenarios,
        "totals": {
            "replication_cases": len(records),
            "passed_replication_cases": len(records) - len(failed),
            "failed_replication_cases": len(failed),
            "input_observations": sum(int(record["input_observations"]) for record in records),
            "accepted_events": sum(int(record["accepted_events"]) for record in records),
            "rejected_events": sum(int(record["rejected_events"]) for record in records),
        },
        "all_record_canonical_sha256": record_digest,
        "failure_categories": dict(sorted(failure_categories.items())),
        "failed_case_records": failed,
        "limitations": [
            "This is a seed-replication confirmation of a synthetic source-contract oracle, not a train/tune/holdout physical air-data evaluation.",
            "The same frozen per-case rules are applied to every seed; no result is used to retune a threshold or source contract.",
            "No production ESKF source/state/API or FCOne controller policy is changed by this campaign.",
        ],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--campaign-protocol", type=Path, default=DEFAULT_CAMPAIGN_PROTOCOL)
    parser.add_argument(
        "--out", type=Path,
        default=ROOT / "validation" / "public" / "airspeed_wind_observability_campaign_v1.json",
    )
    parser.add_argument("--jobs", type=int, default=min(4, max(1, os.cpu_count() or 1)))
    args = parser.parse_args()
    campaign_path = args.campaign_protocol.resolve()
    campaign = load_campaign_protocol(campaign_path)
    base_protocol_path, base_protocol = resolve_base_protocol(campaign)
    result = run_campaign(campaign, base_protocol, jobs=args.jobs)
    result["campaign_protocol"] = {
        "path": (
            str(campaign_path.relative_to(ROOT)) if campaign_path.is_relative_to(ROOT) else str(campaign_path)
        ),
        "file_sha256": file_sha256(campaign_path),
        "semantic_sha256": oracle.canonical_sha256(campaign),
    }
    result["base_protocol"] = {
        "path": str(base_protocol_path.relative_to(ROOT)),
        "file_sha256": file_sha256(base_protocol_path),
        "semantic_sha256": oracle.canonical_sha256(base_protocol),
    }
    result["provenance"] = {
        "campaign_runner_sha256": file_sha256(Path(__file__)),
        "base_oracle_runner_sha256": file_sha256(
            ROOT / "validation" / "run_airspeed_wind_observability.py"
        ),
        "git_commit": capture(["git", "rev-parse", "HEAD"]),
        "git_status": capture(["git", "status", "--short"]),
        "jobs": args.jobs,
    }
    output = args.out.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
