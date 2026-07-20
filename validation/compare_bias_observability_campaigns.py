#!/usr/bin/env python3
"""Compare two frozen bias-observability campaigns trial by trial.

This module deliberately keeps estimator execution separate from acceptance.  It
only compares already-produced campaign summaries and their per-trial metrics.
The comparison therefore cannot silently regenerate inputs or tune thresholds
after seeing candidate results.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import re
from collections import defaultdict
from pathlib import Path
from statistics import mean
from typing import Any, Iterable


TrialKey = tuple[str, str, str, int]


def file_sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def nested(value: object, *keys: str) -> object | None:
    current = value
    for key in keys:
        if not isinstance(current, dict) or key not in current:
            return None
        current = current[key]
    return current


def finite_float(value: object) -> float | None:
    if value is None or isinstance(value, bool):
        return None
    try:
        converted = float(value)
    except (TypeError, ValueError):
        return None
    return converted if math.isfinite(converted) else None


def trial_key(trial: dict[str, object]) -> TrialKey:
    return (
        str(trial["split"]),
        str(trial["trajectory_id"]),
        str(trial["bias_vector_id"]),
        int(trial["seed"]),
    )


def key_label(key: TrialKey) -> str:
    return "/".join((key[0], key[1], key[2], f"seed-{key[3]:05d}"))


def index_trials(campaign: dict[str, object]) -> tuple[dict[TrialKey, dict[str, object]], list[str]]:
    failures: list[str] = []
    indexed: dict[TrialKey, dict[str, object]] = {}
    trials = campaign.get("trials")
    if not isinstance(trials, list):
        return {}, ["campaign trials must be a list"]
    for trial in trials:
        if not isinstance(trial, dict):
            failures.append("campaign contains a non-object trial")
            continue
        try:
            key = trial_key(trial)
        except (KeyError, TypeError, ValueError) as error:
            failures.append(f"invalid trial key: {error}")
            continue
        if key in indexed:
            failures.append(f"duplicate trial key: {key_label(key)}")
        indexed[key] = trial
    return indexed, failures


def trial_directory(summary_path: Path, trial: dict[str, object]) -> Path:
    for name in ("trial_dir", "directory", "path"):
        declared = trial.get(name)
        if isinstance(declared, str) and declared:
            path = Path(declared)
            return path if path.is_absolute() else summary_path.parent / path
    key = trial_key(trial)
    return (
        summary_path.parent
        / "trials"
        / key[0]
        / key[1]
        / key[2]
        / f"seed-{key[3]:05d}"
    )


def load_trial_metrics(summary_path: Path, trial: dict[str, object]) -> dict[str, object]:
    embedded = trial.get("metrics")
    if isinstance(embedded, dict):
        return embedded
    metrics_path = trial_directory(summary_path, trial) / "metrics.json"
    if not metrics_path.is_file():
        return {}
    loaded = json.loads(metrics_path.read_text(encoding="utf-8"))
    return loaded if isinstance(loaded, dict) else {}


def declared_input_sha(trial: dict[str, object]) -> str | None:
    candidates = (
        trial.get("input_sha256"),
        trial.get("input_csv_sha256"),
        nested(trial, "provenance", "input_sha256"),
        nested(trial, "provenance", "input_csv_sha256"),
        nested(trial, "artifacts", "input_sha256"),
        nested(trial, "artifacts", "input_csv_sha256"),
    )
    for candidate in candidates:
        if isinstance(candidate, str) and candidate:
            return candidate
    return None


def input_sha(summary_path: Path, trial: dict[str, object]) -> str | None:
    evidence, _ = input_sha_evidence(summary_path, trial)
    return evidence


def input_sha_evidence(
    summary_path: Path, trial: dict[str, object]
) -> tuple[str | None, list[str]]:
    """Return the trusted input digest and any provenance failures.

    A declared digest is not accepted blindly: when a compacted trial still
    contains ``input.csv``, its bytes must hash to the declared value.  This
    prevents a stale or hand-edited summary from making unequal inputs appear
    paired.  Digest strings are required to be canonical SHA-256 values.
    """
    failures: list[str] = []
    declared = declared_input_sha(trial)
    if declared is not None and not re.fullmatch(r"[0-9a-fA-F]{64}", declared):
        failures.append("declared input SHA-256 is not a 64-hex digest")
        declared = None
    input_path = trial_directory(summary_path, trial) / "input.csv"
    local = file_sha256(input_path) if input_path.is_file() else None
    if declared is not None and local is not None and declared.lower() != local:
        failures.append("declared input SHA-256 does not match input.csv bytes")
    return (declared or local), failures


def analyzer_sha(campaign: dict[str, object], summary_path: Path) -> str | None:
    provenance = campaign.get("provenance")
    if isinstance(provenance, dict):
        for name in ("analyzer_sha256", "analysis_sha256"):
            value = provenance.get(name)
            if isinstance(value, str) and value:
                return value
    trials = campaign.get("trials")
    if not isinstance(trials, list):
        return None
    for trial in trials:
        if not isinstance(trial, dict):
            continue
        command = nested(trial, "commands", "analyzer")
        if not isinstance(command, list) or len(command) < 2:
            continue
        path = Path(str(command[1]))
        if not path.is_absolute():
            path = summary_path.parent / path
        if path.is_file():
            return file_sha256(path)
    return None


def extract_metrics(
    summary_path: Path,
    trial: dict[str, object],
    metric_specs: dict[str, dict[str, object]],
) -> tuple[dict[str, float | None], dict[str, object]]:
    full = load_trial_metrics(summary_path, trial)
    horizontal = trial.get("horizontal_bias")
    general = trial.get("general")
    if not isinstance(horizontal, dict):
        horizontal = {}
    if not isinstance(general, dict):
        general = {}

    raw: dict[str, object | None] = {
        "settling_time_s": horizontal.get("settling_time_s"),
        "final_error_x_m_s2": horizontal.get("final_error_x_m_s2"),
        "final_error_y_m_s2": horizontal.get("final_error_y_m_s2"),
        "terminal_horizontal_error_p95_m_s2": horizontal.get(
            "terminal_horizontal_error_p95_m_s2"
        ),
        "terminal_horizontal_error_max_m_s2": horizontal.get(
            "terminal_horizontal_error_max_m_s2"
        ),
        "horizontal_error_rmse_m_s2": horizontal.get("horizontal_error_rmse_m_s2"),
        "post_alignment_attitude_rmse_deg": general.get(
            "post_alignment_attitude_rmse_deg"
        ),
        "position_rmse_m": general.get("position_rmse_m"),
        "velocity_rmse_m_s": general.get("velocity_rmse_m_s"),
        "position_nis_mean": nested(full, "eskf_consistency", "position_nis", "mean"),
        "velocity_nis_mean": nested(full, "eskf_consistency", "velocity_nis", "mean"),
        "navigation_nees_mean": (
            nested(full, "eskf_consistency", "navigation_nees", "mean")
            if full
            else general.get("navigation_nees_mean")
        ),
        "tilt_accel_bias_joint_nees_mean": nested(
            full, "bias_estimation", "tilt_accel_bias_joint_nees", "mean"
        ),
        "healthy_ratio": general.get("healthy_ratio"),
        "navigation_recoveries": general.get("navigation_recoveries"),
        "joint_nees_invalid_covariance_samples": nested(
            full,
            "bias_estimation",
            "tilt_accel_bias_joint_nees",
            "invalid_covariance_samples",
        ),
    }
    extracted = {name: finite_float(raw.get(name)) for name in metric_specs}
    input_digest, input_failures = input_sha_evidence(summary_path, trial)
    metadata = {
        "passed": not bool(trial.get("gate_failures")),
        "gate_failures": list(trial.get("gate_failures", [])),
        "right_censored": bool(horizontal.get("right_censored")),
        "acceleration_bias_m_s2": trial.get("acceleration_bias_m_s2"),
        "input_sha256": input_digest,
        "input_integrity_failures": input_failures,
    }
    return extracted, metadata


def metric_score(value: float | None, spec: dict[str, object]) -> float | None:
    if value is None:
        return None
    direction = spec.get("direction")
    if direction == "lower":
        return value
    if direction == "higher":
        return -value
    if direction == "consistency":
        expected = finite_float(spec.get("expected_mean"))
        return None if expected is None else abs(value - expected)
    raise ValueError(f"unsupported metric direction {direction!r}")


def pair_delta(
    baseline: float | None,
    candidate: float | None,
    spec: dict[str, object],
) -> tuple[float | None, float | None]:
    raw_delta = None if baseline is None or candidate is None else candidate - baseline
    baseline_score = metric_score(baseline, spec)
    candidate_score = metric_score(candidate, spec)
    score_delta = (
        None
        if baseline_score is None or candidate_score is None
        else candidate_score - baseline_score
    )
    return raw_delta, score_delta


def censoring_transition(baseline: bool, candidate: bool) -> str:
    return {
        (False, False): "settled_to_settled",
        (False, True): "settled_to_censored",
        (True, False): "censored_to_settled",
        (True, True): "censored_to_censored",
    }[(baseline, candidate)]


def pass_transition(baseline: bool, candidate: bool) -> str:
    return {
        (False, False): "fail_to_fail",
        (False, True): "fail_to_pass",
        (True, False): "pass_to_fail",
        (True, True): "pass_to_pass",
    }[(baseline, candidate)]


def mean_or_none(values: Iterable[float | None]) -> float | None:
    finite = [value for value in values if value is not None and math.isfinite(value)]
    return mean(finite) if finite else None


def summarize_group(
    rows: list[dict[str, object]], metric_specs: dict[str, dict[str, object]]
) -> dict[str, object]:
    result: dict[str, object] = {
        "trials": len(rows),
        "baseline_passed": sum(bool(row["baseline_passed"]) for row in rows),
        "candidate_passed": sum(bool(row["candidate_passed"]) for row in rows),
        "pass_to_fail": sum(row["pass_transition"] == "pass_to_fail" for row in rows),
        "censoring_transitions": {
            name: sum(row["censoring_transition"] == name for row in rows)
            for name in (
                "settled_to_settled",
                "settled_to_censored",
                "censored_to_settled",
                "censored_to_censored",
            )
        },
        "metrics": {},
    }
    metrics = result["metrics"]
    assert isinstance(metrics, dict)
    for name, spec in metric_specs.items():
        baseline_mean = mean_or_none(
            nested(row, "metrics", name, "baseline") for row in rows
        )
        candidate_mean = mean_or_none(
            nested(row, "metrics", name, "candidate") for row in rows
        )
        raw_delta, score_delta = pair_delta(baseline_mean, candidate_mean, spec)
        metrics[name] = {
            "baseline_mean": baseline_mean,
            "candidate_mean": candidate_mean,
            "candidate_minus_baseline": raw_delta,
            "regression_score_delta": score_delta,
            "paired_sample_count": sum(
                nested(row, "metrics", name, "baseline") is not None
                and nested(row, "metrics", name, "candidate") is not None
                for row in rows
            ),
        }
    return result


def grouped(
    rows: list[dict[str, object]],
    fields: tuple[str, ...],
    metric_specs: dict[str, dict[str, object]],
) -> dict[str, object]:
    groups: dict[tuple[object, ...], list[dict[str, object]]] = defaultdict(list)
    for row in rows:
        groups[tuple(row[field] for field in fields)].append(row)
    # Keep the ungrouped aggregate addressable as ``all``.  This is important
    # for policy checks and also makes the emitted JSON self-describing.
    if not fields:
        return {"all": summarize_group(rows, metric_specs)}
    return {
        "/".join(str(part) for part in key): summarize_group(group, metric_specs)
        for key, group in sorted(groups.items(), key=lambda item: tuple(map(str, item[0])))
    }


def mirror_analysis(
    rows: list[dict[str, object]], policy: dict[str, object]
) -> list[dict[str, object]]:
    metric_specs = policy["metrics"]
    assert isinstance(metric_specs, dict)
    pairs = policy.get("mirror_pairs", [])
    result: list[dict[str, object]] = []
    for pair in pairs if isinstance(pairs, list) else []:
        if not isinstance(pair, dict):
            continue
        left = str(pair["left"])
        right = str(pair["right"])
        for split, trajectory in sorted(
            {(str(row["split"]), str(row["trajectory_id"])) for row in rows}
        ):
            left_rows = [
                row
                for row in rows
                if row["split"] == split
                and row["trajectory_id"] == trajectory
                and row["bias_vector_id"] == left
            ]
            right_rows = [
                row
                for row in rows
                if row["split"] == split
                and row["trajectory_id"] == trajectory
                and row["bias_vector_id"] == right
            ]
            if not left_rows or not right_rows:
                continue
            for metric_name in policy.get("mirror_metrics", []):
                spec = metric_specs.get(metric_name)
                if not isinstance(spec, dict):
                    continue
                baseline_left = mean_or_none(
                    nested(row, "metrics", metric_name, "baseline") for row in left_rows
                )
                baseline_right = mean_or_none(
                    nested(row, "metrics", metric_name, "baseline") for row in right_rows
                )
                candidate_left = mean_or_none(
                    nested(row, "metrics", metric_name, "candidate") for row in left_rows
                )
                candidate_right = mean_or_none(
                    nested(row, "metrics", metric_name, "candidate") for row in right_rows
                )
                baseline_asymmetry = (
                    None
                    if baseline_left is None or baseline_right is None
                    else abs(baseline_left - baseline_right)
                )
                candidate_asymmetry = (
                    None
                    if candidate_left is None or candidate_right is None
                    else abs(candidate_left - candidate_right)
                )
                result.append({
                    "split": split,
                    "trajectory_id": trajectory,
                    "pair": str(pair.get("id", f"{left}_vs_{right}")),
                    "left": left,
                    "right": right,
                    "metric": metric_name,
                    "baseline_asymmetry": baseline_asymmetry,
                    "candidate_asymmetry": candidate_asymmetry,
                    "candidate_minus_baseline_asymmetry": (
                        None
                        if baseline_asymmetry is None or candidate_asymmetry is None
                        else candidate_asymmetry - baseline_asymmetry
                    ),
                })
    return result


def provenance_failures(
    baseline: dict[str, object],
    candidate: dict[str, object],
    baseline_path: Path,
    candidate_path: Path,
) -> list[str]:
    failures: list[str] = []
    baseline_protocol = nested(baseline, "protocol", "semantic_sha256")
    candidate_protocol = nested(candidate, "protocol", "semantic_sha256")
    if not baseline_protocol or baseline_protocol != candidate_protocol:
        failures.append("protocol semantic SHA-256 mismatch or missing")

    for label, campaign in (("baseline", baseline), ("candidate", candidate)):
        commit = nested(campaign, "provenance", "git_commit")
        if not isinstance(commit, str) or not re.fullmatch(r"[0-9a-fA-F]{40}", commit):
            failures.append(f"{label} source commit is not an immutable 40-hex SHA")
        dirty = nested(campaign, "provenance", "git_status_short")
        if not isinstance(dirty, list) or dirty:
            failures.append(f"{label} source tree was dirty or cleanliness was not recorded")
        if campaign.get("execution_status") != "passed":
            failures.append(f"{label} execution status is not passed")
        execution_failures = campaign.get("execution_failures")
        if not isinstance(execution_failures, list) or execution_failures:
            failures.append(f"{label} has missing, timed-out, or unrecorded trial executions")

    for name in ("generator_sha256", "campaign_runner_sha256"):
        baseline_hash = nested(baseline, "provenance", name)
        candidate_hash = nested(candidate, "provenance", name)
        if not baseline_hash or baseline_hash != candidate_hash:
            failures.append(f"{name} mismatch or missing")
    baseline_analyzer = analyzer_sha(baseline, baseline_path)
    candidate_analyzer = analyzer_sha(candidate, candidate_path)
    if not baseline_analyzer or baseline_analyzer != candidate_analyzer:
        failures.append("analyzer SHA-256 mismatch or missing")
    if baseline.get("mode") != candidate.get("mode"):
        failures.append("campaign modes differ")
    if baseline.get("truth_boundary") != candidate.get("truth_boundary"):
        failures.append("truth-boundary declarations differ")
    return failures


def material_regression(score_delta: float | None, spec: dict[str, object]) -> bool:
    if score_delta is None:
        return False
    tolerance = finite_float(spec.get("material_regression_tolerance"))
    return score_delta > (0.0 if tolerance is None else tolerance)


def acceptance_failures(
    rows: list[dict[str, object]],
    groups: dict[str, dict[str, object]],
    mirrors: list[dict[str, object]],
    policy: dict[str, object],
) -> list[str]:
    failures: list[str] = []
    metric_specs = policy["metrics"]
    assert isinstance(metric_specs, dict)

    for row in rows:
        if row["bias_vector_id"] == "zero" and row["pass_transition"] == "pass_to_fail":
            failures.append(f"zero-bias pass-to-fail: {row['key']}")
        if row["bias_vector_id"] != "zero":
            continue
        for name in policy.get("zero_bias_regression_metrics", []):
            spec = metric_specs.get(name)
            score_delta = nested(row, "metrics", str(name), "regression_score_delta")
            if isinstance(spec, dict) and material_regression(
                finite_float(score_delta), spec
            ):
                failures.append(f"zero-bias material regression {name}: {row['key']}")

    primary = str(policy["primary_improvement_metric"])
    minimum = finite_float(policy.get("minimum_primary_mean_improvement")) or 0.0
    split_groups = groups["split"]
    for split in policy.get("required_splits", []):
        stats = split_groups.get(str(split))
        improvement = nested(stats, "metrics", primary, "regression_score_delta")
        if finite_float(improvement) is None or float(improvement) > -minimum:
            failures.append(f"{split} did not improve {primary} by at least {minimum:g}")

    vector_groups = groups["split_vector"]
    required_fraction = finite_float(policy.get("minimum_nonzero_group_improvement_fraction"))
    required_fraction = 1.0 if required_fraction is None else required_fraction
    maximum_regressions = int(policy.get("maximum_nonzero_regressing_groups", 0))
    for split in policy.get("required_splits", []):
        relevant = [
            stats
            for key, stats in vector_groups.items()
            if key.startswith(f"{split}/") and not key.endswith("/zero")
        ]
        deltas = [
            finite_float(nested(stats, "metrics", primary, "regression_score_delta"))
            for stats in relevant
        ]
        if any(delta is None for delta in deltas) or not deltas:
            failures.append(f"{split} nonzero signed-vector evidence is incomplete")
            continue
        improved = sum(delta < -minimum for delta in deltas if delta is not None)
        regressed = sum(delta > minimum for delta in deltas if delta is not None)
        if improved / len(deltas) < required_fraction:
            failures.append(
                f"{split} improved only {improved}/{len(deltas)} nonzero vector groups"
            )
        if regressed > maximum_regressions:
            failures.append(
                f"{split} regressed in {regressed} nonzero vector groups, maximum "
                f"is {maximum_regressions}"
            )

    for name, stats in groups["split_trajectory_vector"].items():
        if stats["baseline_passed"] == stats["trials"] and stats["candidate_passed"] < stats["trials"]:
            failures.append(f"previously passing trajectory/vector group failed: {name}")

    for name in policy.get("global_regression_metrics", []):
        spec = metric_specs.get(name)
        score_delta = nested(groups["overall"], "all", "metrics", str(name), "regression_score_delta")
        if isinstance(spec, dict) and material_regression(finite_float(score_delta), spec):
            failures.append(f"material global regression: {name}")

    mirror_tolerance = finite_float(policy.get("mirror_asymmetry_worsening_tolerance")) or 0.0
    for item in mirrors:
        worsening = finite_float(item["candidate_minus_baseline_asymmetry"])
        if worsening is None:
            failures.append(
                f"missing mirror evidence: {item['split']}/{item['trajectory_id']}/"
                f"{item['pair']}/{item['metric']}"
            )
        elif worsening > mirror_tolerance:
            failures.append(
                f"mirror asymmetry worsened by {worsening:.6g}: "
                f"{item['split']}/{item['trajectory_id']}/{item['pair']}/{item['metric']}"
            )

    required_metrics = policy.get("required_candidate_metrics", [])
    for row in rows:
        for name in required_metrics:
            if nested(row, "metrics", str(name), "candidate") is None:
                failures.append(f"required candidate metric {name} missing: {row['key']}")
    return failures


def compare_campaigns(
    baseline: dict[str, object],
    candidate: dict[str, object],
    policy: dict[str, object],
    *,
    baseline_path: Path,
    candidate_path: Path,
) -> dict[str, object]:
    failures = provenance_failures(
        baseline, candidate, baseline_path, candidate_path
    )
    baseline_trials, baseline_index_failures = index_trials(baseline)
    candidate_trials, candidate_index_failures = index_trials(candidate)
    failures.extend(f"baseline: {failure}" for failure in baseline_index_failures)
    failures.extend(f"candidate: {failure}" for failure in candidate_index_failures)

    required_splits = set(map(str, policy.get("required_splits", [])))
    expected_count = int(policy.get("expected_trial_count", 576))
    for label, indexed in (("baseline", baseline_trials), ("candidate", candidate_trials)):
        holdout = [key for key in indexed if key[0] == "holdout"]
        if holdout:
            failures.append(f"{label} contains {len(holdout)} forbidden holdout trials")
        unexpected_splits = sorted({key[0] for key in indexed} - required_splits)
        if unexpected_splits:
            failures.append(f"{label} contains unexpected splits: {unexpected_splits}")
        if len(indexed) != expected_count:
            failures.append(f"{label} has {len(indexed)} trials, expected {expected_count}")

    baseline_keys = set(baseline_trials)
    candidate_keys = set(candidate_trials)
    if baseline_keys != candidate_keys:
        missing = sorted(baseline_keys - candidate_keys)
        extra = sorted(candidate_keys - baseline_keys)
        failures.append(
            f"trial-key mismatch: missing candidate={len(missing)}, extra candidate={len(extra)}"
        )

    metric_specs = policy.get("metrics")
    if not isinstance(metric_specs, dict):
        raise ValueError("policy.metrics must be an object")
    typed_specs = {
        str(name): spec
        for name, spec in metric_specs.items()
        if isinstance(spec, dict)
    }
    rows: list[dict[str, object]] = []
    for key in sorted(baseline_keys & candidate_keys):
        baseline_metric, baseline_meta = extract_metrics(
            baseline_path, baseline_trials[key], typed_specs
        )
        candidate_metric, candidate_meta = extract_metrics(
            candidate_path, candidate_trials[key], typed_specs
        )
        baseline_input = baseline_meta["input_sha256"]
        candidate_input = candidate_meta["input_sha256"]
        for label, metadata in (("baseline", baseline_meta), ("candidate", candidate_meta)):
            for failure in metadata.get("input_integrity_failures", []):
                failures.append(f"{label} {failure}: {key_label(key)}")
        if baseline_input is None or candidate_input is None:
            failures.append(f"input SHA-256 missing: {key_label(key)}")
        elif baseline_input != candidate_input:
            failures.append(f"input SHA-256 mismatch: {key_label(key)}")

        metric_comparison: dict[str, object] = {}
        for name, spec in typed_specs.items():
            baseline_value = baseline_metric[name]
            candidate_value = candidate_metric[name]
            raw_delta, score_delta = pair_delta(baseline_value, candidate_value, spec)
            metric_comparison[name] = {
                "baseline": baseline_value,
                "candidate": candidate_value,
                "candidate_minus_baseline": raw_delta,
                "regression_score_delta": score_delta,
            }
        settled_pair = (
            not bool(baseline_meta["right_censored"])
            and not bool(candidate_meta["right_censored"])
        )
        if not settled_pair:
            metric_comparison["settling_time_s"]["candidate_minus_baseline"] = None
            metric_comparison["settling_time_s"]["regression_score_delta"] = None
        rows.append({
            "key": key_label(key),
            "split": key[0],
            "trajectory_id": key[1],
            "bias_vector_id": key[2],
            "seed": key[3],
            "acceleration_bias_m_s2": candidate_meta["acceleration_bias_m_s2"],
            "input_sha256": candidate_input,
            "baseline_passed": baseline_meta["passed"],
            "candidate_passed": candidate_meta["passed"],
            "pass_transition": pass_transition(
                bool(baseline_meta["passed"]), bool(candidate_meta["passed"])
            ),
            "baseline_right_censored": baseline_meta["right_censored"],
            "candidate_right_censored": candidate_meta["right_censored"],
            "censoring_transition": censoring_transition(
                bool(baseline_meta["right_censored"]),
                bool(candidate_meta["right_censored"]),
            ),
            "metrics": metric_comparison,
        })

    groups = {
        "overall": grouped(rows, (), typed_specs),
        "split": grouped(rows, ("split",), typed_specs),
        "split_trajectory": grouped(rows, ("split", "trajectory_id"), typed_specs),
        "split_vector": grouped(rows, ("split", "bias_vector_id"), typed_specs),
        "split_trajectory_vector": grouped(
            rows, ("split", "trajectory_id", "bias_vector_id"), typed_specs
        ),
    }
    mirrors = mirror_analysis(rows, policy)
    failures.extend(acceptance_failures(rows, groups, mirrors, policy))
    failures = list(dict.fromkeys(failures))
    return {
        "schema_version": 1,
        "status": "passed" if not failures else "failed",
        "comparison_scope": "paired_train_tune_only",
        "holdout_opened": False,
        "policy": policy,
        "baseline": {
            "summary_path": str(baseline_path),
            "git_commit": nested(baseline, "provenance", "git_commit"),
            "runner_sha256": nested(baseline, "provenance", "runner_sha256"),
        },
        "candidate": {
            "summary_path": str(candidate_path),
            "git_commit": nested(candidate, "provenance", "git_commit"),
            "runner_sha256": nested(candidate, "provenance", "runner_sha256"),
        },
        "paired_trial_count": len(rows),
        "failures": failures,
        "groups": groups,
        "mirror_asymmetry": mirrors,
        "paired_trials": rows,
    }


def write_paired_csv(result: dict[str, object], path: Path) -> None:
    rows = result["paired_trials"]
    policy = result["policy"]
    assert isinstance(rows, list)
    assert isinstance(policy, dict)
    metric_names = list(policy["metrics"])
    fieldnames = [
        "key", "split", "trajectory_id", "bias_vector_id", "seed",
        "pass_transition", "censoring_transition", "input_sha256",
    ]
    for name in metric_names:
        fieldnames.extend((f"baseline_{name}", f"candidate_{name}", f"delta_{name}"))
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            assert isinstance(row, dict)
            output = {name: row.get(name) for name in fieldnames[:8]}
            for name in metric_names:
                output[f"baseline_{name}"] = nested(row, "metrics", name, "baseline")
                output[f"candidate_{name}"] = nested(row, "metrics", name, "candidate")
                output[f"delta_{name}"] = nested(
                    row, "metrics", name, "candidate_minus_baseline"
                )
            writer.writerow(output)


def write_report(result: dict[str, object], path: Path) -> None:
    failures = result["failures"]
    groups = result["groups"]
    policy = result["policy"]
    assert isinstance(failures, list)
    assert isinstance(groups, dict)
    assert isinstance(policy, dict)
    primary = str(policy["primary_improvement_metric"])
    lines = [
        "# Bias-observability paired A/B comparison",
        "",
        f"Status: **{result['status']}**; paired train/tune trials: "
        f"**{result['paired_trial_count']}**; holdout opened: **no**.",
        "",
        "Candidate-minus-baseline is the raw delta. For NIS/NEES, acceptance uses "
        "the change in absolute distance from the declared theoretical mean, not a "
        "naive preference for smaller values.",
        "",
        "## Acceptance failures",
        "",
    ]
    lines.extend([f"- {failure}" for failure in failures] or ["- None."])
    lines.extend([
        "",
        "## Primary metric by split",
        "",
        f"Primary metric: `{primary}`.",
        "",
        "| Split | Trials | Baseline mean | Candidate mean | Delta | Pass → fail |",
        "| --- | ---: | ---: | ---: | ---: | ---: |",
    ])
    split_groups = groups["split"]
    assert isinstance(split_groups, dict)
    for split, stats in split_groups.items():
        metric = nested(stats, "metrics", primary)
        assert isinstance(metric, dict)
        values = [metric.get(name) for name in (
            "baseline_mean", "candidate_mean", "candidate_minus_baseline"
        )]
        rendered = ["n/a" if value is None else f"{float(value):.6g}" for value in values]
        lines.append(
            f"| {split} | {stats['trials']} | {rendered[0]} | {rendered[1]} | "
            f"{rendered[2]} | {stats['pass_to_fail']} |"
        )
    lines.extend([
        "",
        "## Evidence boundaries",
        "",
        "- The comparison requires exact trial-key pairing and identical input SHA-256.",
        "- Dirty or non-immutable source revisions are rejected.",
        "- Settling-time deltas are computed only for settled-to-settled pairs; "
        "censoring transitions remain categorical.",
        "- The v1 holdout is intentionally excluded because prior smoke execution "
        "contaminated it. This report is not a blind release-holdout result.",
        "",
    ])
    path.write_text("\n".join(lines), encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument(
        "--policy",
        type=Path,
        default=Path(__file__).with_name("bias_observability_ab_policy_v1.json"),
    )
    parser.add_argument("--out-dir", type=Path, required=True)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    baseline_path = args.baseline.resolve()
    candidate_path = args.candidate.resolve()
    policy_path = args.policy.resolve()
    baseline = json.loads(baseline_path.read_text(encoding="utf-8"))
    candidate = json.loads(candidate_path.read_text(encoding="utf-8"))
    policy = json.loads(policy_path.read_text(encoding="utf-8"))
    result = compare_campaigns(
        baseline,
        candidate,
        policy,
        baseline_path=baseline_path,
        candidate_path=candidate_path,
    )
    args.out_dir.mkdir(parents=True, exist_ok=True)
    (args.out_dir / "summary.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    write_paired_csv(result, args.out_dir / "paired_trials.csv")
    write_report(result, args.out_dir / "report.md")
    if result["status"] != "passed":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
