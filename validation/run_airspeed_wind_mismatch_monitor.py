#!/usr/bin/env python3
"""Validation-only v2 TAS residual-pattern monitor study.

This runner is deliberately outside the production ESKF.  It follows a
review-rejected v1 prototype whose nominal control never formed a full window.
The v2 protocol adds explicit post-qualification coverage, short-window time
semantics, one-off impulse controls, and a clean development/holdout split.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import multiprocessing as multiprocessing
import os
import re
import subprocess
from collections import Counter, defaultdict, deque
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

import run_airspeed_wind_observability as base


ROOT = Path(__file__).resolve().parents[1]
RUNNER_PATH = ROOT / "validation" / "run_airspeed_wind_mismatch_monitor.py"
DEFAULT_PROTOCOL = ROOT / "validation" / "airspeed_wind_mismatch_monitor_protocol_v2.json"


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


@dataclass(frozen=True)
class MonitorSample:
    """One causal residual; source truth and injection labels are absent."""

    arrival_timestamp_us: int
    source_timestamp_us: int
    raw_nis: float
    clipped_nis: float


@dataclass
class CumulativeNisMonitor:
    """Short, terminal, causal residual-pattern screen.

    A high individual NIS may be a transient.  A latch consequently needs both
    a clipped cumulative score and multiple independently high contributions.
    The monitor is intentionally not a physical source-quality model.
    """

    window_observations: int
    maximum_window_elapsed_us: int
    maximum_inter_observation_gap_us: int
    per_observation_nis_cap: float
    contribution_nis_floor: float
    minimum_contributing_observations: int
    cumulative_nis_threshold: float
    values: deque[MonitorSample] | None = None
    last_arrival_timestamp_us: int | None = None
    latched: bool = False
    first_latch_timestamp_us: int | None = None
    latch_window: tuple[MonitorSample, ...] | None = None
    eligible_observations: int = 0
    completed_windows: int = 0
    maximum_window_nis: float = 0.0
    maximum_window_contributing_observations: int = 0
    reset_reasons: Counter[str] | None = None
    time_span_trim_count: int = 0

    def __post_init__(self) -> None:
        if self.window_observations < 2:
            raise ValueError("NIS window must contain at least two observations")
        if self.maximum_window_elapsed_us <= 0 or self.maximum_inter_observation_gap_us <= 0:
            raise ValueError("monitor time bounds must be positive")
        if self.maximum_inter_observation_gap_us > self.maximum_window_elapsed_us:
            raise ValueError("monitor gap bound cannot exceed complete-window span")
        for name, value in (
            ("per-observation NIS cap", self.per_observation_nis_cap),
            ("contribution NIS floor", self.contribution_nis_floor),
            ("cumulative NIS threshold", self.cumulative_nis_threshold),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be positive and finite")
        if self.contribution_nis_floor > self.per_observation_nis_cap:
            raise ValueError("contribution NIS floor cannot exceed per-observation cap")
        if not 2 <= self.minimum_contributing_observations <= self.window_observations:
            raise ValueError("monitor requires two or more contributing observations within its window")
        self.values = deque(maxlen=self.window_observations)
        self.reset_reasons = Counter()

    def reset(self, reason: str) -> None:
        """Clear an unfinished window without changing a terminal latch."""

        if self.latched:
            return
        assert self.values is not None
        self.values.clear()
        self.reset_reasons[reason] += 1

    def observe(
        self,
        arrival_timestamp_us: int,
        nis: float,
        *,
        source_timestamp_us: int | None = None,
    ) -> bool:
        """Consume one causal residual and return terminal latch state."""

        source_timestamp = arrival_timestamp_us if source_timestamp_us is None else source_timestamp_us
        if (
            arrival_timestamp_us < 0
            or source_timestamp < 0
            or source_timestamp > arrival_timestamp_us
            or not math.isfinite(nis)
            or nis < 0.0
        ):
            raise ValueError("monitor observation requires non-negative finite timestamp and NIS")
        if self.latched:
            return True
        assert self.values is not None
        if self.last_arrival_timestamp_us is not None:
            delta_us = arrival_timestamp_us - self.last_arrival_timestamp_us
            if delta_us <= 0:
                self.reset("non_monotonic_arrival")
            elif delta_us > self.maximum_inter_observation_gap_us:
                self.reset("inter_observation_gap")
        self.last_arrival_timestamp_us = arrival_timestamp_us
        self.eligible_observations += 1
        self.values.append(
            MonitorSample(
                arrival_timestamp_us=arrival_timestamp_us,
                source_timestamp_us=source_timestamp,
                raw_nis=float(nis),
                clipped_nis=min(float(nis), self.per_observation_nis_cap),
            )
        )
        while self.values and (
            arrival_timestamp_us - self.values[0].arrival_timestamp_us > self.maximum_window_elapsed_us
        ):
            self.values.popleft()
            self.time_span_trim_count += 1
        if len(self.values) < self.window_observations:
            return False
        score = float(sum(sample.clipped_nis for sample in self.values))
        contributors = sum(sample.raw_nis >= self.contribution_nis_floor for sample in self.values)
        self.completed_windows += 1
        self.maximum_window_nis = max(self.maximum_window_nis, score)
        self.maximum_window_contributing_observations = max(
            self.maximum_window_contributing_observations, contributors
        )
        if (
            score >= self.cumulative_nis_threshold
            and contributors >= self.minimum_contributing_observations
        ):
            self.latched = True
            self.first_latch_timestamp_us = arrival_timestamp_us
            self.latch_window = tuple(self.values)
        return self.latched


def load_protocol(path: Path = DEFAULT_PROTOCOL) -> dict[str, Any]:
    protocol = json.loads(path.read_text(encoding="utf-8"))
    if protocol.get("schema_version") != 2:
        raise ValueError("unsupported mismatch-monitor protocol schema")
    status = protocol.get("status")
    if status not in {
        "development_only_validation",
        "sealed_holdout_validation_only",
    }:
        raise ValueError("mismatch-monitor protocol must remain validation-only")
    if protocol.get("scope", {}).get("production_eskf") != "unchanged_16_nominal_15_error_state":
        raise ValueError("mismatch monitor protocol must leave production ESKF unchanged")
    required_paths = (
        "base_protocol_path",
        "base_protocol_file_sha256",
        "base_oracle_runner_file_sha256",
    )
    if any(not isinstance(protocol.get(name), str) or not protocol[name] for name in required_paths):
        raise ValueError("mismatch monitor protocol does not freeze its base source")
    monitor = protocol.get("monitor")
    required_monitor = (
        "window_observations",
        "maximum_window_elapsed_s",
        "maximum_inter_observation_gap_s",
        "per_observation_nis_cap",
        "contribution_nis_floor",
        "minimum_contributing_observations",
        "cumulative_nis_threshold",
        "latch_is_terminal_without_external_reauthorization",
    )
    if not isinstance(monitor, dict) or any(name not in monitor for name in required_monitor):
        raise ValueError("mismatch monitor contract is incomplete")
    if not bool(monitor["latch_is_terminal_without_external_reauthorization"]):
        raise ValueError("mismatch monitor latch must not auto-recover")
    stream = protocol.get("stream")
    required_stream = (
        "base_case_name",
        "tail_heading_deg",
        "pre_injection_clean_observations",
        "persistent_injection_observations",
        "post_impulse_clean_observations",
        "gap_before_impulse_s",
    )
    if not isinstance(stream, dict) or any(name not in stream for name in required_stream):
        raise ValueError("mismatch monitor stream contract is incomplete")
    if str(stream["base_case_name"]) != "multi_heading_nominal":
        raise ValueError("v2 monitor study must begin from the frozen multi-heading base case")
    if any(int(stream[name]) <= 0 for name in (
        "pre_injection_clean_observations",
        "persistent_injection_observations",
        "post_impulse_clean_observations",
    )):
        raise ValueError("monitor stream counts must be positive")
    coverage = protocol.get("coverage")
    required_coverage = (
        "minimum_pre_injection_eligible_observations",
        "minimum_pre_injection_completed_windows",
    )
    if not isinstance(coverage, dict) or any(name not in coverage for name in required_coverage):
        raise ValueError("monitor coverage contract is incomplete")
    seed_sets = protocol.get("seed_sets")
    expected_seed_set_names = (
        {"development"} if status == "development_only_validation" else {"sealed_holdout"}
    )
    if not isinstance(seed_sets, dict) or set(seed_sets) != expected_seed_set_names:
        raise ValueError("mismatch monitor seed-set layout does not match its protocol status")
    all_seeds: list[int] = []
    for name in sorted(seed_sets):
        seeds = seed_sets[name]
        if not isinstance(seeds, list) or not seeds or any(not isinstance(seed, int) for seed in seeds):
            raise ValueError(f"mismatch monitor {name} seeds must be non-empty integers")
        if len(set(seeds)) != len(seeds):
            raise ValueError(f"mismatch monitor {name} seeds must be unique")
        all_seeds.extend(seeds)
    if len(set(all_seeds)) != len(all_seeds):
        raise ValueError("development and sealed holdout seeds must be disjoint")
    cases = protocol.get("case_matrix")
    if not isinstance(cases, list) or not cases:
        raise ValueError("mismatch monitor case matrix must not be empty")
    names: set[str] = set()
    for case in cases:
        if not isinstance(case, dict) or not isinstance(case.get("name"), str):
            raise ValueError("monitor case must have a string name")
        if case["name"] in names:
            raise ValueError("monitor case names must be unique")
        names.add(case["name"])
        expected = case.get("expected")
        if not isinstance(expected, dict) or not isinstance(expected.get("monitor_latched"), bool):
            raise ValueError(f"monitor case {case['name']} has no boolean latch expectation")
        injection = case.get("injection")
        if not isinstance(injection, dict) or injection.get("kind") not in {
            "none", "tas_impulse", "tas_offset", "horizontal_wind_step", "vertical_wind",
        }:
            raise ValueError(f"monitor case {case['name']} has unsupported injection kind")
        if float(case.get("noise_multiplier", 1.0)) <= 0.0:
            raise ValueError(f"monitor case {case['name']} needs a positive noise multiplier")
    return protocol


def resolve_base_protocol(protocol: dict[str, Any]) -> dict[str, Any]:
    relative = Path(str(protocol["base_protocol_path"]))
    if relative.is_absolute() or ".." in relative.parts:
        raise ValueError("base protocol must be repository-relative")
    path = ROOT / relative
    if file_sha256(path) != str(protocol["base_protocol_file_sha256"]):
        raise ValueError("base protocol SHA-256 differs from the frozen v2 monitor protocol")
    if file_sha256(ROOT / "validation" / "run_airspeed_wind_observability.py") != str(
        protocol["base_oracle_runner_file_sha256"]
    ):
        raise ValueError("base oracle/generator SHA-256 differs from the frozen v2 monitor protocol")
    return base.load_protocol(path)


def verify_sealed_holdout(protocol: dict[str, Any]) -> None:
    expected_hash = protocol.get("candidate_runner_file_sha256")
    if not isinstance(expected_hash, str) or len(expected_hash) != 64:
        raise ValueError("sealed holdout requires a frozen candidate runner SHA-256")
    if file_sha256(RUNNER_PATH) != expected_hash:
        raise ValueError("candidate runner SHA-256 differs from the sealed holdout protocol")
    if capture(["git", "status", "--porcelain"]) != "":
        raise ValueError("sealed holdout requires a clean Git worktree")


def monitor_from_protocol(protocol: dict[str, Any]) -> CumulativeNisMonitor:
    config = protocol["monitor"]
    return CumulativeNisMonitor(
        window_observations=int(config["window_observations"]),
        maximum_window_elapsed_us=int(round(float(config["maximum_window_elapsed_s"]) * 1.0e6)),
        maximum_inter_observation_gap_us=int(
            round(float(config["maximum_inter_observation_gap_s"]) * 1.0e6)
        ),
        per_observation_nis_cap=float(config["per_observation_nis_cap"]),
        contribution_nis_floor=float(config["contribution_nis_floor"]),
        minimum_contributing_observations=int(config["minimum_contributing_observations"]),
        cumulative_nis_threshold=float(config["cumulative_nis_threshold"]),
    )


def tail_rng(seed: int, case_name: str) -> np.random.Generator:
    digest = hashlib.sha256(f"aerakia-monitor-v2-tail:{case_name}".encode("utf-8")).digest()
    return np.random.default_rng(int(seed) ^ int.from_bytes(digest[:8], "big"))


def append_observations(
    observations: list[base.TasWindObservation],
    *,
    count: int,
    timestamp_us: int,
    interval_us: int,
    heading_deg: float,
    tas_true_m_s: float,
    wind_ned_m_s: np.ndarray,
    source: dict[str, Any],
    rng: np.random.Generator,
    tas_measurement_offset_m_s: float = 0.0,
) -> int:
    """Append source-like runtime observations; return next source timestamp."""

    now_us = timestamp_us
    for _ in range(count):
        observation, _truth = base._build_observation(
            timestamp_us=now_us,
            heading_deg=heading_deg,
            tas_true_m_s=tas_true_m_s,
            wind_ned_m_s=wind_ned_m_s,
            source=source,
            rng=rng,
            tas_measurement_offset_m_s=tas_measurement_offset_m_s,
        )
        observations.append(observation)
        now_us += interval_us
    return now_us


def build_case_stream(
    case: dict[str, Any],
    protocol: dict[str, Any],
    base_protocol: dict[str, Any],
    *,
    synthetic_seed: int,
) -> tuple[list[base.TasWindObservation], int | None, set[int], int]:
    """Build a deterministic source stream with evaluator-only injection labels."""

    # Keep the frozen base bootstrap unchanged.  A high-noise control is a
    # post-qualification source condition, not a reason to invalidate the
    # unrelated geometry bootstrap before this study can start.
    observations, _truths, _now = base.generate_scenario(
        str(protocol["stream"]["base_case_name"]), base_protocol, synthetic_seed=synthetic_seed
    )
    source = copy.deepcopy(base_protocol["synthetic_source"])
    noise_multiplier = float(case.get("noise_multiplier", 1.0))
    source["tas_noise_std_m_s"] = float(source["tas_noise_std_m_s"]) * noise_multiplier
    source["gnss_velocity_noise_std_m_s"] = float(source["gnss_velocity_noise_std_m_s"]) * noise_multiplier
    rate_hz = float(source["sample_rate_hz"])
    interval_us = int(round(1.0e6 / rate_hz))
    timestamp_us = observations[-1].tas_timestamp_us + interval_us
    heading_deg = float(protocol["stream"]["tail_heading_deg"])
    nominal_wind_ne = source["nominal_wind_ne_m_s"]
    nominal_wind = np.asarray((nominal_wind_ne[0], nominal_wind_ne[1], 0.0), dtype=np.float64)
    nominal_tas = float(source["nominal_tas_m_s"])
    rng = tail_rng(synthetic_seed, str(case["name"]))
    timestamp_us = append_observations(
        observations,
        count=int(protocol["stream"]["pre_injection_clean_observations"]),
        timestamp_us=timestamp_us,
        interval_us=interval_us,
        heading_deg=heading_deg,
        tas_true_m_s=nominal_tas,
        wind_ned_m_s=nominal_wind,
        source=source,
        rng=rng,
    )

    injection = case["injection"]
    kind = str(injection["kind"])
    injection_start_timestamp_us: int | None = None
    injected_source_timestamps: set[int] = set()
    if kind == "none":
        timestamp_us = append_observations(
            observations,
            count=int(protocol["stream"]["post_impulse_clean_observations"]),
            timestamp_us=timestamp_us,
            interval_us=interval_us,
            heading_deg=heading_deg,
            tas_true_m_s=nominal_tas,
            wind_ned_m_s=nominal_wind,
            source=source,
            rng=rng,
        )
    elif kind == "tas_impulse":
        if bool(injection.get("insert_gap_before", False)):
            timestamp_us += int(round(float(protocol["stream"]["gap_before_impulse_s"]) * 1.0e6))
        injection_start_timestamp_us = timestamp_us
        injected_source_timestamps.add(timestamp_us)
        timestamp_us = append_observations(
            observations,
            count=1,
            timestamp_us=timestamp_us,
            interval_us=interval_us,
            heading_deg=heading_deg,
            tas_true_m_s=nominal_tas,
            wind_ned_m_s=nominal_wind,
            source=source,
            rng=rng,
            tas_measurement_offset_m_s=float(injection["value"]),
        )
        timestamp_us = append_observations(
            observations,
            count=int(protocol["stream"]["post_impulse_clean_observations"]),
            timestamp_us=timestamp_us,
            interval_us=interval_us,
            heading_deg=heading_deg,
            tas_true_m_s=nominal_tas,
            wind_ned_m_s=nominal_wind,
            source=source,
            rng=rng,
        )
    else:
        injection_start_timestamp_us = timestamp_us
        injection_count = int(protocol["stream"]["persistent_injection_observations"])
        injected_source_timestamps = {timestamp_us + index * interval_us for index in range(injection_count)}
        injection_wind = nominal_wind.copy()
        tas_offset = 0.0
        if kind == "tas_offset":
            tas_offset = float(injection["value"])
        elif kind == "horizontal_wind_step":
            injection_wind[0] += float(injection["value"])
        elif kind == "vertical_wind":
            injection_wind[2] = float(injection["value"])
        timestamp_us = append_observations(
            observations,
            count=injection_count,
            timestamp_us=timestamp_us,
            interval_us=interval_us,
            heading_deg=heading_deg,
            tas_true_m_s=nominal_tas,
            wind_ned_m_s=injection_wind,
            source=source,
            rng=rng,
            tas_measurement_offset_m_s=tas_offset,
        )
    final_now_us = observations[-1].arrival_timestamp_us
    return observations, injection_start_timestamp_us, injected_source_timestamps, final_now_us


def evaluate_candidate_case(
    case: dict[str, Any],
    protocol: dict[str, Any],
    base_protocol: dict[str, Any],
    *,
    synthetic_seed: int,
) -> dict[str, object]:
    """Run a source-like stream without allowing labels/truth into the monitor."""

    observations, injection_start, injected_timestamps, final_now_us = build_case_stream(
        case, protocol, base_protocol, synthetic_seed=synthetic_seed
    )
    shadow_protocol = copy.deepcopy(base_protocol)
    # Preserve source validity and geometry behavior, but retain all raw
    # post-qualification residuals for this independent monitor study.
    shadow_protocol["causal_oracle"]["maximum_consecutive_post_qualification_innovation_rejections"] = 1_000_000
    shadow = base.CausalWindOracle(shadow_protocol)
    monitor = monitor_from_protocol(protocol)
    source_rejections: Counter[str] = Counter()
    pre_injection_eligible_observations = 0
    pre_injection_completed_windows = 0
    raw_post_geometry_nis_observations = 0
    raw_post_geometry_nis_max = 0.0

    for observation in observations:
        geometry_qualified_before = shadow._geometry_qualified()  # validation-only boundary
        event = shadow.step(observation)
        if not event.accepted and event.reason != "innovation_nis":
            source_rejections[event.reason] += 1
        if not geometry_qualified_before:
            continue
        if event.innovation_nis is None:
            monitor.reset(f"source_without_runtime_nis:{event.reason}")
            continue
        raw_post_geometry_nis_observations += 1
        raw_post_geometry_nis_max = max(raw_post_geometry_nis_max, float(event.innovation_nis))
        before_windows = monitor.completed_windows
        monitor.observe(
            observation.arrival_timestamp_us,
            float(event.innovation_nis),
            source_timestamp_us=observation.tas_timestamp_us,
        )
        before_injection = injection_start is None or observation.tas_timestamp_us < injection_start
        if before_injection:
            pre_injection_eligible_observations += 1
            pre_injection_completed_windows += monitor.completed_windows - before_windows

    expected = case["expected"]
    errors: list[str] = []
    coverage = protocol["coverage"]
    if pre_injection_eligible_observations < int(coverage["minimum_pre_injection_eligible_observations"]):
        errors.append("insufficient_postqualification_coverage")
    if pre_injection_completed_windows < int(coverage["minimum_pre_injection_completed_windows"]):
        errors.append("insufficient_complete_window_coverage")
    expected_latch = bool(expected["monitor_latched"])
    if monitor.latched != expected_latch:
        errors.append(f"monitor_latched={monitor.latched} but expected={expected_latch}")
    latch_window_injection_observations = 0
    latch_delay_s: float | None = None
    pre_injection_latch = False
    if monitor.first_latch_timestamp_us is not None:
        if injection_start is not None:
            latch_delay_s = (monitor.first_latch_timestamp_us - injection_start) * 1.0e-6
            pre_injection_latch = monitor.first_latch_timestamp_us < injection_start
        if monitor.latch_window is not None:
            latch_window_injection_observations = sum(
                sample.source_timestamp_us in injected_timestamps for sample in monitor.latch_window
            )
    if pre_injection_latch:
        errors.append("monitor_latched_before_injection")
    if expected_latch:
        minimum_injection_contributors = int(expected["minimum_latch_window_injection_observations"])
        if latch_window_injection_observations < minimum_injection_contributors:
            errors.append("latch_window_has_insufficient_injection_observations")
        maximum_latch_delay_s = float(expected["maximum_latch_delay_s"])
        if latch_delay_s is None or latch_delay_s > maximum_latch_delay_s:
            errors.append("monitor_latch_exceeded_deadline")
    minimum_gap_resets = int(expected.get("minimum_inter_observation_gap_resets", 0))
    if int(monitor.reset_reasons["inter_observation_gap"]) < minimum_gap_resets:
        errors.append("missing_required_inter_observation_gap_reset")
    return {
        "name": case["name"],
        "synthetic_seed": synthetic_seed,
        "expected_monitor_latched": expected_latch,
        "passed": not errors,
        "errors": errors,
        "input_observations": len(observations),
        "raw_post_geometry_nis_observations": raw_post_geometry_nis_observations,
        "maximum_raw_post_geometry_nis": raw_post_geometry_nis_max if raw_post_geometry_nis_observations else None,
        "pre_injection_eligible_observations": pre_injection_eligible_observations,
        "pre_injection_completed_windows": pre_injection_completed_windows,
        "monitor_latched": monitor.latched,
        "monitor_first_latch_timestamp_us": monitor.first_latch_timestamp_us,
        "monitor_latch_delay_s": latch_delay_s,
        "monitor_latch_window_injection_observations": latch_window_injection_observations,
        "monitor_eligible_observations": monitor.eligible_observations,
        "monitor_completed_windows": monitor.completed_windows,
        "monitor_maximum_window_nis": monitor.maximum_window_nis,
        "monitor_maximum_window_contributing_observations": monitor.maximum_window_contributing_observations,
        "monitor_reset_reasons": dict(sorted(monitor.reset_reasons.items())),
        "monitor_time_span_trim_count": monitor.time_span_trim_count,
        "source_rejections": dict(sorted(source_rejections.items())),
        "shadow_terminal_status": shadow.terminal_status(final_now_us),
    }


def evaluate_task(task: tuple[dict[str, Any], dict[str, Any], dict[str, Any], int]) -> dict[str, object]:
    """Process-pool entry point; each task is isolated validation state."""

    case, protocol, base_protocol, seed = task
    return evaluate_candidate_case(case, protocol, base_protocol, synthetic_seed=seed)


def percentile_summary(values: list[float | int]) -> dict[str, float | None]:
    if not values:
        return {"minimum": None, "p05": None, "p50": None, "p95": None, "maximum": None}
    ordered = sorted(float(value) for value in values)

    def pick(fraction: float) -> float:
        return ordered[int(math.floor(fraction * (len(ordered) - 1)))]

    return {
        "minimum": ordered[0],
        "p05": pick(0.05),
        "p50": pick(0.50),
        "p95": pick(0.95),
        "maximum": ordered[-1],
    }


def zero_event_upper_bound_95(events: int, trials: int) -> float | None:
    """Exact one-sided binomial upper bound when zero events are observed."""

    if trials <= 0 or events != 0:
        return None
    return 1.0 - math.pow(0.05, 1.0 / trials)


def default_output_path(protocol: dict[str, Any], phase: str) -> Path:
    """Return the versioned compact/development artifact path for a protocol.

    A sealed campaign must never silently overwrite a prior version's compact
    record. Keep the allowed protocol identifier deliberately narrow rather
    than deriving an arbitrary filesystem path from JSON.
    """

    match = re.fullmatch(
        r"aerakia-airspeed-wind-mismatch-monitor-(v[0-9]+)",
        str(protocol.get("protocol_id", "")),
    )
    if match is None:
        raise ValueError("mismatch-monitor protocol ID has no safe versioned output name")
    version = match.group(1)
    if phase == "sealed_holdout":
        return ROOT / "validation" / "public" / f"airspeed_wind_mismatch_monitor_{version}.json"
    if phase == "development":
        return ROOT / "build" / f"airspeed-wind-mismatch-monitor-{version}-development.json"
    raise ValueError("unknown mismatch-monitor campaign phase")


def summarize_case(items: list[dict[str, object]], case: dict[str, Any]) -> dict[str, object]:
    expected_latch = bool(case["expected"]["monitor_latched"])
    latches = sum(bool(item["monitor_latched"]) for item in items)
    failures = [item for item in items if not bool(item["passed"])]
    pre_latches = sum(
        item["monitor_first_latch_timestamp_us"] is not None
        and item["monitor_latch_delay_s"] is not None
        and float(item["monitor_latch_delay_s"]) < 0.0
        for item in items
    )
    return {
        "name": case["name"],
        "expected_monitor_latched": expected_latch,
        "replications": len(items),
        "passed_replications": len(items) - len(failures),
        "failed_replications": len(failures),
        "monitor_latches": latches,
        "monitor_non_latches": len(items) - latches,
        "pre_injection_latches": pre_latches,
        "zero_false_latch_one_sided_95_upper_bound": (
            zero_event_upper_bound_95(latches, len(items)) if not expected_latch else None
        ),
        "zero_detection_miss_one_sided_95_upper_bound": (
            zero_event_upper_bound_95(len(items) - latches, len(items)) if expected_latch else None
        ),
        "input_observations": sum(int(item["input_observations"]) for item in items),
        "pre_injection_eligible_observations": percentile_summary(
            [int(item["pre_injection_eligible_observations"]) for item in items]
        ),
        "pre_injection_completed_windows": percentile_summary(
            [int(item["pre_injection_completed_windows"]) for item in items]
        ),
        "monitor_completed_windows": percentile_summary(
            [int(item["monitor_completed_windows"]) for item in items]
        ),
        "maximum_window_nis": percentile_summary(
            [float(item["monitor_maximum_window_nis"]) for item in items]
        ),
        "latch_delay_s": percentile_summary(
            [float(item["monitor_latch_delay_s"]) for item in items if item["monitor_latch_delay_s"] is not None]
        ),
        "latch_window_injection_observations": percentile_summary(
            [int(item["monitor_latch_window_injection_observations"]) for item in items]
        ),
        "failed_case_records": failures,
    }


def select_phase_seeds(
    protocol: dict[str, Any],
    phase: str,
    *,
    start_index: int = 0,
    count: int | None = None,
) -> list[int]:
    """Select a canonical contiguous shard without inventing new seed order."""

    if phase not in ("development", "sealed_holdout"):
        raise ValueError("unknown mismatch-monitor campaign phase")
    if phase not in protocol["seed_sets"]:
        raise ValueError(f"mismatch-monitor protocol has no {phase!r} seed set")
    if start_index < 0:
        raise ValueError("seed start index must be non-negative")
    if count is not None and count <= 0:
        raise ValueError("seed count must be positive when supplied")
    phase_seeds = [int(seed) for seed in protocol["seed_sets"][phase]]
    selected = phase_seeds[start_index:] if count is None else phase_seeds[start_index : start_index + count]
    if not selected:
        raise ValueError("selected seed shard is empty or starts beyond the protocol seed set")
    return selected


def assemble_result(
    protocol: dict[str, Any],
    base_protocol: dict[str, Any],
    *,
    phase: str,
    selected_seeds: list[int],
    records: list[dict[str, object]],
    include_records: bool,
) -> dict[str, object]:
    """Create a compact or shard result from evaluated case records."""

    cases = protocol["case_matrix"]
    records.sort(key=lambda item: (str(item["name"]), int(item["synthetic_seed"])))
    grouped: dict[str, list[dict[str, object]]] = defaultdict(list)
    for record in records:
        grouped[str(record["name"])].append(record)
    summaries = [summarize_case(grouped[str(case["name"])], case) for case in cases]
    failures = [record for record in records if not bool(record["passed"])]
    result: dict[str, object] = {
        "schema_version": 2,
        "study_id": protocol["protocol_id"],
        "phase": phase,
        "status": "passed" if not failures else "failed",
        "monitor": protocol["monitor"],
        "coverage": protocol["coverage"],
        "seed_count": len(selected_seeds),
        "selected_seeds": selected_seeds,
        "selected_seed_canonical_sha256": base.canonical_sha256(selected_seeds),
        "case_summaries": summaries,
        "totals": {
            "replication_cases": len(records),
            "passed_replication_cases": len(records) - len(failures),
            "failed_replication_cases": len(failures),
            "input_observations": sum(int(record["input_observations"]) for record in records),
            "complete_monitor_windows": sum(int(record["monitor_completed_windows"]) for record in records),
        },
        "all_record_canonical_sha256": base.canonical_sha256(records),
        "failed_case_records": failures,
        "limitations": [
            "This is a host-only synthetic residual-pattern screen, not a physical pitot, wind, or air-data source-quality qualification.",
            "A pass does not make vertical wind independently observable and does not promote TAS, a wind state, or a private flight-supervisor decision.",
            "Replication-level zero-event bounds assume independent synthetic seeds; overlapping rolling windows are not treated as independent trials.",
        ],
    }
    if include_records:
        result["records"] = records
    return result


def run_protocol(
    protocol: dict[str, Any],
    base_protocol: dict[str, Any],
    *,
    phase: str,
    jobs: int,
    selected_seeds: list[int] | None = None,
    include_records: bool = False,
) -> dict[str, object]:
    """Run one complete campaign or a contiguous, explicit seed shard."""

    phase_seeds = select_phase_seeds(protocol, phase)
    selected = phase_seeds if selected_seeds is None else [int(seed) for seed in selected_seeds]
    canonical_selected = [seed for seed in phase_seeds if seed in set(selected)]
    if selected != canonical_selected:
        raise ValueError("selected seeds must be unique, protocol members, and in canonical protocol order")
    if jobs <= 0:
        raise ValueError("jobs must be positive")
    cases = protocol["case_matrix"]
    tasks = [(case, protocol, base_protocol, seed) for seed in selected for case in cases]

    # A one-worker campaign has no parallelism to gain. Running it in-process
    # keeps the validation path usable in restricted desktop/sandbox runners
    # that intentionally terminate child-process creation. It also makes the
    # smallest reproduction path independent of multiprocessing semantics.
    if jobs == 1:
        records = [evaluate_task(task) for task in tasks]
    else:
        # Python 3.14 defaults to ``forkserver`` on this Linux host, which is
        # deliberately unavailable in the sandbox. Explicit ``fork`` keeps the
        # independent seed cases parallel without affecting Windows, where
        # spawn remains the only available method.
        methods = multiprocessing.get_all_start_methods()
        context = multiprocessing.get_context("fork") if "fork" in methods else None
        with ProcessPoolExecutor(max_workers=jobs, mp_context=context) as executor:
            records = list(executor.map(evaluate_task, tasks))
    return assemble_result(
        protocol,
        base_protocol,
        phase=phase,
        selected_seeds=selected,
        records=records,
        include_records=include_records,
    )


def decorate_result(
    result: dict[str, object],
    *,
    protocol_path: Path,
    jobs: int,
) -> None:
    """Attach immutable source/provenance information before writing a result."""

    result["protocol"] = {
        "path": str(protocol_path.relative_to(ROOT)) if protocol_path.is_relative_to(ROOT) else str(protocol_path),
        "file_sha256": file_sha256(protocol_path),
        "semantic_sha256": base.canonical_sha256(load_protocol(protocol_path)),
    }
    result["provenance"] = {
        "runner_sha256": file_sha256(RUNNER_PATH),
        "base_oracle_runner_sha256": file_sha256(ROOT / "validation" / "run_airspeed_wind_observability.py"),
        "git_commit": capture(["git", "rev-parse", "HEAD"]),
        "git_status": capture(["git", "status", "--short"]),
        "jobs": jobs,
    }


def merge_shard_results(
    protocol: dict[str, Any],
    base_protocol: dict[str, Any],
    *,
    phase: str,
    shard_paths: list[Path],
) -> dict[str, object]:
    """Fail closed while reassembling compact evidence from raw-record shards."""

    expected_seeds = select_phase_seeds(protocol, phase)
    expected_cases = {str(case["name"]) for case in protocol["case_matrix"]}
    expected_protocol_semantic_sha = base.canonical_sha256(protocol)
    expected_runner_sha = file_sha256(RUNNER_PATH)
    seen_seeds: set[int] = set()
    all_records: list[dict[str, object]] = []
    shard_manifest: list[dict[str, object]] = []
    source_commit: str | None = None
    for raw_path in shard_paths:
        path = raw_path.resolve()
        if not path.is_file():
            raise ValueError(f"missing shard result: {path}")
        item = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(item, dict):
            raise ValueError("shard result must be a JSON object")
        if item.get("study_id") != protocol["protocol_id"] or item.get("phase") != phase:
            raise ValueError("shard belongs to a different study or phase")
        if item.get("protocol", {}).get("semantic_sha256") != expected_protocol_semantic_sha:
            raise ValueError("shard protocol semantic fingerprint differs from the merge protocol")
        provenance = item.get("provenance")
        if not isinstance(provenance, dict) or provenance.get("runner_sha256") != expected_runner_sha:
            raise ValueError("shard runner fingerprint differs from the frozen merge runner")
        if provenance.get("git_status") != "":
            raise ValueError("shard was not generated from a clean Git worktree")
        commit = provenance.get("git_commit")
        if not isinstance(commit, str) or not commit:
            raise ValueError("shard lacks a source Git commit")
        if source_commit is None:
            source_commit = commit
        elif commit != source_commit:
            raise ValueError("shards were generated from different Git commits")
        seeds = item.get("selected_seeds")
        records = item.get("records")
        if not isinstance(seeds, list) or not seeds or not isinstance(records, list):
            raise ValueError("shard must retain selected seeds and raw case records")
        normalized_seeds = [int(seed) for seed in seeds]
        if normalized_seeds != [seed for seed in expected_seeds if seed in set(normalized_seeds)]:
            raise ValueError("shard seeds are not a canonical subset of this protocol")
        if seen_seeds.intersection(normalized_seeds):
            raise ValueError("shard seed sets overlap")
        expected_keys = {(name, seed) for name in expected_cases for seed in normalized_seeds}
        actual_keys: set[tuple[str, int]] = set()
        normalized_records: list[dict[str, object]] = []
        for record in records:
            if not isinstance(record, dict):
                raise ValueError("shard case record must be an object")
            key = (str(record.get("name")), int(record.get("synthetic_seed")))
            if key not in expected_keys or key in actual_keys:
                raise ValueError("shard records are incomplete, foreign, or duplicated")
            actual_keys.add(key)
            normalized_records.append(record)
        if actual_keys != expected_keys:
            raise ValueError("shard does not contain exactly one record per case and selected seed")
        seen_seeds.update(normalized_seeds)
        all_records.extend(normalized_records)
        shard_manifest.append(
            {
                "path": str(path.relative_to(ROOT)) if path.is_relative_to(ROOT) else str(path),
                "file_sha256": file_sha256(path),
                "seed_count": len(normalized_seeds),
                "selected_seed_canonical_sha256": base.canonical_sha256(normalized_seeds),
            }
        )
    if seen_seeds != set(expected_seeds):
        raise ValueError("shards do not cover the complete frozen seed set exactly once")
    result = assemble_result(
        protocol,
        base_protocol,
        phase=phase,
        selected_seeds=expected_seeds,
        records=all_records,
        include_records=False,
    )
    result["campaign_assembly"] = {
        "mode": "strict_raw_record_shard_merge",
        "source_git_commit": source_commit,
        "shard_count": len(shard_manifest),
        "shards": shard_manifest,
    }
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--protocol", type=Path, default=DEFAULT_PROTOCOL)
    parser.add_argument("--phase", choices=("development", "sealed_holdout"), default="development")
    parser.add_argument("--out", type=Path)
    parser.add_argument("--jobs", type=int, default=min(8, max(1, os.cpu_count() or 1)))
    parser.add_argument("--seed-start-index", type=int, default=0)
    parser.add_argument("--seed-count", type=int)
    parser.add_argument(
        "--emit-records",
        action="store_true",
        help="retain raw per-case records for a temporary shard merge",
    )
    parser.add_argument(
        "--merge-shards",
        type=Path,
        nargs="+",
        help="strictly merge clean, raw-record shard JSON files instead of evaluating seeds",
    )
    args = parser.parse_args()
    protocol_path = args.protocol.resolve()
    protocol = load_protocol(protocol_path)
    base_protocol = resolve_base_protocol(protocol)
    if args.phase == "sealed_holdout":
        verify_sealed_holdout(protocol)
    if args.merge_shards is not None:
        if args.seed_start_index != 0 or args.seed_count is not None or args.emit_records:
            raise ValueError("shard merge cannot also select seeds or request raw records")
        result = merge_shard_results(
            protocol,
            base_protocol,
            phase=args.phase,
            shard_paths=args.merge_shards,
        )
    else:
        selected_seeds = select_phase_seeds(
            protocol,
            args.phase,
            start_index=args.seed_start_index,
            count=args.seed_count,
        )
        full_seed_set = selected_seeds == select_phase_seeds(protocol, args.phase)
        if not full_seed_set and args.out is None:
            raise ValueError("partial seed shards require an explicit --out path")
        result = run_protocol(
            protocol,
            base_protocol,
            phase=args.phase,
            jobs=args.jobs,
            selected_seeds=selected_seeds,
            include_records=args.emit_records,
        )
        result["campaign_partition"] = {
            "kind": "complete_campaign" if full_seed_set else "raw_record_shard",
            "phase_seed_count": len(select_phase_seeds(protocol, args.phase)),
        }
    decorate_result(result, protocol_path=protocol_path, jobs=args.jobs)
    if args.out is None:
        output = default_output_path(protocol, args.phase)
    else:
        output = args.out.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
