#!/usr/bin/env python3
"""Opened v5 diagnostic for residual episodes after the failed v4 holdout.

This file deliberately leaves the v2/v4 runner untouched: v4's source hash is
part of its sealed evidence. V5 is an independent development-only instrument,
not a release monitor or a production ESKF component.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import subprocess
from collections import Counter, defaultdict, deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

import run_airspeed_wind_observability as base
import run_airspeed_wind_mismatch_monitor as v4


ROOT = Path(__file__).resolve().parents[1]
RUNNER_PATH = ROOT / "validation" / "run_airspeed_wind_mismatch_monitor_v5.py"
DEFAULT_PROTOCOL = ROOT / "validation" / "airspeed_wind_mismatch_monitor_protocol_v5.json"


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
class EvidenceSample:
    """One causal residual retained only for development trace evidence."""

    arrival_timestamp_us: int
    source_timestamp_us: int
    raw_nis: float
    clipped_nis: float


@dataclass
class ConsecutiveEvidenceMonitor:
    """Terminal monitor that only combines post-reset consecutive high residuals.

    A low residual, a source gap, or an elapsed-span violation begins a new
    episode. This structural change prevents the v4 rolling window from
    combining an arbitrary old residual with one new sample. It does not solve
    the physical source-versus-wind ambiguity and is not flight policy.
    """

    warmup_eligible_observations: int
    maximum_inter_observation_gap_us: int
    maximum_episode_elapsed_us: int
    contribution_nis_floor: float
    per_observation_nis_cap: float
    minimum_consecutive_high_observations: int
    minimum_episode_elapsed_us: int
    cumulative_nis_threshold: float
    trace_observations: int
    last_arrival_timestamp_us: int | None = None
    eligible_observations: int = 0
    armed: bool = False
    latched: bool = False
    first_latch_timestamp_us: int | None = None
    episode: list[EvidenceSample] | None = None
    latch_episode: tuple[EvidenceSample, ...] | None = None
    reset_reasons: Counter[str] | None = None
    trace: deque[dict[str, object]] | None = None

    def __post_init__(self) -> None:
        if self.warmup_eligible_observations < 0:
            raise ValueError("warmup observation count cannot be negative")
        if self.maximum_inter_observation_gap_us <= 0 or self.maximum_episode_elapsed_us <= 0:
            raise ValueError("episode timing bounds must be positive")
        if self.maximum_inter_observation_gap_us > self.maximum_episode_elapsed_us:
            raise ValueError("gap bound cannot exceed episode span")
        if self.minimum_consecutive_high_observations < 2:
            raise ValueError("episode needs at least two high observations")
        if self.minimum_episode_elapsed_us < 0 or self.minimum_episode_elapsed_us > self.maximum_episode_elapsed_us:
            raise ValueError("minimum episode span is outside the configured episode bound")
        for name, value in (
            ("contribution NIS floor", self.contribution_nis_floor),
            ("per-observation NIS cap", self.per_observation_nis_cap),
            ("cumulative NIS threshold", self.cumulative_nis_threshold),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be positive and finite")
        if self.contribution_nis_floor > self.per_observation_nis_cap:
            raise ValueError("contribution NIS floor cannot exceed NIS cap")
        if self.trace_observations <= 0:
            raise ValueError("trace capacity must be positive")
        self.episode = []
        self.reset_reasons = Counter()
        self.trace = deque(maxlen=self.trace_observations)
        self.armed = self.warmup_eligible_observations == 0

    def _clear_episode(self, reason: str) -> None:
        assert self.episode is not None
        if self.episode:
            self.reset_reasons[reason] += 1
            self.episode.clear()

    def _record(self, *, arrival_timestamp_us: int, source_timestamp_us: int, raw_nis: float, reason: str) -> None:
        assert self.episode is not None
        assert self.trace is not None
        score = float(sum(item.clipped_nis for item in self.episode))
        span_us = 0 if len(self.episode) < 2 else arrival_timestamp_us - self.episode[0].arrival_timestamp_us
        self.trace.append(
            {
                "arrival_timestamp_us": arrival_timestamp_us,
                "source_timestamp_us": source_timestamp_us,
                "raw_nis": raw_nis,
                "armed": self.armed,
                "episode_observations": len(self.episode),
                "episode_elapsed_s": span_us * 1.0e-6,
                "episode_clipped_nis": score,
                "latched": self.latched,
                "reason": reason,
            }
        )

    def observe(self, arrival_timestamp_us: int, nis: float, *, source_timestamp_us: int | None = None) -> bool:
        """Consume a runtime residual without access to injected labels or truth."""

        source_timestamp = arrival_timestamp_us if source_timestamp_us is None else source_timestamp_us
        if (
            arrival_timestamp_us < 0
            or source_timestamp < 0
            or source_timestamp > arrival_timestamp_us
            or not math.isfinite(nis)
            or nis < 0.0
        ):
            raise ValueError("episode monitor requires finite non-negative causal timestamps and NIS")
        assert self.episode is not None
        if self.last_arrival_timestamp_us is not None:
            delta_us = arrival_timestamp_us - self.last_arrival_timestamp_us
            if delta_us <= 0:
                self._clear_episode("non_monotonic_arrival")
            elif delta_us > self.maximum_inter_observation_gap_us:
                self._clear_episode("inter_observation_gap")
        self.last_arrival_timestamp_us = arrival_timestamp_us
        self.eligible_observations += 1
        if self.latched:
            self._record(
                arrival_timestamp_us=arrival_timestamp_us,
                source_timestamp_us=source_timestamp,
                raw_nis=float(nis),
                reason="already_latched",
            )
            return True
        if not self.armed:
            if self.eligible_observations >= self.warmup_eligible_observations:
                self.armed = True
                reason = "armed_after_warmup"
            else:
                reason = "warmup"
            self._record(
                arrival_timestamp_us=arrival_timestamp_us,
                source_timestamp_us=source_timestamp,
                raw_nis=float(nis),
                reason=reason,
            )
            return False
        if nis < self.contribution_nis_floor:
            self._clear_episode("low_nis")
            self._record(
                arrival_timestamp_us=arrival_timestamp_us,
                source_timestamp_us=source_timestamp,
                raw_nis=float(nis),
                reason="low_nis",
            )
            return False
        if self.episode and arrival_timestamp_us - self.episode[0].arrival_timestamp_us > self.maximum_episode_elapsed_us:
            self._clear_episode("maximum_episode_span")
        self.episode.append(
            EvidenceSample(
                arrival_timestamp_us=arrival_timestamp_us,
                source_timestamp_us=source_timestamp,
                raw_nis=float(nis),
                clipped_nis=min(float(nis), self.per_observation_nis_cap),
            )
        )
        episode_span_us = arrival_timestamp_us - self.episode[0].arrival_timestamp_us
        score = sum(item.clipped_nis for item in self.episode)
        if (
            len(self.episode) >= self.minimum_consecutive_high_observations
            and episode_span_us >= self.minimum_episode_elapsed_us
            and score >= self.cumulative_nis_threshold
        ):
            self.latched = True
            self.first_latch_timestamp_us = arrival_timestamp_us
            self.latch_episode = tuple(self.episode)
            reason = "latched"
        else:
            reason = "high_evidence"
        self._record(
            arrival_timestamp_us=arrival_timestamp_us,
            source_timestamp_us=source_timestamp,
            raw_nis=float(nis),
            reason=reason,
        )
        return self.latched

    def reset_without_nis(self, reason: str) -> None:
        """Clear an unfinished episode when the source has no runtime residual."""

        if not self.latched:
            self._clear_episode(reason)


def load_protocol(path: Path = DEFAULT_PROTOCOL) -> dict[str, Any]:
    protocol = json.loads(path.read_text(encoding="utf-8"))
    if protocol.get("schema_version") != 1:
        raise ValueError("unsupported v5 residual-diagnostic protocol schema")
    if protocol.get("status") != "development_only_diagnostic":
        raise ValueError("v5 protocol must remain development-only diagnostic")
    if protocol.get("scope", {}).get("production_eskf") != "unchanged_16_nominal_15_error_state":
        raise ValueError("v5 protocol must leave production ESKF unchanged")
    for name in ("base_protocol_path", "base_protocol_file_sha256", "base_oracle_runner_file_sha256"):
        if not isinstance(protocol.get(name), str) or not protocol[name]:
            raise ValueError(f"v5 protocol lacks {name}")
    monitor = protocol.get("monitor")
    required_monitor = (
        "warmup_eligible_observations",
        "maximum_inter_observation_gap_s",
        "maximum_episode_elapsed_s",
        "contribution_nis_floor",
        "per_observation_nis_cap",
        "minimum_consecutive_high_observations",
        "minimum_episode_elapsed_s",
        "cumulative_nis_threshold",
        "trace_observations",
    )
    if not isinstance(monitor, dict) or any(name not in monitor for name in required_monitor):
        raise ValueError("v5 monitor contract is incomplete")
    stream = protocol.get("stream")
    required_stream = (
        "base_case_name",
        "tail_heading_deg",
        "pre_injection_clean_observations",
        "post_impulse_clean_observations",
        "persistent_injection_observations",
        "gap_before_impulse_s",
    )
    if not isinstance(stream, dict) or any(name not in stream for name in required_stream):
        raise ValueError("v5 stream contract is incomplete")
    seeds = protocol.get("seed_sets", {}).get("development")
    if not isinstance(seeds, list) or not seeds or any(not isinstance(seed, int) for seed in seeds):
        raise ValueError("v5 development seeds must be non-empty integers")
    if len(set(seeds)) != len(seeds):
        raise ValueError("v5 development seeds must be unique")
    cases = protocol.get("case_matrix")
    if not isinstance(cases, list) or not cases:
        raise ValueError("v5 case matrix must not be empty")
    names: set[str] = set()
    for case in cases:
        if not isinstance(case, dict) or not isinstance(case.get("name"), str):
            raise ValueError("every v5 case needs a string name")
        if case["name"] in names:
            raise ValueError("v5 case names must be unique")
        names.add(case["name"])
        if not isinstance(case.get("paired_prefix_group"), str) or not case["paired_prefix_group"]:
            raise ValueError(f"v5 case {case['name']} lacks a paired-prefix group")
        if float(case.get("noise_multiplier", 1.0)) <= 0.0:
            raise ValueError(f"v5 case {case['name']} needs positive noise multiplier")
        injection = case.get("injection")
        if not isinstance(injection, dict) or injection.get("kind") not in {
            "none", "tas_impulse_train", "tas_offset", "horizontal_wind_step", "vertical_wind",
        }:
            raise ValueError(f"v5 case {case['name']} has unsupported injection")
        scoring = case.get("scoring")
        expected = case.get("expected")
        if scoring not in {"required", "descriptive"} or not isinstance(expected, dict):
            raise ValueError(f"v5 case {case['name']} has invalid scoring contract")
        if scoring == "required" and not isinstance(expected.get("monitor_latched"), bool):
            raise ValueError(f"required v5 case {case['name']} needs boolean latch expectation")
        if scoring == "descriptive" and expected.get("monitor_latched") is not None:
            raise ValueError(f"descriptive v5 case {case['name']} must not impose a latch expectation")
    return protocol


def resolve_base_protocol(protocol: dict[str, Any]) -> dict[str, Any]:
    relative = Path(str(protocol["base_protocol_path"]))
    if relative.is_absolute() or ".." in relative.parts:
        raise ValueError("v5 base protocol must be repository-relative")
    path = ROOT / relative
    if file_sha256(path) != str(protocol["base_protocol_file_sha256"]):
        raise ValueError("v5 base protocol SHA-256 differs from frozen source")
    base_runner = ROOT / "validation" / "run_airspeed_wind_observability.py"
    if file_sha256(base_runner) != str(protocol["base_oracle_runner_file_sha256"]):
        raise ValueError("v5 base oracle runner SHA-256 differs from frozen source")
    return base.load_protocol(path)


def monitor_from_protocol(protocol: dict[str, Any]) -> ConsecutiveEvidenceMonitor:
    config = protocol["monitor"]
    return ConsecutiveEvidenceMonitor(
        warmup_eligible_observations=int(config["warmup_eligible_observations"]),
        maximum_inter_observation_gap_us=int(round(float(config["maximum_inter_observation_gap_s"]) * 1.0e6)),
        maximum_episode_elapsed_us=int(round(float(config["maximum_episode_elapsed_s"]) * 1.0e6)),
        contribution_nis_floor=float(config["contribution_nis_floor"]),
        per_observation_nis_cap=float(config["per_observation_nis_cap"]),
        minimum_consecutive_high_observations=int(config["minimum_consecutive_high_observations"]),
        minimum_episode_elapsed_us=int(round(float(config["minimum_episode_elapsed_s"]) * 1.0e6)),
        cumulative_nis_threshold=float(config["cumulative_nis_threshold"]),
        trace_observations=int(config["trace_observations"]),
    )


def common_tail_rng(seed: int) -> np.random.Generator:
    """Return case-independent tail noise for paired same-seed comparisons."""

    digest = hashlib.sha256(b"aerakia-monitor-v5-common-tail").digest()
    return np.random.default_rng(int(seed) ^ int.from_bytes(digest[:8], "big"))


def prefix_fingerprint(observations: list[base.TasWindObservation]) -> str:
    """Fingerprint runtime-visible common-prefix fields only."""

    payload = [
        {
            "tas_timestamp_us": item.tas_timestamp_us,
            "arrival_timestamp_us": item.arrival_timestamp_us,
            "gnss_timestamp_us": item.gnss_timestamp_us,
            "tas_m_s": item.tas_m_s,
            "tas_variance_m2_s2": item.tas_variance_m2_s2,
            "gnss_velocity_ned_m_s": item.gnss_velocity_ned_m_s,
            "gnss_velocity_variance_m2_s2": item.gnss_velocity_variance_m2_s2,
            "flight_regime": item.flight_regime,
            "source_qualified": item.source_qualified,
            "pitot_blocked": item.pitot_blocked,
            "pitot_stalled": item.pitot_stalled,
            "rotor_wash": item.rotor_wash,
            "sideslip_qualified": item.sideslip_qualified,
        }
        for item in observations
    ]
    return base.canonical_sha256(payload)


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
    return v4.append_observations(
        observations,
        count=count,
        timestamp_us=timestamp_us,
        interval_us=interval_us,
        heading_deg=heading_deg,
        tas_true_m_s=tas_true_m_s,
        wind_ned_m_s=wind_ned_m_s,
        source=source,
        rng=rng,
        tas_measurement_offset_m_s=tas_measurement_offset_m_s,
    )


def build_case_stream(
    case: dict[str, Any], protocol: dict[str, Any], base_protocol: dict[str, Any], *, synthetic_seed: int,
) -> tuple[list[base.TasWindObservation], int | None, set[int], int, str]:
    """Build a paired stream; injected labels remain unavailable to the monitor."""

    observations, _truth, _now = base.generate_scenario(
        str(protocol["stream"]["base_case_name"]), base_protocol, synthetic_seed=synthetic_seed
    )
    source = copy.deepcopy(base_protocol["synthetic_source"])
    multiplier = float(case.get("noise_multiplier", 1.0))
    source["tas_noise_std_m_s"] = float(source["tas_noise_std_m_s"]) * multiplier
    source["gnss_velocity_noise_std_m_s"] = float(source["gnss_velocity_noise_std_m_s"]) * multiplier
    interval_us = int(round(1.0e6 / float(source["sample_rate_hz"])))
    timestamp_us = observations[-1].tas_timestamp_us + interval_us
    tail_heading = float(protocol["stream"]["tail_heading_deg"])
    wind_ne = source["nominal_wind_ne_m_s"]
    nominal_wind = np.asarray((wind_ne[0], wind_ne[1], 0.0), dtype=np.float64)
    nominal_tas = float(source["nominal_tas_m_s"])
    rng = common_tail_rng(synthetic_seed)
    timestamp_us = append_observations(
        observations,
        count=int(protocol["stream"]["pre_injection_clean_observations"]),
        timestamp_us=timestamp_us,
        interval_us=interval_us,
        heading_deg=tail_heading,
        tas_true_m_s=nominal_tas,
        wind_ned_m_s=nominal_wind,
        source=source,
        rng=rng,
    )
    common_prefix_sha256 = prefix_fingerprint(observations)
    injection = case["injection"]
    kind = str(injection["kind"])
    injection_start: int | None = None
    injected_timestamps: set[int] = set()
    if kind == "none":
        timestamp_us = append_observations(
            observations,
            count=int(protocol["stream"]["post_impulse_clean_observations"]),
            timestamp_us=timestamp_us,
            interval_us=interval_us,
            heading_deg=tail_heading,
            tas_true_m_s=nominal_tas,
            wind_ned_m_s=nominal_wind,
            source=source,
            rng=rng,
        )
    elif kind == "tas_impulse_train":
        if bool(injection.get("insert_gap_before", False)):
            timestamp_us += int(round(float(protocol["stream"]["gap_before_impulse_s"]) * 1.0e6))
        injection_start = timestamp_us
        count = int(injection["count"])
        injected_timestamps = {timestamp_us + index * interval_us for index in range(count)}
        timestamp_us = append_observations(
            observations,
            count=count,
            timestamp_us=timestamp_us,
            interval_us=interval_us,
            heading_deg=tail_heading,
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
            heading_deg=tail_heading,
            tas_true_m_s=nominal_tas,
            wind_ned_m_s=nominal_wind,
            source=source,
            rng=rng,
        )
    else:
        injection_start = timestamp_us
        count = int(protocol["stream"]["persistent_injection_observations"])
        injected_timestamps = {timestamp_us + index * interval_us for index in range(count)}
        injected_wind = nominal_wind.copy()
        tas_offset = 0.0
        if kind == "tas_offset":
            tas_offset = float(injection["value"])
        elif kind == "horizontal_wind_step":
            injected_wind[0] += float(injection["value"])
        elif kind == "vertical_wind":
            injected_wind[2] = float(injection["value"])
        timestamp_us = append_observations(
            observations,
            count=count,
            timestamp_us=timestamp_us,
            interval_us=interval_us,
            heading_deg=tail_heading,
            tas_true_m_s=nominal_tas,
            wind_ned_m_s=injected_wind,
            source=source,
            rng=rng,
            tas_measurement_offset_m_s=tas_offset,
        )
    return observations, injection_start, injected_timestamps, observations[-1].arrival_timestamp_us, common_prefix_sha256


def evaluate_case(
    case: dict[str, Any], protocol: dict[str, Any], base_protocol: dict[str, Any], *, synthetic_seed: int,
) -> dict[str, object]:
    observations, injection_start, injected_timestamps, final_now_us, prefix_sha256 = build_case_stream(
        case, protocol, base_protocol, synthetic_seed=synthetic_seed
    )
    shadow_protocol = copy.deepcopy(base_protocol)
    shadow_protocol["causal_oracle"]["maximum_consecutive_post_qualification_innovation_rejections"] = 1_000_000
    shadow = base.CausalWindOracle(shadow_protocol)
    monitor = monitor_from_protocol(protocol)
    source_rejections: Counter[str] = Counter()
    raw_nis_max = 0.0
    raw_nis_count = 0
    pre_injection_eligible = 0
    for observation in observations:
        geometry_qualified_before = shadow._geometry_qualified()  # validation-only introspection
        event = shadow.step(observation)
        if not event.accepted and event.reason != "innovation_nis":
            source_rejections[event.reason] += 1
        if not geometry_qualified_before:
            continue
        if event.innovation_nis is None:
            monitor.reset_without_nis(f"source_without_runtime_nis:{event.reason}")
            continue
        raw_nis_count += 1
        raw_nis_max = max(raw_nis_max, float(event.innovation_nis))
        if injection_start is not None and observation.tas_timestamp_us < injection_start:
            pre_injection_eligible += 1
        monitor.observe(
            observation.arrival_timestamp_us,
            float(event.innovation_nis),
            source_timestamp_us=observation.tas_timestamp_us,
        )
    expected = case["expected"]
    required = case["scoring"] == "required"
    errors: list[str] = []
    latch_delay_s: float | None = None
    latch_injection_observations = 0
    pre_injection_latch = False
    if monitor.first_latch_timestamp_us is not None and injection_start is not None:
        latch_delay_s = (monitor.first_latch_timestamp_us - injection_start) * 1.0e-6
        pre_injection_latch = monitor.first_latch_timestamp_us < injection_start
    if monitor.latch_episode is not None:
        latch_injection_observations = sum(
            item.source_timestamp_us in injected_timestamps for item in monitor.latch_episode
        )
    if required:
        expected_latch = bool(expected["monitor_latched"])
        if monitor.latched != expected_latch:
            errors.append(f"monitor_latched={monitor.latched} but expected={expected_latch}")
        if pre_injection_latch:
            errors.append("monitor_latched_before_injection")
        if expected_latch:
            minimum = int(expected["minimum_latch_window_injection_observations"])
            if latch_injection_observations < minimum:
                errors.append("latch_episode_has_insufficient_injection_observations")
            maximum_delay = float(expected["maximum_latch_delay_s"])
            if latch_delay_s is None or latch_delay_s > maximum_delay:
                errors.append("monitor_latch_exceeded_deadline")
        minimum_gap_resets = int(expected.get("minimum_inter_observation_gap_resets", 0))
        if int(monitor.reset_reasons["inter_observation_gap"]) < minimum_gap_resets:
            errors.append("missing_required_inter_observation_gap_reset")
    return {
        "name": case["name"],
        "paired_prefix_group": case["paired_prefix_group"],
        "common_prefix_sha256": prefix_sha256,
        "synthetic_seed": synthetic_seed,
        "scoring": case["scoring"],
        "expected_monitor_latched": expected["monitor_latched"],
        "passed": not errors,
        "errors": errors,
        "input_observations": len(observations),
        "raw_post_geometry_nis_observations": raw_nis_count,
        "maximum_raw_post_geometry_nis": raw_nis_max if raw_nis_count else None,
        "pre_injection_eligible_observations": pre_injection_eligible,
        "monitor_eligible_observations": monitor.eligible_observations,
        "monitor_armed": monitor.armed,
        "monitor_latched": monitor.latched,
        "monitor_first_latch_timestamp_us": monitor.first_latch_timestamp_us,
        "monitor_latch_delay_s": latch_delay_s,
        "monitor_latch_episode_injection_observations": latch_injection_observations,
        "monitor_latch_episode_observations": len(monitor.latch_episode or ()),
        "monitor_reset_reasons": dict(sorted(monitor.reset_reasons.items())),
        "trace": list(monitor.trace),
        "source_rejections": dict(sorted(source_rejections.items())),
        "shadow_terminal_status": shadow.terminal_status(final_now_us),
    }


def percentile_summary(values: list[float | int]) -> dict[str, float | None]:
    if not values:
        return {"minimum": None, "p05": None, "p50": None, "p95": None, "maximum": None}
    ordered = sorted(float(value) for value in values)

    def pick(fraction: float) -> float:
        return ordered[int(math.floor(fraction * (len(ordered) - 1)))]

    return {
        "minimum": ordered[0], "p05": pick(0.05), "p50": pick(0.50),
        "p95": pick(0.95), "maximum": ordered[-1],
    }


def summarize_case(items: list[dict[str, object]]) -> dict[str, object]:
    required = str(items[0]["scoring"]) == "required"
    failures = [item for item in items if not bool(item["passed"])]
    latches = sum(bool(item["monitor_latched"]) for item in items)
    return {
        "name": items[0]["name"],
        "scoring": items[0]["scoring"],
        "expected_monitor_latched": items[0]["expected_monitor_latched"],
        "replications": len(items),
        "monitor_latches": latches,
        "monitor_non_latches": len(items) - latches,
        "required_failures": len(failures) if required else 0,
        "descriptive_anomalies": len(failures) if not required else 0,
        "input_observations": sum(int(item["input_observations"]) for item in items),
        "pre_injection_eligible_observations": percentile_summary(
            [int(item["pre_injection_eligible_observations"]) for item in items]
        ),
        "latch_delay_s": percentile_summary(
            [float(item["monitor_latch_delay_s"]) for item in items if item["monitor_latch_delay_s"] is not None]
        ),
        "latch_episode_injection_observations": percentile_summary(
            [int(item["monitor_latch_episode_injection_observations"]) for item in items]
        ),
        "failed_case_records": failures,
    }


def check_common_prefixes(records: list[dict[str, object]]) -> list[dict[str, object]]:
    """Return paired-prefix violations without altering case-level evidence."""

    grouped: dict[tuple[int, str], set[str]] = defaultdict(set)
    names: dict[tuple[int, str], list[str]] = defaultdict(list)
    for record in records:
        key = (int(record["synthetic_seed"]), str(record["paired_prefix_group"]))
        grouped[key].add(str(record["common_prefix_sha256"]))
        names[key].append(str(record["name"]))
    return [
        {
            "synthetic_seed": seed,
            "paired_prefix_group": group,
            "case_names": sorted(case_names),
            "prefix_sha256_values": sorted(grouped[(seed, group)]),
        }
        for (seed, group), case_names in sorted(names.items())
        if len(grouped[(seed, group)]) != 1
    ]


def assemble_result(protocol: dict[str, Any], records: list[dict[str, object]], protocol_path: Path) -> dict[str, object]:
    records.sort(key=lambda item: (str(item["name"]), int(item["synthetic_seed"])))
    groups: dict[str, list[dict[str, object]]] = defaultdict(list)
    for record in records:
        groups[str(record["name"])].append(record)
    summaries = [summarize_case(groups[str(case["name"])]) for case in protocol["case_matrix"]]
    required_failures = [item for item in records if item["scoring"] == "required" and not bool(item["passed"])]
    paired_prefix_failures = check_common_prefixes(records)
    return {
        "schema_version": 1,
        "study_id": protocol["protocol_id"],
        "phase": "development",
        "status": "passed_required_development_checks" if not required_failures and not paired_prefix_failures else "failed_required_development_checks",
        "protocol": {
            "path": str(protocol_path.relative_to(ROOT)),
            "file_sha256": file_sha256(protocol_path),
            "semantic_sha256": base.canonical_sha256(protocol),
        },
        "provenance": {
            "runner_sha256": file_sha256(RUNNER_PATH),
            "base_oracle_runner_sha256": file_sha256(ROOT / "validation" / "run_airspeed_wind_observability.py"),
            "git_commit": capture(["git", "rev-parse", "HEAD"]),
            "git_status": capture(["git", "status", "--short"]),
            "execution_mode": "sequential",
        },
        "monitor": protocol["monitor"],
        "seed_count": len(protocol["seed_sets"]["development"]),
        "selected_seeds": protocol["seed_sets"]["development"],
        "selected_seed_canonical_sha256": base.canonical_sha256(protocol["seed_sets"]["development"]),
        "totals": {
            "replication_cases": len(records),
            "required_failed_replication_cases": len(required_failures),
            "descriptive_replication_cases": sum(item["scoring"] == "descriptive" for item in records),
            "input_observations": sum(int(item["input_observations"]) for item in records),
        },
        "common_prefix_pairing": {
            "status": "passed" if not paired_prefix_failures else "failed",
            "violations": paired_prefix_failures,
        },
        "case_summaries": summaries,
        "required_failed_case_records": required_failures,
        "records": records,
        "limitations": [
            "Opened development evidence only; this is not a sealed holdout and cannot support a release claim.",
            "The monitor is a residual diagnostic, not a source classifier: TAS fault, wind, sideslip, and model mismatch remain causally ambiguous.",
            "No result changes the production 16-nominal/15-error-state ESKF, enables TAS, or authorizes a 17-state wind branch.",
        ],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--protocol", type=Path, default=DEFAULT_PROTOCOL)
    parser.add_argument("--out", type=Path, default=Path("build/airspeed-wind-mismatch-monitor-v5-development.json"))
    args = parser.parse_args()
    protocol_path = args.protocol.resolve()
    protocol = load_protocol(protocol_path)
    base_protocol = resolve_base_protocol(protocol)
    records = [
        evaluate_case(case, protocol, base_protocol, synthetic_seed=seed)
        for seed in protocol["seed_sets"]["development"]
        for case in protocol["case_matrix"]
    ]
    result = assemble_result(protocol, records, protocol_path)
    output = args.out.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["status"] == "passed_required_development_checks" else 1


if __name__ == "__main__":
    raise SystemExit(main())
