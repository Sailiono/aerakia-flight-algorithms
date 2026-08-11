#!/usr/bin/env python3
"""Opened v7 source-time TAS residual-persistence characterization.

This is host-only validation infrastructure.  It does not modify the portable
ESKF, add a TAS API, classify a pitot fault, or authorize a wind-state branch.
"""

from __future__ import annotations

import argparse
import copy
import datetime as dt
import hashlib
import json
import math
import multiprocessing
import os
import platform
import subprocess
import sys
import uuid
from collections import Counter, defaultdict
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

import airspeed_wind_residual_persistence_v7 as monitor_v7
import run_airspeed_wind_observability as base


ROOT = Path(__file__).resolve().parents[1]
RUNNER_PATH = ROOT / "validation" / "run_airspeed_wind_residual_persistence_v7.py"
MONITOR_PATH = ROOT / "validation" / "airspeed_wind_residual_persistence_v7.py"
DEFAULT_PROTOCOL = ROOT / "validation" / "airspeed_wind_residual_persistence_protocol_v7.json"


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


def repository_path(relative_text: str) -> Path:
    relative = Path(relative_text)
    if relative.is_absolute() or ".." in relative.parts:
        raise ValueError("v7 dependency paths must be repository-relative")
    path = (ROOT / relative).resolve()
    if not path.is_relative_to(ROOT):
        raise ValueError("v7 dependency escaped the repository")
    return path


def expand_seed_range(protocol: dict[str, Any], phase: str) -> list[int]:
    if phase not in ("train", "tune"):
        raise ValueError("v7 phase must be train or tune")
    item = protocol["seed_ranges"][phase]
    start = int(item["start"])
    count = int(item["count"])
    if start < 0 or count <= 0:
        raise ValueError("v7 seed ranges require non-negative start and positive count")
    return list(range(start, start + count))


def _range_values(item: dict[str, Any], *, name: str) -> set[int]:
    start = int(item.get("start", -1))
    count = int(item.get("count", 0))
    if start < 0 or count <= 0:
        raise ValueError(f"{name} must have a non-negative start and positive count")
    return set(range(start, start + count))


def load_protocol(path: Path = DEFAULT_PROTOCOL) -> dict[str, Any]:
    protocol = json.loads(path.read_text(encoding="utf-8"))
    if protocol.get("schema_version") != 1:
        raise ValueError("unsupported residual-persistence protocol schema")
    if protocol.get("status") != "opened_train_tune_no_holdout":
        raise ValueError("v7 must remain opened train/tune development with no holdout")
    scope = protocol.get("scope", {})
    if scope.get("production_eskf") != "unchanged_16_nominal_15_error_state":
        raise ValueError("v7 must leave the production ESKF unchanged")
    if scope.get("public_api") != "unchanged_no_tas_or_wind_state":
        raise ValueError("v7 must not add a TAS API or wind state")
    dependencies = protocol.get("dependencies")
    if not isinstance(dependencies, dict):
        raise ValueError("v7 dependency contract is missing")
    for path_key, hash_key in (
        ("base_protocol_path", "base_protocol_file_sha256"),
        ("base_oracle_runner_path", "base_oracle_runner_file_sha256"),
    ):
        if not isinstance(dependencies.get(path_key), str):
            raise ValueError(f"v7 dependency lacks {path_key}")
        expected = dependencies.get(hash_key)
        if not isinstance(expected, str) or len(expected) != 64:
            raise ValueError(f"v7 dependency lacks valid {hash_key}")
        if file_sha256(repository_path(dependencies[path_key])) != expected:
            raise ValueError(f"v7 dependency hash differs for {path_key}")
    evaluator = protocol.get("residual_evaluator")
    if not isinstance(evaluator, dict):
        raise ValueError("v7 residual evaluator contract is missing")
    if evaluator.get("kind") != "frozen_causal_wind_oracle_v1":
        raise ValueError("unsupported v7 residual evaluator")
    if evaluator.get("base_protocol_path") != dependencies.get("base_protocol_path"):
        raise ValueError("v7 evaluator/base protocol paths differ")
    if evaluator.get("base_protocol_sha256") != dependencies.get("base_protocol_file_sha256"):
        raise ValueError("v7 evaluator/base protocol hashes differ")
    overrides = evaluator.get("oracle_overrides")
    if overrides != {"maximum_consecutive_post_qualification_innovation_rejections": 1000000}:
        raise ValueError("v7 evaluator override is not the registered record-only mode")
    if evaluator.get("geometry_gate") != "qualified_before_step":
        raise ValueError("v7 evaluator geometry gate is not the registered causal order")
    if evaluator.get("nis_field") != "innovation_nis":
        raise ValueError("v7 evaluator NIS field is not registered")
    if evaluator.get("feed_events") != ["accepted", "innovation_nis"]:
        raise ValueError("v7 evaluator feed events differ from the registered mapping")
    if evaluator.get("invalid_event_mapping") != {
        "no_nis": "nis_null_source_invalid",
        "source_rejection": "nis_null_source_invalid",
    }:
        raise ValueError("v7 evaluator invalid-event mapping differs")
    if evaluator.get("ordering") != ["arrival_timestamp_us", "tas_timestamp_us"]:
        raise ValueError("v7 evaluator ordering differs from the registered mapping")
    if evaluator.get("truth_or_injection_labels_enter_runtime") is not False:
        raise ValueError("v7 evaluator must reject truth/labels at runtime")
    monitor = protocol.get("monitor")
    required_monitor = (
        "warmup_valid_source_time_s",
        "minimum_warmup_coverage_fraction",
        "initial_quiet_source_time_s",
        "quiet_nis_threshold",
        "high_nis_threshold",
        "minimum_high_episode_source_time_s",
        "minimum_high_observations",
        "source_gap_periods",
        "minimum_source_gap_s",
        "arrival_gap_periods",
        "minimum_arrival_gap_s",
        "pre_trigger_trace_observations",
        "post_trigger_trace_observations",
    )
    if not isinstance(monitor, dict) or any(name not in monitor for name in required_monitor):
        raise ValueError("v7 monitor contract is incomplete")
    fraction = float(monitor["minimum_warmup_coverage_fraction"])
    if not math.isfinite(fraction) or not 0.0 < fraction <= 1.0:
        raise ValueError("v7 warmup coverage fraction must be in (0, 1]")
    for name in required_monitor:
        value = float(monitor[name])
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f"v7 monitor field {name} must be finite and non-negative")
    if float(monitor["high_nis_threshold"]) <= float(monitor["quiet_nis_threshold"]):
        raise ValueError("v7 high NIS threshold must exceed quiet threshold")
    scenario = protocol.get("scenario")
    required_scenario = (
        "monitor_nominal_rate_hz",
        "base_segment_duration_s",
        "pre_injection_duration_s",
        "post_injection_duration_s",
        "tail_heading_deg",
        "gap_before_injection_s",
        "minimum_pre_injection_eligible_source_time_s",
    )
    if not isinstance(scenario, dict) or any(name not in scenario for name in required_scenario):
        raise ValueError("v7 scenario contract is incomplete")
    rate_hz = float(scenario["monitor_nominal_rate_hz"])
    if not math.isfinite(rate_hz) or rate_hz <= 0.0:
        raise ValueError("v7 monitor rate must be positive and finite")
    train = expand_seed_range(protocol, "train")
    tune = expand_seed_range(protocol, "tune")
    if set(train).intersection(tune):
        raise ValueError("v7 train and tune seed ranges overlap")
    retired = protocol.get("retired_seed_ranges")
    if not isinstance(retired, dict) or not retired:
        raise ValueError("v7 retired seed registry is missing")
    retired_values: set[int] = set()
    for name, item in retired.items():
        if not isinstance(item, dict):
            raise ValueError(f"v7 retired range {name} is invalid")
        values = _range_values(item, name=f"retired range {name}")
        if retired_values.intersection(values):
            raise ValueError("v7 retired seed ranges overlap")
        retired_values.update(values)
    if set(train).intersection(retired_values) or set(tune).intersection(retired_values):
        raise ValueError("v7 seed range overlaps a retired range")
    delivery_profiles = protocol.get("delivery_profiles")
    if not isinstance(delivery_profiles, dict) or "aligned_2hz" not in delivery_profiles:
        raise ValueError("v7 delivery profiles are incomplete")
    for profile_name, profile in delivery_profiles.items():
        if not isinstance(profile, dict) or profile.get("kind") not in {"aligned", "bounded_jitter"}:
            raise ValueError(f"v7 delivery profile {profile_name} is unsupported")
        if profile["kind"] == "aligned":
            delay = float(profile.get("fixed_delay_s", -1.0))
            if not math.isfinite(delay) or delay < 0.0:
                raise ValueError(f"v7 delivery profile {profile_name} has invalid fixed delay")
        else:
            for field in ("base_delay_s", "jitter_amplitude_s", "maximum_delay_s"):
                value = float(profile.get(field, -1.0))
                if not math.isfinite(value) or value < 0.0:
                    raise ValueError(f"v7 delivery profile {profile_name} has invalid {field}")
            if float(profile["base_delay_s"]) + float(profile["jitter_amplitude_s"]) > float(profile["maximum_delay_s"]):
                raise ValueError(f"v7 delivery profile {profile_name} exceeds its maximum delay")
    profiles = protocol.get("noise_profiles")
    if not isinstance(profiles, dict) or not profiles:
        raise ValueError("v7 noise profiles are missing")
    for profile_name, profile in profiles.items():
        if not isinstance(profile, dict):
            raise ValueError(f"v7 noise profile {profile_name} is invalid")
        for name in (
            "tas_actual_std_multiplier",
            "gnss_actual_std_multiplier",
            "tas_declared_std_multiplier",
            "gnss_declared_std_multiplier",
        ):
            value = float(profile.get(name, 0.0))
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"v7 noise profile {profile_name} has invalid {name}")
    cases = protocol.get("case_matrix")
    if not isinstance(cases, list) or not cases:
        raise ValueError("v7 case matrix is empty")
    supported_injections = {
        "none",
        "tas_offset_pulse",
        "tas_offset_step",
        "tas_scale_step",
        "horizontal_wind_step",
        "vertical_wind",
    }
    names: set[str] = set()
    for case in cases:
        if not isinstance(case, dict) or not isinstance(case.get("name"), str):
            raise ValueError("every v7 case needs a string name")
        if case["name"] in names:
            raise ValueError("v7 case names must be unique")
        names.add(case["name"])
        if case.get("noise_profile") not in profiles:
            raise ValueError(f"v7 case {case['name']} names an unknown noise profile")
        if not isinstance(case.get("paired_stream_group"), str) or not case["paired_stream_group"]:
            raise ValueError(f"v7 case {case['name']} lacks paired stream group")
        injection = case.get("injection")
        if not isinstance(injection, dict) or injection.get("kind") not in supported_injections:
            raise ValueError(f"v7 case {case['name']} has unsupported injection")
        if case.get("role") not in {
            "nuisance_primary",
            "pulse_curve",
            "persistent_primary",
            "descriptive",
            "structural_gap",
            "coverage_descriptive",
        }:
            raise ValueError(f"v7 case {case['name']} has unsupported role")
        if case["role"] == "persistent_primary":
            deadline = float(case.get("maximum_source_detection_delay_s", -1.0))
            if not math.isfinite(deadline) or deadline <= 0.0:
                raise ValueError(f"v7 persistent case {case['name']} lacks a valid deadline")
        delivery_profile = case.get("delivery_profile", "aligned_2hz")
        if delivery_profile not in delivery_profiles:
            raise ValueError(f"v7 case {case['name']} names an unknown delivery profile")
        paired_null_required = case.get("paired_null_required", True)
        if not isinstance(paired_null_required, bool):
            raise ValueError(f"v7 case {case['name']} paired_null_required must be boolean")
    if "nominal" not in names:
        raise ValueError("v7 needs a nominal paired-null case")
    # Resolve required counterparts during preflight.  Structural transport
    # cases may opt out explicitly, but no other missing pair can survive to
    # the aggregate scorer and fail after a long campaign has finished.
    paired_nulls_by_case(cases)
    scoring = protocol.get("scoring")
    if not isinstance(scoring, dict) or not 0.5 < float(scoring.get("confidence", 0.0)) < 1.0:
        raise ValueError("v7 scoring confidence is invalid")
    return protocol


def resolve_base_protocol(protocol: dict[str, Any]) -> dict[str, Any]:
    path = repository_path(str(protocol["dependencies"]["base_protocol_path"]))
    return base.load_protocol(path)


@dataclass(frozen=True)
class GeneratedStream:
    observations: tuple[base.TasWindObservation, ...]
    injection_start_source_timestamp_us: int
    random_stream_sha256: str
    arrival_schedule_sha256: str
    delivery_profile: str
    nominal_rate_hz: float


def _stream_seed(seed: int, group: str) -> int:
    digest = hashlib.sha256(f"aerakia-v7:{group}".encode("utf-8")).digest()
    return int(seed) ^ int.from_bytes(digest[:8], "big")


def _standard_normal_fingerprint(
    *, seed: int, group: str, rate_hz: float, tas_noise: np.ndarray, gnss_noise: np.ndarray,
) -> str:
    digest = hashlib.sha256()
    digest.update(str(seed).encode("ascii"))
    digest.update(group.encode("utf-8"))
    digest.update(repr(rate_hz).encode("ascii"))
    digest.update(tas_noise.astype("<f8", copy=False).tobytes())
    digest.update(gnss_noise.astype("<f8", copy=False).tobytes())
    return digest.hexdigest()


def _delivery_schedule(
    case: dict[str, Any], protocol: dict[str, Any], *, seed: int,
    source_timestamps_us: list[int],
) -> tuple[list[int], str]:
    """Create a deterministic arrival schedule without changing source data."""

    profile_name = str(case.get("delivery_profile", "aligned_2hz"))
    profile = protocol["delivery_profiles"][profile_name]
    kind = str(profile["kind"])
    source = np.asarray(source_timestamps_us, dtype=np.int64)
    if kind == "aligned":
        delays = np.full(len(source), int(round(float(profile["fixed_delay_s"]) * 1.0e6)), dtype=np.int64)
    else:
        digest = hashlib.sha256(f"aerakia-v7-arrival:{seed}:{profile_name}".encode()).digest()
        rng = np.random.default_rng(int.from_bytes(digest[:8], "big"))
        if kind == "bounded_jitter":
            base_delay = float(profile["base_delay_s"])
            amplitude = float(profile["jitter_amplitude_s"])
            delays = np.rint((base_delay + rng.uniform(-amplitude, amplitude, len(source))) * 1.0e6).astype(np.int64)
        elif kind == "monotonic_burst":
            base_delay = int(round(float(profile["base_delay_s"]) * 1.0e6))
            burst_size = int(profile["burst_size"])
            delays = np.asarray(
                [base_delay + (index // max(1, burst_size)) * 1000 for index in range(len(source))],
                dtype=np.int64,
            )
        else:
            raise ValueError(f"unsupported v7 delivery profile {profile_name}")
        maximum_delay = int(round(float(profile["maximum_delay_s"]) * 1.0e6))
        if np.any(delays < 0) or np.any(delays > maximum_delay):
            raise ValueError(f"delivery profile {profile_name} exceeded its delay bound")
    arrivals = source + delays
    if np.any(np.diff(arrivals) <= 0):
        raise ValueError(f"delivery profile {profile_name} is not strictly monotonic")
    digest = hashlib.sha256()
    digest.update(profile_name.encode("utf-8"))
    digest.update(arrivals.astype("<i8", copy=False).tobytes())
    return [int(value) for value in arrivals], digest.hexdigest()


def generate_case_stream(
    case: dict[str, Any], protocol: dict[str, Any], base_protocol: dict[str, Any], *, seed: int,
) -> GeneratedStream:
    """Generate one fully paired stream without importing the v4/v5 helpers."""

    scenario = protocol["scenario"]
    rate_hz = float(scenario["monitor_nominal_rate_hz"])
    interval_us = int(round(1.0e6 / rate_hz))
    base_duration_s = 3.0 * float(scenario["base_segment_duration_s"])
    injection_nominal_s = base_duration_s + float(scenario["pre_injection_duration_s"])
    total_duration_s = injection_nominal_s + float(scenario["post_injection_duration_s"])
    count = int(round(total_duration_s * rate_hz))
    if count <= 0:
        raise ValueError("v7 stream has no observations")
    group = str(case["paired_stream_group"])
    rng = np.random.default_rng(_stream_seed(seed, group))
    tas_standard_noise = rng.normal(0.0, 1.0, count)
    gnss_standard_noise = rng.normal(0.0, 1.0, (count, 3))
    fingerprint = _standard_normal_fingerprint(
        seed=seed,
        group=group,
        rate_hz=rate_hz,
        tas_noise=tas_standard_noise,
        gnss_noise=gnss_standard_noise,
    )
    base_source = base_protocol["synthetic_source"]
    profile = protocol["noise_profiles"][case["noise_profile"]]
    tas_nominal_std = float(base_source["tas_noise_std_m_s"])
    gnss_nominal_std = float(base_source["gnss_velocity_noise_std_m_s"])
    tas_actual_std = tas_nominal_std * float(profile["tas_actual_std_multiplier"])
    gnss_actual_std = gnss_nominal_std * float(profile["gnss_actual_std_multiplier"])
    tas_declared_std = tas_nominal_std * float(profile["tas_declared_std_multiplier"])
    gnss_declared_std = gnss_nominal_std * float(profile["gnss_declared_std_multiplier"])
    nominal_tas = float(base_source["nominal_tas_m_s"])
    nominal_wind_ne = np.asarray(base_source["nominal_wind_ne_m_s"], dtype=np.float64)
    injection = case["injection"]
    source_gap = bool(injection.get("source_gap_before", False))
    gap_us = int(round(float(scenario["gap_before_injection_s"]) * 1.0e6)) if source_gap else 0
    injection_index = int(round(injection_nominal_s * rate_hz))
    injection_start_source_us = injection_index * interval_us + gap_us
    observations: list[base.TasWindObservation] = []
    source_timestamps: list[int] = []
    segment_s = float(scenario["base_segment_duration_s"])
    for index in range(count):
        nominal_time_s = index / rate_hz
        source_timestamp_us = index * interval_us + (gap_us if index >= injection_index else 0)
        if nominal_time_s < segment_s:
            heading_deg = 0.0
        elif nominal_time_s < 2.0 * segment_s:
            heading_deg = 90.0
        elif nominal_time_s < 3.0 * segment_s:
            heading_deg = 225.0
        else:
            heading_deg = float(scenario["tail_heading_deg"])
        heading_rad = math.radians(heading_deg)
        air_velocity = np.asarray(
            (nominal_tas * math.cos(heading_rad), nominal_tas * math.sin(heading_rad), 0.0),
            dtype=np.float64,
        )
        wind_ned = np.asarray((nominal_wind_ne[0], nominal_wind_ne[1], 0.0), dtype=np.float64)
        after_injection = index >= injection_index
        kind = str(injection["kind"])
        if after_injection and kind == "horizontal_wind_step":
            wind_ned[0] += float(injection["north_delta_m_s"])
        elif after_injection and kind == "vertical_wind":
            wind_ned[2] = float(injection["down_m_s"])
        measured_tas = nominal_tas + float(tas_standard_noise[index]) * tas_actual_std
        if after_injection and kind == "tas_offset_step":
            measured_tas += float(injection["value_m_s"])
        elif after_injection and kind == "tas_scale_step":
            measured_tas *= float(injection["factor"])
        elif after_injection and kind == "tas_offset_pulse":
            elapsed_s = nominal_time_s - injection_nominal_s
            if elapsed_s < float(injection["duration_s"]):
                measured_tas += float(injection["value_m_s"])
        ground_velocity = air_velocity + wind_ned
        measured_velocity = ground_velocity + gnss_standard_noise[index] * gnss_actual_std
        observations.append(
            base.TasWindObservation(
                tas_timestamp_us=source_timestamp_us,
                arrival_timestamp_us=source_timestamp_us,
                gnss_timestamp_us=source_timestamp_us,
                tas_m_s=float(measured_tas),
                tas_variance_m2_s2=tas_declared_std * tas_declared_std,
                gnss_velocity_ned_m_s=tuple(float(value) for value in measured_velocity),
                gnss_velocity_variance_m2_s2=(gnss_declared_std * gnss_declared_std,) * 3,
                flight_regime="fixed_wing_cruise",
            )
        )
        source_timestamps.append(source_timestamp_us)
    arrivals, arrival_fingerprint = _delivery_schedule(
        case, protocol, seed=seed, source_timestamps_us=source_timestamps,
    )
    observations = [
        base.TasWindObservation(
            tas_timestamp_us=item.tas_timestamp_us,
            arrival_timestamp_us=arrivals[index],
            gnss_timestamp_us=item.gnss_timestamp_us,
            tas_m_s=item.tas_m_s,
            tas_variance_m2_s2=item.tas_variance_m2_s2,
            gnss_velocity_ned_m_s=item.gnss_velocity_ned_m_s,
            gnss_velocity_variance_m2_s2=item.gnss_velocity_variance_m2_s2,
            flight_regime=item.flight_regime,
            source_qualified=item.source_qualified,
            pitot_blocked=item.pitot_blocked,
            pitot_stalled=item.pitot_stalled,
            rotor_wash=item.rotor_wash,
            sideslip_qualified=item.sideslip_qualified,
        )
        for index, item in enumerate(observations)
    ]
    return GeneratedStream(
        observations=tuple(observations),
        injection_start_source_timestamp_us=injection_start_source_us,
        random_stream_sha256=fingerprint,
        arrival_schedule_sha256=arrival_fingerprint,
        delivery_profile=str(case.get("delivery_profile", "aligned_2hz")),
        nominal_rate_hz=rate_hz,
    )


def _monitor_from_protocol(
    protocol: dict[str, Any], rate_hz: float,
) -> monitor_v7.SourceTimeResidualPersistenceMonitor:
    config = protocol["monitor"]
    warmup_observations = int(
        math.ceil(
            float(config["warmup_valid_source_time_s"])
            * rate_hz
            * float(config["minimum_warmup_coverage_fraction"])
        )
    )
    source_gap_s = max(
        float(config["minimum_source_gap_s"]),
        float(config["source_gap_periods"]) / rate_hz,
    )
    arrival_gap_s = max(
        float(config["minimum_arrival_gap_s"]),
        float(config["arrival_gap_periods"]) / rate_hz,
    )
    return monitor_v7.SourceTimeResidualPersistenceMonitor(
        monitor_v7.ResidualPersistenceConfig(
            warmup_min_valid_observations=warmup_observations,
            warmup_min_source_span_s=float(config["warmup_valid_source_time_s"]),
            quiet_confirmation_min_observations=2,
            quiet_confirmation_min_source_span_s=float(config["initial_quiet_source_time_s"]),
            quiet_nis_threshold=float(config["quiet_nis_threshold"]),
            high_nis_threshold=float(config["high_nis_threshold"]),
            high_episode_min_observations=int(config["minimum_high_observations"]),
            high_episode_min_source_span_s=float(config["minimum_high_episode_source_time_s"]),
            maximum_source_gap_s=source_gap_s,
            maximum_arrival_gap_s=arrival_gap_s,
            pre_trigger_trace_capacity=int(config["pre_trigger_trace_observations"]),
            post_latch_tail_capacity=int(config["post_trigger_trace_observations"]),
        )
    )


def _trace_event_payload(event: monitor_v7.TraceEvent) -> dict[str, object]:
    return {
        "input_index": event.input_index,
        "source_epoch": event.source_epoch,
        "source_timestamp_us": event.source_timestamp_us,
        "arrival_timestamp_us": event.arrival_timestamp_us,
        "nis": event.nis,
        "source_valid": event.source_valid,
        "state_before": event.state_before.value,
        "state_after": event.state_after.value,
        "accepted_residual": event.accepted_residual,
        "contributed_to_high_episode": event.contributed_to_high_episode,
        "reason": event.reason,
    }


def _trigger_payload(snapshot: monitor_v7.TriggerSnapshot | None) -> dict[str, object] | None:
    if snapshot is None:
        return None
    return {
        "source_epoch": snapshot.source_epoch,
        "latch_source_timestamp_us": snapshot.latch_source_timestamp_us,
        "latch_arrival_timestamp_us": snapshot.latch_arrival_timestamp_us,
        "episode_start_source_timestamp_us": snapshot.episode_start_source_timestamp_us,
        "episode_start_arrival_timestamp_us": snapshot.episode_start_arrival_timestamp_us,
        "quiet_boundary_source_timestamp_us": snapshot.quiet_boundary_source_timestamp_us,
        "quiet_boundary_arrival_timestamp_us": snapshot.quiet_boundary_arrival_timestamp_us,
        "episode_source_span_s": snapshot.episode_source_span_s,
        "episode_observations": [
            {
                "source_epoch": point.source_epoch,
                "source_timestamp_us": point.source_timestamp_us,
                "arrival_timestamp_us": point.arrival_timestamp_us,
                "nis": point.nis,
            }
            for point in snapshot.episode_observations
        ],
        "trace": [_trace_event_payload(event) for event in snapshot.trace],
    }


def _monitor_summary(
    monitor: monitor_v7.SourceTimeResidualPersistenceMonitor,
    *,
    pre_injection_active_source_time_s: float,
    trigger_snapshot_initial_sha256: str | None,
) -> dict[str, object]:
    trigger = _trigger_payload(monitor.trigger_snapshot)
    final_trigger_sha = None if trigger is None else base.canonical_sha256(trigger)
    telemetry = monitor.telemetry()
    continuity = telemetry["continuity"]
    terminal = telemetry["terminal"]
    snapshot_immutable = (
        trigger_snapshot_initial_sha256 is None
        or final_trigger_sha == trigger_snapshot_initial_sha256
    )
    return {
        "state": monitor.state.value,
        "armed": monitor.warmup_ready,
        "latched": monitor.state is monitor_v7.MonitorState.LATCHED,
        "first_latch_source_timestamp_us": (
            None if monitor.trigger_snapshot is None else monitor.trigger_snapshot.latch_source_timestamp_us
        ),
        "first_latch_arrival_timestamp_us": (
            None if monitor.trigger_snapshot is None else monitor.trigger_snapshot.latch_arrival_timestamp_us
        ),
        "episode_start_source_timestamp_us": (
            None if monitor.trigger_snapshot is None else monitor.trigger_snapshot.episode_start_source_timestamp_us
        ),
        "episode_start_arrival_timestamp_us": (
            None if monitor.trigger_snapshot is None else monitor.trigger_snapshot.episode_start_arrival_timestamp_us
        ),
        "pre_injection_active_source_time_s": pre_injection_active_source_time_s,
        "source_gap_events": int(continuity["source_gap_events"]),
        "arrival_gap_events": int(continuity["arrival_gap_events"]),
        "gap_resets_empty_episode": int(continuity["gap_resets_empty_episode"]),
        "gap_resets_nonempty_episode": int(continuity["gap_resets_nonempty_episode"]),
        "trigger_snapshot_immutable": snapshot_immutable,
        "trigger_snapshot_sha256": final_trigger_sha,
        "trigger_snapshot": trigger,
        "post_latch_tail": [_trace_event_payload(event) for event in monitor.post_latch_tail],
        "post_latch_events": int(terminal["post_latch_events"]),
        "telemetry": telemetry,
    }


def build_residual_evaluator(
    protocol: dict[str, Any], base_protocol: dict[str, Any],
) -> tuple[base.CausalWindOracle, str]:
    """Build the sole v7 residual provider from the registered config."""

    evaluator = protocol["residual_evaluator"]
    effective_protocol = copy.deepcopy(base_protocol)
    effective_protocol["causal_oracle"].update(evaluator["oracle_overrides"])
    effective_spec = {
        "kind": evaluator["kind"],
        "base_protocol_sha256": evaluator["base_protocol_sha256"],
        "oracle_overrides": evaluator["oracle_overrides"],
        "geometry_gate": evaluator["geometry_gate"],
        "nis_field": evaluator["nis_field"],
        "feed_events": evaluator["feed_events"],
        "invalid_event_mapping": evaluator["invalid_event_mapping"],
        "ordering": evaluator["ordering"],
        "truth_or_injection_labels_enter_runtime": evaluator[
            "truth_or_injection_labels_enter_runtime"
        ],
    }
    return base.CausalWindOracle(effective_protocol), base.canonical_sha256(effective_spec)


def evaluator_geometry_before_step(shadow: base.CausalWindOracle) -> bool:
    """Single adapter boundary for the validation-only geometry gate."""

    return bool(shadow._geometry_qualified())  # validation-only adapter boundary


def evaluate_case(
    case: dict[str, Any], protocol: dict[str, Any], base_protocol: dict[str, Any], *, seed: int,
) -> dict[str, object]:
    stream = generate_case_stream(case, protocol, base_protocol, seed=seed)
    shadow, evaluator_sha256 = build_residual_evaluator(protocol, base_protocol)
    monitor = _monitor_from_protocol(protocol, stream.nominal_rate_hz)
    monitor.reauthorize(0)
    generated = len(stream.observations)
    source_valid = 0
    post_geometry = 0
    nis_fed = 0
    source_rejections: Counter[str] = Counter()
    geometry_was_qualified = False
    active_segment_start_us: int | None = None
    active_segment_last_us: int | None = None
    maximum_pre_injection_active_span_us = 0
    pre_injection_first_eligible_source_us: int | None = None
    pre_injection_last_eligible_source_us: int | None = None
    trigger_snapshot_initial_sha256: str | None = None
    ordered = sorted(
        stream.observations,
        key=lambda item: (item.arrival_timestamp_us, item.tas_timestamp_us),
    )
    for observation in ordered:
        geometry_before = evaluator_geometry_before_step(shadow)
        event = shadow.step(observation)
        if event.reason not in ("accepted", "innovation_nis"):
            source_rejections[event.reason] += 1
        else:
            source_valid += 1
        if not geometry_before:
            continue
        geometry_was_qualified = True
        post_geometry += 1
        if observation.tas_timestamp_us < stream.injection_start_source_timestamp_us:
            if pre_injection_first_eligible_source_us is None:
                pre_injection_first_eligible_source_us = observation.tas_timestamp_us
            pre_injection_last_eligible_source_us = observation.tas_timestamp_us
        if event.innovation_nis is None:
            decision = monitor.observe(
                arrival_timestamp_us=observation.arrival_timestamp_us,
                source_timestamp_us=observation.tas_timestamp_us,
                nis=None,
                source_valid=False,
                source_epoch=0,
            )
        else:
            nis_fed += 1
            decision = monitor.observe(
                arrival_timestamp_us=observation.arrival_timestamp_us,
                source_timestamp_us=observation.tas_timestamp_us,
                nis=float(event.innovation_nis),
                source_valid=True,
                source_epoch=0,
            )
        if observation.tas_timestamp_us < stream.injection_start_source_timestamp_us:
            active = decision.state in {
                monitor_v7.MonitorState.QUIET_CONFIRMED,
                monitor_v7.MonitorState.HIGH_EPISODE,
                monitor_v7.MonitorState.LATCHED,
            }
            if active:
                if active_segment_start_us is None:
                    active_segment_start_us = observation.tas_timestamp_us
                active_segment_last_us = observation.tas_timestamp_us
                maximum_pre_injection_active_span_us = max(
                    maximum_pre_injection_active_span_us,
                    active_segment_last_us - active_segment_start_us,
                )
            else:
                active_segment_start_us = None
                active_segment_last_us = None
        if monitor.trigger_snapshot is not None and trigger_snapshot_initial_sha256 is None:
            trigger_snapshot_initial_sha256 = base.canonical_sha256(
                _trigger_payload(monitor.trigger_snapshot)
            )
    summary = _monitor_summary(
        monitor,
        pre_injection_active_source_time_s=maximum_pre_injection_active_span_us * 1.0e-6,
        trigger_snapshot_initial_sha256=trigger_snapshot_initial_sha256,
    )
    eligible_span_s = (
        0.0
        if pre_injection_first_eligible_source_us is None
        or pre_injection_last_eligible_source_us is None
        else (pre_injection_last_eligible_source_us - pre_injection_first_eligible_source_us) * 1.0e-6
    )
    latch_source = summary["first_latch_source_timestamp_us"]
    latch_arrival = summary["first_latch_arrival_timestamp_us"]
    source_delay_s = (
        None
        if latch_source is None
        else (int(latch_source) - stream.injection_start_source_timestamp_us) * 1.0e-6
    )
    decision_latency_s = (
        None
        if latch_arrival is None
        else (int(latch_arrival) - stream.injection_start_source_timestamp_us) * 1.0e-6
    )
    pre_injection_latch = latch_source is not None and int(latch_source) < stream.injection_start_source_timestamp_us
    trigger = summary["trigger_snapshot"]
    episode_start = None if trigger is None else int(trigger["episode_start_source_timestamp_us"])
    trigger_points = [] if trigger is None else trigger["episode_observations"]
    if latch_source is None:
        attribution_status = "not_latched"
    elif pre_injection_latch or episode_start is None or episode_start < stream.injection_start_source_timestamp_us:
        attribution_status = "preexisting_episode_ambiguous"
    elif any(int(point["source_timestamp_us"]) < stream.injection_start_source_timestamp_us for point in trigger_points):
        attribution_status = "preexisting_episode_ambiguous"
    else:
        attribution_status = "post_injection_attributed"
    return {
        "name": case["name"],
        "role": case["role"],
        "paired_stream_group": case["paired_stream_group"],
        "noise_profile": case["noise_profile"],
        "synthetic_seed": seed,
        "random_stream_sha256": stream.random_stream_sha256,
        "arrival_schedule_sha256": stream.arrival_schedule_sha256,
        "delivery_profile": stream.delivery_profile,
        "residual_evaluator_sha256": evaluator_sha256,
        "injection_start_source_timestamp_us": stream.injection_start_source_timestamp_us,
        "monitor_latched": bool(summary["latched"]),
        "pre_injection_latch": pre_injection_latch,
        "pre_injection_eligible_source_time_s": eligible_span_s,
        "episode_start_source_timestamp_us": episode_start,
        "attribution_status": attribution_status,
        "source_detection_delay_s": source_delay_s,
        "arrival_decision_latency_s": decision_latency_s,
        "generated_observations": generated,
        "source_valid_observations": source_valid,
        "post_geometry_observations": post_geometry,
        "nis_fed_observations": nis_fed,
        "geometry_qualified": geometry_was_qualified,
        "unexpected_source_rejections": dict(sorted(source_rejections.items())),
        "monitor": summary,
        "shadow_terminal_status": shadow.terminal_status(ordered[-1].arrival_timestamp_us),
    }


def evaluate_task(
    task: tuple[dict[str, Any], dict[str, Any], dict[str, Any], int],
) -> dict[str, object]:
    case, protocol, base_protocol, seed = task
    return evaluate_case(case, protocol, base_protocol, seed=seed)


def run_records(
    protocol: dict[str, Any], base_protocol: dict[str, Any], *, phase: str, jobs: int,
) -> list[dict[str, object]]:
    if jobs <= 0:
        raise ValueError("v7 jobs must be positive")
    tasks = [
        (case, protocol, base_protocol, seed)
        for seed in expand_seed_range(protocol, phase)
        for case in protocol["case_matrix"]
    ]
    if jobs == 1:
        return [evaluate_task(task) for task in tasks]
    methods = multiprocessing.get_all_start_methods()
    context = multiprocessing.get_context("fork") if "fork" in methods else None
    with ProcessPoolExecutor(max_workers=jobs, mp_context=context) as executor:
        return list(executor.map(evaluate_task, tasks))


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    return ordered[int(math.floor(fraction * (len(ordered) - 1)))]


def ceil_order_statistic(values: list[float], fraction: float) -> float | None:
    """Pre-registered 1-indexed order statistic used by primary gates."""

    if not values:
        return None
    ordered = sorted(values)
    rank = max(1, int(math.ceil(fraction * len(ordered))))
    return ordered[rank - 1]


def binomial_cdf(events: int, trials: int, probability: float) -> float:
    if trials < 0 or not 0 <= events <= trials or not 0.0 <= probability <= 1.0:
        raise ValueError("invalid binomial CDF arguments")
    if events == trials:
        return 1.0
    if probability == 0.0:
        return 1.0
    if probability == 1.0:
        return 0.0
    return float(
        sum(
            math.comb(trials, index)
            * math.pow(probability, index)
            * math.pow(1.0 - probability, trials - index)
            for index in range(events + 1)
        )
    )


def clopper_pearson_upper(events: int, trials: int, confidence: float) -> float:
    if trials <= 0 or not 0 <= events <= trials or not 0.0 < confidence < 1.0:
        raise ValueError("invalid Clopper-Pearson upper-bound arguments")
    if events == trials:
        return 1.0
    target = 1.0 - confidence
    low = events / trials
    high = 1.0
    for _ in range(100):
        middle = 0.5 * (low + high)
        if binomial_cdf(events, trials, middle) > target:
            low = middle
        else:
            high = middle
    return high


def clopper_pearson_lower(events: int, trials: int, confidence: float) -> float:
    if trials <= 0 or not 0 <= events <= trials or not 0.0 < confidence < 1.0:
        raise ValueError("invalid Clopper-Pearson lower-bound arguments")
    if events == 0:
        return 0.0
    target = confidence
    low = 0.0
    high = events / trials
    for _ in range(100):
        middle = 0.5 * (low + high)
        if binomial_cdf(events - 1, trials, middle) > target:
            low = middle
        else:
            high = middle
    return high


def summarize_case(items: list[dict[str, object]]) -> dict[str, object]:
    latches = [item for item in items if bool(item["monitor_latched"])]
    delays = [
        float(item["source_detection_delay_s"])
        for item in latches
        if item["source_detection_delay_s"] is not None
    ]
    clean = [item for item in latches if not bool(item["pre_injection_latch"])]
    return {
        "name": items[0]["name"],
        "role": items[0]["role"],
        "replications": len(items),
        "monitor_latches": len(latches),
        "pre_injection_latches": sum(bool(item["pre_injection_latch"]) for item in items),
        "clean_post_injection_latches": len(clean),
        "geometry_qualified_replications": sum(bool(item["geometry_qualified"]) for item in items),
        "monitor_armed_replications": sum(bool(item["monitor"]["armed"]) for item in items),
        "source_detection_delay_s": {
            "minimum": min(delays) if delays else None,
            "p50": percentile(delays, 0.50),
            "p95": percentile(delays, 0.95),
            "maximum": max(delays) if delays else None,
        },
        "generated_observations": sum(int(item["generated_observations"]) for item in items),
        "nis_fed_observations": sum(int(item["nis_fed_observations"]) for item in items),
    }


def check_common_streams(records: list[dict[str, object]]) -> list[dict[str, object]]:
    grouped: dict[tuple[int, str], set[str]] = defaultdict(set)
    names: dict[tuple[int, str], list[str]] = defaultdict(list)
    for record in records:
        key = (int(record["synthetic_seed"]), str(record["paired_stream_group"]))
        grouped[key].add(str(record["random_stream_sha256"]))
        names[key].append(str(record["name"]))
    return [
        {
            "synthetic_seed": seed,
            "paired_stream_group": group,
            "case_names": sorted(names[(seed, group)]),
            "fingerprints": sorted(fingerprints),
        }
        for (seed, group), fingerprints in sorted(grouped.items())
        if len(fingerprints) != 1
    ]


def _record_map(records: list[dict[str, object]]) -> dict[tuple[int, str], dict[str, object]]:
    result: dict[tuple[int, str], dict[str, object]] = {}
    for record in records:
        key = (int(record["synthetic_seed"]), str(record["name"]))
        if key in result:
            raise ValueError("v7 records contain duplicate seed/case keys")
        result[key] = record
    return result


def paired_null_name(case: dict[str, Any], cases: list[dict[str, Any]]) -> str:
    """Find the registered null with matching source/noise/delivery semantics."""

    delivery_profile = case.get("delivery_profile", "aligned_2hz")
    candidates = [
        str(candidate["name"])
        for candidate in cases
        if candidate["injection"]["kind"] == "none"
        and candidate["paired_stream_group"] == case["paired_stream_group"]
        and candidate["noise_profile"] == case["noise_profile"]
        and candidate.get("delivery_profile", "aligned_2hz") == delivery_profile
        and candidate["role"] == "nuisance_primary"
    ]
    if len(candidates) != 1:
        raise ValueError(f"v7 case {case['name']} lacks exactly one paired null")
    return candidates[0]


def paired_nulls_by_case(cases: list[dict[str, Any]]) -> dict[str, str]:
    """Resolve only pairs explicitly required by the frozen case matrix.

    The gap structural case verifies fail-closed transport behavior rather than
    differential residual detection, so it intentionally has no nominal
    counterpart.  That exception is auditable only when represented by an
    explicit ``paired_null_required: false`` protocol field.
    """

    result: dict[str, str] = {}
    for case in cases:
        if case["injection"]["kind"] == "none" or not bool(case.get("paired_null_required", True)):
            continue
        result[str(case["name"])] = paired_null_name(case, cases)
    return result


def compact_failure_record(record: dict[str, object]) -> dict[str, object]:
    monitor = record["monitor"]
    return {
        "name": record["name"],
        "role": record["role"],
        "synthetic_seed": record["synthetic_seed"],
        "monitor_latched": record["monitor_latched"],
        "pre_injection_latch": record["pre_injection_latch"],
        "episode_start_source_timestamp_us": record["episode_start_source_timestamp_us"],
        "attribution_status": record["attribution_status"],
        "source_detection_delay_s": record["source_detection_delay_s"],
        "pre_injection_eligible_source_time_s": record["pre_injection_eligible_source_time_s"],
        "arrival_decision_latency_s": record["arrival_decision_latency_s"],
        "geometry_qualified": record["geometry_qualified"],
        "generated_observations": record["generated_observations"],
        "post_geometry_observations": record["post_geometry_observations"],
        "nis_fed_observations": record["nis_fed_observations"],
        "unexpected_source_rejections": record["unexpected_source_rejections"],
        "pre_injection_active_source_time_s": monitor["pre_injection_active_source_time_s"],
        "source_gap_events": monitor["source_gap_events"],
        "arrival_gap_events": monitor["arrival_gap_events"],
        "trigger_snapshot_sha256": monitor["trigger_snapshot_sha256"],
        "trigger_snapshot": monitor["trigger_snapshot"],
    }


def assemble_result(
    protocol: dict[str, Any],
    records: list[dict[str, object]],
    *,
    phase: str,
    protocol_path: Path,
    jobs: int,
    include_records: bool,
) -> dict[str, object]:
    records.sort(key=lambda item: (str(item["name"]), int(item["synthetic_seed"])))
    seeds = expand_seed_range(protocol, phase)
    grouped: dict[str, list[dict[str, object]]] = defaultdict(list)
    for record in records:
        grouped[str(record["name"])].append(record)
    case_summaries = [summarize_case(grouped[str(case["name"])]) for case in protocol["case_matrix"]]
    by_key = _record_map(records)
    summaries_by_name = {str(item["name"]): item for item in case_summaries}
    null_by_case = paired_nulls_by_case(protocol["case_matrix"])
    for case in protocol["case_matrix"]:
        name = str(case["name"])
        if name not in null_by_case:
            continue
        null_name = null_by_case[name]
        attributable = 0
        paired_null_not_later = 0
        for seed in seeds:
            record = by_key[(seed, name)]
            if not bool(record["monitor_latched"]):
                continue
            nominal = by_key[(seed, null_name)]
            record_latch = record["monitor"]["first_latch_source_timestamp_us"]
            nominal_latch = nominal["monitor"]["first_latch_source_timestamp_us"]
            if nominal_latch is None or int(record_latch) < int(nominal_latch):
                attributable += 1
            else:
                paired_null_not_later += 1
        summaries_by_name[name]["paired_attributable_latches"] = attributable
        summaries_by_name[name]["paired_null_latched_not_later"] = paired_null_not_later
    confidence = float(protocol["scoring"]["confidence"])
    nuisance_names = {
        str(case["name"])
        for case in protocol["case_matrix"]
        if case["role"] in set(protocol["scoring"]["nuisance_family"]["case_roles"])
    }
    nuisance_failed_seeds = [
        seed
        for seed in seeds
        if any(bool(by_key[(seed, name)]["monitor_latched"]) for name in nuisance_names)
    ]
    nuisance_trials = len(seeds)
    nuisance_events = len(nuisance_failed_seeds)
    nuisance_point = nuisance_events / nuisance_trials
    nuisance_upper = clopper_pearson_upper(nuisance_events, nuisance_trials, confidence)
    nuisance_contract = protocol["scoring"]["nuisance_family"]
    nuisance_passed = (
        nuisance_point <= float(nuisance_contract["maximum_point_false_latch_rate"])
        and nuisance_upper <= float(nuisance_contract["maximum_one_sided_clopper_pearson_upper_rate"])
    )
    persistent_cases = [
        case for case in protocol["case_matrix"] if case["role"] == protocol["scoring"]["persistent_offset_family"]["case_role"]
    ]
    persistent_failures: list[dict[str, object]] = []
    persistent_successes = 0
    for seed in seeds:
        seed_errors: list[str] = []
        for case in persistent_cases:
            record = by_key[(seed, str(case["name"]))]
            nominal = by_key[(seed, null_by_case[str(case["name"])])]
            if not bool(record["monitor_latched"]):
                seed_errors.append(f"{case['name']}:not_latched")
                continue
            if bool(record["pre_injection_latch"]):
                seed_errors.append(f"{case['name']}:pre_injection_latch")
            if record["attribution_status"] != "post_injection_attributed":
                seed_errors.append(f"{case['name']}:{record['attribution_status']}")
            record_latch = record["monitor"]["first_latch_source_timestamp_us"]
            nominal_latch = nominal["monitor"]["first_latch_source_timestamp_us"]
            if nominal_latch is not None and int(nominal_latch) <= int(record_latch):
                seed_errors.append(f"{case['name']}:paired_null_latched_not_later")
            delay = record["source_detection_delay_s"]
            if delay is None or float(delay) > float(case["maximum_source_detection_delay_s"]):
                seed_errors.append(f"{case['name']}:deadline")
        if seed_errors:
            persistent_failures.append({"synthetic_seed": seed, "errors": seed_errors})
        else:
            persistent_successes += 1
    persistent_events = len(persistent_failures)
    persistent_trials = len(seeds)
    persistent_point = persistent_events / persistent_trials
    persistent_upper = clopper_pearson_upper(persistent_events, persistent_trials, confidence)
    persistent_contract = protocol["scoring"]["persistent_offset_family"]
    failure_delay_s = max(
        float(case["maximum_source_detection_delay_s"]) for case in persistent_cases
    ) + 0.001
    family_delays: list[float] = []
    for seed in seeds:
        member_delays: list[float] = []
        family_ok = True
        for case in persistent_cases:
            record = by_key[(seed, str(case["name"]))]
            delay = record["source_detection_delay_s"]
            if (
                delay is None
                or record["attribution_status"] != "post_injection_attributed"
                or bool(record["pre_injection_latch"])
                or float(delay) > float(case["maximum_source_detection_delay_s"])
            ):
                family_ok = False
            nominal_latch = by_key[(seed, null_by_case[str(case["name"])])]["monitor"]["first_latch_source_timestamp_us"]
            if nominal_latch is not None and record["monitor"]["first_latch_source_timestamp_us"] is not None:
                if int(nominal_latch) <= int(record["monitor"]["first_latch_source_timestamp_us"]):
                    family_ok = False
            if delay is not None:
                member_delays.append(float(delay))
        family_delays.append(max(member_delays) if family_ok and member_delays else failure_delay_s)
    persistent_delay_p95 = ceil_order_statistic(family_delays, 0.95)
    persistent_latency_passed = (
        persistent_delay_p95 is not None
        and persistent_delay_p95
        <= float(persistent_contract["maximum_family_p95_source_detection_delay_s"])
    )
    persistent_passed = (
        persistent_point <= float(persistent_contract["maximum_point_failure_rate"])
        and persistent_upper <= float(persistent_contract["maximum_one_sided_clopper_pearson_upper_rate"])
        and persistent_latency_passed
    )
    pulse_cases = sorted(
        (
            (float(case["injection"].get("duration_s", math.inf)), str(case["name"]))
            for case in protocol["case_matrix"]
            if case["injection"]["kind"] == "tas_offset_pulse"
            and not bool(case["injection"].get("source_gap_before", False))
        )
    )
    pulse_monotonicity_violations: list[dict[str, object]] = []
    for seed in seeds:
        states = [bool(by_key[(seed, name)]["monitor_latched"]) for _duration, name in pulse_cases]
        if any(states[index] and not states[index + 1] for index in range(len(states) - 1)):
            pulse_monotonicity_violations.append(
                {
                    "synthetic_seed": seed,
                    "states": [
                        {"duration_s": duration, "name": name, "latched": state}
                        for (duration, name), state in zip(pulse_cases, states, strict=True)
                    ],
                }
            )
    minimum_active = float(protocol["scenario"]["minimum_pre_injection_eligible_source_time_s"])
    structural_failures: list[dict[str, object]] = []
    for record in records:
        monitor_summary = record["monitor"]
        reasons: list[str] = []
        if record["role"] not in {"structural_gap", "coverage_descriptive"} and not bool(
            monitor_summary["armed"]
        ):
            reasons.append("monitor_not_armed")
        if record["role"] != "coverage_descriptive" and float(
            record["pre_injection_eligible_source_time_s"]
        ) < minimum_active:
            reasons.append("insufficient_pre_injection_eligible_source_time")
        if record["unexpected_source_rejections"]:
            reasons.append("unexpected_source_rejection")
        if not bool(monitor_summary["trigger_snapshot_immutable"]):
            reasons.append("trigger_snapshot_not_immutable")
        if reasons:
            structural_failures.append(
                {"name": record["name"], "synthetic_seed": record["synthetic_seed"], "reasons": reasons}
            )
    gap_case = next(case for case in protocol["case_matrix"] if case["role"] == "structural_gap")
    for seed in seeds:
        record = by_key[(seed, str(gap_case["name"]))]
        if int(record["monitor"]["source_gap_events"]) < 1 or bool(record["monitor_latched"]):
            structural_failures.append(
                {
                    "name": gap_case["name"],
                    "synthetic_seed": seed,
                    "reasons": ["gap_contract_failed"],
                }
            )
    pairing_failures = check_common_streams(records)
    deterministic_passed = not structural_failures and not pairing_failures and not pulse_monotonicity_violations
    status_passed = nuisance_passed and persistent_passed and deterministic_passed
    retained_keys: set[tuple[int, str]] = set()
    for seed in nuisance_failed_seeds:
        for name in nuisance_names:
            if bool(by_key[(seed, name)]["monitor_latched"]):
                retained_keys.add((seed, name))
    for failure in persistent_failures:
        seed = int(failure["synthetic_seed"])
        for case in persistent_cases:
            retained_keys.add((seed, str(case["name"])))
    for failure in structural_failures:
        retained_keys.add((int(failure["synthetic_seed"]), str(failure["name"])))
    for violation in pulse_monotonicity_violations:
        seed = int(violation["synthetic_seed"])
        for _duration, name in pulse_cases:
            retained_keys.add((seed, name))
    result: dict[str, object] = {
        "schema_version": 1,
        "study_id": protocol["protocol_id"],
        "phase": phase,
        "status": f"passed_{phase}_development_checks" if status_passed else f"failed_{phase}_development_checks",
        "protocol": {
            "path": str(protocol_path.relative_to(ROOT)),
            "file_sha256": file_sha256(protocol_path),
            "semantic_sha256": base.canonical_sha256(protocol),
        },
        "provenance": {
            "runner_sha256": file_sha256(RUNNER_PATH),
            "monitor_module_sha256": file_sha256(MONITOR_PATH),
            "base_oracle_runner_sha256": file_sha256(repository_path(protocol["dependencies"]["base_oracle_runner_path"])),
            "effective_evaluator_sha256": current_evaluator_sha(protocol),
            "git_commit": capture(["git", "rev-parse", "HEAD"]),
            "git_status": capture(["git", "status", "--short"]),
            "jobs": jobs,
            "python_version": sys.version.split()[0],
            "numpy_version": np.__version__,
            "platform": platform.platform(),
        },
        "seed_count": len(seeds),
        "selected_seed_range": {"start": seeds[0], "count": len(seeds), "end": seeds[-1]},
        "selected_seed_canonical_sha256": base.canonical_sha256(seeds),
        "totals": {
            "replication_cases": len(records),
            "generated_observations": sum(int(item["generated_observations"]) for item in records),
            "source_valid_observations": sum(int(item["source_valid_observations"]) for item in records),
            "post_geometry_observations": sum(int(item["post_geometry_observations"]) for item in records),
            "nis_fed_observations": sum(int(item["nis_fed_observations"]) for item in records),
        },
        "primary_endpoints": {
            "nuisance_family_false_latch": {
                "events": nuisance_events,
                "trials": nuisance_trials,
                "point_rate": nuisance_point,
                "one_sided_clopper_pearson_upper": nuisance_upper,
                "confidence": confidence,
                "failed_seeds": nuisance_failed_seeds,
                "passed": nuisance_passed,
            },
            "persistent_offset_family_failure": {
                "events": persistent_events,
                "trials": persistent_trials,
                "successes": persistent_successes,
                "point_rate": persistent_point,
                "one_sided_clopper_pearson_upper": persistent_upper,
                "confidence": confidence,
        "source_detection_delay_p95_s": persistent_delay_p95,
        "family_delay_failure_sentinel": failure_delay_s,
        "family_delays_s": family_delays if len(family_delays) <= 1024 else None,
        "family_delay_order_statistic": "ceil(0.95*N), 1-indexed",
                "latency_gate_passed": persistent_latency_passed,
                "failed_seed_records": persistent_failures,
                "passed": persistent_passed,
            },
        },
        "deterministic_contract": {
            "passed": deterministic_passed,
            "common_random_stream_pairing": {
                "passed": not pairing_failures,
                "violations": pairing_failures,
            },
            "pulse_latch_rate_monotonicity": {
                "passed": not pulse_monotonicity_violations,
                "violations": pulse_monotonicity_violations,
            },
            "coverage_timing_gap_trace": {
                "passed": not structural_failures,
                "failures": structural_failures,
            },
        },
        "case_summaries": case_summaries,
        "retained_failure_records": [
            compact_failure_record(by_key[key]) for key in sorted(retained_keys)
        ],
        "limitations": [
            "Opened synthetic train/tune development evidence only; no v7 holdout exists.",
            "The independent statistical unit is one complete seed/prefix family, not an observation or overlapping window.",
            "A residual episode cannot classify TAS fault versus wind, sideslip, timing, GNSS velocity error, or model mismatch.",
            "The result does not change the production ESKF, Mahony backup, public API, source policy, or control authority.",
        ],
    }
    if include_records:
        result["records"] = records
    return result


def default_output(phase: str) -> Path:
    return ROOT / "validation" / "public" / f"airspeed_wind_residual_persistence_v7_{phase}.json"


def default_claim() -> Path:
    return ROOT / "validation" / "public" / "airspeed_wind_residual_persistence_v7_tune_claim.json"


def default_tune_started() -> Path:
    return ROOT / "validation" / "public" / "airspeed_wind_residual_persistence_v7_tune_started.json"


def git_head_blob(path: Path) -> str | None:
    relative = str(path.relative_to(ROOT))
    return capture(["git", "rev-parse", f"HEAD:{relative}"])


def git_worktree_blob(path: Path) -> str | None:
    return capture(["git", "hash-object", str(path.relative_to(ROOT))])


def current_evaluator_sha(protocol: dict[str, Any]) -> str:
    evaluator = protocol["residual_evaluator"]
    return base.canonical_sha256(
        {
            "kind": evaluator["kind"],
            "base_protocol_sha256": evaluator["base_protocol_sha256"],
            "oracle_overrides": evaluator["oracle_overrides"],
            "geometry_gate": evaluator["geometry_gate"],
            "nis_field": evaluator["nis_field"],
            "feed_events": evaluator["feed_events"],
            "invalid_event_mapping": evaluator["invalid_event_mapping"],
            "ordering": evaluator["ordering"],
            "truth_or_injection_labels_enter_runtime": evaluator[
                "truth_or_injection_labels_enter_runtime"
            ],
        }
    )


def verify_train_artifact(protocol: dict[str, Any]) -> dict[str, Any]:
    train_path = default_output("train")
    if not train_path.is_file():
        raise ValueError("v7 requires a committed train artifact")
    train = json.loads(train_path.read_text(encoding="utf-8"))
    if train.get("phase") != "train" or train.get("status") != "passed_train_development_checks":
        raise ValueError("v7 tune requires a passing train-development record")
    if git_head_blob(train_path) != git_worktree_blob(train_path):
        raise ValueError("v7 train artifact is not the tracked HEAD blob")
    train_commit = train.get("provenance", {}).get("git_commit")
    head_commit = capture(["git", "rev-parse", "HEAD"])
    ancestry = bool(train_commit and head_commit) and subprocess.run(
        ["git", "merge-base", "--is-ancestor", str(train_commit), str(head_commit)],
        cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False,
    ).returncode == 0
    if not ancestry:
        raise ValueError("v7 train provenance commit is not an ancestor of HEAD")
    if train.get("protocol", {}).get("semantic_sha256") != base.canonical_sha256(protocol):
        raise ValueError("v7 tune protocol differs from the train protocol")
    provenance = train.get("provenance", {})
    if provenance.get("runner_sha256") != file_sha256(RUNNER_PATH):
        raise ValueError("v7 tune runner differs from the train runner")
    if provenance.get("monitor_module_sha256") != file_sha256(MONITOR_PATH):
        raise ValueError("v7 tune monitor differs from the train monitor")
    if provenance.get("effective_evaluator_sha256") != current_evaluator_sha(protocol):
        raise ValueError("v7 train evaluator differs from the current evaluator")
    return train


def claim_tune(protocol: dict[str, Any], protocol_path: Path) -> Path:
    if capture(["git", "status", "--short"]) != "":
        raise ValueError("v7 tune claim requires a clean Git worktree")
    train = verify_train_artifact(protocol)
    claim_path = default_claim()
    outcome_path = default_output("tune")
    if claim_path.exists() or outcome_path.exists() or default_tune_started().exists():
        raise ValueError("v7 tune claim/outcome already exists; tune seeds are consumed")
    seeds = expand_seed_range(protocol, "tune")
    claim = {
        "schema_version": 1,
        "claim_type": "v7_single_use_tune",
        "status": "claimed_not_run",
        "created_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "claim_uuid": str(uuid.uuid4()),
        "protocol_path": str(protocol_path.relative_to(ROOT)),
        "protocol_sha256": file_sha256(protocol_path),
        "protocol_semantic_sha256": base.canonical_sha256(protocol),
        "runner_sha256": file_sha256(RUNNER_PATH),
        "monitor_module_sha256": file_sha256(MONITOR_PATH),
        "effective_evaluator_sha256": current_evaluator_sha(protocol),
        "train_artifact_path": str(default_output("train").relative_to(ROOT)),
        "train_artifact_sha256": file_sha256(default_output("train")),
        "train_artifact_head_blob": git_head_blob(default_output("train")),
        "train_commit": train["provenance"]["git_commit"],
        "tune_seed_count": len(seeds),
        "tune_seed_range": {"start": seeds[0], "count": len(seeds), "end": seeds[-1]},
        "tune_seed_canonical_sha256": base.canonical_sha256(seeds),
    }
    payload = (json.dumps(claim, indent=2, sort_keys=True) + "\n").encode("utf-8")
    try:
        descriptor = os.open(claim_path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
    except FileExistsError as error:
        raise ValueError("v7 tune claim already exists; tune seeds are consumed") from error
    with os.fdopen(descriptor, "wb") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())
    return claim_path


def start_tune_execution(protocol: dict[str, Any]) -> Path:
    """Atomically burn the claimed tune run before executing any worker."""

    path = default_tune_started()
    if path.exists():
        raise ValueError("v7 tune already started; the claimed seeds are consumed")
    claim_path = default_claim()
    receipt = {
        "schema_version": 1,
        "receipt_type": "v7_tune_started",
        "status": "started_outcome_pending",
        "created_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "claim_sha256": file_sha256(claim_path),
        "protocol_semantic_sha256": base.canonical_sha256(protocol),
        "runner_sha256": file_sha256(RUNNER_PATH),
        "monitor_module_sha256": file_sha256(MONITOR_PATH),
        "effective_evaluator_sha256": current_evaluator_sha(protocol),
    }
    payload = (json.dumps(receipt, indent=2, sort_keys=True) + "\n").encode("utf-8")
    try:
        descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
    except FileExistsError as error:
        raise ValueError("v7 tune already started; the claimed seeds are consumed") from error
    with os.fdopen(descriptor, "wb") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())
    return path


def verify_tune_prerequisite(protocol: dict[str, Any]) -> None:
    train = verify_train_artifact(protocol)
    claim_path = default_claim()
    if not claim_path.is_file() or git_head_blob(claim_path) != git_worktree_blob(claim_path):
        raise ValueError("v7 tune requires a committed tune claim")
    claim = json.loads(claim_path.read_text(encoding="utf-8"))
    if claim.get("status") != "claimed_not_run":
        raise ValueError("v7 tune claim is not unused")
    if claim.get("train_artifact_sha256") != file_sha256(default_output("train")):
        raise ValueError("v7 tune claim does not match the train artifact")
    if claim.get("protocol_semantic_sha256") != base.canonical_sha256(protocol):
        raise ValueError("v7 tune claim protocol differs")
    protocol_path = ROOT / str(claim.get("protocol_path", ""))
    if not protocol_path.is_file() or claim.get("protocol_sha256") != file_sha256(protocol_path):
        raise ValueError("v7 tune claim protocol file differs")
    if claim.get("runner_sha256") != file_sha256(RUNNER_PATH):
        raise ValueError("v7 tune claim runner differs")
    if claim.get("monitor_module_sha256") != file_sha256(MONITOR_PATH):
        raise ValueError("v7 tune claim monitor differs")
    if claim.get("effective_evaluator_sha256") != current_evaluator_sha(protocol):
        raise ValueError("v7 tune claim evaluator differs")
    if default_output("tune").exists():
        raise ValueError("v7 tune outcome already exists; a second run is forbidden")
    if default_tune_started().exists():
        raise ValueError("v7 tune has already started; a repeat is forbidden")
    if capture(["git", "status", "--short"]) != "":
        raise ValueError("v7 tune requires a clean Git worktree")


def require_clean_train_worktree() -> None:
    """Require the train artifact to describe a committed, reproducible tree."""

    if capture(["git", "status", "--short"]) != "":
        raise ValueError("v7 train requires a clean Git worktree")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--protocol", type=Path, default=DEFAULT_PROTOCOL)
    parser.add_argument("--phase", choices=("train", "tune"), default="train")
    parser.add_argument("--jobs", type=int, default=min(8, max(1, os.cpu_count() or 1)))
    parser.add_argument("--out", type=Path)
    parser.add_argument("--emit-records", action="store_true")
    parser.add_argument("--claim-tune", action="store_true")
    args = parser.parse_args()
    protocol_path = args.protocol.resolve()
    protocol = load_protocol(protocol_path)
    if args.claim_tune:
        if args.phase != "train" or args.out is not None:
            raise ValueError("--claim-tune is a standalone operation")
        claim_path = claim_tune(protocol, protocol_path)
        print(json.dumps({"status": "claimed_not_run", "path": str(claim_path.relative_to(ROOT))}, indent=2))
        return 0
    if args.phase == "train":
        require_clean_train_worktree()
    base_protocol = resolve_base_protocol(protocol)
    started_receipt: Path | None = None
    if args.phase == "tune":
        verify_tune_prerequisite(protocol)
        started_receipt = start_tune_execution(protocol)
    output = default_output(args.phase)
    if args.out is not None:
        raise ValueError("v7 does not allow --out; canonical result paths are immutable")
    if output.exists():
        raise ValueError("v7 canonical result already exists; rerun requires a new protocol")
    records = run_records(protocol, base_protocol, phase=args.phase, jobs=args.jobs)
    result = assemble_result(
        protocol,
        records,
        phase=args.phase,
        protocol_path=protocol_path,
        jobs=args.jobs,
        include_records=args.emit_records,
    )
    if started_receipt is not None:
        result["tune_started_receipt_sha256"] = file_sha256(started_receipt)
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(output.suffix + f".tmp-{os.getpid()}")
    temporary.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    os.replace(temporary, output)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if str(result["status"]).startswith("passed_") else 1


if __name__ == "__main__":
    raise SystemExit(main())
