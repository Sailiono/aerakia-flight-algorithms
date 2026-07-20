#!/usr/bin/env python3
"""Run the frozen horizontal accelerometer-bias cross-validation protocol."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import shutil
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

import numpy as np


def file_sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def input_csv_provenance(path: Path) -> dict[str, object]:
    """Capture immutable input evidence before compact mode removes the CSV."""

    with path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.reader(stream)
        try:
            next(reader)
        except StopIteration as error:
            raise ValueError("input CSV is empty") from error
        row_count = sum(1 for row in reader if row)
    return {
        "input_csv_sha256": file_sha256(path),
        "input_csv_bytes": path.stat().st_size,
        "input_csv_rows": row_count,
    }


def canonical_sha256(value: object) -> str:
    encoded = json.dumps(value, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


INFORMATION_STATES = (
    "excitation_not_achieved",
    "right_censored_after_excitation",
    "converged_after_excitation",
    "converged_before_information_ready",
)


def validate_information_markers(
    manifest: dict[str, object], protocol: dict[str, object]
) -> dict[str, object]:
    """Validate the analyzer-only excitation marker manifest.

    This is intentionally separate from :func:`validate_protocol`: the frozen v1
    protocol remains immutable while the marker manifest can evolve independently.
    """

    failures: list[str] = []
    if manifest.get("schema_version") != 1:
        failures.append("information marker schema_version must be 1")
    if manifest.get("protocol_id") != protocol.get("protocol_id"):
        failures.append("information marker protocol_id does not match protocol")
    if manifest.get("status") not in ("draft", "frozen"):
        failures.append("information marker status must be draft or frozen")
    declared_states = manifest.get("states")
    if declared_states != list(INFORMATION_STATES):
        failures.append("information marker states do not match the v1 classifier")
    markers = manifest.get("markers")
    if not isinstance(markers, dict):
        failures.append("information marker manifest requires a markers object")
        markers = {}
    expected_ids = {
        str(item.get("id"))
        for item in protocol.get("trajectories", [])
        if isinstance(item, dict) and item.get("split") != "holdout"
    }
    actual_ids = set(str(key) for key in markers)
    missing = sorted(expected_ids - actual_ids)
    unexpected = sorted(actual_ids - expected_ids)
    if missing:
        failures.append(f"missing information markers: {missing}")
    if unexpected:
        failures.append(f"unexpected information markers: {unexpected}")
    for trajectory_id, marker in markers.items():
        if not isinstance(marker, dict):
            failures.append(f"marker {trajectory_id} must be an object")
            continue
        value = marker.get("information_ready_time_s")
        if not isinstance(value, (int, float)) or not np.isfinite(float(value)) or float(value) < 0.0:
            failures.append(f"marker {trajectory_id} has invalid information_ready_time_s")
        if not str(marker.get("reason", "")).strip():
            failures.append(f"marker {trajectory_id} must include a reason")
        if not str(marker.get("source", "")).strip():
            failures.append(f"marker {trajectory_id} must include a source")
    return {"passed": not failures, "failures": failures}


def classify_information_state(
    *,
    alignment_time_s: float,
    settling_time_s: float | None,
    scenario_end_time_s: float,
    information_ready_time_s: float,
) -> dict[str, object]:
    """Classify convergence relative to a pre-declared excitation milestone.

    The filter output is the only input besides scenario timing; truth is never
    consulted. ``settling_time_s`` is relative to alignment, whereas all marker
    and convergence timestamps are absolute scenario time.
    """

    alignment = float(alignment_time_s)
    end = float(scenario_end_time_s)
    ready = float(information_ready_time_s)
    convergence = None if settling_time_s is None else alignment + float(settling_time_s)
    time_to_ready = ready - alignment
    time_from_ready = None if convergence is None else convergence - ready
    epsilon = 1.0e-9
    if end + epsilon < ready:
        state = "excitation_not_achieved"
    elif convergence is None:
        state = "right_censored_after_excitation"
    elif convergence < ready - epsilon:
        state = "converged_before_information_ready"
    else:
        state = "converged_after_excitation"
    return {
        "information_ready_time_s": ready,
        "time_to_information_ready_s": time_to_ready,
        "convergence_time_s": convergence,
        "time_from_information_ready_to_convergence_s": time_from_ready,
        "information_state": state,
    }


def information_marker_for_trajectory(
    trajectory: dict[str, object], information_markers: dict[str, object]
) -> dict[str, object]:
    """Resolve analyzer-only excitation metadata without opening sealed holdout.

    The public v1 marker manifest intentionally covers train/tune trajectories
    only.  Release holdout trials still run the estimator, metrics, and gates,
    but they cannot receive an excitation-aware interpretation until a protected
    holdout manifest supplies one.  Missing train/tune markers remain a hard
    configuration error.
    """

    trajectory_id = str(trajectory["id"])
    marker = information_markers.get(trajectory_id)
    if isinstance(marker, dict):
        return {
            "available": True,
            "status": "available",
            "information_ready_time_s": float(marker["information_ready_time_s"]),
        }
    if trajectory.get("split") == "holdout":
        return {
            "available": False,
            "status": "unavailable_sealed_holdout",
            "information_ready_time_s": None,
        }
    raise RuntimeError(f"no information marker for trajectory {trajectory_id}")


def seed_values(split: dict[str, object]) -> list[int]:
    return list(range(int(split["seed_start"]), int(split["seed_stop"])))


def validate_protocol(protocol: dict[str, object]) -> dict[str, object]:
    failures: list[str] = []
    if protocol.get("schema_version") != 1 or protocol.get("status") != "frozen":
        failures.append("protocol must be schema v1 and frozen")
    splits = protocol.get("splits")
    trajectories = protocol.get("trajectories")
    contract = protocol.get("residual_bias_contract")
    if not isinstance(splits, dict) or set(splits) != {"train", "tune", "holdout"}:
        failures.append("protocol requires train, tune, and holdout splits")
        splits = {}
    if not isinstance(trajectories, list) or len(trajectories) < 5:
        failures.append("protocol requires at least five trajectories")
        trajectories = []
    if not isinstance(contract, dict) or not isinstance(contract.get("vectors"), list):
        failures.append("protocol residual-bias vectors are missing")
        vectors: list[dict[str, object]] = []
    else:
        vectors = contract["vectors"]

    split_seeds = {
        name: set(seed_values(value))
        for name, value in splits.items()
        if isinstance(value, dict)
    }
    for first, second in (("train", "tune"), ("train", "holdout"), ("tune", "holdout")):
        overlap = split_seeds.get(first, set()) & split_seeds.get(second, set())
        if overlap:
            failures.append(f"{first}/{second} seed overlap: {sorted(overlap)}")

    trajectory_ids = [str(item.get("id")) for item in trajectories if isinstance(item, dict)]
    motion_ids = [str(item.get("motion")) for item in trajectories if isinstance(item, dict)]
    if len(set(trajectory_ids)) != len(trajectory_ids):
        failures.append("trajectory ids must be unique")
    if len(set(motion_ids)) != len(motion_ids):
        failures.append("trajectory motion names must be unique")
    for item in trajectories:
        if not isinstance(item, dict) or item.get("split") not in splits:
            failures.append(f"trajectory has invalid split: {item}")
        if isinstance(item, dict) and not str(item.get("motion", "")).startswith("bias_cv_"):
            failures.append(f"trajectory does not use independent bias_cv profile: {item}")

    vector_ids = [str(item.get("id")) for item in vectors if isinstance(item, dict)]
    expected_vectors = {
        (0, 0, 0), (-3, 0, 0), (3, 0, 0), (0, -3, 0), (0, 3, 0),
        (-3, -3, 0), (-3, 3, 0), (3, -3, 0), (3, 3, 0),
    }
    actual_vectors: set[tuple[int, int, int]] = set()
    for item in vectors:
        if not isinstance(item, dict):
            continue
        multipliers = item.get("accel_sigma_multipliers")
        if isinstance(multipliers, list) and len(multipliers) == 3:
            actual_vectors.add(tuple(int(value) for value in multipliers))
    if len(vector_ids) != 9 or len(set(vector_ids)) != 9 or actual_vectors != expected_vectors:
        failures.append("protocol must contain the exact nine frozen horizontal-bias vectors")

    release_count = 0
    for item in trajectories:
        if isinstance(item, dict) and item.get("split") in splits:
            release_count += len(seed_values(splits[str(item["split"])])) * len(vectors)
    declared_count = int(protocol.get("execution", {}).get("release_trial_count", -1))
    if release_count != declared_count:
        failures.append(
            f"release trial count {release_count} differs from declared {declared_count}"
        )
    initialization = protocol.get("coordinate_contract", {}).get("initialization")
    if initialization != "cold_start_from_sensor_stream_and_static_hint_only":
        failures.append("protocol initialization must prohibit truth-assisted initialization")
    smoke_seed_offset = protocol.get("execution", {}).get("smoke_seed_offset")
    if not isinstance(smoke_seed_offset, int) or smoke_seed_offset < 0:
        failures.append("execution.smoke_seed_offset must be a non-negative integer")
    else:
        smoke_splits = {
            str(item["split"])
            for item in trajectories
            if isinstance(item, dict) and item.get("split") != "holdout"
        }
        for split_name in smoke_splits:
            if smoke_seed_offset >= len(split_seeds.get(split_name, set())):
                failures.append(
                    f"smoke_seed_offset {smoke_seed_offset} is outside {split_name} split"
                )
    return {
        "passed": not failures,
        "failures": failures,
        "split_seed_counts": {name: len(values) for name, values in split_seeds.items()},
        "release_trial_count": release_count,
        "trajectory_count": len(trajectories),
        "bias_vector_count": len(vectors),
    }


def read_columns(path: Path, names: tuple[str, ...]) -> dict[str, np.ndarray]:
    values = {name: [] for name in names}
    with path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        missing = [name for name in names if name not in (reader.fieldnames or [])]
        if missing:
            raise ValueError("result CSV missing columns: " + ", ".join(missing))
        for row in reader:
            for name in names:
                values[name].append(float(row[name]))
    return {name: np.asarray(column, dtype=np.float64) for name, column in values.items()}


def horizontal_bias_metrics(
    results_csv: Path,
    metric_config: dict[str, object],
    information_ready_time_s: float | None = None,
) -> dict[str, object]:
    names = (
        "ts_us", "eskf_static_aligned",
        "truth_accel_bias_x_m_s2", "truth_accel_bias_y_m_s2",
        "eskf_accel_bias_x_m_s2", "eskf_accel_bias_y_m_s2",
    )
    columns = read_columns(results_csv, names)
    valid = (
        (columns["eskf_static_aligned"] > 0.5)
        & np.isfinite(columns["truth_accel_bias_x_m_s2"])
        & np.isfinite(columns["truth_accel_bias_y_m_s2"])
        & np.isfinite(columns["eskf_accel_bias_x_m_s2"])
        & np.isfinite(columns["eskf_accel_bias_y_m_s2"])
    )
    indices = np.flatnonzero(valid)
    if not len(indices):
        raise ValueError("no aligned finite horizontal-bias estimates")
    time_s = (columns["ts_us"] - columns["ts_us"][0]) * 1.0e-6
    error_x = columns["eskf_accel_bias_x_m_s2"] - columns["truth_accel_bias_x_m_s2"]
    error_y = columns["eskf_accel_bias_y_m_s2"] - columns["truth_accel_bias_y_m_s2"]
    horizontal_error = np.hypot(error_x, error_y)
    window_s = float(metric_config["settling_window_s"])
    candidate_step_s = float(metric_config["candidate_step_s"])
    axis_limit = float(metric_config["per_axis_p95_limit_m_s2"])
    norm_limit = float(metric_config["horizontal_norm_p95_limit_m_s2"])
    first = int(indices[0])
    last_time = float(time_s[int(indices[-1])])
    candidate_times = np.arange(float(time_s[first]), last_time - window_s + 1.0e-9, candidate_step_s)
    window_passes: list[bool] = []
    for start_time in candidate_times:
        selected = valid & (time_s >= start_time) & (time_s <= start_time + window_s)
        window_passes.append(bool(
            np.count_nonzero(selected) >= 2
            and np.percentile(np.abs(error_x[selected]), 95) <= axis_limit
            and np.percentile(np.abs(error_y[selected]), 95) <= axis_limit
            and np.percentile(horizontal_error[selected], 95) <= norm_limit
        ))
    suffix_passes = np.logical_and.accumulate(np.asarray(window_passes, dtype=bool)[::-1])[::-1]
    settling_candidates = np.flatnonzero(suffix_passes)
    settling_time_s = None
    if len(settling_candidates):
        settling_time_s = float(
            candidate_times[int(settling_candidates[0])] - time_s[first]
        )
    terminal_window_s = float(metric_config["terminal_window_s"])
    terminal = valid & (time_s >= last_time - terminal_window_s)
    result: dict[str, object] = {
        "alignment_time_s": float(time_s[first]),
        "settling_time_s": settling_time_s,
        "right_censored": settling_time_s is None,
        "settling_window_s": window_s,
        "terminal_sample_count": int(np.count_nonzero(terminal)),
        "terminal_abs_error_x_median_m_s2": float(np.median(np.abs(error_x[terminal]))),
        "terminal_abs_error_x_p95_m_s2": float(np.percentile(np.abs(error_x[terminal]), 95)),
        "terminal_abs_error_y_median_m_s2": float(np.median(np.abs(error_y[terminal]))),
        "terminal_abs_error_y_p95_m_s2": float(np.percentile(np.abs(error_y[terminal]), 95)),
        "terminal_horizontal_error_median_m_s2": float(np.median(horizontal_error[terminal])),
        "terminal_horizontal_error_p95_m_s2": float(np.percentile(horizontal_error[terminal], 95)),
        "terminal_horizontal_error_max_m_s2": float(np.max(horizontal_error[terminal])),
        "final_error_x_m_s2": float(error_x[int(indices[-1])]),
        "final_error_y_m_s2": float(error_y[int(indices[-1])]),
        "final_horizontal_error_m_s2": float(horizontal_error[int(indices[-1])]),
        "horizontal_error_rmse_m_s2": float(np.sqrt(np.mean(horizontal_error[valid] ** 2))),
    }
    if information_ready_time_s is not None:
        result.update(classify_information_state(
            alignment_time_s=float(time_s[first]),
            settling_time_s=settling_time_s,
            scenario_end_time_s=last_time,
            information_ready_time_s=float(information_ready_time_s),
        ))
    return result


def extract_general_metrics(metrics: dict[str, object]) -> dict[str, float | int]:
    alignment = metrics["cold_start_alignment"]
    navigation = metrics["navigation"]
    consistency = metrics["eskf_consistency"]
    integrity = metrics["eskf_integrity"]
    assert isinstance(alignment, dict)
    assert isinstance(navigation, dict)
    assert isinstance(consistency, dict)
    assert isinstance(integrity, dict)
    nees = consistency["navigation_nees"]
    assert isinstance(nees, dict)
    return {
        "post_alignment_attitude_rmse_deg": float(
            alignment["post_alignment_attitude_rmse_deg"]
        ),
        "position_rmse_m": float(navigation["position_rmse_m"]),
        "velocity_rmse_m_s": float(navigation["velocity_rmse_m_s"]),
        "navigation_nees_mean": float(nees["mean"]),
        "healthy_ratio": float(integrity["healthy_ratio"]),
        "navigation_recoveries": int(integrity["navigation_recoveries"]),
    }


def gate_failures(
    horizontal: dict[str, object],
    general: dict[str, float | int],
    gates: dict[str, object],
) -> list[str]:
    failures: list[str] = []
    settling = horizontal["settling_time_s"]
    if settling is None:
        failures.append("horizontal_settling_time_s=null")
    elif float(settling) > float(gates["horizontal_settling_time_max_s"]):
        failures.append(
            f"horizontal_settling_time_s={float(settling):.6g}>"
            f"{float(gates['horizontal_settling_time_max_s']):.6g}"
        )
    comparisons = (
        ("terminal_horizontal_error_p95_m_s2", horizontal, "horizontal_terminal_p95_max_m_s2", "maximum"),
        ("post_alignment_attitude_rmse_deg", general, "post_alignment_attitude_rmse_max_deg", "maximum"),
        ("position_rmse_m", general, "position_rmse_max_m", "maximum"),
        ("velocity_rmse_m_s", general, "velocity_rmse_max_m_s", "maximum"),
        ("navigation_nees_mean", general, "navigation_nees_mean_min", "minimum"),
        ("navigation_nees_mean", general, "navigation_nees_mean_max", "maximum"),
        ("healthy_ratio", general, "healthy_ratio_min", "minimum"),
        ("navigation_recoveries", general, "navigation_recoveries_max", "maximum"),
    )
    for metric_name, source, gate_name, direction in comparisons:
        value = float(source[metric_name])
        limit = float(gates[gate_name])
        failed = value > limit if direction == "maximum" else value < limit
        if failed:
            operator = ">" if direction == "maximum" else "<"
            failures.append(f"{metric_name}={value:.6g}{operator}{limit:.6g}")
    return failures


def run_logged(
    command: list[str],
    *,
    cwd: Path,
    environment: dict[str, str],
    log_path: Path,
    timeout_s: float,
) -> None:
    completed = subprocess.run(
        command,
        cwd=cwd,
        env=environment,
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=timeout_s,
    )
    log_path.write_text(completed.stdout, encoding="utf-8")
    if completed.returncode:
        raise RuntimeError(
            f"command returned {completed.returncode}: {' '.join(command)}\n"
            f"{completed.stdout[-2000:]}"
        )


def run_trial(
    trial: dict[str, object],
    *,
    root: Path,
    native_runner: Path,
    out_dir: Path,
    protocol: dict[str, object],
    information_markers: dict[str, object],
    timeout_s: float,
    compact: bool,
) -> dict[str, object]:
    trajectory = trial["trajectory"]
    vector = trial["vector"]
    assert isinstance(trajectory, dict)
    assert isinstance(vector, dict)
    trial_dir = (
        out_dir / "trials" / str(trajectory["split"]) / str(trajectory["id"])
        / str(vector["id"]) / f"seed-{int(trial['seed']):05d}"
    )
    trial_dir.mkdir(parents=True, exist_ok=True)
    input_csv = trial_dir / "input.csv"
    results_csv = trial_dir / "results.csv"
    metadata_json = trial_dir / "input-metadata.json"
    noise = protocol["noise"]
    contract = protocol["residual_bias_contract"]
    assert isinstance(noise, dict)
    assert isinstance(contract, dict)
    sigma = float(contract["accelerometer_sigma_m_s2"])
    accel_bias = [
        sigma * float(value) for value in vector["accel_sigma_multipliers"]
    ]
    gyro_bias = [float(value) for value in contract["gyroscope_vector_deg_s"]]
    generator_command = [
        sys.executable,
        str(root / "simulation/tools/generate_synthetic_imu.py"),
        "--out", str(input_csv),
        "--metadata", str(metadata_json),
        "--duration", str(trajectory["duration_s"]),
        "--rate", str(noise["rate_hz"]),
        "--seed", str(trial["seed"]),
        "--motion", str(trajectory["motion"]),
        "--anomaly", "none",
        "--static-hint",
        "--accel-time-semantics", str(
            protocol["coordinate_contract"]["acceleration_time_semantics"]
        ),
        "--accel-bias-vector-m-s2", *(str(value) for value in accel_bias),
        "--gyro-bias-vector-deg-s", *(str(value) for value in gyro_bias),
        "--accel-noise-m-s2", str(noise["accel_noise_m_s2"]),
        "--gyro-noise-deg-s", str(noise["gyro_noise_deg_s"]),
        "--mag-noise-ut", str(noise["mag_noise_ut"]),
        "--gps-position-noise-m", str(noise["gps_position_noise_m"]),
        "--gps-velocity-noise-m-s", str(noise["gps_velocity_noise_m_s"]),
        "--rate-invariant-streams",
    ]
    filter_command = [str(native_runner), "--cold-start", str(input_csv), str(results_csv)]
    analyzer_command = [
        sys.executable,
        str(root / "validation/analyze_results.py"),
        str(results_csv),
        "--out-dir", str(trial_dir),
        "--scenario", str(trajectory["id"]),
        "--no-plots",
    ]
    environment = os.environ.copy()
    environment["MPLCONFIGDIR"] = str((trial_dir / ".matplotlib").resolve())
    started = time.monotonic()
    run_logged(
        generator_command, cwd=root, environment=environment,
        log_path=trial_dir / "01-generator.log", timeout_s=timeout_s,
    )
    run_logged(
        filter_command, cwd=root, environment=environment,
        log_path=trial_dir / "02-filter.log", timeout_s=timeout_s,
    )
    run_logged(
        analyzer_command, cwd=root, environment=environment,
        log_path=trial_dir / "03-analyzer.log", timeout_s=timeout_s,
    )
    generated = json.loads(metadata_json.read_text(encoding="utf-8"))
    if generated.get("acceleration_bias_source") != "explicit_vector":
        raise RuntimeError("generator did not apply explicit accelerometer bias")
    if generated.get("motion") != trajectory["motion"]:
        raise RuntimeError("generator metadata motion does not match protocol")
    if generated.get("acceleration_time_semantics") != protocol["coordinate_contract"][
        "acceleration_time_semantics"
    ]:
        raise RuntimeError("generator acceleration time semantics do not match protocol")
    metrics = json.loads((trial_dir / "metrics.json").read_text(encoding="utf-8"))
    information_marker = information_marker_for_trajectory(
        trajectory, information_markers
    )
    horizontal = horizontal_bias_metrics(
        results_csv,
        protocol["metrics"],
        information_marker["information_ready_time_s"],
    )
    general = extract_general_metrics(metrics)
    failures = gate_failures(horizontal, general, protocol["gates"])
    artifact_provenance = input_csv_provenance(input_csv)
    result: dict[str, object] = {
        "trajectory_id": trajectory["id"],
        "motion": trajectory["motion"],
        "split": trajectory["split"],
        "bias_vector_id": vector["id"],
        "acceleration_bias_m_s2": accel_bias,
        "seed": trial["seed"],
        "runtime_s": time.monotonic() - started,
        "initialization": "cold_start_only",
        "truth_assisted_initialization": False,
        # Preserve the historical key for consumers of v1 reports.  The
        # structured provenance record is authoritative for new campaigns.
        "input_sha256": artifact_provenance["input_csv_sha256"],
        "provenance": artifact_provenance,
        "information_marker": information_marker,
        "commands": {
            "generator": generator_command,
            "filter": filter_command,
            "analyzer": analyzer_command,
        },
        "horizontal_bias": horizontal,
        "general": general,
        "gate_failures": failures,
    }
    (trial_dir / "cross-validation-metrics.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    if compact:
        input_csv.unlink(missing_ok=True)
        results_csv.unlink(missing_ok=True)
    return result


def build_trials(protocol: dict[str, object], mode: str) -> list[dict[str, object]]:
    trajectories = protocol["trajectories"]
    vectors = protocol["residual_bias_contract"]["vectors"]
    splits = protocol["splits"]
    assert isinstance(trajectories, list)
    assert isinstance(vectors, list)
    assert isinstance(splits, dict)
    smoke_vectors = set(protocol["execution"]["smoke_vectors"])
    smoke_seed_offset = int(protocol["execution"]["smoke_seed_offset"])
    trials: list[dict[str, object]] = []
    for trajectory in trajectories:
        assert isinstance(trajectory, dict)
        if mode in ("smoke", "train-tune") and trajectory["split"] == "holdout":
            continue
        split = splits[str(trajectory["split"])]
        seeds = seed_values(split)
        selected_seeds = [seeds[smoke_seed_offset]] if mode == "smoke" else seeds
        selected_vectors = (
            [vector for vector in vectors if vector["id"] in smoke_vectors]
            if mode == "smoke" else vectors
        )
        for seed in selected_seeds:
            for vector in selected_vectors:
                trials.append({"trajectory": trajectory, "vector": vector, "seed": seed})
    return trials


def metric_failures_are_fatal(mode: str) -> bool:
    """Return whether a completed campaign must fail on capability gates."""

    return mode in ("train-tune", "release")


def summarize(results: list[dict[str, object]]) -> dict[str, object]:
    groups: dict[str, dict[str, int]] = {}
    information_states = {state: 0 for state in INFORMATION_STATES}
    information_marker_unavailable_count = 0
    for result in results:
        key = f"{result['split']}/{result['trajectory_id']}/{result['bias_vector_id']}"
        group = groups.setdefault(key, {"trials": 0, "passed": 0, "failed": 0, "censored": 0})
        group["trials"] += 1
        failed = bool(result["gate_failures"])
        group["failed" if failed else "passed"] += 1
        if result["horizontal_bias"]["right_censored"]:
            group["censored"] += 1
        state = result["horizontal_bias"].get("information_state")
        if state in information_states:
            information_states[str(state)] += 1
        marker = result.get("information_marker")
        if isinstance(marker, dict) and not marker.get("available", False):
            information_marker_unavailable_count += 1
    return {
        "trial_count": len(results),
        "passed_count": sum(not result["gate_failures"] for result in results),
        "failed_count": sum(bool(result["gate_failures"]) for result in results),
        "right_censored_count": sum(
            bool(result["horizontal_bias"]["right_censored"]) for result in results
        ),
        "information_state_counts": information_states,
        "information_marker_unavailable_count": information_marker_unavailable_count,
        "groups": groups,
    }


def capture(command: list[str], cwd: Path) -> str | None:
    completed = subprocess.run(
        command, cwd=cwd, text=True, stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL, check=False,
    )
    return completed.stdout.strip() if completed.returncode == 0 else None


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runner", type=Path, required=True)
    parser.add_argument(
        "--protocol", type=Path,
        default=Path("validation/bias_observability_protocol_v1.json"),
    )
    parser.add_argument(
        "--information-markers", type=Path,
        default=Path("validation/bias_observability_information_markers_v1.json"),
        help="analyzer-only excitation milestone manifest for non-holdout trajectories",
    )
    parser.add_argument("--out-dir", type=Path, default=Path("build/bias-observability-v1"))
    parser.add_argument(
        "--mode", choices=("smoke", "train-tune", "release"), default="smoke",
        help=(
            "smoke runs one offset-selected seed and three vectors per non-holdout trajectory; "
            "train-tune runs every non-holdout trial; release is the only mode that opens "
            "the frozen holdout"
        ),
    )
    parser.add_argument("--jobs", type=int, default=4)
    parser.add_argument("--timeout-s", type=float, default=180.0)
    parser.add_argument("--compact", action="store_true")
    parser.add_argument(
        "--report-only", action="store_true",
        help=(
            "deprecated compatibility flag; reports are always written before exit and "
            "train-tune/release capability failures still return non-zero"
        ),
    )
    args = parser.parse_args()
    if args.jobs < 1 or args.timeout_s <= 0.0:
        parser.error("jobs and timeout must be positive")

    root = Path(__file__).resolve().parents[1]
    protocol_path = args.protocol if args.protocol.is_absolute() else root / args.protocol
    information_markers_path = (
        args.information_markers
        if args.information_markers.is_absolute()
        else root / args.information_markers
    )
    out_dir = args.out_dir if args.out_dir.is_absolute() else root / args.out_dir
    protocol = json.loads(protocol_path.read_text(encoding="utf-8"))
    protocol_validation = validate_protocol(protocol)
    if not protocol_validation["passed"]:
        raise SystemExit("invalid protocol: " + "; ".join(protocol_validation["failures"]))
    information_marker_manifest = json.loads(
        information_markers_path.read_text(encoding="utf-8")
    )
    information_marker_validation = validate_information_markers(
        information_marker_manifest, protocol
    )
    if not information_marker_validation["passed"]:
        raise SystemExit(
            "invalid information markers: "
            + "; ".join(information_marker_validation["failures"])
        )
    information_markers = information_marker_manifest["markers"]
    native_runner = args.runner.resolve()
    trials = build_trials(protocol, args.mode)
    if args.mode == "smoke":
        expected_trials = sum(
            len(protocol["execution"]["smoke_vectors"])
            for trajectory in protocol["trajectories"]
            if trajectory["split"] != "holdout"
        )
    elif args.mode == "train-tune":
        expected_trials = sum(
            len(seed_values(protocol["splits"][str(trajectory["split"])]))
            * len(protocol["residual_bias_contract"]["vectors"])
            for trajectory in protocol["trajectories"]
            if trajectory["split"] != "holdout"
        )
    else:
        expected_trials = int(protocol["execution"]["release_trial_count"])
    if len(trials) != expected_trials:
        raise SystemExit(f"trial selection produced {len(trials)}, expected {expected_trials}")
    out_dir.mkdir(parents=True, exist_ok=True)

    started = time.monotonic()
    results: list[dict[str, object]] = []
    execution_failures: list[dict[str, object]] = []
    with ThreadPoolExecutor(max_workers=args.jobs) as executor:
        futures = {
            executor.submit(
                run_trial,
                trial,
                root=root,
                native_runner=native_runner,
                out_dir=out_dir,
                protocol=protocol,
                information_markers=information_markers,
                timeout_s=args.timeout_s,
                compact=args.compact,
            ): trial
            for trial in trials
        }
        completed_count = 0
        for future in as_completed(futures):
            trial = futures[future]
            try:
                results.append(future.result())
            except Exception as error:
                trajectory = trial["trajectory"]
                vector = trial["vector"]
                execution_failures.append({
                    "trajectory_id": trajectory["id"],
                    "split": trajectory["split"],
                    "bias_vector_id": vector["id"],
                    "seed": trial["seed"],
                    "error": f"{type(error).__name__}: {error}",
                })
            completed_count += 1
            if completed_count == 1 or completed_count % 25 == 0 or completed_count == len(trials):
                print(f"completed {completed_count}/{len(trials)} cross-validation trials")
    results.sort(key=lambda item: (
        str(item["split"]), str(item["trajectory_id"]),
        str(item["bias_vector_id"]), int(item["seed"]),
    ))
    summary = summarize(results)
    metric_gate_enforced = metric_failures_are_fatal(args.mode)
    execution_status = "passed" if not execution_failures else "failed"
    capability_status = "passed" if summary["failed_count"] == 0 else "failed"
    if execution_failures:
        status = "execution_failed"
    elif summary["failed_count"]:
        status = "capability_failed"
    else:
        status = "passed"
    output = {
        "schema_version": 1,
        "status": status,
        "execution_status": execution_status,
        "capability_status": capability_status,
        "mode": args.mode,
        "metric_gate_enforced": metric_gate_enforced,
        "report_only_requested": args.report_only,
        "protocol": {
            "id": protocol["protocol_id"],
            "path": str(protocol_path),
            "file_sha256": file_sha256(protocol_path),
            "semantic_sha256": canonical_sha256(protocol),
            "validation": protocol_validation,
        },
        "information_markers": {
            "id": information_marker_manifest["marker_id"],
            "path": str(information_markers_path),
            "file_sha256": file_sha256(information_markers_path),
            "semantic_sha256": canonical_sha256(information_marker_manifest),
            "validation": information_marker_validation,
        },
        "provenance": {
            "git_commit": capture(["git", "rev-parse", "HEAD"], root),
            "git_status_short": (capture(["git", "status", "--short"], root) or "").splitlines(),
            "runner_path": str(native_runner),
            "runner_sha256": file_sha256(native_runner),
            "generator_sha256": file_sha256(root / "simulation/tools/generate_synthetic_imu.py"),
            "analyzer_sha256": file_sha256(root / "validation/analyze_results.py"),
            "campaign_runner_sha256": file_sha256(Path(__file__).resolve()),
            "runtime_s": time.monotonic() - started,
        },
        "truth_boundary": {
            "initialization": "cold_start_only",
            "truth_assisted_initialization": False,
            "filter_command_shape": ["runner", "--cold-start", "input.csv", "results.csv"],
        },
        "execution_failures": execution_failures,
        "summary": summary,
        "trials": results,
    }
    (out_dir / "summary.json").write_text(
        json.dumps(output, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )

    lines = [
        "# Horizontal accelerometer-bias observability cross-validation v1",
        "",
        f"Status: **{status}**; execution: **{execution_status}**; capability: "
        f"**{capability_status}**; mode: **{args.mode}**; completed: "
        f"**{len(results)}/{len(trials)}**; metric failures: **{summary['failed_count']}**; "
        f"execution failures: **{len(execution_failures)}**.",
        "",
        f"Protocol semantic SHA-256: `{output['protocol']['semantic_sha256']}`.",
        "",
        "| Group | Trials | Passed | Failed | Right-censored |",
        "| --- | ---: | ---: | ---: | ---: |",
    ]
    for group, values in summary["groups"].items():
        lines.append(
            f"| {group} | {values['trials']} | {values['passed']} "
            f"| {values['failed']} | {values['censored']} |"
        )
    lines.extend([
        "",
        "The filter was started only with `--cold-start`; truth columns were consumed by the "
        "post-run analyzers and were not supplied as an initialization argument. Smoke mode "
        "uses only train/tune trajectories and reports metric failures without enforcing them; "
        "train-tune and release modes enforce every trial gate. Reports are written before a "
        "non-zero capability exit.",
    ])
    (out_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"cross-validation report: {out_dir / 'report.md'}")
    if execution_failures:
        raise SystemExit(f"{len(execution_failures)} cross-validation trial(s) did not execute")
    if metric_gate_enforced and summary["failed_count"]:
        raise SystemExit(f"{summary['failed_count']} capability metric gate(s) failed")


if __name__ == "__main__":
    main()
