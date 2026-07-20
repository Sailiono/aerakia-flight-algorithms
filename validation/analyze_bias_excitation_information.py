#!/usr/bin/env python3
"""Causal local-information diagnostic for tilt/accelerometer-bias observability.

This analyzer deliberately operates outside the estimator.  It rebuilds the
published 15-state right-error transition from past estimated states and IMU
samples, then uses only actually accepted position/velocity observations.  It
never reads truth, commanded motion, a trajectory identifier, or a future
sample when evaluating a window ending at the current timestamp.

The reported five-dimensional information matrix targets
``[dtheta_x, dtheta_y, dab_x, dab_y, dab_z]``.  All remaining error states are
treated as nuisance variables and projected out of the whitened observation
design matrix.  Consequently, repeated static velocity observations cannot
artificially separate the familiar tilt/horizontal-bias ambiguity.

This is an analyzer-only research tool.  Its structural-readiness thresholds
are diagnostic defaults, not statistical confidence or flight-software gates.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Mapping, Sequence

import numpy as np


STATE_DIMENSION = 15
TARGET_INDICES = np.asarray([0, 1, 9, 10, 11], dtype=np.int64)
TARGET_LABELS = ("tilt_x", "tilt_y", "accel_bias_x", "accel_bias_y", "accel_bias_z")
NUISANCE_INDICES = np.asarray(
    [index for index in range(STATE_DIMENSION) if index not in set(TARGET_INDICES)],
    dtype=np.int64,
)

# Physical scale represented by one unit of normalized error state.  Scaling
# makes eigenvalue and condition-number diagnostics dimensionless.
STATE_SCALES = np.asarray(
    [
        0.05, 0.05, 0.10,       # attitude error, rad
        5.0, 5.0, 5.0,          # velocity error, m/s
        10.0, 10.0, 10.0,       # position error, m
        0.15, 0.15, 0.15,       # accelerometer bias, m/s^2
        0.02, 0.02, 0.02,       # gyroscope bias, rad/s
    ],
    dtype=np.float64,
)


@dataclass(frozen=True)
class InformationThresholds:
    """Structural analyzer thresholds; these do not express estimator confidence."""

    rank_relative_tolerance: float = 1.0e-7
    rank_absolute_tolerance: float = 1.0e-10
    nuisance_relative_tolerance: float = 1.0e-10
    minimum_eigenvalue: float = 1.0e-4
    maximum_condition_number: float = 1.0e8
    minimum_direction_information: float = 1.0e-4
    minimum_window_duration_s: float = 5.0
    minimum_unique_aiding_epochs: int = 5
    minimum_aiding_bin_coverage: float = 0.50
    maximum_aiding_age_s: float = 0.50
    aiding_bin_duration_s: float = 1.0


@dataclass(frozen=True)
class ExcitationSamples:
    """Past-only estimator and sensor channels required by the diagnostic."""

    timestamp_s: np.ndarray
    acceleration_body_m_s2: np.ndarray
    angular_rate_body_rad_s: np.ndarray
    estimated_quaternion_wxyz: np.ndarray
    estimated_accel_bias_m_s2: np.ndarray
    estimated_gyro_bias_rad_s: np.ndarray
    position_attempted: np.ndarray
    position_accepted: np.ndarray
    position_variance_m2: np.ndarray
    velocity_attempted: np.ndarray
    velocity_accepted: np.ndarray
    velocity_variance_m2_s2: np.ndarray

    def validate(self) -> None:
        count = len(self.timestamp_s)
        vector_shapes = {
            "acceleration_body_m_s2": self.acceleration_body_m_s2.shape,
            "angular_rate_body_rad_s": self.angular_rate_body_rad_s.shape,
            "estimated_accel_bias_m_s2": self.estimated_accel_bias_m_s2.shape,
            "estimated_gyro_bias_rad_s": self.estimated_gyro_bias_rad_s.shape,
        }
        for name, shape in vector_shapes.items():
            if shape != (count, 3):
                raise ValueError(f"{name} must have shape ({count}, 3), got {shape}")
        if self.estimated_quaternion_wxyz.shape != (count, 4):
            raise ValueError(
                "estimated_quaternion_wxyz must have shape "
                f"({count}, 4), got {self.estimated_quaternion_wxyz.shape}"
            )
        scalar_names = (
            "position_attempted", "position_accepted", "position_variance_m2",
            "velocity_attempted", "velocity_accepted", "velocity_variance_m2_s2",
        )
        for name in scalar_names:
            if np.asarray(getattr(self, name)).shape != (count,):
                raise ValueError(f"{name} must contain exactly {count} samples")
        if count < 2:
            raise ValueError("at least two samples are required")
        if not np.all(np.isfinite(self.timestamp_s)) or np.any(np.diff(self.timestamp_s) <= 0.0):
            raise ValueError("timestamps must be finite and strictly increasing")
        numeric_arrays = (
            self.acceleration_body_m_s2,
            self.angular_rate_body_rad_s,
            self.estimated_quaternion_wxyz,
            self.estimated_accel_bias_m_s2,
            self.estimated_gyro_bias_rad_s,
        )
        if not all(np.all(np.isfinite(values)) for values in numeric_arrays):
            raise ValueError("IMU and estimated-state channels must be finite")
        for attempted, accepted, variance, name in (
            (
                self.position_attempted,
                self.position_accepted,
                self.position_variance_m2,
                "position",
            ),
            (
                self.velocity_attempted,
                self.velocity_accepted,
                self.velocity_variance_m2_s2,
                "velocity",
            ),
        ):
            if np.any(np.asarray(accepted, dtype=bool) & ~np.asarray(attempted, dtype=bool)):
                raise ValueError(f"accepted {name} updates must also be marked attempted")
            used = np.asarray(accepted, dtype=bool)
            if np.any(~np.isfinite(np.asarray(variance)[used])) or np.any(
                np.asarray(variance)[used] <= 0.0
            ):
                raise ValueError(f"accepted {name} variances must be finite and positive")


def _skew(vector: np.ndarray) -> np.ndarray:
    x_value, y_value, z_value = vector
    return np.asarray(
        [[0.0, -z_value, y_value], [z_value, 0.0, -x_value], [-y_value, x_value, 0.0]],
        dtype=np.float64,
    )


def _quaternion_rotation(quaternion_wxyz: np.ndarray) -> np.ndarray:
    quaternion = np.asarray(quaternion_wxyz, dtype=np.float64)
    norm = float(np.linalg.norm(quaternion))
    if not math.isfinite(norm) or norm <= 1.0e-12:
        raise ValueError("estimated quaternion must have finite nonzero norm")
    w_value, x_value, y_value, z_value = quaternion / norm
    return np.asarray(
        [
            [
                1.0 - 2.0 * (y_value * y_value + z_value * z_value),
                2.0 * (x_value * y_value - z_value * w_value),
                2.0 * (x_value * z_value + y_value * w_value),
            ],
            [
                2.0 * (x_value * y_value + z_value * w_value),
                1.0 - 2.0 * (x_value * x_value + z_value * z_value),
                2.0 * (y_value * z_value - x_value * w_value),
            ],
            [
                2.0 * (x_value * z_value - y_value * w_value),
                2.0 * (y_value * z_value + x_value * w_value),
                1.0 - 2.0 * (x_value * x_value + y_value * y_value),
            ],
        ],
        dtype=np.float64,
    )


def _right_jacobian(rotation_vector: np.ndarray) -> np.ndarray:
    angle = float(np.linalg.norm(rotation_vector))
    skew = _skew(rotation_vector)
    skew_squared = skew @ skew
    if angle < 1.0e-6:
        coefficient_a = 0.5 - angle * angle / 24.0
        coefficient_b = 1.0 / 6.0 - angle * angle / 120.0
    else:
        coefficient_a = (1.0 - math.cos(angle)) / (angle * angle)
        coefficient_b = (angle - math.sin(angle)) / (angle * angle * angle)
    return np.eye(3) - coefficient_a * skew + coefficient_b * skew_squared


def transition_matrix(
    quaternion_wxyz: np.ndarray,
    acceleration_body_m_s2: np.ndarray,
    angular_rate_body_rad_s: np.ndarray,
    dt_s: float,
) -> np.ndarray:
    """Rebuild ``eskf_model_transition`` without using estimator truth."""

    if not math.isfinite(dt_s) or dt_s <= 0.0:
        raise ValueError("transition dt must be finite and positive")
    rotation_nb = _quaternion_rotation(quaternion_wxyz)
    specific_force = np.asarray(acceleration_body_m_s2, dtype=np.float64)
    angular_rate = np.asarray(angular_rate_body_rad_s, dtype=np.float64)
    rotation_vector = angular_rate * dt_s
    rotation_angle = float(np.linalg.norm(rotation_vector))
    delta_rotation = _quaternion_rotation(
        np.concatenate(
            ([math.cos(0.5 * rotation_angle)],
             np.zeros(3) if rotation_angle <= 1.0e-12 else (
                 rotation_vector / rotation_angle
                 * math.sin(0.5 * rotation_angle)
             ))
        )
    )
    transition = np.eye(STATE_DIMENSION, dtype=np.float64)
    transition[0:3, 0:3] = delta_rotation.T
    transition[3:6, 0:3] = -(rotation_nb @ _skew(specific_force)) * dt_s
    transition[3:6, 9:12] = -rotation_nb * dt_s
    transition[6:9, 0:3] = -(rotation_nb @ _skew(specific_force)) * (0.5 * dt_s * dt_s)
    transition[6:9, 3:6] = np.eye(3) * dt_s
    transition[6:9, 9:12] = -rotation_nb * (0.5 * dt_s * dt_s)
    transition[0:3, 12:15] = -_right_jacobian(rotation_vector) * dt_s
    return transition


def _observation_rows(
    samples: ExcitationSamples,
    start_index: int,
    stop_index: int,
) -> tuple[np.ndarray, list[float], dict[str, int]]:
    transition_from_start = np.eye(STATE_DIMENSION, dtype=np.float64)
    rows: list[np.ndarray] = []
    accepted_times: list[float] = []
    counts = {
        "position_attempted": 0,
        "position_accepted": 0,
        "velocity_attempted": 0,
        "velocity_accepted": 0,
    }
    position_h = np.zeros((3, STATE_DIMENSION), dtype=np.float64)
    position_h[:, 6:9] = np.eye(3)
    velocity_h = np.zeros((3, STATE_DIMENSION), dtype=np.float64)
    velocity_h[:, 3:6] = np.eye(3)

    for index in range(start_index, stop_index + 1):
        if index > start_index:
            dt_s = float(samples.timestamp_s[index] - samples.timestamp_s[index - 1])
            acceleration = (
                samples.acceleration_body_m_s2[index]
                - samples.estimated_accel_bias_m_s2[index - 1]
            )
            angular_rate = (
                samples.angular_rate_body_rad_s[index]
                - samples.estimated_gyro_bias_rad_s[index - 1]
            )
            local_transition = transition_matrix(
                samples.estimated_quaternion_wxyz[index - 1],
                acceleration,
                angular_rate,
                dt_s,
            )
            transition_from_start = local_transition @ transition_from_start

        # Results rows contain the post-update state at their timestamp.  The
        # start observation has already shaped this window's error coordinates.
        if index == start_index:
            continue
        if bool(samples.position_attempted[index]):
            counts["position_attempted"] += 1
        if bool(samples.velocity_attempted[index]):
            counts["velocity_attempted"] += 1
        if bool(samples.position_accepted[index]):
            counts["position_accepted"] += 1
            rows.extend(
                (position_h @ transition_from_start)
                / math.sqrt(float(samples.position_variance_m2[index]))
            )
            accepted_times.append(float(samples.timestamp_s[index]))
        if bool(samples.velocity_accepted[index]):
            counts["velocity_accepted"] += 1
            rows.extend(
                (velocity_h @ transition_from_start)
                / math.sqrt(float(samples.velocity_variance_m2_s2[index]))
            )
            accepted_times.append(float(samples.timestamp_s[index]))

    if not rows:
        return np.empty((0, STATE_DIMENSION), dtype=np.float64), accepted_times, counts
    return np.asarray(rows, dtype=np.float64), accepted_times, counts


def _project_nuisance(design: np.ndarray, tolerance: float) -> tuple[np.ndarray, int]:
    scaled_design = design * STATE_SCALES[np.newaxis, :]
    target = scaled_design[:, TARGET_INDICES]
    nuisance = scaled_design[:, NUISANCE_INDICES]
    if nuisance.size == 0 or not np.any(nuisance):
        return target, 0
    singular_values = np.linalg.svd(nuisance, compute_uv=False)
    cutoff = (
        tolerance * singular_values[0]
        if singular_values.size and singular_values[0] > 0.0
        else tolerance
    )
    nuisance_rank = int(np.count_nonzero(singular_values > cutoff))
    if nuisance_rank == 0:
        return target, 0
    basis, _, _ = np.linalg.svd(nuisance, full_matrices=False)
    nuisance_basis = basis[:, :nuisance_rank]
    return target - nuisance_basis @ (nuisance_basis.T @ target), nuisance_rank


def _coverage_metrics(
    samples: ExcitationSamples,
    start_index: int,
    stop_index: int,
    accepted_times: Sequence[float],
    counts: Mapping[str, int],
    bin_duration_s: float,
) -> dict[str, float | int | None]:
    start_time = float(samples.timestamp_s[start_index])
    end_time = float(samples.timestamp_s[stop_index])
    duration = end_time - start_time
    unique_times = np.unique(np.asarray(accepted_times, dtype=np.float64))
    if len(unique_times):
        aiding_age = end_time - float(unique_times[-1])
        gaps = np.diff(np.concatenate(([start_time], unique_times, [end_time])))
        maximum_gap = float(np.max(gaps))
        bin_count = max(1, int(math.ceil(max(duration, 1.0e-12) / bin_duration_s)))
        occupied = np.unique(
            np.minimum(
                ((unique_times - start_time) / bin_duration_s).astype(np.int64),
                bin_count - 1,
            )
        )
        bin_coverage = float(len(occupied) / bin_count)
    else:
        aiding_age = None
        maximum_gap = duration
        bin_coverage = 0.0
    attempted = counts["position_attempted"] + counts["velocity_attempted"]
    accepted = counts["position_accepted"] + counts["velocity_accepted"]
    return {
        **counts,
        "accepted_total": accepted,
        "attempted_total": attempted,
        "acceptance_ratio": float(accepted / attempted) if attempted else None,
        "accepted_unique_timestamps": int(len(unique_times)),
        "window_duration_s": duration,
        "aiding_age_at_window_end_s": aiding_age,
        "maximum_aiding_gap_s": maximum_gap,
        "aiding_bin_duration_s": bin_duration_s,
        "aiding_bin_coverage": bin_coverage,
    }


def analyze_window(
    samples: ExcitationSamples,
    start_index: int = 0,
    stop_index: int | None = None,
    thresholds: InformationThresholds = InformationThresholds(),
) -> dict[str, object]:
    """Analyze one closed, causal window ending at ``stop_index``."""

    samples.validate()
    if stop_index is None:
        stop_index = len(samples.timestamp_s) - 1
    if start_index < 0 or stop_index >= len(samples.timestamp_s) or start_index >= stop_index:
        raise ValueError("window indices must select at least two in-range samples")
    design, accepted_times, counts = _observation_rows(samples, start_index, stop_index)
    projected, nuisance_rank = _project_nuisance(
        design, thresholds.nuisance_relative_tolerance
    )
    information = projected.T @ projected
    information = 0.5 * (information + information.T)
    eigenvalues = np.linalg.eigvalsh(information)
    eigenvalues[np.abs(eigenvalues) < thresholds.rank_absolute_tolerance] = 0.0
    largest = float(max(eigenvalues[-1], 0.0))
    rank_cutoff = max(
        thresholds.rank_absolute_tolerance,
        thresholds.rank_relative_tolerance * largest,
    )
    effective_rank = int(np.count_nonzero(eigenvalues > rank_cutoff))
    minimum = float(max(eigenvalues[0], 0.0))
    condition = float(largest / minimum) if effective_rank == len(TARGET_INDICES) and minimum > 0 else None
    per_direction = {
        label: float(max(information[index, index], 0.0))
        for index, label in enumerate(TARGET_LABELS)
    }
    coverage = _coverage_metrics(
        samples,
        start_index,
        stop_index,
        accepted_times,
        counts,
        thresholds.aiding_bin_duration_s,
    )
    aiding_age = coverage["aiding_age_at_window_end_s"]
    checks = {
        "full_target_rank": effective_rank == len(TARGET_INDICES),
        "minimum_eigenvalue": minimum >= thresholds.minimum_eigenvalue,
        "condition_number": condition is not None
        and condition <= thresholds.maximum_condition_number,
        "per_direction_information": min(per_direction.values(), default=0.0)
        >= thresholds.minimum_direction_information,
        "window_duration": coverage["window_duration_s"]
        >= thresholds.minimum_window_duration_s,
        "unique_aiding_epochs": coverage["accepted_unique_timestamps"]
        >= thresholds.minimum_unique_aiding_epochs,
        "aiding_bin_coverage": coverage["aiding_bin_coverage"]
        >= thresholds.minimum_aiding_bin_coverage,
        "aiding_freshness": aiding_age is not None
        and aiding_age <= thresholds.maximum_aiding_age_s,
    }
    return {
        "schema_version": 2,
        "status": "structural_analyzer_only_not_estimator_gate",
        "causal_window": {
            "start_index": start_index,
            "stop_index": stop_index,
            "start_time_s": float(samples.timestamp_s[start_index]),
            "stop_time_s": float(samples.timestamp_s[stop_index]),
        },
        "target_order": list(TARGET_LABELS),
        "method": {
            "name": "whitened_local_observability_design_with_nuisance_projection",
            "error_state_order": [
                "dtheta_x", "dtheta_y", "dtheta_z",
                "dv_n", "dv_e", "dv_d",
                "dp_n", "dp_e", "dp_d",
                "dab_x", "dab_y", "dab_z",
                "dgb_x", "dgb_y", "dgb_z",
            ],
            "state_scales": STATE_SCALES.tolist(),
            "measurement_sources": [
                "accepted_position_observations", "accepted_velocity_observations"
            ],
            "measurement_design_row_count": int(design.shape[0]),
            "nuisance_design_rank": nuisance_rank,
            "limitations": [
                "local_first_order_linearization",
                "imu_process_noise_and_bias_random_walk_are_not_in_the_information_matrix",
                "preintegration_covariance_is_not_propagated",
                "aiding_temporal_and_cross_source_correlations_are_not_modeled",
                "information_magnitude_depends_on_aiding_rate_and_declared_variance",
                "error_reset_jacobians_are_not_reconstructed",
                "structural_thresholds_are_not_statistical_or_product_release_gates",
            ],
        },
        "thresholds": asdict(thresholds),
        "normalized_information_matrix": information.tolist(),
        "eigenvalues_ascending": eigenvalues.tolist(),
        "effective_rank": effective_rank,
        "rank_cutoff": rank_cutoff,
        "minimum_eigenvalue": minimum,
        "condition_number": condition,
        "per_direction_information": per_direction,
        "aiding_coverage": coverage,
        "structural_readiness_checks_analyzer_only": checks,
        "structural_information_ready_analyzer_only": all(checks.values()),
        "prohibited_inputs": [
            "truth", "future_samples", "trajectory_identifier", "commanded_motion"
        ],
    }


def analyze_causal_timeline(
    samples: ExcitationSamples,
    window_duration_s: float,
    evaluation_interval_s: float,
    thresholds: InformationThresholds = InformationThresholds(),
) -> dict[str, object]:
    """Evaluate trailing windows using only data at or before each evaluation time."""

    samples.validate()
    if window_duration_s <= 0.0 or evaluation_interval_s <= 0.0:
        raise ValueError("window and evaluation intervals must be positive")
    evaluations: list[dict[str, object]] = []
    next_evaluation = float(samples.timestamp_s[0]) + evaluation_interval_s
    first_ready_time: float | None = None
    for stop_index in range(1, len(samples.timestamp_s)):
        stop_time = float(samples.timestamp_s[stop_index])
        if stop_time + 1.0e-12 < next_evaluation and stop_index != len(samples.timestamp_s) - 1:
            continue
        start_time = stop_time - window_duration_s
        start_index = int(np.searchsorted(samples.timestamp_s, start_time, side="left"))
        if start_index >= stop_index:
            continue
        result = analyze_window(samples, start_index, stop_index, thresholds)
        evaluations.append(result)
        if result["structural_information_ready_analyzer_only"] and first_ready_time is None:
            first_ready_time = stop_time
        while next_evaluation <= stop_time + 1.0e-12:
            next_evaluation += evaluation_interval_s
    return {
        "schema_version": 2,
        "status": "structural_analyzer_only_not_estimator_gate",
        "window_duration_s": window_duration_s,
        "evaluation_interval_s": evaluation_interval_s,
        "evaluation_count": len(evaluations),
        "first_structural_information_ready_time_s_analyzer_only": first_ready_time,
        "evaluations": evaluations,
    }


def _read_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None:
            raise ValueError(f"missing CSV header: {path}")
        return list(reader)


def _file_fingerprint(path: Path) -> dict[str, object]:
    digest = hashlib.sha256()
    byte_count = 0
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
            byte_count += len(chunk)
    return {"sha256": digest.hexdigest(), "bytes": byte_count}


def validate_causal_provenance(
    manifest_path: Path,
    replay_path: Path,
    results_path: Path,
) -> dict[str, object]:
    """Bind inputs to declared causal execution artifacts and assertions.

    The validator can prove file identity and reject known-incompatible command
    lines.  It cannot prove the semantics of an arbitrary runner binary, so the
    causal fields remain explicit producer assertions rather than an attested
    no-truth proof.
    """

    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"invalid causal provenance manifest: {manifest_path}") from error
    if not isinstance(manifest, dict) or manifest.get("schema_version") != 2:
        raise ValueError("causal provenance manifest must use schema_version 2")

    actual_sources = {
        "replay": _file_fingerprint(replay_path),
        "results": _file_fingerprint(results_path),
    }
    declared_sources = manifest.get("sources")
    if not isinstance(declared_sources, dict):
        raise ValueError("causal provenance manifest is missing sources")
    for name, actual in actual_sources.items():
        declared = declared_sources.get(name)
        if not isinstance(declared, dict):
            raise ValueError(f"causal provenance manifest is missing sources.{name}")
        if declared.get("sha256") != actual["sha256"]:
            raise ValueError(f"causal provenance {name} SHA-256 does not match")
        if declared.get("bytes") != actual["bytes"]:
            raise ValueError(f"causal provenance {name} byte count does not match")

    execution = manifest.get("execution")
    if not isinstance(execution, dict):
        raise ValueError("causal provenance manifest is missing execution")
    actual_artifacts: dict[str, dict[str, object]] = {}
    declared_artifacts = execution.get("artifacts")
    if not isinstance(declared_artifacts, dict):
        raise ValueError("causal provenance execution is missing artifacts")
    artifact_paths: dict[str, Path] = {}
    for name in ("replay_generator", "estimator_runner"):
        declared = declared_artifacts.get(name)
        if not isinstance(declared, dict) or not isinstance(declared.get("path"), str):
            raise ValueError(f"causal provenance execution is missing artifacts.{name}")
        artifact_path = Path(declared["path"]).expanduser().resolve()
        if not artifact_path.is_file():
            raise ValueError(f"causal provenance artifact does not exist: {name}")
        actual = _file_fingerprint(artifact_path)
        if declared.get("sha256") != actual["sha256"]:
            raise ValueError(f"causal provenance {name} SHA-256 does not match")
        if declared.get("bytes") != actual["bytes"]:
            raise ValueError(f"causal provenance {name} byte count does not match")
        artifact_paths[name] = artifact_path
        actual_artifacts[name] = {"path": str(artifact_path), **actual}

    command = execution.get("command")
    if not isinstance(command, list) or not command or not all(
        isinstance(argument, str) and argument for argument in command
    ):
        raise ValueError("causal provenance execution.command must be a nonempty string list")
    if Path(command[0]).expanduser().resolve() != artifact_paths["estimator_runner"]:
        raise ValueError("causal provenance command does not invoke estimator_runner")
    resolved_arguments = {
        str(Path(argument).expanduser().resolve())
        for argument in command[1:]
        if not argument.startswith("-")
    }
    for name, path in (("replay", replay_path), ("results", results_path)):
        if str(path.resolve()) not in resolved_arguments:
            raise ValueError(f"causal provenance command does not bind the {name} path")
    prohibited_options = {
        "--reference-attitude-init", "--truth-init", "--truth-seeded-init",
        "--truth-derived-stationarity",
    }
    present_prohibited = sorted(prohibited_options.intersection(command))
    if present_prohibited:
        raise ValueError(
            "causal provenance command contains prohibited option(s): "
            + ", ".join(present_prohibited)
        )
    git_commit = execution.get("git_commit")
    if not isinstance(git_commit, str) or len(git_commit) != 40 or any(
        character not in "0123456789abcdef" for character in git_commit
    ):
        raise ValueError("causal provenance execution.git_commit must be 40 lowercase hex digits")

    assertions = manifest.get("causal_estimator_output")
    if not isinstance(assertions, dict):
        raise ValueError("causal provenance manifest is missing causal_estimator_output")
    required_exact = {
        "online_forward_filter": True,
        "future_samples_used": False,
        "truth_seeded_initialization": False,
        "truth_derived_stationarity": False,
    }
    for name, expected in required_exact.items():
        if assertions.get(name) is not expected:
            raise ValueError(f"causal provenance assertion {name} must be {expected}")
    allowed_initialization = {
        "causal_sensor_alignment",
        "causal_external_runtime_seed_without_truth",
    }
    if assertions.get("initialization_source") not in allowed_initialization:
        raise ValueError("initialization_source is not an approved causal source")
    if assertions.get("stationarity_source") not in {"causal_detector", "not_used"}:
        raise ValueError("stationarity_source is not causal or explicitly unused")

    return {
        "manifest": _file_fingerprint(manifest_path),
        "sources": actual_sources,
        "execution": {
            "artifacts": actual_artifacts,
            "command": command,
            "git_commit": git_commit,
        },
        "causal_estimator_output": {
            name: assertions[name]
            for name in (
                "online_forward_filter",
                "future_samples_used",
                "initialization_source",
                "truth_seeded_initialization",
                "stationarity_source",
                "truth_derived_stationarity",
            )
        },
        "evidence_boundary": (
            "artifact_identity_and_declared_causal_assertions_checked;runner_semantics_not_attested"
        ),
        "validation": "passed_artifact_binding_and_causal_assertion_checks_v2",
    }


def _required_float(rows: Sequence[Mapping[str, str]], names: Sequence[str]) -> np.ndarray:
    try:
        return np.asarray([[float(row[name]) for name in names] for row in rows], dtype=np.float64)
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"missing or invalid required columns: {', '.join(names)}") from error


def load_csv_pair(replay_path: Path, results_path: Path) -> ExcitationSamples:
    """Load an exact timestamp-paired replay/results set through explicit allowlists."""

    replay = _read_rows(replay_path)
    results = _read_rows(results_path)
    if len(replay) != len(results):
        raise ValueError("replay and results row counts differ")
    replay_timestamp = _required_float(replay, ("ts_us",))[:, 0]
    result_timestamp = _required_float(results, ("ts_us",))[:, 0]
    if not np.array_equal(replay_timestamp, result_timestamp):
        raise ValueError("replay and results timestamps are not exactly paired")

    acceleration = _required_float(
        replay, ("raw_acc_mg_x", "raw_acc_mg_y", "raw_acc_mg_z")
    ) * (9.80665 / 1000.0)
    angular_rate = _required_float(
        replay, ("raw_gyro_mdps_x", "raw_gyro_mdps_y", "raw_gyro_mdps_z")
    ) * (math.pi / 180000.0)
    quaternion = _required_float(results, ("eskf_q_w", "eskf_q_x", "eskf_q_y", "eskf_q_z"))
    accel_bias = _required_float(
        results,
        ("eskf_accel_bias_x_m_s2", "eskf_accel_bias_y_m_s2", "eskf_accel_bias_z_m_s2"),
    )
    gyro_bias = _required_float(
        results,
        ("eskf_gyro_bias_x_rad_s", "eskf_gyro_bias_y_rad_s", "eskf_gyro_bias_z_rad_s"),
    )
    replay_position_attempted = _required_float(replay, ("position_update",))[:, 0] > 0.5
    replay_velocity_attempted = replay_position_attempted.copy()
    if replay and "gps_position_update" in replay[0]:
        replay_position_attempted |= _required_float(replay, ("gps_position_update",))[:, 0] > 0.5
    if replay and "gps_velocity_update" in replay[0]:
        replay_velocity_attempted |= _required_float(replay, ("gps_velocity_update",))[:, 0] > 0.5
    result_position_attempted = _required_float(results, ("input_position_update",))[:, 0] > 0.5
    if results and "input_velocity_update" in results[0]:
        result_velocity_attempted = (
            _required_float(results, ("input_velocity_update",))[:, 0] > 0.5
        )
    else:
        # Results produced before position/velocity attempt flags were split
        # used input_position_update for the paired GPS observation.
        result_velocity_attempted = result_position_attempted.copy()
    if not np.array_equal(replay_position_attempted, result_position_attempted):
        raise ValueError("replay/results position-attempt flags differ")
    if not np.array_equal(replay_velocity_attempted, result_velocity_attempted):
        raise ValueError("replay/results velocity-attempt flags differ")
    position_accepted = result_position_attempted & (
        _required_float(results, ("eskf_position_accepted",))[:, 0] > 0.5
    )
    velocity_accepted = result_velocity_attempted & (
        _required_float(results, ("eskf_velocity_accepted",))[:, 0] > 0.5
    )
    return ExcitationSamples(
        timestamp_s=replay_timestamp * 1.0e-6,
        acceleration_body_m_s2=acceleration,
        angular_rate_body_rad_s=angular_rate,
        estimated_quaternion_wxyz=quaternion,
        estimated_accel_bias_m_s2=accel_bias,
        estimated_gyro_bias_rad_s=gyro_bias,
        position_attempted=replay_position_attempted,
        position_accepted=position_accepted,
        position_variance_m2=_required_float(replay, ("gps_position_variance_m2",))[:, 0],
        velocity_attempted=replay_velocity_attempted,
        velocity_accepted=velocity_accepted,
        velocity_variance_m2_s2=_required_float(
            replay, ("gps_velocity_variance_m2_s2",)
        )[:, 0],
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--replay", type=Path, required=True)
    parser.add_argument("--results", type=Path, required=True)
    parser.add_argument("--provenance-manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--window-duration-s", type=float, default=20.0)
    parser.add_argument("--evaluation-interval-s", type=float, default=0.5)
    args = parser.parse_args()
    provenance = validate_causal_provenance(
        args.provenance_manifest, args.replay, args.results
    )
    samples = load_csv_pair(args.replay, args.results)
    report = analyze_causal_timeline(
        samples,
        window_duration_s=args.window_duration_s,
        evaluation_interval_s=args.evaluation_interval_s,
    )
    report["causal_upstream_provenance"] = provenance
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
