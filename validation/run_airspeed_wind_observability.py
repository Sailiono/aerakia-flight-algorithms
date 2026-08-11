#!/usr/bin/env python3
"""Validation-only causal TAS/horizontal-wind observability oracle.

This module deliberately does not call the production ESKF and has no public
algorithm API.  It answers a narrower prerequisite: given timestamped,
qualified true airspeed and fresh GNSS ground velocity, does the *causal*
two-dimensional wind information have enough rank and conditioning to justify
opening a separate 17-error-state experiment?  Hidden truth is used only by
the scorer after the oracle has consumed a measurement stream.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import subprocess
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PROTOCOL = ROOT / "validation" / "airspeed_wind_observability_protocol_v1.json"


@dataclass(frozen=True)
class TasWindObservation:
    """Runtime-available inputs; intentionally contains no truth field."""

    tas_timestamp_us: int
    arrival_timestamp_us: int
    gnss_timestamp_us: int | None
    tas_m_s: float
    tas_variance_m2_s2: float
    gnss_velocity_ned_m_s: tuple[float, float, float] | None
    gnss_velocity_variance_m2_s2: tuple[float, float, float] | None
    flight_regime: str
    source_qualified: bool = True
    pitot_blocked: bool = False
    pitot_stalled: bool = False
    rotor_wash: bool = False
    sideslip_qualified: bool = True


@dataclass(frozen=True)
class WindTruthSample:
    """Offline-only synthetic truth.  Never passed to CausalWindOracle."""

    wind_ne_m_s: tuple[float, float]


@dataclass(frozen=True)
class OracleEvent:
    accepted: bool
    reason: str
    information_added: float
    innovation_nis: float | None
    estimate_ne_m_s: tuple[float, float]
    terminal_status: str


def canonical_sha256(value: object) -> str:
    return hashlib.sha256(
        json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode("utf-8")
    ).hexdigest()


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


def load_protocol(path: Path = DEFAULT_PROTOCOL) -> dict[str, Any]:
    protocol = json.loads(path.read_text(encoding="utf-8"))
    if protocol.get("schema_version") != 1:
        raise ValueError("unsupported airspeed/wind protocol schema")
    if protocol.get("status") != "frozen-validation-only":
        raise ValueError("airspeed/wind protocol must remain frozen-validation-only")
    if protocol.get("scope", {}).get("production_eskf") != "unchanged_16_nominal_15_error_state":
        raise ValueError("protocol must explicitly keep the production ESKF unchanged")
    source = protocol.get("source_contract", {})
    oracle = protocol.get("causal_oracle", {})
    required_source = (
        "minimum_tas_m_s", "maximum_delivery_delay_s", "maximum_gnss_age_s", "valid_flight_regimes"
    )
    required_oracle = (
        "initial_wind_prior_std_m_s", "maximum_gauss_newton_iterations",
        "maximum_post_qualification_nis", "maximum_consecutive_post_qualification_innovation_rejections",
        "automatic_recovery_after_source_latch", "minimum_accepted_measurements",
        "minimum_measurement_information_eigenvalue",
        "maximum_measurement_information_condition_number",
        "minimum_air_relative_direction_separation_deg",
        "minimum_distinct_air_relative_direction_clusters",
        "air_relative_direction_cluster_merge_deg",
        "minimum_bootstrap_measurements_after_direction_completion",
        "maximum_fresh_observation_age_s",
    )
    if any(name not in source for name in required_source):
        raise ValueError("source contract is incomplete")
    if any(name not in oracle for name in required_oracle):
        raise ValueError("causal oracle contract is incomplete")
    if not protocol.get("scenario_matrix"):
        raise ValueError("scenario matrix must not be empty")
    if bool(oracle["automatic_recovery_after_source_latch"]):
        raise ValueError("validation-only source latch must not recover automatically")
    return protocol


def as_vector3(value: tuple[float, float, float] | None, *, name: str) -> np.ndarray:
    if value is None:
        raise ValueError(f"{name} is missing")
    vector = np.asarray(value, dtype=np.float64)
    if vector.shape != (3,) or not np.all(np.isfinite(vector)):
        raise ValueError(f"{name} must be a finite 3-vector")
    return vector


def tas_prediction(ground_velocity_ned_m_s: np.ndarray, wind_ne_m_s: np.ndarray) -> float:
    ground = np.asarray(ground_velocity_ned_m_s, dtype=np.float64)
    wind = np.asarray(wind_ne_m_s, dtype=np.float64)
    if ground.shape != (3,) or wind.shape != (2,):
        raise ValueError("ground velocity must be 3D and wind must be horizontal 2D")
    relative = ground - np.array((wind[0], wind[1], 0.0), dtype=np.float64)
    prediction = float(np.linalg.norm(relative))
    if not math.isfinite(prediction) or prediction <= 1.0e-9:
        raise ValueError("TAS Jacobian is undefined at zero relative air velocity")
    return prediction


def tas_jacobian_wind_ne(ground_velocity_ned_m_s: np.ndarray, wind_ne_m_s: np.ndarray) -> np.ndarray:
    ground = np.asarray(ground_velocity_ned_m_s, dtype=np.float64)
    wind = np.asarray(wind_ne_m_s, dtype=np.float64)
    prediction = tas_prediction(ground, wind)
    relative = ground - np.array((wind[0], wind[1], 0.0), dtype=np.float64)
    return -relative[:2] / prediction


def finite_difference_tas_jacobian(
    ground_velocity_ned_m_s: np.ndarray,
    wind_ne_m_s: np.ndarray,
    *,
    epsilon_m_s: float = 1.0e-6,
) -> np.ndarray:
    if not math.isfinite(epsilon_m_s) or epsilon_m_s <= 0.0:
        raise ValueError("finite difference epsilon must be positive and finite")
    wind = np.asarray(wind_ne_m_s, dtype=np.float64)
    derivative = np.empty(2, dtype=np.float64)
    for index in range(2):
        positive = wind.copy()
        negative = wind.copy()
        positive[index] += epsilon_m_s
        negative[index] -= epsilon_m_s
        derivative[index] = (
            tas_prediction(ground_velocity_ned_m_s, positive)
            - tas_prediction(ground_velocity_ned_m_s, negative)
        ) / (2.0 * epsilon_m_s)
    return derivative


def effective_tas_variance(
    ground_velocity_ned_m_s: np.ndarray,
    wind_ne_m_s: np.ndarray,
    tas_variance_m2_s2: float,
    gnss_velocity_variance_m2_s2: np.ndarray,
) -> float:
    if not math.isfinite(tas_variance_m2_s2) or tas_variance_m2_s2 <= 0.0:
        raise ValueError("TAS variance must be positive and finite")
    velocity_variance = np.asarray(gnss_velocity_variance_m2_s2, dtype=np.float64)
    if velocity_variance.shape != (3,) or not np.all(np.isfinite(velocity_variance)) or np.any(velocity_variance <= 0.0):
        raise ValueError("GNSS velocity variance must be a positive finite 3-vector")
    ground = np.asarray(ground_velocity_ned_m_s, dtype=np.float64)
    wind = np.asarray(wind_ne_m_s, dtype=np.float64)
    prediction = tas_prediction(ground, wind)
    relative = ground - np.array((wind[0], wind[1], 0.0), dtype=np.float64)
    velocity_jacobian = relative / prediction
    variance = float(tas_variance_m2_s2 + np.dot(velocity_jacobian * velocity_jacobian, velocity_variance))
    if not math.isfinite(variance) or variance <= 0.0:
        raise ValueError("effective TAS variance is invalid")
    return variance


def acute_direction_separation_deg(directions: Iterable[np.ndarray]) -> float:
    vectors = [np.asarray(item, dtype=np.float64) for item in directions]
    maximum = 0.0
    for index, first in enumerate(vectors):
        first_norm = float(np.linalg.norm(first))
        if first_norm <= 1.0e-12:
            continue
        for second in vectors[index + 1:]:
            second_norm = float(np.linalg.norm(second))
            if second_norm <= 1.0e-12:
                continue
            cosine = float(np.clip(np.dot(first, second) / (first_norm * second_norm), -1.0, 1.0))
            angle = math.degrees(math.acos(cosine))
            maximum = max(maximum, min(angle, 180.0 - angle))
    return maximum


def direction_cluster_count(directions: Iterable[np.ndarray], *, merge_deg: float) -> int:
    """Count full-circle air-relative direction clusters without using truth.

    Local rank two is insufficient to remove the two-circle range ambiguity of
    scalar TAS.  Requiring three separated direction clusters is intentionally
    stricter than a two-dimensional information-rank check.
    """

    if not math.isfinite(merge_deg) or not (0.0 < merge_deg < 90.0):
        raise ValueError("direction-cluster merge angle must be in (0, 90) degrees")
    clusters: list[float] = []
    for direction in directions:
        vector = np.asarray(direction, dtype=np.float64)
        if vector.shape != (2,) or not np.all(np.isfinite(vector)):
            continue
        norm = float(np.linalg.norm(vector))
        if norm <= 1.0e-12:
            continue
        angle = math.degrees(math.atan2(float(vector[1]), float(vector[0]))) % 360.0
        if all(min(abs(angle - other), 360.0 - abs(angle - other)) > merge_deg for other in clusters):
            clusters.append(angle)
    return len(clusters)


def source_rejection_reason(
    observation: TasWindObservation,
    source_contract: dict[str, Any],
    *,
    last_tas_timestamp_us: int | None,
) -> str | None:
    if observation.tas_timestamp_us < 0 or observation.arrival_timestamp_us < 0:
        return "negative_timestamp"
    if observation.arrival_timestamp_us < observation.tas_timestamp_us:
        return "tas_timestamp_in_future"
    if last_tas_timestamp_us is not None and observation.tas_timestamp_us <= last_tas_timestamp_us:
        return "tas_timestamp_not_monotonic"
    delivery_delay_s = (observation.arrival_timestamp_us - observation.tas_timestamp_us) * 1.0e-6
    if delivery_delay_s > float(source_contract["maximum_delivery_delay_s"]):
        return "tas_delivery_delay"
    if observation.gnss_timestamp_us is None or observation.gnss_velocity_ned_m_s is None or observation.gnss_velocity_variance_m2_s2 is None:
        return "missing_gnss_velocity"
    if observation.gnss_timestamp_us < 0:
        return "negative_gnss_timestamp"
    if observation.gnss_timestamp_us > observation.tas_timestamp_us:
        return "gnss_timestamp_in_future"
    gnss_age_s = (observation.tas_timestamp_us - observation.gnss_timestamp_us) * 1.0e-6
    if gnss_age_s > float(source_contract["maximum_gnss_age_s"]):
        return "stale_gnss_velocity"
    if not math.isfinite(observation.tas_m_s) or not math.isfinite(observation.tas_variance_m2_s2):
        return "nonfinite_tas"
    if observation.tas_variance_m2_s2 <= 0.0:
        return "invalid_tas_variance"
    if observation.flight_regime not in set(source_contract["valid_flight_regimes"]):
        return "flight_regime"
    if observation.tas_m_s < float(source_contract["minimum_tas_m_s"]):
        return "low_tas"
    try:
        as_vector3(observation.gnss_velocity_ned_m_s, name="GNSS velocity")
        variance = as_vector3(observation.gnss_velocity_variance_m2_s2, name="GNSS velocity variance")
    except ValueError:
        return "invalid_gnss_velocity"
    if np.any(variance <= 0.0):
        return "invalid_gnss_variance"
    if not observation.source_qualified:
        return "source_unqualified"
    if observation.pitot_blocked:
        return "pitot_blocked"
    if observation.pitot_stalled:
        return "pitot_stalled"
    if observation.rotor_wash:
        return "rotor_wash"
    if not observation.sideslip_qualified:
        return "sideslip_unqualified"
    return None


class CausalWindOracle:
    """Causal information/least-squares oracle with no access to truth."""

    def __init__(self, protocol: dict[str, Any]) -> None:
        self._source = protocol["source_contract"]
        self._config = protocol["causal_oracle"]
        prior_std = float(self._config["initial_wind_prior_std_m_s"])
        if not math.isfinite(prior_std) or prior_std <= 0.0:
            raise ValueError("wind prior standard deviation must be positive and finite")
        self._prior_information = np.eye(2, dtype=np.float64) / (prior_std * prior_std)
        self._estimate = np.zeros(2, dtype=np.float64)
        self._accepted: list[TasWindObservation] = []
        self._last_seen_tas_timestamp_us: int | None = None
        self._last_accepted_arrival_timestamp_us: int | None = None
        self._rejections: Counter[str] = Counter()
        self._last_information = np.zeros((2, 2), dtype=np.float64)
        self._last_covariance = np.eye(2, dtype=np.float64) * prior_std * prior_std
        self._last_directions = np.empty((0, 2), dtype=np.float64)
        self._last_geometry = (0, 0.0, math.inf, 0.0, 0)
        self._consecutive_innovation_rejections = 0
        self._source_latched = False
        self._direction_completion_accepted_count: int | None = None
        self._range_difference_seeded = False

    @property
    def estimate_ne_m_s(self) -> np.ndarray:
        return self._estimate.copy()

    @property
    def accepted_count(self) -> int:
        return len(self._accepted)

    @property
    def rejection_counts(self) -> dict[str, int]:
        return dict(sorted(self._rejections.items()))

    def _measurement_terms(
        self,
        observation: TasWindObservation,
        wind_ne_m_s: np.ndarray,
    ) -> tuple[np.ndarray, float, float, float]:
        velocity = as_vector3(observation.gnss_velocity_ned_m_s, name="GNSS velocity")
        variance = as_vector3(observation.gnss_velocity_variance_m2_s2, name="GNSS velocity variance")
        prediction = tas_prediction(velocity, wind_ne_m_s)
        jacobian = tas_jacobian_wind_ne(velocity, wind_ne_m_s)
        effective_variance = effective_tas_variance(
            velocity, wind_ne_m_s, observation.tas_variance_m2_s2, variance
        )
        return jacobian, prediction, effective_variance, observation.tas_m_s - prediction

    def _solve_from_accepted(self) -> None:
        if not self._accepted:
            return
        estimate = self._estimate.copy()
        iterations = int(self._config["maximum_gauss_newton_iterations"])
        information = self._prior_information.copy()
        velocities = np.asarray(
            [as_vector3(item.gnss_velocity_ned_m_s, name="GNSS velocity") for item in self._accepted],
            dtype=np.float64,
        )
        velocity_variances = np.asarray(
            [
                as_vector3(item.gnss_velocity_variance_m2_s2, name="GNSS velocity variance")
                for item in self._accepted
            ],
            dtype=np.float64,
        )
        tas = np.asarray([item.tas_m_s for item in self._accepted], dtype=np.float64)
        tas_variances = np.asarray(
            [item.tas_variance_m2_s2 for item in self._accepted], dtype=np.float64
        )

        # A scalar speed range from two velocity centres has a mirror
        # ambiguity.  Once the causal stream supplies a third non-collinear
        # centre, squared-range differencing gives a truth-free linear seed.
        # It is only an initializer; final residuals/information still use the
        # original unsquared TAS model and its propagated variance.
        if (
            not self._range_difference_seeded
            and len(self._accepted) >= 3
            and direction_cluster_count(
                velocities[:, :2],
                merge_deg=float(self._config["air_relative_direction_cluster_merge_deg"]),
            ) >= int(self._config["minimum_distinct_air_relative_direction_clusters"])
        ):
            reference_velocity = velocities[0]
            reference_tas = tas[0]
            design = 2.0 * (velocities[1:, :2] - reference_velocity[:2])
            right_hand = (
                np.sum(velocities[1:] * velocities[1:], axis=1)
                - float(np.dot(reference_velocity, reference_velocity))
                - tas[1:] * tas[1:]
                + reference_tas * reference_tas
            )
            if design.size:
                singular_values = np.linalg.svd(design, compute_uv=False)
                if (
                    len(singular_values) == 2
                    # Random GNSS noise makes a same-heading design formally
                    # rank two.  A seed is permitted only after a materially
                    # conditioned pair of velocity-centre directions exists.
                    and singular_values[1] > singular_values[0] * 0.10
                ):
                    seeded, *_ = np.linalg.lstsq(design, right_hand, rcond=None)
                    if np.all(np.isfinite(seeded)):
                        estimate = seeded.astype(np.float64)
                        self._range_difference_seeded = True

        def batch_terms(candidate: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
            relative = velocities - np.array((candidate[0], candidate[1], 0.0), dtype=np.float64)
            predictions = np.linalg.norm(relative, axis=1)
            if np.any(~np.isfinite(predictions)) or np.any(predictions <= 1.0e-9):
                raise RuntimeError("wind least-squares reached zero/non-finite relative air velocity")
            jacobians = -relative[:, :2] / predictions[:, None]
            velocity_jacobians = relative / predictions[:, None]
            effective_variances = tas_variances + np.sum(
                velocity_jacobians * velocity_jacobians * velocity_variances, axis=1
            )
            if np.any(~np.isfinite(effective_variances)) or np.any(effective_variances <= 0.0):
                raise RuntimeError("wind least-squares reached invalid effective variance")
            return jacobians, effective_variances, tas - predictions

        for _ in range(iterations):
            information = self._prior_information.copy()
            jacobians, effective_variances, residuals = batch_terms(estimate)
            weights = 1.0 / effective_variances
            information += jacobians.T @ (jacobians * weights[:, None])
            right_hand = jacobians.T @ (weights * (residuals + jacobians @ estimate))
            candidate = np.linalg.solve(information, right_hand)
            if not np.all(np.isfinite(candidate)):
                raise RuntimeError("wind least-squares produced a non-finite estimate")
            if float(np.linalg.norm(candidate - estimate)) <= 1.0e-10:
                estimate = candidate
                break
            estimate = candidate
        jacobians, effective_variances, _residuals = batch_terms(estimate)
        measurement_information = jacobians.T @ (jacobians * (1.0 / effective_variances)[:, None])
        total_information = self._prior_information + measurement_information
        covariance = np.linalg.inv(total_information)
        if not np.all(np.isfinite(covariance)):
            raise RuntimeError("wind covariance is non-finite")
        self._estimate = estimate
        self._last_information = 0.5 * (measurement_information + measurement_information.T)
        self._last_covariance = 0.5 * (covariance + covariance.T)
        self._last_directions = -jacobians
        eigenvalues = np.linalg.eigvalsh(self._last_information)
        maximum = float(eigenvalues[-1])
        minimum = float(eigenvalues[0])
        rank_tolerance = max(1.0, maximum) * 1.0e-10
        rank = int(np.count_nonzero(eigenvalues > rank_tolerance))
        condition = math.inf if minimum <= rank_tolerance else maximum / minimum
        self._last_geometry = (
            rank,
            minimum,
            condition,
            acute_direction_separation_deg(self._last_directions),
            direction_cluster_count(
                self._last_directions,
                merge_deg=float(self._config["air_relative_direction_cluster_merge_deg"]),
            ),
        )
        if (
            self._direction_completion_accepted_count is None
            and self._last_geometry[4] >= int(self._config["minimum_distinct_air_relative_direction_clusters"])
        ):
            self._direction_completion_accepted_count = len(self._accepted)

    def _observable_geometry(self) -> tuple[int, float, float, float, int]:
        return self._last_geometry

    def _geometry_qualified(self) -> bool:
        rank, minimum_eigenvalue, condition, separation_deg, direction_clusters = self._observable_geometry()
        bootstrap_count = int(self._config["minimum_bootstrap_measurements_after_direction_completion"])
        post_completion = (
            -1 if self._direction_completion_accepted_count is None
            else len(self._accepted) - self._direction_completion_accepted_count
        )
        return (
            len(self._accepted) >= int(self._config["minimum_accepted_measurements"])
            and rank >= 2
            and minimum_eigenvalue >= float(self._config["minimum_measurement_information_eigenvalue"])
            and condition <= float(self._config["maximum_measurement_information_condition_number"])
            and separation_deg >= float(self._config["minimum_air_relative_direction_separation_deg"])
            and direction_clusters >= int(self._config["minimum_distinct_air_relative_direction_clusters"])
            and post_completion >= bootstrap_count
        )

    def terminal_status(self, now_timestamp_us: int) -> str:
        if now_timestamp_us < 0:
            raise ValueError("current time must be non-negative")
        if self._source_latched:
            return "source_latched"
        if self._last_accepted_arrival_timestamp_us is None:
            return "not_observable"
        max_age_us = int(round(float(self._config["maximum_fresh_observation_age_s"]) * 1.0e6))
        if now_timestamp_us - self._last_accepted_arrival_timestamp_us > max_age_us:
            return "stale_no_fresh_gnss_tas"
        if not self._geometry_qualified():
            return "not_observable"
        return "qualified"

    def step(self, observation: TasWindObservation) -> OracleEvent:
        reason = source_rejection_reason(
            observation, self._source, last_tas_timestamp_us=self._last_seen_tas_timestamp_us
        )
        if observation.tas_timestamp_us >= 0 and (
            self._last_seen_tas_timestamp_us is None
            or observation.tas_timestamp_us > self._last_seen_tas_timestamp_us
        ):
            self._last_seen_tas_timestamp_us = observation.tas_timestamp_us
        if reason is not None:
            self._rejections[reason] += 1
            return OracleEvent(
                False, reason, 0.0, None, tuple(float(value) for value in self._estimate),
                self.terminal_status(observation.arrival_timestamp_us),
            )

        if self._source_latched:
            self._rejections["source_latched"] += 1
            return OracleEvent(
                False, "source_latched", 0.0, None,
                tuple(float(value) for value in self._estimate), "source_latched",
            )

        jacobian, _prediction, variance, residual = self._measurement_terms(observation, self._estimate)
        innovation_nis = residual * residual / variance
        if self._geometry_qualified() and innovation_nis > float(self._config["maximum_post_qualification_nis"]):
            self._rejections["innovation_nis"] += 1
            self._consecutive_innovation_rejections += 1
            if self._consecutive_innovation_rejections >= int(
                self._config["maximum_consecutive_post_qualification_innovation_rejections"]
            ):
                self._source_latched = True
            return OracleEvent(
                False, "innovation_nis", 0.0, float(innovation_nis),
                tuple(float(value) for value in self._estimate),
                self.terminal_status(observation.arrival_timestamp_us),
            )

        information_added = float(np.trace(np.outer(jacobian, jacobian) / variance))
        self._accepted.append(observation)
        self._consecutive_innovation_rejections = 0
        self._last_accepted_arrival_timestamp_us = observation.arrival_timestamp_us
        self._solve_from_accepted()
        return OracleEvent(
            True, "accepted", information_added, float(innovation_nis),
            tuple(float(value) for value in self._estimate),
            self.terminal_status(observation.arrival_timestamp_us),
        )

    def summary(self, now_timestamp_us: int) -> dict[str, object]:
        rank, minimum_eigenvalue, condition, separation_deg, direction_clusters = self._observable_geometry()
        return {
            "terminal_status": self.terminal_status(now_timestamp_us),
            "accepted_measurements": len(self._accepted),
            "rejection_counts": self.rejection_counts,
            "estimate_ne_m_s": [float(value) for value in self._estimate],
            "wind_covariance_m2_s2": self._last_covariance.tolist(),
            "measurement_information": self._last_information.tolist(),
            "measurement_information_rank": rank,
            "measurement_information_minimum_eigenvalue": minimum_eigenvalue,
            "measurement_information_condition_number": condition,
            "maximum_acute_air_relative_direction_separation_deg": separation_deg,
            "distinct_air_relative_direction_clusters": direction_clusters,
            "direction_completion_accepted_measurement": self._direction_completion_accepted_count,
            "geometry_qualified_before_freshness": self._geometry_qualified(),
            "range_difference_initializer_used": self._range_difference_seeded,
            "last_accepted_arrival_timestamp_us": self._last_accepted_arrival_timestamp_us,
            "source_latched": self._source_latched,
        }


def known_wind_upper_bound(
    observations: Iterable[TasWindObservation],
    truths: Iterable[WindTruthSample],
    protocol: dict[str, Any],
) -> dict[str, object]:
    """Offline upper-bound model check; known wind is intentionally external."""

    observations_list = list(observations)
    truths_list = list(truths)
    if len(observations_list) != len(truths_list):
        raise ValueError("known-wind observations and truth must have equal lengths")
    source = protocol["source_contract"]
    residuals: list[float] = []
    nis_values: list[float] = []
    finite_difference_errors: list[float] = []
    rejections: Counter[str] = Counter()
    last_tas_timestamp_us: int | None = None
    for observation, truth in zip(observations_list, truths_list, strict=True):
        reason = source_rejection_reason(
            observation, source, last_tas_timestamp_us=last_tas_timestamp_us
        )
        if observation.tas_timestamp_us >= 0 and (
            last_tas_timestamp_us is None or observation.tas_timestamp_us > last_tas_timestamp_us
        ):
            last_tas_timestamp_us = observation.tas_timestamp_us
        if reason is not None:
            rejections[reason] += 1
            continue
        wind = np.asarray(truth.wind_ne_m_s, dtype=np.float64)
        velocity = as_vector3(observation.gnss_velocity_ned_m_s, name="GNSS velocity")
        velocity_variance = as_vector3(
            observation.gnss_velocity_variance_m2_s2, name="GNSS velocity variance"
        )
        prediction = tas_prediction(velocity, wind)
        residual = observation.tas_m_s - prediction
        variance = effective_tas_variance(
            velocity, wind, observation.tas_variance_m2_s2, velocity_variance
        )
        jacobian = tas_jacobian_wind_ne(velocity, wind)
        finite_difference = finite_difference_tas_jacobian(velocity, wind)
        residuals.append(float(residual))
        nis_values.append(float(residual * residual / variance))
        finite_difference_errors.append(float(np.max(np.abs(jacobian - finite_difference))))
    if not residuals:
        return {
            "accepted_measurements": 0,
            "rejection_counts": dict(sorted(rejections.items())),
            "nis_mean": None,
            "residual_rmse_m_s": None,
            "maximum_jacobian_finite_difference_error": None,
        }
    return {
        "accepted_measurements": len(residuals),
        "rejection_counts": dict(sorted(rejections.items())),
        "nis_mean": float(np.mean(nis_values)),
        "residual_rmse_m_s": float(math.sqrt(float(np.mean(np.square(residuals))))),
        "maximum_jacobian_finite_difference_error": float(max(finite_difference_errors)),
    }


def _scenario_segments(name: str) -> list[float]:
    if name == "straight_line":
        return [15.0, 15.0, 15.0]
    return [0.0, 90.0, 225.0]


def _build_observation(
    *,
    timestamp_us: int,
    heading_deg: float,
    tas_true_m_s: float,
    wind_ned_m_s: np.ndarray,
    source: dict[str, Any],
    rng: np.random.Generator,
    flight_regime: str = "fixed_wing_cruise",
    source_qualified: bool = True,
    pitot_blocked: bool = False,
    pitot_stalled: bool = False,
    rotor_wash: bool = False,
    sideslip_qualified: bool = True,
    arrival_delay_s: float = 0.0,
    gnss_available: bool = True,
    tas_measurement_offset_m_s: float = 0.0,
) -> tuple[TasWindObservation, WindTruthSample]:
    heading_rad = math.radians(heading_deg)
    air_velocity = np.array((
        tas_true_m_s * math.cos(heading_rad),
        tas_true_m_s * math.sin(heading_rad),
        0.0,
    ), dtype=np.float64)
    ground_velocity = air_velocity + wind_ned_m_s
    tas_noise = float(source["tas_noise_std_m_s"])
    gnss_noise = float(source["gnss_velocity_noise_std_m_s"])
    measured_tas = float(np.linalg.norm(air_velocity) + rng.normal(0.0, tas_noise) + tas_measurement_offset_m_s)
    measured_velocity = ground_velocity + rng.normal(0.0, gnss_noise, 3)
    arrival_timestamp_us = timestamp_us + int(round(arrival_delay_s * 1.0e6))
    observation = TasWindObservation(
        tas_timestamp_us=timestamp_us,
        arrival_timestamp_us=arrival_timestamp_us,
        gnss_timestamp_us=timestamp_us if gnss_available else None,
        tas_m_s=measured_tas,
        tas_variance_m2_s2=tas_noise * tas_noise,
        gnss_velocity_ned_m_s=tuple(float(value) for value in measured_velocity) if gnss_available else None,
        gnss_velocity_variance_m2_s2=(gnss_noise * gnss_noise,) * 3 if gnss_available else None,
        flight_regime=flight_regime,
        source_qualified=source_qualified,
        pitot_blocked=pitot_blocked,
        pitot_stalled=pitot_stalled,
        rotor_wash=rotor_wash,
        sideslip_qualified=sideslip_qualified,
    )
    return observation, WindTruthSample((float(wind_ned_m_s[0]), float(wind_ned_m_s[1])))


def generate_scenario(name: str, protocol: dict[str, Any]) -> tuple[list[TasWindObservation], list[WindTruthSample], int]:
    """Generate immutable measurement and truth lists; truth stays outside the oracle."""

    source = protocol["synthetic_source"]
    rate_hz = float(source["sample_rate_hz"])
    duration_s = float(source["segment_duration_s"])
    samples_per_segment = int(round(rate_hz * duration_s))
    if samples_per_segment <= 0:
        raise ValueError("synthetic source must produce at least one sample per segment")
    seed = int(source["seed"]) ^ int.from_bytes(hashlib.sha256(name.encode("utf-8")).digest()[:4], "big")
    rng = np.random.default_rng(seed)
    nominal_wind_ne = np.asarray(source["nominal_wind_ne_m_s"], dtype=np.float64)
    nominal_wind_ned = np.array((nominal_wind_ne[0], nominal_wind_ne[1], 0.0), dtype=np.float64)
    nominal_tas = float(source["nominal_tas_m_s"])
    observations: list[TasWindObservation] = []
    truths: list[WindTruthSample] = []
    timestamp_us = 0
    dt_us = int(round(1.0e6 / rate_hz))

    def append_segment(
        heading_deg: float,
        *,
        count: int = samples_per_segment,
        wind_ned_m_s: np.ndarray = nominal_wind_ned,
        tas_true_m_s: float = nominal_tas,
        **kwargs: Any,
    ) -> None:
        nonlocal timestamp_us
        for _ in range(count):
            observation, truth = _build_observation(
                timestamp_us=timestamp_us,
                heading_deg=heading_deg,
                tas_true_m_s=tas_true_m_s,
                wind_ned_m_s=wind_ned_m_s,
                source=source,
                rng=rng,
                **kwargs,
            )
            observations.append(observation)
            truths.append(truth)
            timestamp_us += dt_us

    if name in ("multi_heading_nominal", "straight_line"):
        for heading in _scenario_segments(name):
            append_segment(heading)
    elif name == "hover":
        for heading in _scenario_segments("multi_heading_nominal"):
            append_segment(heading, tas_true_m_s=0.2, flight_regime="hover")
    elif name == "low_tas":
        for heading in _scenario_segments("multi_heading_nominal"):
            append_segment(heading, tas_true_m_s=6.0)
    elif name == "vtol_transition":
        for heading in _scenario_segments("multi_heading_nominal"):
            append_segment(heading, flight_regime="vtol_transition")
    elif name == "rotor_wash":
        for heading in _scenario_segments("multi_heading_nominal"):
            append_segment(heading, rotor_wash=True)
    elif name == "sideslip":
        for heading in _scenario_segments("multi_heading_nominal"):
            append_segment(heading, sideslip_qualified=False)
    elif name == "blocked_pitot":
        for heading in _scenario_segments("multi_heading_nominal"):
            append_segment(heading, pitot_blocked=True)
    elif name == "stalled_pitot":
        for heading in _scenario_segments("multi_heading_nominal"):
            append_segment(heading, pitot_stalled=True)
    elif name == "delayed_tas":
        for heading in _scenario_segments("multi_heading_nominal"):
            append_segment(heading, arrival_delay_s=0.6)
    elif name == "reordered_tas":
        for heading in _scenario_segments("multi_heading_nominal"):
            append_segment(heading)
        if len(observations) < 2:
            raise RuntimeError("reordered scenario needs at least two samples")
        last = observations[-1]
        previous = observations[-2]
        observations[-1] = TasWindObservation(
            tas_timestamp_us=previous.tas_timestamp_us,
            arrival_timestamp_us=last.arrival_timestamp_us,
            gnss_timestamp_us=previous.gnss_timestamp_us,
            tas_m_s=last.tas_m_s,
            tas_variance_m2_s2=last.tas_variance_m2_s2,
            gnss_velocity_ned_m_s=last.gnss_velocity_ned_m_s,
            gnss_velocity_variance_m2_s2=last.gnss_velocity_variance_m2_s2,
            flight_regime=last.flight_regime,
        )
    elif name == "gnss_outage":
        for heading in _scenario_segments("multi_heading_nominal"):
            append_segment(heading)
        append_segment(0.0, count=int(round(rate_hz * 2.0)), gnss_available=False)
    elif name == "wind_shear":
        for heading in _scenario_segments("multi_heading_nominal"):
            append_segment(heading)
        shear_wind = np.array((12.0, nominal_wind_ne[1], 0.0), dtype=np.float64)
        append_segment(45.0, count=int(round(rate_hz * 2.0)), wind_ned_m_s=shear_wind)
    elif name == "tas_scale_bias":
        for heading in _scenario_segments("multi_heading_nominal"):
            append_segment(heading)
        append_segment(45.0, count=int(round(rate_hz * 2.0)), tas_measurement_offset_m_s=4.0)
    elif name == "vertical_wind":
        for heading in _scenario_segments("multi_heading_nominal"):
            append_segment(heading)
        vertical_wind = np.array((nominal_wind_ne[0], nominal_wind_ne[1], 12.0), dtype=np.float64)
        append_segment(45.0, count=int(round(rate_hz * 2.0)), wind_ned_m_s=vertical_wind)
    else:
        raise ValueError(f"unknown airspeed/wind scenario: {name}")
    if not observations:
        raise RuntimeError("scenario produced no observations")
    final_now_us = max(observation.arrival_timestamp_us for observation in observations)
    return observations, truths, final_now_us


def score_causal_oracle(
    oracle_summary: dict[str, object],
    terminal_truth: WindTruthSample,
) -> dict[str, object]:
    estimate = np.asarray(oracle_summary["estimate_ne_m_s"], dtype=np.float64)
    truth = np.asarray(terminal_truth.wind_ne_m_s, dtype=np.float64)
    covariance = np.asarray(oracle_summary["wind_covariance_m2_s2"], dtype=np.float64)
    error = estimate - truth
    nees = float(error @ np.linalg.solve(covariance, error))
    return {
        "terminal_wind_truth_ne_m_s": [float(value) for value in truth],
        "terminal_wind_error_ne_m_s": [float(value) for value in error],
        "terminal_wind_error_norm_m_s": float(np.linalg.norm(error)),
        "terminal_wind_nees_2d_offline_truth_scored": nees,
    }


def evaluate_case(case: dict[str, Any], protocol: dict[str, Any]) -> dict[str, object]:
    name = str(case["name"])
    observations, truths, final_now_us = generate_scenario(name, protocol)
    oracle = CausalWindOracle(protocol)
    events = [oracle.step(observation) for observation in observations]
    causal = oracle.summary(final_now_us)
    causal_score = score_causal_oracle(causal, truths[-1])
    known = known_wind_upper_bound(observations, truths, protocol)
    errors: list[str] = []
    expected_status = str(case["expected_terminal_status"])
    if causal["terminal_status"] != expected_status:
        errors.append(f"terminal status {causal['terminal_status']} != {expected_status}")
    if case["role"] == "positive":
        maximum_error = float(case["maximum_terminal_wind_error_m_s"])
        if float(causal_score["terminal_wind_error_norm_m_s"]) > maximum_error:
            errors.append("terminal wind error exceeds frozen positive bound")
        minimum_nis, maximum_nis = [float(value) for value in case["known_wind_nis_mean_range"]]
        nis_mean = known["nis_mean"]
        if nis_mean is None or not (minimum_nis <= float(nis_mean) <= maximum_nis):
            errors.append("known-wind nominal NIS mean is outside frozen range")
        derivative_error = known["maximum_jacobian_finite_difference_error"]
        if derivative_error is None or float(derivative_error) > 2.0e-6:
            errors.append("analytic TAS Jacobian disagrees with finite difference")
    else:
        if case["role"] == "negative" and causal["terminal_status"] == "qualified":
            errors.append("negative control ended qualified")
        rejections = causal["rejection_counts"]
        assert isinstance(rejections, dict)
        for reason in case["required_rejection_reasons"]:
            if int(rejections.get(reason, 0)) <= 0:
                errors.append(f"required rejection {reason} was absent")
    return {
        "name": name,
        "role": case["role"],
        "passed": not errors,
        "errors": errors,
        "input_observations": len(observations),
        "accepted_events": sum(event.accepted for event in events),
        "rejected_events": sum(not event.accepted for event in events),
        "causal_two_state_wind_oracle": causal,
        "offline_truth_score": causal_score,
        "known_wind_upper_bound": known,
    }


def run_protocol(protocol: dict[str, Any]) -> dict[str, object]:
    cases = [evaluate_case(case, protocol) for case in protocol["scenario_matrix"]]
    return {
        "schema_version": 1,
        "study_id": "aerakia-airspeed-wind-observability-v1",
        "status": "passed" if all(bool(case["passed"]) for case in cases) else "failed",
        "scope": protocol["scope"],
        "protocol": {
            "path": str(DEFAULT_PROTOCOL.relative_to(ROOT)),
            "semantic_sha256": canonical_sha256(protocol),
            "file_sha256": file_sha256(DEFAULT_PROTOCOL),
        },
        "source_contract": protocol["source_contract"],
        "causal_oracle": protocol["causal_oracle"],
        "cases": cases,
        "totals": {
            "cases": len(cases),
            "passed_cases": sum(bool(case["passed"]) for case in cases),
            "positive_cases": sum(case["role"] == "positive" for case in cases),
            "negative_cases": sum(case["role"] == "negative" for case in cases),
            "transport_control_cases": sum(case["role"] == "transport_control" for case in cases),
            "input_observations": sum(int(case["input_observations"]) for case in cases),
        },
        "provenance": {
            "runner_sha256": file_sha256(Path(__file__)),
            "git_commit": capture(["git", "rev-parse", "HEAD"]),
            "git_status": capture(["git", "status", "--short"]),
        },
        "limitations": [
            "The causal oracle is a host-only 2D wind information/least-squares diagnostic, not the production ESKF.",
            "The oracle observes only qualified fixed-wing TAS plus fresh GNSS velocity; it never claims new wind information during a GNSS outage.",
            "Synthetic source qualification flags stand in for physical calibration, sideslip, vertical-wind, and rotor-wash detection that must be implemented and validated privately.",
            "Truth is held outside CausalWindOracle and is used only by the final scorer.",
        ],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--protocol", type=Path, default=DEFAULT_PROTOCOL)
    parser.add_argument(
        "--out", type=Path,
        default=ROOT / "validation" / "public" / "airspeed_wind_observability_v1.json",
    )
    args = parser.parse_args()
    protocol_path = args.protocol.resolve()
    protocol = load_protocol(protocol_path)
    result = run_protocol(protocol)
    result["protocol"]["path"] = str(protocol_path.relative_to(ROOT)) if protocol_path.is_relative_to(ROOT) else str(protocol_path)
    result["protocol"]["file_sha256"] = file_sha256(protocol_path)
    output_path = args.out.resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
