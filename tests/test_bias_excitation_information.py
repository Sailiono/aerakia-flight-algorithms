from __future__ import annotations

import hashlib
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "validation"))

import analyze_bias_excitation_information as analyzer  # noqa: E402


def quaternion_multiply(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return np.asarray(
        [
            lw * rw - lx * rx - ly * ry - lz * rz,
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
        ],
        dtype=np.float64,
    )


def integrate_quaternion(timestamp: np.ndarray, angular_rate: np.ndarray) -> np.ndarray:
    quaternion = np.zeros((len(timestamp), 4), dtype=np.float64)
    quaternion[0, 0] = 1.0
    for index in range(1, len(timestamp)):
        rotation_vector = angular_rate[index] * (timestamp[index] - timestamp[index - 1])
        angle = float(np.linalg.norm(rotation_vector))
        if angle <= 1.0e-12:
            delta = np.asarray([1.0, 0.0, 0.0, 0.0])
        else:
            delta = np.concatenate(
                ([math.cos(0.5 * angle)], rotation_vector / angle * math.sin(0.5 * angle))
            )
        quaternion[index] = quaternion_multiply(quaternion[index - 1], delta)
        quaternion[index] /= np.linalg.norm(quaternion[index])
    return quaternion


def make_samples(
    mode: str,
    duration_s: float = 20.0,
    rate_hz: float = 20.0,
    aiding_rate_hz: float = 5.0,
) -> analyzer.ExcitationSamples:
    timestamp = np.arange(0.0, duration_s + 0.5 / rate_hz, 1.0 / rate_hz)
    count = len(timestamp)
    acceleration = np.zeros((count, 3), dtype=np.float64)
    acceleration[:, 2] = -9.80665
    angular_rate = np.zeros((count, 3), dtype=np.float64)
    if mode == "yaw_only":
        angular_rate[:, 2] = math.radians(30.0)
    elif mode == "single_direction":
        acceleration[:, 0] += 2.0 * np.sin(2.0 * math.pi * timestamp / 4.0)
    elif mode == "multi_direction":
        acceleration[:, 0] += 2.0 * np.sin(2.0 * math.pi * timestamp / 4.0)
        acceleration[:, 1] += 1.5 * np.sin(2.0 * math.pi * timestamp / 3.0 + 0.4)
        angular_rate[:, 0] = math.radians(12.0) * np.sin(2.0 * math.pi * timestamp / 5.0)
        angular_rate[:, 1] = math.radians(10.0) * np.sin(2.0 * math.pi * timestamp / 6.0 + 0.2)
    elif mode not in {"static", "constant_velocity"}:
        raise ValueError(mode)
    update_period = max(1, int(round(rate_hz / aiding_rate_hz)))
    attempted = np.arange(count) % update_period == 0
    return analyzer.ExcitationSamples(
        timestamp_s=timestamp,
        acceleration_body_m_s2=acceleration,
        angular_rate_body_rad_s=angular_rate,
        estimated_quaternion_wxyz=integrate_quaternion(timestamp, angular_rate),
        estimated_accel_bias_m_s2=np.zeros((count, 3), dtype=np.float64),
        estimated_gyro_bias_rad_s=np.zeros((count, 3), dtype=np.float64),
        position_attempted=attempted.copy(),
        position_accepted=attempted.copy(),
        position_variance_m2=np.full(count, 0.25, dtype=np.float64),
        velocity_attempted=attempted.copy(),
        velocity_accepted=attempted.copy(),
        velocity_variance_m2_s2=np.full(count, 0.04, dtype=np.float64),
    )


class BiasExcitationInformationTests(unittest.TestCase):
    STRUCTURAL_READY = "structural_information_ready_analyzer_only"
    STRUCTURAL_CHECKS = "structural_readiness_checks_analyzer_only"

    def test_static_is_a_negative_control(self) -> None:
        report = analyzer.analyze_window(make_samples("static"))
        self.assertLess(report["effective_rank"], 5)
        self.assertFalse(report[self.STRUCTURAL_READY])
        self.assertGreater(report["aiding_coverage"]["accepted_total"], 5)

    def test_fixed_attitude_constant_velocity_is_inertially_indistinguishable(self) -> None:
        static = make_samples("static")
        constant_velocity = make_samples("constant_velocity")
        np.testing.assert_array_equal(
            static.acceleration_body_m_s2, constant_velocity.acceleration_body_m_s2
        )
        np.testing.assert_array_equal(
            static.angular_rate_body_rad_s, constant_velocity.angular_rate_body_rad_s
        )
        report = analyzer.analyze_window(constant_velocity)
        self.assertLess(report["effective_rank"], 5)
        self.assertFalse(report[self.STRUCTURAL_READY])

    def test_yaw_only_is_a_negative_control(self) -> None:
        report = analyzer.analyze_window(make_samples("yaw_only"))
        self.assertLess(report["effective_rank"], 5)
        self.assertFalse(report[self.STRUCTURAL_READY])

    def test_single_direction_does_not_claim_full_excitation(self) -> None:
        report = analyzer.analyze_window(make_samples("single_direction"))
        self.assertLess(report["effective_rank"], 5)
        self.assertFalse(report[self.STRUCTURAL_READY])

    def test_multi_direction_attitude_rate_consistent_structural_positive_control(self) -> None:
        samples = make_samples("multi_direction")
        sample_index = 73
        dt_s = samples.timestamp_s[sample_index] - samples.timestamp_s[sample_index - 1]
        rotation_vector = samples.angular_rate_body_rad_s[sample_index] * dt_s
        angle = float(np.linalg.norm(rotation_vector))
        delta = np.concatenate(
            ([math.cos(0.5 * angle)], rotation_vector / angle * math.sin(0.5 * angle))
        )
        expected = quaternion_multiply(
            samples.estimated_quaternion_wxyz[sample_index - 1], delta
        )
        expected /= np.linalg.norm(expected)
        np.testing.assert_allclose(
            samples.estimated_quaternion_wxyz[sample_index], expected, atol=1.0e-14
        )
        report = analyzer.analyze_window(samples)
        self.assertEqual(report["effective_rank"], 5)
        self.assertTrue(report[self.STRUCTURAL_CHECKS]["full_target_rank"])
        self.assertTrue(report[self.STRUCTURAL_READY])
        self.assertGreater(report["minimum_eigenvalue"], 0.0)
        self.assertIsNotNone(report["condition_number"])
        self.assertEqual(
            set(report["per_direction_information"]),
            {"tilt_x", "tilt_y", "accel_bias_x", "accel_bias_y", "accel_bias_z"},
        )

    def test_short_full_rank_window_is_not_structurally_ready(self) -> None:
        report = analyzer.analyze_window(make_samples("multi_direction", duration_s=1.0))
        self.assertFalse(report[self.STRUCTURAL_CHECKS]["window_duration"])
        self.assertFalse(report[self.STRUCTURAL_READY])

    def test_rejected_aiding_contributes_no_information(self) -> None:
        samples = make_samples("multi_direction")
        rejected = analyzer.ExcitationSamples(
            **{
                **samples.__dict__,
                "position_accepted": np.zeros_like(samples.position_accepted),
                "velocity_accepted": np.zeros_like(samples.velocity_accepted),
            }
        )
        report = analyzer.analyze_window(rejected)
        self.assertEqual(report["effective_rank"], 0)
        self.assertEqual(report["aiding_coverage"]["accepted_total"], 0)
        self.assertFalse(report[self.STRUCTURAL_READY])

    def test_stale_aiding_is_not_structurally_ready(self) -> None:
        samples = make_samples("multi_direction")
        stale = samples.timestamp_s <= samples.timestamp_s[-1] - 2.0
        stale_samples = analyzer.ExcitationSamples(
            **{
                **samples.__dict__,
                "position_accepted": samples.position_accepted & stale,
                "velocity_accepted": samples.velocity_accepted & stale,
            }
        )
        report = analyzer.analyze_window(stale_samples)
        self.assertEqual(report["effective_rank"], 5)
        self.assertFalse(report[self.STRUCTURAL_CHECKS]["aiding_freshness"])
        self.assertFalse(report[self.STRUCTURAL_READY])

    def test_unique_epoch_gate_does_not_double_count_paired_updates(self) -> None:
        samples = make_samples("multi_direction")
        selected = np.zeros(len(samples.timestamp_s), dtype=bool)
        update_indices = np.flatnonzero(samples.position_attempted)[-4:]
        selected[update_indices] = True
        sparse = analyzer.ExcitationSamples(
            **{
                **samples.__dict__,
                "position_attempted": selected,
                "position_accepted": selected,
                "velocity_attempted": selected,
                "velocity_accepted": selected,
            }
        )
        thresholds = analyzer.InformationThresholds(
            minimum_aiding_bin_coverage=0.0,
            maximum_aiding_age_s=10.0,
        )
        report = analyzer.analyze_window(sparse, thresholds=thresholds)
        self.assertEqual(report["aiding_coverage"]["accepted_total"], 8)
        self.assertEqual(report["aiding_coverage"]["accepted_unique_timestamps"], 4)
        self.assertFalse(report[self.STRUCTURAL_CHECKS]["unique_aiding_epochs"])
        self.assertFalse(report[self.STRUCTURAL_READY])

    def test_start_index_post_update_observation_is_excluded(self) -> None:
        samples = make_samples("multi_direction")
        without_start = {
            **samples.__dict__,
            "position_attempted": samples.position_attempted.copy(),
            "position_accepted": samples.position_accepted.copy(),
            "velocity_attempted": samples.velocity_attempted.copy(),
            "velocity_accepted": samples.velocity_accepted.copy(),
        }
        for name in (
            "position_attempted", "position_accepted",
            "velocity_attempted", "velocity_accepted",
        ):
            without_start[name][0] = False
        original = analyzer.analyze_window(samples)
        excluded = analyzer.analyze_window(analyzer.ExcitationSamples(**without_start))
        np.testing.assert_array_equal(
            original["normalized_information_matrix"],
            excluded["normalized_information_matrix"],
        )
        self.assertEqual(original["aiding_coverage"], excluded["aiding_coverage"])

    def test_prefix_result_is_unchanged_by_future_samples(self) -> None:
        samples = make_samples("multi_direction", duration_s=24.0)
        stop_index = np.searchsorted(samples.timestamp_s, 12.0)
        original = analyzer.analyze_window(samples, 0, stop_index)
        modified_channels = {**samples.__dict__}
        for name in (
            "acceleration_body_m_s2", "angular_rate_body_rad_s",
            "estimated_quaternion_wxyz", "estimated_accel_bias_m_s2",
            "estimated_gyro_bias_rad_s",
        ):
            values = getattr(samples, name).copy()
            values[stop_index + 1:] = 1.0e6
            modified_channels[name] = values
        for name in (
            "position_attempted", "position_accepted",
            "velocity_attempted", "velocity_accepted",
        ):
            values = getattr(samples, name).copy()
            values[stop_index + 1:] = ~values[stop_index + 1:]
            modified_channels[name] = values
        for name in ("position_variance_m2", "velocity_variance_m2_s2"):
            values = getattr(samples, name).copy()
            values[stop_index + 1:] = 12345.0
            modified_channels[name] = values
        modified_timestamps = samples.timestamp_s.copy()
        modified_timestamps[stop_index + 1:] += 1000.0
        modified_channels["timestamp_s"] = modified_timestamps
        modified = analyzer.ExcitationSamples(**modified_channels)
        repeated = analyzer.analyze_window(modified, 0, stop_index)
        np.testing.assert_allclose(
            original["normalized_information_matrix"],
            repeated["normalized_information_matrix"],
            rtol=0.0,
            atol=0.0,
        )

    def test_transition_matches_first_order_static_couplings(self) -> None:
        transition = analyzer.transition_matrix(
            np.asarray([1.0, 0.0, 0.0, 0.0]),
            np.asarray([0.0, 0.0, -9.80665]),
            np.zeros(3),
            0.01,
        )
        self.assertAlmostEqual(transition[3, 1], -9.80665 * 0.01)
        self.assertAlmostEqual(transition[4, 0], 9.80665 * 0.01)
        np.testing.assert_allclose(transition[3:6, 9:12], -np.eye(3) * 0.01)

    def test_report_names_and_limitations_prevent_product_gate_interpretation(self) -> None:
        report = analyzer.analyze_window(make_samples("multi_direction"))
        self.assertEqual(report["schema_version"], 2)
        self.assertNotIn("information_ready", report)
        limitations = set(report["method"]["limitations"])
        self.assertIn(
            "imu_process_noise_and_bias_random_walk_are_not_in_the_information_matrix",
            limitations,
        )
        self.assertIn("preintegration_covariance_is_not_propagated", limitations)
        self.assertIn("error_reset_jacobians_are_not_reconstructed", limitations)

    def test_causal_provenance_manifest_validates_exact_files(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            directory = Path(temporary_directory)
            replay = directory / "replay.csv"
            results = directory / "results.csv"
            manifest = directory / "provenance.json"
            generator = directory / "generate_replay.py"
            runner = directory / "runner"
            replay.write_bytes(b"replay\n")
            results.write_bytes(b"results\n")
            generator.write_bytes(b"generator\n")
            runner.write_bytes(b"runner\n")

            def record(path: Path) -> dict[str, object]:
                content = path.read_bytes()
                return {"sha256": hashlib.sha256(content).hexdigest(), "bytes": len(content)}

            manifest.write_text(
                json.dumps(
                    {
                        "schema_version": 2,
                        "sources": {"replay": record(replay), "results": record(results)},
                        "execution": {
                            "artifacts": {
                                "replay_generator": {"path": str(generator), **record(generator)},
                                "estimator_runner": {"path": str(runner), **record(runner)},
                            },
                            "command": [str(runner), str(replay), str(results)],
                            "git_commit": "a" * 40,
                        },
                        "causal_estimator_output": {
                            "online_forward_filter": True,
                            "future_samples_used": False,
                            "initialization_source": "causal_sensor_alignment",
                            "truth_seeded_initialization": False,
                            "stationarity_source": "causal_detector",
                            "truth_derived_stationarity": False,
                        },
                    }
                ),
                encoding="utf-8",
            )
            validated = analyzer.validate_causal_provenance(manifest, replay, results)
            self.assertEqual(
                validated["validation"],
                "passed_artifact_binding_and_causal_assertion_checks_v2",
            )
            self.assertIn("runner_semantics_not_attested", validated["evidence_boundary"])
            results.write_bytes(b"changed\n")
            with self.assertRaisesRegex(ValueError, "results SHA-256"):
                analyzer.validate_causal_provenance(manifest, replay, results)

    def test_causal_provenance_rejects_truth_derived_stationarity(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            directory = Path(temporary_directory)
            replay = directory / "replay.csv"
            results = directory / "results.csv"
            manifest = directory / "provenance.json"
            generator = directory / "generate_replay.py"
            runner = directory / "runner"
            replay.write_bytes(b"replay\n")
            results.write_bytes(b"results\n")
            generator.write_bytes(b"generator\n")
            runner.write_bytes(b"runner\n")
            sources = {
                name: {
                    "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                    "bytes": len(path.read_bytes()),
                }
                for name, path in (("replay", replay), ("results", results))
            }
            manifest.write_text(
                json.dumps(
                    {
                        "schema_version": 2,
                        "sources": sources,
                        "execution": {
                            "artifacts": {
                                "replay_generator": {"path": str(generator), **{
                                    "sha256": hashlib.sha256(generator.read_bytes()).hexdigest(),
                                    "bytes": len(generator.read_bytes()),
                                }},
                                "estimator_runner": {"path": str(runner), **{
                                    "sha256": hashlib.sha256(runner.read_bytes()).hexdigest(),
                                    "bytes": len(runner.read_bytes()),
                                }},
                            },
                            "command": [str(runner), str(replay), str(results)],
                            "git_commit": "b" * 40,
                        },
                        "causal_estimator_output": {
                            "online_forward_filter": True,
                            "future_samples_used": False,
                            "initialization_source": "causal_sensor_alignment",
                            "truth_seeded_initialization": False,
                            "stationarity_source": "causal_detector",
                            "truth_derived_stationarity": True,
                        },
                    }
                ),
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "truth_derived_stationarity"):
                analyzer.validate_causal_provenance(manifest, replay, results)

    def test_causal_provenance_rejects_truth_seeded_initialization(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            directory = Path(temporary_directory)
            replay = directory / "replay.csv"
            results = directory / "results.csv"
            manifest = directory / "provenance.json"
            generator = directory / "generate_replay.py"
            runner = directory / "runner"
            replay.write_bytes(b"replay\n")
            results.write_bytes(b"results\n")
            generator.write_bytes(b"generator\n")
            runner.write_bytes(b"runner\n")
            sources = {
                name: {
                    "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                    "bytes": len(path.read_bytes()),
                }
                for name, path in (("replay", replay), ("results", results))
            }
            manifest.write_text(
                json.dumps(
                    {
                        "schema_version": 2,
                        "sources": sources,
                        "execution": {
                            "artifacts": {
                                "replay_generator": {"path": str(generator), **{
                                    "sha256": hashlib.sha256(generator.read_bytes()).hexdigest(),
                                    "bytes": len(generator.read_bytes()),
                                }},
                                "estimator_runner": {"path": str(runner), **{
                                    "sha256": hashlib.sha256(runner.read_bytes()).hexdigest(),
                                    "bytes": len(runner.read_bytes()),
                                }},
                            },
                            "command": [str(runner), str(replay), str(results)],
                            "git_commit": "c" * 40,
                        },
                        "causal_estimator_output": {
                            "online_forward_filter": True,
                            "future_samples_used": False,
                            "initialization_source": "causal_sensor_alignment",
                            "truth_seeded_initialization": True,
                            "stationarity_source": "causal_detector",
                            "truth_derived_stationarity": False,
                        },
                    }
                ),
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "truth_seeded_initialization"):
                analyzer.validate_causal_provenance(manifest, replay, results)


if __name__ == "__main__":
    unittest.main()
