from __future__ import annotations

import json
import hashlib
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation" / "tools"))
sys.path.insert(0, str(ROOT / "validation"))

import generate_synthetic_imu as generator  # noqa: E402
import run_residual_bias_boundary as boundary  # noqa: E402
import run_bias_observability_cross_validation as cross_validation  # noqa: E402


class ResidualBiasBoundaryTests(unittest.TestCase):
    def test_case_set_exhausts_axis_pairs_and_full_corners(self) -> None:
        cases = boundary.build_boundary_cases()
        coverage = boundary.coverage_summary(cases)
        self.assertTrue(coverage["complete"])
        self.assertEqual(len(cases), 137)
        self.assertEqual(
            coverage["family_counts"],
            {"nominal": 1, "axis": 12, "pairwise": 60, "full_corner": 64},
        )
        self.assertEqual(coverage["axis_sign_boundaries_covered"], 12)
        self.assertEqual(coverage["isolated_pair_sign_combinations_covered"], 60)
        self.assertEqual(coverage["full_corners_covered"], 64)

    def test_physical_vectors_apply_sensor_specific_three_sigma_boundaries(self) -> None:
        case = {
            "sigma_vector": [-1, 0, 1, 1, -1, 0],
        }
        accel, gyro = boundary.physical_bias_vectors(case, 0.05, 0.20, 3.0)
        np.testing.assert_allclose(accel, [-0.15, 0.0, 0.15], rtol=0.0, atol=1e-15)
        np.testing.assert_allclose(gyro, [0.60, -0.60, 0.0], rtol=0.0, atol=1e-15)

    def test_explicit_bias_vector_overrides_random_prior(self) -> None:
        rng = np.random.default_rng(17)
        requested = [-0.15, 0.0, 0.15]
        actual, source = generator.resolve_bias_vector(rng, 9.0, 0.1, requested)
        np.testing.assert_array_equal(actual, requested)
        self.assertEqual(source, "explicit_vector")

    def test_generator_cli_records_exact_override_and_source(self) -> None:
        with tempfile.TemporaryDirectory() as temp_directory:
            output = Path(temp_directory) / "input.csv"
            metadata = Path(temp_directory) / "metadata.json"
            subprocess.run(
                [
                    sys.executable,
                    str(ROOT / "simulation" / "tools" / "generate_synthetic_imu.py"),
                    "--out", str(output),
                    "--metadata", str(metadata),
                    "--duration", "0.02",
                    "--rate", "100",
                    "--accel-bias-std-m-s2", "7",
                    "--gyro-bias-std-deg-s", "8",
                    "--accel-bias-vector-m-s2", "-0.15", "0", "0.15",
                    "--gyro-bias-vector-deg-s", "0.6", "-0.6", "0",
                ],
                check=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
            )
            manifest = json.loads(metadata.read_text(encoding="utf-8"))
            self.assertEqual(manifest["acceleration_bias_m_s2"], [-0.15, 0.0, 0.15])
            self.assertEqual(manifest["gyroscope_bias_deg_s"], [0.6, -0.6, 0.0])
            self.assertEqual(manifest["acceleration_bias_source"], "explicit_vector")
            self.assertEqual(manifest["gyroscope_bias_source"], "explicit_vector")

    def test_suite_command_forwards_both_vectors(self) -> None:
        command = boundary.suite_command(
            root=ROOT,
            runner=Path("/tmp/runner"),
            case_dir=Path("/tmp/case"),
            accel_bias=[-0.15, 0.0, 0.15],
            gyro_bias=[0.6, -0.6, 0.0],
            rate_hz=100.0,
            seed=7,
        )
        accel_index = command.index("--accel-bias-vector-m-s2")
        gyro_index = command.index("--gyro-bias-vector-deg-s")
        self.assertEqual(command[accel_index + 1:accel_index + 4], ["-0.15", "0.0", "0.15"])
        self.assertEqual(command[gyro_index + 1:gyro_index + 4], ["0.6", "-0.6", "0.0"])
        self.assertIn("bias_convergence", command)

    def test_zero_sensor_bias_skips_only_irrelevant_convergence_gate(self) -> None:
        metrics: dict[str, float | int | None] = {
            "accel_error_at_alignment_m_s2": 9.0,
            "accel_final_error_m_s2": 9.0,
            "gyro_final_error_rad_s": 8.0,
            "healthy_ratio": 1.0,
        }
        limits = {
            "accel_final_error_m_s2": {"maximum": 0.05},
            "gyro_final_error_rad_s": {"maximum": 0.001},
            "healthy_ratio": {"minimum": 1.0},
        }
        failures, skipped = boundary.metric_failures(
            metrics, limits, accel_active=False, gyro_active=False
        )
        self.assertEqual(failures, [])
        self.assertEqual(len(skipped), 2)
        failures, _ = boundary.metric_failures(
            metrics, limits, accel_active=True, gyro_active=False
        )
        self.assertEqual(failures, ["accel_final_error_m_s2=9>0.05"])

    def test_reduction_ratio_is_not_gated_after_alignment_already_meets_absolute_limit(self) -> None:
        metrics: dict[str, float | int | None] = {
            "accel_error_at_alignment_m_s2": 0.01,
            "accel_final_error_m_s2": 0.02,
            "accel_error_reduction_ratio": -1.0,
        }
        limits = {
            "accel_final_error_m_s2": {"maximum": 0.05},
            "accel_error_reduction_ratio": {"minimum": 0.60},
        }
        failures, skipped = boundary.metric_failures(
            metrics, limits, accel_active=True, gyro_active=False
        )
        self.assertEqual(failures, [])
        self.assertIn("alignment error already met", skipped[0])


class BiasObservabilityCrossValidationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.protocol_path = ROOT / "validation" / "bias_observability_protocol_v1.json"
        cls.protocol = json.loads(cls.protocol_path.read_text(encoding="utf-8"))

    def test_protocol_is_frozen_disjoint_and_has_exact_release_volume(self) -> None:
        validation = cross_validation.validate_protocol(self.protocol)
        self.assertTrue(validation["passed"], validation["failures"])
        self.assertEqual(validation["trajectory_count"], 5)
        self.assertEqual(validation["bias_vector_count"], 9)
        self.assertEqual(validation["release_trial_count"], 1728)
        self.assertEqual(validation["split_seed_counts"], {
            "train": 16, "tune": 32, "holdout": 64,
        })
        self.assertEqual(self.protocol["splits"]["holdout"], {
            "seed_start": 30000, "seed_stop": 30064,
        })

    def test_smoke_train_tune_and_release_selection_are_deterministic(self) -> None:
        smoke = cross_validation.build_trials(self.protocol, "smoke")
        train_tune = cross_validation.build_trials(self.protocol, "train-tune")
        release = cross_validation.build_trials(self.protocol, "release")
        self.assertEqual(len(smoke), 9)
        self.assertEqual(len(train_tune), 576)
        self.assertEqual(len(release), 1728)
        self.assertTrue(all(trial["trajectory"]["split"] != "holdout" for trial in smoke))
        self.assertTrue(all(trial["trajectory"]["split"] != "holdout" for trial in train_tune))
        self.assertEqual(
            cross_validation.canonical_sha256(self.protocol),
            cross_validation.canonical_sha256(
                json.loads(self.protocol_path.read_text(encoding="utf-8"))
            ),
        )

    def test_smoke_honors_seed_offset_without_opening_holdout(self) -> None:
        protocol = json.loads(json.dumps(self.protocol))
        protocol["execution"]["smoke_seed_offset"] = 2
        validation = cross_validation.validate_protocol(protocol)
        self.assertTrue(validation["passed"], validation["failures"])
        smoke = cross_validation.build_trials(protocol, "smoke")
        self.assertEqual(
            {(trial["trajectory"]["split"], trial["seed"]) for trial in smoke},
            {("train", 2), ("tune", 1002)},
        )

    def test_report_only_cannot_mask_train_tune_or_release_failure(self) -> None:
        self.assertFalse(cross_validation.metric_failures_are_fatal("smoke"))
        self.assertTrue(cross_validation.metric_failures_are_fatal("train-tune"))
        self.assertTrue(cross_validation.metric_failures_are_fatal("release"))

    def test_v2_plan_keeps_holdout_manifest_sealed(self) -> None:
        plan = json.loads(
            (ROOT / "validation" / "bias_observability_protocol_v2_plan.json").read_text(
                encoding="utf-8"
            )
        )
        self.assertEqual(plan["status"], "planned")
        self.assertEqual(plan["sealed_holdout"]["access"], "external_manifest_in_protected_ci")
        self.assertIsNone(plan["sealed_holdout"]["public_seed_range"])
        self.assertIsNone(plan["sealed_holdout"]["public_trajectory_parameters"])

    def test_summary_never_hides_capability_failures(self) -> None:
        results = [
            {
                "split": "train",
                "trajectory_id": "example",
                "bias_vector_id": "zero",
                "seed": 0,
                "gate_failures": [],
                "horizontal_bias": {"right_censored": False},
            },
            {
                "split": "train",
                "trajectory_id": "example",
                "bias_vector_id": "x_pos",
                "seed": 0,
                "gate_failures": ["horizontal_settling_time_s=null"],
                "horizontal_bias": {"right_censored": True},
            },
        ]
        summary = cross_validation.summarize(results)
        self.assertEqual(summary["passed_count"], 1)
        self.assertEqual(summary["failed_count"], 1)
        self.assertEqual(summary["right_censored_count"], 1)

    def test_five_profiles_are_bounded_distinct_and_have_static_boundaries(self) -> None:
        fingerprints: set[bytes] = set()
        for trajectory in self.protocol["trajectories"]:
            generated = generator.generate_bias_observability_trajectory(
                trajectory["motion"], trajectory["duration_s"], 100.0
            )
            time_s, roll, pitch, yaw, acceleration, velocity, position, static = generated
            self.assertEqual(len(time_s), int(trajectory["duration_s"] * 100.0))
            self.assertTrue(np.all(np.isfinite(np.column_stack((
                roll, pitch, yaw, acceleration, velocity, position,
            )))))
            self.assertTrue(np.all(static[time_s < 3.0] == 1))
            self.assertGreater(np.count_nonzero(static[time_s >= trajectory["duration_s"] - 5.0]), 0)
            self.assertLessEqual(float(np.max(np.abs(roll))), 15.0)
            self.assertLessEqual(float(np.max(np.abs(pitch))), 15.0)
            self.assertLessEqual(float(np.max(np.linalg.norm(acceleration, axis=1))), 2.25)
            self.assertLessEqual(float(np.max(np.linalg.norm(velocity, axis=1))), 9.0)
            self.assertLess(float(np.linalg.norm(velocity[-1])), 1.0e-9)
            fingerprints.add(hashlib.sha256(
                np.column_stack((roll, pitch, yaw, acceleration, position)).tobytes()
            ).digest())
        self.assertEqual(len(fingerprints), 5)

    def test_interval_start_zoh_truth_uses_the_same_held_acceleration(self) -> None:
        time_s = np.array([0.0, 0.1, 0.2, 0.3])
        endpoint_acceleration = np.array([
            [1.0, 0.0, 0.0], [2.0, 0.0, 0.0],
            [3.0, 0.0, 0.0], [4.0, 0.0, 0.0],
        ])
        acceleration, velocity, position = generator.interval_start_zoh_kinematics(
            time_s, endpoint_acceleration, np.zeros(3), np.zeros(3)
        )
        np.testing.assert_array_equal(acceleration[:, 0], [1.0, 1.0, 2.0, 3.0])
        for index in range(1, len(time_s)):
            dt = time_s[index] - time_s[index - 1]
            np.testing.assert_allclose(
                velocity[index], velocity[index - 1] + acceleration[index] * dt,
                rtol=0.0, atol=1.0e-15,
            )
            np.testing.assert_allclose(
                position[index],
                position[index - 1] + velocity[index - 1] * dt
                + 0.5 * acceleration[index] * dt * dt,
                rtol=0.0, atol=1.0e-15,
            )


if __name__ == "__main__":
    unittest.main()
