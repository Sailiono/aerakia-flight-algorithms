from __future__ import annotations

import csv
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation" / "tools"))

import generate_synthetic_imu as generator  # noqa: E402


class SyntheticImuInputContractTests(unittest.TestCase):
    def test_quaternion_rotation_is_orthonormal_and_identity_is_exact(self) -> None:
        identity = generator.quaternion_to_body_to_ned_matrix(np.array([1.0, 0.0, 0.0, 0.0]))
        np.testing.assert_allclose(identity, np.eye(3), atol=0.0, rtol=0.0)
        rotation = generator.quaternion_to_body_to_ned_matrix(
            generator.euler_to_quaternion(
                np.asarray([math.radians(20.0)]),
                np.asarray([math.radians(-15.0)]),
                np.asarray([math.radians(35.0)]),
            )[0]
        )
        np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=1.0e-14)
        self.assertAlmostEqual(float(np.linalg.det(rotation)), 1.0, places=14)

    def test_interval_specific_force_is_gravity_for_static_attitude(self) -> None:
        timestamp = np.arange(0.0, 2.0, 0.01)
        acceleration = np.zeros((len(timestamp), 3), dtype=np.float64)
        result = generator.interval_average_body_specific_force(
            timestamp,
            np.zeros(len(timestamp)),
            np.zeros(len(timestamp)),
            np.zeros(len(timestamp)),
            acceleration,
        )
        np.testing.assert_allclose(
            result,
            np.tile([0.0, 0.0, -generator.GRAVITY_M_S2], (len(timestamp), 1)),
            atol=1.0e-12,
            rtol=0.0,
        )

    def test_causal_stationarity_has_no_future_dependency(self) -> None:
        timestamp = np.arange(0.0, 5.0, 0.01) * 1.0e6
        acceleration = np.zeros((len(timestamp), 3), dtype=np.float64)
        acceleration[:, 2] = -generator.GRAVITY_M_S2
        angular_rate = np.zeros_like(acceleration)
        baseline = generator.causal_imu_stationarity_flags(
            timestamp,
            acceleration,
            angular_rate,
            window_s=1.0,
            gyro_threshold_rad_s=0.05,
            acceleration_tolerance_m_s2=0.2 * generator.GRAVITY_M_S2,
        )
        changed_acceleration = acceleration.copy()
        changed_acceleration[250:, 0] = 5.0
        changed_rate = angular_rate.copy()
        changed_rate[250:, 2] = 90.0
        changed = generator.causal_imu_stationarity_flags(
            timestamp,
            changed_acceleration,
            changed_rate,
            window_s=1.0,
            gyro_threshold_rad_s=0.05,
            acceleration_tolerance_m_s2=0.2 * generator.GRAVITY_M_S2,
        )
        np.testing.assert_array_equal(baseline[:250], changed[:250])
        self.assertEqual(int(np.count_nonzero(baseline[:100])), 0)
        self.assertGreater(int(np.count_nonzero(baseline[100:250])), 0)

    def test_causal_stationarity_uses_quantized_input_scale(self) -> None:
        timestamp = np.arange(0.0, 2.0, 0.01) * 1.0e6
        acceleration = np.tile([0.0, 0.0, -generator.GRAVITY_M_S2], (len(timestamp), 1))
        angular_rate = np.zeros_like(acceleration)
        quantized = generator.quantize_replay_measurements(
            acceleration, angular_rate, np.zeros_like(acceleration)
        )
        expected = generator.causal_imu_stationarity_flags(
            timestamp,
            quantized[0],
            quantized[1],
            window_s=1.0,
            gyro_threshold_rad_s=0.05,
            acceleration_tolerance_m_s2=0.2 * generator.GRAVITY_M_S2,
        )
        self.assertEqual(int(np.count_nonzero(expected)), 100)

    def test_interval_deltas_match_published_rates(self) -> None:
        timestamp = np.asarray([0, 10_000, 30_000], dtype=np.int64)
        acceleration = np.asarray([[1.0, -2.0, 3.0]] * 3)
        angular_rate = np.asarray([[10.0, -20.0, 30.0]] * 3)
        interval_us, delta_angle, delta_velocity = generator.interval_deltas_from_published_rates(
            timestamp, acceleration, angular_rate
        )
        np.testing.assert_array_equal(interval_us, [0, 10_000, 20_000])
        np.testing.assert_allclose(
            delta_angle[1], np.radians(angular_rate[1]) * 0.01, atol=1.0e-15
        )
        np.testing.assert_allclose(delta_velocity[2], acceleration[2] * 0.02, atol=1.0e-15)

    def test_csv_v2_contract_contains_auditable_delta_fields_and_causal_metadata(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            csv_path = root / "input.csv"
            metadata_path = root / "metadata.json"
            import subprocess

            subprocess.run(
                [
                    sys.executable,
                    str(ROOT / "simulation/tools/generate_synthetic_imu.py"),
                    "--out", str(csv_path),
                    "--metadata", str(metadata_path),
                    "--duration", "32",
                    "--rate", "50",
                    "--motion", "bias_cv_hover_axis_pulses",
                    "--static-hint",
                    "--stationarity-source", "causal_imu_window",
                    "--measurement-contract", "delta_interval_v2",
                    "--accel-time-semantics", "interval_start_zoh",
                    "--accel-noise-density-m-s2-sqrt-hz", "0.002",
                    "--gyro-noise-density-rad-s-sqrt-hz", "0.0000872664626",
                ],
                cwd=ROOT,
                check=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
            self.assertEqual(metadata["measurement_contract"], "delta_interval_v2")
            self.assertEqual(metadata["stationarity_source"], "causal_imu_window")
            self.assertFalse(metadata["truth_derived_stationarity"])
            with csv_path.open(newline="", encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(int(rows[0]["delta_interval_us"]), 0)
            for row in rows[1:]:
                dt = int(row["delta_interval_us"]) * 1.0e-6
                acceleration = np.asarray(
                    [float(row[f"raw_acc_mg_{axis}"]) for axis in "xyz"]
                ) * generator.GRAVITY_M_S2 / 1000.0
                angular_rate = np.asarray(
                    [float(row[f"raw_gyro_mdps_{axis}"]) for axis in "xyz"]
                ) / 1000.0
                np.testing.assert_allclose(
                    [float(row[f"delta_velocity_m_s_{axis}"]) for axis in "xyz"],
                    acceleration * dt,
                    atol=1.0e-12,
                    rtol=0.0,
                )
                np.testing.assert_allclose(
                    [float(row[f"delta_angle_rad_{axis}"]) for axis in "xyz"],
                    np.radians(angular_rate) * dt,
                    atol=1.0e-12,
                    rtol=0.0,
                )


if __name__ == "__main__":
    unittest.main()
