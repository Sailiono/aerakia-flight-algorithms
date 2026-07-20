from __future__ import annotations

import csv
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np


TOOLS = Path(__file__).resolve().parents[1] / "simulation" / "tools"
sys.path.insert(0, str(TOOLS))

import calibrate_insane_mocap as calibrator  # noqa: E402
import convert_insane_mocap_to_replay as replay  # noqa: E402


class InsaneMocapCalibrationTests(unittest.TestCase):
    @staticmethod
    def _write_csv(path: Path, header: list[str], rows: list[list[float]]) -> None:
        with path.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(header)
            writer.writerows(rows)

    @staticmethod
    def _quat_delta(rate: np.ndarray, dt: float) -> np.ndarray:
        angle = float(np.linalg.norm(rate) * dt)
        if angle < 1.0e-12:
            return np.asarray([1.0, 0.0, 0.0, 0.0])
        return np.concatenate(([math.cos(0.5 * angle)],
                               math.sin(0.5 * angle) * rate / np.linalg.norm(rate)))

    def _sequence(self, root: Path, *, static_only: bool = False) -> tuple[Path, np.ndarray, np.ndarray]:
        sequence = root / "indoor_1_sensors"
        sequence.mkdir(parents=True)
        dt = 0.01
        actual_time = np.arange(0.0, 18.0 + 0.5 * dt, dt)
        body_rotation = np.asarray(
            [[0.997551, -0.059902, -0.037399],
             [0.059902, 0.998203, -0.003872],
             [0.037399, 0.003872, 0.999294]]
        )
        yaw_rotation = np.asarray(
            [[math.cos(0.4), -math.sin(0.4), 0.0],
             [math.sin(0.4), math.cos(0.4), 0.0],
             [0.0, 0.0, 1.0]]
        )
        world_to_ned = yaw_rotation @ np.asarray(
            [[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]]
        )
        inclination = -1.089745187701715
        declination = 0.059505255517495
        magnetic_ned = np.asarray([
            math.cos(inclination) * math.cos(declination),
            math.cos(inclination) * math.sin(declination),
            -math.sin(inclination),
        ])
        magnetic_world = world_to_ned.T @ magnetic_ned
        quaternion = np.empty((len(actual_time), 4), dtype=np.float64)
        quaternion[0] = [1.0, 0.0, 0.0, 0.0]
        rates = np.column_stack((
            np.where(actual_time < 1.0, 0.0, 0.5 * np.sin(0.5 * actual_time) + 0.2 * np.sin(1.1 * actual_time)),
            np.where(actual_time < 1.0, 0.0, 0.4 * np.sin(0.8 * actual_time + 0.7)),
            np.where(actual_time < 1.0, 0.0, 0.3 * np.cos(1.3 * actual_time + 0.2)),
        ))
        if static_only:
            rates[:] = 0.0
        mocap_rates = rates @ body_rotation.T
        for index in range(len(actual_time) - 1):
            quaternion[index + 1] = replay._normalize_quaternions(
                replay._quaternion_multiply(
                    quaternion[index:index + 1],
                    self._quat_delta(
                        0.5 * (mocap_rates[index] + mocap_rates[index + 1]), dt
                    )[None, :],
                )
            )[0]
        rotation_world_mocap = replay._rotation_from_quaternion(quaternion)
        rotation_world_flu = rotation_world_mocap @ body_rotation
        acceleration = np.einsum(
            "nji,j->ni", rotation_world_flu, [0.0, 0.0, replay.GRAVITY_M_S2]
        )
        mag_flu = np.einsum(
            "nji,j->ni", rotation_world_flu, magnetic_world
        )
        imu_rows = [
            [1000.0 + t, *acceleration[index], *rates[index]]
            for index, t in enumerate(actual_time)
        ]
        self._write_csv(
            sequence / "px4_imu.csv",
            ["t", "a_x", "a_y", "a_z", "w_x", "w_y", "w_z"], imu_rows
        )
        mag_rows = [
            [1000.0 + actual_time[index], *mag_flu[index], 0.0, 0.0, 0.0]
            for index in range(0, len(actual_time), 2)
        ]
        self._write_csv(
            sequence / "px4_mag.csv",
            ["t", "cart_x", "cart_y", "cart_z", "spher_az", "spher_el", "spher_norm"],
            mag_rows,
        )
        mocap_rows = [
            [999.988 + t, 0.0, 0.0, 0.0, *quaternion[index]]
            for index, t in enumerate(actual_time)
        ]
        self._write_csv(
            sequence / "mocap_vehicle_data.csv",
            ["t", "p_x", "p_y", "p_z", "q_w", "q_x", "q_y", "q_z"],
            mocap_rows,
        )
        return sequence, body_rotation, world_to_ned

    @staticmethod
    def _base_manifest(path: Path) -> Path:
        identity = np.eye(3).tolist()
        document = {
            "schema_version": 1,
            "sequence": "synthetic-calibration",
            "streams": {
                "imu": "px4_imu.csv",
                "magnetometer": "px4_mag.csv",
                "reference": "mocap_vehicle_data.csv",
            },
            "split": {"name": "calibration", "window_s": [0.0, 8.0]},
            "calibration": {
                "source_sequence": "synthetic-official-calibration",
                "source_split": "external",
                "source_window_s": None,
                "provenance": "synthetic calibration fixture",
                "mocap_time_offset_s": 0.0,
                "mocap_body_rotation": identity,
                "mocap_world_to_ned_rotation": [
                    [0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]
                ],
                "mag_sensor_to_flu_rotation": identity,
                "mag_intrinsic_transform": identity,
                "mag_intrinsic_offset_t": [0.0, 0.0, 0.0],
                "magnetic_declination_rad": 0.059505255517495,
                "magnetic_inclination_rad": -1.089745187701715,
                "yaw_datum": "local_optitrack",
            },
            "audit": {
                "max_mocap_gap_s": 0.05,
                "max_mag_time_residual_s": 0.02,
                "angular_rate_window_s": 0.05,
                "max_angular_rate_rmse_rad_s": 0.35,
                "min_angular_rate_norm_correlation": 0.7,
                "max_gravity_direction_error_deg": 20.0,
                "min_audit_samples": 20,
                "min_static_samples": 20,
            },
        }
        path.write_text(json.dumps(document), encoding="utf-8")
        return path

    def test_calibration_freezes_rotation_time_and_world_datum(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            sequence, expected_body, expected_world = self._sequence(root)
            base = self._base_manifest(root / "base.json")
            out_cal = root / "calibration.json"
            out_dev = root / "development.json"
            out_holdout = root / "holdout.json"
            report = calibrator.calibrate_insane_mocap(
                sequence, base, (0.0, 8.0), out_cal,
                (8.0, 12.0), out_dev, (12.0, 16.0), out_holdout,
                search_min_s=-0.05, search_max_s=0.05, search_step_s=0.002,
            )
            self.assertAlmostEqual(report["time_offset"]["estimate_s"], 0.012, delta=0.003)
            self.assertEqual(report["holdout_samples_read_for_estimation"], 0)
            self.assertLess(report["angular_rate"]["weighted_rmse_rad_s"], 0.02)
            self.assertGreater(report["angular_rate"]["excitation_minimum_to_maximum_ratio"], 0.05)
            self.assertLess(report["world_datum"]["horizontal_yaw_residual_deg"], 0.1)
            calibration = json.loads(out_cal.read_text(encoding="utf-8"))
            development = json.loads(out_dev.read_text(encoding="utf-8"))
            holdout = json.loads(out_holdout.read_text(encoding="utf-8"))
            self.assertEqual(calibration["split"]["name"], "calibration")
            self.assertEqual(development["split"]["name"], "development")
            self.assertEqual(holdout["split"]["name"], "holdout")
            self.assertEqual(
                calibration["calibration"]["source_sequence"],
                "synthetic-calibration",
            )
            self.assertEqual(
                development["calibration"]["source_sequence"],
                holdout["calibration"]["source_sequence"],
            )
            self.assertEqual(
                development["calibration"]["mocap_body_rotation"],
                holdout["calibration"]["mocap_body_rotation"],
            )
            self.assertEqual(
                development["calibration"]["mocap_time_offset_s"],
                holdout["calibration"]["mocap_time_offset_s"],
            )

    def test_rank_deficient_static_window_fails_closed(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            sequence, _, _ = self._sequence(root, static_only=True)
            base = self._base_manifest(root / "base.json")
            with self.assertRaisesRegex(ValueError, "insufficient angular excitation"):
                calibrator.calibrate_insane_mocap(
                    sequence, base, (0.0, 8.0), root / "calibration.json",
                    search_min_s=-0.05, search_max_s=0.05, search_step_s=0.002,
                )


if __name__ == "__main__":
    unittest.main()
