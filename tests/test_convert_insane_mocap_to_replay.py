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

import convert_insane_mocap_to_replay as converter  # noqa: E402


class InsaneMocapConverterTests(unittest.TestCase):
    @staticmethod
    def _write_csv(path: Path, header: list[str], rows: list[list[float]]) -> None:
        with path.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(header)
            writer.writerows(rows)

    def _sequence(self, root: Path, mocap_gap: bool = False) -> Path:
        sequence = root / "indoor_1_sensors"
        sequence.mkdir(parents=True)
        time_s = np.arange(0.0, 4.01, 0.01)
        yaw = np.where(time_s <= 1.0, 0.0, 0.3 * (time_s - 1.0))
        yaw_rate = np.where(time_s <= 1.0, 0.0, 0.3)
        imu_rows = [
            [1000.0 + t, 0.0, 0.0, converter.GRAVITY_M_S2, 0.0, 0.0, rate]
            for t, rate in zip(time_s, yaw_rate)
        ]
        self._write_csv(
            sequence / "px4_imu.csv",
            ["t", "a_x", "a_y", "a_z", "w_x", "w_y", "w_z"],
            imu_rows,
        )
        mag_rows = [
            [1000.0 + t, 20.0e-6, 0.0, 45.0e-6, 0.0, 0.0, 0.0]
            for t in time_s[::2]
        ]
        self._write_csv(
            sequence / "px4_mag.csv",
            ["t", "cart_x", "cart_y", "cart_z", "spher_az", "spher_el", "spher_norm"],
            mag_rows,
        )
        mocap_rows = []
        for index, (t, angle) in enumerate(zip(time_s, yaw)):
            if mocap_gap and 120 <= index <= 150:
                continue
            mocap_rows.append(
                [
                    1000.0 + t, 0.1 * t, 0.0, 0.0,
                    math.cos(0.5 * angle), 0.0, 0.0, math.sin(0.5 * angle),
                ]
            )
        self._write_csv(
            sequence / "mocap_vehicle_data.csv",
            ["t", "p_x", "p_y", "p_z", "q_w", "q_x", "q_y", "q_z"],
            mocap_rows,
        )
        return sequence

    @staticmethod
    def _manifest(
        path: Path,
        *,
        split_name: str = "calibration",
        split_window: tuple[float, float] = (0.2, 3.8),
        calibration_source: str = "calibration",
        calibration_window: tuple[float, float] | None = (0.2, 3.8),
        calibration_sequence: str = "synthetic-indoor",
    ) -> Path:
        document = {
            "schema_version": 1,
            "sequence": "synthetic-indoor",
            "streams": {
                "imu": "px4_imu.csv",
                "magnetometer": "px4_mag.csv",
                "reference": "mocap_vehicle_data.csv",
            },
            "split": {"name": split_name, "window_s": list(split_window)},
            "calibration": {
                "source_sequence": calibration_sequence,
                "source_split": calibration_source,
                "source_window_s": (
                    list(calibration_window) if calibration_window is not None else None
                ),
                "provenance": "synthetic test calibration",
                "mocap_time_offset_s": 0.0,
                "mocap_body_rotation": np.eye(3).tolist(),
                "mocap_world_to_ned_rotation": [
                    [0.0, 1.0, 0.0],
                    [1.0, 0.0, 0.0],
                    [0.0, 0.0, -1.0],
                ],
                "mag_sensor_to_flu_rotation": np.eye(3).tolist(),
                "mag_intrinsic_transform": np.eye(3).tolist(),
                "mag_intrinsic_offset_t": [0.0, 0.0, 0.0],
                "magnetic_declination_rad": 0.1,
                "yaw_datum": "local_optitrack",
            },
            "audit": {
                "max_mocap_gap_s": 0.05,
                "max_mag_time_residual_s": 0.011,
                "angular_rate_window_s": 0.05,
                "max_angular_rate_rmse_rad_s": 0.02,
                "min_angular_rate_norm_correlation": 0.95,
                "max_gravity_direction_error_deg": 1.0,
                "min_audit_samples": 20,
                "min_static_samples": 20,
            },
            "static_hint_duration_s": 0.5,
        }
        path.write_text(json.dumps(document), encoding="utf-8")
        return path

    def test_raw_magnetometer_and_optitrack_remain_separate(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            sequence = self._sequence(root)
            manifest = self._manifest(root / "manifest.json")
            output = root / "replay.csv"
            metadata_path = root / "metadata.json"
            metadata = converter.convert_insane_mocap(
                sequence, manifest, output, metadata_path
            )
            with output.open("r", encoding="utf-8", newline="") as stream:
                rows = list(csv.DictReader(stream))

            self.assertGreater(len(rows), 300)
            magnetometer_rows = [row for row in rows if int(row["mag_update"]) != 0]
            self.assertGreater(len(magnetometer_rows), 100)
            self.assertEqual(magnetometer_rows[0]["raw_mag_cuT_x"], "2000")
            self.assertEqual(magnetometer_rows[0]["raw_mag_cuT_z"], "-4500")
            self.assertTrue(all(int(row["gnss_heading_update"]) == 0 for row in rows))
            self.assertTrue(all(int(row["position_ref_valid"]) == 0 for row in rows))
            self.assertAlmostEqual(float(rows[0]["magnetic_declination_rad"]), 0.1)
            self.assertEqual(metadata["reference_kind"], "independent_optitrack_attitude")
            self.assertEqual(metadata["evidence_class"], "A_candidate_local_yaw")
            self.assertFalse(
                metadata["source_independence"]["generated_ground_truth_used"]
            )
            self.assertEqual(
                set(metadata["input_sha256"]),
                {"px4_imu.csv", "px4_mag.csv", "mocap_vehicle_data.csv", "manifest"},
            )
            self.assertEqual(len(metadata["output_sha256"]), 64)
            self.assertEqual(
                json.loads(metadata_path.read_text(encoding="utf-8"))["output_sha256"],
                metadata["output_sha256"],
            )
            self.assertGreater(
                metadata["frame_time_audit"]["angular_rate"]["norm_correlation"],
                0.99,
            )

    def test_holdout_must_not_overlap_calibration_window(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            path = self._manifest(
                Path(temporary_directory) / "manifest.json",
                split_name="holdout",
                split_window=(2.0, 4.0),
                calibration_source="calibration",
                calibration_window=(1.0, 3.0),
            )
            with self.assertRaisesRegex(ValueError, "must be disjoint"):
                converter._load_manifest(path)

    def test_cross_sequence_holdout_does_not_compare_unrelated_windows(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            path = self._manifest(
                Path(temporary_directory) / "manifest.json",
                split_name="holdout",
                split_window=(2.0, 4.0),
                calibration_source="calibration",
                calibration_window=(1.0, 3.0),
                calibration_sequence="separate-calibration-sequence",
            )
            manifest = converter._load_manifest(path)
            self.assertEqual(
                manifest["calibration"]["source_sequence"],
                "separate-calibration-sequence",
            )

    def test_calibration_source_sequence_is_required(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            path = self._manifest(Path(temporary_directory) / "manifest.json")
            document = json.loads(path.read_text(encoding="utf-8"))
            del document["calibration"]["source_sequence"]
            path.write_text(json.dumps(document), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "source_sequence"):
                converter._load_manifest(path)

    def test_generated_ground_truth_reference_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            sequence = self._sequence(root)
            manifest_path = self._manifest(root / "manifest.json")
            document = json.loads(manifest_path.read_text(encoding="utf-8"))
            document["streams"]["reference"] = "ground_truth_80hz.csv"
            manifest_path.write_text(json.dumps(document), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "mocap_vehicle_data.csv"):
                converter.convert_insane_mocap(
                    sequence, manifest_path, root / "replay.csv"
                )

    def test_frame_and_time_audits_fail_closed(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            sequence = self._sequence(root, mocap_gap=True)
            manifest = self._manifest(root / "manifest.json")
            with self.assertRaisesRegex(ValueError, "OptiTrack bracket gap"):
                converter.convert_insane_mocap(
                    sequence, manifest, root / "replay.csv"
                )

            sequence = self._sequence(root / "second")
            manifest = self._manifest(root / "bad_rotation.json")
            document = json.loads(manifest.read_text(encoding="utf-8"))
            document["calibration"]["mocap_body_rotation"] = [
                [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]
            ]
            manifest.write_text(json.dumps(document), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "proper rotation"):
                converter.convert_insane_mocap(
                    sequence, manifest, root / "bad.csv"
                )


if __name__ == "__main__":
    unittest.main()
