from __future__ import annotations

import csv
import math
import sys
import tempfile
import unittest
from pathlib import Path


TOOLS = Path(__file__).resolve().parents[1] / "simulation" / "tools"
sys.path.insert(0, str(TOOLS))
VALIDATION = Path(__file__).resolve().parents[1] / "validation"
sys.path.insert(0, str(VALIDATION))

import convert_capture_to_golden as converter  # noqa: E402
import convert_ulog_to_replay as ulog_converter  # noqa: E402
import analyze_results as analyzer  # noqa: E402


def record(sequence: int, timestamp_us: int) -> str:
    return (
        f"[IMUCSV] seq={sequence},ts_us={timestamp_us},dt_us=2500,"
        "raw_acc_mg=[0,0,-1000],raw_gyro_mdps=[0,0,0],"
        "raw_mag_cuT=[2200,100,4400],g_est_mg=[0,0,-1000],"
        "rpy_mdeg=[0,0,0]"
    )


class CaptureConverterTests(unittest.TestCase):
    def test_parse_record(self) -> None:
        parsed = converter.parse_record(record(7, 10_000))
        self.assertIsNotNone(parsed)
        assert parsed is not None
        self.assertEqual(parsed["seq"], 7)
        self.assertEqual(parsed["raw_acc_mg_z"], -1000)

    def test_rejects_truncated_record(self) -> None:
        self.assertIsNone(converter.parse_record("[IMUCSV] ts_us=1,dt_us=2500"))

    def test_conversion_deduplicates_and_marks_gap(self) -> None:
        with tempfile.TemporaryDirectory() as temp_directory:
            input_path = Path(temp_directory) / "capture.csv"
            output_path = Path(temp_directory) / "golden.csv"
            with input_path.open("w", encoding="utf-8", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=["ts_sec", "line"])
                writer.writeheader()
                writer.writerow({"ts_sec": "0.01", "line": record(1, 2500)})
                writer.writerow({"ts_sec": "0.02", "line": record(1, 2500)})
                writer.writerow({"ts_sec": "0.03", "line": record(3, 7500)})

            self.assertEqual(converter.convert_capture(input_path, output_path), 2)
            with output_path.open("r", encoding="utf-8", newline="") as stream:
                rows = list(csv.DictReader(stream))

            self.assertEqual(rows[0]["host_ts_us"], "10000")
            self.assertEqual(rows[1]["seq_delta"], "2")
            self.assertEqual(rows[1]["gap_flag"], "1")


class FakeDataset:
    def __init__(self, name: str, data: dict[str, object]) -> None:
        self.name = name
        self.multi_id = 0
        self.data = data


class FakeULog:
    def __init__(
        self, datasets: list[FakeDataset], initial_parameters: dict[str, object] | None = None
    ) -> None:
        self.data_list = datasets
        self.initial_parameters = initial_parameters or {}

    def get_dataset(self, name: str) -> FakeDataset:
        return next(dataset for dataset in self.data_list if dataset.name == name)


class ULogConverterTests(unittest.TestCase):
    def test_quaternion_interpolation_handles_equivalent_signs(self) -> None:
        import numpy as np

        result = ulog_converter.interpolate_quaternions(
            np.array([0, 10]),
            np.array([[1.0, 0.0, 0.0, 0.0], [-1.0, 0.0, 0.0, 0.0]]),
            np.array([5]),
        )
        np.testing.assert_allclose(result[0], [1.0, 0.0, 0.0, 0.0])

    def test_quaternion_interpolation_does_not_cross_reset_group(self) -> None:
        import numpy as np

        half = 2.0 ** -0.5
        result = ulog_converter.interpolate_quaternions(
            np.array([0, 10]),
            np.array([[1.0, 0.0, 0.0, 0.0], [half, 0.0, 0.0, half]]),
            np.array([5]),
            np.array([0, 1]),
        )
        np.testing.assert_allclose(result[0], [1.0, 0.0, 0.0, 0.0])

    def test_conversion_preserves_reset_event_without_absolute_gps(self) -> None:
        import numpy as np

        imu_timestamp = np.array([0, 10_000, 20_000], dtype=np.int64)
        sensor = {
            "timestamp": imu_timestamp,
            "accelerometer_m_s2[0]": np.zeros(3),
            "accelerometer_m_s2[1]": np.zeros(3),
            "accelerometer_m_s2[2]": np.full(3, -9.80665),
            "gyro_rad[0]": np.zeros(3),
            "gyro_rad[1]": np.zeros(3),
            "gyro_rad[2]": np.zeros(3),
        }
        attitude = {
            "timestamp": imu_timestamp,
            "q[0]": np.ones(3),
            "q[1]": np.zeros(3),
            "q[2]": np.zeros(3),
            "q[3]": np.zeros(3),
            "quat_reset_counter": np.array([0, 1, 1]),
            "delta_q_reset[0]": np.ones(3),
            "delta_q_reset[1]": np.zeros(3),
            "delta_q_reset[2]": np.zeros(3),
            "delta_q_reset[3]": np.zeros(3),
        }
        gps = {
            "timestamp": np.array([0, 20_000]),
            "lat": np.array([31_0000000, 31_0000010]),
            "lon": np.array([121_0000000, 121_0000010]),
            "alt": np.array([10_000, 10_100]),
            "fix_type": np.array([3, 3]),
            "vel_n_m_s": np.zeros(2),
            "vel_e_m_s": np.zeros(2),
            "vel_d_m_s": np.zeros(2),
            "eph": np.ones(2),
            "epv": np.ones(2),
            "s_variance_m_s": np.ones(2),
            "vel_m_s": np.array([2.0, 3.0]),
            "vel_ned_valid": np.ones(2),
            "cog_rad": np.array([0.1, 0.2]),
            "c_variance_rad": np.full(2, 0.01),
            "heading": np.array([0.15, 0.25]),
            "heading_offset": np.zeros(2),
            "heading_accuracy": np.full(2, 0.05),
        }
        yaw_estimator = {
            "timestamp": np.array([0, 20_000]),
            "yaw_composite": np.array([0.12, 0.22]),
            "yaw_variance": np.full(2, 0.02),
        }
        fake = FakeULog(
            [FakeDataset("sensor_combined", sensor), FakeDataset("vehicle_attitude", attitude),
             FakeDataset("vehicle_gps_position", gps),
             FakeDataset("yaw_estimator_status", yaw_estimator)],
            {"EKF2_MAG_DECL": -5.5},
        )
        with tempfile.TemporaryDirectory() as temp_directory:
            output = Path(temp_directory) / "replay.csv"
            metadata = ulog_converter.convert_ulog(
                Path(temp_directory) / "fake.ulg", output, ulog_factory=lambda _: fake
            )
            with output.open("r", encoding="utf-8", newline="") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(sum(int(row["ref_attitude_reset_event"]) for row in rows), 1)
            self.assertNotIn("lat", rows[0])
            self.assertNotIn("lon", rows[0])
            self.assertEqual(metadata["gps_updates"], 2)
            self.assertEqual(metadata["gnss_heading_updates"], 2)
            self.assertEqual(metadata["gnss_course_diagnostic_updates"], 2)
            self.assertEqual(metadata["px4_gsf_yaw_updates"], 2)
            self.assertAlmostEqual(float(rows[0]["gnss_heading_rad"]), 0.15)
            self.assertAlmostEqual(metadata["magnetic_declination_deg"], -5.5)
            self.assertAlmostEqual(
                float(rows[0]["magnetic_declination_rad"]), math.radians(-5.5)
            )


class ValidationAnalyzerTests(unittest.TestCase):
    def test_reset_compensation_removes_logged_px4_yaw_step(self) -> None:
        import numpy as np

        half = 2.0 ** -0.5
        columns = {
            "ts_us": np.array([0.0, 1_000_000.0, 2_000_000.0, 3_000_000.0]),
            "truth_yaw_deg": np.array([10.0, 10.0, 100.0, 100.0]),
            "eskf_yaw_deg": np.full(4, 10.0),
            "ref_attitude_reset_event": np.array([0.0, 0.0, 1.0, 0.0]),
            "ref_delta_q_reset_w": np.array([1.0, 1.0, half, half]),
            "ref_delta_q_reset_x": np.zeros(4),
            "ref_delta_q_reset_y": np.zeros(4),
            "ref_delta_q_reset_z": np.array([0.0, 0.0, half, half]),
        }
        compensated = analyzer.reset_compensated_reference_yaw(columns)
        np.testing.assert_allclose(compensated, np.full(4, 10.0), atol=1.0e-9)
        metrics = analyzer.reset_compensated_yaw_metrics(columns)
        self.assertAlmostEqual(metrics["rmse_deg"], 0.0)
        summary = analyzer.reset_summary(columns)
        self.assertEqual(summary["events"], 1.0)
        self.assertAlmostEqual(summary["event_details"][0]["observed_px4_yaw_jump_deg"], 90.0)

    def test_consistency_metrics_use_only_gnss_update_rows(self) -> None:
        import numpy as np

        columns = {
            "ts_us": np.arange(5, dtype=np.float64),
            "input_position_update": np.array([1.0, 0.0, 1.0, 0.0, 1.0]),
            "eskf_position_nis": np.array([3.0, np.nan, 6.0, np.nan, 9.0]),
            "eskf_velocity_nis": np.array([1.0, np.nan, 2.0, np.nan, 3.0]),
            "eskf_navigation_nees": np.array([4.0, np.nan, 6.0, np.nan, 8.0]),
        }
        metrics = analyzer.consistency_metrics(columns, "synthetic")
        assert metrics is not None
        self.assertEqual(metrics["position_nis"]["samples"], 3)
        self.assertAlmostEqual(metrics["position_nis"]["mean"], 6.0)
        self.assertAlmostEqual(metrics["velocity_nis"]["mean"], 2.0)
        self.assertAlmostEqual(metrics["navigation_nees"]["mean"], 6.0)

        real_metrics = analyzer.consistency_metrics(columns, "px4_estimate")
        assert real_metrics is not None
        self.assertNotIn("navigation_nees", real_metrics)


if __name__ == "__main__":
    unittest.main()
