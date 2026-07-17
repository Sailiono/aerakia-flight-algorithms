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
import convert_euroc_to_replay as euroc_converter  # noqa: E402
import convert_ulog_to_replay as ulog_converter  # noqa: E402
import analyze_results as analyzer  # noqa: E402
import generate_synthetic_imu as synthetic_generator  # noqa: E402
import run_monte_carlo as monte_carlo  # noqa: E402


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


class MonteCarloRunnerTests(unittest.TestCase):
    def test_parse_seed_ranges_deduplicates_in_order(self) -> None:
        self.assertEqual(monte_carlo.parse_seeds("0:3,2,5"), [0, 1, 2, 5])

    def test_summary_reports_threshold_failure_seed(self) -> None:
        trial = {
            "seed": 9,
            "position_rmse_m": 0.80,
            "velocity_rmse_m_s": 0.10,
            "attitude_rmse_deg": 0.20,
            "position_nis_mean": 3.0,
            "velocity_nis_mean": 3.0,
            "navigation_nees_mean": 6.0,
            "healthy_ratio": 1.0,
            "navigation_recoveries": 0,
        }
        summary = monte_carlo.summarize_trials([trial])
        self.assertEqual(summary["failures"][0]["seed"], 9)
        self.assertIn("position_rmse_m", summary["failures"][0]["reasons"][0])

    def test_timestamp_jitter_is_deterministic_and_monotonic(self) -> None:
        import numpy as np

        first = synthetic_generator.jittered_timestamps_us(
            100, 100.0, 250.0, np.random.default_rng(42)
        )
        second = synthetic_generator.jittered_timestamps_us(
            100, 100.0, 250.0, np.random.default_rng(42)
        )
        np.testing.assert_array_equal(first, second)
        self.assertTrue(np.all(np.diff(first) > 0))
        self.assertNotEqual(len(set(np.diff(first))), 1)

    def test_trusted_heading_profile_has_fault_dropout_and_recovery(self) -> None:
        import numpy as np

        time_s = np.arange(2000, dtype=np.float64) / 100.0
        _, _, _, yaw_deg = synthetic_generator.generate_motion(
            20.0, 100.0, "heading_recovery"
        )
        valid, updates, heading, _, fault = synthetic_generator.trusted_heading_profile(
            time_s, yaw_deg, 100.0, np.random.default_rng(9)
        )
        self.assertEqual(np.count_nonzero(updates[(time_s >= 8.0) & (time_s < 12.0)]), 0)
        self.assertGreater(np.count_nonzero(fault), 0)
        self.assertGreater(np.count_nonzero(updates[time_s >= 12.0]), 0)
        self.assertTrue(np.all(np.isfinite(heading)))

    def test_bias_excitation_starts_static_then_excites_all_axes(self) -> None:
        import numpy as np

        time_s, roll, pitch, yaw = synthetic_generator.generate_motion(
            40.0, 100.0, "bias_excitation"
        )
        acceleration, velocity, position = synthetic_generator.bias_excitation_profile(
            time_s, 100.0
        )
        self.assertTrue(np.allclose(roll[time_s < 2.0], 0.0))
        self.assertTrue(np.allclose(acceleration[time_s < 2.0], 0.0))
        self.assertGreater(np.ptp(roll), 20.0)
        self.assertGreater(np.ptp(pitch), 15.0)
        self.assertGreater(np.ptp(yaw), 30.0)
        self.assertTrue(np.all(np.ptp(acceleration, axis=0) > 0.5))
        self.assertTrue(np.all(np.isfinite(velocity)))
        self.assertTrue(np.all(np.isfinite(position)))


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


class EurocConverterTests(unittest.TestCase):
    @staticmethod
    def _identity_sensor_yaml() -> str:
        return (
            "sensor_type: imu\n"
            "rate_hz: 200\n"
            "T_BS:\n"
            "  rows: 4\n"
            "  cols: 4\n"
            "  data: [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]\n"
        )

    def test_flu_z_up_is_converted_to_frd_ned(self) -> None:
        import numpy as np

        with tempfile.TemporaryDirectory() as temp_directory:
            sequence = Path(temp_directory) / "MH_test"
            imu_dir = sequence / "mav0" / "imu0"
            truth_dir = sequence / "mav0" / "state_groundtruth_estimate0"
            imu_dir.mkdir(parents=True)
            truth_dir.mkdir(parents=True)
            (imu_dir / "sensor.yaml").write_text(
                self._identity_sensor_yaml(), encoding="utf-8"
            )
            (truth_dir / "sensor.yaml").write_text(
                self._identity_sensor_yaml(), encoding="utf-8"
            )
            timestamp = 1_000_000_000
            imu = np.array(
                [
                    [timestamp, 1.0, 2.0, 3.0, 0.0, 0.0, 9.80665],
                    [timestamp + 5_000_000, 1.0, 2.0, 3.0, 0.0, 0.0, 9.80665],
                    [timestamp + 10_000_000, 1.0, 2.0, 3.0, 0.0, 0.0, 9.80665],
                ]
            )
            truth = np.array(
                [
                    [timestamp, 1.0, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0,
                     1.0, 2.0, 3.0, 0.1, 0.2, 0.3, 0.0, 0.0, 0.1],
                    [timestamp + 10_000_000, 2.0, 4.0, 6.0, 1.0, 0.0, 0.0, 0.0,
                     1.0, 2.0, 3.0, 0.1, 0.2, 0.3, 0.0, 0.0, 0.1],
                ]
            )
            np.savetxt(imu_dir / "data.csv", imu, delimiter=",")
            np.savetxt(truth_dir / "data.csv", truth, delimiter=",")
            output = Path(temp_directory) / "replay.csv"
            metadata = euroc_converter.convert_euroc(
                sequence, output, synthetic_gnss_rate_hz=100.0, seed=1
            )
            with output.open("r", encoding="utf-8", newline="") as stream:
                rows = list(csv.DictReader(stream))

            self.assertEqual(len(rows), 3)
            self.assertEqual(rows[0]["raw_acc_mg_z"], "-1000")
            self.assertEqual(rows[0]["raw_gyro_mdps_y"], str(round(math.degrees(-2.0) * 1000.0)))
            self.assertAlmostEqual(float(rows[-1]["ref_position_n_m"]), 1.0)
            self.assertAlmostEqual(float(rows[-1]["ref_position_e_m"]), -2.0)
            self.assertAlmostEqual(float(rows[-1]["ref_position_d_m"]), -3.0)
            self.assertAlmostEqual(float(rows[0]["ref_velocity_e_m_s"]), -2.0)
            self.assertEqual(sum(int(row["position_update"]) for row in rows), 2)
            self.assertEqual(metadata["reference_kind"], "independent_truth")
            self.assertAlmostEqual(
                metadata["frame_conversion"]["median_source_world_specific_force_m_s2"][2],
                9.80665,
            )

            corrected_output = Path(temp_directory) / "replay_bias_corrected.csv"
            corrected = euroc_converter.convert_euroc(
                sequence, corrected_output, apply_reference_bias=True
            )
            with corrected_output.open("r", encoding="utf-8", newline="") as stream:
                corrected_rows = list(csv.DictReader(stream))
            self.assertEqual(
                corrected_rows[0]["raw_gyro_mdps_y"],
                str(round(math.degrees(-(2.0 - 0.2)) * 1000.0)),
            )
            self.assertTrue(corrected["reference_bias_correction"]["applied"])

            vicon_dir = sequence / "mav0" / "vicon0"
            vicon_dir.mkdir()
            (vicon_dir / "sensor.yaml").write_text(
                "sensor_type: pose\n"
                "T_BS:\n"
                "  rows: 4\n"
                "  cols: 4\n"
                "  data: [0, -1, 0, 1, 1, 0, 0, 2, 0, 0, 1, 3, 0, 0, 0, 1]\n",
                encoding="utf-8",
            )
            root_half = 2.0 ** -0.5
            vicon = np.array(
                [
                    [timestamp + 5_000_000, 1.0, 2.0, 3.0,
                     root_half, 0.0, 0.0, root_half],
                    [timestamp + 10_000_000, 1.5, 2.0, 3.0,
                     root_half, 0.0, 0.0, root_half],
                    [timestamp + 15_000_000, 2.0, 2.0, 3.0,
                     root_half, 0.0, 0.0, root_half],
                ]
            )
            np.savetxt(vicon_dir / "data.csv", vicon, delimiter=",")
            vicon_output = Path(temp_directory) / "replay_vicon.csv"
            vicon_metadata = euroc_converter.convert_euroc(
                sequence, vicon_output, pose_source="vicon", pose_time_offset_us=5_000.0,
                static_hint_duration_s=0.005,
            )
            with vicon_output.open("r", encoding="utf-8", newline="") as stream:
                vicon_rows = list(csv.DictReader(stream))
            self.assertAlmostEqual(float(vicon_rows[-1]["ref_position_n_m"]), 1.0)
            self.assertAlmostEqual(float(vicon_rows[0]["ref_q_w"]), 1.0)
            self.assertAlmostEqual(float(vicon_rows[0]["ref_q_x"]), 0.0, places=7)
            self.assertAlmostEqual(float(vicon_rows[0]["ref_q_y"]), 0.0, places=7)
            self.assertAlmostEqual(float(vicon_rows[0]["ref_q_z"]), 0.0, places=7)
            self.assertEqual(vicon_metadata["pose_reference"]["source"], "vicon")
            self.assertEqual(sum(int(row["static_hint"]) for row in vicon_rows), 2)
            self.assertEqual(
                vicon_metadata["pose_reference"]["logged_timestamp_minus_physical_timestamp_us"],
                5_000.0,
            )


class ValidationAnalyzerTests(unittest.TestCase):
    def test_trusted_heading_metrics_separate_faults_and_dropout(self) -> None:
        import numpy as np

        columns = {
            "ts_us": np.arange(8, dtype=np.float64) * 100_000.0,
            "input_heading_update": np.array([1, 1, 1, 0, 0, 1, 1, 1], dtype=float),
            "eskf_heading_accepted": np.array([1, 1, 0, 0, 0, 1, 1, 1], dtype=float),
            "input_heading_fault": np.array([0, 0, 1, 0, 0, 0, 0, 0], dtype=float),
            "gnss_heading_valid": np.array([1, 1, 1, 0, 0, 1, 1, 1], dtype=float),
            "eskf_heading_innovation_rad": np.array([0, 0, 1.5, 0, 0, 0, 0, 0], dtype=float),
            "eskf_yaw_deg": np.array([0, 0.1, 0.1, 0.2, 0.3, 0.1, 0.0, 0.0]),
            "truth_yaw_deg": np.zeros(8),
        }
        metrics = analyzer.trusted_heading_metrics(columns)
        assert metrics is not None
        self.assertAlmostEqual(metrics["normal_acceptance_ratio"], 1.0)
        self.assertAlmostEqual(metrics["fault_rejection_ratio"], 1.0)
        self.assertAlmostEqual(metrics["dropout_max_abs_yaw_error_deg"], 0.3)
        self.assertAlmostEqual(metrics["recovery_time_s"], 0.0)

    def test_bias_metrics_report_reduction_and_settling(self) -> None:
        import numpy as np

        columns = {"ts_us": np.arange(5, dtype=float) * 1_000_000.0}
        columns["eskf_static_aligned"] = np.array([0, 1, 1, 1, 1], dtype=float)
        for axis in ("x", "y", "z"):
            columns[f"truth_accel_bias_{axis}_m_s2"] = np.zeros(5)
            columns[f"truth_gyro_bias_{axis}_rad_s"] = np.zeros(5)
            columns[f"eskf_accel_bias_{axis}_m_s2"] = np.zeros(5)
            columns[f"eskf_gyro_bias_{axis}_rad_s"] = np.zeros(5)
        columns["eskf_accel_bias_x_m_s2"] = np.array([0.2, 0.2, 0.08, 0.04, 0.03])
        columns["eskf_gyro_bias_x_rad_s"] = np.array([0.002, 0.002, 0.0008, 0.0006, 0.0005])
        metrics = analyzer.bias_metrics(columns)
        assert metrics is not None
        self.assertAlmostEqual(metrics["accel_error_reduction_ratio"], 0.85)
        self.assertAlmostEqual(metrics["accel_settling_time_below_0_05_m_s2_s"], 2.0)
        self.assertAlmostEqual(metrics["gyro_settling_time_below_0_001_rad_s_s"], 1.0)

    def test_cold_start_reports_tilt_without_heading(self) -> None:
        import numpy as np

        columns = {
            "ts_us": np.array([0.0, 1_000_000.0, 2_000_000.0]),
            "eskf_static_tilt_aligned": np.array([0.0, 1.0, 1.0]),
            "eskf_static_heading_aligned": np.zeros(3),
            "truth_roll_deg": np.zeros(3),
            "truth_pitch_deg": np.zeros(3),
            "truth_yaw_deg": np.zeros(3),
            "eskf_roll_deg": np.zeros(3),
            "eskf_pitch_deg": np.zeros(3),
            "eskf_yaw_deg": np.zeros(3),
        }
        for prefix in ("truth", "eskf"):
            columns[f"{prefix}_q_w"] = np.ones(3)
            columns[f"{prefix}_q_x"] = np.zeros(3)
            columns[f"{prefix}_q_y"] = np.zeros(3)
            columns[f"{prefix}_q_z"] = np.zeros(3)
        result = analyzer.cold_start_alignment_metrics(columns)
        self.assertIsNotNone(result)
        assert result is not None
        self.assertFalse(result["heading_alignment_completed"])
        self.assertAlmostEqual(result["post_tilt_alignment_tilt_rmse_deg"], 0.0)
        self.assertNotIn("post_alignment_attitude_rmse_deg", result)

    def test_quaternion_metric_ignores_euler_gimbal_lock_representation(self) -> None:
        import numpy as np

        half = 2.0 ** -0.5
        columns = {
            "ts_us": np.array([0.0, 10_000.0]),
            "truth_roll_deg": np.array([0.0, 0.0]),
            "truth_pitch_deg": np.array([90.0, 90.0]),
            "truth_yaw_deg": np.array([0.0, 0.0]),
            "eskf_roll_deg": np.array([180.0, -180.0]),
            "eskf_pitch_deg": np.array([90.0, 90.0]),
            "eskf_yaw_deg": np.array([180.0, -180.0]),
            "truth_q_w": np.array([half, half]),
            "truth_q_x": np.zeros(2),
            "truth_q_y": np.array([half, half]),
            "truth_q_z": np.zeros(2),
            "eskf_q_w": np.array([half, half]),
            "eskf_q_x": np.zeros(2),
            "eskf_q_y": np.array([half, half]),
            "eskf_q_z": np.zeros(2),
        }
        metrics = analyzer.metrics_for(columns, "eskf", "independent_truth")
        self.assertAlmostEqual(metrics["overall_attitude_rmse_deg"], 0.0)
        self.assertAlmostEqual(metrics["tilt_rmse_deg"], 0.0)
        self.assertGreater(metrics["euler_component_rmse_deg"], 100.0)

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
