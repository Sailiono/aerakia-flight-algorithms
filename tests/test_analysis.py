from __future__ import annotations

import csv
import os
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np


VALIDATION = Path(__file__).resolve().parents[1] / "validation"
sys.path.insert(0, str(VALIDATION))
os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "aerakia-matplotlib"))

import analyze_results as analysis  # noqa: E402
import run_ulog_suite as ulog_suite  # noqa: E402


class ContinuousYawReferenceTests(unittest.TestCase):
    @staticmethod
    def columns(with_reset: bool = True) -> dict[str, np.ndarray]:
        count = 6
        reset = np.array([0, 0, 0, 1, 0, 0], dtype=float) if with_reset else np.zeros(count)
        counter = np.array([0, 0, 0, 1, 1, 1], dtype=float) if with_reset else np.zeros(count)
        angle = np.radians(25.0)
        return {
            "ts_us": np.arange(count, dtype=float) * 1_000_000.0,
            "truth_yaw_deg": np.array([0, 0, 0, 50, 50, 50], dtype=float) if with_reset else np.zeros(count),
            "eskf_yaw_deg": np.zeros(count),
            "ref_attitude_reset_event": reset,
            "ref_attitude_reset_counter": counter,
            "ref_delta_q_reset_w": np.full(count, np.cos(angle)),
            "ref_delta_q_reset_x": np.zeros(count),
            "ref_delta_q_reset_y": np.zeros(count),
            "ref_delta_q_reset_z": np.full(count, np.sin(angle)),
        }

    def test_px4_reset_is_removed_without_realigning_eskf_segment(self) -> None:
        metrics = analysis.continuous_reference_yaw_metrics(self.columns())
        self.assertAlmostEqual(metrics["rmse_deg"], 0.0)
        self.assertEqual(len(metrics["events"]), 1)
        event = metrics["events"][0]
        self.assertAlmostEqual(event["observed_reference_jump_deg"], 50.0)
        self.assertAlmostEqual(event["quaternion_yaw_delta_deg"], 50.0)
        self.assertAlmostEqual(event["aerakia_sample_jump_deg"], 0.0)
        self.assertAlmostEqual(event["continuous_error_after_deg"], 0.0)

    def test_no_reset_matches_globally_aligned_raw_yaw(self) -> None:
        columns = self.columns(with_reset=False)
        continuous = analysis.continuous_reference_yaw_metrics(columns)
        time_s = (columns["ts_us"] - columns["ts_us"][0]) * 1.0e-6
        offset = analysis._initial_alignment_offset(
            time_s, columns["eskf_yaw_deg"], columns["truth_yaw_deg"]
        )
        raw_error = analysis.wrapped_error_deg(
            columns["eskf_yaw_deg"] + offset, columns["truth_yaw_deg"]
        )
        raw_rmse = float(np.sqrt(np.mean(raw_error * raw_error)))
        self.assertAlmostEqual(continuous["rmse_deg"], raw_rmse)
        self.assertEqual(continuous["events"], [])

    def test_gsf_comparison_keeps_dereset_metric_diagnostic(self) -> None:
        count = 12
        columns = {
            "ts_us": np.arange(count, dtype=float) * 1_000_000.0,
            "truth_yaw_deg": np.r_[np.zeros(6), np.full(6, 50.0)],
            "eskf_yaw_deg": np.zeros(count),
            "ref_attitude_reset_event": np.r_[np.zeros(6), 1.0, np.zeros(5)],
            "ref_attitude_reset_counter": np.r_[np.zeros(6), np.ones(6)],
            "ref_gsf_yaw_deg": np.zeros(count),
            "ref_gsf_yaw_variance_rad2": np.full(count, 0.01),
            "ref_gsf_yaw_valid": np.ones(count),
        }
        metrics = analysis.gsf_yaw_comparison_metrics(columns)
        assert metrics is not None
        self.assertAlmostEqual(metrics["eskf"]["rmse_deg"], 0.0)
        self.assertGreater(metrics["px4_raw"]["rmse_deg"], 30.0)
        self.assertAlmostEqual(metrics["px4_continuous"]["rmse_deg"], 0.0)


class CsvLoadingTests(unittest.TestCase):
    def test_reports_exact_missing_cell(self) -> None:
        with tempfile.TemporaryDirectory() as temp_directory:
            path = Path(temp_directory) / "truncated.csv"
            with path.open("w", encoding="utf-8", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow(["a", "b"])
                writer.writerow(["1", ""])
            with self.assertRaisesRegex(ValueError, "'b'.*row 2"):
                analysis.load_columns(path)

    def test_row_counter_rejects_mid_row_truncation(self) -> None:
        with tempfile.TemporaryDirectory() as temp_directory:
            path = Path(temp_directory) / "truncated.csv"
            path.write_bytes(b"a,b\n1,2\n3")
            with self.assertRaisesRegex(ValueError, "truncated mid-row"):
                ulog_suite.csv_data_rows(path)


if __name__ == "__main__":
    unittest.main()
