"""Fail-closed tests for the same-input PX4/Aerakia M0 protocol skeleton."""

from __future__ import annotations

import csv
import importlib.util
import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "validation" / "run_px4_bias_ab_m0.py"
SPEC = importlib.util.spec_from_file_location("run_px4_bias_ab_m0", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
M0 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(M0)


class Px4BiasAbM0Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.manifest_path = ROOT / "validation" / "px4_bias_ab_m0_manifest.json"
        cls.manifest = M0.load_json(cls.manifest_path)
        cls.schema = M0.load_json(ROOT / cls.manifest["schema_path"])

    def test_frozen_protocol_is_self_consistent(self) -> None:
        M0.validate_manifest(self.manifest, self.schema)
        self.assertEqual(
            self.manifest["px4_commit"],
            "de8158101c96ad6b04170dc91f087148104c58eb",
        )
        self.assertEqual(self.manifest["profiles"]["stock"]["px4"]["EKF2_ACC_B_NOISE"], 0.003)
        self.assertEqual(self.manifest["comparison"]["time_alignment_tolerance_us"], 0)

    def test_missing_px4_source_is_blocked(self) -> None:
        with self.assertRaises(M0.BlockedError):
            M0.validate_px4_source(None, self.manifest)

    def test_wrong_px4_commit_is_rejected_before_file_checks(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            with mock.patch.object(M0, "run_git", return_value="0" * 40):
                with self.assertRaisesRegex(M0.ProtocolError, "does not match frozen SHA"):
                    M0.validate_px4_source(Path(temporary), self.manifest)

    def test_csv_contract_rejects_non_monotonic_time(self) -> None:
        section = {
            "required_columns": ["timestamp_us", "flag", "dt_s"],
            "time_column": "timestamp_us",
            "boolean_columns": ["flag"],
            "positive_columns": ["dt_s"],
        }
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "bad.csv"
            with path.open("w", encoding="utf-8", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=section["required_columns"])
                writer.writeheader()
                writer.writerow({"timestamp_us": 10, "flag": 1, "dt_s": 0.01})
                writer.writerow({"timestamp_us": 10, "flag": 0, "dt_s": 0.01})
            with self.assertRaisesRegex(M0.ProtocolError, "not strictly increasing"):
                M0.read_canonical_csv(path, section)

    @staticmethod
    def input_rows() -> list[dict[str, float]]:
        return [
            {
                "timestamp_us": float(second * 1_000_000),
                "truth_accel_bias_x_m_s2": 0.0,
                "truth_accel_bias_y_m_s2": 0.0,
                "truth_accel_bias_z_m_s2": 0.0,
            }
            for second in range(0, 61, 10)
        ]

    @staticmethod
    def output_rows(offset_us: int = 0) -> list[dict[str, float]]:
        rows: list[dict[str, float]] = []
        for second in range(0, 61, 10):
            bias = 0.1 if second < 20 else 0.02
            rows.append({
                "fusion_horizon_timestamp_us": float(second * 1_000_000 + offset_us),
                "accel_bias_x_m_s2": bias,
                "accel_bias_y_m_s2": 0.0,
                "accel_bias_z_m_s2": 0.0,
                "accel_bias_variance_x_m2_s4": 0.01,
                "accel_bias_variance_y_m2_s4": 0.01,
                "accel_bias_variance_z_m2_s4": 0.01,
                "healthy": 1.0,
                "accel_bias_valid": 1.0,
                "accel_bias_learning_inhibit_supported": 1.0,
                "accel_bias_learning_inhibited": 0.0,
            })
        return rows

    def test_same_horizon_comparison_reports_settling(self) -> None:
        result = M0.compare_exports(
            self.input_rows(), self.output_rows(), self.output_rows(), self.manifest,
        )
        self.assertEqual(result["duration_s"], 60.0)
        self.assertEqual(result["aerakia"]["settling_time_s"], 20.0)
        self.assertTrue(result["aerakia"]["absolute_convergence_budget_met"])
        self.assertEqual(result["aerakia"]["accel_bias_valid_ratio"], 1.0)
        self.assertEqual(result["aerakia"], result["px4"])

    def test_invalid_bias_output_cannot_satisfy_settling_gate(self) -> None:
        output = self.output_rows()
        output[-1]["accel_bias_valid"] = 0.0
        result = M0.compare_exports(self.input_rows(), output, output, self.manifest)
        self.assertIsNone(result["aerakia"]["settling_time_s"])
        self.assertFalse(result["aerakia"]["absolute_convergence_budget_met"])
        self.assertLess(result["aerakia"]["accel_bias_valid_ratio"], 1.0)

    def test_mismatched_fusion_horizon_fails_closed(self) -> None:
        with self.assertRaisesRegex(M0.ProtocolError, "not aligned"):
            M0.compare_exports(
                self.input_rows(), self.output_rows(), self.output_rows(offset_us=1), self.manifest,
            )

    def test_cli_without_px4_source_writes_blocked_manifest(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            out_dir = Path(temporary) / "run"
            status = M0.main([
                "--manifest", str(self.manifest_path),
                "--out-dir", str(out_dir),
            ])
            record = json.loads((out_dir / "run-manifest.json").read_text(encoding="utf-8"))
            self.assertEqual(status, 2)
            self.assertEqual(record["status"], "blocked")
            self.assertEqual(record["claim_status"], "no_px4_parity_claim")
            self.assertIn("--px4-source is required", record["reason"])


if __name__ == "__main__":
    unittest.main()
