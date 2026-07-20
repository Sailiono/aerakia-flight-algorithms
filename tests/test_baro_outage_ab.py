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
sys.path.insert(0, str(ROOT / "validation"))
import run_baro_outage_ab as baro_ab  # noqa: E402


class BarometerOutageAbTests(unittest.TestCase):
    def test_shadow_input_audit_derives_allowed_difference(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            active = root / "active.csv"
            shadow = root / "shadow.csv"
            active.write_text("ts_us,imu,baro_update\n1,7,1\n2,8,1\n", encoding="utf-8")
            shadow.write_text("ts_us,imu,baro_update\n1,7,0\n2,8,0\n", encoding="utf-8")
            audit = baro_ab.audit_shadow_inputs(active, shadow)
            self.assertTrue(audit["verified"])
            self.assertEqual(audit["observed_differing_columns"], ["baro_update"])

            shadow.write_text("ts_us,imu,baro_update\n1,9,0\n2,8,0\n", encoding="utf-8")
            with self.assertRaisesRegex(RuntimeError, "column imu"):
                baro_ab.audit_shadow_inputs(active, shadow)

    def test_shadow_command_audit_derives_common_runner_configuration(self) -> None:
        audit = baro_ab.audit_shadow_commands(
            ["runner", "--cold-start", "--supervise-barometer", "active.csv", "active.out"],
            ["runner", "--cold-start", "shadow.csv", "shadow.out"],
        )
        self.assertTrue(audit["verified"])
        self.assertEqual(
            audit["normalized_common_command"],
            ["runner", "--cold-start", "<input>", "<results>"],
        )

    def test_vertical_truth_starts_static_and_is_bounded(self) -> None:
        time_s = np.arange(0.0, 80.0, 0.01)
        position, velocity, acceleration = baro_ab.vertical_truth(time_s)
        self.assertTrue(np.allclose(position[time_s < 2.0], 0.0))
        self.assertTrue(np.allclose(velocity[time_s < 2.0], 0.0))
        self.assertTrue(np.allclose(acceleration[time_s < 2.0], 0.0))
        self.assertLessEqual(float(np.max(np.abs(position[:, 2]))), 4.0 + 1.0e-12)
        self.assertLessEqual(float(np.max(np.abs(velocity[:, 2]))), 0.42)

    def test_fault_profiles_are_deterministic_and_distinct(self) -> None:
        time_s = np.arange(0.0, 30.0, 0.01)
        truth = np.sin(time_s * 0.2)
        streams: dict[str, np.ndarray] = {}
        for fault in baro_ab.FAULTS:
            update, timestamp_us, measured, variance, _ = baro_ab.barometer_stream(
                time_s, truth, fault=fault, outage_start_s=8.0,
                outage_duration_s=10.0, rate_hz=100.0,
                rng=np.random.default_rng(7),
            )
            streams[fault] = measured
            expected_updates = 588 if fault == "delay" else 600
            self.assertEqual(int(np.count_nonzero(update)), expected_updates)
            if fault == "delay":
                self.assertTrue(np.all(
                    timestamp_us[update > 0]
                    <= np.rint(time_s[update > 0] * 1.0e6).astype(np.int64) - 600000
                ))
            self.assertTrue(np.allclose(variance, 0.16))
        self.assertGreater(float(np.mean(streams["constant_bias"] - streams["nominal"])), 0.99)
        self.assertGreater(float(np.max(streams["weather_step"] - streams["nominal"])), 1.99)
        self.assertFalse(np.allclose(streams["random_walk"], streams["nominal"]))
        self.assertFalse(np.allclose(streams["freeze"], streams["nominal"]))
        self.assertFalse(np.allclose(streams["delay"], streams["nominal"]))

    def test_invalid_fault_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            baro_ab.barometer_stream(
                np.arange(10.0), np.zeros(10), fault="unknown",
                outage_start_s=1.0, outage_duration_s=2.0, rate_hz=1.0,
                rng=np.random.default_rng(1),
            )

    def test_score_vertical_reports_baro_acceptance_and_nis(self) -> None:
        rows = [
            {
                "ts_us": str(index * 100000),
                "ref_position_d_m": "0", "ref_velocity_d_m_s": "0",
                "eskf_position_d_m": "0.1", "eskf_velocity_d_m_s": "0.2",
                "eskf_healthy": "1", "input_baro_update": "1",
                "eskf_baro_accepted": "1" if index % 2 == 0 else "0",
                "eskf_baro_nis": str(float(index)),
                "eskf_baro_test_ratio": str(float(index) / 9.0),
                "input_baro_status": "0", "input_baro_age_s": "0",
                "baro_supervisor_enabled": "1", "baro_supervisor_accepted": "1",
                "baro_supervisor_fault_flags": "0", "baro_supervisor_latched": "0",
            }
            for index in range(20)
        ]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "results.csv"
            with path.open("w", encoding="utf-8", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
            metrics = baro_ab.score_vertical(
                path, outage_start_s=0.5, outage_duration_s=1.0
            )
        self.assertEqual(metrics["outage_baro_update_count"], 10)
        self.assertEqual(metrics["outage_baro_accepted_count"], 5)
        self.assertAlmostEqual(metrics["outage_baro_acceptance_ratio"], 0.5)
        self.assertGreater(metrics["outage_baro_nis_p95"], 13.0)

    def test_shadow_failover_switches_on_first_latch(self) -> None:
        fieldnames = [
            "ts_us", "ref_position_d_m", "ref_velocity_d_m_s",
            "eskf_position_d_m", "eskf_velocity_d_m_s", "eskf_healthy",
            "eskf_static_aligned", "baro_supervisor_latched",
        ]
        with tempfile.TemporaryDirectory() as directory:
            supervised_path = Path(directory) / "supervised.csv"
            shadow_path = Path(directory) / "shadow.csv"
            for path, is_shadow in ((supervised_path, False), (shadow_path, True)):
                with path.open("w", encoding="utf-8", newline="") as stream:
                    writer = csv.DictWriter(stream, fieldnames=fieldnames)
                    writer.writeheader()
                    for index in range(30):
                        writer.writerow({
                            "ts_us": index * 100000,
                            "ref_position_d_m": 0.0, "ref_velocity_d_m_s": 0.0,
                            "eskf_position_d_m": 0.2 if is_shadow else (10.0 if index >= 10 else 0.1),
                            "eskf_velocity_d_m_s": 0.1 if is_shadow else (2.0 if index >= 10 else 0.05),
                            "eskf_healthy": 1,
                            "eskf_static_aligned": 1,
                            "baro_supervisor_latched": 0 if is_shadow else int(index >= 10),
                        })
            metrics = baro_ab.score_shadow_failover(
                supervised_path, shadow_path, outage_start_s=0.0, outage_duration_s=2.0,
                input_provenance_equivalent=True, configuration_equivalent=True,
            )
        self.assertEqual(metrics["shadow_failover_count"], 1)
        self.assertAlmostEqual(metrics["shadow_failover_time_s"], 1.0)
        self.assertAlmostEqual(metrics["shadow_failover_position_reset_m"], -9.8)
        self.assertLess(metrics["outage_vertical_position_rmse_m"], 0.2)
        self.assertTrue(metrics["shadow_failover_upper_bound"])
        self.assertFalse(metrics["shadow_full_state_verified"])
        self.assertFalse(metrics["shadow_reset_semantics_verified"])

    def test_shadow_failover_fails_closed_when_shadow_is_unhealthy(self) -> None:
        fieldnames = [
            "ts_us", "ref_position_d_m", "ref_velocity_d_m_s",
            "eskf_position_d_m", "eskf_velocity_d_m_s", "eskf_healthy",
            "eskf_static_aligned", "baro_supervisor_latched",
        ]
        with tempfile.TemporaryDirectory() as directory:
            supervised_path = Path(directory) / "supervised.csv"
            shadow_path = Path(directory) / "shadow.csv"
            for path, is_shadow in ((supervised_path, False), (shadow_path, True)):
                with path.open("w", encoding="utf-8", newline="") as stream:
                    writer = csv.DictWriter(stream, fieldnames=fieldnames)
                    writer.writeheader()
                    for index in range(20):
                        writer.writerow({
                            "ts_us": index * 100000,
                            "ref_position_d_m": 0.0, "ref_velocity_d_m_s": 0.0,
                            "eskf_position_d_m": 0.2, "eskf_velocity_d_m_s": 0.1,
                            "eskf_healthy": 0 if is_shadow and index >= 10 else 1,
                            "eskf_static_aligned": 1,
                            "baro_supervisor_latched": 0 if is_shadow else int(index >= 10),
                        })
            metrics = baro_ab.score_shadow_failover(
                supervised_path, shadow_path, outage_start_s=0.0, outage_duration_s=1.5,
                input_provenance_equivalent=True, configuration_equivalent=True,
            )
        self.assertEqual(metrics["shadow_failover_candidate_count"], 1)
        self.assertEqual(metrics["shadow_failover_count"], 0)
        self.assertIn("shadow_unhealthy", metrics["shadow_switch_blocked_reasons"])

    def test_shadow_failover_requires_provenance_and_configuration_evidence(self) -> None:
        fieldnames = [
            "ts_us", "ref_position_d_m", "ref_velocity_d_m_s",
            "eskf_position_d_m", "eskf_velocity_d_m_s", "eskf_healthy",
            "eskf_static_aligned", "baro_supervisor_latched",
        ]
        with tempfile.TemporaryDirectory() as directory:
            supervised_path = Path(directory) / "supervised.csv"
            shadow_path = Path(directory) / "shadow.csv"
            for path, is_shadow in ((supervised_path, False), (shadow_path, True)):
                with path.open("w", encoding="utf-8", newline="") as stream:
                    writer = csv.DictWriter(stream, fieldnames=fieldnames)
                    writer.writeheader()
                    for index in range(10):
                        writer.writerow({
                            "ts_us": index * 100000,
                            "ref_position_d_m": 0.0, "ref_velocity_d_m_s": 0.0,
                            "eskf_position_d_m": 0.0, "eskf_velocity_d_m_s": 0.0,
                            "eskf_healthy": 1, "eskf_static_aligned": 1,
                            "baro_supervisor_latched": 0 if is_shadow else int(index >= 5),
                        })
            metrics = baro_ab.score_shadow_failover(
                supervised_path, shadow_path, outage_start_s=0.0, outage_duration_s=0.8,
            )
        self.assertEqual(metrics["shadow_failover_count"], 0)
        self.assertIn("input_provenance_unverified", metrics["shadow_switch_blocked_reasons"])
        self.assertIn("configuration_equivalence_unverified", metrics["shadow_switch_blocked_reasons"])

    def test_strict_json_value_replaces_nonfinite_numbers_with_null(self) -> None:
        sanitized = baro_ab.strict_json_value({
            "nan": math.nan, "positive_inf": math.inf,
            "nested": [1.0, -math.inf],
        })
        encoded = json.dumps(sanitized, allow_nan=False)
        decoded = json.loads(encoded)
        self.assertIsNone(decoded["nan"])
        self.assertIsNone(decoded["positive_inf"])
        self.assertIsNone(decoded["nested"][1])


if __name__ == "__main__":
    unittest.main()
