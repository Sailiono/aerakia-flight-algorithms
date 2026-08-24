"""Tests for the validation-only TAS/horizontal-wind observability oracle."""

from __future__ import annotations

import copy
import dataclasses
import json
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "validation"))
import run_airspeed_wind_observability as study  # noqa: E402


class AirspeedWindObservabilityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.protocol = study.load_protocol()

    def test_analytic_tas_jacobian_matches_finite_difference(self) -> None:
        velocity = np.array((25.0, -6.0, 1.5), dtype=np.float64)
        wind = np.array((4.5, -3.0), dtype=np.float64)
        analytic = study.tas_jacobian_wind_ne(velocity, wind)
        finite_difference = study.finite_difference_tas_jacobian(velocity, wind)
        np.testing.assert_allclose(analytic, finite_difference, rtol=0.0, atol=2.0e-6)

    def test_positive_multihdg_case_qualifies_without_truth_in_oracle_input(self) -> None:
        observations, truths, now_us = study.generate_scenario("multi_heading_nominal", self.protocol)
        self.assertFalse(hasattr(observations[0], "wind_ne_m_s"))
        oracle = study.CausalWindOracle(self.protocol)
        for observation in observations:
            oracle.step(observation)
        summary = oracle.summary(now_us)
        self.assertEqual(summary["terminal_status"], "qualified")
        self.assertGreaterEqual(summary["measurement_information_rank"], 2)
        self.assertTrue(summary["range_difference_initializer_used"])
        score = study.score_causal_oracle(summary, truths[-1])
        self.assertLess(score["terminal_wind_error_norm_m_s"], 0.9)

    def test_straight_line_never_claims_two_dimensional_observability(self) -> None:
        observations, _truths, now_us = study.generate_scenario("straight_line", self.protocol)
        oracle = study.CausalWindOracle(self.protocol)
        for observation in observations:
            oracle.step(observation)
        summary = oracle.summary(now_us)
        self.assertEqual(summary["terminal_status"], "not_observable")
        self.assertLess(
            summary["measurement_information_minimum_eigenvalue"],
            self.protocol["causal_oracle"]["minimum_measurement_information_eigenvalue"],
        )
        self.assertFalse(summary["range_difference_initializer_used"])

    def test_invalid_delayed_samples_do_not_change_information_or_estimate(self) -> None:
        observations, _truths, _now_us = study.generate_scenario("multi_heading_nominal", self.protocol)
        oracle = study.CausalWindOracle(self.protocol)
        for observation in observations[:32]:
            oracle.step(observation)
        baseline = oracle.summary(observations[31].arrival_timestamp_us)
        delayed = dataclasses.replace(observations[32], arrival_timestamp_us=observations[32].tas_timestamp_us + 600_000)
        event = oracle.step(delayed)
        after = oracle.summary(delayed.arrival_timestamp_us)
        self.assertFalse(event.accepted)
        self.assertEqual(event.reason, "tas_delivery_delay")
        np.testing.assert_allclose(after["estimate_ne_m_s"], baseline["estimate_ne_m_s"], atol=0.0)
        np.testing.assert_allclose(after["measurement_information"], baseline["measurement_information"], atol=0.0)

    def test_gnss_outage_ages_qualified_wind_instead_of_extending_information(self) -> None:
        observations, _truths, now_us = study.generate_scenario("gnss_outage", self.protocol)
        oracle = study.CausalWindOracle(self.protocol)
        for observation in observations:
            oracle.step(observation)
        summary = oracle.summary(now_us)
        self.assertEqual(summary["terminal_status"], "stale_no_fresh_gnss_tas")
        self.assertGreater(summary["rejection_counts"]["missing_gnss_velocity"], 0)

    def test_persistent_model_mismatch_latches_and_never_auto_recovers(self) -> None:
        observations, _truths, now_us = study.generate_scenario("wind_shear", self.protocol)
        oracle = study.CausalWindOracle(self.protocol)
        for observation in observations:
            oracle.step(observation)
        summary = oracle.summary(now_us)
        self.assertEqual(summary["terminal_status"], "source_latched")
        self.assertTrue(summary["source_latched"])
        self.assertGreaterEqual(summary["rejection_counts"]["innovation_nis"], 3)
        self.assertGreater(summary["rejection_counts"]["source_latched"], 0)

    def test_reordered_transport_control_rejects_without_mutating_prior_information(self) -> None:
        observations, _truths, _now_us = study.generate_scenario("reordered_tas", self.protocol)
        oracle = study.CausalWindOracle(self.protocol)
        for observation in observations[:-1]:
            oracle.step(observation)
        before = oracle.summary(observations[-2].arrival_timestamp_us)
        event = oracle.step(observations[-1])
        after = oracle.summary(observations[-1].arrival_timestamp_us)
        self.assertFalse(event.accepted)
        self.assertEqual(event.reason, "tas_timestamp_not_monotonic")
        np.testing.assert_allclose(after["estimate_ne_m_s"], before["estimate_ne_m_s"], atol=0.0)
        np.testing.assert_allclose(after["measurement_information"], before["measurement_information"], atol=0.0)

    def test_protocol_matrix_passes_and_all_negative_controls_fail_closed(self) -> None:
        result = study.run_protocol(self.protocol)
        self.assertEqual(result["status"], "passed")
        for case in result["cases"]:
            self.assertTrue(case["passed"], msg=f"{case['name']}: {case['errors']}")
            if case["role"] == "negative":
                self.assertNotEqual(case["causal_two_state_wind_oracle"]["terminal_status"], "qualified")

    def test_protocol_rejects_any_attempt_to_turn_the_oracle_into_production_eskf(self) -> None:
        bad = copy.deepcopy(self.protocol)
        bad["scope"]["production_eskf"] = "17_state_wind_enabled"
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "bad-airspeed-protocol.json"
            path.write_text(json.dumps(bad), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "production ESKF unchanged"):
                study.load_protocol(path)


if __name__ == "__main__":
    unittest.main()
