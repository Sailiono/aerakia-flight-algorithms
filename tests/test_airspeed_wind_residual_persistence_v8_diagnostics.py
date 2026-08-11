"""Focused semantic checks for the independent v8 policy-shape probe.

These tests intentionally do not invoke the v7 campaign runner, touch its
protocol, or create a public validation artifact.  They only prove that the
new diagnostic keeps continuity fail-closed and that the recent-boundary age
comparison is observable before a v8 train is frozen.
"""

from __future__ import annotations

import sys
import unittest
from dataclasses import fields
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "validation"))

import diagnose_airspeed_wind_residual_persistence_v8 as v8  # noqa: E402


class ResidualPersistenceV8DiagnosticTests(unittest.TestCase):
    def test_recent_boundary_age_sweep_is_explicit(self) -> None:
        report = v8.run_handcrafted_diagnostics(boundary_ages_s=(1.0, 2.0, 3.0))
        self.assertEqual(report["status"], "completed_handcrafted_development_diagnostic_only")
        self.assertEqual(report["boundary_age_sweep"], [1.0, 2.0, 3.0])
        self.assertEqual(report["policy_comparison_count"], 45)

        policies = report["policies"]
        for age in ("1s", "2s", "3s"):
            root_cause = policies[age]["midband_then_persistent_high"]
            expired = policies[age]["expired_boundary"]
            gap = policies[age]["gap_then_high"]
            if age == "1s":
                self.assertTrue(all(not result["latched"] for result in root_cause.values()))
            else:
                self.assertTrue(all(result["latched"] for result in root_cause.values()))
            self.assertTrue(all(not result["latched"] for result in expired.values()))
            self.assertTrue(all(not result["latched"] for result in gap.values()))

    def test_graded_evidence_can_bridge_one_midband_without_unbounded_age(self) -> None:
        report = v8.run_handcrafted_diagnostics(boundary_ages_s=(1.0, 2.0))
        age_one = report["policies"]["1s"]["intermittent_high"]
        self.assertTrue(age_one["graded_evidence"]["latched"])
        self.assertFalse(age_one["recent_boundary"]["latched"])
        self.assertFalse(age_one["bounded_retry"]["latched"])

        age_two = report["policies"]["2s"]["intermittent_high"]
        self.assertTrue(age_two["recent_boundary"]["latched"])
        self.assertTrue(age_two["bounded_retry"]["latched"])

    def test_graded_evidence_starts_at_zero_on_episode_onset(self) -> None:
        result = v8.run_probe(
            v8.PolicyKind.GRADED_EVIDENCE,
            v8.diagnostic_scenarios()["intermittent_high"],
            v8.ProbeConfig(recent_quiet_boundary_max_age_s=2.0),
        )
        onset = next(
            event for event in result.trace if event.source_timestamp_us == 4_500_000
        )
        self.assertEqual(onset.nis, 5.0)
        self.assertEqual(onset.graded_evidence_s, 0.0)
        self.assertEqual(onset.high_observations, 1)

    def test_continuity_reset_is_fail_closed_for_every_policy(self) -> None:
        scenario = v8.diagnostic_scenarios()["gap_then_high"]
        for policy in v8.PolicyKind:
            with self.subTest(policy=policy.value):
                result = v8.run_probe(policy, scenario, v8.ProbeConfig(recent_quiet_boundary_max_age_s=3.0))
                self.assertFalse(result.latched)
                self.assertGreaterEqual(result.reset_count, 1)

    def test_same_epoch_gap_requires_fresh_qualification_before_recovery(self) -> None:
        config = v8.ProbeConfig(recent_quiet_boundary_max_age_s=2.0)
        probe = v8.CausalPolicyProbe(v8.PolicyKind.RECENT_BOUNDARY, config)
        probe.reauthorize(0)
        for timestamp in (0.0, 0.5, 1.0, 1.5, 2.0):
            probe.observe(v8.ProbeInput(int(timestamp * 1e6), int((timestamp + 0.05) * 1e6), 1.0))
        probe.observe(v8.ProbeInput(4_000_000, 4_050_000, 5.0))
        self.assertFalse(probe.result().latched)
        for timestamp in (4.5, 5.0, 5.5, 6.0, 6.5, 7.0):
            probe.observe(v8.ProbeInput(int(timestamp * 1e6), int((timestamp + 0.05) * 1e6), 1.0))
        for timestamp in (7.5, 8.0, 8.5, 9.0):
            probe.observe(v8.ProbeInput(int(timestamp * 1e6), int((timestamp + 0.05) * 1e6), 5.0))
        self.assertTrue(probe.result().latched)

    def test_probe_input_has_no_truth_or_injection_label_fields(self) -> None:
        names = {field.name for field in fields(v8.ProbeInput)}
        self.assertEqual(
            names,
            {"source_timestamp_us", "arrival_timestamp_us", "nis", "source_valid", "source_epoch"},
        )

    def test_retry_budget_exhaustion_is_visible(self) -> None:
        scenario = v8.diagnostic_scenarios()["retry_exhaustion"]
        result = v8.run_probe(
            v8.PolicyKind.BOUNDED_RETRY,
            scenario,
            v8.ProbeConfig(recent_quiet_boundary_max_age_s=3.0, retry_max_aborts_after_boundary=1),
        )
        self.assertFalse(result.latched)
        self.assertEqual(result.retry_aborts_used, 2)
        self.assertTrue(any(event.reason == "retry_budget_exhausted" for event in result.trace))


if __name__ == "__main__":
    unittest.main()
