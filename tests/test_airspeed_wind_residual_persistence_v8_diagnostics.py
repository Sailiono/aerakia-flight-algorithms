"""Focused semantic checks for the independent v8 policy-shape probe.

These tests intentionally do not invoke the v7 campaign runner, touch its
protocol, or create a public validation artifact.  They prove that the four
diagnostic policy shapes keep continuity fail-closed and that the
recent-boundary age comparison is observable before a v8 train is frozen.
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
        self.assertEqual(report["policy_comparison_count"], 60)

        policies = report["policies"]
        for age in ("1s", "2s", "3s"):
            root_cause = policies[age]["midband_then_persistent_high"]
            expired = policies[age]["expired_boundary"]
            gap = policies[age]["gap_then_high"]
            if age == "1s":
                self.assertTrue(
                    all(
                        not result["latched"]
                        for name, result in root_cause.items()
                        if name != "partial_quiet_probation"
                    )
                )
                self.assertTrue(root_cause["partial_quiet_probation"]["latched"])
            else:
                self.assertTrue(
                    all(result["latched"] for result in root_cause.values())
                )
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

    def test_partial_quiet_probation_requires_stricter_high_episode(self) -> None:
        result = v8.run_probe(
            v8.PolicyKind.PARTIAL_QUIET_PROBATION,
            v8.diagnostic_scenarios()["midband_then_persistent_high"],
            v8.ProbeConfig(recent_quiet_boundary_max_age_s=1.0),
        )
        self.assertTrue(result.latched)
        self.assertTrue(result.episode_boundary_is_partial)
        self.assertEqual(result.latch_source_timestamp_us, 71_000_000)
        self.assertEqual(result.episode_start_source_timestamp_us, 69_000_000)
        self.assertEqual(result.trace[-1].high_observations, 5)

    def test_partial_probation_cannot_bootstrap_initial_qualification(self) -> None:
        config = v8.ProbeConfig(recent_quiet_boundary_max_age_s=3.0)
        result = v8.run_probe(
            v8.PolicyKind.PARTIAL_QUIET_PROBATION,
            [
                *[
                    v8.ProbeInput(int(t * 1e6), int((t + 0.05) * 1e6), 1.0)
                    for t in (0.0, 0.5, 1.0, 1.5, 2.0)
                ],
                *[
                    v8.ProbeInput(int(t * 1e6), int((t + 0.05) * 1e6), 5.0)
                    for t in (2.5, 3.0, 3.5, 4.0, 4.5)
                ],
            ],
            config,
        )
        self.assertFalse(result.latched)
        self.assertFalse(result.full_boundary_seen)
        self.assertTrue(any(event.reason == "no_recent_quiet_boundary" for event in result.trace))

    def test_partial_policy_has_independent_finite_retry_budget(self) -> None:
        config = v8.ProbeConfig(
            recent_quiet_boundary_max_age_s=3.0,
            partial_retry_max_aborts_after_boundary=0,
        )
        result = v8.run_probe(
            v8.PolicyKind.PARTIAL_QUIET_PROBATION,
            [
                *[
                    v8.ProbeInput(int(t * 1e6), int((t + 0.05) * 1e6), 1.0)
                    for t in (0.0, 0.5, 1.0, 1.5, 2.0, 2.5)
                ],
                # Retire a complete boundary, then establish a distinct
                # partial boundary from new quiet observations.
                v8.ProbeInput(3_000_000, 3_050_000, 3.0),
                v8.ProbeInput(3_500_000, 3_550_000, 1.0),
                v8.ProbeInput(4_000_000, 4_050_000, 1.0),
                v8.ProbeInput(4_500_000, 4_550_000, 5.0),
                v8.ProbeInput(5_000_000, 5_050_000, 5.0),
                # With a zero retry budget, this abort exhausts the partial
                # boundary and subsequent high residuals cannot reuse it.
                v8.ProbeInput(5_500_000, 5_550_000, 3.0),
                *[
                    v8.ProbeInput(int(t * 1e6), int((t + 0.05) * 1e6), 5.0)
                    for t in (6.0, 6.5, 7.0, 7.5, 8.0)
                ],
            ],
            config,
        )
        self.assertFalse(result.latched)
        self.assertEqual(result.retry_aborts_used, 0)
        self.assertEqual(result.partial_retry_aborts_used, 1)
        self.assertTrue(result.partial_retry_exhausted)
        self.assertTrue(
            any(
                event.reason == "partial_retry_budget_exhausted_midband"
                for event in result.trace
            )
        )

    def test_partial_probation_requires_new_quiet_after_midband(self) -> None:
        result = v8.run_probe(
            v8.PolicyKind.PARTIAL_QUIET_PROBATION,
            [
                *[
                    v8.ProbeInput(int(t * 1e6), int((t + 0.05) * 1e6), 1.0)
                    for t in (0.0, 0.5, 1.0, 1.5, 2.0, 2.5)
                ],
                # A mid-band sample retires the full boundary.  High evidence
                # alone cannot start a probationary episode; it needs a new
                # post-mid-band partial quiet run first.
                v8.ProbeInput(3_000_000, 3_050_000, 3.0),
                *[
                    v8.ProbeInput(int(t * 1e6), int((t + 0.05) * 1e6), 5.0)
                    for t in (3.5, 4.0, 4.5, 5.0, 5.5, 6.0)
                ],
            ],
            v8.ProbeConfig(recent_quiet_boundary_max_age_s=3.0),
        )
        self.assertFalse(result.latched)
        self.assertTrue(result.full_boundary_seen)
        self.assertTrue(any(event.reason == "partial_boundary_retired_midband" for event in result.trace))
        self.assertTrue(any(event.reason == "no_recent_quiet_boundary" for event in result.trace))

    @staticmethod
    def _feed(probe: v8.CausalPolicyProbe, samples: list[tuple[float, float]]) -> v8.ProbeResult:
        for source_s, nis in samples:
            source_us = int(round(source_s * 1e6))
            arrival_us = int(round((source_s + 0.05) * 1e6))
            probe.observe(v8.ProbeInput(source_us, arrival_us, nis))
        return probe.result()

    def test_boundary_age_exactly_at_expiry_is_admissible(self) -> None:
        for policy in (v8.PolicyKind.RECENT_BOUNDARY, v8.PolicyKind.GRADED_EVIDENCE):
            with self.subTest(policy=policy.value):
                probe = v8.CausalPolicyProbe(
                    policy,
                    v8.ProbeConfig(recent_quiet_boundary_max_age_s=3.0),
                )
                probe.reauthorize(0)
                samples = [(t, 1.0) for t in (0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0)]
                samples += [(5.0, 3.0), (6.0, 3.0)]
                samples += [(7.0, 5.0), (7.5, 5.0), (8.0, 5.0), (8.5, 5.0)]
                result = self._feed(probe, samples)
                self.assertTrue(result.latched)
                self.assertEqual(result.episode_start_source_timestamp_us, 7_000_000)
                onset = next(event for event in result.trace if event.source_timestamp_us == 7_000_000)
                self.assertAlmostEqual(onset.recent_boundary_age_s or -1.0, 3.0)

    def test_boundary_age_just_past_expiry_is_rejected(self) -> None:
        for policy in (v8.PolicyKind.RECENT_BOUNDARY, v8.PolicyKind.GRADED_EVIDENCE):
            with self.subTest(policy=policy.value):
                probe = v8.CausalPolicyProbe(
                    policy,
                    v8.ProbeConfig(recent_quiet_boundary_max_age_s=3.0),
                )
                probe.reauthorize(0)
                samples = [(t, 1.0) for t in (0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0)]
                samples += [(5.0, 3.0), (6.0, 3.0)]
                onset = 7.000001
                samples += [(onset + index * 0.5, 5.0) for index in range(4)]
                result = self._feed(probe, samples)
                self.assertFalse(result.latched)
                self.assertTrue(any(event.reason == "no_recent_quiet_boundary" for event in result.trace))

    def test_episode_onset_before_expiry_can_complete_after_expiry(self) -> None:
        probe = v8.CausalPolicyProbe(
            v8.PolicyKind.GRADED_EVIDENCE,
            v8.ProbeConfig(recent_quiet_boundary_max_age_s=3.0),
        )
        probe.reauthorize(0)
        samples = [(t, 1.0) for t in (0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0)]
        samples += [(5.0, 3.0), (6.0, 3.0)]
        samples += [(6.5, 5.0), (7.0, 5.0), (7.5, 5.0), (8.0, 5.0)]
        result = self._feed(probe, samples)
        self.assertTrue(result.latched)
        self.assertEqual(result.episode_start_source_timestamp_us, 6_500_000)
        self.assertEqual(result.latch_source_timestamp_us, 8_000_000)

    def test_graded_evidence_uses_source_span_under_rate_and_arrival_jitter(self) -> None:
        probe = v8.CausalPolicyProbe(
            v8.PolicyKind.GRADED_EVIDENCE,
            v8.ProbeConfig(recent_quiet_boundary_max_age_s=3.0),
        )
        probe.reauthorize(0)
        for source_s in (index / 10.0 for index in range(36)):
            source_us = int(round(source_s * 1e6))
            arrival_offset = 0.02 + (0.03 if int(round(source_s * 10)) % 3 == 0 else 0.0)
            probe.observe(v8.ProbeInput(source_us, int(round((source_s + arrival_offset) * 1e6)), 1.0))
        high_times = (4.5, 4.63, 4.91, 5.37, 5.99, 6.18, 6.42, 6.63)
        for index, source_s in enumerate(high_times):
            source_us = int(round(source_s * 1e6))
            arrival_offset = 0.03 + (0.07 if index % 2 else 0.0)
            probe.observe(v8.ProbeInput(source_us, int(round((source_s + arrival_offset) * 1e6)), 5.0))
        result = probe.result()
        self.assertTrue(result.latched)
        self.assertEqual(result.episode_start_source_timestamp_us, 4_500_000)
        self.assertEqual(result.latch_source_timestamp_us, 6_180_000)


if __name__ == "__main__":
    unittest.main()
