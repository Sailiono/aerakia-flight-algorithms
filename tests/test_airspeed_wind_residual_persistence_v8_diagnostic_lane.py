"""Focused contract tests for the v8 diagnostic-only residual lane.

The diagnostic lane is intentionally orthogonal to the existing policy
comparator.  It may record an unqualified persistent residual after a quiet
boundary expires, but it must never turn that observation into a normal
``ProbeState.LATCHED`` result or a production-control claim.
"""

from __future__ import annotations

import sys
import unittest
from dataclasses import fields

sys.path.insert(0, "validation")

import diagnose_airspeed_wind_residual_persistence_v8 as v8  # noqa: E402


class ResidualPersistenceV8DiagnosticLaneTests(unittest.TestCase):
    @staticmethod
    def _input(source_s: float, nis: float, *, arrival_offset_s: float = 0.05) -> v8.ProbeInput:
        return v8.ProbeInput(
            int(round(source_s * 1_000_000.0)),
            int(round((source_s + arrival_offset_s) * 1_000_000.0)),
            nis,
        )

    @classmethod
    def _stale_boundary_stream(cls, onset_s: float = 7.000001) -> list[v8.ProbeInput]:
        # The mid-band samples maintain transport continuity while leaving the
        # last complete quiet boundary at 4 s.  The first high sample is just
        # beyond a 3 s recent-boundary horizon.
        samples = [
            cls._input(t, 1.0)
            for t in (0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0)
        ]
        samples.extend(cls._input(t, 3.0) for t in (5.0, 6.0))
        samples.extend(cls._input(onset_s + 0.5 * index, 5.0) for index in range(4))
        return samples

    @staticmethod
    def _run(events: list[v8.ProbeInput], age_s: float = 3.0) -> v8.ProbeResult:
        return v8.run_probe(
            v8.PolicyKind.RECENT_BOUNDARY,
            events,
            v8.ProbeConfig(recent_quiet_boundary_max_age_s=age_s),
        )

    def test_expired_boundary_records_diagnostic_without_control_latch(self) -> None:
        result = self._run(self._stale_boundary_stream())
        self.assertFalse(result.latched)
        self.assertEqual(result.final_state, v8.ProbeState.UNQUALIFIED.value)
        self.assertEqual(
            result.diagnostic_status,
            v8.DiagnosticResidualStatus.UNQUALIFIED_PERSISTENT_RESIDUAL.value,
        )
        self.assertIsNotNone(result.diagnostic_snapshot)
        assert result.diagnostic_snapshot is not None
        self.assertEqual(result.diagnostic_snapshot.prior_quiet_boundary_source_timestamp_us, 4_000_000)
        self.assertEqual(result.diagnostic_snapshot.episode_start_source_timestamp_us, 7_000_001)
        self.assertEqual(result.diagnostic_snapshot.latch_source_timestamp_us, 8_500_001)
        self.assertEqual(result.diagnostic_snapshot.high_observations, 4)
        self.assertAlmostEqual(result.diagnostic_snapshot.high_source_span_s, 1.5)

    def test_diagnostic_snapshot_is_immutable_after_latch(self) -> None:
        probe = v8.CausalPolicyProbe(
            v8.PolicyKind.RECENT_BOUNDARY,
            v8.ProbeConfig(recent_quiet_boundary_max_age_s=3.0),
        )
        probe.reauthorize(0)
        events = self._stale_boundary_stream()
        for event in events:
            probe.observe(event)
        first = probe.result().diagnostic_snapshot
        self.assertIsNotNone(first)
        for t in (9.0, 9.5, 10.0):
            probe.observe(self._input(t, 5.0))
        self.assertEqual(probe.result().diagnostic_snapshot, first)
        self.assertEqual(probe.result().diagnostic_status, v8.DiagnosticResidualStatus.UNQUALIFIED_PERSISTENT_RESIDUAL.value)

    def test_fresh_boundary_uses_normal_policy_without_diagnostic_lane(self) -> None:
        result = self._run(
            [
                *[self._input(t, 1.0) for t in (0.0, 0.5, 1.0, 1.5, 2.0)],
                *[self._input(t, 1.0) for t in (2.5,)],
                *[self._input(t, 5.0) for t in (3.0, 3.5, 4.0, 4.5)],
            ]
        )
        self.assertTrue(result.latched)
        self.assertEqual(result.final_state, v8.ProbeState.LATCHED.value)
        self.assertEqual(result.diagnostic_status, v8.DiagnosticResidualStatus.NONE.value)
        self.assertIsNone(result.diagnostic_snapshot)

    def test_unqualified_startup_high_never_creates_diagnostic(self) -> None:
        result = self._run(
            [self._input(t, 5.0) for t in (0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0)]
        )
        self.assertFalse(result.latched)
        self.assertEqual(result.diagnostic_status, v8.DiagnosticResidualStatus.NONE.value)
        self.assertIsNone(result.diagnostic_snapshot)
        self.assertFalse(result.full_boundary_seen)

    def test_gap_clears_diagnostic_snapshot_and_epoch_provenance(self) -> None:
        probe = v8.CausalPolicyProbe(
            v8.PolicyKind.RECENT_BOUNDARY,
            v8.ProbeConfig(recent_quiet_boundary_max_age_s=3.0),
        )
        probe.reauthorize(0)
        for event in self._stale_boundary_stream():
            probe.observe(event)
        self.assertIsNotNone(probe.result().diagnostic_snapshot)
        probe.observe(self._input(12.0, 5.0))  # source gap, fail-closed reset
        result = probe.result()
        self.assertEqual(result.reset_count, 1)
        self.assertEqual(result.diagnostic_status, v8.DiagnosticResidualStatus.NONE.value)
        self.assertIsNone(result.diagnostic_snapshot)
        self.assertFalse(result.full_boundary_seen)

    def test_midband_interrupt_does_not_bridge_unfinished_diagnostic_episode(self) -> None:
        events = self._stale_boundary_stream()[:11]  # through 6 s mid-band
        events.extend(self._input(t, 5.0) for t in (7.000001, 7.500001, 8.000001))
        events.append(self._input(8.500001, 3.0))
        events.extend(self._input(t, 5.0) for t in (9.000001, 9.500001, 10.000001, 10.500001))
        result = self._run(events)
        self.assertFalse(result.latched)
        self.assertEqual(result.diagnostic_status, v8.DiagnosticResidualStatus.NONE.value)
        self.assertIsNone(result.diagnostic_snapshot)

    def test_boundary_age_splits_control_and_diagnostic_paths(self) -> None:
        exact = self._run(self._stale_boundary_stream(onset_s=7.0))
        just_late = self._run(self._stale_boundary_stream(onset_s=7.000001))
        self.assertTrue(exact.latched)
        self.assertEqual(exact.diagnostic_status, v8.DiagnosticResidualStatus.NONE.value)
        self.assertFalse(just_late.latched)
        self.assertEqual(
            just_late.diagnostic_status,
            v8.DiagnosticResidualStatus.UNQUALIFIED_PERSISTENT_RESIDUAL.value,
        )

    def test_source_time_not_sample_count_controls_diagnostic_span(self) -> None:
        sparse = self._stale_boundary_stream()
        # Keep the same source-time onset/span but reduce the number of high
        # observations; count and span are both required.
        sparse = sparse[:11] + [self._input(t, 5.0) for t in (7.000001, 7.800001, 8.500001)]
        result = self._run(sparse)
        self.assertFalse(result.latched)
        self.assertEqual(result.diagnostic_status, v8.DiagnosticResidualStatus.NONE.value)
        self.assertIsNone(result.diagnostic_snapshot)

    def test_payload_contains_no_truth_or_injection_fields(self) -> None:
        result = self._run(self._stale_boundary_stream())
        payload = v8._result_payload(result)
        self.assertIn("diagnostic_status", payload)
        self.assertIn("diagnostic_snapshot", payload)
        forbidden = {"injection", "truth", "scenario", "case_name", "fault_label"}
        self.assertTrue(forbidden.isdisjoint(payload))
        event_fields = {field.name for field in fields(v8.ProbeEvent)}
        self.assertTrue(forbidden.isdisjoint(event_fields))

    def test_diagnostic_status_is_not_control_authority(self) -> None:
        result = self._run(self._stale_boundary_stream())
        self.assertEqual(result.final_state, v8.ProbeState.UNQUALIFIED.value)
        self.assertFalse(result.latched)
        self.assertNotEqual(
            result.diagnostic_status,
            v8.ProbeState.LATCHED.value,
        )


if __name__ == "__main__":
    unittest.main()
