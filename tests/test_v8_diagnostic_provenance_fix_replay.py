"""Focused tests for paired replay of the v8 diagnostic provenance fix."""

from __future__ import annotations

import sys
import unittest

sys.path.insert(0, "validation")

import diagnose_airspeed_wind_residual_persistence_v8 as v8  # noqa: E402
import replay_v8_diagnostic_provenance_fix as replay  # noqa: E402


class V8DiagnosticProvenanceFixReplayTests(unittest.TestCase):
    @staticmethod
    def _record() -> dict[str, object]:
        inputs = [
            v8.ProbeInput(int(t * 1e6), int((t + 0.05) * 1e6), 1.0)
            for t in (0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0)
        ]
        inputs.extend(
            v8.ProbeInput(int(t * 1e6), int((t + 0.05) * 1e6), 3.0)
            for t in (5.0, 6.0, 7.000001)
        )
        inputs.extend(
            v8.ProbeInput(int(t * 1e6), int((t + 0.05) * 1e6), 5.0)
            for t in (7.500001, 8.000001, 8.500001, 9.000001)
        )
        result = v8.run_probe(
            v8.PolicyKind.RECENT_BOUNDARY,
            inputs,
            v8.ProbeConfig(recent_quiet_boundary_max_age_s=1.0),
        )
        return {
            "candidate_policies": {
                "1s": {
                    v8.PolicyKind.RECENT_BOUNDARY.value: v8._result_payload(result)
                }
            }
        }

    def test_causal_input_recovery_has_no_truth_or_case_label(self) -> None:
        inputs = replay.causal_inputs_from_record(self._record())
        self.assertGreater(len(inputs), 0)
        self.assertTrue(all(item.source_epoch == 0 for item in inputs))
        self.assertTrue(all(item.source_valid == (item.nis is not None) for item in inputs))
        self.assertFalse(any(hasattr(item, "injection") for item in inputs))

    def test_primary_comparator_ignores_diagnostic_only_delta(self) -> None:
        inputs = replay.causal_inputs_from_record(self._record())
        current = v8.run_probe(
            v8.PolicyKind.RECENT_BOUNDARY,
            inputs,
            v8.ProbeConfig(recent_quiet_boundary_max_age_s=1.0),
        )
        old = v8._result_payload(current)
        old["diagnostic_status"] = v8.DiagnosticResidualStatus.NONE.value
        old["diagnostic_snapshot"] = None
        self.assertTrue(replay.primary_comparator_matches(old, current))

    def test_primary_comparator_detects_policy_state_change(self) -> None:
        inputs = replay.causal_inputs_from_record(self._record())
        current = v8.run_probe(
            v8.PolicyKind.RECENT_BOUNDARY,
            inputs,
            v8.ProbeConfig(recent_quiet_boundary_max_age_s=1.0),
        )
        old = v8._result_payload(current)
        old["latched"] = not current.latched
        self.assertFalse(replay.primary_comparator_matches(old, current))


if __name__ == "__main__":
    unittest.main()
