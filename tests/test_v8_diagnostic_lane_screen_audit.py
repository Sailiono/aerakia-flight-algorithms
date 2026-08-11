"""Unit tests for v8 diagnostic-lane attribution accounting."""

from __future__ import annotations

import sys
import unittest

sys.path.insert(0, "validation")

import audit_v8_diagnostic_lane_screen as audit  # noqa: E402


def result(
    *,
    latched: bool,
    episode_start_us: int | None,
    delay_s: float | None,
    diagnostic_start_us: int | None = None,
) -> dict[str, object]:
    return {
        "latched": latched,
        "episode_start_source_timestamp_us": episode_start_us,
        "source_detection_delay_s": delay_s,
        "diagnostic_snapshot": (
            None
            if diagnostic_start_us is None
            else {"episode_start_source_timestamp_us": diagnostic_start_us}
        ),
    }


class V8DiagnosticLaneScreenAuditTests(unittest.TestCase):
    def test_clean_control_latch_is_not_diagnostic_only(self) -> None:
        flags = audit.classify_member(
            result=result(latched=True, episode_start_us=10_000_000, delay_s=1.5),
            injection_source_us=10_000_000,
            maximum_delay_s=6.0,
        )
        self.assertTrue(flags["control_clean"])
        self.assertFalse(flags["control_preexisting_ambiguous"])
        self.assertFalse(flags["diagnostic_clean"])
        self.assertTrue(flags["descriptive_union_clean"])

    def test_preexisting_control_latch_is_not_credited_clean(self) -> None:
        flags = audit.classify_member(
            result=result(latched=True, episode_start_us=9_999_999, delay_s=1.0),
            injection_source_us=10_000_000,
            maximum_delay_s=6.0,
        )
        self.assertFalse(flags["control_clean"])
        self.assertTrue(flags["control_preexisting_ambiguous"])
        self.assertFalse(flags["descriptive_union_clean"])

    def test_diagnostic_only_episode_can_enter_descriptive_union(self) -> None:
        flags = audit.classify_member(
            result=result(
                latched=False,
                episode_start_us=None,
                delay_s=None,
                diagnostic_start_us=10_000_001,
            ),
            injection_source_us=10_000_000,
            maximum_delay_s=6.0,
        )
        self.assertFalse(flags["control_clean"])
        self.assertTrue(flags["diagnostic_clean"])
        self.assertTrue(flags["descriptive_union_clean"])

    def test_preexisting_diagnostic_episode_remains_ambiguous(self) -> None:
        flags = audit.classify_member(
            result=result(
                latched=False,
                episode_start_us=None,
                delay_s=None,
                diagnostic_start_us=9_999_999,
            ),
            injection_source_us=10_000_000,
            maximum_delay_s=6.0,
        )
        self.assertTrue(flags["diagnostic_present"])
        self.assertTrue(flags["diagnostic_preexisting_ambiguous"])
        self.assertFalse(flags["diagnostic_clean"])
        self.assertFalse(flags["descriptive_union_clean"])

    def test_late_control_latch_is_not_credited_clean(self) -> None:
        flags = audit.classify_member(
            result=result(latched=True, episode_start_us=10_000_000, delay_s=6.1),
            injection_source_us=10_000_000,
            maximum_delay_s=6.0,
        )
        self.assertTrue(flags["control_late"])
        self.assertFalse(flags["control_clean"])
        self.assertFalse(flags["descriptive_union_clean"])

    def test_family_summary_keeps_control_and_union_separate(self) -> None:
        rows = []
        for case in audit.PERSISTENT_CASES:
            rows.append(
                (
                    1,
                    case,
                    {
                        "raw_control_latch": False,
                        "control_clean": False,
                        "control_preexisting_ambiguous": False,
                        "control_late": False,
                        "diagnostic_present": True,
                        "diagnostic_clean": True,
                        "diagnostic_preexisting_ambiguous": False,
                        "descriptive_union_clean": True,
                    },
                )
            )
        summary = audit.summarize_members(rows)
        self.assertEqual(summary["control_clean_family_passes"], 0)
        self.assertEqual(summary["descriptive_union_clean_family_passes"], 1)
        self.assertEqual(len(summary["rescued_families"]), 1)


if __name__ == "__main__":
    unittest.main()
