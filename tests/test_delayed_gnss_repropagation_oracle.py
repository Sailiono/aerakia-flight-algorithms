"""Unit tests for the delayed-GNSS oracle's fail-closed campaign checks."""

from __future__ import annotations

import importlib.util
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "run_delayed_gnss_repropagation_oracle",
    ROOT / "validation" / "run_delayed_gnss_repropagation_oracle.py",
)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def passing_payload() -> dict[str, object]:
    return {
        "status": MODULE.EXPECTED_STATUS,
        "input_contract": MODULE.EXPECTED_INPUT_CONTRACT,
        "source_gps_accepted": True,
        "repropagated": True,
        "healthy": True,
        "covariance_psd": True,
        "pre_delivery_max_state_difference": 1.0e-3,
        "post_delivery_max_state_difference": 0.0,
        "post_delivery_max_covariance_difference": 0.0,
        "post_delivery_metadata_match": True,
        "state_tolerance": 1.0e-12,
        "covariance_tolerance": 1.0e-12,
        "pass": True,
    }


def passing_sequential_payload() -> dict[str, object]:
    first = {
        "event_index": 1,
        "source_timestamp_us": 3_000_000,
        "delivery_timestamp_us": 3_150_000,
        "source_gps_accepted": True,
        "repropagated": True,
        "pre_delivery_max_state_difference": 1.0e-3,
        "post_delivery_max_state_difference": 0.0,
        "post_delivery_max_covariance_difference": 0.0,
        "post_delivery_metadata_match": True,
    }
    second = {
        "event_index": 2,
        "source_timestamp_us": 4_000_000,
        "delivery_timestamp_us": 4_150_000,
        "source_gps_accepted": True,
        "repropagated": True,
        "pre_delivery_max_state_difference": 1.0e-3,
        "post_delivery_max_state_difference": 0.0,
        "post_delivery_max_covariance_difference": 0.0,
        "post_delivery_metadata_match": True,
    }
    return {
        "status": MODULE.EXPECTED_STATUS,
        "scenario": MODULE.SEQUENTIAL_SCENARIO,
        "input_contract": MODULE.EXPECTED_INPUT_CONTRACT,
        "sequential_non_overlapping": True,
        "healthy": True,
        "covariance_psd": True,
        "state_tolerance": 1.0e-12,
        "covariance_tolerance": 1.0e-12,
        "event_count": 2,
        "events": [first, second],
        "final_max_state_difference": 0.0,
        "final_max_covariance_difference": 0.0,
        "final_metadata_match": True,
        "pass": True,
    }


def passing_overlap_payload() -> dict[str, object]:
    older = {
        "event_index": 1,
        "source_timestamp_us": 3_000_000,
        "delivery_timestamp_us": 3_150_000,
        "source_gps_accepted": True,
        "repropagated": True,
        "pending_earlier_event_count_at_delivery": 0,
        "zero_delay_equivalence_required_after_delivery": True,
        "pre_delivery_max_state_difference": 1.0e-3,
        "post_delivery_max_state_difference": 0.0,
        "post_delivery_max_covariance_difference": 0.0,
        "post_delivery_metadata_match": True,
    }
    newer = {
        "event_index": 2,
        "source_timestamp_us": 3_020_000,
        "delivery_timestamp_us": 3_030_000,
        "source_gps_accepted": True,
        "repropagated": True,
        "pending_earlier_event_count_at_delivery": 1,
        "zero_delay_equivalence_required_after_delivery": False,
        "pre_delivery_max_state_difference": 1.0e-3,
        "post_delivery_max_state_difference": 1.0e-4,
        "post_delivery_max_covariance_difference": 1.0e-3,
        "post_delivery_metadata_match": True,
    }
    return {
        "status": MODULE.EXPECTED_STATUS,
        "scenario": MODULE.OVERLAPPING_SCENARIO,
        "input_contract": MODULE.EXPECTED_INPUT_CONTRACT,
        "overlapping_delivery_windows": True,
        "delivery_order_reversed": True,
        "healthy": True,
        "covariance_psd": True,
        "state_tolerance": 1.0e-12,
        "covariance_tolerance": 1.0e-12,
        "event_count": 2,
        "events": [older, newer],
        "final_max_state_difference": 0.0,
        "final_max_covariance_difference": 0.0,
        "final_metadata_match": True,
        "pass": True,
    }


class DelayedGnssRepropagationOracleTest(unittest.TestCase):
    def test_portable_path_removes_workstation_root(self) -> None:
        self.assertEqual(
            MODULE.portable_path(ROOT / "build" / "oracle"),
            "build/oracle",
        )

    def test_complete_case_passes_all_checks(self) -> None:
        checks = MODULE.validate_case(passing_payload())
        self.assertTrue(all(check["passed"] for check in checks))

    def test_missing_metadata_fails_closed(self) -> None:
        payload = passing_payload()
        payload["post_delivery_metadata_match"] = False
        checks = MODULE.validate_case(payload)
        failed = {check["name"] for check in checks if not check["passed"]}
        self.assertIn("post_delivery_metadata_matches_zero_delay", failed)

    def test_post_replay_error_above_tolerance_fails(self) -> None:
        payload = passing_payload()
        payload["post_delivery_max_state_difference"] = 1.0e-6
        checks = MODULE.validate_case(payload)
        failed = {check["name"] for check in checks if not check["passed"]}
        self.assertIn("post_delivery_state_matches_zero_delay", failed)

    def test_sequential_case_requires_both_replays(self) -> None:
        checks = MODULE.validate_case(passing_sequential_payload())
        self.assertTrue(all(check["passed"] for check in checks))

    def test_sequential_case_rejects_overlapping_schedule(self) -> None:
        payload = passing_sequential_payload()
        events = payload["events"]
        assert isinstance(events, list)
        second = events[1]
        assert isinstance(second, dict)
        second["source_timestamp_us"] = 3_150_000
        checks = MODULE.validate_case(payload)
        failed = {check["name"] for check in checks if not check["passed"]}
        self.assertIn("second_source_is_after_first_delivery", failed)

    def test_sequential_case_requires_second_repropagation(self) -> None:
        payload = passing_sequential_payload()
        events = payload["events"]
        assert isinstance(events, list)
        second = events[1]
        assert isinstance(second, dict)
        second["repropagated"] = False
        checks = MODULE.validate_case(payload)
        failed = {check["name"] for check in checks if not check["passed"]}
        self.assertIn("second_repropagation_executed", failed)

    def test_overlap_case_passes_all_checks(self) -> None:
        checks = MODULE.validate_case(passing_overlap_payload())
        self.assertTrue(all(check["passed"] for check in checks))

    def test_overlap_case_rejects_non_overlapping_delivery_window(self) -> None:
        payload = passing_overlap_payload()
        payload["overlapping_delivery_windows"] = False
        checks = MODULE.validate_case(payload)
        failed = {check["name"] for check in checks if not check["passed"]}
        self.assertIn("overlap_delivery_windows_are_explicit_and_real", failed)

    def test_overlap_case_rejects_non_reversed_delivery_order(self) -> None:
        payload = passing_overlap_payload()
        events = payload["events"]
        assert isinstance(events, list)
        newer = events[1]
        assert isinstance(newer, dict)
        newer["delivery_timestamp_us"] = 3_160_000
        checks = MODULE.validate_case(payload)
        failed = {check["name"] for check in checks if not check["passed"]}
        self.assertIn("overlap_delivery_order_is_reversed", failed)

    def test_overlap_case_rejects_non_chronological_source_order(self) -> None:
        payload = passing_overlap_payload()
        events = payload["events"]
        assert isinstance(events, list)
        newer = events[1]
        assert isinstance(newer, dict)
        newer["source_timestamp_us"] = 3_000_000
        checks = MODULE.validate_case(payload)
        failed = {check["name"] for check in checks if not check["passed"]}
        self.assertIn("overlap_source_order_is_chronological", failed)

    def test_overlap_case_requires_pending_earlier_event(self) -> None:
        payload = passing_overlap_payload()
        events = payload["events"]
        assert isinstance(events, list)
        newer = events[1]
        assert isinstance(newer, dict)
        newer["pending_earlier_event_count_at_delivery"] = 0
        checks = MODULE.validate_case(payload)
        failed = {check["name"] for check in checks if not check["passed"]}
        self.assertIn("newer_delivery_reports_one_pending_earlier_event", failed)

    def test_overlap_case_rejects_premature_equivalence(self) -> None:
        payload = passing_overlap_payload()
        events = payload["events"]
        assert isinstance(events, list)
        newer = events[1]
        assert isinstance(newer, dict)
        newer["post_delivery_max_state_difference"] = 0.0
        checks = MODULE.validate_case(payload)
        failed = {check["name"] for check in checks if not check["passed"]}
        self.assertIn(
            "newer_delivery_remains_observably_different_while_older_event_is_pending",
            failed,
        )

    def test_overlap_delay_below_pending_window_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "overlap requires delays"):
            MODULE.parse_delays("overlap", "20,50")


if __name__ == "__main__":
    unittest.main()
