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


class DelayedGnssRepropagationOracleTest(unittest.TestCase):
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


if __name__ == "__main__":
    unittest.main()
