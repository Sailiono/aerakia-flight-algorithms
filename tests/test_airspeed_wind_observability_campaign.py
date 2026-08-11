"""Integrity tests for the frozen multi-seed TAS/wind confirmation campaign."""

from __future__ import annotations

import copy
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "validation"))
import run_airspeed_wind_observability as oracle  # noqa: E402
import run_airspeed_wind_observability_campaign as campaign_runner  # noqa: E402


class AirspeedWindCampaignTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.campaign = campaign_runner.load_campaign_protocol()
        _path, cls.base = campaign_runner.resolve_base_protocol(cls.campaign)

    def test_frozen_campaign_names_match_every_base_case_once(self) -> None:
        names = [str(case["name"]) for case in self.base["scenario_matrix"]]
        self.assertEqual(self.campaign["scenario_names"], names)
        self.assertEqual(len(self.campaign["scenario_names"]), len(set(self.campaign["scenario_names"])))

    def test_duplicate_confirmation_seed_is_rejected(self) -> None:
        bad = copy.deepcopy(self.campaign)
        bad["confirmation_seed_splits"]["replication_b_1"][0] = bad["confirmation_seed_splits"]["replication_a_1"][0]
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "campaign.json"
            path.write_text(json.dumps(bad), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "multiple splits"):
                campaign_runner.load_campaign_protocol(path)

    def test_small_disjoint_confirmation_matrix_reuses_frozen_case_rules(self) -> None:
        small = copy.deepcopy(self.campaign)
        small["scenario_names"] = ["multi_heading_nominal", "straight_line", "wind_shear"]
        small["confirmation_seed_splits"] = {"replication_a": [101], "replication_b": [202]}
        result = campaign_runner.run_campaign(small, self.base, jobs=1)
        self.assertEqual(result["status"], "passed")
        self.assertEqual(result["totals"]["replication_cases"], 6)
        self.assertEqual(result["totals"]["failed_replication_cases"], 0)
        self.assertEqual(result["failure_categories"], {})
        wind_shear = next(item for item in result["scenario_summaries"] if item["name"] == "wind_shear")
        self.assertEqual(wind_shear["terminal_status_counts"], {"source_latched": 2})

    def test_complete_shards_merge_once_without_overlap(self) -> None:
        small = copy.deepcopy(self.campaign)
        small["scenario_names"] = ["multi_heading_nominal", "straight_line"]
        small["confirmation_seed_splits"] = {"replication_a": [101], "replication_b": [202]}
        shards: list[tuple[str, dict[str, object]]] = []
        for split in small["confirmation_seed_splits"]:
            scoped = copy.deepcopy(small)
            scoped["confirmation_seed_splits"] = {split: small["confirmation_seed_splits"][split]}
            shards.append((split, campaign_runner.run_campaign(scoped, self.base, jobs=1, include_records=True)))
        merged = campaign_runner.merge_shard_results(small, self.base, shards)
        self.assertEqual(merged["status"], "passed")
        self.assertEqual(merged["totals"]["replication_cases"], 4)
        self.assertEqual(merged["execution_backend"], "merged_shards")

    def test_record_digest_changes_when_a_record_changes(self) -> None:
        records = [{"scenario": "a", "passed": True}, {"scenario": "b", "passed": True}]
        first = oracle.canonical_sha256(records)
        changed = copy.deepcopy(records)
        changed[1]["passed"] = False
        self.assertNotEqual(first, oracle.canonical_sha256(changed))


if __name__ == "__main__":
    unittest.main()
