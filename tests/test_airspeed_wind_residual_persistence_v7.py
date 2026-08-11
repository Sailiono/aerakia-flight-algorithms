"""Focused contract checks for the standalone v7 persistence monitor."""

from __future__ import annotations

import math
import copy
import json
import tempfile
import unittest
from dataclasses import FrozenInstanceError
from pathlib import Path
from unittest import mock

import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "validation"))

import airspeed_wind_residual_persistence_v7 as v7
import run_airspeed_wind_residual_persistence_v7 as campaign


class ResidualPersistenceV7Tests(unittest.TestCase):
    def make_monitor(self, **changes: object) -> v7.SourceTimeResidualPersistenceMonitor:
        values: dict[str, object] = {
            "warmup_min_valid_observations": 3,
            "warmup_min_source_span_s": 0.5,
            "quiet_nis_threshold": 2.0,
            "high_nis_threshold": 4.0,
            "high_episode_min_observations": 4,
            "high_episode_min_source_span_s": 1.5,
            "maximum_source_gap_s": 1.25,
            "maximum_arrival_gap_s": 1.25,
            "pre_trigger_trace_capacity": 16,
            "post_latch_tail_capacity": 5,
        }
        values.update(changes)
        return v7.SourceTimeResidualPersistenceMonitor(
            v7.ResidualPersistenceConfig(**values)  # type: ignore[arg-type]
        )

    @staticmethod
    def observe(
        monitor: v7.SourceTimeResidualPersistenceMonitor,
        source_us: int,
        nis: float | None,
        *,
        arrival_us: int | None = None,
        epoch: int = 1,
        valid: bool = True,
    ) -> v7.MonitorDecision:
        return monitor.observe(
            source_epoch=epoch,
            source_timestamp_us=source_us,
            arrival_timestamp_us=source_us if arrival_us is None else arrival_us,
            nis=nis,
            source_valid=valid,
        )

    def qualify_at_rate(
        self,
        monitor: v7.SourceTimeResidualPersistenceMonitor,
        *,
        rate_hz: float,
        epoch: int = 1,
        arrival_delay_us: int = 0,
    ) -> int:
        interval_us = int(round(1.0e6 / rate_hz))
        source_us = 0
        for _ in range(1000):
            self.observe(
                monitor,
                source_us,
                1.0,
                arrival_us=source_us + arrival_delay_us,
                epoch=epoch,
            )
            if monitor.state is v7.MonitorState.QUIET_CONFIRMED:
                return source_us + interval_us
            source_us += interval_us
        self.fail("monitor did not obtain quiet qualification")

    def test_explicit_state_path_and_reauthorization(self) -> None:
        monitor = self.make_monitor()
        self.assertEqual(monitor.state, v7.MonitorState.UNQUALIFIED)
        rejected = self.observe(monitor, 0, 1.0)
        self.assertFalse(rejected.accepted_residual)
        self.assertEqual(rejected.reason, "source_epoch_not_authorized")

        monitor.reauthorize(1)
        next_source = self.qualify_at_rate(monitor, rate_hz=2.0)
        self.assertEqual(monitor.state, v7.MonitorState.QUIET_CONFIRMED)

        for index in range(4):
            decision = self.observe(monitor, next_source + index * 500_000, 5.0)
        self.assertEqual(decision.reason, "high_episode_latched")
        self.assertEqual(monitor.state, v7.MonitorState.LATCHED)
        self.assertIsNotNone(monitor.trigger_snapshot)

        retained = monitor.trigger_snapshot
        monitor.reauthorize(2)
        self.assertEqual(monitor.state, v7.MonitorState.UNQUALIFIED)
        self.assertEqual(monitor.authorized_source_epoch, 2)
        self.assertIs(monitor.trigger_snapshot, retained)

    def test_high_episode_uses_source_time_across_supported_rate_matrix(self) -> None:
        for rate_hz in (1.0, 1.5, 2.0, 5.0, 10.0, 20.0):
            with self.subTest(rate_hz=rate_hz):
                monitor = self.make_monitor(
                    maximum_source_gap_s=1.1,
                    maximum_arrival_gap_s=1.1,
                )
                monitor.reauthorize(1)
                interval_us = int(round(1.0e6 / rate_hz))
                next_source = self.qualify_at_rate(monitor, rate_hz=rate_hz)
                for index in range(1000):
                    source_us = next_source + index * interval_us
                    self.observe(monitor, source_us, 5.0)
                    if monitor.state is v7.MonitorState.LATCHED:
                        break
                self.assertEqual(monitor.state, v7.MonitorState.LATCHED)
                snapshot = monitor.trigger_snapshot
                self.assertIsNotNone(snapshot)
                assert snapshot is not None
                self.assertGreaterEqual(snapshot.episode_source_span_s, 1.5)
                self.assertGreaterEqual(len(snapshot.episode_observations), 4)
                self.assertEqual(
                    snapshot.latch_source_timestamp_us
                    - snapshot.episode_observations[0].source_timestamp_us,
                    int(round(snapshot.episode_source_span_s * 1.0e6)),
                )

    def test_source_and_arrival_gap_counters_are_independent(self) -> None:
        monitor = self.make_monitor(maximum_source_gap_s=1.0, maximum_arrival_gap_s=1.0)
        monitor.reauthorize(1)
        self.observe(monitor, 0, 1.0, arrival_us=5_000_000)
        source_only = self.observe(monitor, 2_000_000, 1.0, arrival_us=5_500_000)
        self.assertTrue(source_only.source_gap_observed)
        self.assertFalse(source_only.arrival_gap_observed)
        self.assertEqual(monitor.counters.source_gap_events, 1)
        self.assertEqual(monitor.counters.arrival_gap_events, 0)
        self.assertEqual(monitor.counters.gap_resets_empty_episode, 1)

        monitor.reauthorize(2)
        self.observe(monitor, 0, 1.0, epoch=2)
        arrival_only = self.observe(monitor, 500_000, 1.0, arrival_us=2_000_000, epoch=2)
        self.assertFalse(arrival_only.source_gap_observed)
        self.assertTrue(arrival_only.arrival_gap_observed)
        self.assertEqual(monitor.counters.source_gap_events, 1)
        self.assertEqual(monitor.counters.arrival_gap_events, 1)

    def test_gap_fact_survives_a_following_invalid_residual_decision(self) -> None:
        monitor = self.make_monitor(maximum_source_gap_s=0.75, maximum_arrival_gap_s=0.75)
        monitor.reauthorize(1)
        self.observe(monitor, 0, 1.0)
        decision = self.observe(monitor, 1_000_000, None)
        self.assertTrue(decision.source_gap_observed)
        self.assertTrue(decision.arrival_gap_observed)
        self.assertEqual(decision.reason, "missing_nis")
        self.assertEqual(monitor.counters.gap_resets_empty_episode, 1)

    def test_gap_telemetry_distinguishes_empty_and_nonempty_episode(self) -> None:
        monitor = self.make_monitor(maximum_source_gap_s=1.0, maximum_arrival_gap_s=1.0)
        monitor.reauthorize(1)
        next_source = self.qualify_at_rate(monitor, rate_hz=2.0)
        self.observe(monitor, next_source, 5.0)
        self.assertEqual(monitor.state, v7.MonitorState.HIGH_EPISODE)
        gap = self.observe(monitor, next_source + 2_000_000, 5.0)
        self.assertTrue(gap.source_gap_observed)
        self.assertTrue(gap.arrival_gap_observed)
        self.assertEqual(monitor.counters.gap_resets_nonempty_episode, 1)
        self.assertEqual(monitor.counters.gap_resets_empty_episode, 0)
        self.assertEqual(monitor.state, v7.MonitorState.UNQUALIFIED)
        self.assertEqual(len(monitor.high_episode_observations), 0)

    def test_monotonic_arrival_jitter_does_not_change_source_time_span(self) -> None:
        monitor = self.make_monitor(maximum_arrival_gap_s=2.0)
        monitor.reauthorize(1)
        next_source = self.qualify_at_rate(monitor, rate_hz=2.0, arrival_delay_us=200_000)
        delays = (180_000, 260_000, 210_000, 300_000)
        for index, delay_us in enumerate(delays):
            self.observe(
                monitor,
                next_source + index * 500_000,
                5.0,
                arrival_us=next_source + index * 500_000 + delay_us,
            )
        snapshot = monitor.trigger_snapshot
        self.assertIsNotNone(snapshot)
        assert snapshot is not None
        self.assertAlmostEqual(snapshot.episode_source_span_s, 1.5)
        self.assertEqual(
            snapshot.latch_arrival_timestamp_us - snapshot.latch_source_timestamp_us,
            delays[-1],
        )

    def test_nonmonotonic_source_or_arrival_never_contributes_and_revokes(self) -> None:
        monitor = self.make_monitor()
        monitor.reauthorize(1)
        self.observe(monitor, 0, 1.0, arrival_us=1_000_000)
        arrival_bad = self.observe(monitor, 500_000, 5.0, arrival_us=900_000)
        self.assertFalse(arrival_bad.accepted_residual)
        self.assertEqual(monitor.counters.nonmonotonic_arrival_events, 1)
        self.assertIsNone(monitor.authorized_source_epoch)
        self.assertEqual(len(monitor.high_episode_observations), 0)

        monitor.reauthorize(2)
        self.observe(monitor, 1_000_000, 1.0, arrival_us=1_100_000, epoch=2)
        source_bad = self.observe(monitor, 900_000, 5.0, arrival_us=1_200_000, epoch=2)
        self.assertFalse(source_bad.accepted_residual)
        self.assertEqual(monitor.counters.nonmonotonic_source_events, 1)
        self.assertIsNone(monitor.authorized_source_epoch)

    def test_invalid_no_nis_and_epoch_change_fail_closed(self) -> None:
        cases = (
            {"nis": None, "valid": True, "counter": "missing_nis_events"},
            {"nis": 1.0, "valid": False, "counter": "invalid_source_events"},
            {"nis": math.nan, "valid": True, "counter": "invalid_nis_events"},
        )
        for case in cases:
            with self.subTest(counter=case["counter"]):
                monitor = self.make_monitor()
                monitor.reauthorize(1)
                result = self.observe(
                    monitor,
                    0,
                    case["nis"],  # type: ignore[arg-type]
                    valid=bool(case["valid"]),
                )
                self.assertFalse(result.accepted_residual)
                self.assertIsNone(monitor.authorized_source_epoch)
                self.assertEqual(monitor.state, v7.MonitorState.UNQUALIFIED)
                self.assertEqual(getattr(monitor.counters, str(case["counter"])), 1)

        monitor = self.make_monitor()
        monitor.reauthorize(1)
        changed = self.observe(monitor, 0, 1.0, epoch=2)
        self.assertFalse(changed.accepted_residual)
        self.assertEqual(monitor.counters.source_epoch_mismatch_events, 1)
        self.assertEqual(monitor.counters.source_epoch_reset_events, 1)
        self.assertIsNone(monitor.authorized_source_epoch)

        monitor.reauthorize(3)
        self.qualify_at_rate(monitor, rate_hz=2.0, epoch=3)
        monitor.source_epoch_reset()
        self.assertEqual(monitor.state, v7.MonitorState.UNQUALIFIED)
        self.assertIsNone(monitor.authorized_source_epoch)
        self.assertEqual(monitor.counters.source_epoch_reset_events, 2)
        after_reset = self.observe(monitor, 10_000_000, 1.0, epoch=3)
        self.assertEqual(after_reset.reason, "source_epoch_not_authorized")

    def test_insufficient_warmup_coverage_cannot_confirm_quiet(self) -> None:
        monitor = self.make_monitor(
            warmup_min_valid_observations=5,
            warmup_min_source_span_s=2.0,
        )
        monitor.reauthorize(1)
        for index in range(4):
            self.observe(monitor, index * 500_000, 1.0)
        self.assertFalse(monitor.warmup_ready)
        self.assertEqual(monitor.state, v7.MonitorState.UNQUALIFIED)
        self.assertEqual(monitor.counters.warmup_completions, 0)
        self.assertEqual(monitor.counters.quiet_confirmations, 0)

    def test_no_quiet_coverage_cannot_start_or_latch_high_episode(self) -> None:
        monitor = self.make_monitor()
        monitor.reauthorize(1)
        for index in range(20):
            self.observe(monitor, index * 500_000, 5.0)
        self.assertEqual(monitor.state, v7.MonitorState.UNQUALIFIED)
        self.assertTrue(monitor.warmup_ready)
        self.assertEqual(monitor.counters.warmup_completions, 1)
        self.assertEqual(monitor.counters.quiet_confirmations, 0)
        self.assertEqual(monitor.counters.high_episode_starts, 0)
        self.assertEqual(monitor.counters.latches, 0)

    def test_midband_residual_requires_a_fresh_quiet_boundary(self) -> None:
        monitor = self.make_monitor()
        monitor.reauthorize(1)
        next_source = self.qualify_at_rate(monitor, rate_hz=2.0)
        middle = self.observe(monitor, next_source, 3.0)
        self.assertEqual(middle.state, v7.MonitorState.UNQUALIFIED)
        self.assertEqual(middle.reason, "quiet_confirmation_lost_midband")

        self.observe(monitor, next_source + 500_000, 5.0)
        self.assertEqual(monitor.state, v7.MonitorState.UNQUALIFIED)
        self.observe(monitor, next_source + 1_000_000, 1.0)
        self.observe(monitor, next_source + 1_500_000, 1.0)
        self.assertEqual(monitor.state, v7.MonitorState.QUIET_CONFIRMED)
        self.observe(monitor, next_source + 2_000_000, 5.0)
        self.assertEqual(monitor.state, v7.MonitorState.HIGH_EPISODE)
        aborted = self.observe(monitor, next_source + 2_500_000, 3.0)
        self.assertEqual(aborted.state, v7.MonitorState.UNQUALIFIED)
        self.assertEqual(aborted.reason, "high_episode_aborted_by_low_residual")
        self.assertEqual(len(monitor.high_episode_observations), 0)

    def test_trigger_snapshot_is_immutable_and_post_tail_is_bounded(self) -> None:
        monitor = self.make_monitor(post_latch_tail_capacity=5)
        monitor.reauthorize(1)
        next_source = self.qualify_at_rate(monitor, rate_hz=2.0)
        for index in range(4):
            self.observe(monitor, next_source + index * 500_000, 5.0)
        snapshot = monitor.trigger_snapshot
        self.assertIsNotNone(snapshot)
        assert snapshot is not None
        snapshot_value = snapshot
        snapshot_trace = snapshot.trace
        snapshot_episode = snapshot.episode_observations

        for index in range(100):
            self.observe(monitor, next_source + (index + 4) * 500_000, float(index % 7))
        self.assertIs(monitor.trigger_snapshot, snapshot_value)
        self.assertEqual(monitor.trigger_snapshot.trace, snapshot_trace)
        self.assertEqual(monitor.trigger_snapshot.episode_observations, snapshot_episode)
        self.assertEqual(len(monitor.post_latch_tail), 5)
        self.assertEqual(monitor.counters.post_latch_events, 100)
        self.assertTrue(all(event.reason == "already_latched" for event in monitor.post_latch_tail))
        with self.assertRaises(FrozenInstanceError):
            snapshot.episode_source_span_s = 0.0  # type: ignore[misc]

    def test_layered_telemetry_exposes_semantic_counts(self) -> None:
        monitor = self.make_monitor()
        monitor.reauthorize(7)
        self.observe(monitor, 0, 1.0, epoch=7)
        telemetry = monitor.telemetry()
        self.assertEqual(
            set(telemetry),
            {"input", "authorization", "continuity", "qualification", "evidence", "terminal"},
        )
        self.assertEqual(telemetry["input"]["input_events"], 1)
        self.assertEqual(telemetry["authorization"]["authorized_source_epoch"], 7)
        self.assertEqual(telemetry["qualification"]["state"], "UNQUALIFIED")

    def test_configuration_rejects_ambiguous_or_nonfinite_bounds(self) -> None:
        with self.assertRaises(ValueError):
            v7.ResidualPersistenceConfig(quiet_nis_threshold=4.0, high_nis_threshold=4.0)
        with self.assertRaises(ValueError):
            v7.ResidualPersistenceConfig(high_episode_min_source_span_s=math.nan)
        with self.assertRaises(ValueError):
            v7.ResidualPersistenceConfig(high_episode_min_observations=1)


class ResidualPersistenceV7CampaignTests(unittest.TestCase):
    def test_protocol_and_dependency_contract_fail_closed(self) -> None:
        protocol = campaign.load_protocol()
        self.assertEqual(protocol["status"], "opened_train_tune_no_holdout")
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "protocol.json"
            broken = copy.deepcopy(protocol)
            broken["status"] = "sealed_holdout"
            path.write_text(json.dumps(broken), encoding="utf-8")
            with self.assertRaises(ValueError):
                campaign.load_protocol(path)
            broken = copy.deepcopy(protocol)
            broken["dependencies"]["base_oracle_runner_file_sha256"] = "0" * 64
            path.write_text(json.dumps(broken), encoding="utf-8")
            with self.assertRaises(ValueError):
                campaign.load_protocol(path)

    def test_common_random_stream_covers_the_complete_paired_case(self) -> None:
        protocol = campaign.load_protocol()
        base_protocol = campaign.resolve_base_protocol(protocol)
        nominal = next(case for case in protocol["case_matrix"] if case["name"] == "nominal")
        pulse = next(
            case for case in protocol["case_matrix"] if case["name"] == "tas_offset_pulse_1p0s"
        )
        first = campaign.generate_case_stream(nominal, protocol, base_protocol, seed=70001)
        second = campaign.generate_case_stream(pulse, protocol, base_protocol, seed=70001)
        self.assertEqual(first.random_stream_sha256, second.random_stream_sha256)
        self.assertEqual(len(first.observations), len(second.observations))
        injection_index = next(
            index
            for index, observation in enumerate(first.observations)
            if observation.tas_timestamp_us >= first.injection_start_source_timestamp_us
        )
        self.assertEqual(first.observations[:injection_index], second.observations[:injection_index])
        self.assertNotEqual(
            first.observations[injection_index].tas_m_s,
            second.observations[injection_index].tas_m_s,
        )

    def test_delivery_profile_changes_arrival_only_and_is_fingerprinted(self) -> None:
        protocol = campaign.load_protocol()
        base_protocol = campaign.resolve_base_protocol(protocol)
        nominal = next(case for case in protocol["case_matrix"] if case["name"] == "nominal")
        jittered = copy.deepcopy(nominal)
        jittered["delivery_profile"] = "bounded_jitter_2hz"
        aligned = campaign.generate_case_stream(nominal, protocol, base_protocol, seed=70001)
        jitter = campaign.generate_case_stream(jittered, protocol, base_protocol, seed=70001)
        self.assertEqual(aligned.random_stream_sha256, jitter.random_stream_sha256)
        self.assertNotEqual(aligned.arrival_schedule_sha256, jitter.arrival_schedule_sha256)
        self.assertEqual(
            [item.tas_timestamp_us for item in aligned.observations],
            [item.tas_timestamp_us for item in jitter.observations],
        )
        self.assertTrue(
            all(
                item.arrival_timestamp_us >= item.tas_timestamp_us
                for item in jitter.observations
            )
        )
        self.assertTrue(
            all(
                later.arrival_timestamp_us > earlier.arrival_timestamp_us
                for earlier, later in zip(jitter.observations, jitter.observations[1:])
            )
        )
        integrated = campaign.evaluate_case(jittered, protocol, base_protocol, seed=70001)
        self.assertEqual(integrated["unexpected_source_rejections"], {})
        self.assertGreater(int(integrated["nis_fed_observations"]), 0)

    def test_trigger_payload_records_source_onset_attribution(self) -> None:
        protocol = campaign.load_protocol()
        base_protocol = campaign.resolve_base_protocol(protocol)
        case = next(
            case for case in protocol["case_matrix"]
            if case["name"] == "persistent_tas_offset_positive"
        )
        record = campaign.evaluate_case(case, protocol, base_protocol, seed=70001)
        self.assertEqual(record["attribution_status"], "post_injection_attributed")
        trigger = record["monitor"]["trigger_snapshot"]
        self.assertIsNotNone(trigger)
        assert trigger is not None
        self.assertGreaterEqual(
            int(trigger["episode_start_source_timestamp_us"]),
            int(record["injection_start_source_timestamp_us"]),
        )
        self.assertTrue(
            all(
                int(point["source_timestamp_us"]) >= int(record["injection_start_source_timestamp_us"])
                for point in trigger["episode_observations"]
            )
        )

    def test_delivery_matched_null_is_selected_for_jittered_persistent_case(self) -> None:
        protocol = campaign.load_protocol()
        case = next(
            case for case in protocol["case_matrix"]
            if case["name"] == "persistent_tas_offset_positive_bounded_jitter"
        )
        self.assertEqual(
            campaign.paired_null_name(case, protocol["case_matrix"]),
            "nominal_bounded_jitter",
        )

    def test_structural_gap_explicitly_exempts_paired_null_scoring(self) -> None:
        protocol = campaign.load_protocol()
        gap_case = next(
            case
            for case in protocol["case_matrix"]
            if case["name"] == "gap_then_tas_offset_pulse_1p0s"
        )
        self.assertFalse(gap_case["paired_null_required"])
        self.assertNotIn(
            gap_case["name"], campaign.paired_nulls_by_case(protocol["case_matrix"])
        )

    def test_assemble_result_skips_unpaired_structural_gap_attribution(self) -> None:
        """A transport-gap coverage case is not a residual-detection pair."""

        protocol = copy.deepcopy(campaign.load_protocol())
        protocol["seed_ranges"]["train"] = {
            "start": 70001,
            "count": 1,
            "purpose": "focused aggregate-scoring contract check",
        }
        seed = 70001
        minimum_active = float(
            protocol["scenario"]["minimum_pre_injection_eligible_source_time_s"]
        )
        stream_fingerprints = {
            str(case["paired_stream_group"]): f"stream:{case['paired_stream_group']}"
            for case in protocol["case_matrix"]
        }
        records: list[dict[str, object]] = []
        for case in protocol["case_matrix"]:
            is_structural_gap = case["role"] == "structural_gap"
            records.append(
                {
                    "name": case["name"],
                    "role": case["role"],
                    "synthetic_seed": seed,
                    "paired_stream_group": case["paired_stream_group"],
                    "random_stream_sha256": stream_fingerprints[
                        str(case["paired_stream_group"])
                    ],
                    "monitor_latched": False,
                    "pre_injection_latch": False,
                    "source_detection_delay_s": None,
                    "geometry_qualified": True,
                    "generated_observations": 1,
                    "source_valid_observations": 1,
                    "nis_fed_observations": 1,
                    "post_geometry_observations": 1,
                    "pre_injection_eligible_source_time_s": minimum_active,
                    "episode_start_source_timestamp_us": None,
                    "attribution_status": "not_latched",
                    "arrival_decision_latency_s": None,
                    "unexpected_source_rejections": {},
                    "monitor": {
                        "armed": not is_structural_gap,
                        "trigger_snapshot_immutable": True,
                        "source_gap_events": 1 if is_structural_gap else 0,
                        "arrival_gap_events": 0,
                        "pre_injection_active_source_time_s": minimum_active,
                        "trigger_snapshot_sha256": None,
                        "trigger_snapshot": None,
                        "first_latch_source_timestamp_us": None,
                    },
                }
            )

        gap_case = next(
            case
            for case in protocol["case_matrix"]
            if case["name"] == "gap_then_tas_offset_pulse_1p0s"
        )
        with self.assertRaises(ValueError):
            campaign.paired_null_name(gap_case, protocol["case_matrix"])

        result = campaign.assemble_result(
            protocol,
            records,
            phase="train",
            protocol_path=campaign.DEFAULT_PROTOCOL,
            jobs=1,
            include_records=False,
        )

        self.assertEqual(result["phase"], "train")
        summaries = {summary["name"]: summary for summary in result["case_summaries"]}
        gap_summary = summaries["gap_then_tas_offset_pulse_1p0s"]
        self.assertNotIn("paired_attributable_latches", gap_summary)
        self.assertNotIn("paired_null_latched_not_later", gap_summary)

    def test_required_pair_is_rejected_during_protocol_preflight(self) -> None:
        protocol = campaign.load_protocol()
        broken = copy.deepcopy(protocol)
        gap_case = next(
            case
            for case in broken["case_matrix"]
            if case["name"] == "gap_then_tas_offset_pulse_1p0s"
        )
        gap_case["paired_null_required"] = True
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "protocol.json"
            path.write_text(json.dumps(broken), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "lacks exactly one paired null"):
                campaign.load_protocol(path)

    def test_tune_start_receipt_is_exclusive(self) -> None:
        protocol = campaign.load_protocol()
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            claim = root / "claim.json"
            claim.write_text("{}\n", encoding="utf-8")
            started = root / "started.json"
            with (
                mock.patch.object(campaign, "default_claim", return_value=claim),
                mock.patch.object(campaign, "default_tune_started", return_value=started),
            ):
                self.assertEqual(campaign.start_tune_execution(protocol), started)
                self.assertTrue(started.is_file())
                with self.assertRaises(ValueError):
                    campaign.start_tune_execution(protocol)

    def test_train_requires_a_clean_worktree(self) -> None:
        with mock.patch.object(campaign, "capture", return_value=" M dirty-file"):
            with self.assertRaisesRegex(ValueError, "train requires a clean Git worktree"):
                campaign.require_clean_train_worktree()

    def test_exact_one_sided_binomial_bounds_cover_zero_and_all_events(self) -> None:
        confidence = 0.975
        self.assertAlmostEqual(
            campaign.clopper_pearson_upper(0, 128, confidence),
            1.0 - math.pow(1.0 - confidence, 1.0 / 128.0),
            places=12,
        )
        self.assertAlmostEqual(
            campaign.clopper_pearson_lower(128, 128, confidence),
            math.pow(1.0 - confidence, 1.0 / 128.0),
            places=12,
        )

    def test_nominal_and_gap_cases_have_real_monitor_coverage(self) -> None:
        protocol = campaign.load_protocol()
        base_protocol = campaign.resolve_base_protocol(protocol)
        cases = {
            case["name"]: case
            for case in protocol["case_matrix"]
            if case["name"] in {"nominal", "gap_then_tas_offset_pulse_1p0s"}
        }
        nominal = campaign.evaluate_case(cases["nominal"], protocol, base_protocol, seed=70001)
        gap = campaign.evaluate_case(
            cases["gap_then_tas_offset_pulse_1p0s"], protocol, base_protocol, seed=70001
        )
        minimum_active = float(protocol["scenario"]["minimum_pre_injection_eligible_source_time_s"])
        self.assertTrue(nominal["monitor"]["armed"])
        self.assertGreaterEqual(
            float(nominal["pre_injection_eligible_source_time_s"]), minimum_active
        )
        self.assertGreater(int(nominal["nis_fed_observations"]), 0)
        self.assertFalse(nominal["monitor_latched"])
        self.assertGreaterEqual(int(gap["monitor"]["source_gap_events"]), 1)
        self.assertFalse(gap["monitor_latched"])


if __name__ == "__main__":
    unittest.main()
