from __future__ import annotations

import copy
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "validation"))

import run_bias_observability_cross_validation as cross_validation  # noqa: E402


class BiasObservabilityResumeTests(unittest.TestCase):
    def setUp(self) -> None:
        self.trial: dict[str, object] = {
            "trajectory": {
                "id": "hover_axis_pulses",
                "motion": "bias_cv_hover_axis_pulses",
                "split": "train",
            },
            "vector": {"id": "x_pos"},
            "seed": 7,
        }

    def _valid_record(self) -> dict[str, object]:
        return {
            "trajectory_id": "hover_axis_pulses",
            "motion": "bias_cv_hover_axis_pulses",
            "split": "train",
            "bias_vector_id": "x_pos",
            "seed": 7,
            "input_sha256": "a" * 64,
            "horizontal_bias": {},
            "general": {},
            "commands": {"filter": ["aerakia_validation_runner", "--cold-start"]},
        }

    def _record_path(self, root: Path) -> Path:
        return (
            root / "trials/train/hover_axis_pulses/x_pos/seed-00007"
            / "cross-validation-metrics.json"
        )

    def _write_record(self, root: Path, record: dict[str, object]) -> None:
        path = self._record_path(root)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(record), encoding="utf-8")

    def test_reuses_only_a_complete_exact_match(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            record = self._valid_record()
            self._write_record(root, record)

            self.assertEqual(
                cross_validation.load_resumable_trial(self.trial, out_dir=root),
                record,
            )

    def test_rejects_identity_or_shape_mismatch(self) -> None:
        for field, value in (
            ("trajectory_id", "other"),
            ("motion", "other_motion"),
            ("split", "tune"),
            ("bias_vector_id", "x_neg"),
            ("seed", 8),
            ("horizontal_bias", []),
            ("general", []),
        ):
            with self.subTest(field=field):
                with tempfile.TemporaryDirectory() as temporary:
                    root = Path(temporary)
                    record = self._valid_record()
                    record[field] = value
                    self._write_record(root, record)

                    self.assertIsNone(
                        cross_validation.load_resumable_trial(self.trial, out_dir=root)
                    )

    def test_rejects_invalid_input_hash_or_experimental_mode(self) -> None:
        for mutation in (
            lambda record: record.update({"input_sha256": "not-a-hash"}),
            lambda record: record["commands"].update({
                "filter": [
                    "aerakia_validation_runner",
                    "--experimental-static-tilt-accel-bias-correlation",
                ]
            }),
        ):
            with tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                record = copy.deepcopy(self._valid_record())
                mutation(record)
                self._write_record(root, record)

                self.assertIsNone(cross_validation.load_resumable_trial(self.trial, out_dir=root))


if __name__ == "__main__":
    unittest.main()
