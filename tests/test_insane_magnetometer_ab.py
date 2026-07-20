from __future__ import annotations

import csv
import hashlib
import json
import math
import os
import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "validation"))
import run_insane_magnetometer_ab as insane_ab  # noqa: E402


HEADER = [
    "seq", "ts_us", "raw_mag_cuT_x", "raw_mag_cuT_y", "raw_mag_cuT_z",
    "mag_valid", "mag_update", "ref_q_w", "ref_q_x", "ref_q_y", "ref_q_z",
    "magnetic_declination_rad",
    "sentinel",
]


def write_replay(path: Path, headings_deg: tuple[float, ...] = (0.0, 10.0, -5.0)) -> None:
    field_norm_ut = 50.0
    inclination_rad = math.radians(30.0)
    horizontal_ut = field_norm_ut * math.cos(inclination_rad)
    down_ut = field_norm_ut * math.sin(inclination_rad)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(HEADER)
        for index, heading_deg in enumerate(headings_deg):
            heading_rad = math.radians(heading_deg)
            writer.writerow([
                index, index * 10000,
                round(100.0 * horizontal_ut * math.cos(heading_rad)),
                round(100.0 * horizontal_ut * math.sin(heading_rad)),
                round(100.0 * down_ut),
                1, 1, 1.0, 0.0, 0.0, 0.0, 0.0, f"keep-{index}",
            ])


class InsaneMagnetometerAbTests(unittest.TestCase):
    def test_paired_input_is_mechanically_identical_except_mag_flags(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "source.csv"
            on_path = root / "on.csv"
            off_path = root / "off.csv"
            write_replay(source)
            audit = insane_ab.create_paired_inputs(source, on_path, off_path)
            expected_hash = hashlib.sha256(source.read_bytes()).hexdigest()
            self.assertEqual(insane_ab.sha256_file(on_path), expected_hash)
            self.assertEqual(audit["data_rows"], 3)
            self.assertEqual(audit["file_lines"], 4)
            self.assertEqual(audit["physical_magnetometer_updates"], 3)
            self.assertTrue(audit["all_other_fields_text_identical"])
            with off_path.open("r", encoding="utf-8", newline="") as stream:
                rows = list(csv.DictReader(stream))
            self.assertTrue(all(row["mag_valid"] == "0" for row in rows))
            self.assertTrue(all(row["mag_update"] == "0" for row in rows))
            self.assertEqual([row["sentinel"] for row in rows], ["keep-0", "keep-1", "keep-2"])

    def test_pair_audit_fails_if_any_unrelated_field_changes(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "source.csv"
            on_path = root / "on.csv"
            off_path = root / "off.csv"
            write_replay(source)
            insane_ab.create_paired_inputs(source, on_path, off_path)
            with off_path.open("r", encoding="utf-8", newline="") as stream:
                rows = list(csv.reader(stream))
            rows[2][-1] = "tampered"
            with off_path.open("w", encoding="utf-8", newline="") as stream:
                csv.writer(stream).writerows(rows)
            with self.assertRaisesRegex(ValueError, "paired invariant failed"):
                insane_ab.audit_paired_inputs(on_path, off_path)

    def test_field_audit_uses_reference_frame_and_reports_drift(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            replay = Path(directory) / "replay.csv"
            write_replay(replay)
            audit = insane_ab.magnetic_field_audit(replay)
        self.assertEqual(audit["physical_magnetometer_updates"], 3)
        drift = audit["horizontal_magnetic_heading_relative_drift_deg"]
        self.assertAlmostEqual(drift["minimum"], -5.0, delta=0.02)
        self.assertAlmostEqual(drift["maximum"], 10.0, delta=0.02)
        self.assertAlmostEqual(drift["final"], -5.0, delta=0.02)
        self.assertGreater(drift["p95_absolute"], 9.0)
        norm = audit["field_norm_ut"]
        inclination = audit["inclination_deg_positive_down"]
        datum = audit["declared_datum_residual_deg"]
        self.assertAlmostEqual(norm["median"], 50.0, delta=0.02)
        self.assertAlmostEqual(inclination["median"], 30.0, delta=0.02)
        self.assertAlmostEqual(datum["minimum"], -5.0, delta=0.02)
        self.assertAlmostEqual(datum["maximum"], 10.0, delta=0.02)

    def test_field_audit_rejects_a_changing_declared_datum(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            replay = Path(directory) / "replay.csv"
            write_replay(replay)
            with replay.open("r", encoding="utf-8", newline="") as stream:
                rows = list(csv.reader(stream))
            rows[2][rows[0].index("magnetic_declination_rad")] = "0.1"
            with replay.open("w", encoding="utf-8", newline="") as stream:
                csv.writer(stream).writerows(rows)
            with self.assertRaisesRegex(ValueError, "datum changes"):
                insane_ab.magnetic_field_audit(replay)

    def test_end_to_end_records_hashes_commands_rows_and_truth_boundary(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            replay = root / "replay.csv"
            runner = root / "fake_runner.py"
            analyzer = root / "fake_analyzer.py"
            out_dir = root / "output"
            write_replay(replay)
            runner.write_text(
                "#!/usr/bin/env python3\n"
                "import shutil, sys\n"
                "assert sys.argv[1] == '--reference-attitude-init'\n"
                "shutil.copyfile(sys.argv[2], sys.argv[3])\n",
                encoding="utf-8",
            )
            os.chmod(runner, 0o755)
            analyzer.write_text(
                "import argparse, csv, json\n"
                "from pathlib import Path\n"
                "p=argparse.ArgumentParser(); p.add_argument('results'); "
                "p.add_argument('--out-dir', type=Path, required=True); "
                "p.add_argument('--scenario'); p.add_argument('--reference-kind'); "
                "p.add_argument('--no-plots', action='store_true'); a=p.parse_args()\n"
                "a.out_dir.mkdir(parents=True, exist_ok=True)\n"
                "with open(a.results, newline='', encoding='utf-8') as f: "
                "n=sum(1 for _ in csv.DictReader(f))\n"
                "(a.out_dir/'metrics.json').write_text(json.dumps({"
                "'samples':n,'reference_kind':a.reference_kind})+'\\n', encoding='utf-8')\n",
                encoding="utf-8",
            )
            output = insane_ab.run_ab(
                replay, runner, out_dir, analyzer_path=analyzer, scenario="unit-insane-ab"
            )
            expected_replay_hash = hashlib.sha256(replay.read_bytes()).hexdigest()
            expected_runner_hash = hashlib.sha256(runner.read_bytes()).hexdigest()
            expected_analyzer_hash = hashlib.sha256(analyzer.read_bytes()).hexdigest()
            summary_text = (out_dir / "summary.json").read_text(encoding="utf-8")
            summary = json.loads(summary_text, parse_constant=lambda value: self.fail(value))
        self.assertEqual(output["status"], "completed")
        self.assertFalse(output["truth_never_enters_initialization"])
        self.assertFalse(output["cold_start_absolute_heading_evidence"])
        self.assertIn("used once to initialize both arms", output["truth_usage"])
        self.assertEqual(output["pair_audit"]["data_rows"], 3)
        self.assertEqual(len(output["provenance"]["commands"]), 4)
        for arm in output["arms"].values():
            self.assertEqual(arm["runner_command"][1], "--reference-attitude-init")
            self.assertEqual(arm["replay_file_lines"], 4)
            self.assertEqual(arm["results_file_lines"], 4)
            self.assertEqual(arm["metrics"]["samples"], 3)
            self.assertEqual(arm["metrics"]["reference_kind"], "independent_truth")
        self.assertEqual(summary["provenance"]["source_replay_sha256"], expected_replay_hash)
        self.assertEqual(summary["provenance"]["native_runner_sha256"], expected_runner_hash)
        self.assertEqual(summary["provenance"]["analyzer_sha256"], expected_analyzer_hash)
        self.assertNotIn("NaN", summary_text)

    def test_fail_closed_on_missing_physical_magnetometer_updates(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            replay = Path(directory) / "replay.csv"
            write_replay(replay)
            with replay.open("r", encoding="utf-8", newline="") as stream:
                rows = list(csv.reader(stream))
            valid_index = rows[0].index("mag_valid")
            for row in rows[1:]:
                row[valid_index] = "0"
            with replay.open("w", encoding="utf-8", newline="") as stream:
                csv.writer(stream).writerows(rows)
            with self.assertRaisesRegex(ValueError, "no physical valid updates"):
                insane_ab.create_paired_inputs(
                    replay, Path(directory) / "on.csv", Path(directory) / "off.csv"
                )


if __name__ == "__main__":
    unittest.main()
