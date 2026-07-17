from __future__ import annotations

import csv
import sys
import tempfile
import unittest
from pathlib import Path


TOOLS = Path(__file__).resolve().parents[1] / "simulation" / "tools"
sys.path.insert(0, str(TOOLS))

import convert_capture_to_golden as converter  # noqa: E402


def record(sequence: int, timestamp_us: int) -> str:
    return (
        f"[IMUCSV] seq={sequence},ts_us={timestamp_us},dt_us=2500,"
        "raw_acc_mg=[0,0,-1000],raw_gyro_mdps=[0,0,0],"
        "raw_mag_cuT=[2200,100,4400],g_est_mg=[0,0,-1000],"
        "rpy_mdeg=[0,0,0]"
    )


class CaptureConverterTests(unittest.TestCase):
    def test_parse_record(self) -> None:
        parsed = converter.parse_record(record(7, 10_000))
        self.assertIsNotNone(parsed)
        assert parsed is not None
        self.assertEqual(parsed["seq"], 7)
        self.assertEqual(parsed["raw_acc_mg_z"], -1000)

    def test_rejects_truncated_record(self) -> None:
        self.assertIsNone(converter.parse_record("[IMUCSV] ts_us=1,dt_us=2500"))

    def test_conversion_deduplicates_and_marks_gap(self) -> None:
        with tempfile.TemporaryDirectory() as temp_directory:
            input_path = Path(temp_directory) / "capture.csv"
            output_path = Path(temp_directory) / "golden.csv"
            with input_path.open("w", encoding="utf-8", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=["ts_sec", "line"])
                writer.writeheader()
                writer.writerow({"ts_sec": "0.01", "line": record(1, 2500)})
                writer.writerow({"ts_sec": "0.02", "line": record(1, 2500)})
                writer.writerow({"ts_sec": "0.03", "line": record(3, 7500)})

            self.assertEqual(converter.convert_capture(input_path, output_path), 2)
            with output_path.open("r", encoding="utf-8", newline="") as stream:
                rows = list(csv.DictReader(stream))

            self.assertEqual(rows[0]["host_ts_us"], "10000")
            self.assertEqual(rows[1]["seq_delta"], "2")
            self.assertEqual(rows[1]["gap_flag"], "1")


if __name__ == "__main__":
    unittest.main()
