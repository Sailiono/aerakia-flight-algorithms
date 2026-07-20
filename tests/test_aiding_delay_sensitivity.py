#!/usr/bin/env python3
"""Unit tests for the timestamp-preserving GNSS delay schedule."""

from __future__ import annotations

import csv
import importlib.util
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "run_aiding_delay_sensitivity",
    ROOT / "validation" / "run_aiding_delay_sensitivity.py",
)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class AidingDelaySensitivityTest(unittest.TestCase):
    def write_source(self, path: Path) -> None:
        fields = [
            "ts_us", "position_update",
            "gps_position_n_m", "gps_position_e_m", "gps_position_d_m",
            "gps_velocity_n_m_s", "gps_velocity_e_m_s", "gps_velocity_d_m_s",
            "gps_position_variance_m2", "gps_velocity_variance_m2_s2",
        ]
        with path.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            for timestamp_us in range(0, 601_000, 100_000):
                writer.writerow({
                    "ts_us": timestamp_us,
                    "position_update": int(timestamp_us in (0, 200_000, 400_000)),
                    "gps_position_n_m": timestamp_us / 1_000_000.0,
                    "gps_position_e_m": 0.0,
                    "gps_position_d_m": 0.0,
                    "gps_velocity_n_m_s": 1.0,
                    "gps_velocity_e_m_s": 0.0,
                    "gps_velocity_d_m_s": 0.0,
                    "gps_position_variance_m2": 1.0,
                    "gps_velocity_variance_m2_s2": 1.0,
                })

    def test_moves_update_and_preserves_physical_timestamp(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            source = directory / "source.csv"
            destination = directory / "delayed.csv"
            self.write_source(source)
            result = MODULE.schedule_delayed_gps(source, destination, 100)
            self.assertEqual(result["source_updates"], 3)
            self.assertEqual(result["scheduled_updates"], 3)
            with destination.open("r", encoding="utf-8", newline="") as stream:
                rows = list(csv.DictReader(stream))
            deliveries = [row for row in rows if row["position_update"] == "1"]
            self.assertEqual([row["ts_us"] for row in deliveries], ["100000", "300000", "500000"])
            self.assertEqual(
                [row["gps_timestamp_us"] for row in deliveries], ["0", "200000", "400000"]
            )

    def test_drops_only_delivery_beyond_available_imu_rows(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            source = directory / "source.csv"
            destination = directory / "delayed.csv"
            self.write_source(source)
            result = MODULE.schedule_delayed_gps(source, destination, 300)
            self.assertEqual(result["scheduled_updates"], 2)
            self.assertEqual(result["dropped_at_end"], 1)


if __name__ == "__main__":
    unittest.main()
