from __future__ import annotations

import json
import os
import subprocess
import tempfile
import unittest
from pathlib import Path


class StaticImuCalibrationCliTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        configured = os.environ.get("AERAKIA_STATIC_IMU_CALIBRATION_CLI")
        cls.executable = Path(configured) if configured else (
            Path(__file__).resolve().parents[1] / "build" / "aerakia_static_imu_calibration_cli"
        )
        if not cls.executable.is_file():
            raise unittest.SkipTest("host calibration CLI has not been built")

    def test_valid_six_pose_csv_writes_accepted_result(self) -> None:
        rows = """acc_x_m_s2,acc_y_m_s2,acc_z_m_s2,gyro_x_rad_s,gyro_y_rad_s,gyro_z_rad_s,sample_count
9.92665,0,0,0.001,0.002,0.003,400
-9.68665,0,0,0.001,0.002,0.003,400
0.12,9.80665,0,0.001,0.002,0.003,400
0.12,-9.80665,0,0.001,0.002,0.003,400
0.12,0,9.80665,0.001,0.002,0.003,400
0.12,0,-9.80665,0.001,0.002,0.003,400
"""
        with tempfile.TemporaryDirectory() as temporary:
            source = Path(temporary) / "poses.csv"
            output = Path(temporary) / "result.json"
            source.write_text(rows, encoding="utf-8")
            completed = subprocess.run(
                [str(self.executable), str(source), str(output)],
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(completed.returncode, 0, completed.stderr)
            result = json.loads(output.read_text(encoding="utf-8"))
            self.assertTrue(result["accepted"])
            self.assertAlmostEqual(result["accelerometer_bias_m_s2"][0], 0.12, places=4)

    def test_malformed_or_excessive_input_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            source = Path(temporary) / "bad.csv"
            output = Path(temporary) / "result.json"
            source.write_text("header\nnot,a,pose\n", encoding="utf-8")
            completed = subprocess.run(
                [str(self.executable), str(source), str(output)],
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(completed.returncode, 2)

    def test_header_and_sample_count_contract_is_strict(self) -> None:
        valid_rows = """acc_x_m_s2,acc_y_m_s2,acc_z_m_s2,gyro_x_rad_s,gyro_y_rad_s,gyro_z_rad_s,sample_count
9.92665,0,0,0.001,0.002,0.003,{count}
-9.68665,0,0,0.001,0.002,0.003,400
0.12,9.80665,0,0.001,0.002,0.003,400
0.12,-9.80665,0,0.001,0.002,0.003,400
0.12,0,9.80665,0.001,0.002,0.003,400
0.12,0,-9.80665,0.001,0.002,0.003,400
"""
        cases = {
            "wrong-header": valid_rows.format(count="400").replace("acc_x_m_s2", "wrong", 1),
            "fractional-count": valid_rows.format(count="1.5"),
            "out-of-range-count": valid_rows.format(count="4294967296"),
        }
        with tempfile.TemporaryDirectory() as temporary:
            for name, content in cases.items():
                source = Path(temporary) / f"{name}.csv"
                output = Path(temporary) / f"{name}.json"
                source.write_text(content, encoding="utf-8")
                completed = subprocess.run(
                    [str(self.executable), str(source), str(output)],
                    capture_output=True,
                    text=True,
                    check=False,
                )
                self.assertEqual(completed.returncode, 2, f"{name}: {completed.stderr}")


if __name__ == "__main__":
    unittest.main()
