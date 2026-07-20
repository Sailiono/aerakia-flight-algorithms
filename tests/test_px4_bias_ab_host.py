"""Focused tests for the official-PX4 M0 host build entry point."""

from __future__ import annotations

import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "validation" / "build_px4_bias_ab_host.py"
SPEC = importlib.util.spec_from_file_location("build_px4_bias_ab_host", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
HOST = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(HOST)


class Px4BiasAbHostBuildTests(unittest.TestCase):
    def test_standalone_build_selects_official_ekf_core_sources(self) -> None:
        project = (ROOT / "validation" / "px4_bias_ab_host" / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn("${EKF}/ekf.cpp", project)
        self.assertIn("${EKF}/estimator_interface.cpp", project)
        self.assertIn("${EKF}/aid_sources/gnss/gps_control.cpp", project)
        self.assertIn("${EKF}/aid_sources/magnetometer/mag_control.cpp", project)
        self.assertIn("target_link_libraries(aerakia_px4_bias_ab_runner PRIVATE aerakia_px4_ecl_ekf)", project)
        self.assertIn("target_compile_definitions(aerakia_px4_ecl_ekf PUBLIC", project)

    def test_runner_keeps_truth_out_of_the_filter_input_path(self) -> None:
        runner = (ROOT / "validation" / "px4_bias_ab_host_runner.cpp").read_text(encoding="utf-8")
        self.assertIn("never reads any truth columns", runner)
        self.assertNotIn('get_float(fields, columns, "truth_', runner)
        self.assertIn("const StateSample &state = ekf.state();", runner)
        self.assertIn("ekf.time_delayed_us()", runner)
        self.assertIn("ekf2_abias_init", runner)
        self.assertIn("PX4 delayed fusion horizon regressed", runner)

    def test_missing_px4_source_writes_blocked_build_manifest(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            build_dir = Path(temporary) / "build"
            status = HOST.main([
                "--px4-source", str(Path(temporary) / "missing-px4"),
                "--build-dir", str(build_dir),
            ])
            record = json.loads((build_dir / "build-manifest.json").read_text(encoding="utf-8"))
            self.assertEqual(status, 2)
            self.assertEqual(record["status"], "blocked")
            self.assertEqual(record["claim_status"], "no_px4_parity_claim")
            self.assertIn("missing", record["reason"])

    def test_uses_an_out_of_tree_read_only_px4_core_build(self) -> None:
        source = MODULE_PATH.read_text(encoding="utf-8")
        self.assertIn('"px4_bias_ab_host" / "CMakeLists.txt"', source)
        self.assertIn("-DAERAKIA_PX4_SOURCE=", source)
        self.assertIn("aerakia_px4_bias_ab_runner", source)
        self.assertNotIn("git clone", source)
        self.assertNotIn("git fetch", source)

    def test_standalone_build_validates_its_actual_official_source_set(self) -> None:
        source = MODULE_PATH.read_text(encoding="utf-8")
        self.assertIn("validate_standalone_ekf_sources", source)
        self.assertIn("aid_sources/gnss/gps_control.cpp", source)
        self.assertIn("does not configure PX4's root CMake project", source)

    def test_build_records_the_uorb_generator_dependency_version(self) -> None:
        source = MODULE_PATH.read_text(encoding="utf-8")
        self.assertIn("def empy_version", source)
        self.assertIn('"empy_version": empy', source)


if __name__ == "__main__":
    unittest.main()
