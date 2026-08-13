"""Unit tests for the Cortex-M7 preflight report helpers.

These tests deliberately do not require an installed ARM toolchain. The full
cross-compilation evidence is produced by the validation script itself.
"""

from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "run_cortex_m7_cross_compile",
    ROOT / "validation" / "run_cortex_m7_cross_compile.py",
)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class CortexM7CrossCompileTests(unittest.TestCase):
    def test_all_portable_library_sources_are_cross_compiled(self) -> None:
        self.assertIn("src/static_imu_calibration.c", MODULE.SOURCES)

    def test_evidence_manifest_matches_reported_source_commit(self) -> None:
        report = MODULE.load_evidence_report(
            ROOT / "validation" / "public" / "cortex_m7_cross_compile_v1.json"
        )
        self.assertEqual(report["source_manifest"], MODULE.build_source_manifest())
        self.assertEqual(
            report["source_manifest_sha256"],
            MODULE.source_manifest_sha256(report["source_manifest"]),
        )
        self.assertEqual(
            report["source_manifest"],
            MODULE.source_manifest_for_commit(str(report["git_commit"])),
        )

    def test_section_parser_aggregates_subsections_and_ignores_metadata(self) -> None:
        sections = MODULE.parse_sections(
            """\
.text 0 0
.text.eskf_predict 120 0
.rodata.constants 16 0
.data.state 4 0
.bss.scratch 8 0
.comment 31 0
.ARM.attributes 50 0
Total 229 0
"""
        )
        self.assertEqual(
            sections,
            {"text": 120, "rodata": 16, "data": 4, "bss": 8, "other": 0, "reported_total": 148},
        )

    def test_layout_probe_requires_all_public_structures(self) -> None:
        parsed = MODULE.parse_probe_sizes(
            """\
00000000 2912 R aerakia_eskf_size_probe
00000000 2136 R eskf_handle_size_probe
00000000 504 R aerakia_navigation_estimate_size_probe
"""
        )
        self.assertEqual(
            parsed,
            {
                "aerakia_eskf_bytes": 2912,
                "eskf_handle_bytes": 2136,
                "navigation_estimate_bytes": 504,
            },
        )
        with self.assertRaisesRegex(RuntimeError, "navigation_estimate_bytes"):
            MODULE.parse_probe_sizes("00000000 2912 R aerakia_eskf_size_probe\n")

    def test_external_symbol_categories_preserve_double_math_visibility(self) -> None:
        categories = MODULE.categorize_symbols(
            ["memcpy", "sqrtf", "atan2", "__aeabi_ul2d", "board_hook"]
        )
        self.assertEqual(categories["compiler_runtime"], ["__aeabi_ul2d"])
        self.assertEqual(categories["libc"], ["memcpy"])
        self.assertEqual(categories["libm_double"], ["atan2"])
        self.assertEqual(categories["libm_float"], ["sqrtf"])
        self.assertEqual(categories["other"], ["board_hook"])

    def test_work_directory_must_be_a_dedicated_build_subdirectory(self) -> None:
        valid = MODULE.ensure_safe_work_root(ROOT / "build" / "cross-compile-unit-test")
        self.assertEqual(valid, (ROOT / "build" / "cross-compile-unit-test").resolve())
        with self.assertRaisesRegex(RuntimeError, "dedicated directory"):
            MODULE.ensure_safe_work_root(ROOT / "build")
        with self.assertRaisesRegex(RuntimeError, "must be below"):
            MODULE.ensure_safe_work_root(ROOT / "validation")


if __name__ == "__main__":
    unittest.main()
