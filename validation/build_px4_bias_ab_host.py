#!/usr/bin/env python3
"""Build the official pinned PX4 ecl_EKF M0 replay executable without modifying PX4."""

from __future__ import annotations

import argparse
import importlib.util
import json
import shutil
import subprocess
import sys
import time
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
M0_PATH = ROOT / "validation" / "run_px4_bias_ab_m0.py"
M0_SPEC = importlib.util.spec_from_file_location("run_px4_bias_ab_m0", M0_PATH)
assert M0_SPEC is not None and M0_SPEC.loader is not None
m0 = importlib.util.module_from_spec(M0_SPEC)
M0_SPEC.loader.exec_module(m0)


class BlockedError(RuntimeError):
    """A required host build dependency is absent."""


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=Path("validation/px4_bias_ab_m0_manifest.json"))
    parser.add_argument("--px4-source", type=Path, required=True)
    parser.add_argument("--build-dir", type=Path, default=Path("build/px4-bias-ab-host"))
    parser.add_argument("--generator", default="Ninja")
    parser.add_argument(
        "--python", type=Path,
        help="Python interpreter with PX4's kconfiglib dependency; defaults to the current interpreter",
    )
    return parser.parse_args(argv)


def write_record(directory: Path, record: dict[str, Any]) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    (directory / "build-manifest.json").write_text(
        json.dumps(record, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def executable_path(build_dir: Path) -> Path:
    suffix = ".exe" if sys.platform == "win32" else ""
    return build_dir / f"aerakia_px4_bias_ab_runner{suffix}"


def run(command: list[str], cwd: Path) -> None:
    try:
        result = subprocess.run(command, cwd=cwd, check=False, text=True, capture_output=True, timeout=600)
    except (OSError, subprocess.TimeoutExpired) as error:
        raise BlockedError(f"cannot execute {' '.join(command)}: {error}") from error
    if result.returncode != 0:
        detail = (result.stdout + "\n" + result.stderr).strip()
        raise RuntimeError(f"command failed ({result.returncode}): {' '.join(command)}\n{detail}")


def empy_version(python: Path) -> str:
    """Return the exact official uORB template-engine version or block early."""
    try:
        result = subprocess.run(
            [str(python), "-c", "import em; print(em.__version__)"],
            check=False, text=True, capture_output=True, timeout=30,
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        raise BlockedError(f"cannot inspect empy with {python}: {error}") from error
    version = result.stdout.strip()
    if result.returncode != 0 or not version:
        detail = result.stderr.strip()
        raise BlockedError(
            f"PX4 uORB generator requires the Python em module via {python}"
            + (f": {detail}" if detail else "")
        )
    return version


def validate_standalone_ekf_sources(source: Path) -> None:
    """Check only source files compiled by the standalone official ecl_EKF build.

    The M0 project deliberately does not configure PX4's root CMake project:
    that target fetches unrelated board, simulation, DDS, and MAVLink
    submodules before exposing the EKF.  Requiring every such submodule would
    turn an irrelevant dependency into a false algorithm blocker.  The frozen
    commit, audited EKF-file hashes, clean Git tree, and these exact source
    paths remain mandatory; no alternate PX4 revision or estimator source is
    accepted.
    """
    required = (
        "src/modules/ekf2/EKF/ekf.cpp",
        "src/modules/ekf2/EKF/estimator_interface.cpp",
        "src/modules/ekf2/EKF/aid_sources/gnss/gps_control.cpp",
        "src/modules/ekf2/EKF/aid_sources/barometer/baro_height_control.cpp",
        "src/modules/ekf2/EKF/aid_sources/magnetometer/mag_control.cpp",
        "src/modules/ekf2/EKF/yaw_estimator/EKFGSF_yaw.cpp",
        "src/lib/geo/geo.cpp",
        "src/lib/lat_lon_alt/lat_lon_alt.cpp",
        "src/lib/world_magnetic_model/geo_mag_declination.cpp",
        "src/lib/matrix/matrix/Vector3.hpp",
    )
    missing = [relative for relative in required if not (source / relative).is_file()]
    if missing:
        raise BlockedError(
            "frozen PX4 standalone EKF build is incomplete: missing "
            + ", ".join(missing)
        )


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    manifest_path = args.manifest if args.manifest.is_absolute() else ROOT / args.manifest
    build_dir = args.build_dir if args.build_dir.is_absolute() else ROOT / args.build_dir
    record: dict[str, Any] = {
        "schema_version": 1,
        "status": "failed",
        "started_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "claim_status": "no_px4_parity_claim",
    }
    try:
        manifest = m0.load_json(manifest_path)
        schema_path = ROOT / manifest["schema_path"]
        m0.validate_manifest(manifest, m0.load_json(schema_path))
        if shutil.which("cmake") is None:
            raise BlockedError("cmake is required to build the official PX4 host runner")
        if args.generator == "Ninja" and shutil.which("ninja") is None:
            raise BlockedError("ninja is required by the selected PX4 host build generator")
        px4_source = m0.validate_px4_source(args.px4_source, manifest)
        validate_standalone_ekf_sources(args.px4_source)
        # Do not resolve this path: a virtualenv's bin/python is commonly a
        # symlink to the system interpreter, and resolving it loses the venv
        # site-packages that contain PX4's kconfiglib dependency.
        python = Path(args.python or Path(sys.executable)).absolute()
        if not python.is_file():
            raise BlockedError(f"selected PX4 Python interpreter is missing: {python}")
        empy = empy_version(python)
        cmake_project = ROOT / "validation" / "px4_bias_ab_host" / "CMakeLists.txt"
        runner = ROOT / "validation" / "px4_bias_ab_host_runner.cpp"
        if not cmake_project.is_file() or not runner.is_file():
            raise RuntimeError("PX4 M0 host runner files are missing from the Aerakia checkout")
        record.update({
            "px4_source": px4_source,
            "manifest_sha256": m0.sha256_file(manifest_path),
            "cmake_project_sha256": m0.sha256_file(cmake_project),
            "runner_sha256": m0.sha256_file(runner),
            "build_dir": str(build_dir.resolve()),
            "python": str(python),
            "empy_version": empy,
        })
        configure = [
            "cmake", "-S", str(cmake_project.parent), "-B", str(build_dir),
            "-G", args.generator,
            f"-DAERAKIA_PX4_SOURCE={args.px4_source.resolve()}",
            f"-DPython3_EXECUTABLE={python}",
            f"-DPYTHON_EXECUTABLE={python}",
            f"-DAERAKIA_PX4_BIAS_AB_RUNNER_SOURCE={runner.resolve()}",
        ]
        run(configure, ROOT)
        run(["cmake", "--build", str(build_dir), "--target", "aerakia_px4_bias_ab_runner"], ROOT)
        binary = executable_path(build_dir)
        if not binary.is_file():
            raise RuntimeError(f"PX4 build succeeded without expected runner {binary}")
        record.update({
            "status": "completed",
            "runner": str(binary.resolve()),
            "runner_sha256": m0.sha256_file(binary),
            "next_step": "Run this binary on an immutable M0 event CSV, then use run_px4_bias_ab_m0.py to score both canonical exports.",
        })
        return_code = 0
    except (BlockedError, m0.BlockedError) as error:
        record.update({"status": "blocked", "reason": str(error)})
        return_code = 2
    except (RuntimeError, m0.ProtocolError, OSError, KeyError, TypeError, ValueError) as error:
        record.update({"status": "failed", "reason": str(error)})
        return_code = 1
    finally:
        record["finished_utc"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        write_record(build_dir, record)
    print(json.dumps({"status": record["status"], "reason": record.get("reason")}, sort_keys=True))
    return return_code


if __name__ == "__main__":
    sys.exit(main())
