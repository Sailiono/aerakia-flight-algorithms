#!/usr/bin/env python3
"""Run and record the complete hardware-independent host regression."""

from __future__ import annotations

import argparse
import json
import os
import platform
import shlex
import subprocess
import sys
import time
from pathlib import Path


def command_text(command: list[str]) -> str:
    return shlex.join(command)


def run_recorded(
    command: list[str],
    *,
    cwd: Path,
    log_path: Path,
    environment: dict[str, str] | None = None,
) -> dict[str, object]:
    started = time.monotonic()
    completed = subprocess.run(
        command,
        cwd=cwd,
        env=environment,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )
    duration_s = time.monotonic() - started
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_path.write_text(completed.stdout, encoding="utf-8")
    print(f"$ {command_text(command)}")
    if completed.stdout:
        print(completed.stdout, end="" if completed.stdout.endswith("\n") else "\n")
    record: dict[str, object] = {
        "command": command,
        "cwd": str(cwd),
        "duration_s": duration_s,
        "log": str(log_path),
        "returncode": completed.returncode,
    }
    if completed.returncode != 0:
        raise subprocess.CalledProcessError(completed.returncode, command, completed.stdout)
    return record


def capture(command: list[str], cwd: Path) -> str | None:
    completed = subprocess.run(
        command,
        cwd=cwd,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    if completed.returncode != 0:
        return None
    return completed.stdout.strip()


def find_runner(build_dir: Path) -> Path:
    candidates = (
        build_dir / "aerakia_validation_runner",
        build_dir / "aerakia_validation_runner.exe",
        build_dir / "Release" / "aerakia_validation_runner",
        build_dir / "Release" / "aerakia_validation_runner.exe",
    )
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    locations = ", ".join(str(candidate) for candidate in candidates)
    raise FileNotFoundError(f"validation runner not found; checked: {locations}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-dir", type=Path, default=Path("build/host-regression"))
    parser.add_argument("--out-dir", type=Path, default=Path("build/host-regression-report"))
    parser.add_argument("--build-type", default="Release")
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[1]
    build_dir = (root / args.build_dir).resolve() if not args.build_dir.is_absolute() else args.build_dir
    out_dir = (root / args.out_dir).resolve() if not args.out_dir.is_absolute() else args.out_dir
    logs_dir = out_dir / "logs"
    validation_dir = out_dir / "validation"
    out_dir.mkdir(parents=True, exist_ok=True)

    environment = os.environ.copy()
    matplotlib_config = out_dir / ".matplotlib"
    matplotlib_config.mkdir(parents=True, exist_ok=True)
    environment["MPLCONFIGDIR"] = str(matplotlib_config)

    commands: list[dict[str, object]] = []
    status = "failed"
    failure: str | None = None
    started_utc = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    try:
        commands.append(run_recorded(
            ["cmake", "-S", str(root), "-B", str(build_dir),
             f"-DCMAKE_BUILD_TYPE={args.build_type}"],
            cwd=root, log_path=logs_dir / "01-cmake-configure.log", environment=environment,
        ))
        commands.append(run_recorded(
            ["cmake", "--build", str(build_dir), "--parallel"],
            cwd=root, log_path=logs_dir / "02-cmake-build.log", environment=environment,
        ))
        commands.append(run_recorded(
            ["ctest", "--test-dir", str(build_dir), "--output-on-failure"],
            cwd=root, log_path=logs_dir / "03-ctest.log", environment=environment,
        ))
        commands.append(run_recorded(
            [sys.executable, "-m", "unittest", "discover", "-s", "tests", "-p", "test_*.py"],
            cwd=root, log_path=logs_dir / "04-python-tests.log", environment=environment,
        ))
        runner = find_runner(build_dir)
        commands.append(run_recorded(
            [sys.executable, str(root / "validation" / "run_suite.py"),
             "--runner", str(runner), "--out-dir", str(validation_dir)],
            cwd=root, log_path=logs_dir / "05-deterministic-suite.log", environment=environment,
        ))
        commands.append(run_recorded(
            [sys.executable, str(root / "validation" / "check_thresholds.py"),
             str(validation_dir / "summary.json"), "--thresholds",
             str(root / "validation" / "thresholds.json")],
            cwd=root, log_path=logs_dir / "06-thresholds.log", environment=environment,
        ))
        status = "passed"
    except (OSError, subprocess.CalledProcessError) as error:
        failure = str(error)
        raise
    finally:
        manifest = {
            "schema_version": 1,
            "status": status,
            "failure": failure,
            "started_utc": started_utc,
            "finished_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "git_commit": capture(["git", "rev-parse", "HEAD"], root),
            "git_status": capture(["git", "status", "--short"], root),
            "host": {
                "platform": platform.platform(),
                "machine": platform.machine(),
                "python": sys.version,
                "cmake": capture(["cmake", "--version"], root),
                "compiler": capture(["cc", "--version"], root),
            },
            "paths": {
                "build_dir": str(build_dir),
                "output_dir": str(out_dir),
            },
            "commands": commands,
        }
        (out_dir / "run-manifest.json").write_text(
            json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )

    print(f"Host regression passed: {out_dir}")


if __name__ == "__main__":
    main()
