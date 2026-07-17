#!/usr/bin/env python3
"""Validate, convert, replay, score, and baseline-check public datasets."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import shlex
import subprocess
import sys
import time
from pathlib import Path
from typing import Any


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def nested_value(document: dict[str, Any], dotted_path: str) -> Any:
    value: Any = document
    for component in dotted_path.split("."):
        if not isinstance(value, dict) or component not in value:
            raise KeyError(f"missing metric path: {dotted_path}")
        value = value[component]
    return value


def compare_value(
    actual: Any,
    expected: Any,
    *,
    absolute_tolerance: float,
    relative_tolerance: float,
) -> tuple[bool, float | None]:
    if isinstance(expected, bool) or isinstance(actual, bool):
        return actual is expected, None
    if isinstance(expected, int) and isinstance(actual, int):
        return actual == expected, float(abs(actual - expected))
    if isinstance(expected, (int, float)) and isinstance(actual, (int, float)):
        delta = abs(float(actual) - float(expected))
        limit = max(absolute_tolerance, relative_tolerance * abs(float(expected)))
        return delta <= limit, delta
    return actual == expected, None


def run_recorded(
    command: list[str], *, cwd: Path, log_path: Path, environment: dict[str, str]
) -> dict[str, Any]:
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
    print(f"$ {shlex.join(command)}")
    if completed.stdout:
        print(completed.stdout, end="" if completed.stdout.endswith("\n") else "\n")
    record = {
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
        command, cwd=cwd, text=True, stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL, check=False,
    )
    return completed.stdout.strip() if completed.returncode == 0 else None


def write_report(path: Path, manifest: dict[str, Any]) -> None:
    lines = [
        "# Public dataset regression", "",
        f"- Status: **{manifest['status']}**",
        f"- Dataset: {manifest['dataset']}",
        f"- Git commit: `{manifest.get('git_commit')}`",
        f"- Unique recorded IMU samples: {manifest.get('unique_input_samples', 0):,}",
        f"- Total replayed IMU samples: {manifest.get('total_replayed_samples', 0):,}",
        f"- Tracks: {len(manifest.get('tracks', []))}", "",
        "| Sequence | Track | Samples | Duration | Metrics | Result |",
        "| --- | --- | ---: | ---: | ---: | --- |",
    ]
    for track in manifest.get("tracks", []):
        comparisons = track["comparisons"]
        passed = sum(1 for item in comparisons if item["passed"])
        lines.append(
            f"| `{track['sequence']}` | `{track['track']}` | {track['samples']:,} "
            f"| {track['duration_s']:.3f} s | {passed}/{len(comparisons)} "
            f"| {'PASS' if track['all_metrics_passed'] else 'FAIL'} |"
        )
    lines.extend([
        "", "## Interpretation", "",
        "- Repeated tracks use the same immutable recorded samples with different declared "
        "initialization or reference-bias handling; they increase algorithm-path coverage, not "
        "the amount of unique physical data.",
        "- Synthetic GNSS is generated deterministically from external reference position and "
        "velocity. Navigation metrics validate fusion and covariance behavior, not a recorded "
        "GNSS receiver.",
        "- EuRoC has no magnetometer or direct trusted heading. It cannot close physical yaw "
        "observability or magnetic-disturbance claims.",
    ])
    if manifest.get("failure"):
        lines.extend(["", "## Failure", "", f"```text\n{manifest['failure']}\n```"])
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument(
        "--manifest", type=Path, default=Path("validation/public/euroc_manifest.json")
    )
    parser.add_argument("--runner", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, default=Path("build/public-dataset-suite"))
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[1]
    manifest_path = args.manifest if args.manifest.is_absolute() else root / args.manifest
    data_root = args.data_root.resolve()
    runner = args.runner.resolve()
    out_dir = args.out_dir if args.out_dir.is_absolute() else root / args.out_dir
    out_dir = out_dir.resolve()
    converter = root / "simulation" / "tools" / "convert_euroc_to_replay.py"
    analyzer = root / "validation" / "analyze_results.py"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    absolute_tolerance = float(manifest["default_absolute_tolerance"])
    relative_tolerance = float(manifest["default_relative_tolerance"])
    environment = os.environ.copy()
    environment["MPLCONFIGDIR"] = str(out_dir / ".matplotlib")
    Path(environment["MPLCONFIGDIR"]).mkdir(parents=True, exist_ok=True)
    out_dir.mkdir(parents=True, exist_ok=True)

    run_manifest: dict[str, Any] = {
        "schema_version": 1,
        "status": "failed",
        "dataset": manifest["dataset"],
        "source_doi": manifest["source_doi"],
        "source_license": manifest["source_license"],
        "started_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "git_commit": capture(["git", "rev-parse", "HEAD"], root),
        "git_status": capture(["git", "status", "--short"], root),
        "host": {
            "platform": platform.platform(),
            "machine": platform.machine(),
            "python": sys.version,
            "compiler": capture(["cc", "--version"], root),
        },
        "manifest": str(manifest_path),
        "manifest_sha256": sha256(manifest_path),
        "data_root": str(data_root),
        "runner": str(runner),
        "input_checks": [],
        "commands": [],
        "tracks": [],
    }
    total_samples = 0
    unique_samples = 0
    failures: list[str] = []
    try:
        if not runner.is_file():
            raise FileNotFoundError(f"native validation runner not found: {runner}")
        for sequence in manifest["sequences"]:
            sequence_dir = data_root / sequence["relative_path"]
            baseline_path = root / sequence["baseline"]
            baseline = json.loads(baseline_path.read_text(encoding="utf-8"))
            unique_samples += int(baseline["samples"])
            for relative_path, expected_hash in sequence["required_sha256"].items():
                input_path = sequence_dir / relative_path
                if not input_path.is_file():
                    raise FileNotFoundError(f"required dataset input missing: {input_path}")
                actual_hash = sha256(input_path)
                check = {
                    "sequence": sequence["id"],
                    "path": str(input_path),
                    "expected_sha256": expected_hash,
                    "actual_sha256": actual_hash,
                    "passed": actual_hash == expected_hash,
                    "bytes": input_path.stat().st_size,
                }
                run_manifest["input_checks"].append(check)
                if not check["passed"]:
                    raise ValueError(f"SHA-256 mismatch: {input_path}")

            for track in sequence["tracks"]:
                track_name = f"{sequence['id']}__{track['id']}"
                track_dir = out_dir / track_name
                track_dir.mkdir(parents=True, exist_ok=True)
                replay_path = track_dir / "replay.csv"
                source_path = track_dir / "source.json"
                results_path = track_dir / "results.csv"
                report_dir = track_dir / "report"
                commands = [
                    [sys.executable, str(converter), str(sequence_dir), "--out", str(replay_path),
                     "--metadata", str(source_path), *track["converter_args"]],
                    [str(runner), *track["runner_args"], str(replay_path), str(results_path)],
                    [sys.executable, str(analyzer), str(results_path), "--out-dir", str(report_dir),
                     "--scenario", track_name, "--reference-kind", "independent_truth"],
                ]
                for index, command in enumerate(commands, start=1):
                    run_manifest["commands"].append(run_recorded(
                        command, cwd=root,
                        log_path=track_dir / "logs" / f"{index:02d}.log",
                        environment=environment,
                    ))

                source = json.loads(source_path.read_text(encoding="utf-8"))
                metrics = json.loads((report_dir / "metrics.json").read_text(encoding="utf-8"))
                expected_metrics = baseline["tracks"][track["id"]]
                comparisons: list[dict[str, Any]] = []
                for baseline_key, metric_path in track["metric_paths"].items():
                    actual = nested_value(metrics, metric_path)
                    expected = expected_metrics[baseline_key]
                    passed, delta = compare_value(
                        actual, expected,
                        absolute_tolerance=absolute_tolerance,
                        relative_tolerance=relative_tolerance,
                    )
                    comparison = {
                        "metric": baseline_key,
                        "metric_path": metric_path,
                        "actual": actual,
                        "expected": expected,
                        "delta": delta,
                        "passed": passed,
                    }
                    comparisons.append(comparison)
                    if not passed:
                        failures.append(
                            f"{track_name}/{baseline_key}: actual={actual!r} expected={expected!r}"
                        )
                total_samples += int(source["samples"])
                run_manifest["tracks"].append({
                    "sequence": sequence["id"],
                    "track": track["id"],
                    "samples": source["samples"],
                    "duration_s": source["duration_s"],
                    "input_sha256": source["input_sha256"],
                    "comparisons": comparisons,
                    "all_metrics_passed": all(item["passed"] for item in comparisons),
                    "metrics_path": str(report_dir / "metrics.json"),
                })
        if failures:
            raise RuntimeError("public dataset baseline mismatch:\n" + "\n".join(failures))
        run_manifest["status"] = "passed"
    except Exception as error:
        run_manifest["failure"] = str(error)
        raise
    finally:
        run_manifest["finished_utc"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        run_manifest["total_replayed_samples"] = total_samples
        run_manifest["unique_input_samples"] = unique_samples
        run_manifest["comparison_failures"] = failures
        (out_dir / "run-manifest.json").write_text(
            json.dumps(run_manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        write_report(out_dir / "report.md", run_manifest)

    print(
        f"Public dataset suite passed: {len(run_manifest['tracks'])} tracks, "
        f"{total_samples} replayed IMU samples -> {out_dir}"
    )


if __name__ == "__main__":
    main()
