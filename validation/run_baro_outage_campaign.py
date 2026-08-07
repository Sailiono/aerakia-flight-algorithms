#!/usr/bin/env python3
"""Run a complete, resumable four-arm barometer campaign with bounded parallelism."""

from __future__ import annotations

import argparse
import concurrent.futures
import hashlib
import json
import os
import subprocess
import sys
import time
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
SINGLE_SHARD = ROOT / "validation" / "run_baro_outage_ab.py"
MERGER = ROOT / "validation" / "merge_baro_outage_campaign.py"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def split_csv(value: str) -> list[str]:
    return [item.strip() for item in value.split(",") if item.strip()]


def parse_outage_groups(value: str, requested: list[float]) -> list[list[float]]:
    groups = [[float(item) for item in split_csv(group)] for group in value.split("/") if group.strip()]
    if not groups or any(not group or any(item <= 0.0 for item in group) for group in groups):
        raise ValueError("each outage shard group must contain positive durations")
    flattened = [item for group in groups for item in group]
    if len(flattened) != len(set(flattened)) or set(flattened) != set(requested):
        raise ValueError("outage shard groups must partition the requested --outages exactly once")
    return groups


def completed_shard(path: Path, *, fault: str, outages: list[float], seed: int) -> bool:
    summary_path = path / "summary.json"
    try:
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        trials = summary["trials"]
        actual = {
            (str(trial["fault"]), float(trial["outage_duration_s"]), int(trial["seed"]))
            for trial in trials
        }
        expected = {(fault, outage, seed) for outage in outages}
        return summary["status"] == "completed" and actual == expected and len(trials) == len(expected)
    except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError):
        return False


def run_task(command: list[str], log_path: Path) -> tuple[list[str], int, str]:
    completed = subprocess.run(
        command,
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )
    log_path.write_text(completed.stdout, encoding="utf-8")
    return command, completed.returncode, completed.stdout


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runner", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, default=Path("build/baro-outage-release"))
    parser.add_argument("--outages", default="5,10,30,60,120")
    parser.add_argument("--faults", default="nominal,constant_bias,random_walk,weather_step,freeze,delay")
    parser.add_argument("--seeds", type=int, default=20)
    parser.add_argument("--rate", type=float, default=100.0)
    parser.add_argument(
        "--shard-outage-groups", default="5,10,30/60/120",
        help="slash-separated outage groups; must partition --outages exactly once",
    )
    parser.add_argument("--jobs", type=int, default=min(8, os.cpu_count() or 1))
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args()
    if args.seeds <= 0 or args.rate <= 0.0 or args.jobs <= 0:
        parser.error("--seeds, --rate, and --jobs must be positive")
    runner = args.runner.resolve()
    if not runner.is_file():
        parser.error(f"--runner does not exist: {runner}")
    outages = [float(item) for item in split_csv(args.outages)]
    faults = split_csv(args.faults)
    if not outages or any(item <= 0.0 for item in outages):
        parser.error("--outages must contain positive values")
    try:
        outage_groups = parse_outage_groups(args.shard_outage_groups, outages)
    except ValueError as error:
        parser.error(str(error))
    allowed_faults = {"nominal", "constant_bias", "random_walk", "weather_step", "freeze", "delay"}
    if not faults or any(item not in allowed_faults for item in faults):
        parser.error(f"--faults must be selected from: {', '.join(sorted(allowed_faults))}")

    out_dir = args.out_dir.resolve()
    shard_root = out_dir / "shards"
    scheduled: list[tuple[list[str], Path]] = []
    skipped = 0
    for fault in faults:
        for seed in range(args.seeds):
            for group in outage_groups:
                group_name = "-".join(f"{outage:g}" for outage in group)
                shard_dir = shard_root / fault / f"outages-{group_name}s" / f"seed-{seed:04d}"
                if args.resume and completed_shard(shard_dir, fault=fault, outages=group, seed=seed):
                    skipped += 1
                    continue
                shard_dir.mkdir(parents=True, exist_ok=True)
                command = [
                    sys.executable, str(SINGLE_SHARD),
                    "--runner", str(runner),
                    "--out-dir", str(shard_dir),
                    "--outages", ",".join(f"{outage:g}" for outage in group),
                    "--faults", fault,
                    "--seeds", "1",
                    "--seed-start", str(seed),
                    "--rate", f"{args.rate:g}",
                    "--compact",
                ]
                scheduled.append((command, shard_dir / "campaign-shard.log"))

    started = time.monotonic()
    failures: list[dict[str, Any]] = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as pool:
        pending = {pool.submit(run_task, command, log): (command, log) for command, log in scheduled}
        for future in concurrent.futures.as_completed(pending):
            command, log = pending[future]
            try:
                _, returncode, output = future.result()
            except Exception as error:  # pragma: no cover - defensive process boundary
                failures.append({"command": command, "log": str(log), "error": str(error)})
                continue
            if returncode != 0:
                failures.append({"command": command, "log": str(log), "returncode": returncode, "output": output[-2000:]})
    if failures:
        (out_dir / "campaign-failures.json").write_text(
            json.dumps(failures, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        raise RuntimeError(f"{len(failures)} barometer shards failed; inspect {out_dir / 'campaign-failures.json'}")

    merge_command = [
        sys.executable, str(MERGER),
        "--input-dir", str(shard_root),
        "--out-dir", str(out_dir),
        "--expected-seeds", str(args.seeds),
    ]
    merged = subprocess.run(
        merge_command,
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )
    (out_dir / "merge.log").write_text(merged.stdout, encoding="utf-8")
    if merged.returncode != 0:
        raise RuntimeError(f"merge failed: {merged.stdout[-2000:]}")
    manifest = {
        "schema_version": 1,
        "status": "completed",
        "scope": "parallel scheduling only; each shard replays the native C99 runner independently",
        "scheduled_shards": len(scheduled),
        "resumed_shards": skipped,
        "completed_trials": len(faults) * len(outages) * args.seeds,
        "faults": faults,
        "outages_s": outages,
        "seeds": args.seeds,
        "rate_hz": args.rate,
        "jobs": args.jobs,
        "runtime_s": time.monotonic() - started,
        "provenance": {
            "runner_sha256": sha256(runner),
            "single_shard_script_sha256": sha256(SINGLE_SHARD),
            "merger_sha256": sha256(MERGER),
        },
    }
    (out_dir / "campaign-manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(f"Barometer campaign: {out_dir / 'report.md'}")


if __name__ == "__main__":
    main()
