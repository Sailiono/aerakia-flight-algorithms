#!/usr/bin/env python3
"""Convert a text telemetry capture into the normalized golden IMU CSV.

The input is a CSV with a required ``line`` column and either ``ts_sec`` or
``host_ts_us`` for the optional recorder timestamp. A line may contain one or
more records with this hardware-neutral shape::

    [IMUCSV] ts_us=...,dt_us=...,seq=...,raw_acc_mg=[x,y,z],...

The capture producer and transport are deliberately outside this repository.
"""

from __future__ import annotations

import argparse
import csv
import re
import statistics
from pathlib import Path
from typing import Iterable


RECORD_PREFIX = "[IMUCSV]"
INTEGER_TRIPLE = re.compile(r"^\[\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)\s*\]$")

VECTOR_COLUMNS = {
    "raw_acc_mg": ("raw_acc_mg_x", "raw_acc_mg_y", "raw_acc_mg_z"),
    "raw_gyro_mdps": ("raw_gyro_mdps_x", "raw_gyro_mdps_y", "raw_gyro_mdps_z"),
    "raw_mag_cuT": ("raw_mag_cuT_x", "raw_mag_cuT_y", "raw_mag_cuT_z"),
    "g_est_mg": ("g_est_mg_x", "g_est_mg_y", "g_est_mg_z"),
    "e_acc_milli": ("e_acc_milli_x", "e_acc_milli_y", "e_acc_milli_z"),
    "rpy_mdeg": ("roll_mdeg", "pitch_mdeg", "yaw_mdeg"),
}

SCALAR_COLUMNS = (
    "innov_acc_milli",
    "innov_mag_milli",
    "acc_w_milli",
    "acc_n_milli",
    "acc_dot_milli",
)

OUTPUT_COLUMNS = (
    "seq", "host_ts_us", "ts_us", "dt_us", "dt_expected_us",
    "seq_delta", "ts_us_delta", "ts_monotonic_ok", "gap_flag",
    "raw_acc_mg_x", "raw_acc_mg_y", "raw_acc_mg_z",
    "raw_gyro_mdps_x", "raw_gyro_mdps_y", "raw_gyro_mdps_z",
    "raw_mag_cuT_x", "raw_mag_cuT_y", "raw_mag_cuT_z",
    "g_est_mg_x", "g_est_mg_y", "g_est_mg_z",
    "innov_acc_milli", "innov_mag_milli", "acc_w_milli", "acc_n_milli",
    "e_acc_milli_x", "e_acc_milli_y", "e_acc_milli_z", "acc_dot_milli",
    "roll_mdeg", "pitch_mdeg", "yaw_mdeg",
)


def split_top_level_commas(payload: str) -> list[str]:
    parts: list[str] = []
    buffer: list[str] = []
    depth = 0
    for character in payload:
        if character == "[":
            depth += 1
        elif character == "]" and depth > 0:
            depth -= 1

        if character == "," and depth == 0:
            token = "".join(buffer).strip()
            if token:
                parts.append(token)
            buffer.clear()
        else:
            buffer.append(character)

    token = "".join(buffer).strip()
    if token:
        parts.append(token)
    return parts


def split_records(line: str) -> list[str]:
    starts: list[int] = []
    search_from = 0
    while True:
        start = line.find(RECORD_PREFIX, search_from)
        if start < 0:
            break
        starts.append(start)
        search_from = start + len(RECORD_PREFIX)

    return [
        line[start : starts[index + 1] if index + 1 < len(starts) else len(line)].strip()
        for index, start in enumerate(starts)
    ]


def parse_record(record: str) -> dict[str, int] | None:
    if not record.startswith(RECORD_PREFIX):
        return None

    payload = record[len(RECORD_PREFIX) :].lstrip(" :")
    values: dict[str, str] = {}
    for token in split_top_level_commas(payload):
        if "=" not in token:
            continue
        name, value = token.split("=", 1)
        if name.strip():
            values[name.strip()] = value.strip()

    try:
        timestamp_us = int(values["ts_us"])
        delta_us = int(values["dt_us"])
    except (KeyError, ValueError):
        return None

    if timestamp_us < 0 or timestamp_us > 1_000_000_000_000_000:
        return None
    if delta_us <= 0 or delta_us > 1_000_000:
        return None

    try:
        sequence = int(values.get("seq", "-1"))
    except ValueError:
        sequence = -1

    output = {"seq": sequence, "ts_us": timestamp_us, "dt_us": delta_us}
    present_vectors: set[str] = set()
    for input_name, output_names in VECTOR_COLUMNS.items():
        match = INTEGER_TRIPLE.match(values.get(input_name, ""))
        if match is None:
            continue
        output.update(zip(output_names, (int(item) for item in match.groups())))
        present_vectors.add(input_name)

    # The normalized analyzer requires both the measurement and its gravity
    # estimate; truncated records are rejected instead of silently zero-filled.
    if not {"raw_acc_mg", "g_est_mg"}.issubset(present_vectors):
        return None

    for name in SCALAR_COLUMNS:
        if name not in values:
            continue
        try:
            output[name] = int(values[name])
        except ValueError:
            pass
    return output


def recorder_timestamp_us(row: dict[str, str]) -> int:
    try:
        explicit = int((row.get("host_ts_us") or "").strip())
        if explicit >= 0:
            return explicit
    except ValueError:
        pass

    try:
        return int(round(float((row.get("ts_sec") or "").strip()) * 1_000_000.0))
    except ValueError:
        return 0


def capture_rows(path: Path) -> Iterable[dict[str, int]]:
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None or "line" not in reader.fieldnames:
            raise ValueError("capture CSV must contain a 'line' column")

        for capture_row in reader:
            host_timestamp = recorder_timestamp_us(capture_row)
            for record in split_records(capture_row.get("line") or ""):
                parsed = parse_record(record)
                if parsed is not None:
                    parsed["host_ts_us"] = host_timestamp
                    yield parsed


def deduplicate(rows: Iterable[dict[str, int]]) -> list[dict[str, int]]:
    unique: list[dict[str, int]] = []
    keys: set[tuple[int, ...]] = set()
    for row in rows:
        sequence = row["seq"]
        key = (sequence, row["ts_us"]) if sequence >= 0 else (row["ts_us"],)
        if key not in keys:
            keys.add(key)
            unique.append(row)
    return unique


def annotate_continuity(rows: list[dict[str, int]]) -> None:
    expected_delta = int(statistics.median(row["dt_us"] for row in rows))
    previous_sequence: int | None = None
    previous_timestamp: int | None = None

    for row in rows:
        sequence_delta = 0
        timestamp_delta = 0
        monotonic = 1
        gap = 0

        if previous_timestamp is not None:
            timestamp_delta = row["ts_us"] - previous_timestamp
            monotonic = int(timestamp_delta > 0)
            gap = int(timestamp_delta <= 0 or timestamp_delta > expected_delta * 1.5)

        if previous_sequence is not None and row["seq"] >= 0 and previous_sequence >= 0:
            sequence_delta = row["seq"] - previous_sequence
            if sequence_delta != 1:
                gap = 1

        row.update(
            {
                "dt_expected_us": expected_delta,
                "seq_delta": sequence_delta,
                "ts_us_delta": timestamp_delta,
                "ts_monotonic_ok": monotonic,
                "gap_flag": gap,
            }
        )
        previous_sequence = row["seq"]
        previous_timestamp = row["ts_us"]


def convert_capture(input_path: Path, output_path: Path) -> int:
    rows = deduplicate(capture_rows(input_path))
    if not rows:
        raise ValueError("capture contains no complete IMUCSV records")

    annotate_continuity(rows)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=OUTPUT_COLUMNS)
        writer.writeheader()
        for row in rows:
            writer.writerow({column: row.get(column, "") for column in OUTPUT_COLUMNS})
    return len(rows)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture_csv", type=Path, help="input capture CSV")
    parser.add_argument("--out", type=Path, default=None, help="output path")
    args = parser.parse_args()

    output_path = args.out or args.capture_csv.with_name("golden_imu.csv")
    count = convert_capture(args.capture_csv, output_path)
    print(f"Wrote {count} samples to {output_path}")


if __name__ == "__main__":
    main()
