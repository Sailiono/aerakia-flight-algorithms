#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Estimate required UART baud rate for printing IMUCSV telemetry.

Given a text capture CSV, this script:
- Finds all "[IMUCSV]" segments (handles multiple segments concatenated in one CSV row).
- Computes average bytes per IMUCSV sample (UTF-8 payload bytes + optional newline).
- Estimates required baud for a target sample rate.
- Computes observed sample rate (from host timestamps) and observed throughput.

Notes
- UART typically uses 8N1 framing -> ~10 bits per byte on the wire.
- Real throughput is usually lower due to ISR/driver/USB-serial overhead.
  Use `--efficiency` (e.g. 0.85) as a guard band.

Example
  python simulation/tools/estimate_uart_baud_for_imu_csv.py \
    artifacts/serial_output.csv \
    --target-hz 400
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional


_IMUCSV_PREFIX = "[IMUCSV]"


def _split_imu_segments(line: str) -> List[str]:
    if _IMUCSV_PREFIX not in line:
        return []

    idxs: List[int] = []
    start = 0
    while True:
        i = line.find(_IMUCSV_PREFIX, start)
        if i < 0:
            break
        idxs.append(i)
        start = i + len(_IMUCSV_PREFIX)

    segs: List[str] = []
    for k, i0 in enumerate(idxs):
        i1 = idxs[k + 1] if (k + 1) < len(idxs) else len(line)
        seg = line[i0:i1].strip()
        if seg:
            segs.append(seg)
    return segs


@dataclass
class Stats:
    samples: int
    bytes_total: int
    bytes_avg: float
    bytes_p50: int
    bytes_p95: int
    host_ts_first_us: Optional[int]
    host_ts_last_us: Optional[int]

    @property
    def duration_s(self) -> float:
        if self.host_ts_first_us is None or self.host_ts_last_us is None:
            return 0.0
        if self.host_ts_last_us <= self.host_ts_first_us:
            return 0.0
        return (self.host_ts_last_us - self.host_ts_first_us) * 1e-6

    @property
    def observed_hz(self) -> float:
        d = self.duration_s
        if d <= 0.0:
            return 0.0
        return float(self.samples) / d

    @property
    def observed_bytes_per_s(self) -> float:
        d = self.duration_s
        if d <= 0.0:
            return 0.0
        return float(self.bytes_total) / d


def compute_stats(serial_csv_path: Path, include_newline_bytes: int) -> Stats:
    if not serial_csv_path.exists():
        raise FileNotFoundError(serial_csv_path)

    sizes: List[int] = []
    bytes_total = 0
    samples = 0
    host_ts_first_us: Optional[int] = None
    host_ts_last_us: Optional[int] = None

    with serial_csv_path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None or "line" not in reader.fieldnames:
            raise RuntimeError(f"Unexpected columns in {serial_csv_path}: {reader.fieldnames}")

        for rec in reader:
            line = (rec.get("line") or "")
            segs = _split_imu_segments(line)
            if not segs:
                continue

            # host timestamp, if available
            try:
                host_ts_sec = float((rec.get("ts_sec") or "").strip() or 0.0)
                host_ts_us = int(host_ts_sec * 1e6 + 0.5)
            except ValueError:
                host_ts_us = 0

            if host_ts_us > 0:
                if host_ts_first_us is None:
                    host_ts_first_us = host_ts_us
                host_ts_last_us = host_ts_us

            for seg in segs:
                # Approximate bytes on the wire for the payload.
                # UART receives text bytes plus line ending; include_newline_bytes can be 0/1/2.
                b = len(seg.encode("utf-8")) + include_newline_bytes
                sizes.append(b)
                bytes_total += b
                samples += 1

    if samples == 0:
        raise RuntimeError(f"No {_IMUCSV_PREFIX} segments found in {serial_csv_path}")

    sizes_sorted = sorted(sizes)
    p50 = sizes_sorted[int(round(0.50 * (len(sizes_sorted) - 1)))]
    p95 = sizes_sorted[int(round(0.95 * (len(sizes_sorted) - 1)))]

    return Stats(
        samples=samples,
        bytes_total=bytes_total,
        bytes_avg=float(bytes_total) / float(samples),
        bytes_p50=int(p50),
        bytes_p95=int(p95),
        host_ts_first_us=host_ts_first_us,
        host_ts_last_us=host_ts_last_us,
    )


def baud_required(bytes_per_sample: float, target_hz: float, bits_per_byte: float, efficiency: float) -> float:
    if target_hz <= 0:
        return 0.0
    if efficiency <= 0.0 or efficiency > 1.0:
        raise ValueError("efficiency must be in (0, 1]")
    return (bytes_per_sample * target_hz * bits_per_byte) / efficiency


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("serial_csv", help="Path to serial_output.csv")
    ap.add_argument("--target-hz", type=float, default=400.0, help="Desired IMUCSV sample rate")
    ap.add_argument("--bits-per-byte", type=float, default=10.0, help="UART framing bits per byte (8N1 ~ 10)")
    ap.add_argument(
        "--efficiency",
        type=float,
        default=0.85,
        help="Throughput efficiency factor (guard band for overhead), in (0,1]",
    )
    ap.add_argument(
        "--newline-bytes",
        type=int,
        default=2,
        help="Line ending bytes to include per sample (0,1,2). Windows CRLF is typically 2.",
    )
    args = ap.parse_args()

    serial_csv_path = Path(args.serial_csv)
    st = compute_stats(serial_csv_path, include_newline_bytes=int(args.newline_bytes))

    req_avg = baud_required(st.bytes_avg, float(args.target_hz), float(args.bits_per_byte), float(args.efficiency))
    req_p95 = baud_required(float(st.bytes_p95), float(args.target_hz), float(args.bits_per_byte), float(args.efficiency))

    print("IMUCSV UART budget estimate")
    print(f"- file: {serial_csv_path}")
    print(f"- samples: {st.samples}")
    if st.duration_s > 0:
        print(f"- duration (host): {st.duration_s:.3f} s")
        print(f"- observed rate: {st.observed_hz:.2f} Hz")
        print(f"- observed throughput: {st.observed_bytes_per_s/1024.0:.2f} KiB/s")
        print(f"- observed baud equiv (@{args.bits_per_byte:g}b/B): {st.observed_bytes_per_s*args.bits_per_byte:.0f} bps")
    else:
        print("- duration (host): n/a (host ts missing)")

    print("- bytes/sample:")
    print(f"  - avg: {st.bytes_avg:.1f} B")
    print(f"  - p50: {st.bytes_p50} B")
    print(f"  - p95: {st.bytes_p95} B")

    print("- target:")
    print(f"  - target_hz: {args.target_hz:g} Hz")
    print(f"  - bits_per_byte: {args.bits_per_byte:g}")
    print(f"  - efficiency: {args.efficiency:g}")

    print("- required baud (rule-of-thumb):")
    print(f"  - using avg bytes/sample: {req_avg:.0f} bps")
    print(f"  - using p95 bytes/sample: {req_p95:.0f} bps")


if __name__ == "__main__":
    main()
