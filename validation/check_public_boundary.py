#!/usr/bin/env python3
"""Reject files and includes that cross the public algorithm boundary."""

from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path, PurePosixPath


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]

PRIVATE_DIRECTORY_NAMES = {
    "bsp",
    "core",
    "cubemx",
    "drivers",
    "iar_files",
    "middlewares",
}

PRIVATE_SUFFIXES = {".ewp", ".eww", ".icf", ".ioc", ".ulg"}

PRIVATE_FILENAME_PATTERNS = (
    re.compile(r"^startup_stm32", re.IGNORECASE),
)

PRIVATE_INCLUDE_PATTERNS = (
    re.compile(r"^cmsis_os(?:2)?\.h$", re.IGNORECASE),
    re.compile(r"^freertos\.h$", re.IGNORECASE),
    re.compile(r"^stm32[^/]*\.h$", re.IGNORECASE),
    re.compile(r"^drv[^/]*\.h$", re.IGNORECASE),
    re.compile(r"^sensor_hal\.h$", re.IGNORECASE),
    re.compile(r"^periph_[^/]*\.h$", re.IGNORECASE),
)

INCLUDE_RE = re.compile(r'^\s*#\s*include\s*[<"]([^>"]+)[>"]')


def tracked_files() -> list[PurePosixPath]:
    result = subprocess.run(
        ["git", "ls-files", "-z"],
        cwd=REPOSITORY_ROOT,
        check=True,
        capture_output=True,
    )
    return [
        PurePosixPath(item.decode("utf-8"))
        for item in result.stdout.split(b"\0")
        if item
    ]


def path_violations(path: PurePosixPath) -> list[str]:
    failures: list[str] = []
    lowered_parts = {part.lower() for part in path.parts[:-1]}
    private_parts = sorted(lowered_parts & PRIVATE_DIRECTORY_NAMES)
    if private_parts:
        failures.append(f"private directory name: {', '.join(private_parts)}")
    if path.suffix.lower() in PRIVATE_SUFFIXES:
        failures.append(f"private/generated file suffix: {path.suffix}")
    for pattern in PRIVATE_FILENAME_PATTERNS:
        if pattern.search(path.name):
            failures.append(f"private/generated filename: {path.name}")
    return failures


def include_violations(path: PurePosixPath) -> list[str]:
    if not path.parts or path.parts[0] not in {"include", "src"}:
        return []
    local_path = REPOSITORY_ROOT / Path(path)
    try:
        contents = local_path.read_text(encoding="utf-8")
    except UnicodeDecodeError:
        return ["non-text source under include/ or src/"]

    failures: list[str] = []
    for line_number, line in enumerate(contents.splitlines(), start=1):
        match = INCLUDE_RE.match(line)
        if not match:
            continue
        include_name = match.group(1).replace("\\", "/")
        basename = include_name.rsplit("/", 1)[-1]
        if any(pattern.search(basename) for pattern in PRIVATE_INCLUDE_PATTERNS):
            failures.append(
                f"hardware/platform include at line {line_number}: {include_name}"
            )
    return failures


def main() -> int:
    failures: list[str] = []
    for path in tracked_files():
        for reason in (*path_violations(path), *include_violations(path)):
            failures.append(f"{path}: {reason}")

    if failures:
        print("Public repository boundary violations:", file=sys.stderr)
        for failure in failures:
            print(f"  - {failure}", file=sys.stderr)
        return 1

    print("Public repository boundary check passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
