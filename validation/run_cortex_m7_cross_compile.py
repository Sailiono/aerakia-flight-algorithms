#!/usr/bin/env python3
"""Compile the portable library for the FCOne Cortex-M7 target profile.

This is a compiler/layout preflight only. It does not link an FCOne image,
measure target execution time, or replace board-level verification.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import shutil
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


ROOT = Path(__file__).resolve().parents[1]
BUILD_ROOT = ROOT / "build"
SOURCES = (
    "src/barometer_supervisor.c",
    "src/eskf.c",
    "src/eskf_adapter.c",
    "src/eskf_math.c",
    "src/eskf_models.c",
    "src/mag_gate.c",
    "src/mahony.c",
    "src/static_imu_calibration.c",
)
FLOAT_CORE_SOURCES = (
    "src/eskf.c",
    "src/eskf_math.c",
    "src/eskf_models.c",
)
EVIDENCE_HEADER_SOURCES = tuple(
    str(path.relative_to(ROOT))
    for path in sorted(
        (
            *ROOT.glob("include/aerakia/**/*.h"),
            *ROOT.glob("src/**/*.h"),
        )
    )
)
EVIDENCE_MANIFEST_SOURCES = (*SOURCES, *EVIDENCE_HEADER_SOURCES)
EVIDENCE_SOURCES = (*EVIDENCE_MANIFEST_SOURCES, "validation/run_cortex_m7_cross_compile.py")
BASE_FLAGS = (
    "-mcpu=cortex-m7",
    "-mthumb",
    "-mfloat-abi=hard",
    "-mfpu=fpv5-d16",
    "-std=c99",
    "-Wall",
    "-Wextra",
    "-Werror",
    "-pedantic",
    "-Os",
    "-ffreestanding",
    "-fno-builtin",
    "-ffunction-sections",
    "-fdata-sections",
)
PROFILES = {
    "double": (),
    "float": ("-DAERAKIA_ESKF_CORE_USE_FLOAT=1",),
}
SECTION_LINE = re.compile(r"^(?P<name>\S+)\s+(?P<size>\d+)\s+\d+$")
NM_LINE = re.compile(r"^\s*\d+\s+(?P<size>\d+)\s+\S\s+(?P<name>\S+)\s*$")
MATH_DOUBLE_SYMBOLS = frozenset({"asin", "atan2", "cos", "fabs", "fmax", "fmin", "hypot", "sin", "sqrt"})
MATH_FLOAT_SYMBOLS = frozenset(
    {"asinf", "atan2f", "cosf", "fabsf", "fmaxf", "fminf", "hypotf", "sinf", "sqrtf"}
)
LIBC_SYMBOLS = frozenset({"memcpy", "memset"})


@dataclass(frozen=True)
class Toolchain:
    compiler: str
    archiver: str
    size: str
    nm: str


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def build_source_manifest(
    sources: Iterable[str] = EVIDENCE_MANIFEST_SOURCES,
    root: Path = ROOT,
) -> list[dict[str, str]]:
    return sorted(
        [{"path": source, "sha256": sha256(root / source)} for source in sources],
        key=lambda item: item["path"],
    )


def source_manifest_sha256(source_manifest: Iterable[dict[str, str]]) -> str:
    normalized = [dict(item) for item in source_manifest]
    return hashlib.sha256(
        json.dumps(normalized, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()


def git_show(commit: str, path: str) -> bytes:
    completed = subprocess.run(
        ["git", "show", f"{commit}:{path}"],
        cwd=ROOT,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    if completed.returncode != 0:
        raise RuntimeError(
            f"git show {commit}:{path} failed: "
            f"{completed.stderr.decode('utf-8', errors='replace')}"
        )
    return completed.stdout


def source_manifest_for_commit(
    commit: str,
    sources: Iterable[str] = EVIDENCE_MANIFEST_SOURCES,
) -> list[dict[str, str]]:
    return sorted(
        [
            {
                "path": source,
                "sha256": hashlib.sha256(git_show(commit, source)).hexdigest(),
            }
            for source in sources
        ],
        key=lambda item: item["path"],
    )


def load_evidence_report(path: Path) -> dict[str, object]:
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def git_status_lines(
    paths: Iterable[str],
    cwd: Path = ROOT,
) -> list[str]:
    completed = subprocess.run(
        ["git", "status", "--porcelain", "--", *paths],
        cwd=cwd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    if completed.returncode != 0:
        raise RuntimeError(f"git status failed: {completed.stderr}")
    return [line for line in completed.stdout.splitlines() if line.strip()]


def relative(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(ROOT))
    except ValueError:
        return str(path)


def ensure_safe_work_root(work_root: Path) -> Path:
    """Allow recursive cleanup only below this repository's generated build tree."""
    resolved_work_root = work_root.resolve()
    resolved_build_root = BUILD_ROOT.resolve()
    try:
        resolved_work_root.relative_to(resolved_build_root)
    except ValueError as error:
        raise RuntimeError(
            f"--work-dir must be below {resolved_build_root}, not {resolved_work_root}"
        ) from error
    if resolved_work_root == resolved_build_root:
        raise RuntimeError("--work-dir must be a dedicated directory below build/")
    return resolved_work_root


def run(command: list[str], *, cwd: Path = ROOT) -> str:
    completed = subprocess.run(
        command,
        cwd=cwd,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )
    if completed.returncode != 0:
        raise RuntimeError(
            f"command failed ({completed.returncode}): {' '.join(command)}\n{completed.stdout}"
        )
    return completed.stdout


def first_line(output: str) -> str:
    return next((line.strip() for line in output.splitlines() if line.strip()), "")


def resolve_tool(name: str) -> str:
    candidate = Path(name)
    if candidate.is_file():
        return str(candidate)
    resolved = shutil.which(name)
    if resolved is None:
        raise RuntimeError(f"required tool is unavailable: {name}")
    return resolved


def parse_sections(output: str) -> dict[str, int]:
    totals = {"text": 0, "rodata": 0, "data": 0, "bss": 0, "other": 0}
    for line in output.splitlines():
        match = SECTION_LINE.match(line.strip())
        if match is None:
            continue
        name = match.group("name")
        size = int(match.group("size"))
        if name.startswith(".text"):
            totals["text"] += size
        elif name.startswith(".rodata"):
            totals["rodata"] += size
        elif name.startswith(".data"):
            totals["data"] += size
        elif name.startswith(".bss"):
            totals["bss"] += size
        elif name not in {"Total", ".comment", ".ARM.attributes"}:
            totals["other"] += size
    totals["reported_total"] = sum(totals.values())
    return totals


def parse_probe_sizes(output: str) -> dict[str, int]:
    expected = {
        "aerakia_eskf_size_probe": "aerakia_eskf_bytes",
        "eskf_handle_size_probe": "eskf_handle_bytes",
        "aerakia_navigation_estimate_size_probe": "navigation_estimate_bytes",
    }
    sizes: dict[str, int] = {}
    for line in output.splitlines():
        match = NM_LINE.match(line)
        if match is not None and match.group("name") in expected:
            sizes[expected[match.group("name")]] = int(match.group("size"))
    missing = sorted(set(expected.values()) - set(sizes))
    if missing:
        raise RuntimeError(f"target layout probe omitted symbols: {', '.join(missing)}")
    return sizes


def undefined_symbols(toolchain: Toolchain, objects: Iterable[Path]) -> list[str]:
    symbols: set[str] = set()
    for object_path in objects:
        for line in run([toolchain.nm, "-u", str(object_path)]).splitlines():
            text = line.strip()
            if text:
                symbols.add(text.split()[-1])
    return sorted(symbols)


def categorize_symbols(symbols: Iterable[str]) -> dict[str, list[str]]:
    """Classify final external symbols without pretending to link an FCOne image."""
    categories = {
        "compiler_runtime": [],
        "libc": [],
        "libm_double": [],
        "libm_float": [],
        "other": [],
    }
    for symbol in sorted(set(symbols)):
        if symbol.startswith("__aeabi_") or symbol.startswith("__gnu_"):
            categories["compiler_runtime"].append(symbol)
        elif symbol in LIBC_SYMBOLS:
            categories["libc"].append(symbol)
        elif symbol in MATH_DOUBLE_SYMBOLS:
            categories["libm_double"].append(symbol)
        elif symbol in MATH_FLOAT_SYMBOLS:
            categories["libm_float"].append(symbol)
        else:
            categories["other"].append(symbol)
    return categories


def source_dependency_map(
    toolchain: Toolchain,
    object_reports: Iterable[tuple[str, Path]],
    external_symbols: Iterable[str],
) -> dict[str, list[str]]:
    external = set(external_symbols)
    return {
        source: [symbol for symbol in undefined_symbols(toolchain, [object_path]) if symbol in external]
        for source, object_path in object_reports
    }


def write_probe(path: Path) -> None:
    path.write_text(
        "#include <aerakia/eskf_adapter.h>\n"
        "const unsigned char aerakia_eskf_size_probe[sizeof(AerakiaEskf)] = {0};\n"
        "const unsigned char eskf_handle_size_probe[sizeof(ESKF_Handle)] = {0};\n"
        "const unsigned char aerakia_navigation_estimate_size_probe["
        "sizeof(AerakiaNavigationEstimate)] = {0};\n",
        encoding="utf-8",
    )


def float_core_promotion_check(
    toolchain: Toolchain,
    profile_flags: tuple[str, ...],
    includes: tuple[str, ...],
    profile_root: Path,
) -> dict[str, object]:
    """Reject hidden double arithmetic in the float ESKF core before target work."""
    check_root = profile_root / "float-core-promotion-check"
    check_root.mkdir(parents=True, exist_ok=True)
    for source_name in FLOAT_CORE_SOURCES:
        source = ROOT / source_name
        object_path = check_root / f"{source.stem}.o"
        run([
            toolchain.compiler,
            *BASE_FLAGS,
            "-Wdouble-promotion",
            "-Werror=double-promotion",
            *profile_flags,
            *includes,
            "-c",
            str(source),
            "-o",
            str(object_path),
        ])
    return {
        "status": "passed",
        "sources": list(FLOAT_CORE_SOURCES),
        "flags": ["-Wdouble-promotion", "-Werror=double-promotion"],
    }


def build_profile(
    profile: str,
    profile_flags: tuple[str, ...],
    toolchain: Toolchain,
    work_root: Path,
) -> dict[str, object]:
    profile_root = work_root / profile
    profile_root.mkdir(parents=True, exist_ok=True)
    includes = (f"-I{ROOT / 'include'}", f"-I{ROOT / 'src'}")
    objects: list[Path] = []
    source_objects: list[tuple[str, Path]] = []
    object_reports: list[dict[str, object]] = []
    for source_name in SOURCES:
        source = ROOT / source_name
        object_path = profile_root / f"{source.stem}.o"
        command = [
            toolchain.compiler,
            *BASE_FLAGS,
            *profile_flags,
            *includes,
            "-c",
            str(source),
            "-o",
            str(object_path),
        ]
        run(command)
        sections = parse_sections(run([toolchain.size, "-A", "-d", str(object_path)]))
        objects.append(object_path)
        source_objects.append((source_name, object_path))
        object_reports.append(
            {
                "source": source_name,
                "object_bytes": object_path.stat().st_size,
                "sections": sections,
            }
        )
    archive = profile_root / "libaerakia_algorithms.a"
    run([toolchain.archiver, "rcs", str(archive), *(str(path) for path in objects)])

    # Resolve the archive's own symbols first. The remainder is what the FCOne
    # application/startup link must provide, not a misleading union of every
    # translation unit's internal references.
    relocatable = profile_root / "aerakia_algorithms_relocatable.o"
    run([
        toolchain.compiler,
        "-mcpu=cortex-m7",
        "-mthumb",
        "-mfloat-abi=hard",
        "-mfpu=fpv5-d16",
        "-r",
        "-nostdlib",
        "-o",
        str(relocatable),
        *(str(path) for path in objects),
    ])
    external_symbols = undefined_symbols(toolchain, [relocatable])

    probe_source = profile_root / "layout_probe.c"
    probe_object = profile_root / "layout_probe.o"
    write_probe(probe_source)
    run(
        [
            toolchain.compiler,
            *BASE_FLAGS,
            *profile_flags,
            *includes,
            "-c",
            str(probe_source),
            "-o",
            str(probe_object),
        ]
    )
    layout = parse_probe_sizes(
        run([toolchain.nm, "-S", "-t", "d", "--defined-only", str(probe_object)])
    )
    aggregate = {
        key: sum(int(item["sections"][key]) for item in object_reports)
        for key in ("text", "rodata", "data", "bss", "other", "reported_total")
    }
    result: dict[str, object] = {
        "profile": profile,
        "preprocessor_definitions": [flag[2:] for flag in profile_flags if flag.startswith("-D")],
        "library_archive_bytes": archive.stat().st_size,
        "library_section_sum_bytes": aggregate,
        "relocatable_object_bytes": relocatable.stat().st_size,
        "relocatable_section_sum_bytes": parse_sections(
            run([toolchain.size, "-A", "-d", str(relocatable)])
        ),
        "target_layout_bytes": layout,
        "external_link_dependencies": {
            "symbols": external_symbols,
            "categories": categorize_symbols(external_symbols),
            "by_source": source_dependency_map(toolchain, source_objects, external_symbols),
        },
        "objects": object_reports,
    }
    if profile == "float":
        result["float_core_double_promotion_check"] = float_core_promotion_check(
            toolchain, profile_flags, includes, profile_root
        )
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler", default="arm-none-eabi-gcc")
    parser.add_argument("--archiver", default="arm-none-eabi-ar")
    parser.add_argument("--size", default="arm-none-eabi-size")
    parser.add_argument("--nm", default="arm-none-eabi-nm")
    parser.add_argument(
        "--out",
        type=Path,
        default=Path("build/cortex-m7-cross/cortex_m7_cross_compile.json"),
    )
    parser.add_argument(
        "--work-dir",
        type=Path,
        default=Path("build/cortex-m7-cross/work"),
    )
    parser.add_argument("--keep-work", action="store_true")
    args = parser.parse_args()
    dirty = git_status_lines(EVIDENCE_SOURCES)
    if dirty:
        parser.error(
            "refusing to generate evidence from a dirty source tree: "
            + ", ".join(dirty)
        )
    try:
        toolchain = Toolchain(
            compiler=resolve_tool(args.compiler),
            archiver=resolve_tool(args.archiver),
            size=resolve_tool(args.size),
            nm=resolve_tool(args.nm),
        )
    except RuntimeError as error:
        parser.error(str(error))

    output = (args.out if args.out.is_absolute() else ROOT / args.out).resolve()
    try:
        work_root = ensure_safe_work_root(
            args.work_dir if args.work_dir.is_absolute() else ROOT / args.work_dir
        )
    except RuntimeError as error:
        parser.error(str(error))
    try:
        output.relative_to(work_root)
    except ValueError:
        pass
    else:
        parser.error("--out must not be inside --work-dir because cleanup would remove it")
    if work_root.exists():
        shutil.rmtree(work_root)
    profiles: list[dict[str, object]] = []
    try:
        for profile, flags in PROFILES.items():
            profiles.append(build_profile(profile, flags, toolchain, work_root))
        source_manifest = build_source_manifest()
        result = {
            "schema_version": 1,
            "status": "cross_compile_preflight_not_target_runtime_qualification",
            "target": {
                "cpu": "cortex-m7",
                "instruction_set": "thumb",
                "floating_point_abi": "hard",
                "fpu": "fpv5-d16",
            },
            "compiler": {
                "path": toolchain.compiler,
                "version": first_line(run([toolchain.compiler, "--version"])),
            },
            "runner_sha256": sha256(ROOT / "validation/run_cortex_m7_cross_compile.py"),
            "base_flags": list(BASE_FLAGS),
            "source_manifest": source_manifest,
            "source_manifest_sha256": source_manifest_sha256(source_manifest),
            "profiles": profiles,
            "limitations": [
                "This compiles and archives the portable library only; no FCOne application image is linked.",
                "Archive section sums are not final Flash/RAM use after linker garbage collection, startup, HAL, RTOS, logging, or application code.",
                "No Cortex-M7 execution time, stack high-water mark, cache/DMA interaction, exception behavior, or numerical runtime result is measured.",
                "External symbols are collected after a relocatable link resolves library-internal references; they are application/startup link dependencies, not a complete FCOne image closure.",
                "The float profile is a compiler/layout evaluation, not authorization to change the reviewed public double default.",
            ],
            "git_commit": run(["git", "rev-parse", "HEAD"]).strip(),
        }
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(json.dumps(result, indent=2, sort_keys=True))
    finally:
        if not args.keep_work:
            shutil.rmtree(work_root, ignore_errors=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
