#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

"""Verify M1 Copter Application SRAM and GNSS RAM linker boundaries."""

import argparse
import json
from pathlib import Path
import sys

from verify_m1_gcs_map import (
    APP_ALIAS,
    APP_BYTES,
    APP_ORIGIN,
    GNSS_BYTES,
    GNSS_ORIGIN,
    MapError,
    Span,
    parse_map,
    require_region,
    span_json,
)


def require_region_allow_empty(
    span: Span, origin: int, size: int, name: str, alias: int = 0
) -> None:
    start = span.start - alias
    end = span.end - alias
    if start < origin or end > origin + size or start > end:
        raise MapError(
            f"{name} 0x{span.start:08x}..0x{span.end:08x} outside "
            f"0x{origin:08x}..0x{origin + size:08x}"
        )


def require_symbol(symbols: dict[str, int], name: str) -> int:
    if name not in symbols:
        raise MapError(f"missing linker symbol: {name}")
    return symbols[name]


def inspect(path: Path) -> dict[str, object]:
    if not path.is_file():
        raise MapError(f"linker map is missing: {path}")
    memory, sections, symbols = parse_map(path)
    expected_memory = {
        "ram": Span(APP_ORIGIN, APP_ORIGIN + APP_BYTES),
        "gnssram": Span(GNSS_ORIGIN, GNSS_ORIGIN + GNSS_BYTES),
    }
    for name, expected in expected_memory.items():
        if memory.get(name) != expected:
            raise MapError(f"memory region mismatch: {name}")

    app_code = sections.get("application.code")
    app_rodata = sections.get("application.rodata")
    app_data = sections.get(".data")
    app_bss = sections.get(".bss")
    for name, span in (
        ("application code", app_code),
        ("application rodata", app_rodata),
        ("application data", app_data),
        ("application bss", app_bss),
    ):
        if span is None or span.size <= 0:
            raise MapError(f"missing non-empty section: {name}")
    assert app_code is not None
    assert app_rodata is not None
    assert app_data is not None
    assert app_bss is not None
    require_region(app_code, APP_ORIGIN, APP_BYTES, "application code")
    require_region(app_rodata, APP_ORIGIN, APP_BYTES, "application rodata")
    require_region(
        app_data, APP_ORIGIN, APP_BYTES, "application data", APP_ALIAS
    )
    require_region(
        app_bss, APP_ORIGIN, APP_BYTES, "application bss", APP_ALIAS
    )

    app_heap = Span(
        require_symbol(symbols, "_ebss"),
        require_symbol(symbols, "__stack"),
    )
    require_region(
        app_heap, APP_ORIGIN, APP_BYTES, "application heap", APP_ALIAS
    )

    gnss_text_start = require_symbol(symbols, "_sgnsstext")
    gnss_bss_start = require_symbol(symbols, "_gnssramsbss")
    gnss_bss_end = require_symbol(symbols, "_gnssramebss")
    gnss_heap = Span(
        require_symbol(symbols, "_sgnssheap"),
        require_symbol(symbols, "_egnssheap"),
    )
    gnss_code = sections.get(
        ".gnssram.text", Span(gnss_text_start, gnss_text_start)
    )
    gnss_data = sections.get(
        ".gnssram.data", Span(gnss_code.end, gnss_bss_start)
    )
    gnss_bss = sections.get(
        ".gnssram.bss", Span(gnss_bss_start, gnss_bss_end)
    )
    gnss_rodata = Span(gnss_code.end, gnss_code.end)

    gnss_static = (
        ("GNSS code", gnss_code),
        ("GNSS rodata", gnss_rodata),
        ("GNSS data", gnss_data),
        ("GNSS bss", gnss_bss),
    )
    for name, span in gnss_static:
        require_region_allow_empty(span, GNSS_ORIGIN, GNSS_BYTES, name)
        if span.size != 0:
            raise MapError(f"{name} must be empty in output-disabled vehicle")
    require_region(gnss_heap, GNSS_ORIGIN, GNSS_BYTES, "GNSS heap")
    expected_gnss_heap = Span(GNSS_ORIGIN, GNSS_ORIGIN + GNSS_BYTES)
    if gnss_heap != expected_gnss_heap:
        raise MapError("GNSS heap does not cover the complete GNSS RAM region")
    boundaries = (
        gnss_code.start,
        gnss_code.end,
        gnss_rodata.start,
        gnss_rodata.end,
        gnss_data.start,
        gnss_data.end,
        gnss_bss.start,
        gnss_bss.end,
        gnss_heap.start,
        gnss_heap.end,
    )
    if tuple(sorted(boundaries)) != boundaries:
        raise MapError("GNSS code/rodata/data/bss/heap boundaries overlap")

    return {
        "format": "spresense-m1-copter-memory-v1",
        "application_sram": {
            "code": span_json(app_code),
            "rodata": span_json(app_rodata),
            "data": span_json(app_data, APP_ALIAS),
            "bss": span_json(app_bss, APP_ALIAS),
            "heap_envelope": span_json(app_heap, APP_ALIAS),
        },
        "gnss_ram": {
            "code": span_json(gnss_code),
            "rodata": span_json(gnss_rodata),
            "data": span_json(gnss_data),
            "bss": span_json(gnss_bss),
            "heap": span_json(gnss_heap),
            "static_sections": "required-empty",
            "heap_policy": "complete-region",
        },
        "hardware_runtime_verified": False,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument(
        "--vehicle", choices=("copter", "rover"), default="copter"
    )
    args = parser.parse_args()
    try:
        report = inspect(args.map)
        report["format"] = f"spresense-m1-{args.vehicle}-memory-v1"
        args.output.write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
    except (MapError, OSError, UnicodeError) as error:
        print(
            f"spresense_m1_{args.vehicle}_map=FAIL reason={error}",
            file=sys.stderr,
        )
        return 1
    print(
        f"spresense_m1_{args.vehicle}_map=PASS "
        f"app_heap_bytes={report['application_sram']['heap_envelope']['bytes']} "
        f"gnss_heap_bytes={report['gnss_ram']['heap']['bytes']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
