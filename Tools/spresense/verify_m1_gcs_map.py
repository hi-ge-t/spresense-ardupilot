#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

"""Verify Application SRAM and GNSS RAM boundaries from a GNU ld map."""

from dataclasses import dataclass
import argparse
import json
from pathlib import Path
import re
import sys


APP_ORIGIN = 0x0D000000
APP_BYTES = 1536 * 1024
APP_ALIAS = 0x20000000
GNSS_ORIGIN = 0x09000000
GNSS_BYTES = 640 * 1024
MEMORY_RE = re.compile(
    r"^([A-Za-z][A-Za-z0-9_]*)\s+(0x[0-9a-fA-F]+)\s+"
    r"(0x[0-9a-fA-F]+)(?:\s+.*)?$"
)
SECTION_RE = re.compile(
    r"^(\.[A-Za-z0-9_.-]+)\s+(0x[0-9a-fA-F]+)\s+"
    r"(0x[0-9a-fA-F]+)(?:\s+.*)?$"
)
INPUT_RE = re.compile(
    r"^\s+(\.[A-Za-z0-9_.-]+)\s+(0x[0-9a-fA-F]+)\s+"
    r"(0x[0-9a-fA-F]+)(?:\s+.*)?$"
)
SYMBOL_RE = re.compile(
    r"^\s*(0x[0-9a-fA-F]+)\s+(?:PROVIDE\s*\(\s*)?"
    r"([A-Za-z_][A-Za-z0-9_]*)\s*(?:=|\)|$)"
)


class MapError(ValueError):
    pass


@dataclass(frozen=True)
class Span:
    start: int
    end: int

    @property
    def size(self) -> int:
        return self.end - self.start


def merged(spans: list[Span], name: str) -> Span:
    values = [span for span in spans if span.size > 0]
    if not values:
        raise MapError(f"missing non-empty section: {name}")
    return Span(min(value.start for value in values),
                max(value.end for value in values))


def require_region(span: Span, origin: int, size: int, name: str,
                   alias: int = 0) -> None:
    start = span.start - alias
    end = span.end - alias
    if start < origin or end > origin + size or start >= end:
        raise MapError(
            f"{name} 0x{span.start:08x}..0x{span.end:08x} outside "
            f"0x{origin:08x}..0x{origin + size:08x}"
        )


def parse_map(path: Path):
    memory: dict[str, Span] = {}
    sections: dict[str, Span] = {}
    symbols: dict[str, int] = {}
    app_code: list[Span] = []
    app_rodata: list[Span] = []
    in_memory = False
    in_map = False
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.rstrip()
        if line == "Memory Configuration":
            in_memory = True
            continue
        if line == "Linker script and memory map":
            in_memory = False
            in_map = True
            continue
        if line == "Cross Reference Table":
            in_map = False
            continue
        if in_memory:
            match = MEMORY_RE.match(line)
            if match:
                start = int(match.group(2), 16)
                memory[match.group(1)] = Span(
                    start, start + int(match.group(3), 16)
                )
            continue
        if not in_map:
            continue
        match = SECTION_RE.match(line)
        if match:
            start = int(match.group(2), 16)
            sections[match.group(1)] = Span(
                start, start + int(match.group(3), 16)
            )
            continue
        match = INPUT_RE.match(line)
        if match:
            name = match.group(1)
            start = int(match.group(2), 16)
            span = Span(start, start + int(match.group(3), 16))
            if name == ".text" or name.startswith(".text."):
                app_code.append(span)
            elif (name == ".rodata" or name.startswith(".rodata.") or
                  name.startswith(".gnu.linkonce.r.")):
                app_rodata.append(span)
            continue
        match = SYMBOL_RE.match(line)
        if match:
            symbols.setdefault(match.group(2), int(match.group(1), 16))
    sections["application.code"] = merged(app_code, "application code")
    sections["application.rodata"] = merged(
        app_rodata, "application rodata"
    )
    return memory, sections, symbols


def span_json(span: Span, alias: int = 0) -> dict[str, int | str]:
    value: dict[str, int | str] = {
        "start": f"0x{span.start:08x}",
        "end": f"0x{span.end:08x}",
        "bytes": span.size,
    }
    if alias:
        value["backing_start"] = f"0x{span.start - alias:08x}"
        value["backing_end"] = f"0x{span.end - alias:08x}"
    return value


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

    required_symbols = (
        "g_m1_gcs_gnss_rodata",
        "g_m1_gcs_gnss_data",
        "_ebss",
        "__stack",
        "_sgnssheap",
        "_egnssheap",
    )
    for name in required_symbols:
        if name not in symbols:
            raise MapError(f"missing linker symbol: {name}")

    gnss_data_output = sections.get(".gnssram.data")
    if gnss_data_output is None or gnss_data_output.size <= 0:
        raise MapError("missing non-empty section: .gnssram.data")
    starts = sorted(
        (symbols["g_m1_gcs_gnss_rodata"],
         symbols["g_m1_gcs_gnss_data"])
    )
    if starts[0] == starts[1] or starts[0] != gnss_data_output.start:
        raise MapError("GNSS RAM rodata/data boundaries overlap or drift")
    named_starts = {
        "rodata": symbols["g_m1_gcs_gnss_rodata"],
        "data": symbols["g_m1_gcs_gnss_data"],
    }
    gnss_split = {}
    for name, start in named_starts.items():
        later = [candidate for candidate in starts if candidate > start]
        gnss_split[name] = Span(
            start, min(later) if later else gnss_data_output.end
        )

    required = {
        "app_code": sections.get("application.code"),
        "app_rodata": sections.get("application.rodata"),
        "app_data": sections.get(".data"),
        "app_bss": sections.get(".bss"),
        "gnss_code": sections.get(".gnssram.text"),
        "gnss_rodata": gnss_split["rodata"],
        "gnss_data": gnss_split["data"],
        "gnss_bss": sections.get(".gnssram.bss"),
    }
    for name, span in required.items():
        if span is None or span.size <= 0:
            raise MapError(f"missing non-empty section: {name}")
    app_code = required["app_code"]
    app_rodata = required["app_rodata"]
    app_data = required["app_data"]
    app_bss = required["app_bss"]
    gnss_code = required["gnss_code"]
    gnss_rodata = required["gnss_rodata"]
    gnss_data = required["gnss_data"]
    gnss_bss = required["gnss_bss"]
    assert all(value is not None for value in required.values())
    require_region(app_code, APP_ORIGIN, APP_BYTES, "application code")
    require_region(app_rodata, APP_ORIGIN, APP_BYTES, "application rodata")
    require_region(app_data, APP_ORIGIN, APP_BYTES, "application data",
                   APP_ALIAS)
    require_region(app_bss, APP_ORIGIN, APP_BYTES, "application bss",
                   APP_ALIAS)
    for name, span in (
        ("GNSS code", gnss_code),
        ("GNSS rodata", gnss_rodata),
        ("GNSS data", gnss_data),
        ("GNSS bss", gnss_bss),
    ):
        require_region(span, GNSS_ORIGIN, GNSS_BYTES, name)
    app_heap = Span(symbols["_ebss"], symbols["__stack"])
    gnss_heap = Span(symbols["_sgnssheap"], symbols["_egnssheap"])
    require_region(app_heap, APP_ORIGIN, APP_BYTES, "application heap",
                   APP_ALIAS)
    require_region(gnss_heap, GNSS_ORIGIN, GNSS_BYTES, "GNSS heap")
    if gnss_heap.start < gnss_bss.end:
        raise MapError("GNSS heap overlaps GNSS bss")

    return {
        "format": "spresense-m1-gcs-memory-v1",
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
        },
        "hardware_runtime_verified": False,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    try:
        report = inspect(args.map)
        args.output.write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
    except (MapError, OSError, UnicodeError) as error:
        print(f"spresense_m1_gcs_map=FAIL reason={error}", file=sys.stderr)
        return 1
    print(
        "spresense_m1_gcs_map=PASS "
        f"gnss_heap_bytes={report['gnss_ram']['heap']['bytes']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
