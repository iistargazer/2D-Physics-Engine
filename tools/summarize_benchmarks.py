#!/usr/bin/env python3
"""Print aligned text summaries of the phys2d benchmark CSV files.

Usage:
    python tools/summarize_benchmarks.py
    python tools/summarize_benchmarks.py build/ccd.csv build/joints.csv

With no arguments the default benchmark outputs under ``build/`` are used.
A file that is missing, empty or unreadable is reported as a warning and
skipped; only a run in which *no* input file could be read exits non-zero.

The parser is deliberately tolerant, because the benchmark programs emit
plain ``printf`` output rather than a strict CSV dialect:

* lines without a comma are treated as free-form notes (for example the
  ``Compiler: ...`` banner printed by ``benchmark_broadphase``);
* a row whose fields are mostly non-numeric starts a new table, which is
  what lets ``stability.csv`` hold two tables with different columns;
* a row whose fields are mostly numeric is data.

Standard library only. Tested on Python 3.13 (Windows).
"""

from __future__ import annotations

import sys
from pathlib import Path

DEFAULT_FILES = (
    "build/broadphase_benchmark.csv",
    "build/stability.csv",
    "build/joints.csv",
    "build/ccd.csv",
    "build/showcase.csv",
    "build/active.csv",
)


def is_number(text: str) -> bool:
    """True when ``text`` parses as a float."""
    try:
        float(text)
    except ValueError:
        return False
    return True


def parse(text: str):
    """Split raw benchmark output into notes and tables.

    Returns ``(notes, tables)`` where ``tables`` is a list of
    ``(header, rows)`` tuples, both lists of strings.
    """
    notes: list[str] = []
    tables: list[tuple[list[str], list[list[str]]]] = []
    header: list[str] | None = None
    rows: list[list[str]] = []

    for raw_line in text.splitlines():
        line = raw_line.strip()
        if not line:
            continue
        if "," not in line:
            notes.append(line)
            continue
        fields = [field.strip() for field in line.split(",")]
        numeric = sum(1 for field in fields if is_number(field))
        if numeric * 2 < len(fields):
            # Mostly text: a header row. Flush the previous table first.
            if header is not None:
                tables.append((header, rows))
            header, rows = fields, []
        else:
            if header is None:
                header = [f"col{index + 1}" for index in range(len(fields))]
            rows.append(fields)

    if header is not None:
        tables.append((header, rows))
    return notes, tables


def render_table(header: list[str], rows: list[list[str]]) -> list[str]:
    """Format one table with per-column widths and numeric right alignment."""
    columns = len(header)
    padded: list[list[str]] = []
    for row in rows:
        cells = list(row[:columns])
        cells += [""] * (columns - len(cells))
        padded.append(cells)

    widths = [len(name) for name in header]
    for cells in padded:
        for index, cell in enumerate(cells):
            widths[index] = max(widths[index], len(cell))

    numeric_columns = [
        bool(padded) and all(is_number(cells[index]) for cells in padded)
        for index in range(columns)
    ]

    def format_row(cells: list[str]) -> str:
        parts = []
        for index, cell in enumerate(cells):
            parts.append(cell.rjust(widths[index]) if numeric_columns[index]
                         else cell.ljust(widths[index]))
        return " | ".join(parts).rstrip()

    lines = [format_row(header)]
    lines.append("-+-".join("-" * width for width in widths))
    lines.extend(format_row(cells) for cells in padded)
    return lines


def main(argv: list[str]) -> int:
    files = argv[1:] or list(DEFAULT_FILES)
    read_any = False
    total_tables = 0
    total_rows = 0

    for name in files:
        path = Path(name)
        print("=" * 78)
        print(f"FILE {path}")
        print("=" * 78)

        try:
            text = path.read_text(encoding="utf-8", errors="replace")
        except FileNotFoundError:
            print(f"  WARNING: missing file, skipped: {path}")
            print()
            continue
        except OSError as error:
            print(f"  WARNING: unreadable ({error.__class__.__name__}), skipped: {path}")
            print()
            continue

        read_any = True
        notes, tables = parse(text)

        for note in notes:
            print(f"  note: {note}")
        if notes:
            print()

        if not tables:
            print("  WARNING: no table rows found in this file")
            print()
            continue

        for index, (header, rows) in enumerate(tables, start=1):
            total_tables += 1
            total_rows += len(rows)
            columns = len(header)
            print(f"  table {index}: {len(rows)} data row(s), {columns} column(s)")
            print()
            for line in render_table(header, rows):
                print(f"  {line}")
            print()

    print("=" * 78)
    if not read_any:
        print("No benchmark CSV could be read.", file=sys.stderr)
        return 1
    print(f"Summarised {len(files)} requested file(s): "
          f"{total_tables} table(s), {total_rows} data row(s).")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
