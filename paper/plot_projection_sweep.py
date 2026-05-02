#!/usr/bin/env python3
"""Render diagnostic plots from an existing deterministic projection sweep."""

from __future__ import annotations

import argparse
import csv
import gzip
import json
from pathlib import Path
import tarfile
from typing import Any

from paper.projection_plotting import render_projection_diagnostic_plots
from paper.projection_sanity import (
    DEFAULT_EXCURSION_BINS_NMI,
    summarize_projection_rows,
    summarize_projection_rows_by_excursion,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Render plots from a deterministic projection sweep.")
    parser.add_argument(
        "--input-dir",
        type=Path,
        default=Path("paper_eval_outputs/projection_deterministic_51_full"),
        help="Existing deterministic sweep directory containing row CSV outputs.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Directory for rendered PDF plots. Defaults to <input-dir>/plots.",
    )
    parser.add_argument(
        "--max-excursion-nmi",
        type=float,
        default=None,
        help="Optional cap on maximum radial excursion before summarizing and plotting.",
    )
    parser.add_argument(
        "--write-rows",
        action="store_true",
        help="Write the row-level data used for plotting into the output directory.",
    )
    parser.add_argument("--compress-rows", action="store_true", help="Write row-level CSV outputs as gzip files.")
    parser.add_argument(
        "--combined-rows",
        action="store_true",
        help="Write one combined row-level CSV instead of separate straight and mixed-turn row files.",
    )
    parser.add_argument(
        "--archive-output",
        type=Path,
        default=None,
        help="Optional .tar.gz path containing all files written to the output directory.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    input_dir = args.input_dir
    output_dir = args.output_dir or (input_dir / "plots")
    summary = load_summary(input_dir / "summary.json")
    excursion_bins_nmi = tuple(summary.get("config", {}).get("excursion_bins_nmi", DEFAULT_EXCURSION_BINS_NMI))
    if args.max_excursion_nmi is not None:
        excursion_bins_nmi = tuple(
            edge_nmi for edge_nmi in excursion_bins_nmi if float(edge_nmi) <= args.max_excursion_nmi + 1e-9
        )
        if not excursion_bins_nmi or excursion_bins_nmi[-1] < args.max_excursion_nmi - 1e-9:
            excursion_bins_nmi = (*excursion_bins_nmi, float(args.max_excursion_nmi))
    straight_rows, mixed_turn_rows = load_projection_rows(input_dir)
    if not straight_rows or not mixed_turn_rows:
        raise FileNotFoundError(
            "Expected separate straight/mixed row CSVs or a combined projection_rows.csv in the input directory."
        )
    if args.max_excursion_nmi is not None:
        straight_rows = filter_rows_by_excursion(straight_rows, args.max_excursion_nmi)
        mixed_turn_rows = filter_rows_by_excursion(mixed_turn_rows, args.max_excursion_nmi)
        if not straight_rows or not mixed_turn_rows:
            raise ValueError("Excursion cap removed all rows from at least one suite.")
        write_filtered_summaries(
            summary=summary,
            straight_rows=straight_rows,
            mixed_turn_rows=mixed_turn_rows,
            output_dir=output_dir,
            excursion_bins_nmi=excursion_bins_nmi,
            max_excursion_nmi=args.max_excursion_nmi,
        )
    if args.write_rows:
        write_row_outputs(
            output_dir,
            straight_rows=straight_rows,
            mixed_turn_rows=mixed_turn_rows,
            compress_rows=args.compress_rows,
            combined_rows=args.combined_rows,
        )
    outputs = render_projection_diagnostic_plots(
        straight_rows=straight_rows,
        mixed_turn_rows=mixed_turn_rows,
        output_dir=output_dir,
        excursion_bins_nmi=excursion_bins_nmi,
    )
    for path in outputs:
        print(path)
    if args.archive_output is not None:
        archive_output_dir(output_dir, args.archive_output)
        print(args.archive_output)


def load_summary(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def load_projection_rows(input_dir: Path) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    straight_rows = load_row_files(input_dir, "straight_rows_lat_*.csv*")
    mixed_turn_rows = load_row_files(input_dir, "mixed_turn_rows_lat_*.csv*")
    if straight_rows or mixed_turn_rows:
        return straight_rows, mixed_turn_rows

    combined_rows = load_row_files(input_dir, "projection_rows.csv*")
    if not combined_rows:
        return [], []
    return split_combined_rows(combined_rows)


def load_row_files(input_dir: Path, pattern: str) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for path in sorted(input_dir.glob(pattern)):
        opener = gzip.open if path.suffix == ".gz" else Path.open
        with opener(path, "rt", encoding="utf-8", newline="") as handle:
            rows.extend(csv.DictReader(handle))
    return rows


def split_combined_rows(rows: list[dict[str, Any]]) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    straight_rows = [row for row in rows if row.get("suite_name") == "straight_sweep"]
    mixed_turn_rows = [row for row in rows if row.get("suite_name") == "mixed_turn_sweep"]
    return straight_rows, mixed_turn_rows


def filter_rows_by_excursion(rows: list[dict[str, Any]], max_excursion_nmi: float) -> list[dict[str, Any]]:
    return [row for row in rows if float(row["max_radial_excursion_nmi"]) <= max_excursion_nmi + 1e-9]


def write_filtered_summaries(
    *,
    summary: dict[str, Any],
    straight_rows: list[dict[str, Any]],
    mixed_turn_rows: list[dict[str, Any]],
    output_dir: Path,
    excursion_bins_nmi: tuple[float, ...],
    max_excursion_nmi: float,
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    config = dict(summary.get("config", {}))
    config["max_excursion_cap_nmi"] = float(max_excursion_nmi)
    payload = {
        "config": config,
        "straight_summary": [summarize_projection_rows(straight_rows, suite_name="straight_sweep")],
        "straight_by_excursion": summarize_projection_rows_by_excursion(
            straight_rows,
            suite_name="straight_sweep",
            excursion_bins_nmi=excursion_bins_nmi,
        ),
        "mixed_turn_summary": [summarize_projection_rows(mixed_turn_rows, suite_name="mixed_turn_sweep")],
        "mixed_turn_by_excursion": summarize_projection_rows_by_excursion(
            mixed_turn_rows,
            suite_name="mixed_turn_sweep",
            excursion_bins_nmi=excursion_bins_nmi,
        ),
        "mixed_turn_by_excursion_and_turn_count": build_turn_count_excursion_rows(
            mixed_turn_rows,
            excursion_bins_nmi=excursion_bins_nmi,
        ),
    }
    write_json(output_dir / "summary.json", payload)
    write_csv(output_dir / "straight_summary_by_latitude.csv", payload["straight_summary"])
    write_csv(output_dir / "straight_by_excursion.csv", payload["straight_by_excursion"])
    write_csv(output_dir / "mixed_turn_summary_by_latitude.csv", payload["mixed_turn_summary"])
    write_csv(output_dir / "mixed_turn_by_excursion.csv", payload["mixed_turn_by_excursion"])
    write_csv(
        output_dir / "mixed_turn_by_excursion_and_turn_count.csv",
        payload["mixed_turn_by_excursion_and_turn_count"],
    )


def write_row_outputs(
    output_dir: Path,
    *,
    straight_rows: list[dict[str, Any]],
    mixed_turn_rows: list[dict[str, Any]],
    compress_rows: bool,
    combined_rows: bool,
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    suffix = ".csv.gz" if compress_rows else ".csv"
    if combined_rows:
        write_csv(output_dir / f"projection_rows{suffix}", [*straight_rows, *mixed_turn_rows])
        return
    write_csv(output_dir / f"straight_rows{suffix}", straight_rows)
    write_csv(output_dir / f"mixed_turn_rows{suffix}", mixed_turn_rows)


def build_turn_count_excursion_rows(
    rows: list[dict[str, Any]],
    *,
    excursion_bins_nmi: tuple[float, ...],
) -> list[dict[str, Any]]:
    summaries: list[dict[str, Any]] = []
    for turn_count in (0, 1, 2):
        subset = [row for row in rows if int(row["turn_count"]) == turn_count]
        summaries.extend(
            summarize_projection_rows_by_excursion(
                subset,
                suite_name="mixed_turn_sweep",
                subset_name=f"turn_count_{turn_count}",
                excursion_bins_nmi=excursion_bins_nmi,
            )
        )
    return summaries


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        raise ValueError(f"Cannot write empty CSV to {path}")
    opener = gzip.open if path.suffix == ".gz" else Path.open
    fieldnames = fieldnames_for_rows(rows)
    with opener(path, "wt", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def fieldnames_for_rows(rows: list[dict[str, Any]]) -> list[str]:
    fieldnames: list[str] = []
    for row in rows:
        for key in row:
            if key not in fieldnames:
                fieldnames.append(key)
    return fieldnames


def archive_output_dir(output_dir: Path, archive_path: Path) -> None:
    archive_path.parent.mkdir(parents=True, exist_ok=True)
    archive_resolved = archive_path.resolve()
    with tarfile.open(archive_path, "w:gz") as archive:
        for path in sorted(output_dir.rglob("*")):
            if not path.is_file() or path.resolve() == archive_resolved:
                continue
            archive.add(path, arcname=path.relative_to(output_dir))


if __name__ == "__main__":
    main()
