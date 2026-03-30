#!/usr/bin/env python3
"""Render diagnostic plots from an existing deterministic projection sweep."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
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
        default=Path("paper_eval_outputs/campbell_projection_deterministic_51_full"),
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
    straight_rows = load_row_files(input_dir, "straight_rows_lat_*.csv")
    mixed_turn_rows = load_row_files(input_dir, "mixed_turn_rows_lat_*.csv")
    if not straight_rows or not mixed_turn_rows:
        raise FileNotFoundError(
            "Expected straight_rows_lat_*.csv and mixed_turn_rows_lat_*.csv in the input directory."
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
    outputs = render_projection_diagnostic_plots(
        straight_rows=straight_rows,
        mixed_turn_rows=mixed_turn_rows,
        output_dir=output_dir,
        excursion_bins_nmi=excursion_bins_nmi,
    )
    for path in outputs:
        print(path)


def load_summary(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def load_row_files(input_dir: Path, pattern: str) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for path in sorted(input_dir.glob(pattern)):
        with path.open("r", encoding="utf-8", newline="") as handle:
            rows.extend(csv.DictReader(handle))
    return rows


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
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


if __name__ == "__main__":
    main()
