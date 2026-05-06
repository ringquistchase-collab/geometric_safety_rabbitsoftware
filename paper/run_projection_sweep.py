#!/usr/bin/env python3
"""Run deterministic projection-sensitivity sweeps for the manuscript."""

from __future__ import annotations

import argparse
import csv
import gzip
import json
from pathlib import Path
from typing import Any

from paper.projection_sanity import (
    DEFAULT_EXCURSION_BINS_NMI,
    DEFAULT_MIXED_SWEEP_BEARING_STEP_DEG,
    DEFAULT_MIXED_SWEEP_HEADING_STEP_DEG,
    DEFAULT_MIXED_SWEEP_TURN_ANGLES_DEG,
    DEFAULT_STRAIGHT_SWEEP_BEARING_STEP_DEG,
    DEFAULT_STRAIGHT_SWEEP_HEADING_STEP_DEG,
    DEFAULT_SWEEP_HORIZONS_S,
    DEFAULT_SWEEP_INITIAL_RANGES_NMI,
    DEFAULT_SWEEP_LATITUDES_DEG,
    DEFAULT_SWEEP_NOMINAL_SPEEDS_KT,
    DEFAULT_SWEEP_SPEED_DIFFS_KT,
    ProjectionDeterministicSweepConfig,
    run_projection_deterministic_sweep,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run deterministic projection-sensitivity sweeps.")
    parser.add_argument("--output-dir", type=Path, default=Path("paper_eval_outputs/projection_deterministic_sweep"))
    parser.add_argument(
        "--latitudes",
        type=float,
        nargs="+",
        default=list(DEFAULT_SWEEP_LATITUDES_DEG),
        help="Anchor latitudes in degrees.",
    )
    parser.add_argument("--anchor-lon", type=float, default=0.0, help="Shared anchor longitude in degrees.")
    parser.add_argument("--time-step-s", type=float, default=1.0)
    parser.add_argument("--n-jobs", type=int, default=1)
    parser.add_argument("--chunk-size", type=int, default=256)
    parser.add_argument(
        "--initial-ranges-nmi",
        type=float,
        nargs="+",
        default=list(DEFAULT_SWEEP_INITIAL_RANGES_NMI),
        help="Initial separation radii for the deterministic sweeps.",
    )
    parser.add_argument(
        "--horizons-s",
        type=float,
        nargs="+",
        default=list(DEFAULT_SWEEP_HORIZONS_S),
        help="Projection horizons in seconds.",
    )
    parser.add_argument(
        "--nominal-speeds-kt",
        type=float,
        nargs="+",
        default=list(DEFAULT_SWEEP_NOMINAL_SPEEDS_KT),
        help="Nominal speeds used in the sweeps.",
    )
    parser.add_argument(
        "--speed-diffs-kt",
        type=float,
        nargs="+",
        default=list(DEFAULT_SWEEP_SPEED_DIFFS_KT),
        help="Speed-uncertainty half-widths used in the sweeps.",
    )
    parser.add_argument(
        "--straight-bearing-step-deg",
        type=float,
        default=DEFAULT_STRAIGHT_SWEEP_BEARING_STEP_DEG,
    )
    parser.add_argument(
        "--straight-heading-step-deg",
        type=float,
        default=DEFAULT_STRAIGHT_SWEEP_HEADING_STEP_DEG,
    )
    parser.add_argument(
        "--mixed-bearing-step-deg",
        type=float,
        default=DEFAULT_MIXED_SWEEP_BEARING_STEP_DEG,
    )
    parser.add_argument(
        "--mixed-heading-step-deg",
        type=float,
        default=DEFAULT_MIXED_SWEEP_HEADING_STEP_DEG,
    )
    parser.add_argument(
        "--mixed-turn-angles-deg",
        type=float,
        nargs="+",
        default=list(DEFAULT_MIXED_SWEEP_TURN_ANGLES_DEG),
        help="Signed turn-angle options for the deterministic mixed-turn sweep.",
    )
    parser.add_argument(
        "--excursion-bins-nmi",
        type=float,
        nargs="+",
        default=list(DEFAULT_EXCURSION_BINS_NMI),
        help="Excursion-bin upper edges in NMI.",
    )
    parser.add_argument("--quick", action="store_true", help="Run a smoke-sized deterministic sweep.")
    parser.add_argument("--skip-rows", action="store_true", help="Skip writing row-level CSV outputs.")
    parser.add_argument("--compress-rows", action="store_true", help="Write row-level CSV outputs as gzip files.")
    parser.add_argument(
        "--combined-rows",
        action="store_true",
        help="Write one combined row-level CSV instead of separate straight and mixed-turn row files.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config = (
        ProjectionDeterministicSweepConfig.quick_defaults()
        if args.quick
        else ProjectionDeterministicSweepConfig(
            latitudes_deg=tuple(args.latitudes),
            anchor_lon_deg=args.anchor_lon,
            time_step_s=args.time_step_s,
            n_jobs=args.n_jobs,
            chunk_size=args.chunk_size,
            initial_ranges_nmi=tuple(args.initial_ranges_nmi),
            horizons_s=tuple(args.horizons_s),
            straight_relative_bearing_step_deg=args.straight_bearing_step_deg,
            straight_heading_step_deg=args.straight_heading_step_deg,
            mixed_relative_bearing_step_deg=args.mixed_bearing_step_deg,
            mixed_heading_step_deg=args.mixed_heading_step_deg,
            nominal_speeds_kt=tuple(args.nominal_speeds_kt),
            speed_diffs_kt=tuple(args.speed_diffs_kt),
            mixed_turn_angles_deg=tuple(args.mixed_turn_angles_deg),
            excursion_bins_nmi=tuple(args.excursion_bins_nmi),
        )
    )

    output_dir = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    result = run_projection_deterministic_sweep(config)
    write_outputs(
        result,
        output_dir,
        write_rows=not args.skip_rows,
        compress_rows=args.compress_rows,
        combined_rows=args.combined_rows,
    )


def write_outputs(
    result: dict[str, Any],
    output_dir: Path,
    *,
    write_rows: bool,
    compress_rows: bool,
    combined_rows: bool,
) -> None:
    payload = {
        "config": result["config"],
        "straight_summary": result["straight_summary"],
        "straight_by_excursion": result["straight_by_excursion"],
        "mixed_turn_summary": result["mixed_turn_summary"],
        "mixed_turn_by_excursion": result["mixed_turn_by_excursion"],
        "mixed_turn_by_excursion_and_turn_count": result["mixed_turn_by_excursion_and_turn_count"],
    }
    write_json(output_dir / "summary.json", payload)
    write_csv(output_dir / "straight_summary_by_latitude.csv", result["straight_summary"])
    write_csv(output_dir / "straight_by_excursion.csv", result["straight_by_excursion"])
    write_csv(output_dir / "mixed_turn_summary_by_latitude.csv", result["mixed_turn_summary"])
    write_csv(output_dir / "mixed_turn_by_excursion.csv", result["mixed_turn_by_excursion"])
    write_csv(
        output_dir / "mixed_turn_by_excursion_and_turn_count.csv",
        result["mixed_turn_by_excursion_and_turn_count"],
    )
    if write_rows:
        write_row_outputs(
            output_dir,
            straight_rows_by_latitude=result["straight_rows_by_latitude"],
            mixed_turn_rows_by_latitude=result["mixed_turn_rows_by_latitude"],
            compress_rows=compress_rows,
            combined_rows=combined_rows,
        )


def write_row_outputs(
    output_dir: Path,
    *,
    straight_rows_by_latitude: dict[float, list[dict[str, Any]]],
    mixed_turn_rows_by_latitude: dict[float, list[dict[str, Any]]],
    compress_rows: bool,
    combined_rows: bool,
) -> None:
    suffix = ".csv.gz" if compress_rows else ".csv"
    if combined_rows:
        rows = [
            row
            for rows_by_latitude in (straight_rows_by_latitude, mixed_turn_rows_by_latitude)
            for latitude_rows in rows_by_latitude.values()
            for row in latitude_rows
        ]
        write_csv(output_dir / f"projection_rows{suffix}", rows)
        return

    for latitude_deg, rows in straight_rows_by_latitude.items():
        write_csv(output_dir / f"straight_rows_lat_{format_latitude_tag(latitude_deg)}{suffix}", rows)
    for latitude_deg, rows in mixed_turn_rows_by_latitude.items():
        write_csv(output_dir / f"mixed_turn_rows_lat_{format_latitude_tag(latitude_deg)}{suffix}", rows)


def format_latitude_tag(latitude_deg: float) -> str:
    return f"{latitude_deg:+05.1f}".replace(".", "p").replace("+", "plus_").replace("-", "minus_")


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


if __name__ == "__main__":
    main()
