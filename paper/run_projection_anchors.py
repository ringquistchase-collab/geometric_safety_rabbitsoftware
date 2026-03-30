#!/usr/bin/env python3
"""Compare projection-sensitivity summaries across local anchor choices."""

from __future__ import annotations

import argparse
import csv
from dataclasses import replace
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
    summarize_projection_rows,
    summarize_projection_rows_by_excursion,
)

ANCHOR_MODES = ("a", "b", "midpoint")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Compare projection error under different local anchor choices.")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("paper_eval_outputs/projection_anchor_mode_comparison"),
    )
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
    )
    parser.add_argument(
        "--horizons-s",
        type=float,
        nargs="+",
        default=list(DEFAULT_SWEEP_HORIZONS_S),
    )
    parser.add_argument(
        "--nominal-speeds-kt",
        type=float,
        nargs="+",
        default=list(DEFAULT_SWEEP_NOMINAL_SPEEDS_KT),
    )
    parser.add_argument(
        "--speed-diffs-kt",
        type=float,
        nargs="+",
        default=list(DEFAULT_SWEEP_SPEED_DIFFS_KT),
    )
    parser.add_argument("--straight-bearing-step-deg", type=float, default=DEFAULT_STRAIGHT_SWEEP_BEARING_STEP_DEG)
    parser.add_argument("--straight-heading-step-deg", type=float, default=DEFAULT_STRAIGHT_SWEEP_HEADING_STEP_DEG)
    parser.add_argument("--mixed-bearing-step-deg", type=float, default=DEFAULT_MIXED_SWEEP_BEARING_STEP_DEG)
    parser.add_argument("--mixed-heading-step-deg", type=float, default=DEFAULT_MIXED_SWEEP_HEADING_STEP_DEG)
    parser.add_argument(
        "--mixed-turn-angles-deg",
        type=float,
        nargs="+",
        default=list(DEFAULT_MIXED_SWEEP_TURN_ANGLES_DEG),
    )
    parser.add_argument(
        "--excursion-bins-nmi",
        type=float,
        nargs="+",
        default=list(DEFAULT_EXCURSION_BINS_NMI),
    )
    parser.add_argument(
        "--cap-excursion-nmi",
        type=float,
        default=100.0,
        help="Optional radial-excursion cap for postprocessed summaries.",
    )
    parser.add_argument("--quick", action="store_true", help="Run a smoke-sized comparison.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    base_config = (
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

    comparison_rows: list[dict[str, Any]] = []
    by_excursion_rows: list[dict[str, Any]] = []
    payload: dict[str, Any] = {
        "base_config": {
            **base_config.__dict__,
            "cap_excursion_nmi": args.cap_excursion_nmi,
        },
        "modes": {},
    }
    for anchor_mode in ANCHOR_MODES:
        config = replace(base_config, anchor_mode=anchor_mode)
        result = run_projection_deterministic_sweep(config)
        straight_rows = cap_rows(result["straight_rows_by_latitude"], args.cap_excursion_nmi)
        mixed_turn_rows = cap_rows(result["mixed_turn_rows_by_latitude"], args.cap_excursion_nmi)
        straight_summary = summarize_capped_suite(
            straight_rows,
            suite_name="straight_sweep",
            cap_excursion_nmi=args.cap_excursion_nmi,
        )
        mixed_turn_summary = summarize_capped_suite(
            mixed_turn_rows,
            suite_name="mixed_turn_sweep",
            cap_excursion_nmi=args.cap_excursion_nmi,
        )
        straight_by_excursion = summarize_projection_rows_by_excursion(
            flatten_rows(straight_rows),
            suite_name="straight_sweep",
            excursion_bins_nmi=config.excursion_bins_nmi,
            subset_name=f"cap_{args.cap_excursion_nmi:g}_nmi",
        )
        mixed_turn_by_excursion = summarize_projection_rows_by_excursion(
            flatten_rows(mixed_turn_rows),
            suite_name="mixed_turn_sweep",
            excursion_bins_nmi=config.excursion_bins_nmi,
            subset_name=f"cap_{args.cap_excursion_nmi:g}_nmi",
        )
        payload["modes"][anchor_mode] = {
            "config": config.__dict__,
            "straight_summary": straight_summary,
            "mixed_turn_summary": mixed_turn_summary,
            "straight_by_excursion": straight_by_excursion,
            "mixed_turn_by_excursion": mixed_turn_by_excursion,
        }
        comparison_rows.extend([straight_summary, mixed_turn_summary])
        by_excursion_rows.extend(straight_by_excursion)
        by_excursion_rows.extend(mixed_turn_by_excursion)

    write_json(output_dir / "summary.json", payload)
    write_csv(output_dir / "anchor_mode_comparison.csv", comparison_rows)
    write_csv(output_dir / "anchor_mode_by_excursion.csv", by_excursion_rows)


def cap_rows(
    rows_by_latitude: dict[float, list[dict[str, Any]]],
    cap_excursion_nmi: float,
) -> dict[float, list[dict[str, Any]]]:
    """Keep only encounters with radial excursion below the requested cap."""
    return {
        latitude_deg: [row for row in rows if float(row["max_radial_excursion_nmi"]) <= cap_excursion_nmi]
        for latitude_deg, rows in rows_by_latitude.items()
    }


def flatten_rows(rows_by_latitude: dict[float, list[dict[str, Any]]]) -> list[dict[str, Any]]:
    """Flatten latitude-partitioned rows into one list."""
    flat_rows: list[dict[str, Any]] = []
    for rows in rows_by_latitude.values():
        flat_rows.extend(rows)
    return flat_rows


def summarize_capped_suite(
    rows_by_latitude: dict[float, list[dict[str, Any]]],
    *,
    suite_name: str,
    cap_excursion_nmi: float,
) -> dict[str, Any]:
    """Summarize one capped suite over all configured latitudes."""
    rows = flatten_rows(rows_by_latitude)
    if not rows:
        raise ValueError(f"No rows remain in {suite_name} after applying the excursion cap.")
    summary = summarize_projection_rows(
        rows,
        suite_name=suite_name,
        subset_name=f"cap_{cap_excursion_nmi:g}_nmi",
    )
    summary["cap_excursion_nmi"] = float(cap_excursion_nmi)
    return summary


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
