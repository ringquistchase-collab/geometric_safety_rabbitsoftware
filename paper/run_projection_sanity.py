#!/usr/bin/env python3
"""Run the projection sanity-check experiments for the manuscript."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt

from paper.projection_sanity import (
    DEFAULT_PROJECTION_LATITUDES_DEG,
    ProjectionSanityConfig,
    run_projection_sanity_check,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the projection sanity-check experiments.")
    parser.add_argument("--output-dir", type=Path, default=Path("paper_eval_outputs/projection_sanity"))
    parser.add_argument(
        "--aggregate-run-dirs",
        type=Path,
        nargs="+",
        default=None,
        help="Aggregate summaries from existing projection-sanity run directories.",
    )
    parser.add_argument(
        "--latitudes",
        type=float,
        nargs="+",
        default=list(DEFAULT_PROJECTION_LATITUDES_DEG),
        help="Anchor latitudes in degrees.",
    )
    parser.add_argument("--anchor-lon", type=float, default=0.0, help="Shared anchor longitude in degrees.")
    parser.add_argument("--straight-n", type=int, default=20_000)
    parser.add_argument("--mixed-turn-n", type=int, default=20_000)
    parser.add_argument("--seed", type=int, default=20260327)
    parser.add_argument("--time-step-s", type=float, default=1.0)
    parser.add_argument("--n-jobs", type=int, default=1)
    parser.add_argument("--chunk-size", type=int, default=256)
    parser.add_argument("--stress-bearing-step-deg", type=float, default=45.0)
    parser.add_argument("--stress-heading-step-deg", type=float, default=90.0)
    parser.add_argument("--quick", action="store_true", help="Run a smoke-sized projection sanity check.")
    parser.add_argument("--skip-rows", action="store_true", help="Skip writing per-encounter row CSV files.")
    parser.add_argument("--skip-plots", action="store_true", help="Skip writing the summary plot.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.aggregate_run_dirs:
        aggregate_existing_projection_runs(args.aggregate_run_dirs, args.output_dir, write_plot=not args.skip_plots)
        return

    config = (
        ProjectionSanityConfig.quick_defaults()
        if args.quick
        else ProjectionSanityConfig(
            latitudes_deg=tuple(args.latitudes),
            anchor_lon_deg=args.anchor_lon,
            straight_n=args.straight_n,
            mixed_turn_n=args.mixed_turn_n,
            seed=args.seed,
            time_step_s=args.time_step_s,
            n_jobs=args.n_jobs,
            chunk_size=args.chunk_size,
            stress_bearing_step_deg=args.stress_bearing_step_deg,
            stress_heading_step_deg=args.stress_heading_step_deg,
        )
    )

    output_dir = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    result = run_projection_sanity_check(config)
    write_projection_outputs(result, output_dir, write_rows=not args.skip_rows, write_plot=not args.skip_plots)


def aggregate_existing_projection_runs(run_dirs: list[Path], output_dir: Path, *, write_plot: bool) -> None:
    straight_rows: list[dict[str, Any]] = []
    mixed_turn_rows: list[dict[str, Any]] = []
    mixed_turn_turn_count_rows: list[dict[str, Any]] = []
    stress_rows: list[dict[str, Any]] = []
    configs = []

    for run_dir in run_dirs:
        summary_path = run_dir / "summary.json"
        if not summary_path.exists():
            raise FileNotFoundError(f"Expected summary.json in {run_dir}")
        summary = read_json(summary_path)
        configs.append(summary["config"])
        straight_rows.extend(summary["straight"])
        mixed_turn_rows.extend(summary["mixed_turn"])
        for turn_count_rows in summary["mixed_turn_turn_count"].values():
            mixed_turn_turn_count_rows.extend(turn_count_rows)
        stress_rows.extend(summary["stress"])

    validate_projection_configs(configs)
    straight_rows.sort(key=lambda row: float(row["anchor_lat_deg"]))
    mixed_turn_rows.sort(key=lambda row: float(row["anchor_lat_deg"]))
    mixed_turn_turn_count_rows.sort(key=lambda row: (float(row["anchor_lat_deg"]), str(row["subset_name"])))
    stress_rows.sort(key=lambda row: float(row["anchor_lat_deg"]))

    output_dir.mkdir(parents=True, exist_ok=True)
    combined = {
        "config": merged_projection_config(configs, straight_rows),
        "straight": straight_rows,
        "mixed_turn": mixed_turn_rows,
        "mixed_turn_turn_count": regroup_turn_count_rows(mixed_turn_turn_count_rows),
        "stress": stress_rows,
    }
    write_json(output_dir / "summary.json", combined)
    write_csv(output_dir / "straight_summary_by_latitude.csv", straight_rows)
    write_csv(output_dir / "mixed_turn_summary_by_latitude.csv", mixed_turn_rows)
    write_csv(output_dir / "mixed_turn_turn_count_summary.csv", mixed_turn_turn_count_rows)
    write_csv(output_dir / "stress_summary_by_latitude.csv", stress_rows)
    if write_plot:
        plot_projection_summary(
            output_dir / "projection_error_by_latitude.pdf",
            straight_rows,
            mixed_turn_rows,
            stress_rows,
        )


def write_projection_outputs(
    result: dict[str, Any],
    output_dir: Path,
    *,
    write_rows: bool,
    write_plot: bool,
) -> None:
    payload = {
        "config": result["config"],
        "straight": result["straight"],
        "mixed_turn": result["mixed_turn"],
        "mixed_turn_turn_count": result["mixed_turn_turn_count"],
        "stress": result["stress"],
    }
    write_json(output_dir / "summary.json", payload)
    write_csv(output_dir / "straight_summary_by_latitude.csv", result["straight"])
    write_csv(output_dir / "mixed_turn_summary_by_latitude.csv", result["mixed_turn"])
    mixed_turn_turn_count_rows = flatten_turn_count_rows(result["mixed_turn_turn_count"])
    write_csv(output_dir / "mixed_turn_turn_count_summary.csv", mixed_turn_turn_count_rows)
    write_csv(output_dir / "stress_summary_by_latitude.csv", result["stress"])
    if write_rows:
        for latitude_deg, rows in result["straight_rows_by_latitude"].items():
            write_csv(output_dir / f"straight_rows_lat_{format_latitude_tag(latitude_deg)}.csv", rows)
        for latitude_deg, rows in result["mixed_turn_rows_by_latitude"].items():
            write_csv(output_dir / f"mixed_turn_rows_lat_{format_latitude_tag(latitude_deg)}.csv", rows)
        for latitude_deg, rows in result["stress_rows_by_latitude"].items():
            write_csv(output_dir / f"stress_rows_lat_{format_latitude_tag(latitude_deg)}.csv", rows)
    if write_plot:
        plot_projection_summary(
            output_dir / "projection_error_by_latitude.pdf",
            result["straight"],
            result["mixed_turn"],
            result["stress"],
        )


def validate_projection_configs(configs: list[dict[str, Any]]) -> None:
    if not configs:
        raise ValueError("At least one projection run summary is required.")
    reference = {key: value for key, value in configs[0].items() if key != "latitudes_deg"}
    for candidate in configs[1:]:
        trimmed = {key: value for key, value in candidate.items() if key != "latitudes_deg"}
        if trimmed != reference:
            raise ValueError("Projection run configs do not match across aggregated directories.")


def merged_projection_config(
    configs: list[dict[str, Any]],
    summary_rows: list[dict[str, Any]],
) -> dict[str, Any]:
    merged = dict(configs[0])
    merged["latitudes_deg"] = [float(row["anchor_lat_deg"]) for row in summary_rows]
    return merged


def plot_projection_summary(
    path: Path,
    straight_rows: list[dict[str, Any]],
    mixed_turn_rows: list[dict[str, Any]],
    stress_rows: list[dict[str, Any]],
) -> None:
    fig, axes = plt.subplots(1, 3, figsize=(15.0, 4.5), sharex=True)

    plot_projection_panel(
        axes[0],
        straight_rows,
        title="Straight-heading suite",
    )
    plot_projection_panel(
        axes[1],
        mixed_turn_rows,
        title="Representative mixed-turn suite",
    )
    plot_projection_panel(
        axes[2],
        stress_rows,
        title="Deterministic stress suite",
    )

    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path)
    plt.close(fig)


def plot_projection_panel(ax: Any, rows: list[dict[str, Any]], *, title: str) -> None:
    latitudes = [float(row["anchor_lat_deg"]) for row in rows]
    p95_errors_nmi = [float(row["min_separation_error_p95_m"]) / 1852.0 for row in rows]
    max_errors_nmi = [float(row["min_separation_error_max_m"]) / 1852.0 for row in rows]
    ax.plot(latitudes, p95_errors_nmi, marker="o", linewidth=2.0, label="p95 min-separation error")
    ax.plot(latitudes, max_errors_nmi, marker="s", linewidth=2.0, label="max min-separation error")
    ax.set_title(title, fontsize=13)
    ax.set_xlabel("Anchor latitude (deg)", fontsize=12)
    ax.set_ylabel("Error (NMI)", fontsize=12)
    ax.grid(True, color="#d9d9d9", linewidth=0.8, alpha=0.8)
    ax.tick_params(axis="both", labelsize=11)
    ax.legend(loc="upper left", fontsize=10, frameon=True, framealpha=0.95)


def flatten_turn_count_rows(turn_count_rows: dict[int, list[dict[str, Any]]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for turn_count in sorted(turn_count_rows):
        rows.extend(turn_count_rows[turn_count])
    rows.sort(key=lambda row: (float(row["anchor_lat_deg"]), str(row["subset_name"])))
    return rows


def regroup_turn_count_rows(rows: list[dict[str, Any]]) -> dict[int, list[dict[str, Any]]]:
    regrouped: dict[int, list[dict[str, Any]]] = {0: [], 1: [], 2: []}
    for row in rows:
        subset_name = str(row["subset_name"])
        turn_count = int(subset_name.removeprefix("turn_count_"))
        regrouped[turn_count].append(row)
    for value in regrouped.values():
        value.sort(key=lambda row: float(row["anchor_lat_deg"]))
    return regrouped


def format_latitude_tag(latitude_deg: float) -> str:
    return f"{latitude_deg:+05.1f}".replace(".", "p").replace("+", "plus_").replace("-", "minus_")


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        raise ValueError(f"Cannot write empty CSV to {path}")
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


if __name__ == "__main__":
    main()
