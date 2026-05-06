"""Render diagnostic plots for projection-sensitivity sweeps."""

# ruff: noqa: I001

from __future__ import annotations

import math
import os
from itertools import pairwise
from pathlib import Path
from typing import Any

os.environ.setdefault("MPLBACKEND", "Agg")

import matplotlib.pyplot as plt
import numpy as np

from paper.projection_sanity import DEFAULT_EXCURSION_BINS_NMI

AMBIGUITY_BAND_NMI = 0.05
PLOT_FIGSIZE_WIDE = (11.5, 4.6)
PLOT_FIGSIZE_TALL = (11.0, 8.0)
MIN_LOG_ERROR_NMI = 1e-6
MANUSCRIPT_FONT_FAMILY = ["Times New Roman", "Times", "Nimbus Roman", "Liberation Serif", "serif"]


def render_projection_diagnostic_plots(
    *,
    straight_rows: list[dict[str, Any]],
    mixed_turn_rows: list[dict[str, Any]],
    output_dir: Path,
    excursion_bins_nmi: tuple[float, ...] = DEFAULT_EXCURSION_BINS_NMI,
) -> list[Path]:
    """Render a standard diagnostic plot set for deterministic projection sweeps."""
    configure_manuscript_plot_style()
    output_dir.mkdir(parents=True, exist_ok=True)
    outputs = [
        output_dir / "projection_min_error_vs_excursion.pdf",
        output_dir / "projection_error_by_range_and_horizon.pdf",
        output_dir / "projection_orientation_disagreement.pdf",
        output_dir / "projection_disagreement_by_excursion.pdf",
    ]
    plot_projection_error_vs_excursion(
        straight_rows=straight_rows,
        mixed_turn_rows=mixed_turn_rows,
        output_path=outputs[0],
    )
    plot_projection_error_by_range_and_horizon(
        straight_rows=straight_rows,
        mixed_turn_rows=mixed_turn_rows,
        output_path=outputs[1],
    )
    plot_projection_orientation_disagreement(
        straight_rows=straight_rows,
        mixed_turn_rows=mixed_turn_rows,
        output_path=outputs[2],
    )
    plot_projection_disagreement_by_excursion(
        straight_rows=straight_rows,
        mixed_turn_rows=mixed_turn_rows,
        excursion_bins_nmi=excursion_bins_nmi,
        output_path=outputs[3],
    )
    return outputs


def configure_manuscript_plot_style() -> None:
    """Use journal-friendly fonts and embed TrueType text in generated PDFs."""
    plt.rcParams.update(
        {
            "font.family": "serif",
            "font.serif": MANUSCRIPT_FONT_FAMILY,
            "font.size": 12.0,
            "mathtext.fontset": "stix",
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
            "axes.unicode_minus": False,
        }
    )


def plot_projection_error_vs_excursion(
    *,
    straight_rows: list[dict[str, Any]],
    mixed_turn_rows: list[dict[str, Any]],
    output_path: Path,
) -> None:
    """Plot minimum-separation error against radial excursion."""
    fig, axes = plt.subplots(1, 2, figsize=PLOT_FIGSIZE_WIDE, sharey=True, constrained_layout=True)
    min_excursion_nmi = min(
        float(np.min(_array(straight_rows, "max_radial_excursion_nmi"))),
        float(np.min(_array(mixed_turn_rows, "max_radial_excursion_nmi"))),
    )
    max_excursion_nmi = max(
        float(np.max(_array(straight_rows, "max_radial_excursion_nmi"))),
        float(np.max(_array(mixed_turn_rows, "max_radial_excursion_nmi"))),
    )
    x_lower_nmi = 5.0 * math.floor(min_excursion_nmi / 5.0)
    x_upper_nmi = max(100.0, math.ceil(max_excursion_nmi / 5.0) * 5.0)
    excursion_edges_nmi = _line_bin_edges(x_lower_nmi, x_upper_nmi, 5.0)

    for ax, rows, title in (
        (axes[0], straight_rows, "Straight-heading sweep"),
        (axes[1], mixed_turn_rows, "Mixed-turn sweep"),
    ):
        centers_nmi, p25_errors_nmi, median_errors_nmi, p75_errors_nmi, p95_errors_nmi = _binned_percentiles(
            _array(rows, "max_radial_excursion_nmi"),
            _array(rows, "worst_min_separation_error_nmi"),
            excursion_edges_nmi,
            (25.0, 50.0, 75.0, 95.0),
        )
        ax.fill_between(
            centers_nmi,
            np.maximum(p25_errors_nmi, MIN_LOG_ERROR_NMI),
            np.maximum(p75_errors_nmi, MIN_LOG_ERROR_NMI),
            color="#9ecae1",
            alpha=0.45,
            linewidth=0.0,
            label="Interquartile range",
        )
        ax.plot(centers_nmi, median_errors_nmi, color="#e66101", linewidth=2.0, label="Median")
        ax.plot(centers_nmi, p95_errors_nmi, color="#5e3c99", linewidth=2.0, linestyle="--", label="p95")
        ax.axhline(
            AMBIGUITY_BAND_NMI,
            color="black",
            linewidth=1.2,
            linestyle=":",
            label="0.05 NMI ambiguity band",
        )
        ax.set_title(title, fontsize=14)
        ax.set_xlabel("Maximum radial excursion from projection origin (NMI)", fontsize=12.5)
        ax.set_yscale("log")
        ax.set_xlim(x_lower_nmi, x_upper_nmi)
        ax.grid(alpha=0.25, linewidth=0.6)
        ax.tick_params(labelsize=11.5)
        ax.legend(loc="lower left", fontsize=10.5, frameon=True)
    axes[0].set_ylabel("Worst minimum-separation error (NMI)", fontsize=12.5)
    save_figure(fig, output_path)


def plot_projection_error_by_range_and_horizon(
    *,
    straight_rows: list[dict[str, Any]],
    mixed_turn_rows: list[dict[str, Any]],
    output_path: Path,
) -> None:
    """Plot how error size varies with initial range and horizon."""
    fig, axes = plt.subplots(2, 2, figsize=PLOT_FIGSIZE_TALL, constrained_layout=True)
    row_specs = (
        ("Straight-heading sweep", straight_rows),
        ("Mixed-turn sweep", mixed_turn_rows),
    )
    heatmap_specs = (
        ("p95 minimum-separation error (NMI)", _p95_min_error_nmi),
        ("Rate above 0.05 NMI ambiguity band", _rate_above_ambiguity),
    )

    value_matrices = []
    grids = []
    for _, rows in row_specs:
        initial_ranges_nmi = sorted({float(row["initial_range_nmi"]) for row in rows})
        horizons_min = sorted({float(row["projection_time_s"]) / 60.0 for row in rows})
        grids.append((initial_ranges_nmi, horizons_min))
        grouped = _group_rows(
            rows,
            lambda row: (
                float(row["initial_range_nmi"]),
                float(row["projection_time_s"]) / 60.0,
            ),
        )
        value_matrices.append(
            tuple(
                _build_heatmap_matrix(grouped, initial_ranges_nmi, horizons_min, reducer)
                for _, reducer in heatmap_specs
            )
        )

    error_vmax = max(np.nanmax(mats[0]) for mats in value_matrices)
    rate_vmax = max(np.nanmax(mats[1]) for mats in value_matrices)

    for row_index, ((title, _rows), (initial_ranges_nmi, horizons_min), matrices) in enumerate(
        zip(row_specs, grids, value_matrices, strict=True)
    ):
        for col_index, ((panel_title, _reducer), matrix) in enumerate(zip(heatmap_specs, matrices, strict=True)):
            ax = axes[row_index, col_index]
            vmax = error_vmax if col_index == 0 else rate_vmax
            image = ax.imshow(
                matrix,
                origin="lower",
                aspect="auto",
                cmap="viridis",
                vmin=0.0,
                vmax=vmax,
            )
            ax.set_title(f"{title}\n{panel_title}", fontsize=11)
            ax.set_xticks(range(len(horizons_min)))
            ax.set_xticklabels([f"{value:.0f}" for value in horizons_min], fontsize=10)
            ax.set_yticks(range(len(initial_ranges_nmi)))
            ax.set_yticklabels([f"{value:.0f}" for value in initial_ranges_nmi], fontsize=10)
            ax.set_xlabel("Projection horizon (min)", fontsize=10)
            ax.set_ylabel("Initial separation radius (NMI)", fontsize=10)
            colorbar = fig.colorbar(image, ax=ax, pad=0.02)
            if col_index == 0:
                colorbar.set_label("NMI", fontsize=10)
            else:
                colorbar.set_label("Fraction of encounters", fontsize=10)
            colorbar.ax.tick_params(labelsize=9)
    save_figure(fig, output_path)


def plot_projection_orientation_disagreement(
    *,
    straight_rows: list[dict[str, Any]],
    mixed_turn_rows: list[dict[str, Any]],
    output_path: Path,
) -> None:
    """Plot disagreement rates by initial orientation variables."""
    fig, axes = plt.subplots(1, 2, figsize=PLOT_FIGSIZE_WIDE, constrained_layout=True)
    vmax = max(
        _orientation_rate_max(straight_rows),
        _orientation_rate_max(mixed_turn_rows),
    )
    for ax, rows, title in (
        (axes[0], straight_rows, "Straight-heading sweep"),
        (axes[1], mixed_turn_rows, "Mixed-turn sweep"),
    ):
        bearing_values = sorted({_relative_bearing_deg(row) for row in rows})
        heading_values = sorted({_heading_difference_deg(row) for row in rows})
        grouped = _group_rows(rows, lambda row: (_heading_difference_deg(row), _relative_bearing_deg(row)))
        matrix = _build_heatmap_matrix(grouped, heading_values, bearing_values, _threshold_disagreement_rate)
        image = ax.imshow(
            matrix,
            origin="lower",
            aspect="auto",
            cmap="magma",
            vmin=0.0,
            vmax=vmax,
        )
        ax.set_title(title, fontsize=12)
        ax.set_xticks(range(len(bearing_values)))
        ax.set_xticklabels([f"{value:.0f}" for value in bearing_values], fontsize=9)
        ax.set_yticks(range(len(heading_values)))
        ax.set_yticklabels([f"{value:.0f}" for value in heading_values], fontsize=9)
        ax.set_xlabel("Initial relative bearing of aircraft B (deg)", fontsize=10)
        ax.set_ylabel("Initial heading difference B-A (deg)", fontsize=10)
        colorbar = fig.colorbar(image, ax=ax, pad=0.02)
        colorbar.set_label("Threshold-disagreement rate", fontsize=10)
        colorbar.ax.tick_params(labelsize=9)
    save_figure(fig, output_path)


def plot_projection_disagreement_by_excursion(
    *,
    straight_rows: list[dict[str, Any]],
    mixed_turn_rows: list[dict[str, Any]],
    excursion_bins_nmi: tuple[float, ...],
    output_path: Path,
) -> None:
    """Plot disagreement rates against excursion, including turn-count splits."""
    fig, axes = plt.subplots(1, 2, figsize=PLOT_FIGSIZE_WIDE, sharey=True, constrained_layout=True)
    min_excursion_nmi = min(
        float(np.min(_array(straight_rows, "max_radial_excursion_nmi"))),
        float(np.min(_array(mixed_turn_rows, "max_radial_excursion_nmi"))),
    )
    x_lower_nmi = 5.0 * math.floor(min_excursion_nmi / 5.0)
    bounds_nmi = (0.0, *excursion_bins_nmi)
    centers_nmi = np.asarray(
        [(left + right) * 0.5 for left, right in pairwise(bounds_nmi)],
        dtype=np.float64,
    )

    straight_threshold = _binned_rate(straight_rows, bounds_nmi, _threshold_disagreement, None)
    mixed_threshold = _binned_rate(mixed_turn_rows, bounds_nmi, _threshold_disagreement, None)
    straight_outside = _binned_rate(straight_rows, bounds_nmi, _outside_ambiguity_threshold_disagreement, None)
    mixed_outside = _binned_rate(mixed_turn_rows, bounds_nmi, _outside_ambiguity_threshold_disagreement, None)

    axes[0].plot(centers_nmi, straight_threshold, marker="o", linewidth=2.0, color="#1f77b4", label="Straight")
    axes[0].plot(centers_nmi, mixed_threshold, marker="s", linewidth=2.0, color="#d62728", label="Mixed-turn")
    axes[0].plot(
        centers_nmi,
        straight_outside,
        linewidth=1.8,
        color="#1f77b4",
        linestyle="--",
        label="Straight, outside ambiguity",
    )
    axes[0].plot(
        centers_nmi,
        mixed_outside,
        linewidth=1.8,
        color="#d62728",
        linestyle="--",
        label="Mixed-turn, outside ambiguity",
    )
    axes[0].set_title("Overall rates by excursion bin", fontsize=12)
    axes[0].set_xlabel("Maximum radial excursion from projection origin (NMI)", fontsize=11)
    axes[0].set_ylabel("Encounter fraction", fontsize=11)
    axes[0].set_xlim(x_lower_nmi, float(excursion_bins_nmi[-1]))
    axes[0].grid(alpha=0.25, linewidth=0.6)
    axes[0].tick_params(labelsize=10)
    axes[0].legend(loc="lower left", fontsize=9, frameon=True)

    for turn_count, color in ((0, "#4daf4a"), (1, "#ff7f00"), (2, "#984ea3")):
        threshold_rates = _binned_rate(mixed_turn_rows, bounds_nmi, _threshold_disagreement, turn_count)
        outside_rates = _binned_rate(mixed_turn_rows, bounds_nmi, _outside_ambiguity_threshold_disagreement, turn_count)
        axes[1].plot(
            centers_nmi,
            threshold_rates,
            marker="o",
            linewidth=2.0,
            color=color,
            label=f"turn_count = {turn_count}",
        )
        axes[1].plot(
            centers_nmi,
            outside_rates,
            linewidth=1.8,
            color=color,
            linestyle="--",
            label=f"turn_count = {turn_count}, outside ambiguity",
        )
    axes[1].set_title("Mixed-turn rates by excursion and turn count", fontsize=12)
    axes[1].set_xlabel("Maximum radial excursion from projection origin (NMI)", fontsize=11)
    axes[1].set_xlim(x_lower_nmi, float(excursion_bins_nmi[-1]))
    axes[1].grid(alpha=0.25, linewidth=0.6)
    axes[1].tick_params(labelsize=10)
    axes[1].legend(loc="lower left", fontsize=9, frameon=True)
    save_figure(fig, output_path)


def save_figure(fig: Any, output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, bbox_inches="tight")
    plt.close(fig)


def _array(rows: list[dict[str, Any]], key: str) -> np.ndarray:
    return np.asarray([float(row[key]) for row in rows], dtype=np.float64)


def _line_bin_edges(lower_nmi: float, upper_nmi: float, step_nmi: float) -> np.ndarray:
    step_count = math.ceil((upper_nmi - lower_nmi) / step_nmi)
    upper_bound = lower_nmi + step_count * step_nmi
    return np.linspace(lower_nmi, upper_bound, step_count + 1)


def _binned_percentiles(
    x_values: np.ndarray,
    y_values: np.ndarray,
    x_edges: np.ndarray,
    percentiles: tuple[float, ...],
) -> tuple[np.ndarray, ...]:
    centers = 0.5 * (x_edges[:-1] + x_edges[1:])
    outputs = [centers]
    for percentile in percentiles:
        series = np.full(centers.shape, np.nan, dtype=np.float64)
        for index, (left, right) in enumerate(pairwise(x_edges)):
            mask = (x_values >= left) & (x_values < right)
            if not np.any(mask):
                continue
            series[index] = float(np.percentile(y_values[mask], percentile))
        outputs.append(series)
    return tuple(outputs)


def _group_rows(
    rows: list[dict[str, Any]],
    key_fn: Any,
) -> dict[tuple[float, float], list[dict[str, Any]]]:
    grouped: dict[tuple[float, float], list[dict[str, Any]]] = {}
    for row in rows:
        grouped.setdefault(key_fn(row), []).append(row)
    return grouped


def _build_heatmap_matrix(
    grouped_rows: dict[tuple[float, float], list[dict[str, Any]]],
    y_values: list[float],
    x_values: list[float],
    reducer: Any,
) -> np.ndarray:
    matrix = np.full((len(y_values), len(x_values)), np.nan, dtype=np.float64)
    for row_index, y_value in enumerate(y_values):
        for col_index, x_value in enumerate(x_values):
            bucket = grouped_rows.get((y_value, x_value))
            if bucket:
                matrix[row_index, col_index] = reducer(bucket)
    return matrix


def _p95_min_error_nmi(rows: list[dict[str, Any]]) -> float:
    return float(np.percentile(_array(rows, "worst_min_separation_error_nmi"), 95.0))


def _rate_above_ambiguity(rows: list[dict[str, Any]]) -> float:
    return float(np.mean(_array(rows, "worst_min_separation_error_nmi") > AMBIGUITY_BAND_NMI))


def _threshold_disagreement_rate(rows: list[dict[str, Any]]) -> float:
    return float(np.mean(_array(rows, "threshold_disagreement_any_corner") > 0.5))


def _orientation_rate_max(rows: list[dict[str, Any]]) -> float:
    grouped = _group_rows(rows, lambda row: (_heading_difference_deg(row), _relative_bearing_deg(row)))
    return max((_threshold_disagreement_rate(bucket) for bucket in grouped.values()), default=0.0)


def _relative_bearing_deg(row: dict[str, Any]) -> float:
    east_m = float(row["rel_east_m"])
    north_m = float(row["rel_north_m"])
    return float((math.degrees(math.atan2(east_m, north_m)) + 360.0) % 360.0)


def _heading_difference_deg(row: dict[str, Any]) -> float:
    a_heading_deg = float(row["a_heading0_deg"])
    b_heading_deg = float(row["b_heading0_deg"])
    return float((b_heading_deg - a_heading_deg) % 360.0)


def _threshold_disagreement(row: dict[str, Any]) -> bool:
    return int(row["threshold_disagreement_any_corner"]) == 1


def _outside_ambiguity_threshold_disagreement(row: dict[str, Any]) -> bool:
    return _threshold_disagreement(row) and float(row["worst_min_separation_error_nmi"]) > AMBIGUITY_BAND_NMI


def _binned_rate(
    rows: list[dict[str, Any]],
    bounds_nmi: tuple[float, ...],
    predicate: Any,
    turn_count: int | None,
) -> np.ndarray:
    rates = np.full(len(bounds_nmi) - 1, np.nan, dtype=np.float64)
    for index, (lower_nmi, upper_nmi) in enumerate(pairwise(bounds_nmi)):
        bucket = [
            row
            for row in rows
            if lower_nmi <= float(row["max_radial_excursion_nmi"]) < upper_nmi
            and (turn_count is None or int(row["turn_count"]) == turn_count)
        ]
        if bucket:
            rates[index] = float(np.mean([predicate(row) for row in bucket]))
    return rates
