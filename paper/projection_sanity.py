"""Projection sanity-check helpers for the manuscript's local-plane model."""

from __future__ import annotations

from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict, dataclass
from itertools import pairwise
import math
from typing import Any

import numpy as np

from geometric_safety.relevant_aircraft import _turn_displacement_basis
from geometric_safety.util import DEG_TO_RAD, EARTH_RADIUS_IN_METERS, KT_TO_MPS, NMI_TO_M, RAD_TO_DEG
from paper.evaluation import (
    AMBIGUITY_BAND_M,
    DEFAULT_THRESHOLD_M,
    Encounter,
    encounter_to_row,
    generate_mixed_turn_suite,
    generate_straight_suite,
)

DEFAULT_PROJECTION_LATITUDES_DEG = (0.0, 30.0, 45.0, 60.0, 75.0)
DEFAULT_PROJECTION_ANCHOR_LON_DEG = 0.0
DEFAULT_REPRESENTATIVE_N = 20_000
DEFAULT_PROJECTION_TIME_STEP_S = 1.0
DEFAULT_PROJECTION_WORKERS = 1
DEFAULT_PROJECTION_CHUNK_SIZE = 256
DEFAULT_PROJECTION_ANCHOR_MODE = "a"
DEFAULT_STRESS_RELATIVE_RANGE_NMI = 40.0
DEFAULT_STRESS_PROJECTION_TIME_S = 1_200.0
DEFAULT_STRESS_SPEED_KT = 450.0
DEFAULT_STRESS_SPEED_DIFF_KT = 30.0
DEFAULT_STRESS_TURN_RATE_DEG_S = 3.5
DEFAULT_STRESS_TURN_OPTIONS_DEG = (-60.0, -30.0, 0.0, 30.0, 60.0)


@dataclass(frozen=True)
class ProjectionSanityConfig:
    """Configuration for the projection sanity-check experiments."""

    latitudes_deg: tuple[float, ...] = DEFAULT_PROJECTION_LATITUDES_DEG
    anchor_lon_deg: float = DEFAULT_PROJECTION_ANCHOR_LON_DEG
    straight_n: int = DEFAULT_REPRESENTATIVE_N
    mixed_turn_n: int = DEFAULT_REPRESENTATIVE_N
    seed: int = 20260327
    time_step_s: float = DEFAULT_PROJECTION_TIME_STEP_S
    n_jobs: int = DEFAULT_PROJECTION_WORKERS
    chunk_size: int = DEFAULT_PROJECTION_CHUNK_SIZE
    anchor_mode: str = DEFAULT_PROJECTION_ANCHOR_MODE
    stress_bearing_step_deg: float = 45.0
    stress_heading_step_deg: float = 90.0
    threshold_m: float = DEFAULT_THRESHOLD_M
    quick: bool = False

    @classmethod
    def quick_defaults(cls) -> ProjectionSanityConfig:
        return cls(
            latitudes_deg=(0.0, 60.0),
            straight_n=200,
            mixed_turn_n=200,
            seed=20260327,
            n_jobs=2,
            chunk_size=32,
            stress_bearing_step_deg=90.0,
            stress_heading_step_deg=180.0,
            quick=True,
        )


DEFAULT_SWEEP_LATITUDES_DEG = (51.0,)
DEFAULT_SWEEP_INITIAL_RANGES_NMI = (6.0, 10.0, 20.0, 30.0, 40.0)
DEFAULT_SWEEP_HORIZONS_S = (300.0, 600.0, 900.0, 1_200.0)
DEFAULT_STRAIGHT_SWEEP_BEARING_STEP_DEG = 30.0
DEFAULT_STRAIGHT_SWEEP_HEADING_STEP_DEG = 30.0
DEFAULT_MIXED_SWEEP_BEARING_STEP_DEG = 60.0
DEFAULT_MIXED_SWEEP_HEADING_STEP_DEG = 60.0
DEFAULT_SWEEP_NOMINAL_SPEEDS_KT = (250.0, 350.0, 450.0)
DEFAULT_SWEEP_SPEED_DIFFS_KT = (5.0, 30.0)
DEFAULT_MIXED_SWEEP_TURN_ANGLES_DEG = (-60.0, 0.0, 60.0)
DEFAULT_EXCURSION_BINS_NMI = (25.0, 50.0, 75.0, 100.0, 150.0, 200.0, 250.0)


@dataclass(frozen=True)
class ProjectionDeterministicSweepConfig:
    """Configuration for deterministic projection-sensitivity sweeps."""

    latitudes_deg: tuple[float, ...] = DEFAULT_SWEEP_LATITUDES_DEG
    anchor_lon_deg: float = DEFAULT_PROJECTION_ANCHOR_LON_DEG
    time_step_s: float = DEFAULT_PROJECTION_TIME_STEP_S
    n_jobs: int = DEFAULT_PROJECTION_WORKERS
    chunk_size: int = DEFAULT_PROJECTION_CHUNK_SIZE
    anchor_mode: str = DEFAULT_PROJECTION_ANCHOR_MODE
    initial_ranges_nmi: tuple[float, ...] = DEFAULT_SWEEP_INITIAL_RANGES_NMI
    horizons_s: tuple[float, ...] = DEFAULT_SWEEP_HORIZONS_S
    straight_relative_bearing_step_deg: float = DEFAULT_STRAIGHT_SWEEP_BEARING_STEP_DEG
    straight_heading_step_deg: float = DEFAULT_STRAIGHT_SWEEP_HEADING_STEP_DEG
    mixed_relative_bearing_step_deg: float = DEFAULT_MIXED_SWEEP_BEARING_STEP_DEG
    mixed_heading_step_deg: float = DEFAULT_MIXED_SWEEP_HEADING_STEP_DEG
    nominal_speeds_kt: tuple[float, ...] = DEFAULT_SWEEP_NOMINAL_SPEEDS_KT
    speed_diffs_kt: tuple[float, ...] = DEFAULT_SWEEP_SPEED_DIFFS_KT
    mixed_turn_angles_deg: tuple[float, ...] = DEFAULT_MIXED_SWEEP_TURN_ANGLES_DEG
    excursion_bins_nmi: tuple[float, ...] = DEFAULT_EXCURSION_BINS_NMI
    threshold_m: float = DEFAULT_THRESHOLD_M
    quick: bool = False

    @classmethod
    def quick_defaults(cls) -> ProjectionDeterministicSweepConfig:
        return cls(
            latitudes_deg=(51.0,),
            n_jobs=2,
            chunk_size=16,
            initial_ranges_nmi=(6.0, 20.0),
            horizons_s=(300.0, 900.0),
            straight_relative_bearing_step_deg=90.0,
            straight_heading_step_deg=90.0,
            mixed_relative_bearing_step_deg=180.0,
            mixed_heading_step_deg=180.0,
            nominal_speeds_kt=(250.0, 450.0),
            speed_diffs_kt=(5.0,),
            mixed_turn_angles_deg=(-60.0, 0.0, 60.0),
            quick=True,
        )


def run_projection_sanity_check(config: ProjectionSanityConfig) -> dict[str, Any]:
    """Run the straight, mixed-turn, and stress suites across the configured latitudes."""
    straight_reference = generate_straight_suite(config.straight_n, config.seed, config.threshold_m)
    mixed_turn_reference = generate_mixed_turn_suite(config.mixed_turn_n, config.seed + 1, config.threshold_m)
    stress_reference = generate_projection_stress_suite(
        threshold_m=config.threshold_m,
        bearing_step_deg=config.stress_bearing_step_deg,
        heading_step_deg=config.stress_heading_step_deg,
    )

    straight_by_latitude = []
    mixed_turn_by_latitude = []
    mixed_turn_turn_count_by_latitude: dict[int, list[dict[str, Any]]] = {0: [], 1: [], 2: []}
    stress_by_latitude = []
    straight_rows_by_latitude: dict[float, list[dict[str, Any]]] = {}
    mixed_turn_rows_by_latitude: dict[float, list[dict[str, Any]]] = {}
    stress_rows_by_latitude: dict[float, list[dict[str, Any]]] = {}

    for latitude_deg in config.latitudes_deg:
        straight_rows = evaluate_projection_suite(
            straight_reference,
            anchor_lat_deg=latitude_deg,
            anchor_lon_deg=config.anchor_lon_deg,
            time_step_s=config.time_step_s,
            suite_name="straight",
            n_jobs=config.n_jobs,
            chunk_size=config.chunk_size,
            anchor_mode=config.anchor_mode,
        )
        mixed_turn_rows = evaluate_projection_suite(
            mixed_turn_reference,
            anchor_lat_deg=latitude_deg,
            anchor_lon_deg=config.anchor_lon_deg,
            time_step_s=config.time_step_s,
            suite_name="mixed_turn",
            n_jobs=config.n_jobs,
            chunk_size=config.chunk_size,
            anchor_mode=config.anchor_mode,
        )
        stress_rows = evaluate_projection_suite(
            stress_reference,
            anchor_lat_deg=latitude_deg,
            anchor_lon_deg=config.anchor_lon_deg,
            time_step_s=config.time_step_s,
            suite_name="stress",
            n_jobs=config.n_jobs,
            chunk_size=config.chunk_size,
            anchor_mode=config.anchor_mode,
        )
        straight_by_latitude.append(summarize_projection_rows(straight_rows))
        mixed_turn_by_latitude.append(summarize_projection_rows(mixed_turn_rows))
        for turn_count in (0, 1, 2):
            subset_rows = [row for row in mixed_turn_rows if int(row["turn_count"]) == turn_count]
            mixed_turn_turn_count_by_latitude[turn_count].append(
                summarize_projection_rows(
                    subset_rows,
                    suite_name="mixed_turn",
                    subset_name=f"turn_count_{turn_count}",
                )
            )
        stress_by_latitude.append(summarize_projection_rows(stress_rows))
        straight_rows_by_latitude[latitude_deg] = straight_rows
        mixed_turn_rows_by_latitude[latitude_deg] = mixed_turn_rows
        stress_rows_by_latitude[latitude_deg] = stress_rows

    straight_by_latitude.sort(key=lambda row: row["anchor_lat_deg"])
    mixed_turn_by_latitude.sort(key=lambda row: row["anchor_lat_deg"])
    stress_by_latitude.sort(key=lambda row: row["anchor_lat_deg"])
    for rows in mixed_turn_turn_count_by_latitude.values():
        rows.sort(key=lambda row: row["anchor_lat_deg"])
    return {
        "config": asdict(config),
        "straight": straight_by_latitude,
        "mixed_turn": mixed_turn_by_latitude,
        "mixed_turn_turn_count": mixed_turn_turn_count_by_latitude,
        "stress": stress_by_latitude,
        "straight_rows_by_latitude": straight_rows_by_latitude,
        "mixed_turn_rows_by_latitude": mixed_turn_rows_by_latitude,
        "stress_rows_by_latitude": stress_rows_by_latitude,
    }


def run_projection_deterministic_sweep(config: ProjectionDeterministicSweepConfig) -> dict[str, Any]:
    """Run deterministic straight and mixed-turn projection sweeps."""
    straight_encounters = generate_deterministic_straight_sweep(config)
    mixed_turn_encounters = generate_deterministic_mixed_turn_sweep(config)

    straight_summary_rows = []
    straight_excursion_rows = []
    mixed_turn_summary_rows = []
    mixed_turn_excursion_rows = []
    mixed_turn_turn_count_excursion_rows = []
    straight_rows_by_latitude: dict[float, list[dict[str, Any]]] = {}
    mixed_turn_rows_by_latitude: dict[float, list[dict[str, Any]]] = {}

    for latitude_deg in config.latitudes_deg:
        straight_rows = evaluate_projection_suite(
            straight_encounters,
            anchor_lat_deg=latitude_deg,
            anchor_lon_deg=config.anchor_lon_deg,
            time_step_s=config.time_step_s,
            suite_name="straight_sweep",
            n_jobs=config.n_jobs,
            chunk_size=config.chunk_size,
            anchor_mode=config.anchor_mode,
        )
        mixed_turn_rows = evaluate_projection_suite(
            mixed_turn_encounters,
            anchor_lat_deg=latitude_deg,
            anchor_lon_deg=config.anchor_lon_deg,
            time_step_s=config.time_step_s,
            suite_name="mixed_turn_sweep",
            n_jobs=config.n_jobs,
            chunk_size=config.chunk_size,
            anchor_mode=config.anchor_mode,
        )
        straight_summary_rows.append(summarize_projection_rows(straight_rows, suite_name="straight_sweep"))
        mixed_turn_summary_rows.append(summarize_projection_rows(mixed_turn_rows, suite_name="mixed_turn_sweep"))
        straight_excursion_rows.extend(
            summarize_projection_rows_by_excursion(
                straight_rows,
                suite_name="straight_sweep",
                excursion_bins_nmi=config.excursion_bins_nmi,
            )
        )
        mixed_turn_excursion_rows.extend(
            summarize_projection_rows_by_excursion(
                mixed_turn_rows,
                suite_name="mixed_turn_sweep",
                excursion_bins_nmi=config.excursion_bins_nmi,
            )
        )
        for turn_count in (0, 1, 2):
            subset = [row for row in mixed_turn_rows if int(row["turn_count"]) == turn_count]
            mixed_turn_turn_count_excursion_rows.extend(
                summarize_projection_rows_by_excursion(
                    subset,
                    suite_name="mixed_turn_sweep",
                    subset_name=f"turn_count_{turn_count}",
                    excursion_bins_nmi=config.excursion_bins_nmi,
                )
            )
        straight_rows_by_latitude[latitude_deg] = straight_rows
        mixed_turn_rows_by_latitude[latitude_deg] = mixed_turn_rows

    straight_summary_rows.sort(key=lambda row: float(row["anchor_lat_deg"]))
    mixed_turn_summary_rows.sort(key=lambda row: float(row["anchor_lat_deg"]))
    straight_excursion_rows.sort(key=lambda row: (float(row["anchor_lat_deg"]), float(row["excursion_bin_lower_nmi"])))
    mixed_turn_excursion_rows.sort(
        key=lambda row: (float(row["anchor_lat_deg"]), float(row["excursion_bin_lower_nmi"]))
    )
    mixed_turn_turn_count_excursion_rows.sort(
        key=lambda row: (
            float(row["anchor_lat_deg"]),
            str(row["subset_name"]),
            float(row["excursion_bin_lower_nmi"]),
        )
    )
    return {
        "config": asdict(config),
        "straight_summary": straight_summary_rows,
        "straight_by_excursion": straight_excursion_rows,
        "mixed_turn_summary": mixed_turn_summary_rows,
        "mixed_turn_by_excursion": mixed_turn_excursion_rows,
        "mixed_turn_by_excursion_and_turn_count": mixed_turn_turn_count_excursion_rows,
        "straight_rows_by_latitude": straight_rows_by_latitude,
        "mixed_turn_rows_by_latitude": mixed_turn_rows_by_latitude,
    }


def generate_deterministic_straight_sweep(config: ProjectionDeterministicSweepConfig) -> list[Encounter]:
    """Construct the deterministic straight-heading projection sweep."""
    headings = np.arange(0.0, 360.0, config.straight_heading_step_deg, dtype=np.float64)
    relative_bearings = np.arange(0.0, 360.0, config.straight_relative_bearing_step_deg, dtype=np.float64)

    encounters = []
    for initial_range_nmi in config.initial_ranges_nmi:
        rel_range_m = initial_range_nmi * NMI_TO_M
        for horizon_s in config.horizons_s:
            for rel_bearing_deg in relative_bearings:
                rel_bearing_rad = float(rel_bearing_deg) * DEG_TO_RAD
                rel_east_m = rel_range_m * math.sin(rel_bearing_rad)
                rel_north_m = rel_range_m * math.cos(rel_bearing_rad)
                for a_heading0_deg in headings:
                    for b_heading0_deg in headings:
                        for nominal_speed_kt in config.nominal_speeds_kt:
                            encounters.extend(
                                Encounter(
                                    rel_east_m=float(rel_east_m),
                                    rel_north_m=float(rel_north_m),
                                    a_heading0_deg=float(a_heading0_deg),
                                    a_target_heading_deg=float(a_heading0_deg),
                                    a_speed_kt=float(nominal_speed_kt),
                                    a_turn_rate_deg_sec=0.0,
                                    b_heading0_deg=float(b_heading0_deg),
                                    b_target_heading_deg=float(b_heading0_deg),
                                    b_speed_kt=float(nominal_speed_kt),
                                    b_turn_rate_deg_sec=0.0,
                                    speed_diff_kt=float(speed_diff_kt),
                                    projection_time_s=float(horizon_s),
                                    separation_threshold_m=config.threshold_m,
                                )
                                for speed_diff_kt in config.speed_diffs_kt
                            )
    return encounters


def generate_deterministic_mixed_turn_sweep(config: ProjectionDeterministicSweepConfig) -> list[Encounter]:
    """Construct the deterministic mixed-turn projection sweep."""
    headings = np.arange(0.0, 360.0, config.mixed_heading_step_deg, dtype=np.float64)
    relative_bearings = np.arange(0.0, 360.0, config.mixed_relative_bearing_step_deg, dtype=np.float64)

    encounters = []
    for initial_range_nmi in config.initial_ranges_nmi:
        rel_range_m = initial_range_nmi * NMI_TO_M
        for horizon_s in config.horizons_s:
            for rel_bearing_deg in relative_bearings:
                rel_bearing_rad = float(rel_bearing_deg) * DEG_TO_RAD
                rel_east_m = rel_range_m * math.sin(rel_bearing_rad)
                rel_north_m = rel_range_m * math.cos(rel_bearing_rad)
                for a_heading0_deg in headings:
                    for b_heading0_deg in headings:
                        for a_turn_angle_deg in config.mixed_turn_angles_deg:
                            for b_turn_angle_deg in config.mixed_turn_angles_deg:
                                for nominal_speed_kt in config.nominal_speeds_kt:
                                    encounters.extend(
                                        Encounter(
                                            rel_east_m=float(rel_east_m),
                                            rel_north_m=float(rel_north_m),
                                            a_heading0_deg=float(a_heading0_deg),
                                            a_target_heading_deg=float((a_heading0_deg + a_turn_angle_deg) % 360.0),
                                            a_speed_kt=float(nominal_speed_kt),
                                            a_turn_rate_deg_sec=(
                                                0.0 if abs(a_turn_angle_deg) < 1e-9 else DEFAULT_STRESS_TURN_RATE_DEG_S
                                            ),
                                            b_heading0_deg=float(b_heading0_deg),
                                            b_target_heading_deg=float((b_heading0_deg + b_turn_angle_deg) % 360.0),
                                            b_speed_kt=float(nominal_speed_kt),
                                            b_turn_rate_deg_sec=(
                                                0.0 if abs(b_turn_angle_deg) < 1e-9 else DEFAULT_STRESS_TURN_RATE_DEG_S
                                            ),
                                            speed_diff_kt=float(speed_diff_kt),
                                            projection_time_s=float(horizon_s),
                                            separation_threshold_m=config.threshold_m,
                                        )
                                        for speed_diff_kt in config.speed_diffs_kt
                                    )
    return encounters


def generate_projection_stress_suite(
    *,
    threshold_m: float = DEFAULT_THRESHOLD_M,
    bearing_step_deg: float = 45.0,
    heading_step_deg: float = 90.0,
) -> list[Encounter]:
    """Construct a deterministic upper-end stress suite over encounter orientations."""
    rel_range_m = DEFAULT_STRESS_RELATIVE_RANGE_NMI * NMI_TO_M
    headings = np.arange(0.0, 360.0, heading_step_deg, dtype=np.float64)
    relative_bearings = np.arange(0.0, 360.0, bearing_step_deg, dtype=np.float64)

    encounters = []
    for rel_bearing_deg in relative_bearings:
        rel_bearing_rad = float(rel_bearing_deg) * DEG_TO_RAD
        rel_east_m = rel_range_m * math.sin(rel_bearing_rad)
        rel_north_m = rel_range_m * math.cos(rel_bearing_rad)
        for a_heading0_deg in headings:
            for b_heading0_deg in headings:
                for a_turn_angle_deg in DEFAULT_STRESS_TURN_OPTIONS_DEG:
                    encounters.extend(
                        Encounter(
                            rel_east_m=float(rel_east_m),
                            rel_north_m=float(rel_north_m),
                            a_heading0_deg=float(a_heading0_deg),
                            a_target_heading_deg=float((a_heading0_deg + a_turn_angle_deg) % 360.0),
                            a_speed_kt=DEFAULT_STRESS_SPEED_KT,
                            a_turn_rate_deg_sec=(
                                0.0 if abs(a_turn_angle_deg) < 1e-9 else DEFAULT_STRESS_TURN_RATE_DEG_S
                            ),
                            b_heading0_deg=float(b_heading0_deg),
                            b_target_heading_deg=float((b_heading0_deg + b_turn_angle_deg) % 360.0),
                            b_speed_kt=DEFAULT_STRESS_SPEED_KT,
                            b_turn_rate_deg_sec=(
                                0.0 if abs(b_turn_angle_deg) < 1e-9 else DEFAULT_STRESS_TURN_RATE_DEG_S
                            ),
                            speed_diff_kt=DEFAULT_STRESS_SPEED_DIFF_KT,
                            projection_time_s=DEFAULT_STRESS_PROJECTION_TIME_S,
                            separation_threshold_m=threshold_m,
                        )
                        for b_turn_angle_deg in DEFAULT_STRESS_TURN_OPTIONS_DEG
                    )
    return encounters


def evaluate_projection_suite(
    encounters: list[Encounter],
    *,
    anchor_lat_deg: float,
    anchor_lon_deg: float,
    time_step_s: float,
    suite_name: str,
    n_jobs: int = DEFAULT_PROJECTION_WORKERS,
    chunk_size: int = DEFAULT_PROJECTION_CHUNK_SIZE,
    anchor_mode: str = DEFAULT_PROJECTION_ANCHOR_MODE,
) -> list[dict[str, Any]]:
    """Evaluate one encounter suite at a fixed anchor latitude."""
    if n_jobs <= 1 or len(encounters) <= chunk_size:
        return [
            evaluate_projection_encounter(
                encounter,
                anchor_lat_deg=anchor_lat_deg,
                anchor_lon_deg=anchor_lon_deg,
                time_step_s=time_step_s,
                suite_name=suite_name,
                anchor_mode=anchor_mode,
            )
            for encounter in encounters
        ]

    chunks = list(chunked_encounters(encounters, chunk_size))
    rows: list[dict[str, Any]] = []
    with ProcessPoolExecutor(max_workers=n_jobs) as executor:
        futures = [
            executor.submit(
                evaluate_projection_suite_chunk,
                chunk,
                anchor_lat_deg=anchor_lat_deg,
                anchor_lon_deg=anchor_lon_deg,
                time_step_s=time_step_s,
                suite_name=suite_name,
                anchor_mode=anchor_mode,
            )
            for chunk in chunks
        ]
        for future in futures:
            rows.extend(future.result())
    return rows


def chunked_encounters(encounters: list[Encounter], chunk_size: int) -> list[list[Encounter]]:
    """Split encounters into fixed-size chunks for worker pools."""
    return [encounters[index : index + chunk_size] for index in range(0, len(encounters), chunk_size)]


def evaluate_projection_suite_chunk(
    encounters: list[Encounter],
    *,
    anchor_lat_deg: float,
    anchor_lon_deg: float,
    time_step_s: float,
    suite_name: str,
    anchor_mode: str = DEFAULT_PROJECTION_ANCHOR_MODE,
) -> list[dict[str, Any]]:
    """Worker entrypoint for projection-suite chunks."""
    return [
        evaluate_projection_encounter(
            encounter,
            anchor_lat_deg=anchor_lat_deg,
            anchor_lon_deg=anchor_lon_deg,
            time_step_s=time_step_s,
            suite_name=suite_name,
            anchor_mode=anchor_mode,
        )
        for encounter in encounters
    ]


def evaluate_projection_encounter(
    encounter: Encounter,
    *,
    anchor_lat_deg: float,
    anchor_lon_deg: float,
    time_step_s: float,
    suite_name: str,
    anchor_mode: str = DEFAULT_PROJECTION_ANCHOR_MODE,
) -> dict[str, Any]:
    """Evaluate projection distortion on one encounter across all speed corners."""
    times_s = build_projection_times(encounter.projection_time_s, time_step_s)
    speed_pairs = speed_corner_pairs_kt(encounter)

    worst_pointwise_error_m = -math.inf
    worst_min_error_m = -math.inf
    worst_pointwise_corner = ""
    worst_min_corner = ""
    threshold_disagreement_corners_n = 0
    max_radial_excursion_nmi = 0.0

    for corner_label, a_speed_kt, b_speed_kt in speed_pairs:
        corner_result = evaluate_projection_corner(
            encounter,
            anchor_lat_deg=anchor_lat_deg,
            anchor_lon_deg=anchor_lon_deg,
            time_step_s=time_step_s,
            times_s=times_s,
            a_speed_kt=a_speed_kt,
            b_speed_kt=b_speed_kt,
            anchor_mode=anchor_mode,
        )
        if corner_result["pointwise_error_m"] > worst_pointwise_error_m:
            worst_pointwise_error_m = corner_result["pointwise_error_m"]
            worst_pointwise_corner = corner_label
        if corner_result["min_separation_error_m"] > worst_min_error_m:
            worst_min_error_m = corner_result["min_separation_error_m"]
            worst_min_corner = corner_label
        threshold_disagreement_corners_n += int(corner_result["threshold_disagreement"])
        max_radial_excursion_nmi = max(max_radial_excursion_nmi, corner_result["max_radial_excursion_nmi"])

    row = encounter_to_row(encounter)
    row.update(
        {
            "suite_name": suite_name,
            "anchor_lat_deg": anchor_lat_deg,
            "anchor_lon_deg": anchor_lon_deg,
            "anchor_mode": anchor_mode,
            "time_step_s": time_step_s,
            "worst_pointwise_error_m": worst_pointwise_error_m,
            "worst_pointwise_error_nmi": worst_pointwise_error_m / NMI_TO_M,
            "worst_min_separation_error_m": worst_min_error_m,
            "worst_min_separation_error_nmi": worst_min_error_m / NMI_TO_M,
            "worst_pointwise_error_corner": worst_pointwise_corner,
            "worst_min_separation_error_corner": worst_min_corner,
            "threshold_disagreement_any_corner": int(threshold_disagreement_corners_n > 0),
            "threshold_disagreement_corners_n": threshold_disagreement_corners_n,
            "max_radial_excursion_nmi": max_radial_excursion_nmi,
        }
    )
    return row


def evaluate_projection_corner(
    encounter: Encounter,
    *,
    anchor_lat_deg: float,
    anchor_lon_deg: float,
    time_step_s: float,
    times_s: np.ndarray,
    a_speed_kt: float,
    b_speed_kt: float,
    anchor_mode: str = DEFAULT_PROJECTION_ANCHOR_MODE,
) -> dict[str, float | bool]:
    """Evaluate one projection corner by comparing planar and spherical separations."""
    a_track_xy = sample_exact_trajectory_xy(
        np.zeros(2, dtype=np.float64),
        encounter.a_heading0_deg,
        encounter.a_target_heading_deg,
        a_speed_kt * KT_TO_MPS,
        encounter.a_turn_rate_deg_sec,
        times_s,
    )
    b_track_xy = sample_exact_trajectory_xy(
        np.array([-encounter.rel_east_m, -encounter.rel_north_m], dtype=np.float64),
        encounter.b_heading0_deg,
        encounter.b_target_heading_deg,
        b_speed_kt * KT_TO_MPS,
        encounter.b_turn_rate_deg_sec,
        times_s,
    )
    a_track_xy, b_track_xy = shift_tracks_for_anchor_mode(a_track_xy, b_track_xy, encounter, anchor_mode)

    planar_separation_m = np.linalg.norm(a_track_xy - b_track_xy, axis=1)
    a_lat_deg, a_lon_deg = embed_local_track_on_sphere(a_track_xy, anchor_lat_deg, anchor_lon_deg)
    b_lat_deg, b_lon_deg = embed_local_track_on_sphere(b_track_xy, anchor_lat_deg, anchor_lon_deg)
    spherical_separation_m = great_circle_distance_m(a_lat_deg, a_lon_deg, b_lat_deg, b_lon_deg)

    pointwise_error_m = float(np.max(np.abs(planar_separation_m - spherical_separation_m)))
    min_separation_error_m = float(abs(np.min(planar_separation_m) - np.min(spherical_separation_m)))
    threshold_disagreement = bool(
        (np.min(planar_separation_m) >= encounter.separation_threshold_m)
        != (np.min(spherical_separation_m) >= encounter.separation_threshold_m)
    )
    max_radial_excursion_nmi = float(
        max(np.max(np.linalg.norm(a_track_xy, axis=1)), np.max(np.linalg.norm(b_track_xy, axis=1))) / NMI_TO_M
    )
    return {
        "pointwise_error_m": pointwise_error_m,
        "min_separation_error_m": min_separation_error_m,
        "threshold_disagreement": threshold_disagreement,
        "max_radial_excursion_nmi": max_radial_excursion_nmi,
        "time_step_s": time_step_s,
    }


def shift_tracks_for_anchor_mode(
    a_track_xy: np.ndarray,
    b_track_xy: np.ndarray,
    encounter: Encounter,
    anchor_mode: str,
) -> tuple[np.ndarray, np.ndarray]:
    """Translate tracks so the requested local anchor lies at the origin."""
    if anchor_mode == "a":
        shift_xy = np.zeros(2, dtype=np.float64)
    elif anchor_mode == "b":
        shift_xy = np.array([encounter.rel_east_m, encounter.rel_north_m], dtype=np.float64)
    elif anchor_mode == "midpoint":
        shift_xy = 0.5 * np.array([encounter.rel_east_m, encounter.rel_north_m], dtype=np.float64)
    else:
        raise ValueError(f"Unsupported anchor_mode={anchor_mode!r}")
    return a_track_xy + shift_xy, b_track_xy + shift_xy


def build_projection_times(projection_time_s: float, time_step_s: float) -> np.ndarray:
    """Construct a fixed time grid that includes the horizon endpoint."""
    step_count = math.floor(projection_time_s / time_step_s + 1e-12)
    times_s = np.arange(step_count + 1, dtype=np.float64) * time_step_s
    if times_s[-1] < projection_time_s:
        times_s = np.concatenate([times_s, np.asarray([projection_time_s], dtype=np.float64)])
    else:
        times_s[-1] = projection_time_s
    return times_s


def speed_corner_pairs_kt(encounter: Encounter) -> list[tuple[str, float, float]]:
    """Return the four speed-corner pairs for an encounter."""
    a_min = max(encounter.a_speed_kt - encounter.speed_diff_kt, 0.0)
    a_max = encounter.a_speed_kt + encounter.speed_diff_kt
    b_min = max(encounter.b_speed_kt - encounter.speed_diff_kt, 0.0)
    b_max = encounter.b_speed_kt + encounter.speed_diff_kt
    return [
        ("a_min_b_min", a_min, b_min),
        ("a_min_b_max", a_min, b_max),
        ("a_max_b_min", a_max, b_min),
        ("a_max_b_max", a_max, b_max),
    ]


def sample_exact_trajectory_xy(
    start_xy: np.ndarray,
    heading0_deg: float,
    target_heading_deg: float,
    speed_mps: float,
    turn_rate_deg_sec: float,
    times_s: np.ndarray,
) -> np.ndarray:
    """Sample one exact turn-then-straight trajectory in local coordinates."""
    xy = np.empty((times_s.shape[0], 2), dtype=np.float64)
    for idx, t_s in enumerate(times_s):
        basis = _turn_displacement_basis(heading0_deg, target_heading_deg, turn_rate_deg_sec, float(t_s))
        xy[idx, 0] = start_xy[0] + speed_mps * basis[0]
        xy[idx, 1] = start_xy[1] + speed_mps * basis[1]
    return xy


def embed_local_track_on_sphere(
    track_xy: np.ndarray,
    anchor_lat_deg: float,
    anchor_lon_deg: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Map a local ENU track back to latitude/longitude using the model's local inverse."""
    east_m = track_xy[:, 0]
    north_m = track_xy[:, 1]
    anchor_lat_rad = anchor_lat_deg * DEG_TO_RAD
    lat_deg = anchor_lat_deg + (north_m / EARTH_RADIUS_IN_METERS) * RAD_TO_DEG
    lon_deg = anchor_lon_deg + (east_m / (EARTH_RADIUS_IN_METERS * math.cos(anchor_lat_rad))) * RAD_TO_DEG
    return lat_deg, lon_deg


def great_circle_distance_m(
    lat_a_deg: np.ndarray,
    lon_a_deg: np.ndarray,
    lat_b_deg: np.ndarray,
    lon_b_deg: np.ndarray,
) -> np.ndarray:
    """Compute the great-circle distance between two arrays of latitude/longitude points."""
    lat_a_rad = lat_a_deg * DEG_TO_RAD
    lon_a_rad = lon_a_deg * DEG_TO_RAD
    lat_b_rad = lat_b_deg * DEG_TO_RAD
    lon_b_rad = lon_b_deg * DEG_TO_RAD

    d_lat = lat_b_rad - lat_a_rad
    d_lon = lon_b_rad - lon_a_rad
    sin_d_lat = np.sin(d_lat / 2.0)
    sin_d_lon = np.sin(d_lon / 2.0)
    hav = sin_d_lat * sin_d_lat + np.cos(lat_a_rad) * np.cos(lat_b_rad) * sin_d_lon * sin_d_lon
    central_angle = 2.0 * np.arctan2(np.sqrt(hav), np.sqrt(np.maximum(1.0 - hav, 0.0)))
    return EARTH_RADIUS_IN_METERS * central_angle


def summarize_projection_rows(
    rows: list[dict[str, Any]],
    *,
    suite_name: str | None = None,
    subset_name: str = "all",
) -> dict[str, Any]:
    """Reduce per-encounter rows into one latitude summary row."""
    if not rows:
        raise ValueError("Projection-suite rows must not be empty.")

    pointwise_errors_m = np.asarray([float(row["worst_pointwise_error_m"]) for row in rows], dtype=np.float64)
    min_errors_m = np.asarray([float(row["worst_min_separation_error_m"]) for row in rows], dtype=np.float64)
    radial_excursions_nmi = np.asarray([float(row["max_radial_excursion_nmi"]) for row in rows], dtype=np.float64)
    threshold_disagreement_encounter_n = sum(int(row["threshold_disagreement_any_corner"]) for row in rows)
    threshold_disagreement_corner_n = sum(int(row["threshold_disagreement_corners_n"]) for row in rows)
    total_corner_n = 4 * len(rows)

    return {
        "suite_name": rows[0]["suite_name"] if suite_name is None else suite_name,
        "subset_name": subset_name,
        "anchor_lat_deg": float(rows[0]["anchor_lat_deg"]),
        "anchor_lon_deg": float(rows[0]["anchor_lon_deg"]),
        "anchor_mode": str(rows[0]["anchor_mode"]),
        "n": len(rows),
        "corner_n": total_corner_n,
        "time_step_s": float(rows[0]["time_step_s"]),
        "pointwise_error_median_m": float(np.median(pointwise_errors_m)),
        "pointwise_error_p95_m": float(np.percentile(pointwise_errors_m, 95.0)),
        "pointwise_error_p99_m": float(np.percentile(pointwise_errors_m, 99.0)),
        "pointwise_error_max_m": float(np.max(pointwise_errors_m)),
        "min_separation_error_median_m": float(np.median(min_errors_m)),
        "min_separation_error_p95_m": float(np.percentile(min_errors_m, 95.0)),
        "min_separation_error_p99_m": float(np.percentile(min_errors_m, 99.0)),
        "min_separation_error_max_m": float(np.max(min_errors_m)),
        "max_radial_excursion_p95_nmi": float(np.percentile(radial_excursions_nmi, 95.0)),
        "max_radial_excursion_max_nmi": float(np.max(radial_excursions_nmi)),
        "threshold_disagreement_encounter_n": threshold_disagreement_encounter_n,
        "threshold_disagreement_encounter_rate": threshold_disagreement_encounter_n / float(len(rows)),
        "threshold_disagreement_corner_n": threshold_disagreement_corner_n,
        "threshold_disagreement_corner_rate": threshold_disagreement_corner_n / float(total_corner_n),
        "min_separation_error_above_ambiguity_n": int(np.sum(min_errors_m > AMBIGUITY_BAND_M)),
        "pointwise_error_above_ambiguity_n": int(np.sum(pointwise_errors_m > AMBIGUITY_BAND_M)),
    }


def summarize_projection_rows_by_excursion(
    rows: list[dict[str, Any]],
    *,
    suite_name: str,
    excursion_bins_nmi: tuple[float, ...],
    subset_name: str = "all",
) -> list[dict[str, Any]]:
    """Summarize projection rows by maximum radial excursion from the anchor."""
    if not rows:
        return []

    bounds = (0.0, *excursion_bins_nmi)
    summaries = []
    for lower_nmi, upper_nmi in pairwise(bounds):
        bucket = [row for row in rows if lower_nmi <= float(row["max_radial_excursion_nmi"]) < upper_nmi]
        if not bucket:
            continue
        summary = summarize_projection_rows(bucket, suite_name=suite_name, subset_name=subset_name)
        summary["excursion_bin_lower_nmi"] = float(lower_nmi)
        summary["excursion_bin_upper_nmi"] = float(upper_nmi)
        summaries.append(summary)
    return summaries
