from __future__ import annotations

from pathlib import Path
from typing import Any, Literal

from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
import numpy as np
from pydantic import BaseModel, Field

from geometric_safety.demo import (
    CUSTOM_TURN_PRESET_ID,
    build_turn_debug_payload,
    get_turn_preset_options,
)
from geometric_safety.relevant_aircraft import (
    catch_up_projection_interval,
    catch_up_projection_interval_with_turns,
    compute_relative_velocity_hull,
)
from geometric_safety.util import (
    EARTH_RADIUS_IN_METERS,
    KT_TO_MPS,
    NMI_TO_M,
    heading_to_unit_vector,
    latlon_to_local_xy,
)

BASE_DIR = Path(__file__).parent
TEMPLATES_DIR = BASE_DIR / "templates"
STATIC_DIR = BASE_DIR / "static"
templates = Jinja2Templates(directory=str(TEMPLATES_DIR))

ModelMode = Literal["fixed_heading", "turn_aware"]


class VisualParams(BaseModel):
    model_mode: ModelMode = "fixed_heading"
    turn_preset_id: str = CUSTOM_TURN_PRESET_ID

    separation_threshold_nm: float = Field(default=5.0, ge=1.0, le=50.0)
    speed_diff_kt: float = Field(default=50.0, ge=0.0, le=300.0)
    projection_time_min: float = Field(default=30.0, ge=5.0, le=30.0)
    grid_density: int = Field(default=150, ge=25, le=300)
    view_half_extent_nm: float = Field(default=25.0, ge=5.0, le=300.0)

    a_lat: float = Field(default=0.0)
    a_lon: float = Field(default=0.0)
    a_heading: float = Field(default=0.0)
    a_target_heading_deg: float = Field(default=0.0, ge=-360.0, le=360.0)
    a_turn_rate_deg_sec: float = Field(default=0.0, ge=0.0, le=10.0)
    a_speed_kt: float = Field(default=400.0, ge=0.0)

    b_lat: float = Field(default=-0.1)
    b_lon: float = Field(default=0.1)
    b_heading: float = Field(default=90.0)
    b_target_heading_deg: float = Field(default=90.0, ge=-360.0, le=360.0)
    b_turn_rate_deg_sec: float = Field(default=0.0, ge=0.0, le=10.0)
    b_speed_kt: float = Field(default=300.0, ge=0.0)

    turn_speed_schedule_uncertainty_kt: float = Field(default=0.0, ge=0.0, le=150.0)


DEFAULT_UI_VALUES: dict[str, Any] = {
    "model_mode": "fixed_heading",
    "turn_preset_id": CUSTOM_TURN_PRESET_ID,
    "separation_threshold_nm": 5.0,
    "speed_diff_kt": 50.0,
    "projection_time_min": 30.0,
    "grid_density": 150,
    "view_half_extent_nm": 25.0,
    "a_lat": 0.0,
    "a_lon": 0.0,
    "a_heading": 0.0,
    "a_target_heading_deg": 0.0,
    "a_turn_rate_deg_sec": 0.0,
    "a_speed_kt": 400.0,
    "b_lat": -0.1,
    "b_lon": 0.1,
    "b_heading": 90.0,
    "b_target_heading_deg": 90.0,
    "b_turn_rate_deg_sec": 0.0,
    "b_speed_kt": 300.0,
    "turn_speed_schedule_uncertainty_kt": 0.0,
}


def _grid_bounds(params: VisualParams) -> tuple[tuple[float, float], tuple[float, float]]:
    lat_half_span_deg = params.view_half_extent_nm / 60.0
    ref_lat_rad = np.deg2rad(params.a_lat)
    cos_lat = max(np.cos(ref_lat_rad), 1e-6)
    lon_half_span_deg = params.view_half_extent_nm / (60.0 * cos_lat)

    lat_min = params.a_lat - lat_half_span_deg
    lat_max = params.a_lat + lat_half_span_deg
    lon_min = params.a_lon - lon_half_span_deg
    lon_max = params.a_lon + lon_half_span_deg
    return (lat_min, lat_max), (lon_min, lon_max)


def _grid_coordinate_vectors(params: VisualParams) -> tuple[np.ndarray, np.ndarray]:
    (lat_min, lat_max), (lon_min, lon_max) = _grid_bounds(params)
    lats = np.linspace(lat_min, lat_max, params.grid_density, dtype=np.float64)
    lons = np.linspace(lon_min, lon_max, params.grid_density, dtype=np.float64)
    return lats, lons


def _local_bounds_from_grid(params: VisualParams, padding_m: float = 0.0) -> dict[str, float]:
    (lat_min, lat_max), (lon_min, lon_max) = _grid_bounds(params)
    corners = [
        (lat_min, lon_min),
        (lat_min, lon_max),
        (lat_max, lon_min),
        (lat_max, lon_max),
    ]
    xy_points = [latlon_to_local_xy(lat, lon, ref_lat=params.a_lat, ref_lon=params.a_lon) for lat, lon in corners]
    xs = [float(p[0]) for p in xy_points]
    ys = [float(p[1]) for p in xy_points]
    return {
        "x_min": min(xs) - padding_m,
        "x_max": max(xs) + padding_m,
        "y_min": min(ys) - padding_m,
        "y_max": max(ys) + padding_m,
    }


def _aircraft_payload(params: VisualParams) -> dict[str, dict[str, float]]:
    return {
        "a": {
            "lat": float(params.a_lat),
            "lon": float(params.a_lon),
            "heading": float(params.a_heading),
            "target_heading_deg": float(params.a_target_heading_deg),
            "turn_rate_deg_sec": float(params.a_turn_rate_deg_sec),
            "speed_kt": float(params.a_speed_kt),
        },
        "b": {
            "lat": float(params.b_lat),
            "lon": float(params.b_lon),
            "heading": float(params.b_heading),
            "target_heading_deg": float(params.b_target_heading_deg),
            "turn_rate_deg_sec": float(params.b_turn_rate_deg_sec),
            "speed_kt": float(params.b_speed_kt),
        },
    }


def _fixed_heading_grid_payload(params: VisualParams) -> dict[str, Any]:
    separation_threshold_m = params.separation_threshold_nm * NMI_TO_M
    projection_time_s = params.projection_time_min * 60.0
    b_lats, b_lons = _grid_coordinate_vectors(params)
    safe_matrix = np.zeros((b_lats.size, b_lons.size), dtype=bool)
    time_matrix_mins = np.zeros_like(safe_matrix, dtype=np.float64)
    velocity_hull = compute_relative_velocity_hull(
        a_heading=params.a_heading,
        a_speed_kt=params.a_speed_kt,
        b_heading=params.b_heading,
        b_speed_kt=params.b_speed_kt,
        speed_diff_kt=params.speed_diff_kt,
        projection_time_s=projection_time_s,
    )

    ref_lat_rad = np.deg2rad(params.a_lat)
    north_offsets = EARTH_RADIUS_IN_METERS * np.deg2rad(b_lats - params.a_lat)
    east_offsets = EARTH_RADIUS_IN_METERS * np.deg2rad(b_lons - params.a_lon) * np.cos(ref_lat_rad)
    rel_vec = np.empty(2, dtype=np.float64)

    for i, north in enumerate(north_offsets):
        rel_vec[1] = -north
        for j, east in enumerate(east_offsets):
            rel_vec[0] = -east
            is_sep, _, closest_time_s = catch_up_projection_interval(
                a_lat=params.a_lat,
                a_lon=params.a_lon,
                a_heading=params.a_heading,
                a_speed_kt=params.a_speed_kt,
                b_lat=float(b_lats[i]),
                b_lon=float(b_lons[j]),
                b_heading=params.b_heading,
                b_speed_kt=params.b_speed_kt,
                separation_threshold_m=separation_threshold_m,
                speed_diff_kt=params.speed_diff_kt,
                projection_time_s=projection_time_s,
                rel_pos_override=rel_vec,
                velocity_hull=velocity_hull,
            )
            safe_matrix[i, j] = is_sep
            time_matrix_mins[i, j] = closest_time_s / 60.0

    grid_payload = {
        "model_mode": "fixed_heading",
        "turn_preset_id": params.turn_preset_id,
        "lats": b_lats.tolist(),
        "lons": b_lons.tolist(),
        "safe": safe_matrix.tolist(),
        "closest_time_min": time_matrix_mins.tolist(),
        "stats": {
            "safe_percentage": float(100.0 * safe_matrix.mean()),
            "mean_closest_time_min": float(np.mean(time_matrix_mins)),
        },
        "lat_bounds": [float(b_lats[0]), float(b_lats[-1])],
        "lon_bounds": [float(b_lons[0]), float(b_lons[-1])],
        "view_half_extent_nm": float(params.view_half_extent_nm),
    }
    grid_payload.update(_aircraft_payload(params))
    return grid_payload


def _turn_aware_grid_payload(params: VisualParams) -> dict[str, Any]:
    separation_threshold_m = params.separation_threshold_nm * NMI_TO_M
    projection_time_s = params.projection_time_min * 60.0
    b_lats, b_lons = _grid_coordinate_vectors(params)
    safe_matrix = np.zeros((b_lats.size, b_lons.size), dtype=bool)
    time_matrix_mins = np.zeros_like(safe_matrix, dtype=np.float64)

    for i, b_lat in enumerate(b_lats):
        for j, b_lon in enumerate(b_lons):
            is_sep, _, closest_time_s = catch_up_projection_interval_with_turns(
                a_lat=params.a_lat,
                a_lon=params.a_lon,
                a_heading0_deg=params.a_heading,
                a_target_heading_deg=params.a_target_heading_deg,
                a_speed_kt=params.a_speed_kt,
                a_turn_rate_deg_sec=params.a_turn_rate_deg_sec,
                b_lat=float(b_lat),
                b_lon=float(b_lon),
                b_heading0_deg=params.b_heading,
                b_target_heading_deg=params.b_target_heading_deg,
                b_speed_kt=params.b_speed_kt,
                b_turn_rate_deg_sec=params.b_turn_rate_deg_sec,
                separation_threshold_m=separation_threshold_m,
                speed_diff_kt=params.speed_diff_kt,
                projection_time_s=projection_time_s,
                turn_speed_schedule_uncertainty_kt=params.turn_speed_schedule_uncertainty_kt,
            )
            safe_matrix[i, j] = is_sep
            time_matrix_mins[i, j] = closest_time_s / 60.0

    grid_payload = {
        "model_mode": "turn_aware",
        "turn_preset_id": params.turn_preset_id,
        "lats": b_lats.tolist(),
        "lons": b_lons.tolist(),
        "safe": safe_matrix.tolist(),
        "closest_time_min": time_matrix_mins.tolist(),
        "stats": {
            "safe_percentage": float(100.0 * safe_matrix.mean()),
            "mean_closest_time_min": float(np.mean(time_matrix_mins)),
        },
        "lat_bounds": [float(b_lats[0]), float(b_lats[-1])],
        "lon_bounds": [float(b_lons[0]), float(b_lons[-1])],
        "view_half_extent_nm": float(params.view_half_extent_nm),
    }
    grid_payload.update(_aircraft_payload(params))
    return grid_payload


def _segment_points(
    origin: np.ndarray,
    heading_vec: np.ndarray,
    speed_kt: float,
    speed_diff_kt: float,
    time_s: float,
) -> tuple[np.ndarray, np.ndarray]:
    min_speed = max(speed_kt - speed_diff_kt, 0.0)
    max_speed = speed_kt + speed_diff_kt
    slow_point = origin + heading_vec * (min_speed * KT_TO_MPS * time_s)
    fast_point = origin + heading_vec * (max_speed * KT_TO_MPS * time_s)
    return slow_point, fast_point


def _fixed_heading_time_payload(params: VisualParams) -> dict[str, Any]:
    separation_threshold_m = params.separation_threshold_nm * NMI_TO_M
    projection_time_s = params.projection_time_min * 60.0

    is_sep, min_distance_m, closest_time_s = catch_up_projection_interval(
        a_lat=params.a_lat,
        a_lon=params.a_lon,
        a_heading=params.a_heading,
        a_speed_kt=params.a_speed_kt,
        b_lat=params.b_lat,
        b_lon=params.b_lon,
        b_heading=params.b_heading,
        b_speed_kt=params.b_speed_kt,
        separation_threshold_m=separation_threshold_m,
        speed_diff_kt=params.speed_diff_kt,
        projection_time_s=projection_time_s,
    )

    a_origin = np.zeros(2, dtype=np.float64)
    b_origin = latlon_to_local_xy(params.b_lat, params.b_lon, ref_lat=params.a_lat, ref_lon=params.a_lon)
    a_heading_vec = heading_to_unit_vector(params.a_heading)
    b_heading_vec = heading_to_unit_vector(params.b_heading)
    step_s = 1.0
    times = list(np.arange(0.0, projection_time_s, step_s))
    if not times or (projection_time_s - times[-1]) > 1e-6:
        times.append(projection_time_s)

    frames: list[dict[str, Any]] = []
    points_for_extent: list[np.ndarray] = [a_origin.copy(), b_origin.copy()]

    for t in times:
        a_slow, a_fast = _segment_points(a_origin, a_heading_vec, params.a_speed_kt, params.speed_diff_kt, t)
        b_slow, b_fast = _segment_points(b_origin, b_heading_vec, params.b_speed_kt, params.speed_diff_kt, t)
        frames.append(
            {
                "time_s": float(t),
                "time_min": float(t / 60.0),
                "a": {"slow": a_slow.tolist(), "fast": a_fast.tolist()},
                "b": {"slow": b_slow.tolist(), "fast": b_fast.tolist()},
            }
        )
        points_for_extent.extend((a_slow, a_fast, b_slow, b_fast))

    xy_bounds = _local_bounds_from_grid(params, padding_m=separation_threshold_m)
    extent_x_min = min(float(p[0]) for p in points_for_extent) - separation_threshold_m
    extent_x_max = max(float(p[0]) for p in points_for_extent) + separation_threshold_m
    extent_y_min = min(float(p[1]) for p in points_for_extent) - separation_threshold_m
    extent_y_max = max(float(p[1]) for p in points_for_extent) + separation_threshold_m
    xy_bounds["x_min"] = min(xy_bounds["x_min"], extent_x_min)
    xy_bounds["x_max"] = max(xy_bounds["x_max"], extent_x_max)
    xy_bounds["y_min"] = min(xy_bounds["y_min"], extent_y_min)
    xy_bounds["y_max"] = max(xy_bounds["y_max"], extent_y_max)

    return {
        "model_mode": "fixed_heading",
        "turn_preset_id": params.turn_preset_id,
        "origins": {"a": a_origin.tolist(), "b": b_origin.tolist()},
        "frames": frames,
        "frame_interval_s": step_s,
        "bounds_xy": xy_bounds,
        "stats": {
            "is_separated": bool(is_sep),
            "min_distance_nm": float(min_distance_m / NMI_TO_M),
            "closest_time_min": float(closest_time_s / 60.0),
        },
        "separation_threshold_m": float(separation_threshold_m),
        "lat_bounds": list(_grid_bounds(params)[0]),
        "lon_bounds": list(_grid_bounds(params)[1]),
        "view_half_extent_nm": float(params.view_half_extent_nm),
    }


def _turn_aware_time_payload(params: VisualParams) -> dict[str, Any]:
    payload = build_turn_debug_payload(
        a_lat=params.a_lat,
        a_lon=params.a_lon,
        a_heading0_deg=params.a_heading,
        a_target_heading_deg=params.a_target_heading_deg,
        a_speed_kt=params.a_speed_kt,
        a_turn_rate_deg_sec=params.a_turn_rate_deg_sec,
        b_lat=params.b_lat,
        b_lon=params.b_lon,
        b_heading0_deg=params.b_heading,
        b_target_heading_deg=params.b_target_heading_deg,
        b_speed_kt=params.b_speed_kt,
        b_turn_rate_deg_sec=params.b_turn_rate_deg_sec,
        separation_threshold_m=params.separation_threshold_nm * NMI_TO_M,
        speed_diff_kt=params.speed_diff_kt,
        projection_time_s=params.projection_time_min * 60.0,
        turn_speed_schedule_uncertainty_kt=params.turn_speed_schedule_uncertainty_kt,
    )
    payload["turn_preset_id"] = params.turn_preset_id
    payload["lat_bounds"] = list(_grid_bounds(params)[0])
    payload["lon_bounds"] = list(_grid_bounds(params)[1])
    payload["view_half_extent_nm"] = float(params.view_half_extent_nm)
    payload.update(_aircraft_payload(params))
    return payload


def _compute_grid_payload(params: VisualParams) -> dict[str, Any]:
    if params.model_mode == "turn_aware":
        return _turn_aware_grid_payload(params)
    return _fixed_heading_grid_payload(params)


def _time_series_payload(params: VisualParams) -> dict[str, Any]:
    if params.model_mode == "turn_aware":
        return _turn_aware_time_payload(params)
    return _fixed_heading_time_payload(params)


app = FastAPI(title="Relevant-Aircraft Visualiser")
app.mount("/static", StaticFiles(directory=STATIC_DIR), name="relevant-aircraft-static")


@app.get("/", response_class=HTMLResponse)
def index(request: Request) -> HTMLResponse:
    context = {
        "request": request,
        "defaults": DEFAULT_UI_VALUES,
        "turn_presets": get_turn_preset_options(),
    }
    return templates.TemplateResponse(request, "relevant_aircraft.html", context)


@app.post("/api/grid")
def grid_visual(params: VisualParams) -> dict[str, Any]:
    return _compute_grid_payload(params)


@app.post("/api/time_sweep")
def time_sweep_visual(params: VisualParams) -> dict[str, Any]:
    return _time_series_payload(params)


def get_app() -> FastAPI:
    return app


if __name__ == "__main__":
    import uvicorn

    uvicorn.run("apps.relevant_aircraft.relevant_aircraft_app:app", host="0.0.0.0", port=8008, reload=False)
