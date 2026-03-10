from __future__ import annotations

import json
import math
import re

import pytest

pytest.importorskip("fastapi")
from fastapi.testclient import TestClient

from apps.relevant_aircraft.relevant_aircraft_app import app
from geometric_safety.demo import (
    TURN_DEMO_SCENARIOS,
    build_turn_debug_payload,
    get_turn_preset_options,
    heading_summary,
    turn_duration_s,
)
from geometric_safety.relevant_aircraft import catch_up_projection_interval_with_turns
from geometric_safety.util import NMI_TO_M


def fixed_heading_payload() -> dict[str, float | int | str]:
    return {
        "model_mode": "fixed_heading",
        "turn_preset_id": "custom",
        "separation_threshold_nm": 5.0,
        "speed_diff_kt": 40.0,
        "projection_time_min": 10.0,
        "grid_density": 25,
        "view_half_extent_nm": 20.0,
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


def turn_preset_payload(preset_id: str = "single_turn_dodge") -> dict[str, float | int | str]:
    preset = next(option for option in get_turn_preset_options() if option["id"] == preset_id)
    assert preset["params"] is not None
    payload = dict(preset["params"])
    payload["grid_density"] = 25
    return payload


@pytest.fixture
def client() -> TestClient:
    return TestClient(app)


def test_fixed_heading_grid_payload_shape(client: TestClient) -> None:
    response = client.post("/api/grid", json=fixed_heading_payload())
    assert response.status_code == 200
    data = response.json()

    assert data["model_mode"] == "fixed_heading"
    assert len(data["lats"]) == 25
    assert len(data["lons"]) == 25
    assert len(data["safe"]) == 25
    assert len(data["closest_time_min"]) == 25
    assert isinstance(data["safe"][0][0], bool)
    assert "safe_percentage" in data["stats"]
    assert data["a"]["heading"] == pytest.approx(0.0)
    assert data["b"]["heading"] == pytest.approx(90.0)


def test_turn_aware_grid_payload_shape(client: TestClient) -> None:
    payload = turn_preset_payload("mutual_closing_turns")
    response = client.post("/api/grid", json=payload)
    assert response.status_code == 200
    data = response.json()

    assert data["model_mode"] == "turn_aware"
    assert data["turn_preset_id"] == "mutual_closing_turns"
    assert len(data["lats"]) == 25
    assert len(data["safe"]) == 25
    assert all(len(row) == 25 for row in data["safe"])
    assert all(len(row) == 25 for row in data["closest_time_min"])
    assert data["a"]["target_heading_deg"] == pytest.approx(payload["a_target_heading_deg"])
    assert data["b"]["turn_rate_deg_sec"] == pytest.approx(payload["b_turn_rate_deg_sec"])


def test_turn_aware_time_sweep_payload_shape(client: TestClient) -> None:
    payload = turn_preset_payload("single_turn_cut_in")
    response = client.post("/api/time_sweep", json=payload)
    assert response.status_code == 200
    data = response.json()

    assert data["model_mode"] == "turn_aware"
    assert data["turn_preset_id"] == "single_turn_cut_in"
    assert data["frames"]
    assert data["trajectories"]["times_s"]
    assert len(data["trajectories"]["times_s"]) == len(data["trajectories"]["a"]["nominal"])
    assert len(data["trajectories"]["times_s"]) == len(data["trajectories"]["b"]["max"])
    assert 0 <= data["closest_frame_index"] < len(data["frames"])
    assert data["frames"][0]["relative_hull"]
    assert math.isfinite(data["summary"]["min_distance_nm"])
    assert math.isfinite(data["summary"]["closest_time_min"])
    assert math.isfinite(data["distance_curve"]["threshold_nm"])
    assert set(data["bounds_xy"]) >= {"x_min", "x_max", "y_min", "y_max"}
    assert set(data["relative_bounds_xy"]) >= {"x_min", "x_max", "y_min", "y_max"}


def test_fixed_heading_time_sweep_payload_shape(client: TestClient) -> None:
    response = client.post("/api/time_sweep", json=fixed_heading_payload())
    assert response.status_code == 200
    data = response.json()

    assert data["model_mode"] == "fixed_heading"
    assert data["turn_preset_id"] == "custom"
    assert data["frames"]
    assert data["frame_interval_s"] == pytest.approx(1.0)
    assert data["origins"]["a"] == pytest.approx([0.0, 0.0])
    assert len(data["frames"][0]["a"]["slow"]) == 2
    assert len(data["frames"][0]["b"]["fast"]) == 2
    assert set(data["bounds_xy"]) >= {"x_min", "x_max", "y_min", "y_max"}
    assert math.isfinite(data["stats"]["min_distance_nm"])
    assert math.isfinite(data["stats"]["closest_time_min"])


def test_turn_demo_payload_matches_direct_solver() -> None:
    scenario = next(scenario for scenario in TURN_DEMO_SCENARIOS if scenario.preset_id == "single_turn_dodge")
    b_lat, b_lon = scenario.b_latlon()
    payload = build_turn_debug_payload(
        a_lat=scenario.a_lat,
        a_lon=scenario.a_lon,
        a_heading0_deg=scenario.a_heading_deg,
        a_target_heading_deg=scenario.a_target_heading_deg,
        a_speed_kt=scenario.a_speed_kt,
        a_turn_rate_deg_sec=scenario.a_turn_rate_deg_sec,
        b_lat=b_lat,
        b_lon=b_lon,
        b_heading0_deg=scenario.b_heading_deg,
        b_target_heading_deg=scenario.b_target_heading_deg,
        b_speed_kt=scenario.b_speed_kt,
        b_turn_rate_deg_sec=scenario.b_turn_rate_deg_sec,
        separation_threshold_m=scenario.separation_threshold_nm * NMI_TO_M,
        speed_diff_kt=scenario.speed_diff_kt,
        projection_time_s=scenario.projection_time_min * 60.0,
    )
    direct_is_sep, _, _ = catch_up_projection_interval_with_turns(
        a_lat=scenario.a_lat,
        a_lon=scenario.a_lon,
        a_heading0_deg=scenario.a_heading_deg,
        a_target_heading_deg=scenario.a_target_heading_deg,
        a_speed_kt=scenario.a_speed_kt,
        a_turn_rate_deg_sec=scenario.a_turn_rate_deg_sec,
        b_lat=b_lat,
        b_lon=b_lon,
        b_heading0_deg=scenario.b_heading_deg,
        b_target_heading_deg=scenario.b_target_heading_deg,
        b_speed_kt=scenario.b_speed_kt,
        b_turn_rate_deg_sec=scenario.b_turn_rate_deg_sec,
        separation_threshold_m=scenario.separation_threshold_nm * NMI_TO_M,
        speed_diff_kt=scenario.speed_diff_kt,
        projection_time_s=scenario.projection_time_min * 60.0,
    )

    assert payload["summary"]["is_separated"] is direct_is_sep
    assert payload["turns"]["a"]["end_time_s"] == pytest.approx(
        turn_duration_s(scenario.a_heading_deg, scenario.a_target_heading_deg, scenario.a_turn_rate_deg_sec)
    )
    assert payload["turns"]["b"]["end_time_s"] == pytest.approx(
        turn_duration_s(scenario.b_heading_deg, scenario.b_target_heading_deg, scenario.b_turn_rate_deg_sec)
    )


def test_turn_preset_round_trip_through_endpoint_model(client: TestClient) -> None:
    payload = turn_preset_payload("mutual_opening_turns")
    response = client.post("/api/time_sweep", json=payload)
    assert response.status_code == 200
    data = response.json()

    assert data["turn_preset_id"] == "mutual_opening_turns"
    assert data["a"]["target_heading_deg"] == pytest.approx(payload["a_target_heading_deg"])
    assert data["b"]["target_heading_deg"] == pytest.approx(payload["b_target_heading_deg"])


def test_heading_summary_marks_zero_rate_heading_change_as_instant() -> None:
    assert heading_summary(90.0, 140.0, 0.0, 340.0) == "instant 90->140 deg @ 340 kt"


def test_index_template_embeds_turn_presets_and_full_heading_ranges(client: TestClient) -> None:
    response = client.get("/")
    assert response.status_code == 200
    html = response.text

    preset_match = re.search(
        r'<script id="turnPresetData" type="application/json">\s*(.*?)\s*</script>',
        html,
        flags=re.DOTALL,
    )
    assert preset_match is not None
    preset_data = json.loads(preset_match.group(1))
    mutual_opening = next(option for option in preset_data if option["id"] == "mutual_opening_turns")
    assert mutual_opening["params"]["b_heading"] == pytest.approx(225.0)
    assert mutual_opening["params"]["b_target_heading_deg"] == pytest.approx(270.0)
    assert mutual_opening["params"]["b_turn_rate_deg_sec"] == pytest.approx(1.0)

    for slider_id in ("aHeadingSlider", "bHeadingSlider", "aTargetHeadingSlider", "bTargetHeadingSlider"):
        assert re.search(rf'id="{slider_id}"[^>]*min="0"[^>]*max="360"', html, flags=re.DOTALL)
