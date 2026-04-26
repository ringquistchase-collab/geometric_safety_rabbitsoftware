import math

import pytest

from geometric_safety import (
    VERTICAL_OVERLAP_NEVER_RESOLVES_S,
    catch_up_projection_interval_with_turns_and_vertical,
    catch_up_projection_interval_with_vertical,
    time_to_vertical_overlap_resolution,
    vertical_band_gap_fl,
    vertical_bands_are_resolved,
)
from geometric_safety.relevant_aircraft import catch_up_projection_interval
from geometric_safety.util import DEG_TO_RAD, EARTH_RADIUS_IN_METERS, NMI_TO_M, RAD_TO_DEG


def _local_xy_to_latlon(east_m: float, north_m: float, ref_lat: float, ref_lon: float) -> tuple[float, float]:
    ref_lat_rad = ref_lat * DEG_TO_RAD
    lat = ref_lat + (north_m / EARTH_RADIUS_IN_METERS) * RAD_TO_DEG
    lon = ref_lon + (east_m / (EARTH_RADIUS_IN_METERS * math.cos(ref_lat_rad))) * RAD_TO_DEG
    return lat, lon


class TestVerticalOverlapResolution:
    def test_already_distinct_bands_return_zero(self):
        assert vertical_band_gap_fl(100.0, 100.0, 120.0, 120.0) == pytest.approx(20.0)
        assert vertical_bands_are_resolved(100.0, 100.0, 120.0, 120.0)
        assert time_to_vertical_overlap_resolution(100.0, 100.0, 120.0, 120.0) == pytest.approx(0.0)

    def test_both_climbing_resolves_when_lower_band_edge_clears(self):
        resolution_s = time_to_vertical_overlap_resolution(
            a_current_fl=100.0,
            a_selected_fl=150.0,
            b_current_fl=140.0,
            b_selected_fl=200.0,
            vertical_rate_fpm=1000.0,
            required_gap_fl=10.0,
        )
        assert resolution_s == pytest.approx(120.0)

    def test_both_descending_resolves_when_upper_band_edge_clears(self):
        resolution_s = time_to_vertical_overlap_resolution(
            a_current_fl=200.0,
            a_selected_fl=140.0,
            b_current_fl=150.0,
            b_selected_fl=100.0,
            vertical_rate_fpm=1000.0,
            required_gap_fl=10.0,
        )
        assert resolution_s == pytest.approx(120.0)

    def test_opposite_climb_and_descent_resolves_in_closed_form(self):
        resolution_s = time_to_vertical_overlap_resolution(
            a_current_fl=100.0,
            a_selected_fl=200.0,
            b_current_fl=250.0,
            b_selected_fl=150.0,
            vertical_rate_fpm=1000.0,
            required_gap_fl=10.0,
        )
        assert resolution_s == pytest.approx(480.0)

    def test_one_aircraft_levels_before_resolution(self):
        resolution_s = time_to_vertical_overlap_resolution(
            a_current_fl=100.0,
            a_selected_fl=150.0,
            b_current_fl=200.0,
            b_selected_fl=140.0,
            vertical_rate_fpm=1000.0,
            required_gap_fl=10.0,
        )
        assert resolution_s == pytest.approx(360.0)

    def test_rate_parameter_scales_time(self):
        slow_resolution_s = time_to_vertical_overlap_resolution(
            a_current_fl=100.0,
            a_selected_fl=100.0,
            b_current_fl=100.0,
            b_selected_fl=120.0,
            vertical_rate_fpm=500.0,
            required_gap_fl=10.0,
        )
        fast_resolution_s = time_to_vertical_overlap_resolution(
            a_current_fl=100.0,
            a_selected_fl=100.0,
            b_current_fl=100.0,
            b_selected_fl=120.0,
            vertical_rate_fpm=2000.0,
            required_gap_fl=10.0,
        )

        assert slow_resolution_s == pytest.approx(120.0)
        assert fast_resolution_s == pytest.approx(30.0)

    def test_returns_never_sentinel_when_selected_bands_never_clear(self):
        resolution_s = time_to_vertical_overlap_resolution(
            a_current_fl=100.0,
            a_selected_fl=150.0,
            b_current_fl=200.0,
            b_selected_fl=145.0,
            vertical_rate_fpm=1000.0,
            required_gap_fl=10.0,
        )
        assert resolution_s == pytest.approx(VERTICAL_OVERLAP_NEVER_RESOLVES_S)

    def test_zero_rate_keeps_unresolved_overlap_forever(self):
        resolution_s = time_to_vertical_overlap_resolution(
            a_current_fl=100.0,
            a_selected_fl=100.0,
            b_current_fl=105.0,
            b_selected_fl=105.0,
            vertical_rate_fpm=0.0,
            required_gap_fl=10.0,
        )
        assert resolution_s == pytest.approx(VERTICAL_OVERLAP_NEVER_RESOLVES_S)

    def test_negative_rate_is_treated_as_magnitude(self):
        positive_rate_s = time_to_vertical_overlap_resolution(
            a_current_fl=100.0,
            a_selected_fl=100.0,
            b_current_fl=100.0,
            b_selected_fl=120.0,
            vertical_rate_fpm=1000.0,
            required_gap_fl=10.0,
        )
        negative_rate_s = time_to_vertical_overlap_resolution(
            a_current_fl=100.0,
            a_selected_fl=100.0,
            b_current_fl=100.0,
            b_selected_fl=120.0,
            vertical_rate_fpm=-1000.0,
            required_gap_fl=10.0,
        )

        assert negative_rate_s == pytest.approx(positive_rate_s)


class TestCombinedSafety:
    def test_vertical_resolution_before_lateral_loss_is_safe(self):
        a_lat, a_lon = 51.0, -1.0
        b_lat, b_lon = _local_xy_to_latlon(0.0, 20.0 * NMI_TO_M, a_lat, a_lon)
        separation_m = 5.0 * NMI_TO_M

        lateral_is_sep, _, _ = catch_up_projection_interval(
            a_lat=a_lat,
            a_lon=a_lon,
            a_heading=0.0,
            a_speed_kt=360.0,
            b_lat=b_lat,
            b_lon=b_lon,
            b_heading=180.0,
            b_speed_kt=360.0,
            separation_threshold_m=separation_m,
            speed_diff_kt=0.0,
            projection_time_s=600.0,
        )
        assert not lateral_is_sep

        combined_is_safe, min_distance_m, closest_time_s, vertical_time_s = catch_up_projection_interval_with_vertical(
            a_lat=a_lat,
            a_lon=a_lon,
            a_heading=0.0,
            a_speed_kt=360.0,
            a_current_fl=100.0,
            a_selected_fl=100.0,
            b_lat=b_lat,
            b_lon=b_lon,
            b_heading=180.0,
            b_speed_kt=360.0,
            b_current_fl=100.0,
            b_selected_fl=120.0,
            separation_threshold_m=separation_m,
            speed_diff_kt=0.0,
            projection_time_s=600.0,
            vertical_rate_fpm=1000.0,
        )

        assert combined_is_safe
        assert vertical_time_s == pytest.approx(60.0)
        assert min_distance_m > separation_m
        assert closest_time_s == pytest.approx(60.0)

    def test_lateral_loss_before_vertical_resolution_is_unsafe(self):
        a_lat, a_lon = 51.0, -1.0
        b_lat, b_lon = _local_xy_to_latlon(0.0, 20.0 * NMI_TO_M, a_lat, a_lon)
        separation_m = 5.0 * NMI_TO_M

        combined_is_safe, min_distance_m, _, vertical_time_s = catch_up_projection_interval_with_vertical(
            a_lat=a_lat,
            a_lon=a_lon,
            a_heading=0.0,
            a_speed_kt=360.0,
            a_current_fl=100.0,
            a_selected_fl=100.0,
            b_lat=b_lat,
            b_lon=b_lon,
            b_heading=180.0,
            b_speed_kt=360.0,
            b_current_fl=100.0,
            b_selected_fl=120.0,
            separation_threshold_m=separation_m,
            speed_diff_kt=0.0,
            projection_time_s=600.0,
            vertical_rate_fpm=500.0,
        )

        assert not combined_is_safe
        assert vertical_time_s == pytest.approx(120.0)
        assert min_distance_m < separation_m

    def test_already_resolved_vertical_bands_shortcut_to_safe(self):
        is_safe, min_distance_m, closest_time_s, vertical_time_s = catch_up_projection_interval_with_vertical(
            a_lat=51.0,
            a_lon=-1.0,
            a_heading=0.0,
            a_speed_kt=360.0,
            a_current_fl=100.0,
            a_selected_fl=100.0,
            b_lat=51.0,
            b_lon=-1.0,
            b_heading=180.0,
            b_speed_kt=360.0,
            b_current_fl=120.0,
            b_selected_fl=120.0,
            separation_threshold_m=5.0 * NMI_TO_M,
            speed_diff_kt=0.0,
            projection_time_s=600.0,
        )

        assert is_safe
        assert min_distance_m == pytest.approx(0.0)
        assert closest_time_s == pytest.approx(0.0)
        assert vertical_time_s == pytest.approx(0.0)

    def test_never_resolving_vertical_bands_use_full_lateral_horizon(self):
        a_lat, a_lon = 51.0, -1.0
        b_lat, b_lon = _local_xy_to_latlon(0.0, 20.0 * NMI_TO_M, a_lat, a_lon)
        separation_m = 5.0 * NMI_TO_M

        combined_is_safe, min_distance_m, _, vertical_time_s = catch_up_projection_interval_with_vertical(
            a_lat=a_lat,
            a_lon=a_lon,
            a_heading=0.0,
            a_speed_kt=360.0,
            a_current_fl=100.0,
            a_selected_fl=150.0,
            b_lat=b_lat,
            b_lon=b_lon,
            b_heading=180.0,
            b_speed_kt=360.0,
            b_current_fl=200.0,
            b_selected_fl=145.0,
            separation_threshold_m=separation_m,
            speed_diff_kt=0.0,
            projection_time_s=600.0,
            vertical_rate_fpm=1000.0,
        )

        assert not combined_is_safe
        assert vertical_time_s == pytest.approx(VERTICAL_OVERLAP_NEVER_RESOLVES_S)
        assert min_distance_m < separation_m

    def test_turn_aware_combined_matches_fixed_heading_without_turns(self):
        a_lat, a_lon = 51.0, -1.0
        b_lat, b_lon = _local_xy_to_latlon(0.0, 20.0 * NMI_TO_M, a_lat, a_lon)
        separation_m = 5.0 * NMI_TO_M

        fixed = catch_up_projection_interval_with_vertical(
            a_lat=a_lat,
            a_lon=a_lon,
            a_heading=0.0,
            a_speed_kt=360.0,
            a_current_fl=100.0,
            a_selected_fl=100.0,
            b_lat=b_lat,
            b_lon=b_lon,
            b_heading=180.0,
            b_speed_kt=360.0,
            b_current_fl=100.0,
            b_selected_fl=120.0,
            separation_threshold_m=separation_m,
            speed_diff_kt=0.0,
            projection_time_s=600.0,
            vertical_rate_fpm=1000.0,
        )
        turn_aware = catch_up_projection_interval_with_turns_and_vertical(
            a_lat=a_lat,
            a_lon=a_lon,
            a_heading0_deg=0.0,
            a_target_heading_deg=0.0,
            a_speed_kt=360.0,
            a_turn_rate_deg_sec=0.0,
            a_current_fl=100.0,
            a_selected_fl=100.0,
            b_lat=b_lat,
            b_lon=b_lon,
            b_heading0_deg=180.0,
            b_target_heading_deg=180.0,
            b_speed_kt=360.0,
            b_turn_rate_deg_sec=0.0,
            b_current_fl=100.0,
            b_selected_fl=120.0,
            separation_threshold_m=separation_m,
            speed_diff_kt=0.0,
            projection_time_s=600.0,
            vertical_rate_fpm=1000.0,
        )

        assert turn_aware[0] == fixed[0]
        assert turn_aware[1] == pytest.approx(fixed[1], abs=1e-6)
        assert turn_aware[2] == pytest.approx(fixed[2], abs=1e-6)
        assert turn_aware[3] == pytest.approx(fixed[3], abs=1e-6)
