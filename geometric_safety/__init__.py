"""Geometric reachability-based safety filtering for air traffic control."""

from geometric_safety.relevant_aircraft import (
    # Constants
    DEG_TO_RAD,
    EARTH_RADIUS_IN_METERS,
    KT_TO_MPS,
    RAD_TO_DEG,
    TURN_HEADING_EPS_DEG,
    TURN_RATE_EPS_DEG_PER_S,
    TURN_TIME_CERT_MIN_INTERVAL_S,
    # Core solvers
    catch_up_projection_interval,
    catch_up_projection_interval_with_turns,
    compute_relative_velocity_hull,
    # Geometry utilities
    heading_diff,
    heading_to_unit_vector,
    latlon_to_local_xy,
    local_xy_to_latlon,
    # Demo/app helpers
    CUSTOM_TURN_PRESET_ID,
    TURN_DEMO_SCENARIOS,
    TurnDemoScenario,
    build_turn_debug_payload,
    get_turn_preset_options,
    heading_summary,
    speed_bounds_mps,
    turn_duration_s,
)

NMI_TO_M = 1852.0
M_TO_NMI = 1.0 / NMI_TO_M
