"""Geometric reachability-based safety filtering for air traffic control."""

from geometric_safety.relevant_aircraft import (
    DEG_TO_RAD as DEG_TO_RAD,
    EARTH_RADIUS_IN_METERS as EARTH_RADIUS_IN_METERS,
    KT_TO_MPS as KT_TO_MPS,
    RAD_TO_DEG as RAD_TO_DEG,
    TURN_HEADING_EPS_DEG as TURN_HEADING_EPS_DEG,
    TURN_RATE_EPS_DEG_PER_S as TURN_RATE_EPS_DEG_PER_S,
    TURN_TIME_CERT_MIN_INTERVAL_S as TURN_TIME_CERT_MIN_INTERVAL_S,
    catch_up_projection_interval as catch_up_projection_interval,
    catch_up_projection_interval_with_turns as catch_up_projection_interval_with_turns,
    compute_relative_velocity_hull as compute_relative_velocity_hull,
    heading_diff as heading_diff,
    heading_to_unit_vector as heading_to_unit_vector,
    latlon_to_local_xy as latlon_to_local_xy,
)

NMI_TO_M = 1852.0
M_TO_NMI = 1.0 / NMI_TO_M
