"""Vertical cleared-band timing for aircraft level changes."""

import numba

# Levels are represented directly as flight levels throughout this module. The only
# physical unit conversion needed is the user-facing climb/descent rate in feet/minute.
FT_PER_FL = 100.0
SECONDS_PER_MINUTE = 60.0
DEFAULT_VERTICAL_RATE_FPM = 1000.0
DEFAULT_VERTICAL_SEPARATION_FL = 10.0

# A real resolution time is always non-negative, so a negative sentinel can represent
# "never resolves" without relying on infinity semantics under Numba fast-math.
VERTICAL_OVERLAP_NEVER_RESOLVES_S = -1.0


@numba.njit(cache=True, fastmath=True)
def _rate_fl_per_second(vertical_rate_fpm: float) -> float:
    """Convert a climb/descent rate in feet per minute to flight levels per second."""
    if vertical_rate_fpm <= 0.0:
        return 0.0
    return vertical_rate_fpm / FT_PER_FL / SECONDS_PER_MINUTE


@numba.njit(cache=True, fastmath=True)
def level_at_time_fl(current_fl: float, selected_fl: float, vertical_rate_fpm: float, t_s: float) -> float:
    """
    Return the aircraft level after flying toward its selected level for `t_s`.

    Levels are expressed as flight levels, so FL350 is passed as `350.0`. The vertical
    rate is expressed in feet per minute and is applied with the sign required to move
    from the current level to the selected level. Once selected level is reached, the
    aircraft remains there.

    Parameters
    ----------
    current_fl : float
        Aircraft current flight level.
    selected_fl : float
        Aircraft selected flight level.
    vertical_rate_fpm : float
        Climb/descent rate in feet per minute.
    t_s : float
        Elapsed time in seconds.

    Returns
    -------
    float
        Aircraft flight level at `t_s`.
    """
    # The vertical model is deliberately simple: fly at constant vertical rate toward
    # selected level, then clamp there once the selected level has been reached.
    rate_flps = _rate_fl_per_second(vertical_rate_fpm)
    if rate_flps <= 0.0 or t_s <= 0.0 or current_fl == selected_fl:
        return current_fl

    delta_fl = selected_fl - current_fl
    max_step_fl = rate_flps * t_s
    if abs(delta_fl) <= max_step_fl:
        return selected_fl
    if delta_fl > 0.0:
        return current_fl + max_step_fl
    return current_fl - max_step_fl


@numba.njit(cache=True, fastmath=True)
def _vertical_band_at_time_fl(
    current_fl: float, selected_fl: float, vertical_rate_fpm: float, t_s: float
) -> tuple[float, float]:
    """Return the remaining cleared-to-selected level band at time `t_s`."""
    # At time t, the aircraft still occupies the controller-relevant interval from its
    # instantaneous level to its selected level. The order depends on climb vs descent.
    level_fl = level_at_time_fl(current_fl, selected_fl, vertical_rate_fpm, t_s)
    if level_fl <= selected_fl:
        return level_fl, selected_fl
    return selected_fl, level_fl


@numba.njit(cache=True, fastmath=True)
def vertical_band_gap_fl(
    a_current_fl: float,
    a_selected_fl: float,
    b_current_fl: float,
    b_selected_fl: float,
    vertical_rate_fpm: float = DEFAULT_VERTICAL_RATE_FPM,
    t_s: float = 0.0,
) -> float:
    """
    Return the signed gap between the two remaining vertical bands.

    A non-negative value means the bands no longer overlap. A value of `10.0`, for
    example, means the bands are separated by 10 flight levels, i.e. 1000 ft. When the
    bands overlap, the returned value is negative by the amount of overlap.

    Parameters
    ----------
    a_current_fl : float
        Aircraft A current flight level.
    a_selected_fl : float
        Aircraft A selected flight level.
    b_current_fl : float
        Aircraft B current flight level.
    b_selected_fl : float
        Aircraft B selected flight level.
    vertical_rate_fpm : float, optional
        Climb/descent rate in feet per minute, used for both aircraft.
    t_s : float, optional
        Elapsed time in seconds.

    Returns
    -------
    float
        Signed gap between the remaining vertical bands, in flight levels.
    """
    # Either B can be above A, or A can be above B. The signed gap is whichever of those
    # two directional gaps is larger; negative values mean both directions still overlap.
    a_low_fl, a_high_fl = _vertical_band_at_time_fl(a_current_fl, a_selected_fl, vertical_rate_fpm, t_s)
    b_low_fl, b_high_fl = _vertical_band_at_time_fl(b_current_fl, b_selected_fl, vertical_rate_fpm, t_s)
    b_above_a_gap_fl = b_low_fl - a_high_fl
    a_above_b_gap_fl = a_low_fl - b_high_fl
    return max(b_above_a_gap_fl, a_above_b_gap_fl)


@numba.njit(cache=True, fastmath=True)
def vertical_bands_are_resolved(
    a_current_fl: float,
    a_selected_fl: float,
    b_current_fl: float,
    b_selected_fl: float,
    required_gap_fl: float = DEFAULT_VERTICAL_SEPARATION_FL,
    vertical_rate_fpm: float = DEFAULT_VERTICAL_RATE_FPM,
    t_s: float = 0.0,
) -> bool:
    """
    Return whether the remaining vertical bands are distinct by `required_gap_fl`.

    Parameters
    ----------
    a_current_fl : float
        Aircraft A current flight level.
    a_selected_fl : float
        Aircraft A selected flight level.
    b_current_fl : float
        Aircraft B current flight level.
    b_selected_fl : float
        Aircraft B selected flight level.
    required_gap_fl : float, optional
        Required gap between the two remaining vertical bands, in flight levels.
    vertical_rate_fpm : float, optional
        Climb/descent rate in feet per minute, used for both aircraft.
    t_s : float, optional
        Elapsed time in seconds.

    Returns
    -------
    bool
        True if the remaining vertical bands are separated by at least
        `required_gap_fl`.
    """
    if required_gap_fl < 0.0:
        required_gap_fl = 0.0
    gap_fl = vertical_band_gap_fl(
        a_current_fl=a_current_fl,
        a_selected_fl=a_selected_fl,
        b_current_fl=b_current_fl,
        b_selected_fl=b_selected_fl,
        vertical_rate_fpm=vertical_rate_fpm,
        t_s=t_s,
    )
    return gap_fl >= required_gap_fl


@numba.njit(cache=True, fastmath=True)
def _time_to_selected_s(current_fl: float, selected_fl: float, vertical_rate_fpm: float) -> float:
    """Return how long the aircraft takes to reach selected level."""
    # A zero rate is only valid if there is no level change to make. Otherwise the
    # aircraft never reaches selected level under this model.
    rate_flps = _rate_fl_per_second(vertical_rate_fpm)
    if rate_flps <= 0.0:
        if current_fl == selected_fl:
            return 0.0
        return VERTICAL_OVERLAP_NEVER_RESOLVES_S
    return abs(selected_fl - current_fl) / rate_flps


@numba.njit(cache=True, fastmath=True)
def _first_directional_gap_crossing_in_interval_s(
    a_current_fl: float,
    a_selected_fl: float,
    b_current_fl: float,
    b_selected_fl: float,
    vertical_rate_fpm: float,
    required_gap_fl: float,
    start_s: float,
    end_s: float,
    b_above_a: bool,
) -> float:
    """Return when one directional band gap first reaches `required_gap_fl`."""
    # Between two adjacent "aircraft reaches selected level" events, each band edge is
    # linear in time, so one directional gap is linear too. A single interpolation gives
    # the first crossing if the interval endpoints bracket the required gap.
    start_a_low_fl, start_a_high_fl = _vertical_band_at_time_fl(a_current_fl, a_selected_fl, vertical_rate_fpm, start_s)
    start_b_low_fl, start_b_high_fl = _vertical_band_at_time_fl(b_current_fl, b_selected_fl, vertical_rate_fpm, start_s)
    start_gap_fl = start_b_low_fl - start_a_high_fl if b_above_a else start_a_low_fl - start_b_high_fl
    if start_gap_fl >= required_gap_fl:
        return start_s

    if end_s <= start_s:
        return VERTICAL_OVERLAP_NEVER_RESOLVES_S

    end_a_low_fl, end_a_high_fl = _vertical_band_at_time_fl(a_current_fl, a_selected_fl, vertical_rate_fpm, end_s)
    end_b_low_fl, end_b_high_fl = _vertical_band_at_time_fl(b_current_fl, b_selected_fl, vertical_rate_fpm, end_s)
    end_gap_fl = end_b_low_fl - end_a_high_fl if b_above_a else end_a_low_fl - end_b_high_fl
    if end_gap_fl < required_gap_fl or end_gap_fl <= start_gap_fl:
        return VERTICAL_OVERLAP_NEVER_RESOLVES_S

    return start_s + (required_gap_fl - start_gap_fl) * (end_s - start_s) / (end_gap_fl - start_gap_fl)


@numba.njit(cache=True, fastmath=True)
def time_to_vertical_overlap_resolution(
    a_current_fl: float,
    a_selected_fl: float,
    b_current_fl: float,
    b_selected_fl: float,
    vertical_rate_fpm: float = DEFAULT_VERTICAL_RATE_FPM,
    required_gap_fl: float = DEFAULT_VERTICAL_SEPARATION_FL,
) -> float:
    """
    Return the first time at which the remaining vertical bands are resolved.

    Each aircraft is assumed to fly from current level toward selected level at
    `vertical_rate_fpm`, stopping once selected level is reached. At any time, its
    vertical band is the interval between its current level at that time and its
    selected level. The returned time is the first second at which those two bands are
    separated by at least `required_gap_fl`.

    Returns `0.0` if the bands are already resolved, and
    `VERTICAL_OVERLAP_NEVER_RESOLVES_S` if they never resolve under the selected levels
    and climb/descent rate.

    Parameters
    ----------
    a_current_fl : float
        Aircraft A current flight level.
    a_selected_fl : float
        Aircraft A selected flight level.
    b_current_fl : float
        Aircraft B current flight level.
    b_selected_fl : float
        Aircraft B selected flight level.
    vertical_rate_fpm : float, optional
        Climb/descent rate in feet per minute, used for both aircraft.
    required_gap_fl : float, optional
        Required gap between the two remaining vertical bands, in flight levels.

    Returns
    -------
    float
        First time in seconds at which the remaining vertical bands are separated by
        `required_gap_fl`; `0.0` if already resolved; or
        `VERTICAL_OVERLAP_NEVER_RESOLVES_S` if they never resolve.

    Notes
    -----
    The calculation is closed form. Band edges are piecewise linear in time, and their
    slopes can only change when one aircraft reaches its selected level. The function
    therefore checks at most two linear intervals and both possible vertical orderings.
    """
    if required_gap_fl < 0.0:
        required_gap_fl = 0.0

    if vertical_bands_are_resolved(
        a_current_fl=a_current_fl,
        a_selected_fl=a_selected_fl,
        b_current_fl=b_current_fl,
        b_selected_fl=b_selected_fl,
        required_gap_fl=required_gap_fl,
        vertical_rate_fpm=vertical_rate_fpm,
        t_s=0.0,
    ):
        return 0.0

    a_done_s = _time_to_selected_s(a_current_fl, a_selected_fl, vertical_rate_fpm)
    b_done_s = _time_to_selected_s(b_current_fl, b_selected_fl, vertical_rate_fpm)
    if a_done_s == VERTICAL_OVERLAP_NEVER_RESOLVES_S or b_done_s == VERTICAL_OVERLAP_NEVER_RESOLVES_S:
        return VERTICAL_OVERLAP_NEVER_RESOLVES_S

    # The band geometry can only change slope when an aircraft reaches selected level.
    # That leaves at most two linear intervals to test: before the first aircraft levels,
    # then before the second one levels.
    first_done_s = min(a_done_s, b_done_s)
    second_done_s = max(a_done_s, b_done_s)

    for interval_index in range(2):
        if interval_index == 0:
            start_s = 0.0
            end_s = first_done_s
        else:
            start_s = first_done_s
            end_s = second_done_s

        # Check both possible final orderings separately. This matters for crossing
        # climb/descent cases because max(B-above-A, A-above-B) is not itself one line.
        b_above_crossing_s = _first_directional_gap_crossing_in_interval_s(
            a_current_fl=a_current_fl,
            a_selected_fl=a_selected_fl,
            b_current_fl=b_current_fl,
            b_selected_fl=b_selected_fl,
            vertical_rate_fpm=vertical_rate_fpm,
            required_gap_fl=required_gap_fl,
            start_s=start_s,
            end_s=end_s,
            b_above_a=True,
        )
        a_above_crossing_s = _first_directional_gap_crossing_in_interval_s(
            a_current_fl=a_current_fl,
            a_selected_fl=a_selected_fl,
            b_current_fl=b_current_fl,
            b_selected_fl=b_selected_fl,
            vertical_rate_fpm=vertical_rate_fpm,
            required_gap_fl=required_gap_fl,
            start_s=start_s,
            end_s=end_s,
            b_above_a=False,
        )

        if b_above_crossing_s != VERTICAL_OVERLAP_NEVER_RESOLVES_S and (
            a_above_crossing_s == VERTICAL_OVERLAP_NEVER_RESOLVES_S or b_above_crossing_s <= a_above_crossing_s
        ):
            return b_above_crossing_s
        if a_above_crossing_s != VERTICAL_OVERLAP_NEVER_RESOLVES_S:
            return a_above_crossing_s

    # After both aircraft have levelled, the remaining bands are fixed at their selected
    # levels. If they are not resolved then, they never will be under this model.
    if vertical_bands_are_resolved(
        a_current_fl=a_current_fl,
        a_selected_fl=a_selected_fl,
        b_current_fl=b_current_fl,
        b_selected_fl=b_selected_fl,
        required_gap_fl=required_gap_fl,
        vertical_rate_fpm=vertical_rate_fpm,
        t_s=second_done_s,
    ):
        return second_done_s

    return VERTICAL_OVERLAP_NEVER_RESOLVES_S
