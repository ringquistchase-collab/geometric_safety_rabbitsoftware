"""
Relative-aircraft catch-up projection using convex geometry.

For a longer design-level exposition of the same mathematics and implementation
choices, see `relevant_aircraft_design_note.md` at the repository root.

Plain-language overview
-----------------------
This module asks a practical controller-style question:

    "Given where two aircraft are now, where they are pointed, how fast they might be
    going, and how they may turn, can they possibly come too close within the next
    few minutes?"

The key idea is not to simulate one nominal future. Instead, we construct the entire
envelope of futures allowed by the model and then ask whether any member of that
envelope enters the protected separation region.

There are two versions of the calculation:

1. Straight-heading method
   If both aircraft keep a fixed heading, the full set of possible relative motions can
   be collapsed into one convex 2-D shape. The problem then becomes: how close can that
   shape come to the origin? Under its assumptions, this version is exact.

2. Turn-aware method
   If one or both aircraft are turning at a fixed rate toward a target heading, the
   possible relative positions still form a simple convex shape at any one chosen time,
   but that shape changes over time. The method therefore:
   - computes the exact worst-case distance at selected times,
   - certifies whole time intervals using an interval-local bound on how fast the
     reachable hull can move,
   - and treats any interval that cannot be certified finely enough as unsafe.

In short:

    straight flight -> one exact geometric check,
    turning flight -> many exact geometric checks plus a conservative time certificate.

This means the turn-aware version is intentionally cautious: it should only say "safe"
when the whole modelled time horizon has been certified safe. The public API also
offers an optional additional robustness buffer for in-turn speed variation beyond the
core constant-speed turn model.

Formal statement
----------------
The remainder of this docstring recasts the same idea in a more formal style.

The underlying question is:

    Given two aircraft states now, bounded speed uncertainty, and a finite horizon
    [0, T], can any admissible motion bring the aircraft closer than a required
    separation distance D?

Both public methods answer that same decision problem, but under different kinematic
models:

1. `catch_up_projection_interval`
   Exact for fixed headings and bounded constant speeds.
2. `catch_up_projection_interval_with_turns`
   Exact in speed and conservative in time when each aircraft may execute one fixed-rate
   turn toward a target heading and then continue straight.

The sections below are intended to bridge implementation and exposition: they define the
objects being optimised, explain why the straight-line method reduces to a single convex
projection, and explain why the turning method requires a certified search in time.


Problem statement
-----------------
Fix a local east/north tangent plane. Let

    p_A(0), p_B(0) in R^2

be the current aircraft positions, and define the initial relative position

    r_0 = p_A(0) - p_B(0).

Let each aircraft speed lie in a closed interval

    s_A in [s_A^-, s_A^+],   s_B in [s_B^-, s_B^+],

with speeds treated as constant over the projection horizon once chosen. The core
quantity of interest is

    d_* = min ||p_A(t) - p_B(t)||
          over t in [0, T] and all admissible speeds.

The safety decision is then simply:

    safe  <=>  d_* >= D.

So the aim is not to predict one nominal closest approach, but to bound the worst case
over an admissible envelope of motion.


Method 1: Straight headings
---------------------------
For the straight-line model, each aircraft has a fixed heading. If `u_A` and `u_B` are
the corresponding unit heading vectors, then

    p_A(t) = p_A(0) + t s_A u_A,
    p_B(t) = p_B(0) + t s_B u_B.

Hence the relative motion is

    r(t) = r_0 + t (s_A u_A - s_B u_B).

The set of admissible relative velocities is the image of the speed rectangle under an
affine map:

    V = { s_A u_A - s_B u_B : s_A in [s_A^-, s_A^+], s_B in [s_B^-, s_B^+] }.

Because the speed intervals are one-dimensional, `V` is the convex hull of the four
corner velocities obtained from the speed extremes. Over the full horizon, the set of
all admissible relative displacements is

    S(T) = { t v : t in [0, T], v in V }.

Since `V` is convex and contains all admissible relative velocities,

    S(T) = conv({0} union {T v_i}),

where `v_i` are the four extreme relative velocities. This reduces the continuous-time
reachability problem to a single compact convex polygon in displacement space.

The worst-case separation becomes

    d_* = min_{d in S(T)} ||r_0 + d||.

Equivalently, one projects `-r_0` onto `S(T)`:

    d^* = argmin_{d in S(T)} ||d - (-r_0)||.

Then

    d_* = ||r_0 + d^*||.

Thus the straight-line method is exact under its model: one convex hull construction and
one Euclidean projection solve the full problem. The reported closest time is recovered
geometrically from the location of the projection on the displacement cone.


Method 2: One fixed-rate turn, then straight
--------------------------------------------
The turn-aware method assumes each aircraft follows a deterministic heading profile:

    - start at heading h_0,
    - turn at fixed rate `turn_rate_deg_sec` toward target heading h_T,
    - stop turning once the target heading is reached,
    - continue straight on h_T thereafter.

Let Delta h be the signed heading change and let

    tau = |Delta h| / |turn_rate_deg_sec|

be the turn duration when `turn_rate_deg_sec != 0`. Write the unit heading vector as

    u(h) = (sin h, cos h).

For a fixed speed s, the aircraft position can be written in the form

    p(t; s) = p(0) + s b(t),

where the basis vector `b(t)` depends only on the heading profile, not on speed:

    b(t) = integral_0^t u(h(xi)) d xi.

Because the heading profile is piecewise simple, this integral is available in closed
form. During the turn,

    h(t) = h_0 + sigma |turn_rate_deg_sec| t,

with `sigma` the turn direction, so `b(t)` is obtained by analytically integrating the
rotating unit vector. After the turn, one appends the straight segment

    (t - tau) u(h_T).

This closed-form basis is what `_turn_displacement_basis` computes.

The key structural fact is that, at any fixed time t, position is affine in speed.
Therefore the relative-position set at that time is

    R(t) = r_0 + { s_A b_A(t) - s_B b_B(t)
                   : s_A in [s_A^-, s_A^+], s_B in [s_B^-, s_B^+] }.

Again this is the image of a speed rectangle under an affine map, so `R(t)` is a small
convex polygon: a point, segment, or parallelogram described by the four speed-corner
combinations. The exact worst-case distance at time t is then

    d(t) = min_{r in R(t)} ||r||,

which is computed by projecting the origin onto that polygon.

Thus, unlike the straight-line method, the difficulty is no longer the speed envelope at
a fixed time; that part remains exact and convex. The difficulty is minimising `d(t)`
over continuous time:

    d_* = min_{t in [0, T]} d(t).


Certified adaptive time search
------------------------------
The implementation does not march over a fixed time grid. Instead it evaluates `d(t)`
exactly at selected times and certifies whole time intervals as safe whenever possible.

Let `R(t)` be the fixed-time reachable relative-position hull and let

    d(t) = dist(0, R(t)).

Each hull corner is a trajectory of the form

    r_i(t) = r_0 + s_A b_A(t) - s_B b_B(t),

so its instantaneous speed is

    ||r_i'(t)|| = ||s_A u_A(t) - s_B u_B(t)||.

A valid Lipschitz constant for `d(t)` on an interval `I = [t_0, t_1]` is therefore

    L_I = sup_{t in I} max_i ||r_i'(t)||,

because the convex hull cannot move faster, in Hausdorff distance, than its fastest
corner and distance-to-set is 1-Lipschitz with respect to that motion.

This is tighter than the simpler global bound

    L_global = s_A^+ + s_B^+,

which remains available as an option but is no longer the default. The local
bound is preferred because it uses the known heading profiles on the interval instead of
assuming worst-case anti-parallel closing everywhere.

The practical reason this is cheap is that each heading profile is piecewise linear in
time: turn at fixed rate until a known completion time, then hold the target heading.
Hence the relative heading on any subinterval between turn-completion events is linear.
For one speed corner pair,

    ||s_A u_A(t) - s_B u_B(t)||^2
        = s_A^2 + s_B^2 - 2 s_A s_B cos(delta(t)),

with `delta(t)` the relative heading. Over a linear-heading subinterval, the maximum is
attained at an endpoint or at an internal anti-parallel crossing

    delta(t) = (2k + 1) pi.

So the interval-local bound can be computed exactly with a constant amount of work:
check the interval endpoints, split only at turn-completion events if needed, and test
whether an anti-parallel crossing occurs on each linear piece.

Once a Lipschitz constant `L_I` is available, an interval [t_0, t_1] with endpoint
distances `d_0 = d(t_0)` and `d_1 = d(t_1)` satisfies the lower bound

    d(t) >= (d_0 + d_1 - L_I (t_1 - t_0)) / 2.

Therefore:

    if (d_0 + d_1 - L_I (t_1 - t_0)) / 2 >= D,
    the entire interval is certified safe.

The algorithm starts with [0, T]. If that interval cannot be certified, it samples the
midpoint exactly, splits the interval in two, and repeats. Refinement continues until
each interval is either:

    - certified safe, or
    - smaller than a configured minimum interval width.

Intervals that remain uncertified at that minimum width are conservatively treated as
unsafe. This gives the central guarantee:

    the turn-aware method never returns "safe" unless all remaining time intervals have
    been certified safe under the model.

The reported `min_distance_m` and `closest_time_s` in the turning case are informative
diagnostics based on sampled points and interval certificates. The boolean safety result
is the primary mathematically guaranteed output.


Relationship between the two methods
------------------------------------
The straight-line method solves one global convex projection problem because the entire
reachable set over [0, T] collapses into a single displacement hull.

The turn-aware method solves a family of exact convex projection problems, one per
sampled time, because turning destroys the simple cone structure in time even though the
speed envelope remains convex at each fixed time.

So the distinction is:

    straight headings: exact in speed, exact in time,
    fixed-rate turns: exact in speed, conservative in time.


Structured hull optimization
----------------------------
Both methods share the same fixed-time geometric core:

    base + s_A a_basis - s_B b_basis,

with `s_A` and `s_B` drawn from closed speed intervals. The image of the speed
rectangle is therefore determined by just four speed-corner points.

This observation is now used explicitly in the implementation:

1. A shared helper constructs those four corner points for both
   - the straight-line displacement hull at `t = T`, and
   - the turn-aware relative-position hull at a fixed time `t`.
2. The hot-path hull construction for these tiny 4- and 5-point inputs uses a
   specialized small-set hull builder rather than the full generic monotone-chain
   implementation.
3. The generic `_convex_hull` function is retained as a reference implementation and
   sanity oracle; tests compare the optimized structured path against it.

This optimization does not change the mathematics of the reachable sets. It only
exploits the fact that the relevant hulls in this module are tiny and highly
structured.


How this maps to functions
--------------------------
- `heading_to_unit_vector`: heading -> east/north direction.
- `latlon_to_local_xy`: latitude/longitude -> local east/north coordinates.
- `_small_convex_hull`: optimized hull builder for the module's tiny structured inputs.
- `_speed_rectangle_corner_points`: shared affine speed-corner kernel.
- `compute_relative_velocity_hull`: straight-line reachable displacement hull over
  [0, T].
- `_closest_point_on_convex_polygon`: Euclidean projection onto a convex polygon.
- `_closest_time_from_projection`: straight-line time recovery from the projected point.
- `catch_up_projection_interval`: exact straight-line safety test.
- `_turn_displacement_basis`: closed-form unit-speed displacement for a turn-then-straight
  heading law.
- `_interval_local_lipschitz_mps`: exact interval-local corner-speed supremum used as a
  turn-aware Lipschitz constant.
- `_relative_position_hull_at_time`: exact relative-position hull at one time.
- `_min_distance_to_relative_hull_at_time`: exact worst-case distance at one time.
- `_interval_distance_lower_bound`: Lipschitz certificate for a time interval.
- `catch_up_projection_interval_with_turns`: adaptive certified time search.


Safety interpretation
---------------------
- Straight-line method:
  - the full `(is_separated, min_distance_m, closest_time_s)` tuple is exact under the
    fixed-heading, bounded-constant-speed model.
- Turn-aware method:
  - exact in speed,
  - conservative in time,
  - and certified safe only when every remaining interval has been proved safe.
- The turn-aware API can optionally inflate the separation threshold by a conservative
  robustness buffer if one wants protection against speed variation during an active
  turn, beyond the core constant-speed model.
- If `is_separated` is `True` in the turn-aware method, the model certifies that no
  admissible speed realisation violates separation in the horizon considered.
- If `is_separated` is `False` in the turn-aware method, a separation loss is feasible
  in the modelled envelope or an interval could not be certified safe at the configured
  time resolution. With the optional robustness buffer enabled, `False` can also mean
  that the buffered threshold could not be certified even though the raw model remained
  clear of the nominal threshold.
- In the turn-aware method, `min_distance_m` and `closest_time_s` are diagnostics that
  summarize the most critical sampled point or interval certificate encountered.
- This is a deterministic reachability check, not a probability-of-conflict estimate.


Assumptions and limits
----------------------
- Horizontal 2-D kinematics only.
- Local tangent-plane projection; appropriate for local interactions, not long-haul
  geodesic fidelity.
- Symmetric bounded speed uncertainty, clipped at zero.
- No wind, acceleration, or stochastic dynamics.
- Straight-line mode assumes headings are constant over [0, T].
- Turn-aware mode assumes at most one fixed-rate turn per aircraft, from current
  heading to target heading, followed by straight flight.
- Turn-aware mode is conservative in time because it certifies intervals rather than
  solving the continuous-time minimum in closed form.
"""

from dataclasses import dataclass
import math
from typing import TYPE_CHECKING, Any

import numba
import numpy as np

if TYPE_CHECKING:
    from matplotlib.axes import Axes
else:
    Axes = Any

KT_TO_MPS = 1852.0 / 3600.0
DEG_TO_RAD = np.pi / 180.0
RAD_TO_DEG = 180.0 / np.pi
EARTH_RADIUS_IN_METERS = 6378137.0

# Headings within this tolerance are treated as straight flight so tiny numerical turns
# do not trigger the slower turn-aware path.
TURN_HEADING_EPS_DEG = 1.0

# Turn rates below this threshold are treated as zero to avoid dividing by effectively
# vanishing angular velocity.
TURN_RATE_EPS_DEG_PER_S = 1e-6

# The adaptive certifier stops subdividing once an interval is this short. We use 3 s
# because radar updates arrive every 6 s in practice, so certifying below half a scan
# interval is usually a good speed/precision balance.
TURN_TIME_CERT_MIN_INTERVAL_S = 3.0

# The turn-aware certifier defaults to an interval-local Lipschitz bound because it is
# less conservative than the simple global closing-rate bound while remaining easy to
# justify. Passing `use_interval_local_lipschitz=False` switches to the simpler global
# bound `a_max + b_max` for comparison or debugging.
TURN_USE_INTERVAL_LOCAL_LIPSCHITZ_DEFAULT = True

# Optional robustness assumption for the turn-aware solver: during an active turn, an
# aircraft may vary its instantaneous speed within `nominal +/- this value` even though
# the core reachable-set model still uses one constant speed over the horizon. The code
# converts this to a conservative additive separation buffer and keeps it disabled by
# default.
TURN_SPEED_SCHEDULE_UNCERTAINTY_KT_DEFAULT = 0.0


@numba.jit(nopython=True, fastmath=True, cache=True)
def clip_value(x: float, min_val: float, max_val: float) -> float:
    """Return `x` clipped to the closed interval `[min_val, max_val]`."""
    if x < min_val:
        return min_val
    if x > max_val:
        return max_val
    return x


@numba.njit(cache=True, fastmath=True)
def heading_to_unit_vector(heading_deg: float) -> np.ndarray:
    """
    Convert an aviation heading into a local east/north unit vector.

    Parameters
    ----------
    heading_deg: float
        Heading in degrees.

    Returns
    -------
    np.ndarray
        Two-element unit vector `(east, north)`.

    Notes
    -----
    The module uses the aviation convention "clockwise from north", so the components
    are

        east  = sin(heading),
        north = cos(heading).

    This differs from the standard mathematical angle convention.
    """
    heading_rad = heading_deg * DEG_TO_RAD
    out = np.empty(2, dtype=np.float64)
    out[0] = np.sin(heading_rad)
    out[1] = np.cos(heading_rad)
    return out


@numba.njit(cache=True, fastmath=True)
def latlon_to_local_xy(lat: float, lon: float, ref_lat: float, ref_lon: float) -> np.ndarray:
    """
    Project latitude/longitude into a local east/north tangent-plane frame.

    Parameters
    ----------
    lat / lon: float
        Point to project, in degrees.
    ref_lat / ref_lon: float
        Reference position used as the local origin, in degrees.

    Returns
    -------
    np.ndarray
        East/north offset in metres from `(ref_lat, ref_lon)` to `(lat, lon)`.

    Notes
    -----
    This is the local projection used throughout the module. It is the first-order
    tangent-plane approximation:

        east  ≈ R cos(ref_lat) Δlon,
        north ≈ R Δlat.

    That is appropriate for local conflict checks, but not intended for long-range
    geodesic fidelity.
    """
    d_lat = (lat - ref_lat) * DEG_TO_RAD
    d_lon = (lon - ref_lon) * DEG_TO_RAD
    ref_lat_rad = ref_lat * DEG_TO_RAD
    east = EARTH_RADIUS_IN_METERS * d_lon * np.cos(ref_lat_rad)
    north = EARTH_RADIUS_IN_METERS * d_lat
    out = np.empty(2, dtype=np.float64)
    out[0] = east
    out[1] = north
    return out


@numba.njit(cache=True, fastmath=True)
def cross_2d(o: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
    """
    Compute the signed 2-D cross product of the vectors `OA` and `OB`.

    Parameters
    ----------
    o: np.ndarray
        Origin of the vectors.
    a / b: np.ndarray
        Vector endpoints.

    Returns
    -------
    float
        Signed area proportional to the parallelogram area. Positive values indicate
        that `O -> A -> B` is a counter-clockwise turn.
    """
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


@numba.njit(cache=True, fastmath=True)
def _small_convex_hull(pts: np.ndarray) -> np.ndarray:
    """
    Compute a convex hull for tiny point sets without the generic monotone-chain path.

    This helper is used only for the structured 4- and 5-point hulls that arise from
    the speed-rectangle geometry in this module. The full `_convex_hull` implementation
    is retained below as the generic reference path and sanity oracle.
    """
    n = pts.shape[0]

    if n == 0:
        return np.empty((0, 2), dtype=np.float64)

    # Deduplicate in-place; these structured point sets are tiny enough that an O(n^2)
    # pass is cheaper than setting up a more general sorting pipeline.
    unique_pts = np.empty((n, 2), dtype=np.float64)
    unique_len = 0
    for i in range(n):
        x = pts[i, 0]
        y = pts[i, 1]
        duplicate = False
        for j in range(unique_len):
            if unique_pts[j, 0] == x and unique_pts[j, 1] == y:
                duplicate = True
                break
        if not duplicate:
            unique_pts[unique_len, 0] = x
            unique_pts[unique_len, 1] = y
            unique_len += 1

    if unique_len == 0:
        return np.empty((0, 2), dtype=np.float64)

    if unique_len == 1:
        out = np.empty((1, 2), dtype=np.float64)
        out[0, :] = unique_pts[0]
        return out

    if unique_len == 2:
        out = np.empty((2, 2), dtype=np.float64)
        if unique_pts[1, 0] < unique_pts[0, 0] or (
            unique_pts[1, 0] == unique_pts[0, 0] and unique_pts[1, 1] < unique_pts[0, 1]
        ):
            out[0, :] = unique_pts[1]
            out[1, :] = unique_pts[0]
        else:
            out[0, :] = unique_pts[0]
            out[1, :] = unique_pts[1]
        return out

    # Gift-wrap the hull. For <= 5 points this is simpler and cheaper than invoking the
    # fully generic hull builder.
    start = 0
    for i in range(1, unique_len):
        if unique_pts[i, 0] < unique_pts[start, 0] or (
            unique_pts[i, 0] == unique_pts[start, 0] and unique_pts[i, 1] < unique_pts[start, 1]
        ):
            start = i

    hull_idx = np.empty(unique_len, dtype=np.int64)
    hull_len = 0
    p = start

    while True:
        hull_idx[hull_len] = p
        hull_len += 1

        q = -1
        for r in range(unique_len):
            if r == p:
                continue
            if q == -1:
                q = r
                continue

            turn = cross_2d(unique_pts[p], unique_pts[q], unique_pts[r])
            if turn > 0.0:
                q = r
            elif turn == 0.0:
                dx_q = unique_pts[q, 0] - unique_pts[p, 0]
                dy_q = unique_pts[q, 1] - unique_pts[p, 1]
                dx_r = unique_pts[r, 0] - unique_pts[p, 0]
                dy_r = unique_pts[r, 1] - unique_pts[p, 1]
                if dx_r * dx_r + dy_r * dy_r > dx_q * dx_q + dy_q * dy_q:
                    q = r

        if q == start:
            break

        p = q

        if hull_len >= unique_len:
            break

    hull = np.empty((hull_len, 2), dtype=np.float64)
    for i in range(hull_len):
        hull[i, :] = unique_pts[hull_idx[i]]

    return hull


@numba.njit(cache=True, fastmath=True)
def _speed_rectangle_corner_points(
    base: np.ndarray,
    a_basis: np.ndarray,
    a_min_speed: float,
    a_max_speed: float,
    b_basis: np.ndarray,
    b_min_speed: float,
    b_max_speed: float,
) -> np.ndarray:
    """
    Map a 2-D speed rectangle to its four relative-space corner points.

    For both the straight and turn-aware methods, the reachable relative displacement or
    position set has the affine form

        base + s_A a_basis - s_B b_basis,

    with `s_A` and `s_B` drawn from closed speed intervals. The four speed-corner
    combinations therefore determine the entire convex set.
    """
    pts = np.empty((4, 2), dtype=np.float64)
    anchor_x = base[0] + a_min_speed * a_basis[0] - b_min_speed * b_basis[0]
    anchor_y = base[1] + a_min_speed * a_basis[1] - b_min_speed * b_basis[1]
    delta_a_x = (a_max_speed - a_min_speed) * a_basis[0]
    delta_a_y = (a_max_speed - a_min_speed) * a_basis[1]
    delta_b_x = -(b_max_speed - b_min_speed) * b_basis[0]
    delta_b_y = -(b_max_speed - b_min_speed) * b_basis[1]

    pts[0, 0] = anchor_x
    pts[0, 1] = anchor_y
    pts[1, 0] = anchor_x + delta_a_x
    pts[1, 1] = anchor_y + delta_a_y
    pts[2, 0] = anchor_x + delta_a_x + delta_b_x
    pts[2, 1] = anchor_y + delta_a_y + delta_b_y
    pts[3, 0] = anchor_x + delta_b_x
    pts[3, 1] = anchor_y + delta_b_y

    return pts


@numba.njit(cache=True, fastmath=True)
def _convex_hull(pts: np.ndarray) -> np.ndarray:
    """
    Compute the convex hull of a set of 2-D points using Andrew's monotone chain.

    Parameters
    ----------
    pts: np.ndarray
        Array of (x, y) coordinates with shape (N, 2).

    Returns
    -------
    np.ndarray
        Hull vertices ordered counter-clockwise, with duplicate and collinear interior
        points removed.
    """
    n = pts.shape[0]

    if n == 0:
        return np.empty((0, 2), dtype=np.float64)

    # Selection-sort indices by (x, y) because `np.lexsort` is not available in Numba's
    # nopython mode.
    idx = np.empty(n, dtype=np.int64)
    for i in range(n):
        idx[i] = i

    for i in range(n):
        min_i = i
        for j in range(i + 1, n):
            pi = pts[idx[min_i]]
            pj = pts[idx[j]]
            if pj[0] < pi[0] or (pj[0] == pi[0] and pj[1] < pi[1]):
                min_i = j
        tmp = idx[i]
        idx[i] = idx[min_i]
        idx[min_i] = tmp

    # Deduplicate after sorting so the hull construction only sees distinct points.
    uniq_idx = np.empty(n, dtype=np.int64)
    uniq_len = 0
    prev_x = np.inf
    prev_y = np.inf
    for k in range(n):
        i_idx = idx[k]
        x = pts[i_idx, 0]
        y = pts[i_idx, 1]
        if k == 0 or x != prev_x or y != prev_y:
            uniq_idx[uniq_len] = i_idx
            uniq_len += 1
            prev_x = x
            prev_y = y

    m = uniq_len
    if m == 0:
        return np.empty((0, 2), dtype=np.float64)

    pts_unique = np.empty((m, 2), dtype=np.float64)
    for i in range(m):
        pts_unique[i, :] = pts[uniq_idx[i]]

    if m == 1:
        return pts_unique

    # Build the lower chain of the monotone hull, removing non-left turns so only the
    # outer envelope remains.
    lower = np.empty(m, dtype=np.int64)
    lower_len = 0
    for i in range(m):
        while lower_len >= 2:
            p1 = pts_unique[lower[lower_len - 2]]
            p2 = pts_unique[lower[lower_len - 1]]
            p3 = pts_unique[i]
            if cross_2d(p1, p2, p3) <= 0.0:
                lower_len -= 1
            else:
                break
        lower[lower_len] = i
        lower_len += 1

    # Build the upper chain by scanning the sorted points in reverse.
    upper = np.empty(m, dtype=np.int64)
    upper_len = 0
    for i in range(m - 1, -1, -1):
        while upper_len >= 2:
            p1 = pts_unique[upper[upper_len - 2]]
            p2 = pts_unique[upper[upper_len - 1]]
            p3 = pts_unique[i]
            if cross_2d(p1, p2, p3) <= 0.0:
                upper_len -= 1
            else:
                break
        upper[upper_len] = i
        upper_len += 1

    hull_len = lower_len + upper_len - 2
    hull = np.empty((hull_len, 2), dtype=np.float64)

    h = 0
    for i in range(lower_len):
        hull[h, :] = pts_unique[lower[i]]
        h += 1
    for j in range(1, upper_len - 1):
        hull[h, :] = pts_unique[upper[j]]
        h += 1

    return hull


@numba.njit(cache=True, fastmath=True)
def _point_in_convex_polygon(pt: np.ndarray, hull: np.ndarray) -> bool:
    """
    Check whether a point lies inside or on the boundary of a convex polygon.

    Parameters
    ----------
    pt: np.ndarray
        Query point.
    hull: np.ndarray
        Convex polygon vertices in order.

    Returns
    -------
    bool
        `True` if the point is inside or on the boundary.
    """
    hull_len = hull.shape[0]
    if hull_len < 3:
        return False

    sign = 0

    # Use a scale-aware collinearity tolerance so very small and very large polygons are
    # handled consistently.
    max_norm = 0.0
    for i in range(hull_len):
        norm_val = np.sqrt(hull[i, 0] * hull[i, 0] + hull[i, 1] * hull[i, 1])
        if norm_val > max_norm:
            max_norm = norm_val
    scale = max(max_norm, 1.0)

    for i in range(hull_len):
        a = hull[i]
        b = hull[(i + 1) % hull_len]
        cross = cross_2d(a, b, pt)

        # Points exactly on an edge are treated as inside.
        if abs(cross) <= 1e-12 * scale:
            continue

        # For a convex polygon with consistent winding, the cross-product sign must not
        # change as we walk around the boundary.
        if sign == 0:
            sign = 1 if cross > 0.0 else -1
        elif (cross > 0.0 and sign == -1) or (cross < 0.0 and sign == 1):
            return False

    return True


@numba.njit(cache=True, fastmath=True)
def _closest_point_on_segment(pt: np.ndarray, a: np.ndarray, b: np.ndarray) -> tuple[np.ndarray, float]:
    """
    Project a point onto the closed line segment `AB`.

    Parameters
    ----------
    pt: np.ndarray
        Query point.
    a / b: np.ndarray
        Segment endpoints.

    Returns
    -------
    tuple[np.ndarray, float]
        Closest point on the segment and its Euclidean distance from `pt`.
    """
    ab = b - a
    denom = np.dot(ab, ab)

    if denom == 0.0:
        return a, float(np.linalg.norm(pt - a))

    t = np.dot(pt - a, ab) / denom
    if t < 0.0:
        t = 0.0
    elif t > 1.0:
        t = 1.0
    proj = a + t * ab  # pyright: ignore[reportOperatorIssue]

    return proj, float(np.linalg.norm(pt - proj))


@numba.njit(cache=True, fastmath=True)
def _closest_point_on_convex_polygon(pt: np.ndarray, hull: np.ndarray) -> tuple[np.ndarray, float]:
    """
    Find the Euclidean projection of a point onto a convex polygon.

    Parameters
    ----------
    pt: np.ndarray
        Query point.
    hull: np.ndarray
        Convex polygon vertices.

    Returns
    -------
    tuple[np.ndarray, float]
        Projected point and the corresponding Euclidean distance.
    """
    hull_len = hull.shape[0]

    if hull_len == 0:
        raise ValueError("Hull must contain at least one point.")

    best_point = hull[0]
    best_dist = float(np.linalg.norm(pt - best_point))

    if hull_len == 1:
        return best_point, best_dist

    if hull_len == 2:
        return _closest_point_on_segment(pt, hull[0], hull[1])

    # Interior points project to themselves.
    if _point_in_convex_polygon(pt, hull):
        return pt, 0.0

    # Otherwise the projection must lie on one of the polygon edges.
    for i in range(hull_len):
        a = hull[i]
        b = hull[(i + 1) % hull_len]

        cand_point, cand_dist = _closest_point_on_segment(pt, a, b)

        if cand_dist < best_dist:
            best_dist = cand_dist
            best_point = cand_point

    return best_point, best_dist


@numba.njit(cache=True)
def _closest_time_from_projection(closest_vec: np.ndarray, hull: np.ndarray, T: float) -> float:
    """
    Map a reachable straight-line displacement back to the earliest elapsed time.

    The straight reachable set is `S(T) = conv({0} U {T v_i})`. For a point `p in S(T)`,
    the earliest achievable time is the smallest `alpha in [0, 1]` such that

        p in alpha S(T).

    In 2-D, Caratheodory's theorem implies that this earliest representation can be
    expressed using the origin and at most two outer hull vertices, so the code searches
    all one- and two-vertex combinations and returns `t = alpha T`.

    Parameters
    ----------
    closest_vec : np.ndarray
        A point in the reachable displacement set (on or inside the hull).
    hull : np.ndarray
        Convex hull vertices of the reachable set.
    T : float
        Projection horizon in seconds.

    Returns
    -------
    float
        The time `t in [0, T]` at which this displacement is achieved in the straight
        constant-heading model.
    """
    eps = 1e-12

    # The origin corresponds to zero elapsed time.
    norm_closest = float(np.linalg.norm(closest_vec))
    if norm_closest <= eps:
        return 0.0

    hull_arr = np.asarray(hull, dtype=np.float64)
    n = hull_arr.shape[0]

    if n == 0:
        return 0.0

    # The straight-line reachable set is `S(T) = conv({0} U {T v_i})`. For any point
    # `p in S(T)`, the earliest achievable time is the smallest `alpha in [0, 1]` such
    # that `p in alpha S(T)`. Because we are in 2-D and the set is convex, Caratheodory's
    # theorem implies `p` can be represented using at most two hull vertices plus the
    # origin:
    #
    #     p = u a + v b,   u >= 0, v >= 0, u + v <= 1.
    #
    # The corresponding time is then `t = T (u + v)`. We therefore search all hull
    # vertex pairs and keep the smallest valid `u + v`.
    best_alpha = np.inf

    # Degenerate ray case: if `p` lies on the segment from the origin to a hull vertex,
    # then only one outer vertex is needed in the representation.
    for i in range(n):
        a = hull_arr[i]
        a_norm = float(np.linalg.norm(a))
        if a_norm <= eps:
            continue

        cross = a[0] * closest_vec[1] - a[1] * closest_vec[0]
        if abs(cross) > eps * max(a_norm, norm_closest):
            continue

        dot = float(np.dot(a, closest_vec))
        if dot < -eps:
            continue

        alpha = norm_closest / a_norm
        if alpha <= 1.0 + eps and alpha < best_alpha:
            best_alpha = alpha

    # General 2-D case: solve `closest_vec = u a + v b` for each vertex pair.
    for i in range(n):
        a = hull_arr[i]
        for j in range(i + 1, n):
            b = hull_arr[j]
            det = a[0] * b[1] - a[1] * b[0]

            if abs(det) <= eps:
                continue

            u = (closest_vec[0] * b[1] - closest_vec[1] * b[0]) / det
            v = (a[0] * closest_vec[1] - a[1] * closest_vec[0]) / det

            if u < -eps or v < -eps:
                continue

            alpha = u + v
            if alpha <= 1.0 + eps and alpha < best_alpha:
                best_alpha = alpha

    if best_alpha == np.inf:
        # Numerical fallback: recover the radial gauge from the furthest projection of
        # the hull on the displacement direction. This should rarely be needed.
        direction = closest_vec / norm_closest
        best_proj = 0.0
        for i in range(n):
            proj = float(np.dot(hull_arr[i], direction))
            if proj > best_proj:
                best_proj = proj

        if best_proj <= eps:
            return 0.0

        best_alpha = norm_closest / best_proj

    return float(clip_value(T * best_alpha, 0.0, T))


@numba.njit(cache=True, fastmath=True)
def compute_relative_velocity_hull(
    a_heading: float,
    a_speed_kt: float,
    b_heading: float,
    b_speed_kt: float,
    speed_diff_kt: float,
    projection_time_s: float,
) -> np.ndarray:
    """
    Build the straight-line reachable relative-displacement hull over the projection window.

    Parameters
    ----------
    a_heading / b_heading: float
        Headings of aircraft A and B in degrees.
    a_speed_kt / b_speed_kt: float
        Nominal speeds in knots.
    speed_diff_kt: float
        Symmetric speed uncertainty used for both aircraft.
    projection_time_s: float
        Projection horizon in seconds.

    Returns
    -------
    np.ndarray
        Convex hull vertices spanning all admissible relative displacements in `[0, T]`.

    Notes
    -----
    The returned hull is `conv({0} U {T v_i})`, not just the outer velocity polygon
    scaled by `T`. Including the origin makes the hull represent all intermediate times
    in one object rather than only the exact-time slice at `t = T`.
    """
    a_min = max(a_speed_kt - speed_diff_kt, 0.0) * KT_TO_MPS
    a_max = (a_speed_kt + speed_diff_kt) * KT_TO_MPS
    b_min = max(b_speed_kt - speed_diff_kt, 0.0) * KT_TO_MPS
    b_max = (b_speed_kt + speed_diff_kt) * KT_TO_MPS

    dir_a = heading_to_unit_vector(a_heading)
    dir_b = heading_to_unit_vector(b_heading)
    a_basis = projection_time_s * dir_a
    b_basis = projection_time_s * dir_b

    # Append the origin so the hull covers the whole projection interval `[0, T]`, not
    # just the outer displacement slice at exactly `t = T`.
    reachable_points = np.empty((5, 2), dtype=np.float64)
    reachable_points[0, 0] = 0.0
    reachable_points[0, 1] = 0.0
    reachable_points[1:] = _speed_rectangle_corner_points(
        base=np.zeros(2, dtype=np.float64),
        a_basis=a_basis,
        a_min_speed=a_min,
        a_max_speed=a_max,
        b_basis=b_basis,
        b_min_speed=b_min,
        b_max_speed=b_max,
    )
    return _small_convex_hull(reachable_points)


@numba.njit(cache=True, fastmath=True)
def catch_up_projection_interval(
    a_lat: float,
    a_lon: float,
    a_heading: float,
    a_speed_kt: float,
    b_lat: float,
    b_lon: float,
    b_heading: float,
    b_speed_kt: float,
    separation_threshold_m: float,
    speed_diff_kt: float,
    projection_time_s: float,
    rel_pos_override: np.ndarray | None = None,
    velocity_hull: np.ndarray | None = None,
) -> tuple[bool, float, float]:
    """
    Evaluate straight-line horizontal separation over a finite projection horizon.

    Parameters
    ----------
    a_lat / a_lon: float
        Aircraft A latitude/longitude in degrees.
    a_heading: float
        Aircraft A heading in degrees.
    a_speed_kt: float
        Aircraft A nominal speed in knots.
    b_lat / b_lon: float
        Aircraft B latitude/longitude in degrees.
    b_heading: float
        Aircraft B heading in degrees.
    b_speed_kt: float
        Aircraft B nominal speed in knots.
    separation_threshold_m: float
        Required spacing (metres).
    speed_diff_kt: float
        Speed uncertainty (knots) applied symmetrically.
    projection_time_s: float
        Projection horizon in seconds.
    rel_pos_override: np.ndarray | None
        Optional pre-computed relative position (metres).
    velocity_hull: np.ndarray | None
        Optional pre-computed reachable hull.

    Returns
    -------
    tuple[bool, float, float]
        `(is_separated, min_distance_m, closest_time_s)`.

    Notes
    -----
    This is the exact fixed-heading method described in the module docstring. The whole
    speed-and-time envelope is reduced to one convex reachable-displacement hull, and
    the result follows from projecting `-rel_pos0` onto that hull.
    """
    if rel_pos_override is None:
        rel_pos0 = -latlon_to_local_xy(b_lat, b_lon, ref_lat=a_lat, ref_lon=a_lon)
    else:
        rel_pos0 = np.asarray(rel_pos_override, dtype=np.float64)

    hull = (
        np.asarray(velocity_hull, dtype=np.float64)
        if velocity_hull is not None
        else compute_relative_velocity_hull(
            a_heading=a_heading,
            a_speed_kt=a_speed_kt,
            b_heading=b_heading,
            b_speed_kt=b_speed_kt,
            speed_diff_kt=speed_diff_kt,
            projection_time_s=projection_time_s,
        )
    )

    # Project the negative initial relative position onto the reachable displacement
    # hull. The resulting displacement is the one that brings the pair as close together
    # as the model allows.
    closest_vec, _ = _closest_point_on_convex_polygon(-rel_pos0, hull)
    rel_after = rel_pos0 + closest_vec
    min_distance_m = float(np.linalg.norm(rel_after))
    closest_time_s = _closest_time_from_projection(closest_vec, hull, projection_time_s)

    return min_distance_m >= separation_threshold_m, min_distance_m, closest_time_s


@numba.jit(nopython=True, fastmath=True, cache=True)
def heading_diff(h1: float, h2: float, abs_diff: bool = False) -> float:
    """
    Return the wrapped heading difference `h2 - h1` in degrees.

    Parameters
    ----------
    h1 : float
        First heading in degrees.
    h2 : float
        Second heading in degrees.
    abs_diff : bool, optional
        If True, return the absolute difference, by default False

    Returns
    -------
    float
        Heading difference in `[-180, 180)`, or its absolute value if `abs_diff=True`.
    """
    diff = (h2 - h1 + 180.0) % 360.0 - 180.0

    if abs_diff:
        diff = abs(diff)

    return diff


@numba.njit(cache=True, fastmath=True)
def _turn_displacement_basis(
    heading0_deg: float,
    target_heading_deg: float,
    turn_rate_deg_sec: float,
    t_s: float,
) -> np.ndarray:
    """
    Return the unit-speed displacement basis up to time `t_s`.

    The returned vector has units of seconds; multiplying by a speed in m/s gives a
    displacement in metres. Small/no-turn cases fall back to straight flight on the
    target heading so the same heading law covers both turning and straight motion.

    Notes
    -----
    If the aircraft turns at fixed rate until the target heading is reached and then
    flies straight, its position can be written as

        p(t; s) = p(0) + s b(t),

    where `s` is the chosen constant speed and `b(t)` is this basis vector. This
    separation between speed and heading profile is what makes the fixed-time reachable
    set affine in speed.
    """
    out = np.empty(2, dtype=np.float64)
    turn_angle_deg = heading_diff(heading0_deg, target_heading_deg)

    # When the heading change is negligible or the turn rate is effectively zero, the
    # motion law reduces to straight flight on the target heading.
    if abs(turn_angle_deg) < TURN_HEADING_EPS_DEG or abs(turn_rate_deg_sec) <= TURN_RATE_EPS_DEG_PER_S:
        d = heading_to_unit_vector(target_heading_deg)
        out[0] = d[0] * t_s
        out[1] = d[1] * t_s
        return out

    # The sign of the heading difference chooses left vs. right turn; the magnitude of
    # the supplied turn rate gives the angular speed.
    signed_turn_rate_rad_per_s = math.copysign(abs(turn_rate_deg_sec) * DEG_TO_RAD, turn_angle_deg)
    turn_duration_s = abs(turn_angle_deg) / abs(turn_rate_deg_sec)
    turn_time_s = clip_value(t_s, 0.0, turn_duration_s)
    heading0_rad = heading0_deg * DEG_TO_RAD
    heading_turn_end_rad = heading0_rad + signed_turn_rate_rad_per_s * turn_time_s

    # Integrate the unit heading vector analytically over the turn interval. Because
    # speed is factored out, this gives a displacement basis in "seconds".
    out[0] = (math.cos(heading0_rad) - math.cos(heading_turn_end_rad)) / signed_turn_rate_rad_per_s
    out[1] = (math.sin(heading_turn_end_rad) - math.sin(heading0_rad)) / signed_turn_rate_rad_per_s

    # If the requested time extends past the turn, append the straight segment flown
    # on the target heading.
    if t_s > turn_duration_s:
        d = heading_to_unit_vector(target_heading_deg)
        straight_time_s = t_s - turn_duration_s
        out[0] += d[0] * straight_time_s
        out[1] += d[1] * straight_time_s

    return out


@numba.njit(cache=True, fastmath=True)
def _unwrapped_heading_deg_at_time(
    heading0_deg: float,
    target_heading_deg: float,
    turn_rate_deg_sec: float,
    t_s: float,
) -> float:
    """
    Return the deterministic heading profile value at time `t_s` without angle wrapping.

    The unwrapped representation is useful for interval-local Lipschitz bounds because
    relative heading then varies piecewise linearly in time. A wrapped heading would
    introduce artificial discontinuities at 0/360 deg.
    """
    turn_angle_deg = heading_diff(heading0_deg, target_heading_deg)

    if abs(turn_angle_deg) < TURN_HEADING_EPS_DEG or abs(turn_rate_deg_sec) <= TURN_RATE_EPS_DEG_PER_S:
        return target_heading_deg

    signed_turn_rate_deg_sec = math.copysign(abs(turn_rate_deg_sec), turn_angle_deg)
    turn_duration_s = abs(turn_angle_deg) / abs(turn_rate_deg_sec)
    turn_time_s = clip_value(t_s, 0.0, turn_duration_s)
    return heading0_deg + signed_turn_rate_deg_sec * turn_time_s


@numba.njit(cache=True, fastmath=True)
def _relative_speed_mps(a_speed_mps: float, b_speed_mps: float, delta_heading_deg: float) -> float:
    """
    Return the instantaneous relative speed magnitude for one speed pair.

    If the current headings differ by `delta`, then

        ||s_A u_A - s_B u_B||^2 = s_A^2 + s_B^2 - 2 s_A s_B cos(delta).

    This quantity controls how quickly any one speed-corner trajectory can move inside
    the turn-aware reachable set.
    """
    cos_delta = math.cos(delta_heading_deg * DEG_TO_RAD)
    sq = a_speed_mps * a_speed_mps + b_speed_mps * b_speed_mps - 2.0 * a_speed_mps * b_speed_mps * cos_delta
    return math.sqrt(max(0.0, sq))


@numba.njit(cache=True, fastmath=True)
def _max_relative_speed_on_linear_heading_interval(
    a_speed_mps: float,
    b_speed_mps: float,
    delta_start_deg: float,
    delta_end_deg: float,
) -> float:
    """
    Maximise relative speed for one speed-corner pair over a linear heading interval.

    On any interval where both aircraft headings are linear in time, the relative
    heading is also linear in time. The relative speed depends only on `cos(delta)`, so
    the maximum over the interval occurs either:

    - at one endpoint, or
    - at an internal anti-parallel crossing `delta = (2k + 1) * 180 deg`,
      where the relative speed is exactly `a_speed_mps + b_speed_mps`.
    """
    best = max(
        _relative_speed_mps(a_speed_mps, b_speed_mps, delta_start_deg),
        _relative_speed_mps(a_speed_mps, b_speed_mps, delta_end_deg),
    )

    delta_lo = min(delta_start_deg, delta_end_deg)
    delta_hi = max(delta_start_deg, delta_end_deg)
    first_odd_multiple = 180.0 + 360.0 * math.ceil((delta_lo - 180.0) / 360.0)

    if first_odd_multiple <= delta_hi:
        best = max(best, a_speed_mps + b_speed_mps)

    return best


@numba.njit(cache=True, fastmath=True)
def _interval_local_lipschitz_mps(
    a_heading0_deg: float,
    a_target_heading_deg: float,
    a_turn_rate_deg_sec: float,
    a_min_speed_mps: float,
    a_max_speed_mps: float,
    b_heading0_deg: float,
    b_target_heading_deg: float,
    b_turn_rate_deg_sec: float,
    b_min_speed_mps: float,
    b_max_speed_mps: float,
    t0_s: float,
    t1_s: float,
    global_lipschitz_mps: float,
) -> float:
    """
    Return the exact interval-local corner-speed supremum used as a Lipschitz constant.

    Motivation
    ----------
    A simple valid global bound is `a_max + b_max`, the largest possible closing rate
    across the whole horizon. That is always safe, but it is often too pessimistic
    because the two aircraft headings may be far from anti-parallel on most intervals.

    The distance-to-set function is 1-Lipschitz with respect to the Hausdorff motion of
    the reachable relative-position hull. Each hull corner moves with velocity

        v(t) = s_A u_A(t) - s_B u_B(t),

    so a valid interval-local Lipschitz constant for `d(t)` is

        sup_{t in [t0, t1]} max_corners ||v_corner(t)||.

    Because each heading profile is piecewise linear in time, the relative heading on
    each subinterval is linear. Therefore the supremum is attained at a subinterval
    endpoint or at an anti-parallel crossing, which makes the bound cheap to compute and
    straightforward to justify.
    """
    if t1_s <= t0_s:
        return 0.0

    breakpoints = np.empty(4, dtype=np.float64)
    breakpoint_count = 0
    breakpoints[breakpoint_count] = t0_s
    breakpoint_count += 1

    a_turn_angle_deg = heading_diff(a_heading0_deg, a_target_heading_deg)
    if abs(a_turn_angle_deg) >= TURN_HEADING_EPS_DEG and abs(a_turn_rate_deg_sec) > TURN_RATE_EPS_DEG_PER_S:
        a_turn_end_s = abs(a_turn_angle_deg) / abs(a_turn_rate_deg_sec)
        if t0_s < a_turn_end_s < t1_s:
            breakpoints[breakpoint_count] = a_turn_end_s
            breakpoint_count += 1

    b_turn_angle_deg = heading_diff(b_heading0_deg, b_target_heading_deg)
    if abs(b_turn_angle_deg) >= TURN_HEADING_EPS_DEG and abs(b_turn_rate_deg_sec) > TURN_RATE_EPS_DEG_PER_S:
        b_turn_end_s = abs(b_turn_angle_deg) / abs(b_turn_rate_deg_sec)
        if t0_s < b_turn_end_s < t1_s:
            breakpoints[breakpoint_count] = b_turn_end_s
            breakpoint_count += 1

    breakpoints[breakpoint_count] = t1_s
    breakpoint_count += 1

    # Sort and deduplicate the small breakpoint list.
    for i in range(breakpoint_count):
        min_i = i
        for j in range(i + 1, breakpoint_count):
            if breakpoints[j] < breakpoints[min_i]:
                min_i = j
        tmp = breakpoints[i]
        breakpoints[i] = breakpoints[min_i]
        breakpoints[min_i] = tmp

    best = 0.0
    unique_breakpoints = np.empty(4, dtype=np.float64)
    unique_count = 0
    for i in range(breakpoint_count):
        if unique_count == 0 or abs(breakpoints[i] - unique_breakpoints[unique_count - 1]) > 1e-12:
            unique_breakpoints[unique_count] = breakpoints[i]
            unique_count += 1

    for i in range(unique_count - 1):
        left_t_s = unique_breakpoints[i]
        right_t_s = unique_breakpoints[i + 1]
        if right_t_s <= left_t_s:
            continue

        delta_left_deg = _unwrapped_heading_deg_at_time(
            a_heading0_deg, a_target_heading_deg, a_turn_rate_deg_sec, left_t_s
        ) - _unwrapped_heading_deg_at_time(b_heading0_deg, b_target_heading_deg, b_turn_rate_deg_sec, left_t_s)
        delta_right_deg = _unwrapped_heading_deg_at_time(
            a_heading0_deg, a_target_heading_deg, a_turn_rate_deg_sec, right_t_s
        ) - _unwrapped_heading_deg_at_time(b_heading0_deg, b_target_heading_deg, b_turn_rate_deg_sec, right_t_s)

        for a_speed_mps in (a_min_speed_mps, a_max_speed_mps):
            for b_speed_mps in (b_min_speed_mps, b_max_speed_mps):
                cand = _max_relative_speed_on_linear_heading_interval(
                    a_speed_mps,
                    b_speed_mps,
                    delta_left_deg,
                    delta_right_deg,
                )
                if cand > best:
                    best = cand
                    if best >= global_lipschitz_mps:
                        return global_lipschitz_mps

    return min(best, global_lipschitz_mps)


@numba.njit(cache=True, fastmath=True)
def _relative_position_hull_at_time(
    rel_pos0: np.ndarray,
    a_heading0_deg: float,
    a_target_heading_deg: float,
    a_turn_rate_deg_sec: float,
    a_min_speed_mps: float,
    a_max_speed_mps: float,
    b_heading0_deg: float,
    b_target_heading_deg: float,
    b_turn_rate_deg_sec: float,
    b_min_speed_mps: float,
    b_max_speed_mps: float,
    t_s: float,
) -> np.ndarray:
    """
    Build the exact reachable relative-position hull at a single time `t_s`.

    With fixed turn-rate profiles, each aircraft position is linear in its constant
    speed. The relative position set is therefore the affine image of a speed rectangle,
    i.e. a segment or parallelogram represented by up to four corner points.

    Returns
    -------
    np.ndarray
        Convex hull of all reachable relative positions `r(t_s)` over the speed bounds.
    """
    # For a fixed time, position = start + speed * basis(time). The speed bounds for A
    # and B therefore generate a four-corner rectangle in speed space, which maps to at
    # most four corner points in relative-position space.
    a_basis = _turn_displacement_basis(a_heading0_deg, a_target_heading_deg, a_turn_rate_deg_sec, t_s)
    b_basis = _turn_displacement_basis(b_heading0_deg, b_target_heading_deg, b_turn_rate_deg_sec, t_s)
    pts = _speed_rectangle_corner_points(
        base=rel_pos0,
        a_basis=a_basis,
        a_min_speed=a_min_speed_mps,
        a_max_speed=a_max_speed_mps,
        b_basis=b_basis,
        b_min_speed=b_min_speed_mps,
        b_max_speed=b_max_speed_mps,
    )

    # Collapse duplicate/collinear corners so downstream projection logic can assume a
    # clean convex polygon representation.
    return _small_convex_hull(pts)


@numba.njit(cache=True, fastmath=True)
def _min_distance_to_relative_hull_at_time(
    rel_pos0: np.ndarray,
    a_heading0_deg: float,
    a_target_heading_deg: float,
    a_turn_rate_deg_sec: float,
    a_min_speed_mps: float,
    a_max_speed_mps: float,
    b_heading0_deg: float,
    b_target_heading_deg: float,
    b_turn_rate_deg_sec: float,
    b_min_speed_mps: float,
    b_max_speed_mps: float,
    t_s: float,
) -> float:
    """
    Return the exact worst-case relative distance at time `t_s`.

    This is the distance from the origin to the fixed-time reachable relative-position
    hull produced by `_relative_position_hull_at_time`.
    """
    # The exact worst-case distance at a fixed time is just the distance from the origin
    # to the reachable relative-position hull at that time.
    hull = _relative_position_hull_at_time(
        rel_pos0=rel_pos0,
        a_heading0_deg=a_heading0_deg,
        a_target_heading_deg=a_target_heading_deg,
        a_turn_rate_deg_sec=a_turn_rate_deg_sec,
        a_min_speed_mps=a_min_speed_mps,
        a_max_speed_mps=a_max_speed_mps,
        b_heading0_deg=b_heading0_deg,
        b_target_heading_deg=b_target_heading_deg,
        b_turn_rate_deg_sec=b_turn_rate_deg_sec,
        b_min_speed_mps=b_min_speed_mps,
        b_max_speed_mps=b_max_speed_mps,
        t_s=t_s,
    )
    origin = np.zeros(2, dtype=np.float64)
    _closest_point, min_dist_m = _closest_point_on_convex_polygon(origin, hull)
    return min_dist_m


@numba.njit(cache=True, fastmath=True)
def _interval_distance_lower_bound(start_dist_m: float, end_dist_m: float, lipschitz_mps: float, dt_s: float) -> float:
    """
    Lower bound the minimum of an L-Lipschitz distance function over an interval.

    If `d(t)` is `L`-Lipschitz on `[t0, t1]`, then the minimum achievable value on the
    interval is at least `(d(t0) + d(t1) - L * (t1 - t0)) / 2`.

    This is the certificate used by the turn-aware adaptive search: if the bound is
    still above the separation threshold, the whole interval can be declared safe
    without any additional sampling inside it.
    """
    return max(0.0, 0.5 * (start_dist_m + end_dist_m - lipschitz_mps * dt_s))


@numba.njit(cache=True, fastmath=True)
def _turn_speed_schedule_buffer_m(
    heading0_deg: float,
    target_heading_deg: float,
    turn_rate_deg_sec: float,
    projection_time_s: float,
    turn_speed_schedule_uncertainty_kt: float,
) -> float:
    """
    Return a one-aircraft robustness buffer for bounded speed variation during a turn.

    The core turn-aware solver assumes one constant speed over the horizon. If, in
    reality, an aircraft may vary its instantaneous speed within
    `nominal +/- turn_speed_schedule_uncertainty_kt` while the turn is active, then the
    fixed-time position can move away from the constant-speed model segment. This helper
    returns a conservative bound on that mismatch, assuming the speed variation only
    occurs during the active turning portion of the horizon.

    The bound is

        0.5 * t_active * Delta_s * sin(theta_active / 2),

    where `Delta_s` is the full width of the in-turn speed interval, `t_active` is the
    amount of turn time seen within the projection horizon, and `theta_active` is the
    corresponding heading sweep in radians.
    """
    if projection_time_s <= 0.0 or turn_speed_schedule_uncertainty_kt <= 0.0:
        return 0.0

    turn_angle_deg = abs(heading_diff(heading0_deg, target_heading_deg))
    if turn_angle_deg < TURN_HEADING_EPS_DEG or abs(turn_rate_deg_sec) <= TURN_RATE_EPS_DEG_PER_S:
        return 0.0

    turn_duration_s = turn_angle_deg / abs(turn_rate_deg_sec)
    active_turn_time_s = min(projection_time_s, turn_duration_s)
    active_turn_sweep_deg = min(turn_angle_deg, abs(turn_rate_deg_sec) * projection_time_s)

    if active_turn_time_s <= 0.0 or active_turn_sweep_deg <= 0.0:
        return 0.0

    delta_speed_mps = 2.0 * turn_speed_schedule_uncertainty_kt * KT_TO_MPS
    active_turn_sweep_rad = active_turn_sweep_deg * DEG_TO_RAD
    return 0.5 * active_turn_time_s * delta_speed_mps * math.sin(0.5 * active_turn_sweep_rad)


@numba.njit(cache=True, fastmath=True)
def catch_up_projection_interval_with_turns(
    a_lat: float,
    a_lon: float,
    a_heading0_deg: float,
    a_target_heading_deg: float,
    a_speed_kt: float,
    a_turn_rate_deg_sec: float,
    b_lat: float,
    b_lon: float,
    b_heading0_deg: float,
    b_target_heading_deg: float,
    b_speed_kt: float,
    b_turn_rate_deg_sec: float,
    separation_threshold_m: float,
    speed_diff_kt: float,
    projection_time_s: float,
    use_interval_local_lipschitz: bool = TURN_USE_INTERVAL_LOCAL_LIPSCHITZ_DEFAULT,
    turn_speed_schedule_uncertainty_kt: float = TURN_SPEED_SCHEDULE_UNCERTAINTY_KT_DEFAULT,
) -> tuple[bool, float, float]:
    """
    Evaluate horizontal separation when one or both aircraft may execute one turn.

    For turning cases, this function is exact in speed and conservative in time:
    at each sampled time it computes the exact reachable relative-position hull for
    the constant-speed envelope, then uses an adaptive time search with a Lipschitz
    bound to certify intervals as safe. Intervals that cannot be certified at the
    configured minimum width are treated as unsafe.

    For pairs where neither aircraft triggers the turn-aware path under
    `TURN_HEADING_EPS_DEG` and `TURN_RATE_EPS_DEG_PER_S`, it delegates directly to the
    exact straight-line `catch_up_projection_interval`.

    Parameters
    ----------
    a_lat, a_lon : float
        Aircraft A current position.
    a_heading0_deg : float
        Aircraft A current heading (start of turn, or straight heading).
    a_target_heading_deg : float
        Aircraft A heading after turn (== heading0 if straight).
    a_speed_kt : float
        Aircraft A nominal speed in knots.
    a_turn_rate_deg_sec : float
        Aircraft A rate of turn in deg/s; 0.0 if straight.
    b_lat, b_lon : float
        Aircraft B current position.
    b_heading0_deg : float
        Aircraft B current heading.
    b_target_heading_deg : float
        Aircraft B heading after turn.
    b_speed_kt : float
        Aircraft B nominal speed in knots.
    b_turn_rate_deg_sec : float
        Aircraft B rate of turn in deg/s; 0.0 if straight.
    separation_threshold_m : float
        Required spacing in metres.
    speed_diff_kt : float
        Symmetric speed uncertainty in knots.
    projection_time_s : float
        Total projection horizon in seconds.
    use_interval_local_lipschitz : bool, optional
        If True, use the tighter interval-local Lipschitz bound derived from the
        commanded heading profiles. If False, use the simpler global bound
        `a_max + b_max`.
    turn_speed_schedule_uncertainty_kt : float, optional
        Optional robustness assumption for turning aircraft only. If positive, the
        function inflates the separation threshold by a conservative additive buffer
        corresponding to instantaneous in-turn speed variation within
        `nominal +/- turn_speed_schedule_uncertainty_kt`, even though the core reachable
        set still assumes one constant speed over the horizon. The buffer is zero for
        straight aircraft and is disabled by default.

    Returns
    -------
    tuple[bool, float, float]
        `(is_separated, min_distance_m, closest_time_s)`.

        For turning cases, `min_distance_m` and `closest_time_s` are informative
        sampled/certified estimates. The primary contract is the boolean separation
        result, which is sound with respect to the modelled envelope. If
        `turn_speed_schedule_uncertainty_kt > 0`, the boolean decision also includes
        the requested robustness buffer, while the distance/time pair remains a raw
        diagnostic of the constant-speed geometry.

    Notes
    -----
    The algorithm is:

    1. Evaluate the exact reachable relative-position hull at the interval endpoints.
    2. Use the endpoint distances and a Lipschitz constant to test whether the whole
       interval is already certified safe. By default this Lipschitz constant is the
       exact interval-local supremum of the corner relative speeds; the simpler global
       bound `a_max + b_max` remains available as an option.
    3. If not, split the interval at its midpoint, evaluate that fixed-time hull
       exactly, and continue with the two child intervals using an explicit DFS stack.
    4. Treat any interval shorter than `TURN_TIME_CERT_MIN_INTERVAL_S` that still
       cannot be certified as potentially unsafe.

    So the boolean return value is a certificate about the whole horizon, while the
    distance/time pair is a compact summary of the most critical sampled or certified
    point encountered during the search. The optional turn-speed-schedule buffer is
    applied by inflating the threshold, not by changing the reachable-set geometry.
    """
    a_turn_angle_deg = heading_diff(a_heading0_deg, a_target_heading_deg)
    b_turn_angle_deg = heading_diff(b_heading0_deg, b_target_heading_deg)
    a_is_turning = abs(a_turn_angle_deg) >= TURN_HEADING_EPS_DEG and abs(a_turn_rate_deg_sec) > TURN_RATE_EPS_DEG_PER_S
    b_is_turning = abs(b_turn_angle_deg) >= TURN_HEADING_EPS_DEG and abs(b_turn_rate_deg_sec) > TURN_RATE_EPS_DEG_PER_S

    if not a_is_turning and not b_is_turning:
        return catch_up_projection_interval(
            a_lat=a_lat,
            a_lon=a_lon,
            a_heading=a_target_heading_deg,
            a_speed_kt=a_speed_kt,
            b_lat=b_lat,
            b_lon=b_lon,
            b_heading=b_target_heading_deg,
            b_speed_kt=b_speed_kt,
            separation_threshold_m=separation_threshold_m,
            speed_diff_kt=speed_diff_kt,
            projection_time_s=projection_time_s,
        )

    # Convert the speed envelope once up front; all later geometry is done in metres and
    # seconds.
    a_min_speed_mps = max((a_speed_kt - speed_diff_kt), 0.0) * KT_TO_MPS
    a_max_speed_mps = (a_speed_kt + speed_diff_kt) * KT_TO_MPS
    b_min_speed_mps = max((b_speed_kt - speed_diff_kt), 0.0) * KT_TO_MPS
    b_max_speed_mps = (b_speed_kt + speed_diff_kt) * KT_TO_MPS

    # Relative position is always represented as A minus B in the local tangent plane.
    rel_pos0 = -latlon_to_local_xy(b_lat, b_lon, ref_lat=a_lat, ref_lon=a_lon)

    # A conservative global Lipschitz constant for distance is the maximum possible
    # closing rate, bounded by the sum of the two maximum speeds. The default path
    # tightens this per interval using the known heading profiles, but the simple global
    # bound is retained as an opt-out option for debugging and comparison.
    global_lipschitz_mps = a_max_speed_mps + b_max_speed_mps

    # Optional robustness buffer for speed variation during active turns. This keeps the
    # geometry and certification logic unchanged and instead inflates the required
    # threshold by a horizon-wide conservative margin.
    turn_speed_schedule_buffer_m = _turn_speed_schedule_buffer_m(
        heading0_deg=a_heading0_deg,
        target_heading_deg=a_target_heading_deg,
        turn_rate_deg_sec=a_turn_rate_deg_sec,
        projection_time_s=projection_time_s,
        turn_speed_schedule_uncertainty_kt=turn_speed_schedule_uncertainty_kt,
    ) + _turn_speed_schedule_buffer_m(
        heading0_deg=b_heading0_deg,
        target_heading_deg=b_target_heading_deg,
        turn_rate_deg_sec=b_turn_rate_deg_sec,
        projection_time_s=projection_time_s,
        turn_speed_schedule_uncertainty_kt=turn_speed_schedule_uncertainty_kt,
    )
    effective_separation_threshold_m = separation_threshold_m + turn_speed_schedule_buffer_m

    if projection_time_s <= 0.0:
        min_dist_now = float(np.linalg.norm(rel_pos0))
        return min_dist_now >= separation_threshold_m, min_dist_now, 0.0

    # Always check the horizon endpoints explicitly before doing any interval logic.
    start_dist_m = float(np.linalg.norm(rel_pos0))
    if start_dist_m < separation_threshold_m:
        return False, start_dist_m, 0.0

    end_dist_m = _min_distance_to_relative_hull_at_time(
        rel_pos0=rel_pos0,
        a_heading0_deg=a_heading0_deg,
        a_target_heading_deg=a_target_heading_deg,
        a_turn_rate_deg_sec=a_turn_rate_deg_sec,
        a_min_speed_mps=a_min_speed_mps,
        a_max_speed_mps=a_max_speed_mps,
        b_heading0_deg=b_heading0_deg,
        b_target_heading_deg=b_target_heading_deg,
        b_turn_rate_deg_sec=b_turn_rate_deg_sec,
        b_min_speed_mps=b_min_speed_mps,
        b_max_speed_mps=b_max_speed_mps,
        t_s=projection_time_s,
    )

    # These track the smallest sampled distance we have seen. They are useful both for
    # diagnostics and for early conflict detection when a sampled midpoint is already bad.
    best_sample_dist_m = start_dist_m
    best_sample_time_s = 0.0
    if end_dist_m < best_sample_dist_m:
        best_sample_dist_m = end_dist_m
        best_sample_time_s = projection_time_s

    # If the whole horizon already certifies as safe from the endpoint distances, we can
    # return immediately without any subdivision.
    initial_lower_bound_m = _interval_distance_lower_bound(
        start_dist_m=start_dist_m,
        end_dist_m=end_dist_m,
        lipschitz_mps=(
            _interval_local_lipschitz_mps(
                a_heading0_deg=a_heading0_deg,
                a_target_heading_deg=a_target_heading_deg,
                a_turn_rate_deg_sec=a_turn_rate_deg_sec,
                a_min_speed_mps=a_min_speed_mps,
                a_max_speed_mps=a_max_speed_mps,
                b_heading0_deg=b_heading0_deg,
                b_target_heading_deg=b_target_heading_deg,
                b_turn_rate_deg_sec=b_turn_rate_deg_sec,
                b_min_speed_mps=b_min_speed_mps,
                b_max_speed_mps=b_max_speed_mps,
                t0_s=0.0,
                t1_s=projection_time_s,
                global_lipschitz_mps=global_lipschitz_mps,
            )
            if use_interval_local_lipschitz
            else global_lipschitz_mps
        ),
        dt_s=projection_time_s,
    )
    if initial_lower_bound_m >= effective_separation_threshold_m:
        return True, best_sample_dist_m, best_sample_time_s

    # Depth-first interval subdivision uses an explicit stack because Numba handles
    # arrays well but not Python recursion.
    stack_capacity = 4
    span_s = projection_time_s
    while span_s > TURN_TIME_CERT_MIN_INTERVAL_S:
        stack_capacity += 1
        span_s *= 0.5

    left_t_stack = np.empty(stack_capacity, dtype=np.float64)
    right_t_stack = np.empty(stack_capacity, dtype=np.float64)
    left_d_stack = np.empty(stack_capacity, dtype=np.float64)
    right_d_stack = np.empty(stack_capacity, dtype=np.float64)

    top = 0
    left_t_stack[top] = 0.0
    right_t_stack[top] = projection_time_s
    left_d_stack[top] = start_dist_m
    right_d_stack[top] = end_dist_m
    top += 1

    # If we fail to certify a tiny interval, report the most critical lower bound we saw
    # rather than an arbitrary large sampled distance.
    most_critical_lower_bound_m = initial_lower_bound_m
    most_critical_time_s = 0.5 * projection_time_s

    while top > 0:
        top -= 1
        left_t_s = left_t_stack[top]
        right_t_s = right_t_stack[top]
        left_dist_m = left_d_stack[top]
        right_dist_m = right_d_stack[top]
        dt_s = right_t_s - left_t_s
        interval_lipschitz_mps = (
            _interval_local_lipschitz_mps(
                a_heading0_deg=a_heading0_deg,
                a_target_heading_deg=a_target_heading_deg,
                a_turn_rate_deg_sec=a_turn_rate_deg_sec,
                a_min_speed_mps=a_min_speed_mps,
                a_max_speed_mps=a_max_speed_mps,
                b_heading0_deg=b_heading0_deg,
                b_target_heading_deg=b_target_heading_deg,
                b_turn_rate_deg_sec=b_turn_rate_deg_sec,
                b_min_speed_mps=b_min_speed_mps,
                b_max_speed_mps=b_max_speed_mps,
                t0_s=left_t_s,
                t1_s=right_t_s,
                global_lipschitz_mps=global_lipschitz_mps,
            )
            if use_interval_local_lipschitz
            else global_lipschitz_mps
        )

        interval_lower_bound_m = _interval_distance_lower_bound(
            start_dist_m=left_dist_m,
            end_dist_m=right_dist_m,
            lipschitz_mps=interval_lipschitz_mps,
            dt_s=dt_s,
        )
        if interval_lower_bound_m < most_critical_lower_bound_m:
            most_critical_lower_bound_m = interval_lower_bound_m
            most_critical_time_s = 0.5 * (left_t_s + right_t_s)

        # This whole interval is certified safe; no need to look inside it.
        if interval_lower_bound_m >= effective_separation_threshold_m:
            continue

        # At the minimum certification width we stop refining. If the interval still
        # cannot be certified, we conservatively report it as potentially unsafe.
        if dt_s <= TURN_TIME_CERT_MIN_INTERVAL_S:
            if best_sample_dist_m < separation_threshold_m:
                return False, best_sample_dist_m, best_sample_time_s
            return False, most_critical_lower_bound_m, most_critical_time_s

        # Sample the midpoint to tighten both halves of the interval. If the midpoint
        # itself is already within the threshold, we have a direct witness of conflict.
        mid_t_s = 0.5 * (left_t_s + right_t_s)
        mid_dist_m = _min_distance_to_relative_hull_at_time(
            rel_pos0=rel_pos0,
            a_heading0_deg=a_heading0_deg,
            a_target_heading_deg=a_target_heading_deg,
            a_turn_rate_deg_sec=a_turn_rate_deg_sec,
            a_min_speed_mps=a_min_speed_mps,
            a_max_speed_mps=a_max_speed_mps,
            b_heading0_deg=b_heading0_deg,
            b_target_heading_deg=b_target_heading_deg,
            b_turn_rate_deg_sec=b_turn_rate_deg_sec,
            b_min_speed_mps=b_min_speed_mps,
            b_max_speed_mps=b_max_speed_mps,
            t_s=mid_t_s,
        )
        if mid_dist_m < best_sample_dist_m:
            best_sample_dist_m = mid_dist_m
            best_sample_time_s = mid_t_s

        if mid_dist_m < separation_threshold_m:
            return False, best_sample_dist_m, best_sample_time_s

        # Push the two child intervals. DFS keeps memory small and tends to find nearby
        # conflicts quickly once an interval starts looking critical.
        left_t_stack[top] = mid_t_s
        right_t_stack[top] = right_t_s
        left_d_stack[top] = mid_dist_m
        right_d_stack[top] = right_dist_m
        top += 1

        left_t_stack[top] = left_t_s
        right_t_stack[top] = mid_t_s
        left_d_stack[top] = left_dist_m
        right_d_stack[top] = mid_dist_m
        top += 1

    return True, best_sample_dist_m, best_sample_time_s


# ==========================================================================
# Demo Helpers And Canned Scenarios
# ==========================================================================
#
# Everything below this line is auxiliary support for the interactive FastAPI
# visualiser and the standalone `python -m falcon.optimisation.relevant_aircraft`
# demos. Keeping it in this file preserves a single handoff-friendly module that
# contains both the core reachability logic and the demo scaffolding built around
# it. The core solver above does not depend on any of these helpers.

DEMO_REF_LAT = 51.0
DEMO_REF_LON = -1.0
DEMO_NMI_TO_M = 1852.0
DEMO_M_TO_NMI = 1.0 / DEMO_NMI_TO_M
CUSTOM_TURN_PRESET_ID = "custom"


@dataclass(frozen=True, slots=True)
class TurnDemoScenario:
    """Serializable canned scenario used by the app and the standalone turn demo."""

    preset_id: str
    title: str
    a_lat: float
    a_lon: float
    a_heading_deg: float
    a_target_heading_deg: float
    a_speed_kt: float
    a_turn_rate_deg_sec: float
    b_east_m: float
    b_north_m: float
    b_heading_deg: float
    b_target_heading_deg: float
    b_speed_kt: float
    b_turn_rate_deg_sec: float
    speed_diff_kt: float
    separation_threshold_nm: float
    projection_time_min: float
    view_half_extent_nm: float
    grid_density: int = 75

    def b_latlon(self) -> tuple[float, float]:
        return local_xy_to_latlon(self.b_east_m, self.b_north_m, self.a_lat, self.a_lon)

    def to_visual_params(self) -> dict[str, float | int | str]:
        b_lat, b_lon = self.b_latlon()
        return {
            "model_mode": "turn_aware",
            "turn_preset_id": self.preset_id,
            "separation_threshold_nm": self.separation_threshold_nm,
            "speed_diff_kt": self.speed_diff_kt,
            "projection_time_min": self.projection_time_min,
            "grid_density": self.grid_density,
            "view_half_extent_nm": self.view_half_extent_nm,
            "a_lat": self.a_lat,
            "a_lon": self.a_lon,
            "a_heading": self.a_heading_deg,
            "a_target_heading_deg": self.a_target_heading_deg,
            "a_speed_kt": self.a_speed_kt,
            "a_turn_rate_deg_sec": self.a_turn_rate_deg_sec,
            "b_lat": b_lat,
            "b_lon": b_lon,
            "b_heading": self.b_heading_deg,
            "b_target_heading_deg": self.b_target_heading_deg,
            "b_speed_kt": self.b_speed_kt,
            "b_turn_rate_deg_sec": self.b_turn_rate_deg_sec,
            "turn_speed_schedule_uncertainty_kt": 0.0,
        }


TURN_DEMO_SCENARIOS: tuple[TurnDemoScenario, ...] = (
    TurnDemoScenario(
        preset_id="single_turn_dodge",
        title="Single-turn dodge",
        a_lat=DEMO_REF_LAT,
        a_lon=DEMO_REF_LON,
        a_heading_deg=0.0,
        a_target_heading_deg=270.0,
        a_speed_kt=340.0,
        a_turn_rate_deg_sec=1.0,
        b_east_m=-12000.0,
        b_north_m=-4000.0,
        b_heading_deg=90.0,
        b_target_heading_deg=90.0,
        b_speed_kt=340.0,
        b_turn_rate_deg_sec=0.0,
        speed_diff_kt=25.0,
        separation_threshold_nm=5.0,
        projection_time_min=15.0,
        view_half_extent_nm=18.0,
    ),
    TurnDemoScenario(
        preset_id="single_turn_cut_in",
        title="Single-turn cut-in",
        a_lat=DEMO_REF_LAT,
        a_lon=DEMO_REF_LON,
        a_heading_deg=135.0,
        a_target_heading_deg=135.0,
        a_speed_kt=340.0,
        a_turn_rate_deg_sec=0.0,
        b_east_m=8000.0,
        b_north_m=-8000.0,
        b_heading_deg=180.0,
        b_target_heading_deg=225.0,
        b_speed_kt=360.0,
        b_turn_rate_deg_sec=1.0,
        speed_diff_kt=25.0,
        separation_threshold_nm=5.0,
        projection_time_min=10.0,
        view_half_extent_nm=18.0,
    ),
    TurnDemoScenario(
        preset_id="mutual_opening_turns",
        title="Mutual opening turns",
        a_lat=DEMO_REF_LAT,
        a_lon=DEMO_REF_LON,
        a_heading_deg=180.0,
        a_target_heading_deg=135.0,
        a_speed_kt=380.0,
        a_turn_rate_deg_sec=1.0,
        b_east_m=4000.0,
        b_north_m=-12000.0,
        b_heading_deg=225.0,
        b_target_heading_deg=270.0,
        b_speed_kt=320.0,
        b_turn_rate_deg_sec=1.0,
        speed_diff_kt=25.0,
        separation_threshold_nm=5.0,
        projection_time_min=15.0,
        view_half_extent_nm=20.0,
    ),
    TurnDemoScenario(
        preset_id="mutual_closing_turns",
        title="Mutual closing turns",
        a_lat=DEMO_REF_LAT,
        a_lon=DEMO_REF_LON,
        a_heading_deg=180.0,
        a_target_heading_deg=135.0,
        a_speed_kt=360.0,
        a_turn_rate_deg_sec=1.0,
        b_east_m=8000.0,
        b_north_m=8000.0,
        b_heading_deg=270.0,
        b_target_heading_deg=225.0,
        b_speed_kt=360.0,
        b_turn_rate_deg_sec=1.0,
        speed_diff_kt=25.0,
        separation_threshold_nm=5.0,
        projection_time_min=10.0,
        view_half_extent_nm=18.0,
    ),
)


def get_turn_preset_options() -> list[dict[str, Any]]:
    """Return canned turn scenarios in the UI payload format used by the app."""
    options: list[dict[str, Any]] = [{"id": CUSTOM_TURN_PRESET_ID, "title": "Custom", "params": None}]
    options.extend(
        {"id": scenario.preset_id, "title": scenario.title, "params": scenario.to_visual_params()}
        for scenario in TURN_DEMO_SCENARIOS
    )
    return options


def speed_bounds_mps(speed_kt: float, speed_diff_kt: float) -> tuple[float, float]:
    """Return the symmetric min/max speed envelope for a nominal speed."""
    return max(speed_kt - speed_diff_kt, 0.0) * KT_TO_MPS, (speed_kt + speed_diff_kt) * KT_TO_MPS


def local_xy_to_latlon(east_m: float, north_m: float, ref_lat: float, ref_lon: float) -> tuple[float, float]:
    """Approximate inverse of `latlon_to_local_xy` for demo scenario setup."""
    ref_lat_rad = ref_lat * DEG_TO_RAD
    lat = ref_lat + (north_m / EARTH_RADIUS_IN_METERS) * RAD_TO_DEG
    lon = ref_lon + (east_m / (EARTH_RADIUS_IN_METERS * math.cos(ref_lat_rad))) * RAD_TO_DEG
    return lat, lon


def turn_duration_s(heading0_deg: float, target_heading_deg: float, turn_rate_deg_sec: float) -> float:
    """Return the deterministic duration of a commanded constant-rate turn."""
    if turn_rate_deg_sec == 0.0 or heading0_deg == target_heading_deg:
        return 0.0
    return abs(heading_diff(heading0_deg, target_heading_deg, abs_diff=True)) / abs(turn_rate_deg_sec)


def heading_summary(
    heading0_deg: float,
    target_heading_deg: float,
    turn_rate_deg_sec: float,
    speed_kt: float,
) -> str:
    """Format one aircraft's lateral plan for debug text overlays and tooltips."""
    if heading0_deg == target_heading_deg:
        return f"{heading0_deg:.0f} deg straight @ {speed_kt:.0f} kt"
    if turn_rate_deg_sec == 0.0:
        return f"instant {heading0_deg:.0f}->{target_heading_deg:.0f} deg @ {speed_kt:.0f} kt"
    return f"{heading0_deg:.0f}->{target_heading_deg:.0f} deg @ {turn_rate_deg_sec:.1f} deg/s, {speed_kt:.0f} kt"


def sample_trajectory_xy(
    start_xy: np.ndarray,
    heading0_deg: float,
    target_heading_deg: float,
    speed_mps: float,
    turn_rate_deg_sec: float,
    times_s: np.ndarray,
) -> np.ndarray:
    """Sample one exact turn-then-straight trajectory in local XY."""
    xy = np.empty((times_s.shape[0], 2), dtype=np.float64)
    for idx in range(times_s.shape[0]):
        basis = _turn_displacement_basis(heading0_deg, target_heading_deg, turn_rate_deg_sec, float(times_s[idx]))
        xy[idx, 0] = start_xy[0] + speed_mps * basis[0]
        xy[idx, 1] = start_xy[1] + speed_mps * basis[1]
    return xy


def speed_segment_at_time(
    start_xy: np.ndarray,
    heading0_deg: float,
    target_heading_deg: float,
    turn_rate_deg_sec: float,
    nominal_speed_mps: float,
    min_speed_mps: float,
    max_speed_mps: float,
    t_s: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return the min/nominal/max positions for one aircraft at time `t_s`."""
    basis = _turn_displacement_basis(heading0_deg, target_heading_deg, turn_rate_deg_sec, t_s)
    pos_min = start_xy + min_speed_mps * basis  # pyright: ignore[reportOperatorIssue]
    pos_nom = start_xy + nominal_speed_mps * basis  # pyright: ignore[reportOperatorIssue]
    pos_max = start_xy + max_speed_mps * basis  # pyright: ignore[reportOperatorIssue]
    return pos_min, pos_nom, pos_max


def envelope_distance_curve(
    rel_pos0: np.ndarray,
    a_heading0_deg: float,
    a_target_heading_deg: float,
    a_turn_rate_deg_sec: float,
    a_min_speed_mps: float,
    a_max_speed_mps: float,
    b_heading0_deg: float,
    b_target_heading_deg: float,
    b_turn_rate_deg_sec: float,
    b_min_speed_mps: float,
    b_max_speed_mps: float,
    times_s: np.ndarray,
) -> np.ndarray:
    """Sample the exact fixed-time envelope distance `d(t)` over a time grid."""
    distances_m = np.empty(times_s.shape[0], dtype=np.float64)
    for idx in range(times_s.shape[0]):
        distances_m[idx] = _min_distance_to_relative_hull_at_time(
            rel_pos0=rel_pos0,
            a_heading0_deg=a_heading0_deg,
            a_target_heading_deg=a_target_heading_deg,
            a_turn_rate_deg_sec=a_turn_rate_deg_sec,
            a_min_speed_mps=a_min_speed_mps,
            a_max_speed_mps=a_max_speed_mps,
            b_heading0_deg=b_heading0_deg,
            b_target_heading_deg=b_target_heading_deg,
            b_turn_rate_deg_sec=b_turn_rate_deg_sec,
            b_min_speed_mps=b_min_speed_mps,
            b_max_speed_mps=b_max_speed_mps,
            t_s=float(times_s[idx]),
        )
    return distances_m


def corridor_polygon(traj_min: np.ndarray, traj_max: np.ndarray) -> np.ndarray:
    """Return a closed polygon spanning the min/max speed trajectory corridor."""
    return np.vstack((traj_min, traj_max[::-1]))


def focus_window_end_times(
    projection_time_s: float,
    closest_time_s: float,
    a_turn_end_s: float,
    b_turn_end_s: float,
) -> tuple[float, float]:
    """Choose spatial and distance-curve windows that focus on the relevant interaction."""
    spatial_focus_end_s = min(
        projection_time_s,
        max(150.0, closest_time_s + 60.0, a_turn_end_s + 30.0, b_turn_end_s + 30.0),
    )
    time_focus_end_s = min(
        projection_time_s,
        max(180.0, closest_time_s + 90.0, a_turn_end_s + 45.0, b_turn_end_s + 45.0),
    )
    return spatial_focus_end_s, time_focus_end_s


def square_bounds(points_xy: np.ndarray, pad_m: float) -> dict[str, float]:
    """Return square XY bounds with padding around a point cloud."""
    min_x = float(np.min(points_xy[:, 0]))
    max_x = float(np.max(points_xy[:, 0]))
    min_y = float(np.min(points_xy[:, 1]))
    max_y = float(np.max(points_xy[:, 1]))
    centre_x = 0.5 * (min_x + max_x)
    centre_y = 0.5 * (min_y + max_y)
    half_span_m = 0.5 * max(max_x - min_x, max_y - min_y) + pad_m
    return {
        "x_min": centre_x - half_span_m,
        "x_max": centre_x + half_span_m,
        "y_min": centre_y - half_span_m,
        "y_max": centre_y + half_span_m,
        "half_span_m": half_span_m,
    }


def build_turn_debug_payload(
    *,
    a_lat: float,
    a_lon: float,
    a_heading0_deg: float,
    a_target_heading_deg: float,
    a_speed_kt: float,
    a_turn_rate_deg_sec: float,
    b_lat: float,
    b_lon: float,
    b_heading0_deg: float,
    b_target_heading_deg: float,
    b_speed_kt: float,
    b_turn_rate_deg_sec: float,
    separation_threshold_m: float,
    speed_diff_kt: float,
    projection_time_s: float,
    turn_speed_schedule_uncertainty_kt: float = 0.0,
) -> dict[str, Any]:
    """Build the full turn-aware debug payload consumed by the web app."""
    is_sep, min_distance_m, closest_time_s = catch_up_projection_interval_with_turns(
        a_lat=a_lat,
        a_lon=a_lon,
        a_heading0_deg=a_heading0_deg,
        a_target_heading_deg=a_target_heading_deg,
        a_speed_kt=a_speed_kt,
        a_turn_rate_deg_sec=a_turn_rate_deg_sec,
        b_lat=b_lat,
        b_lon=b_lon,
        b_heading0_deg=b_heading0_deg,
        b_target_heading_deg=b_target_heading_deg,
        b_speed_kt=b_speed_kt,
        b_turn_rate_deg_sec=b_turn_rate_deg_sec,
        separation_threshold_m=separation_threshold_m,
        speed_diff_kt=speed_diff_kt,
        projection_time_s=projection_time_s,
        turn_speed_schedule_uncertainty_kt=turn_speed_schedule_uncertainty_kt,
    )

    a_nom_speed_mps = a_speed_kt * KT_TO_MPS
    b_nom_speed_mps = b_speed_kt * KT_TO_MPS
    a_min_speed_mps, a_max_speed_mps = speed_bounds_mps(a_speed_kt, speed_diff_kt)
    b_min_speed_mps, b_max_speed_mps = speed_bounds_mps(b_speed_kt, speed_diff_kt)
    a_turn_end_s = turn_duration_s(a_heading0_deg, a_target_heading_deg, a_turn_rate_deg_sec)
    b_turn_end_s = turn_duration_s(b_heading0_deg, b_target_heading_deg, b_turn_rate_deg_sec)

    a_xy0 = latlon_to_local_xy(a_lat, a_lon, a_lat, a_lon)
    b_xy0 = latlon_to_local_xy(b_lat, b_lon, a_lat, a_lon)
    rel_pos0 = a_xy0 - b_xy0  # pyright: ignore[reportOperatorIssue]

    spatial_focus_end_s, time_focus_end_s = focus_window_end_times(
        projection_time_s,
        closest_time_s,
        a_turn_end_s,
        b_turn_end_s,
    )
    traj_sample_count = min(361, max(161, int(spatial_focus_end_s / 1.5) + 1))
    traj_times_s = np.linspace(0.0, spatial_focus_end_s, traj_sample_count)
    if 0.0 < closest_time_s < spatial_focus_end_s:
        traj_times_s = np.unique(np.concatenate((traj_times_s, np.asarray([closest_time_s], dtype=np.float64))))

    curve_sample_count = min(601, max(241, int(projection_time_s / 2.0) + 1))
    curve_times_s = np.linspace(0.0, projection_time_s, curve_sample_count)
    if 0.0 < closest_time_s < projection_time_s:
        curve_times_s = np.unique(np.concatenate((curve_times_s, np.asarray([closest_time_s], dtype=np.float64))))

    a_traj_nom = sample_trajectory_xy(
        a_xy0,
        a_heading0_deg,
        a_target_heading_deg,
        a_nom_speed_mps,
        a_turn_rate_deg_sec,
        traj_times_s,
    )
    b_traj_nom = sample_trajectory_xy(
        b_xy0,
        b_heading0_deg,
        b_target_heading_deg,
        b_nom_speed_mps,
        b_turn_rate_deg_sec,
        traj_times_s,
    )
    a_traj_min = sample_trajectory_xy(
        a_xy0,
        a_heading0_deg,
        a_target_heading_deg,
        a_min_speed_mps,
        a_turn_rate_deg_sec,
        traj_times_s,
    )
    a_traj_max = sample_trajectory_xy(
        a_xy0,
        a_heading0_deg,
        a_target_heading_deg,
        a_max_speed_mps,
        a_turn_rate_deg_sec,
        traj_times_s,
    )
    b_traj_min = sample_trajectory_xy(
        b_xy0,
        b_heading0_deg,
        b_target_heading_deg,
        b_min_speed_mps,
        b_turn_rate_deg_sec,
        traj_times_s,
    )
    b_traj_max = sample_trajectory_xy(
        b_xy0,
        b_heading0_deg,
        b_target_heading_deg,
        b_max_speed_mps,
        b_turn_rate_deg_sec,
        traj_times_s,
    )

    envelope_distances_m = envelope_distance_curve(
        rel_pos0=rel_pos0,
        a_heading0_deg=a_heading0_deg,
        a_target_heading_deg=a_target_heading_deg,
        a_turn_rate_deg_sec=a_turn_rate_deg_sec,
        a_min_speed_mps=a_min_speed_mps,
        a_max_speed_mps=a_max_speed_mps,
        b_heading0_deg=b_heading0_deg,
        b_target_heading_deg=b_target_heading_deg,
        b_turn_rate_deg_sec=b_turn_rate_deg_sec,
        b_min_speed_mps=b_min_speed_mps,
        b_max_speed_mps=b_max_speed_mps,
        times_s=curve_times_s,
    )
    a_curve_nom = sample_trajectory_xy(
        a_xy0,
        a_heading0_deg,
        a_target_heading_deg,
        a_nom_speed_mps,
        a_turn_rate_deg_sec,
        curve_times_s,
    )
    b_curve_nom = sample_trajectory_xy(
        b_xy0,
        b_heading0_deg,
        b_target_heading_deg,
        b_nom_speed_mps,
        b_turn_rate_deg_sec,
        curve_times_s,
    )
    nominal_distances_m = np.linalg.norm(a_curve_nom - b_curve_nom, axis=1)
    time_focus_mask = curve_times_s <= time_focus_end_s + 1e-9
    curve_times_focus_s = curve_times_s[time_focus_mask]
    envelope_distances_focus_nm = envelope_distances_m[time_focus_mask] * DEMO_M_TO_NMI
    nominal_distances_focus_nm = nominal_distances_m[time_focus_mask] * DEMO_M_TO_NMI

    frames: list[dict[str, Any]] = []
    relative_points: list[np.ndarray] = [np.zeros(2, dtype=np.float64)]
    closest_frame_index = 0
    closest_delta_s = float("inf")
    for idx, t_s in enumerate(traj_times_s):
        a_pos_min, a_pos_nom, a_pos_max = speed_segment_at_time(
            a_xy0,
            a_heading0_deg,
            a_target_heading_deg,
            a_turn_rate_deg_sec,
            a_nom_speed_mps,
            a_min_speed_mps,
            a_max_speed_mps,
            float(t_s),
        )
        b_pos_min, b_pos_nom, b_pos_max = speed_segment_at_time(
            b_xy0,
            b_heading0_deg,
            b_target_heading_deg,
            b_turn_rate_deg_sec,
            b_nom_speed_mps,
            b_min_speed_mps,
            b_max_speed_mps,
            float(t_s),
        )
        rel_hull = _relative_position_hull_at_time(
            rel_pos0=rel_pos0,
            a_heading0_deg=a_heading0_deg,
            a_target_heading_deg=a_target_heading_deg,
            a_turn_rate_deg_sec=a_turn_rate_deg_sec,
            a_min_speed_mps=a_min_speed_mps,
            a_max_speed_mps=a_max_speed_mps,
            b_heading0_deg=b_heading0_deg,
            b_target_heading_deg=b_target_heading_deg,
            b_turn_rate_deg_sec=b_turn_rate_deg_sec,
            b_min_speed_mps=b_min_speed_mps,
            b_max_speed_mps=b_max_speed_mps,
            t_s=float(t_s),
        )
        rel_nom = a_pos_nom - b_pos_nom  # pyright: ignore[reportOperatorIssue]
        rel_closest_point, _ = _closest_point_on_convex_polygon(np.zeros(2, dtype=np.float64), rel_hull)
        relative_points.extend((rel_hull, rel_nom[None, :], rel_closest_point[None, :]))
        nominal_distance_nm = float(np.linalg.norm(rel_nom) * DEMO_M_TO_NMI)
        envelope_distance_nm = float(np.linalg.norm(rel_closest_point) * DEMO_M_TO_NMI)
        frames.append(
            {
                "time_s": float(t_s),
                "time_min": float(t_s / 60.0),
                "a": {"min": a_pos_min.tolist(), "nominal": a_pos_nom.tolist(), "max": a_pos_max.tolist()},
                "b": {"min": b_pos_min.tolist(), "nominal": b_pos_nom.tolist(), "max": b_pos_max.tolist()},
                "relative_hull": rel_hull.tolist(),
                "relative_nominal": rel_nom.tolist(),
                "relative_closest_point": rel_closest_point.tolist(),
                "nominal_distance_nm": nominal_distance_nm,
                "envelope_distance_nm": envelope_distance_nm,
            }
        )
        delta_s = abs(float(t_s) - closest_time_s)
        if delta_s < closest_delta_s:
            closest_delta_s = delta_s
            closest_frame_index = idx

    a_nom_turn_end_xy = None
    if 0.0 < a_turn_end_s <= projection_time_s:
        a_nom_turn_end_xy = sample_trajectory_xy(
            a_xy0,
            a_heading0_deg,
            a_target_heading_deg,
            a_nom_speed_mps,
            a_turn_rate_deg_sec,
            np.asarray([a_turn_end_s], dtype=np.float64),
        )[0]
    b_nom_turn_end_xy = None
    if 0.0 < b_turn_end_s <= projection_time_s:
        b_nom_turn_end_xy = sample_trajectory_xy(
            b_xy0,
            b_heading0_deg,
            b_target_heading_deg,
            b_nom_speed_mps,
            b_turn_rate_deg_sec,
            np.asarray([b_turn_end_s], dtype=np.float64),
        )[0]

    spatial_points = [a_traj_nom, b_traj_nom, a_traj_min, a_traj_max, b_traj_min, b_traj_max]
    if a_nom_turn_end_xy is not None:
        spatial_points.append(a_nom_turn_end_xy[None, :])
    if b_nom_turn_end_xy is not None:
        spatial_points.append(b_nom_turn_end_xy[None, :])
    spatial_bounds = square_bounds(np.vstack(spatial_points), pad_m=max(0.35 * separation_threshold_m, 1500.0))
    relative_bounds = square_bounds(np.vstack(relative_points), pad_m=max(0.4 * separation_threshold_m, 750.0))

    return {
        "model_mode": "turn_aware",
        "origins": {"a": a_xy0.tolist(), "b": b_xy0.tolist()},
        "summary": {
            "is_separated": bool(is_sep),
            "min_distance_nm": float(min_distance_m * DEMO_M_TO_NMI),
            "closest_time_min": float(closest_time_s / 60.0),
            "closest_time_s": float(closest_time_s),
        },
        "frame_interval_s": float(traj_times_s[1] - traj_times_s[0]) if traj_times_s.shape[0] > 1 else 0.0,
        "closest_frame_index": closest_frame_index,
        "frames": frames,
        "trajectories": {
            "times_s": traj_times_s.tolist(),
            "a": {
                "nominal": a_traj_nom.tolist(),
                "min": a_traj_min.tolist(),
                "max": a_traj_max.tolist(),
                "corridor": corridor_polygon(a_traj_min, a_traj_max).tolist(),
            },
            "b": {
                "nominal": b_traj_nom.tolist(),
                "min": b_traj_min.tolist(),
                "max": b_traj_max.tolist(),
                "corridor": corridor_polygon(b_traj_min, b_traj_max).tolist(),
            },
        },
        "distance_curve": {
            "times_s": curve_times_focus_s.tolist(),
            "envelope_nm": envelope_distances_focus_nm.tolist(),
            "nominal_nm": nominal_distances_focus_nm.tolist(),
            "threshold_nm": float(separation_threshold_m * DEMO_M_TO_NMI),
        },
        "turns": {
            "a": {
                "start_heading_deg": float(a_heading0_deg),
                "target_heading_deg": float(a_target_heading_deg),
                "turn_rate_deg_sec": float(a_turn_rate_deg_sec),
                "end_time_s": float(a_turn_end_s),
                "end_position": None if a_nom_turn_end_xy is None else a_nom_turn_end_xy.tolist(),
            },
            "b": {
                "start_heading_deg": float(b_heading0_deg),
                "target_heading_deg": float(b_target_heading_deg),
                "turn_rate_deg_sec": float(b_turn_rate_deg_sec),
                "end_time_s": float(b_turn_end_s),
                "end_position": None if b_nom_turn_end_xy is None else b_nom_turn_end_xy.tolist(),
            },
        },
        "bounds_xy": spatial_bounds,
        "relative_bounds_xy": relative_bounds,
        "separation_threshold_m": float(separation_threshold_m),
        "info": {
            "a_heading_summary": heading_summary(a_heading0_deg, a_target_heading_deg, a_turn_rate_deg_sec, a_speed_kt),
            "b_heading_summary": heading_summary(b_heading0_deg, b_target_heading_deg, b_turn_rate_deg_sec, b_speed_kt),
            "speed_diff_kt": float(speed_diff_kt),
            "turn_speed_schedule_uncertainty_kt": float(turn_speed_schedule_uncertainty_kt),
            "spatial_focus_end_s": float(spatial_focus_end_s),
            "time_focus_end_s": float(time_focus_end_s),
            "projection_time_s": float(projection_time_s),
        },
    }


# ==========================================================================
# Plotting Helpers Used By The Standalone Demos
# ==========================================================================


@numba.jit(nopython=True, fastmath=True, cache=True, inline="always", error_model="numpy")
def move_location(
    lon: float,
    lat: float,
    dist_m: float,
    heading_deg: float,
    earth_radius_in_meters: float = EARTH_RADIUS_IN_METERS,
) -> tuple[float, float]:
    """
    Propagate a latitude/longitude point along a great-circle heading.

    This helper is only used by the standalone plotting demos, so it prioritises
    readability over the main module's strict local tangent-plane approximation.
    """
    if dist_m == 0.0:
        return lon, lat

    lat_rad = lat * DEG_TO_RAD
    lon_rad = lon * DEG_TO_RAD
    heading_rad = heading_deg * DEG_TO_RAD

    cos_lat_rad = math.cos(lat_rad)
    sin_lat_rad = math.sin(lat_rad)
    norm_dist = dist_m / earth_radius_in_meters
    cos_norm_dist = math.cos(norm_dist)
    sin_norm_dist = math.sin(norm_dist)
    cos_heading_rad = math.cos(heading_rad)
    sin_heading_rad = math.sin(heading_rad)

    lat_new = math.asin(sin_lat_rad * cos_norm_dist + cos_lat_rad * sin_norm_dist * cos_heading_rad)
    lon_new = lon_rad + math.atan2(
        sin_heading_rad * sin_norm_dist * cos_lat_rad,
        cos_norm_dist - sin_lat_rad * math.sin(lat_new),
    )

    return lon_new * RAD_TO_DEG, lat_new * RAD_TO_DEG


def draw_demo_heading_arrow(
    ax: Axes,
    xy: np.ndarray,
    heading_deg: float,
    length_m: float,
    color: str,
    style: str = "-",
    label: str | None = None,
) -> None:
    """Draw a heading arrow in the local XY plotting frame."""
    d = heading_to_unit_vector(heading_deg)
    dx, dy = float(d[0]) * length_m, float(d[1]) * length_m
    ax.annotate(
        "",
        xytext=(xy[0], xy[1]),
        xy=(xy[0] + dx, xy[1] + dy),
        arrowprops={"arrowstyle": "->", "color": color, "linestyle": style, "lw": 1.5},
    )
    if label:
        ax.annotate(label, (xy[0] + dx * 0.5, xy[1] + dy * 0.5), fontsize=8, color=color)


def set_demo_square_limits(ax: Axes, points_xy: np.ndarray, pad_m: float) -> float:
    """
    Fit a square plotting window around a point cloud.

    A square view preserves turn geometry visually and avoids long straight tails
    dominating one axis while hiding the interesting turn interaction.
    """
    min_x = float(np.min(points_xy[:, 0]))
    max_x = float(np.max(points_xy[:, 0]))
    min_y = float(np.min(points_xy[:, 1]))
    max_y = float(np.max(points_xy[:, 1]))
    centre_x = 0.5 * (min_x + max_x)
    centre_y = 0.5 * (min_y + max_y)
    half_span_m = 0.5 * max(max_x - min_x, max_y - min_y) + pad_m
    ax.set_xlim(centre_x - half_span_m, centre_x + half_span_m)
    ax.set_ylim(centre_y - half_span_m, centre_y + half_span_m)
    return half_span_m


#
# Standalone demo entrypoint
# --------------------------
# The executable demo path lives last so the file reads naturally as:
# 1. core solver
# 2. demo-support helpers
# 3. plotting helpers
# 4. command-line demo wiring
#
if __name__ == "__main__":
    import argparse

    from matplotlib.patches import Circle, Polygon
    import matplotlib.pyplot as plt

    parser = argparse.ArgumentParser(description="Relevant aircraft projection demos")
    parser.add_argument(
        "demo",
        choices=["heatmap", "turns"],
        help="'heatmap': straight-ahead relevant-region heatmaps; 'turns': concrete turn-aware scenarios",
    )
    args = parser.parse_args()

    # ===================================================================
    # DEMO: heatmap — straight-ahead relevant-region heatmaps
    # ===================================================================
    if args.demo == "heatmap":
        a_lat, a_lon = 0.0, 0.0
        a_speed_kt, b_speed_kt, speed_diff_kt = 400.0, 300.0, 50.0
        separation_threshold_m = 5.0 * DEMO_NMI_TO_M
        projection_time_s = 30.0 * 60.0

        n_lats, n_lons = 150, 150
        lat_min, lat_max = -0.5, 0.5
        lon_min, lon_max = -0.5, 0.5
        b_lats = np.linspace(lat_min, lat_max, n_lats)
        b_lons = np.linspace(lon_min, lon_max, n_lons)

        for b_heading in [0, 10, 45, 90, 135, 180, 225, 270, 315]:
            safe_matrix = np.zeros((n_lats, n_lons), dtype=bool)

            for i in range(n_lats):
                for j in range(n_lons):
                    is_sep, _, _ = catch_up_projection_interval_with_turns(
                        a_lat=a_lat,
                        a_lon=a_lon,
                        a_heading0_deg=0.0,
                        a_target_heading_deg=0.0,
                        a_speed_kt=a_speed_kt,
                        a_turn_rate_deg_sec=0.0,
                        b_lat=float(b_lats[i]),
                        b_lon=float(b_lons[j]),
                        b_heading0_deg=float(b_heading),
                        b_target_heading_deg=float(b_heading),
                        b_speed_kt=b_speed_kt,
                        b_turn_rate_deg_sec=0.0,
                        separation_threshold_m=separation_threshold_m,
                        speed_diff_kt=speed_diff_kt,
                        projection_time_s=projection_time_s,
                    )
                    safe_matrix[i, j] = is_sep

            b_lat_ex, b_lon_ex = -0.1, 0.1
            b_ext_lon, b_ext_lat = move_location(b_lon_ex, b_lat_ex, 5 * DEMO_NMI_TO_M, b_heading)
            a_ext_lon, a_ext_lat = move_location(a_lon, a_lat, 5 * DEMO_NMI_TO_M, 0.0)

            fig, ax = plt.subplots(figsize=(6, 6))
            b_lon_grid, b_lat_grid = np.meshgrid(b_lons, b_lats)
            ax.pcolormesh(b_lon_grid, b_lat_grid, safe_matrix, shading="auto", cmap="RdYlGn", alpha=0.5)
            ax.plot(a_lon, a_lat, "bo", label="Aircraft A")
            ax.annotate(
                "", xytext=(a_lon, a_lat), xy=(a_ext_lon, a_ext_lat), arrowprops={"arrowstyle": "->", "color": "b"}
            )
            ax.plot(b_lon_ex, b_lat_ex, "ko", label="Aircraft B")
            ax.annotate(
                "",
                xytext=(b_lon_ex, b_lat_ex),
                xy=(b_ext_lon, b_ext_lat),
                arrowprops={"arrowstyle": "->", "color": "k"},
            )
            ax.set_xlabel("Longitude")
            ax.set_ylabel("Latitude")
            ax.set_title(
                f"relevant region; B flying {b_heading}\u00b0; "
                f"A={a_speed_kt:g}kt, B={b_speed_kt:g}kt, \u0394={speed_diff_kt:g}kt"
            )
            ax.set_xlim(lon_min, lon_max)
            ax.set_ylim(lat_min, lat_max)
            ax.legend()
            plt.show()

    # ===================================================================
    # DEMO: turns — concrete turn-aware scenario plots
    # ===================================================================
    if args.demo == "turns":
        NMI = DEMO_NMI_TO_M
        COLOR_A = "tab:blue"
        COLOR_B = "tab:orange"
        COLOR_SAFE = "tab:green"
        COLOR_CONFLICT = "tab:red"

        for scenario in TURN_DEMO_SCENARIOS:
            title = scenario.title
            a_lat_v, a_lon_v = scenario.a_lat, scenario.a_lon
            a_h0, a_ht = scenario.a_heading_deg, scenario.a_target_heading_deg
            a_spd, a_turn_rate_deg_sec = scenario.a_speed_kt, scenario.a_turn_rate_deg_sec
            b_east_m, b_north_m = scenario.b_east_m, scenario.b_north_m
            b_lat_v, b_lon_v = local_xy_to_latlon(b_east_m, b_north_m, a_lat_v, a_lon_v)
            b_h0, b_ht = scenario.b_heading_deg, scenario.b_target_heading_deg
            b_spd, b_turn_rate_deg_sec = scenario.b_speed_kt, scenario.b_turn_rate_deg_sec
            sep_m = scenario.separation_threshold_nm * NMI
            speed_diff = scenario.speed_diff_kt
            proj_s = scenario.projection_time_min * 60.0
            a_nom_speed_mps = a_spd * KT_TO_MPS
            b_nom_speed_mps = b_spd * KT_TO_MPS
            a_min_speed_mps, a_max_speed_mps = speed_bounds_mps(a_spd, speed_diff)
            b_min_speed_mps, b_max_speed_mps = speed_bounds_mps(b_spd, speed_diff)
            a_turn_end_s = turn_duration_s(a_h0, a_ht, a_turn_rate_deg_sec)
            b_turn_end_s = turn_duration_s(b_h0, b_ht, b_turn_rate_deg_sec)

            # Run the turn-aware CUPI
            is_sep, min_dist, closest_time = catch_up_projection_interval_with_turns(
                a_lat=a_lat_v,
                a_lon=a_lon_v,
                a_heading0_deg=a_h0,
                a_target_heading_deg=a_ht,
                a_speed_kt=a_spd,
                a_turn_rate_deg_sec=a_turn_rate_deg_sec,
                b_lat=b_lat_v,
                b_lon=b_lon_v,
                b_heading0_deg=b_h0,
                b_target_heading_deg=b_ht,
                b_speed_kt=b_spd,
                b_turn_rate_deg_sec=b_turn_rate_deg_sec,
                separation_threshold_m=sep_m,
                speed_diff_kt=speed_diff,
                projection_time_s=proj_s,
            )

            # Work in the same local XY frame as the CUPI calculations.
            a_xy0 = latlon_to_local_xy(a_lat_v, a_lon_v, a_lat_v, a_lon_v)
            b_xy0 = latlon_to_local_xy(b_lat_v, b_lon_v, a_lat_v, a_lon_v)
            rel_pos0 = a_xy0 - b_xy0  # pyright: ignore[reportOperatorIssue]

            # The classifier looks over the full model horizon, but the demo should zoom
            # in on the part of the geometry where the turn actually matters.
            spatial_focus_end_s = min(
                proj_s,
                max(150.0, closest_time + 60.0, a_turn_end_s + 30.0, b_turn_end_s + 30.0),
            )
            time_focus_end_s = min(
                proj_s,
                max(180.0, closest_time + 90.0, a_turn_end_s + 45.0, b_turn_end_s + 45.0),
            )

            traj_sample_count = min(361, max(161, int(spatial_focus_end_s / 1.5) + 1))
            traj_times_s = np.linspace(0.0, spatial_focus_end_s, traj_sample_count)
            if 0.0 < closest_time < spatial_focus_end_s:
                traj_times_s = np.unique(np.concatenate((traj_times_s, np.asarray([closest_time], dtype=np.float64))))

            a_traj_nom = sample_trajectory_xy(a_xy0, a_h0, a_ht, a_nom_speed_mps, a_turn_rate_deg_sec, traj_times_s)
            b_traj_nom = sample_trajectory_xy(b_xy0, b_h0, b_ht, b_nom_speed_mps, b_turn_rate_deg_sec, traj_times_s)
            a_traj_min = sample_trajectory_xy(a_xy0, a_h0, a_ht, a_min_speed_mps, a_turn_rate_deg_sec, traj_times_s)
            a_traj_max = sample_trajectory_xy(a_xy0, a_h0, a_ht, a_max_speed_mps, a_turn_rate_deg_sec, traj_times_s)
            b_traj_min = sample_trajectory_xy(b_xy0, b_h0, b_ht, b_min_speed_mps, b_turn_rate_deg_sec, traj_times_s)
            b_traj_max = sample_trajectory_xy(b_xy0, b_h0, b_ht, b_max_speed_mps, b_turn_rate_deg_sec, traj_times_s)

            curve_sample_count = min(601, max(241, int(proj_s / 2.0) + 1))
            curve_times_s = np.linspace(0.0, proj_s, curve_sample_count)
            if 0.0 < closest_time < proj_s:
                curve_times_s = np.unique(np.concatenate((curve_times_s, np.asarray([closest_time], dtype=np.float64))))

            envelope_distances_m = envelope_distance_curve(
                rel_pos0=rel_pos0,
                a_heading0_deg=a_h0,
                a_target_heading_deg=a_ht,
                a_turn_rate_deg_sec=a_turn_rate_deg_sec,
                a_min_speed_mps=a_min_speed_mps,
                a_max_speed_mps=a_max_speed_mps,
                b_heading0_deg=b_h0,
                b_target_heading_deg=b_ht,
                b_turn_rate_deg_sec=b_turn_rate_deg_sec,
                b_min_speed_mps=b_min_speed_mps,
                b_max_speed_mps=b_max_speed_mps,
                times_s=curve_times_s,
            )
            a_curve_nom = sample_trajectory_xy(a_xy0, a_h0, a_ht, a_nom_speed_mps, a_turn_rate_deg_sec, curve_times_s)
            b_curve_nom = sample_trajectory_xy(b_xy0, b_h0, b_ht, b_nom_speed_mps, b_turn_rate_deg_sec, curve_times_s)
            nominal_distances_m = np.linalg.norm(a_curve_nom - b_curve_nom, axis=1)
            time_focus_mask = curve_times_s <= time_focus_end_s + 1e-9
            curve_times_focus_s = curve_times_s[time_focus_mask]
            envelope_distances_focus_nmi = envelope_distances_m[time_focus_mask] / NMI
            nominal_distances_focus_nmi = nominal_distances_m[time_focus_mask] / NMI
            threshold_nmi = sep_m / NMI

            a_pos_min, a_pos_nom, a_pos_max = speed_segment_at_time(
                a_xy0, a_h0, a_ht, a_turn_rate_deg_sec, a_nom_speed_mps, a_min_speed_mps, a_max_speed_mps, closest_time
            )
            b_pos_min, b_pos_nom, b_pos_max = speed_segment_at_time(
                b_xy0, b_h0, b_ht, b_turn_rate_deg_sec, b_nom_speed_mps, b_min_speed_mps, b_max_speed_mps, closest_time
            )
            rel_hull = _relative_position_hull_at_time(
                rel_pos0=rel_pos0,
                a_heading0_deg=a_h0,
                a_target_heading_deg=a_ht,
                a_turn_rate_deg_sec=a_turn_rate_deg_sec,
                a_min_speed_mps=a_min_speed_mps,
                a_max_speed_mps=a_max_speed_mps,
                b_heading0_deg=b_h0,
                b_target_heading_deg=b_ht,
                b_turn_rate_deg_sec=b_turn_rate_deg_sec,
                b_min_speed_mps=b_min_speed_mps,
                b_max_speed_mps=b_max_speed_mps,
                t_s=closest_time,
            )
            origin = np.zeros(2, dtype=np.float64)
            rel_nom = a_pos_nom - b_pos_nom  # pyright: ignore[reportOperatorIssue]
            rel_closest_point, _ = _closest_point_on_convex_polygon(origin, rel_hull)
            a_speed_span_t_m = float(np.linalg.norm(a_pos_max - a_pos_min))
            b_speed_span_t_m = float(np.linalg.norm(b_pos_max - b_pos_min))
            a_speed_span_end_m = float(np.linalg.norm(a_traj_max[-1] - a_traj_min[-1]))
            b_speed_span_end_m = float(np.linalg.norm(b_traj_max[-1] - b_traj_min[-1]))
            a_corridor_xy = corridor_polygon(a_traj_min, a_traj_max)
            b_corridor_xy = corridor_polygon(b_traj_min, b_traj_max)
            inset_half_window_s = min(20.0, max(12.0, 0.18 * spatial_focus_end_s))
            inset_start_s = max(0.0, closest_time - inset_half_window_s)
            inset_end_s = min(spatial_focus_end_s, closest_time + inset_half_window_s)
            inset_mask = (traj_times_s >= inset_start_s - 1e-9) & (traj_times_s <= inset_end_s + 1e-9)
            a_traj_nom_inset = a_traj_nom[inset_mask]
            b_traj_nom_inset = b_traj_nom[inset_mask]
            a_traj_min_inset = a_traj_min[inset_mask]
            a_traj_max_inset = a_traj_max[inset_mask]
            b_traj_min_inset = b_traj_min[inset_mask]
            b_traj_max_inset = b_traj_max[inset_mask]
            a_corridor_inset = corridor_polygon(a_traj_min_inset, a_traj_max_inset)
            b_corridor_inset = corridor_polygon(b_traj_min_inset, b_traj_max_inset)

            a_nom_turn_end_xy = None
            if 0.0 < a_turn_end_s <= spatial_focus_end_s:
                a_nom_turn_end_xy = sample_trajectory_xy(
                    a_xy0,
                    a_h0,
                    a_ht,
                    a_nom_speed_mps,
                    a_turn_rate_deg_sec,
                    np.asarray([a_turn_end_s], dtype=np.float64),
                )[0]
            b_nom_turn_end_xy = None
            if 0.0 < b_turn_end_s <= spatial_focus_end_s:
                b_nom_turn_end_xy = sample_trajectory_xy(
                    b_xy0,
                    b_h0,
                    b_ht,
                    b_nom_speed_mps,
                    b_turn_rate_deg_sec,
                    np.asarray([b_turn_end_s], dtype=np.float64),
                )[0]

            # Plot
            fig = plt.figure(figsize=(18, 8.4))
            grid = fig.add_gridspec(
                2,
                3,
                height_ratios=[1.0, 0.42],
                width_ratios=[1.2, 0.9, 1.15],
                hspace=0.28,
                wspace=0.3,
            )
            ax_xy = fig.add_subplot(grid[0, 0])
            ax_xy_local = fig.add_subplot(grid[1, 0])
            ax_rel = fig.add_subplot(grid[:, 1])
            ax_dist = fig.add_subplot(grid[:, 2])
            result_str = "SEPARATED" if is_sep else "CONFLICT"
            result_color = COLOR_SAFE if is_sep else COLOR_CONFLICT
            nominal_sep_nmi = float(np.linalg.norm(rel_nom)) / NMI
            fig.suptitle(
                f"{title} | {result_str} | "
                f"reported envelope distance {min_dist / NMI:.2f} NMI at t*={closest_time:.0f}s",
                fontsize=14,
                y=0.98,
            )

            # Spatial view: nominal trajectories plus min/max speed envelopes and the
            # speed-reachable segments at the reported critical time `t*`.
            ax_xy.fill(
                a_corridor_xy[:, 0],
                a_corridor_xy[:, 1],
                facecolor=COLOR_A,
                edgecolor="none",
                alpha=0.12,
                zorder=1,
                label="A speed corridor",
            )
            ax_xy.fill(
                b_corridor_xy[:, 0],
                b_corridor_xy[:, 1],
                facecolor=COLOR_B,
                edgecolor="none",
                alpha=0.12,
                zorder=1,
                label="B speed corridor",
            )
            ax_xy.plot(a_traj_nom[:, 0], a_traj_nom[:, 1], color=COLOR_A, lw=2.6, label="A nominal")
            ax_xy.plot(b_traj_nom[:, 0], b_traj_nom[:, 1], color=COLOR_B, lw=2.6, label="B nominal")
            ax_xy.plot(
                a_traj_min[:, 0],
                a_traj_min[:, 1],
                color=COLOR_A,
                lw=1.3,
                ls=":",
                alpha=0.85,
                zorder=2,
                label="_nolegend_",
            )
            ax_xy.plot(
                a_traj_max[:, 0],
                a_traj_max[:, 1],
                color=COLOR_A,
                lw=1.3,
                ls=":",
                alpha=0.85,
                zorder=2,
                label="_nolegend_",
            )
            ax_xy.plot(
                b_traj_min[:, 0],
                b_traj_min[:, 1],
                color=COLOR_B,
                lw=1.3,
                ls=":",
                alpha=0.85,
                zorder=2,
                label="_nolegend_",
            )
            ax_xy.plot(
                b_traj_max[:, 0],
                b_traj_max[:, 1],
                color=COLOR_B,
                lw=1.3,
                ls=":",
                alpha=0.85,
                zorder=2,
                label="_nolegend_",
            )

            # Start positions
            ax_xy.scatter([a_xy0[0]], [a_xy0[1]], c=COLOR_A, s=88, zorder=5, edgecolors="white", linewidths=1.5)
            ax_xy.scatter([b_xy0[0]], [b_xy0[1]], c=COLOR_B, s=88, zorder=5, edgecolors="white", linewidths=1.5)
            ax_xy.annotate("A", (a_xy0[0], a_xy0[1]), xytext=(8, 8), textcoords="offset points", color=COLOR_A)
            ax_xy.annotate("B", (b_xy0[0], b_xy0[1]), xytext=(8, 8), textcoords="offset points", color=COLOR_B)

            # Heading arrows: solid = initial heading, dashed = target heading
            spatial_focus_points = np.vstack(
                (
                    a_traj_nom,
                    b_traj_nom,
                    a_traj_min,
                    a_traj_max,
                    b_traj_min,
                    b_traj_max,
                    a_pos_min[None, :],
                    a_pos_max[None, :],
                    b_pos_min[None, :],
                    b_pos_max[None, :],
                )
            )
            spatial_half_span_m = set_demo_square_limits(
                ax_xy,
                spatial_focus_points,
                pad_m=max(0.35 * sep_m, 1500.0),
            )
            arrow_len = max(1800.0, 0.15 * spatial_half_span_m)
            draw_demo_heading_arrow(ax_xy, a_xy0, a_h0, arrow_len, COLOR_A)
            draw_demo_heading_arrow(ax_xy, b_xy0, b_h0, arrow_len, COLOR_B)
            if a_h0 != a_ht:
                draw_demo_heading_arrow(ax_xy, a_xy0, a_ht, arrow_len * 0.72, COLOR_A, style="--")
            if b_h0 != b_ht:
                draw_demo_heading_arrow(ax_xy, b_xy0, b_ht, arrow_len * 0.72, COLOR_B, style="--")

            if a_nom_turn_end_xy is not None:
                ax_xy.scatter(
                    [a_nom_turn_end_xy[0]],
                    [a_nom_turn_end_xy[1]],
                    marker="^",
                    s=64,
                    color=COLOR_A,
                    zorder=6,
                    label="A turn end",
                )
            if b_nom_turn_end_xy is not None:
                ax_xy.scatter(
                    [b_nom_turn_end_xy[0]],
                    [b_nom_turn_end_xy[1]],
                    marker="^",
                    s=64,
                    color=COLOR_B,
                    zorder=6,
                    label="B turn end",
                )

            # Highlight the reachable speed segment for each aircraft at the reported
            # critical time `t*`. This shows the 1-D position uncertainty induced by
            # speed.
            ax_xy.plot(
                [a_pos_min[0], a_pos_max[0]],
                [a_pos_min[1], a_pos_max[1]],
                color=COLOR_A,
                lw=5,
                alpha=0.24,
                solid_capstyle="round",
                label="A reachable @ t*",
            )
            ax_xy.plot(
                [b_pos_min[0], b_pos_max[0]],
                [b_pos_min[1], b_pos_max[1]],
                color=COLOR_B,
                lw=5,
                alpha=0.24,
                solid_capstyle="round",
                label="B reachable @ t*",
            )
            ax_xy.scatter([a_pos_nom[0]], [a_pos_nom[1]], color=COLOR_A, s=40, zorder=6)
            ax_xy.scatter([b_pos_nom[0]], [b_pos_nom[1]], color=COLOR_B, s=40, zorder=6)
            ax_xy.plot(
                [a_pos_nom[0], b_pos_nom[0]],
                [a_pos_nom[1], b_pos_nom[1]],
                color="0.5",
                lw=1.1,
                ls="--",
                alpha=0.9,
                label="Nominal sep @ t*",
            )
            ax_xy.annotate(
                f"t*={closest_time:.0f}s",
                (0.5 * (a_pos_nom[0] + b_pos_nom[0]), 0.5 * (a_pos_nom[1] + b_pos_nom[1])),
                xytext=(10, -14),
                textcoords="offset points",
                fontsize=8,
                color="0.3",
            )

            ax_xy.text(
                0.02,
                0.98,
                f"A: {heading_summary(a_h0, a_ht, a_turn_rate_deg_sec, a_spd)}\n"
                f"B: {heading_summary(b_h0, b_ht, b_turn_rate_deg_sec, b_spd)}\n"
                f"Speed envelope: +/-{speed_diff:.0f} kt\n"
                f"A span @ t*: {a_speed_span_t_m:.0f} m, shown end: {a_speed_span_end_m:.0f} m\n"
                f"B span @ t*: {b_speed_span_t_m:.0f} m, shown end: {b_speed_span_end_m:.0f} m\n"
                f"Nominal sep @ t*: {nominal_sep_nmi:.2f} NMI\n"
                f"Spatial view: 0-{spatial_focus_end_s:.0f}s\n"
                f"Model horizon: 0-{proj_s:.0f}s",
                transform=ax_xy.transAxes,
                verticalalignment="top",
                fontsize=9.5,
                bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.9, "edgecolor": "0.7"},
            )

            # Dedicated local-view panel around the reported critical time. The top
            # spatial plot keeps the large-scale context; this lower panel makes the
            # small speed-induced spread readable without covering the main geometry.
            ax_xy_local.fill(
                a_corridor_inset[:, 0],
                a_corridor_inset[:, 1],
                facecolor=COLOR_A,
                edgecolor="none",
                alpha=0.12,
            )
            ax_xy_local.fill(
                b_corridor_inset[:, 0],
                b_corridor_inset[:, 1],
                facecolor=COLOR_B,
                edgecolor="none",
                alpha=0.12,
            )
            ax_xy_local.plot(a_traj_nom_inset[:, 0], a_traj_nom_inset[:, 1], color=COLOR_A, lw=2.2)
            ax_xy_local.plot(b_traj_nom_inset[:, 0], b_traj_nom_inset[:, 1], color=COLOR_B, lw=2.2)
            ax_xy_local.plot(a_traj_min_inset[:, 0], a_traj_min_inset[:, 1], color=COLOR_A, lw=1.0, ls=":", alpha=0.8)
            ax_xy_local.plot(a_traj_max_inset[:, 0], a_traj_max_inset[:, 1], color=COLOR_A, lw=1.0, ls=":", alpha=0.8)
            ax_xy_local.plot(b_traj_min_inset[:, 0], b_traj_min_inset[:, 1], color=COLOR_B, lw=1.0, ls=":", alpha=0.8)
            ax_xy_local.plot(b_traj_max_inset[:, 0], b_traj_max_inset[:, 1], color=COLOR_B, lw=1.0, ls=":", alpha=0.8)
            ax_xy_local.plot(
                [a_pos_min[0], a_pos_max[0]],
                [a_pos_min[1], a_pos_max[1]],
                color=COLOR_A,
                lw=4.0,
                alpha=0.32,
                solid_capstyle="round",
            )
            ax_xy_local.plot(
                [b_pos_min[0], b_pos_max[0]],
                [b_pos_min[1], b_pos_max[1]],
                color=COLOR_B,
                lw=4.0,
                alpha=0.32,
                solid_capstyle="round",
            )
            ax_xy_local.scatter([a_pos_nom[0]], [a_pos_nom[1]], color=COLOR_A, s=28, zorder=5)
            ax_xy_local.scatter([b_pos_nom[0]], [b_pos_nom[1]], color=COLOR_B, s=28, zorder=5)
            ax_xy_local.plot(
                [a_pos_nom[0], b_pos_nom[0]],
                [a_pos_nom[1], b_pos_nom[1]],
                color="0.45",
                lw=1.0,
                ls="--",
                alpha=0.85,
            )
            inset_points_xy = np.vstack(
                (
                    a_traj_nom_inset,
                    b_traj_nom_inset,
                    a_traj_min_inset,
                    a_traj_max_inset,
                    b_traj_min_inset,
                    b_traj_max_inset,
                    a_pos_min[None, :],
                    a_pos_max[None, :],
                    b_pos_min[None, :],
                    b_pos_max[None, :],
                )
            )
            set_demo_square_limits(
                ax_xy_local,
                inset_points_xy,
                pad_m=max(250.0, 0.3 * max(a_speed_span_end_m, b_speed_span_end_m)),
            )
            ax_xy_local.set_title(f"Local view [{inset_start_s:.0f}, {inset_end_s:.0f}] s", fontsize=10)
            ax_xy_local.set_xlabel("East (m)")
            ax_xy_local.set_ylabel("North (m)")
            ax_xy_local.set_aspect("equal", "box")
            ax_xy_local.grid(True, alpha=0.2)
            ax_xy_local.tick_params(labelsize=8)
            for spine in ax_xy_local.spines.values():
                spine.set_edgecolor("0.5")
                spine.set_linewidth(1.0)

            # Relative-hull view: at the reported critical time `t*`, the reachable
            # relative positions form a small convex set. Conflict means this set
            # touches the threshold disk around the origin.
            ax_rel.add_patch(
                Circle((0.0, 0.0), radius=sep_m, facecolor="none", edgecolor=COLOR_CONFLICT, ls=":", lw=1.4)
            )
            if rel_hull.shape[0] >= 3:
                ax_rel.add_patch(
                    Polygon(rel_hull, closed=True, facecolor=result_color, edgecolor=result_color, alpha=0.18, lw=2.0)
                )
                hull_closed = np.vstack((rel_hull, rel_hull[0]))
                ax_rel.plot(
                    hull_closed[:, 0], hull_closed[:, 1], color=result_color, lw=2.0, label="Reachable hull @ t*"
                )
            elif rel_hull.shape[0] == 2:
                ax_rel.plot(rel_hull[:, 0], rel_hull[:, 1], color=result_color, lw=3.0, label="Reachable hull @ t*")
            else:
                ax_rel.scatter(rel_hull[:, 0], rel_hull[:, 1], color=result_color, s=48, label="Reachable hull @ t*")
            ax_rel.scatter([0.0], [0.0], color="0.2", s=40, zorder=5, label="Protected centre")
            ax_rel.scatter([rel_nom[0]], [rel_nom[1]], color="0.2", s=42, zorder=6, label="Nominal relative @ t*")
            ax_rel.plot(
                [0.0, rel_closest_point[0]],
                [0.0, rel_closest_point[1]],
                color="0.35",
                lw=1.2,
                ls="--",
            )
            ax_rel.scatter([rel_closest_point[0]], [rel_closest_point[1]], color=result_color, s=48, zorder=6)
            rel_extent_m = max(
                1.25 * sep_m,
                float(np.max(np.abs(rel_hull))) if rel_hull.size > 0 else 0.0,
                float(np.max(np.abs(rel_nom))),
            )
            rel_extent_m = rel_extent_m * 1.15 + 500.0
            ax_rel.set_xlim(-rel_extent_m, rel_extent_m)
            ax_rel.set_ylim(-rel_extent_m, rel_extent_m)
            ax_rel.set_aspect("equal", "box")
            ax_rel.set_title("Relative hull at t*")
            ax_rel.set_xlabel("Relative east (m)")
            ax_rel.set_ylabel("Relative north (m)")
            ax_rel.grid(True, alpha=0.25)
            ax_rel.legend(
                loc="upper center",
                bbox_to_anchor=(0.5, -0.12),
                fontsize=9,
            )

            # Time view: show the sampled exact envelope distance against the nominal
            # trajectory distance so the certification result is easier to interpret.
            ax_dist.plot(
                curve_times_focus_s,
                envelope_distances_focus_nmi,
                color=result_color,
                lw=2.6,
                label="Envelope d(t)",
            )
            ax_dist.plot(
                curve_times_focus_s,
                nominal_distances_focus_nmi,
                color="0.25",
                lw=1.6,
                ls="--",
                label="Nominal speed",
            )
            ax_dist.axhline(threshold_nmi, color=COLOR_CONFLICT, lw=1.2, ls=":", label="Threshold")
            ax_dist.axvline(closest_time, color="0.4", lw=1.0, ls="--", alpha=0.8)
            ax_dist.scatter([closest_time], [min_dist / NMI], color=result_color, zorder=5, s=42)
            conflict_mask = [bool(v) for v in envelope_distances_focus_nmi < threshold_nmi]
            ax_dist.fill_between(
                curve_times_focus_s,
                envelope_distances_focus_nmi,
                threshold_nmi,
                where=conflict_mask,
                color=COLOR_CONFLICT,
                alpha=0.14,
                interpolate=True,
            )
            if 0.0 < a_turn_end_s <= time_focus_end_s:
                ax_dist.axvline(a_turn_end_s, color=COLOR_A, lw=1.0, ls=":", alpha=0.45)
            if 0.0 < b_turn_end_s <= time_focus_end_s:
                ax_dist.axvline(b_turn_end_s, color=COLOR_B, lw=1.0, ls=":", alpha=0.45)

            ax_xy.set_xlabel("East (m)")
            ax_xy.set_ylabel("North (m)")
            ax_xy.set_title("Trajectories (zoomed)")
            ax_xy.set_aspect("equal", "box")
            ax_xy.grid(True, alpha=0.25)

            ax_dist.set_title("Distance vs time")
            ax_dist.set_xlabel("Time (s)")
            ax_dist.set_ylabel("Distance (NMI)")
            ax_dist.set_xlim(0.0, time_focus_end_s)
            y_max_nmi = max(
                float(np.max(envelope_distances_focus_nmi)),
                float(np.max(nominal_distances_focus_nmi)),
                threshold_nmi,
            )
            y_min_nmi = min(
                float(np.min(envelope_distances_focus_nmi)),
                float(np.min(nominal_distances_focus_nmi)),
                threshold_nmi,
            )
            y_pad_nmi = max(0.25, 0.06 * max(y_max_nmi - y_min_nmi, 1.0))
            ax_dist.set_ylim(max(0.0, y_min_nmi - y_pad_nmi), y_max_nmi + y_pad_nmi)
            ax_dist.grid(True, alpha=0.25)
            ax_dist.legend(
                loc="upper center",
                bbox_to_anchor=(0.5, -0.12),
                ncol=3,
                fontsize=9,
            )
            ax_xy_local.legend(
                *ax_xy.get_legend_handles_labels(),
                loc="upper center",
                bbox_to_anchor=(0.5, -0.28),
                ncol=2,
                fontsize=9,
            )
            fig.subplots_adjust(top=0.9, bottom=0.18, left=0.055, right=0.985)
            plt.show()
