# Relevant Aircraft Method Note

This note is the code-facing summary of the method implemented in
`geometric_safety/relevant_aircraft.py`.

It is deliberately short. It records:

1. the bounded motion model the code is solving
2. what is exact and what is conservative
3. the key implementation structure
4. the safety guarantees and the main limitations.

The longer research-oriented note is intentionally kept separate from this file.

## 1. Model scope

The method solves a specific horizontal pairwise separation problem in a local
east/north tangent plane.

Assumptions:

- horizontal 2-D motion only
- finite projection horizon `T`
- each aircraft speed lies in a closed interval
- for any one realization, each aircraft chooses one constant speed from that interval
  and keeps it for the full horizon
- no wind, no vertical motion, no acceleration model
- straight solver: each aircraft holds one fixed heading
- turn-aware solver: each aircraft may execute at most one fixed-rate turn toward a
  target heading, then continue straight.

This note is about the implemented model, not about general aircraft motion.

## 2. Decision problem

Let the current aircraft positions be `p_A(0)` and `p_B(0)`, and let `D` be the
protected separation distance.

The decision question is:

- over all admissible motions in the model, can the aircraft come within distance `D`
  during the horizon `[0, T]`?

Equivalently:

- `safe <=> d_* >= D`

where `d_*` is the minimum pairwise separation over time and all admissible speeds.

The implementation does not propagate one nominal future. It constructs the reachable
envelope induced by bounded speeds and checks that envelope against the protected region.

## 3. Straight-heading method

If both aircraft maintain fixed headings, relative motion is linear:

- `r(t) = r_0 + t (s_A u_A - s_B u_B)`.

The full reachable relative-displacement set over `[0, T]` is one compact convex hull:

- `S(T) = conv({0} union {T v_i})`

where `v_i` are the four relative-velocity corners induced by the speed bounds.

The worst-case separation problem is then exactly:

- `d_* = dist(-r_0, S(T))`.

So the straight solver is exact for the stated model:

- exact in speed
- exact in time
- exact safe/unsafe decision
- exact reported minimum distance and closest time.

One useful bonus is that bounded time-varying speed is effectively handled "for free" in
the straight case. With fixed heading, only total path length matters, so the same
reachable set is recovered from the speed bounds alone.

## 4. Turn-aware method

If one or both aircraft may execute one fixed-rate turn, the code uses a different
decomposition.

At any fixed time `t`, each aircraft position still has the affine form:

- `p(t; s) = p(0) + s b(t)`

because the heading law is deterministic and the chosen speed is constant. Therefore the
relative-position set at fixed time is the affine image of the speed rectangle, so it is
always a point, a segment, or a parallelogram determined by four speed corners.

At fixed time, the method is exact:

- it computes the minimum distance from the origin to that fixed-time reachable hull.

Over continuous time, the method is conservative:

- it does not claim a closed-form exact minimum over the full horizon
- instead it certifies time intervals safe using exact endpoint distances and a valid
  Lipschitz bound
- if an interval cannot be certified and becomes too small, the method reports unsafe.

So the turn-aware method is:

- exact in bounded speed at sampled times
- certified conservative in continuous time.

## 5. Lipschitz certification

The turn-aware certification step uses the fact that if `d(t)` is Lipschitz on an
interval `[t_0, t_1]`, then endpoint distances imply a lower bound throughout that
interval.

The implementation supports two Lipschitz bounds:

- a simple global bound from the sum of speed maxima
- a tighter interval-local corner-speed bound.

The interval-local bound is the default. It is still safe, but usually less
conservative, because it uses the actual heading geometry on the interval rather than
assuming worst-case closing everywhere.

## 6. Structured geometry in the implementation

The hot-path geometry in this module is tiny:

- four-point hulls from speed corners
- five-point hulls when the origin is added in the straight case.

The code therefore has two implementation choices that are worth keeping in mind:

1. a shared speed-corner kernel is used by both the straight and turn-aware paths
2. a small structured hull builder is used for these tiny cases, while the generic hull
   routine is retained as a reference implementation and test oracle.

This keeps the code small and improves runtime without changing the mathematics.

## 7. Safety interpretation

The two public safety checks have different contracts.

`catch_up_projection_interval`

- exact for the stated straight-heading model
- `safe = True` means no admissible realization violates separation
- `safe = False` means some admissible realization violates or touches the threshold.

`catch_up_projection_interval_with_turns`

- safe only when the whole horizon has been certified safe
- `is_separated = True` is the key guarantee
- `is_separated = False` means either a feasible loss of separation was found or the
  interval search could not certify safety within the configured minimum width
- the reported closest time and minimum distance are diagnostic summaries of sampled and
  certified quantities, not a claim of an exact continuous-time minimizer.

## 8. Optional robustness buffer

The turn-aware method assumes constant chosen speed during a turn. That assumption is
structurally necessary for the current four-corner fixed-time hull.

The implementation therefore also supports an optional model-mismatch buffer:

- `turn_speed_schedule_uncertainty_kt`

This is off by default. When positive, it inflates the turn-aware separation threshold by
a conservative additive buffer derived from bounded in-turn speed variation. This does
not change the reachable-set geometry or the certification logic; it simply adds
robustness against a broader speed model at the cost of extra conservatism.

This buffer is only relevant to the turn-aware path. The straight path already absorbs
bounded time-varying speed through the geometry described above.

## 9. Main limitations

The method does not cover:

- multiple turns per aircraft
- acceleration models
- wind
- vertical motion
- long-range Earth-curvature effects
- exact guarantees outside the stated bounded model.

Those limitations are deliberate. The geometry is intentionally small enough to be
provable against the model and cheap enough to run in a hot filtering or optimization
loop.
