# Relevant Aircraft Projection Design Note

This note records the mathematical model and implementation choices behind
`geometric_safety/relevant_aircraft.py`. It is intended as a design document now
and as a starting point for later paper-writing. The emphasis is on what is being
computed, why the geometry is valid, where conservatism enters, and what guarantees the
returned safety decision does and does not provide.

It has six jobs:

1. state the model precisely enough that the safety claim is meaningful
2. explain why the straight-heading method is exact
3. explain why the turn-aware method is exact in speed but conservative in time
4. record the relevant prior work and the right claim boundaries
5. describe the robustness extension and the deliberate limitations of the model
6. record the remaining proof work and a plausible paper structure.

## 1. Model scope and problem statement

### 1.1 Scope and assumptions

This note describes a specific bounded kinematic model. The method does not solve
"general aircraft motion"; it solves the following narrower problem.

- horizontal 2-D motion only, in a local east/north tangent plane
- finite projection horizon `T`
- each aircraft speed lies in a closed interval
- for any one realization, each aircraft chooses one constant speed from that interval
  and keeps it for the full horizon
- the speed interval represents bounded uncertainty in constant speed, not acceleration
  or arbitrary time-varying speed
- no wind, no vertical motion, no stochastic disturbances
- no longitudinal acceleration model
- straight-heading solver: each aircraft holds one fixed heading for the full horizon
- turn-aware solver: each aircraft may execute at most one fixed-rate turn toward a
  target heading; once the target heading is reached, the aircraft continues straight;
  the turn rate is treated as constant while the turn is active
- local tangent-plane geometry is assumed to be accurate enough over the interaction
  scale of interest

Two points matter especially for the later mathematics:

1. The constant-speed assumption is what makes both reachable sets affine in speed.
   Without it, the core convex-hull reductions in this note would no longer hold in
   their current form.
2. The turn-aware method is exact in the bounded speed envelope at each fixed time, but
   conservative in continuous time because it certifies intervals instead of solving the
   full time-dependent minimum in closed form.

There is one useful nuance here:

- in the straight-heading case, bounded time-varying speed is effectively handled "for
  free" because position depends only on total distance travelled along a fixed
  direction
- in the turn-aware case, that is no longer true, because speeding up early versus late
  during a turn changes the spatial endpoint

So constant speed is not just a convenient simplification for the turning method. It is
structurally necessary for the current four-corner fixed-time reachable hull.

### 1.2 Problem statement

Work in a local east/north tangent plane. Let the current aircraft positions be

- `p_A(0), p_B(0) in R^2`

and define the initial relative position

- `r_0 = p_A(0) - p_B(0)`.

Each aircraft has a bounded speed interval

- `s_A in [s_A^-, s_A^+]`
- `s_B in [s_B^-, s_B^+]`

and a finite horizon `T`.

The core decision problem is:

- over all admissible motions in the model, can the aircraft come within a protected
  separation distance `D`?

Equivalently, define

- `d_* = min ||p_A(t) - p_B(t)||`
- where the minimum is over `t in [0, T]` and all admissible speeds.

Then

- `safe <=> d_* >= D`.

The central modelling choice in this module is not to propagate one nominal future.
Instead, it constructs the full admissible envelope induced by bounded speeds and then
tests that envelope against the protected region.

### 1.3 Reading guide

The rest of the note is structured as follows:

1. Sections 2 and 3 describe the two geometric solvers: exact straight-heading
   projection and exact fixed-time turn-aware hulls.
2. Sections 4 and 5 explain the certification layer that turns the fixed-time turn
   geometry into a finite-horizon safety decision.
3. Sections 6 through 8 cover implementation structure, guarantees, and robustness to
   in-turn speed variation.
4. Sections 9 and 10 are forward-looking and focus on proof obligations and paper
   structure.

### 1.4 Related work and positioning

This note should not pretend that the general area is empty. There is substantial prior
work on aircraft conflict detection under uncertainty, on maneuver-aware prediction, and
on reachable-set or convex-envelope methods. What matters for later paper-writing is to
record which parts of that literature are genuinely close to the present method, and
which gap this implementation is actually trying to fill.

The short version is:

- this is not the first uncertainty-aware aircraft conflict detection method
- this is not the first turn-aware method
- this is not the first reachable-set or convex-hull method
- the plausible contribution is the particular combination of exact straight-heading
  separation checking under interval-bounded speed uncertainty, exact fixed-time
  reachable hulls for one fixed-rate turn per aircraft, a certified conservative search
  in time, a very small structured geometry implementation that is cheap enough for a
  hot pairwise filter, and an explicit robustness extension for in-turn speed
  variation.

That distinction matters. If this work is written up, it should not be sold as a paper
whose value depends on one ingredient being entirely new. The stronger and more
defensible position is that it isolates a narrow model class that is operationally
plausible, then gives a pairwise safety primitive for that class with unusually clean
guarantees, small geometry, and measured hot-path cost. The integration is the point.

The references below are the current working set. They are not yet a final bibliography,
but they are the papers and reports most likely to matter when positioning the work.

1. Heinz Erzberger, *Transforming the NAS: The Next Generation Air Traffic Control
   System*, NASA TP-2004-212828, 2004.
   Link: <https://ntrs.nasa.gov/api/citations/20050110294/downloads/20050110294.pdf>
   Relevance:
   - clear prior art for turn-aware conflict detection in ATM
   - explicitly discusses conflict detection in a turn using multi-trajectory analysis.
   Why it is not the same:
   - it samples or enumerates maneuver hypotheses rather than building the exact
     fixed-time speed hull used here
   - it does not provide the same certified exact-in-speed / conservative-in-time
     geometric decision structure.

2. Anthony Narkawicz and Cesar Munoz, *A Mathematical Analysis of Conflict Prevention*,
   1. Link: <https://ntrs.nasa.gov/api/citations/20090034971/downloads/20090034971.pdf>
   Relevance:
   - strong prior art for exact geometric analysis of pairwise aircraft separation in the
     straight-line setting
   - important evidence that ATM papers can benefit from formal mathematical structure.
   Why it is not the same:
   - the analysis is for linear-motion conflict prevention bands, not the current
     turn-aware reachable-set construction
   - it does not handle one fixed-rate turn per aircraft in the way this module does.

3. Karim Mechqrane, Jean-Philippe Clarke, and Nicolas Beldiceanu, *A CP approach to
   aircraft conflict avoidance*, 2012.
   Link: <https://www.m-hikari.com/ams/ams-2012/ams-45-48-2012/mechqraneAMS45-48-2012.pdf>
   Relevance:
   - this is the closest geometric relative I found
   - under speed uncertainty, the paper represents possible aircraft positions by line
     segments, and after a heading change by parallelograms, then tests polygon
     distances.
   Why it is not the same:
   - time is discretized
   - heading changes are modeled as changes of direction rather than the current
     fixed-rate turn with current heading, target heading, and turn rate
   - the emphasis is conflict-avoidance optimization, not a certified pairwise safety
     filter.

4. Alfonso Valenzuela and Damian Rivas, and related ATM work on convex hulls of possible
   future aircraft positions under maneuver uncertainty.
   One accessible source link used in this scan:
   <https://citeseerx.ist.psu.edu/document?doi=ceb6f2ca26dd4301ee14fdb227bbd5ce84b9186c&repid=rep1&type=pdf>
   Relevance:
   - clear prior art for the idea of representing uncertain future aircraft motion by
     convex envelopes and checking separation between those envelopes
   - close in spirit to the current use of small convex hulls.
   Why it is not the same:
   - the focus is more on maneuver generation, benchmark construction, or discretized
     uncertainty envelopes than on the present continuous-time certified pairwise test
   - the current method makes the fixed-rate-turn geometry and the conservative-in-time
     certification explicit.

5. Yucong Lin and Srikanth Saripalli, *Collision avoidance for UAVs using reachable
   sets*, 2015.
   Link:
   <https://asu.elsevierpure.com/en/publications/collision-avoidance-for-uavs-using-reachable-sets>
   Relevance:
   - prior art for reachable-set methods with aircraft-like turning kinematics
   - useful evidence that limited-turn-rate motion is a natural reachable-set model.
   Why it is not the same:
   - the problem is UAV path planning and collision avoidance, not ATM-style pairwise
     separation certification
   - the method is not built around interval-bounded constant-speed uncertainty with the
     small four-corner hull used here.

6. Xin Yang et al., *Multi-aircraft Conflict Detection and Resolution Based on
   Probabilistic Reach Sets*, 2017.
   Link:
   <https://pure.bit.edu.cn/en/publications/multi-aircraft-conflict-detection-and-resolution-based-on-probabi>
   Relevance:
   - direct prior art for reach-set methods in aircraft conflict detection and
     resolution under uncertainty.
   Why it is not the same:
   - the construction is probabilistic rather than deterministic
   - the reachable sets are not the small exact fixed-time affine hulls used here
   - the safety interpretation is therefore different from the present certified
     worst-case filter.

7. Lei Hao et al., *Probabilistic multi-aircraft conflict detection approach for
   trajectory-based operation*, Transportation Research Part C, 2018.
   Link:
   <https://www.sciencedirect.com/science/article/abs/pii/S0968090X18302560>
   Relevance:
   - direct prior art for uncertainty-aware conflict detection in trajectory-based ATM
   - useful reference point for how far the literature already goes in probabilistic
     trajectory uncertainty modelling.
   Why it is not the same:
   - the treatment is probabilistic and intent-centric
   - it is not an exact straight solver plus certified turn-aware geometric filter in
     the current sense.

The main paper-positioning point is therefore not that earlier work ignored
uncertainty, ignored turns, or ignored convex geometry. Those claims would be false and
easy to attack. The defensible claim is much narrower: this module implements a
pairwise horizontal separation certifier for a very specific bounded model, and does so
in a form that is:

- exact for the straight-heading model
- exact in bounded speed at fixed time for the one-turn model
- conservative, explicit, and cheaply checkable over continuous time
- structured for runtime use rather than mainly for offline optimization or broad
  probabilistic trajectory analysis.

That last point may matter more than it first appears. A recurring impression from the
literature is that a great deal of good academic work lives far from an industrial hot
path: the models are richer, the uncertainty is broader, or the objective is global
conflict resolution rather than a small pairwise safety predicate. The present method is
more limited, but the limitation is deliberate. It trades generality for a geometry that
is small enough to reason about, prove against the stated model, and run cheaply inside
a larger optimization or filtering loop.

A reasonable introduction-level positioning paragraph would therefore be something like
the following:

> Prior work already contains uncertainty-aware conflict detection, turn-aware
> prediction, and convex or reachable-set formulations of future aircraft motion.
> The contribution here is not a claim to have invented any one of those ingredients in
> isolation. Instead, the contribution is a deliberately narrow pairwise safety checker
> for a bounded horizontal model that combines an exact straight-heading reduction, exact
> fixed-time reachable hulls for one fixed-rate turn per aircraft, and a certified
> conservative search in time. The resulting method is mathematically explicit, admits a
> clean safety interpretation, and is small enough to be plausible as a runtime
> primitive inside a larger industrial optimization or filtering loop.

### 1.5 Claim boundaries

If this note becomes a paper, it should be explicit about what is and is not being
claimed. That is partly about honesty and partly about reviewer-proofing.

The paper should be comfortable claiming:

- an exact pairwise separation check for the stated straight-heading model
- an exact fixed-time reachable hull for the stated one-turn-then-straight model with
  constant chosen speeds
- a certified conservative continuous-time search procedure for that same one-turn model
- a compact runtime-oriented implementation with measurable hot-path cost
- an optional robustness extension for bounded in-turn speed variation.

The paper should not claim:

- the first uncertainty-aware aircraft conflict detection method
- the first turn-aware or reachable-set aircraft conflict method
- exactness for arbitrary time-varying speed, acceleration, wind, or multiple turns
- correctness outside the bounded model stated in Section 1.1
- that every reported diagnostic quantity in the turn-aware solver is exact continuous
  truth, rather than part of a conservative decision procedure.

In short, the contribution is not "general aircraft conflict detection." It is a small,
certifiable pairwise primitive for a deliberately narrow but operationally plausible
model class.

## 2. Straight-heading method

### 2.1 Kinematic model

If both aircraft maintain fixed headings, let `u_A` and `u_B` be the corresponding unit
heading vectors. Then

- `p_A(t) = p_A(0) + t s_A u_A`
- `p_B(t) = p_B(0) + t s_B u_B`

so the relative motion is

- `r(t) = r_0 + t (s_A u_A - s_B u_B)`.

This model is more robust than it first appears. If speed were allowed to vary
arbitrarily within the same bounds while heading stayed fixed, then

- `p(t) = p(0) + u integral_0^t s(xi) d xi`

so only total path length along the fixed direction matters. At time `t`, that path
length still lies in the interval `[s^- t, s^+ t]`. Therefore the same fixed-time
reachable set is recovered, and the straight-line method continues to represent the
bounded-speed envelope exactly.

### 2.2 Relative-velocity hull

The admissible relative velocities form the affine image of the speed rectangle

- `V = { s_A u_A - s_B u_B }`

with `s_A` and `s_B` ranging over their closed intervals. Since each speed interval is
one-dimensional, `V` is the convex hull of the four speed-corner combinations. Writing
those corner relative velocities as `v_i`, the full reachable relative-displacement set
over `[0, T]` is

- `S(T) = { t v : t in [0, T], v in V }`
- `S(T) = conv({0} union {T v_i})`.

This is the key simplification: the continuous family of straight-line trajectories
collapses into one compact convex polygon in displacement space.

### 2.3 Closest approach as one convex projection

The worst-case distance over the full horizon is

- `d_* = min_{d in S(T)} ||r_0 + d||`.

Equivalently, project `-r_0` onto `S(T)`:

- `d^* = argmin_{d in S(T)} ||d - (-r_0)||`.

Then

- `d_* = ||r_0 + d^*||`.

This is exact under the straight-line model. There is no time discretization and no
conservatism from the speed envelope itself. Once the closest point on the displacement
hull is known, the corresponding closest time can be recovered as the smallest
`alpha in [0, 1]` such that the projected displacement lies in `alpha S(T)`. In 2-D,
Caratheodory's theorem implies that this earliest representation uses the origin and at
most two outer hull vertices, which is exactly what the implementation searches for.

### 2.4 Safety guarantee

For the stated model assumptions, `catch_up_projection_interval` is exact:

- if it returns safe, no admissible constant-speed straight-heading realisation violates
  separation on `[0, T]`
- if it returns unsafe, some admissible realisation does violate separation or touches
  the threshold.

## 3. Turn-aware certified method

### 3.1 Kinematic model

The turn-aware path allows each aircraft to execute at most one fixed-rate turn toward a
target heading and then continue straight. For one aircraft:

- initial heading `h_0`
- target heading `h_T`
- constant turn rate `turn_rate_deg_sec`

with the turn terminating once the target heading is reached. If `Delta h` is the
signed heading change, the turn duration is

- `tau = |Delta h| / |turn_rate_deg_sec|` when `turn_rate_deg_sec != 0`.

The heading vector is

- `u(h) = (sin h, cos h)`.

For a fixed chosen speed `s`, position can be written as

- `p(t; s) = p(0) + s b(t)`

where

- `b(t) = integral_0^t u(h(xi)) d xi`.

The important structural fact is that `b(t)` depends only on the heading law, not on the
speed. The implementation computes this basis in closed form in
`_turn_displacement_basis`.

This is where the constant-speed assumption becomes essential. If speed were allowed to
vary with time during the turn, position would instead be

- `p(t) = p(0) + integral_0^t s(xi) u(h(xi)) d xi`

and the timing of the speed variation would matter because `u(h(xi))` changes direction
throughout the turn. Two speed schedules with the same minimum, maximum, and average can
end at different spatial positions if one is faster earlier in the turn and the other is
faster later. The affine form `p(t; s) = p(0) + s b(t)` would then fail, and with it the
four-corner reachable-hull construction used by the current implementation.

### 3.2 Fixed-time reachable hull

At any fixed time `t`, relative position is

- `R(t) = r_0 + { s_A b_A(t) - s_B b_B(t) }`

with the speeds ranging over their intervals. This is again the affine image of the
speed rectangle, so it is determined by the four speed-corner combinations. Therefore
`R(t)` is always a tiny convex set: a point, a segment, or a parallelogram.

Define the exact worst-case distance at time `t` by

- `d(t) = min_{r in R(t)} ||r||`.

This fixed-time problem is still exact and convex. The only new difficulty is that the
time-dependent minimum

- `d_* = min_{t in [0, T]} d(t)`

is no longer reducible to one global displacement cone.

### 3.3 Exact-in-speed, conservative-in-time

The implementation therefore separates the problem into two layers:

1. At a chosen time `t`, compute `d(t)` exactly by projecting the origin onto the
   fixed-time hull `R(t)`.
2. Over time, use interval certification rather than assuming a closed-form solution for
   the continuous-time minimum.

This is the central design choice for the turn-aware method:

- exact with respect to the bounded speed envelope
- conservative with respect to continuous time.

## 4. Interval certification by Lipschitz bounds

### 4.1 Why a Lipschitz bound is enough

Suppose we know the exact distances `d(t_0)` and `d(t_1)` at the endpoints of an
interval `I = [t_0, t_1]`, and suppose `d(t)` is `L_I`-Lipschitz on that interval.
Then for all `t in I`,

- `d(t) >= (d(t_0) + d(t_1) - L_I (t_1 - t_0)) / 2`.

So if that lower bound is already at least the separation threshold `D`, the entire
interval is certified safe without any further sampling inside it.

### 4.2 Simple global bound

A simple valid global bound is

- `L_global = s_A^+ + s_B^+`

because the relative closing speed can never exceed the sum of the speed maxima.

This is easy to justify but often quite loose. It assumes the pair could be maximally
closing throughout the entire interval even when the heading geometry rules that out.

### 4.3 Interval-local corner-speed bound

The default implementation uses a tighter interval-local bound.

For a fixed speed corner, the relative hull corner trajectory has derivative

- `r_i'(t) = s_A u_A(t) - s_B u_B(t)`.

Hence a valid Lipschitz constant on `I` is

- `L_I = sup_{t in I} max_i ||r_i'(t)||`.

This is valid because:

1. each hull corner moves with speed `||r_i'(t)||`
2. the convex hull cannot move faster in Hausdorff distance than its fastest corner
3. distance to a moving closed set is 1-Lipschitz with respect to Hausdorff motion.

The practical point is that this bound is still cheap. Between turn-completion events,
each heading evolves linearly in time, so the relative heading `delta(t)` is linear on
that piece. For one speed corner pair,

- `||s_A u_A(t) - s_B u_B(t)||^2 = s_A^2 + s_B^2 - 2 s_A s_B cos(delta(t))`.

On a piece where `delta(t)` is linear, the maximum occurs at:

- an endpoint, or
- an internal anti-parallel crossing `delta(t) = (2k + 1) pi`.

So the exact interval-local corner-speed supremum is obtained by checking only a
constant number of candidate times on each piece. That supremum is then used as the
Lipschitz constant in the certification rule.

### 4.4 Why this bound is a good default

This bound is a good default for three reasons:

1. It is still provably safe.
2. It is tighter than the global `s_A^+ + s_B^+` bound whenever the known heading
   geometry excludes full anti-parallel closing over the whole interval.
3. A tighter bound reduces unnecessary interval subdivision, which often lowers both
   conservatism and total runtime.

The simple global bound is still kept as an option for comparison, debugging, and
sanity-checking the local derivation.

## 5. Adaptive certified search in time

The interval search works as follows:

1. Evaluate `d(t)` exactly at the interval endpoints.
2. Compute a valid Lipschitz constant on that interval.
3. If the lower bound implied by those endpoint distances is at least `D`, mark the
   whole interval safe.
4. Otherwise, sample the midpoint exactly and split the interval in two.
5. Repeat until every interval is either certified safe or smaller than a configured
   minimum width.

If an interval reaches the minimum width and still cannot be certified safe, the method
returns unsafe. This is where conservatism enters. It is a deliberate choice: the code
refuses to declare safety unless it has an interval certificate for the full horizon.

## 6. Structured hull optimization

The straight and turning paths share the same fixed-time affine structure:

- `base + s_A a_basis - s_B b_basis`.

Therefore the reachable set is always determined by four speed-corner points. The code
now exploits that explicitly.

### 6.1 Shared corner kernel

The helper `_speed_rectangle_corner_points` computes the four affine speed-corner points
for both:

- the straight-line displacement hull at `t = T`
- the turn-aware relative-position hull at a chosen time `t`.

This removes duplicated corner-construction logic and makes the shared convex geometry
explicit in the implementation.

### 6.2 Small structured hull builder

The hot-path hulls in this module are tiny:

- four-point hulls from speed corners
- five-point hulls for the straight displacement cone when the origin is included.

Using the full generic monotone-chain hull on these tiny structured inputs is correct
but unnecessary. `small_convex_hull` specializes the hot path for exactly these cases.

The generic `convex_hull` routine is intentionally retained as a reference
implementation. Tests compare the optimized path against it so that the optimization
remains a change in cost, not a change in mathematical meaning.

## 7. Safety guarantees

### 7.1 Straight-heading solver

Under the fixed-heading, bounded-constant-speed model:

- exact in speed
- exact in time
- exact safe/unsafe decision
- exact reported minimum distance and closest time.

### 7.2 Turn-aware solver

Under the one-turn-then-straight model:

- exact in speed at every sampled time
- certified safe only when every interval is proved safe
- conservative in time because uncertified small intervals are treated as unsafe.

So in the turn-aware path:

- `is_separated` is the key guarantee
- `is_separated = True` means the whole horizon was certified safe under the model
- `is_separated = False` means either a feasible loss of separation was witnessed or an
  interval could not be certified safe at the configured minimum width
- the reported closest time and minimum distance are diagnostic summaries of the sampled
  points and interval certificates, not a claim of a closed-form exact continuous-time
  minimizer.

## 8. Robustness to in-turn speed variation

The current turn-aware solver assumes that each aircraft chooses one constant speed for
the full horizon. That assumption is structurally necessary for the four-corner
fixed-time reachable hull. Still, it is useful to quantify how far the true position
could move away from the current model if speed were allowed to vary within the same
bounds during the turn.

### 8.1 One-aircraft fixed-time error bound

For one aircraft, let

- `u(t)` be the unit heading vector
- `s(t) in [s^-, s^+]` be an admissible time-varying speed
- `b(t) = integral_0^t u(xi) d xi`

The true position offset from the start is

- `x_true(t) = integral_0^t s(xi) u(xi) d xi`

whereas the current model allows only the constant-speed segment

- `x_model(t; s) = s b(t)`, with `s in [s^-, s^+]`.

Let `Delta s = s^+ - s^-`, and let `Theta(t)` be the total heading sweep experienced by
the aircraft on `[0, t]`. For the one-turn model in this module, `Theta(t)` is at most
`pi` radians. Then the true variable-speed position lies within the following distance
of the current constant-speed model set:

- `dist(x_true(t), {x_model(t; s) : s in [s^-, s^+]})`
- `<= (t Delta s / 2) sin(Theta(t) / 2)`.

This bound has the right qualitative behaviour:

- straight flight: `Theta(t) = 0`, so the model-mismatch bound is `0`
- small turns: the error grows approximately linearly with both turn angle and speed
  range
- larger speed ranges or longer times produce larger potential mismatch.

The bound comes from two facts:

1. after subtracting the time-average speed, the positive and negative speed deviations
   have equal total mass, and that mass is at most `t Delta s / 4`
2. the weighted heading averages induced by those positive and negative parts both lie
   in the convex hull of the heading arc, whose diameter is `2 sin(Theta(t) / 2)` for
   `Theta(t) <= pi`.

Multiplying those two factors gives the stated bound.

### 8.2 Relative-position bound for two aircraft

If both aircraft may vary speed during their turns, then at fixed time `t` the
relative-position mismatch is bounded by the sum of the two one-aircraft errors:

- `error_rel(t) <= error_A(t) + error_B(t)`.

So the true variable-speed relative-position set lies within a radius
`error_A(t) + error_B(t)` of the current constant-speed relative hull. In practical
terms, this suggests a conservative robustness buffer:

- either inflate the separation threshold by that amount at time `t`
- or use the supremum of that amount over the horizon as a single global buffer.

The current Python implementation now exposes this idea as an optional turn-aware
argument:

- `turn_speed_schedule_uncertainty_kt`

which is off by default. When it is positive, the code computes a conservative
horizon-wide additive buffer from the one-aircraft bounds above and inflates the
separation threshold by that amount. This keeps the reachable-set geometry and the
certification logic unchanged, at the cost of additional conservatism.

If one wants the robustness buffer to cover in-turn speed reordering within the same
speed interval already used by the main model, a natural choice is

- `turn_speed_schedule_uncertainty_kt = speed_diff_kt`.

### 8.3 Example in real units

Suppose one aircraft has an in-turn speed uncertainty of `+/-15 kt` around nominal, so

- `Delta s = 30 kt approx 15.43 m/s`.

Suppose further that after `t = 30 s` it has swept through `Theta = 45 deg`, i.e.
`Theta = pi / 4`. Then

- `error(t) <= (30 * 15.43 / 2) sin(22.5 deg)`
- `error(t) <= 88.6 m`.

So for one aircraft, bounded in-turn speed variation can move the true position by at
most about `89 m` away from the constant-speed model segment at that time.

If two aircraft both had the same bound at the same time, the relative-position
mismatch would be bounded by

- `88.6 m + 88.6 m = 177.2 m`
- `approx 0.096 NMI`.

That is small compared with a `5 NMI` separation threshold, but not negligible if one
cares about tight margins or formal robustness to model mismatch.

As a more aggressive upper-envelope example, suppose the in-turn speed uncertainty is
`+/-50 kt`, so

- `Delta s = 100 kt approx 51.44 m/s`.

Suppose the aircraft sweeps through a full `90 deg` turn. If that turn is flown at
`3 deg/s`, its duration is `30 s`, so taking `t = 30 s` and `Theta = 90 deg` gives

- `error(t) <= (30 * 51.44 / 2) sin(45 deg)`
- `error(t) <= 545.5 m`
- `approx 0.295 NMI`

for one aircraft.

If two aircraft both had that same aggressive bound at the same time, the relative
position mismatch would be bounded by

- `545.5 m + 545.5 m = 1091.0 m`
- `approx 0.589 NMI`.

That is no longer tiny relative to a `5 NMI` threshold. It suggests that if one wants
robustness to genuine in-turn speed variation at this level, an explicit additional
buffer could be justified.

The dependence on turn duration is also important. The same `90 deg` turn flown at
`1.5 deg/s` lasts `60 s`, which doubles the bound:

- one-aircraft mismatch: about `1091 m` (`0.589 NMI`)
- two-aircraft relative mismatch: about `2182 m` (`1.178 NMI`).

So large in-turn speed variation is most relevant when the turn is both wide and slow.

## 9. Proof plan for a paper

The implementation is already close to a paper-friendly proof structure. The clearest
way to present it would be as a sequence of propositions and one main certification
theorem.

For the model stated in Section 1, the core mathematical claims should all be
provable. Nothing in Sections 2 through 7 relies on a heuristic geometric guess. The
main ingredients are standard facts from convex geometry, Euclidean projection, and
Lipschitz analysis of moving sets.

What the current Python code and test suite provide is strong validation:

- exact-oracle comparisons for the structured hull shortcuts
- numerical checks of the interval-local corner-speed bound
- falsification sweeps for the straight and turn-aware decision logic
- regression tests for the specific bugs that motivated the rewrite.

That validation is important, but it is not the proof. The proof still needs to be
written in mathematics-first form. The good news is that, for the stated setting, the
proof obligations are modest and fairly clean.

The main caveat is scope. The proofs apply to the model in Section 1:

- local tangent-plane geometry
- bounded constant speeds for each realization
- at most one fixed-rate turn per aircraft
- exact real arithmetic.

They do not prove anything stronger, such as:

- robustness to geodesic or long-range Earth-curvature effects
- robustness to floating-point roundoff in arbitrary edge cases
- correctness under acceleration, wind, multiple turns, or general time-varying speed
  schedules.

So the right statement is:

- yes, the mathematical claims for the implemented model look provable
- no, the code itself does not replace the written proof
- and the paper should be careful not to overstate what setting those proofs cover.

### 9.0 Risk assessment by claim

The claims do not all carry the same proof risk.

Lowest risk:

1. Straight-heading reachable-set characterization
2. Straight-heading closest-approach projection formulation
3. Fixed-time turn-aware hull as an affine image of the speed rectangle.

These are almost direct consequences of the model equations.

Moderate but still clean:

1. Interval-local corner-speed bound
2. Certified-search safety theorem.

These need a little more care because they move from pointwise geometry to set-valued
motion and interval certification, but they are still standard enough that I would
expect a compact proof.

Slightly more delicate:

1. The earliest-time recovery lemma for the straight solver
2. The optional robustness extension for in-turn speed variation.

These are still tractable, but they are the places where a paper would most benefit
from a careful lemma statement rather than relying on intuition.

### 9.1 Proposition: straight-heading reachable set

Statement:

Let

- `V = {s_A u_A - s_B u_B : s_A in [s_A^-, s_A^+], s_B in [s_B^-, s_B^+]}`.

If headings are fixed and speeds are constant over the horizon, then the full reachable
relative-displacement set on `[0, T]` is

- `S(T) = {t v : t in [0, T], v in V}`
- `= conv({0} union {T v_i})`

where `v_i` are the four speed-corner relative velocities.

Why it matters:

- it collapses the full continuous-time straight problem to one compact convex polygon.

Proof sketch:

1. The set `V` is the affine image of the speed rectangle, so it is convex and equals
   the convex hull of its four corner images `v_i`.
2. If `d in S(T)`, then `d = t v = (t / T) (T v)` for some `v in V`, hence
   `d in conv({0} union {T v_i})`.
3. Conversely, any point in `conv({0} union {T v_i})` can be written as
   `(1 - lambda) 0 + lambda T v` with `lambda in [0, 1]` and `v in V`, hence equals
   `t v` with `t = lambda T in [0, T]`.

So the two sets coincide exactly.

### 9.2 Proposition: straight-heading closest approach is one convex projection

Statement:

Under the straight-heading model,

- `d_* = min_{d in S(T)} ||r_0 + d||`
- `= dist(-r_0, S(T))`.

Equivalently, if `d^*` is the Euclidean projection of `-r_0` onto `S(T)`, then

- `d_* = ||r_0 + d^*||`.

Why it matters:

- it gives exactness of the straight solver once Proposition 9.1 is known.

Proof sketch:

1. Every admissible relative trajectory has the form `r(t) = r_0 + d` for some
   `d in S(T)`.
2. Therefore minimizing separation over time and speed is exactly the same as minimizing
   `||r_0 + d||` over `d in S(T)`.
3. Since `S(T)` is compact and convex, Euclidean projection onto `S(T)` is well-defined
   and attains the minimum distance from `-r_0` to `S(T)`.

This is the whole straight-heading safety argument.

### 9.3 Lemma: earliest-time recovery in the straight case

Statement:

Let `p in S(T)`. Define

- `alpha_*(p) = inf {alpha in [0, 1] : p in alpha S(T)}`.

Then the earliest time at which `p` is reachable is `t_*(p) = T alpha_*(p)`.
Moreover, because the ambient space is 2-D, `alpha_*(p)` can be realized using the
origin and at most two nonzero hull vertices.

Why it matters:

- it justifies the time-recovery step after the closest-point projection has been found.

Proof sketch:

1. By definition of `S(T)`, `p` is reachable at time `t` if and only if
   `p in (t / T) S(T)`.
2. Therefore the earliest reachable time is exactly `T` times the gauge of `p` with
   respect to `S(T)`.
3. Since `S(T)` is a compact convex subset of `R^2` containing the origin, Caratheodory
   implies that any point in `alpha S(T)` can be represented using the origin and at
   most two outer vertices.
4. That is why the implementation only needs to search one-vertex and two-vertex
   supports.

### 9.4 Proposition: fixed-time turn-aware hull

Statement:

Under the one-turn-then-straight model and constant chosen speeds, define

- `b_A(t) = integral_0^t u(h_A(xi)) d xi`
- `b_B(t) = integral_0^t u(h_B(xi)) d xi`.

Then for each fixed time `t`,

- `R(t) = r_0 + {s_A b_A(t) - s_B b_B(t)}`

with `s_A` and `s_B` ranging over their intervals. Hence `R(t)` is the affine image of
the speed rectangle and is therefore always a point, a segment, or a parallelogram.

Why it matters:

- it is the exact-in-speed core of the turn-aware method.

Proof sketch:

1. The heading law is deterministic, so for each aircraft the basis `b(t)` depends only
   on time and on the commanded turn, not on speed.
2. Position therefore has the affine form `p(t; s) = p(0) + s b(t)`.
3. Subtracting the two aircraft positions gives a map from the 2-D speed rectangle into
   relative-position space that is affine in `(s_A, s_B)`.
4. The image of a rectangle under an affine map is a convex polygon with at most four
   vertices, degenerating only when the geometry becomes lower-dimensional.

### 9.5 Proposition: interval-local corner-speed bound

Statement:

Let `d(t) = dist(0, R(t))`. On any interval `I = [t_0, t_1]`, a valid Lipschitz
constant for `d(t)` is

- `L_I = sup_{t in I} max_i ||r_i'(t)||`

where `r_i(t)` runs over the fixed-time hull corners.

Why it matters:

- this is what makes the adaptive certification step mathematically simple.

Proof sketch:

1. Each corner of `R(t)` evolves as `r_i(t) = r_0 + s_A b_A(t) - s_B b_B(t)`, so its
   instantaneous speed is `||r_i'(t)|| = ||s_A u_A(t) - s_B u_B(t)||`.
2. The Hausdorff motion of a convex hull is bounded by the largest motion of its
   vertices, so the set-valued map `t -> R(t)` is Lipschitz in Hausdorff distance with
   constant `L_I`.
3. Distance to a closed set is 1-Lipschitz with respect to Hausdorff perturbations, so
   `d(t)` is itself `L_I`-Lipschitz on `I`.

The computational lemma underneath this proposition is:

- between turn-completion events, each heading is linear in time
- therefore the relative heading is linear in time
- for a fixed speed corner pair, the squared corner speed is
  `s_A^2 + s_B^2 - 2 s_A s_B cos(delta(t))`
- hence the supremum on such a subinterval occurs at an endpoint or at an internal
  anti-parallel crossing.

This proves both validity and efficient computability of the interval-local bound.

### 9.6 Theorem: certified adaptive search

Statement:

If the turn-aware algorithm returns `is_separated = True`, then every admissible
trajectory in the stated model remains outside the protected separation region on
`[0, T]`.

Why it matters:

- this is the central safety guarantee of the turn-aware solver.

Proof sketch:

1. On each interval `I = [t_0, t_1]`, Proposition 9.5 provides a valid Lipschitz
   constant `L_I` for `d(t)`.
2. Therefore
   `d(t) >= (d(t_0) + d(t_1) - L_I (t_1 - t_0)) / 2`
   for all `t in I`.
3. If this lower bound exceeds the separation threshold, the whole interval is safe.
4. Otherwise the algorithm subdivides, unless it has reached the minimum certification
   width, in which case it reports unsafe.
5. So a `True` return is possible only if the entire horizon has been covered by
   certified-safe intervals.

That is exactly the desired safety statement.

### 9.7 Corollary: conservatism

Statement:

- the turn-aware solver may return `False` in cases where the modelled aircraft would in
  fact remain separated, but it cannot return `True` without a certificate for the full
  horizon.

This is the right place in a paper to explain the trade:

- exact fixed-time geometry
- conservative time search
- intentionally asymmetric safe/unsafe semantics.

### 9.8 Optional robustness extension

If the paper wants to address the "constant speed during turns" objection directly, the
robustness material in Section 8 can be turned into a short proposition:

- bounded in-turn speed variation stays within an additive spatial buffer of the current
  constant-speed hull
- that buffer vanishes in the straight case
- and it can be folded into the decision rule by threshold inflation.

That is not part of the core theorem, but it is a useful extension and rebuttal.

## 10. Paper outline

This section is not meant to say "this is the only correct paper structure." It records
the structure that currently looks best if the target venue is *Transportation Research
Part C*. A different venue would likely justify a different emphasis.

Accordingly, the outline below assumes two framing choices:

1. the paper is positioned exactly as in Sections 1.4 and 1.5: not as a claim of first
   invention of uncertainty-aware or turn-aware conflict detection, but as a narrow
   pairwise safety primitive with clean guarantees, explicit limitations, and runtime
   relevance
2. the paper is being shaped for a `TR Part C` audience: transportation-systems readers
   who can tolerate mathematics, but who will care at least as much about problem fit,
   guarantees, and practical computational plausibility as about the formal details
   themselves.

An introduction for such a paper should make four points quickly:

- prior work already contains uncertainty-aware, turn-aware, and reachable-set ideas
- the present contribution is the integration of those ideas into a very specific bounded
  model with an unusually clear safety story
- the straight case is exact, and the one-turn case is exact in speed and conservative in
  time
- the method is intentionally small enough to sit in a hot industrial filtering or
  optimization loop.

The earlier ten-part outline is probably too granular. It reads more like a list of
topics than a paper argument. A better structure is one that moves in the same order as
the contribution itself:

1. define the bounded model
2. show the exact straight case
3. extend to the turn-aware certified case
4. explain the guarantee and the limitations
5. show that the method is computationally plausible.

With that in mind, the recommended structure is:

1. Introduction and positioning
   - conflict detection setting and transportation-systems motivation
   - short related-work paragraph: prior uncertainty-aware, turn-aware, and reachable-set
     work already exists
   - contribution statement for this narrower model class
   - summary of guarantees and runtime intent
   - why this narrower primitive matters in a larger operational pipeline.
2. Model and decision problem
   - local tangent-plane geometry
   - bounded speed model
   - straight-heading and one-turn-then-straight kinematics
   - protected-separation decision question `d_* >= D`
   - explicit scope assumptions.
3. Exact straight-heading method
   - reachable displacement hull
   - closest approach as one convex projection
   - earliest-time recovery
   - exactness result.
4. Turn-aware certified method
   - fixed-time turn basis
   - affine-in-speed reachable hull
   - interval-local Lipschitz bound
   - adaptive certification in time
   - safe/unsafe semantics.
5. Guarantees, robustness, and limitations
   - what is exact and what is conservative
   - optional in-turn speed-variation buffer
   - why straight flight gets variable speed "for free"
   - what is outside the model: multiple turns, acceleration, wind, vertical motion.
6. Computational structure and implementation
   - shared corner kernel
   - small structured hull builder
   - asymptotic cost
   - practical runtime measurements and hot-path suitability
   - why that cost profile matters for transportation-system deployment.
7. Empirical evaluation
   - exactness checks for the straight case
   - falsification and conservative-behaviour checks for the turn-aware case
   - benchmark timings
   - illustrative scenarios or figures
   - if possible, one short comparison against a simpler baseline such as dense temporal
     sampling without certification.
8. Conclusion
   - narrow model, clean guarantee, transportation relevance, practical runtime
   - what the method is useful for
   - obvious next extensions.

This structure is better for three reasons:

- it groups the whole turn-aware story into one coherent method section rather than
  scattering it across multiple sections
- it keeps guarantees and model boundaries adjacent, which is where reviewers will look
  for the real value and the real limits
- it gives computation and evaluation enough weight to support the "industrial primitive"
  positioning.

Appendices could then hold:

- the detailed proofs from Section 9
- the extended related-work notes from Section 1.4, if the main paper needs to stay lean
- implementation notes
- extra benchmark tables or scenario plots.
