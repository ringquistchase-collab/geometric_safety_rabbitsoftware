# geometric_safety

Geometric reachability-based safety filtering for air traffic control.

## Overview

This package implements the Catch-Up Projection Interval (CUPI)
solver for lateral aircraft separation filtering. Given two aircraft
with known positions, headings, speed bounds, and optional turn
profiles, it determines whether they can possibly come within a
separation threshold over a projection horizon.

Two solver variants:

- **Fixed heading** (`catch_up_projection_interval`) — exact
  geometric check for straight-line flight. Reduces the problem
  to one convex projection.
- **Turn-aware** (`catch_up_projection_interval_with_turns`) —
  handles one constant-rate turn per aircraft, then straight.
  Uses exact fixed-time hull evaluations with interval
  certification in time.

The solver uses only numpy and numba (no ATC domain dependencies),
making it reusable outside Falcon.

## Installation

Add to an existing uv project:

```bash
uv add geometric-safety \
    --git https://github.com/project-bluebird/geometric_safety.git
```

This adds two entries to your `pyproject.toml`:

```toml
# in [project] dependencies
"geometric_safety>=0.1.0"

# in [tool.uv.sources]
geometric_safety = {
    git = "https://github.com/project-bluebird/geometric_safety"
}
```

To pin a specific commit or tag, pass `--rev`:

```bash
uv add geometric-safety \
    --git https://github.com/project-bluebird/geometric_safety.git \
    --rev <commit-or-tag>
```

For local development of this repo itself:

```bash
uv sync
```

## Usage

```python
from geometric_safety import catch_up_projection_interval_with_turns

is_separated, min_distance_m, closest_time_s = (
    catch_up_projection_interval_with_turns(
        a_lat=51.0, a_lon=-1.0,
        a_heading0_deg=0.0, a_target_heading_deg=270.0,
        a_speed_kt=340.0, a_turn_rate_deg_sec=1.5,
        b_lat=51.03, b_lon=-0.97,
        b_heading0_deg=90.0, b_target_heading_deg=90.0,
        b_speed_kt=340.0, b_turn_rate_deg_sec=0.0,
        separation_threshold_m=9260.0,  # 5 NMI
        speed_diff_kt=15.0,
        projection_time_s=900.0,
    )
)
```

## Tests

```bash
uv run pytest tests/
uv run pytest tests/test_relevant_aircraft_turns.py
uv run pytest tests/test_relevant_aircraft_app.py
```

## Relevant-aircraft visualiser

An interactive FastAPI interface for inspecting the CUPI geometry.
Supports both fixed-heading and turn-aware modes.

- **Fixed heading mode** — sweeps aircraft B's initial position
  across a configurable grid, showing the straight-line reachable
  envelopes over time.
- **Turn aware mode** — uses configured initial headings, target
  headings, turn rates, and optional turn-speed robustness margin.
  The detail view shows spatial trajectories, the relative hull at
  the selected time, and the distance-vs-time curve.

Turn-aware mode includes canned presets:

- Single-turn dodge
- Single-turn cut-in
- Mutual opening turns
- Mutual closing turns

```bash
uv sync --group apps
uv run uvicorn app:app --reload
```

Then browse to <http://127.0.0.1:8000/>.

## Benchmarks

Isolated CUPI timing benchmarks:

```bash
uv run python scripts/benchmark_relevant_aircraft.py
uv run python scripts/benchmark_relevant_aircraft.py \
    --random-cases 5000
```

## Paper evaluation

The `paper/` directory contains the evaluation code for the
accompanying manuscript. See [`paper/README.md`](paper/README.md)
for reproduction instructions.

## Documentation

- `relative_aircraft_projection_walkthrough.ipynb` — interactive
  walkthrough notebook
