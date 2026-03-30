# Paper evaluation

This directory contains the evaluation code for the paper:

> **Certified Pairwise Lateral Safety for Bounded-Speed Aircraft
> Encounters with Single-Turn Manoeuvres**
> George De Ath

It is separate from the core `geometric_safety` solver so that
downstream users of the library are not affected by paper-specific
infrastructure.

## Reproducing all paper results

### Prerequisites

```bash
git clone <repo-url> && cd geometric_safety
uv sync
```

### Full pipeline

The paper results come from three commands. All output goes into
a single `paper_results/` directory. On a 128-core machine the
full pipeline takes roughly 30-60 minutes.

#### Step 1: Main evaluation (5 seeds, 100k encounters per suite)

This produces all numerical results and 5 of the 6 paper figures.

```bash
uv run python -m paper.run_evaluation \
    --profile large100k \
    --repeat-seeds 5 \
    --n-jobs 128 \
    --output-dir paper_results
```

#### Step 2: Projection sensitivity sweep

This produces the data for the supplementary projection figure.

```bash
uv run python -m paper.run_projection_sweep \
    --n-jobs 128 \
    --output-dir paper_results/projection
```

#### Step 3: Render projection figure

This reads the sweep data and renders the 6th paper figure.

```bash
uv run python -m paper.plot_projection_sweep \
    --input-dir paper_results/projection \
    --output-dir paper_results/projection/plots \
    --max-excursion-nmi 100
```

### Quick smoke test

To verify the pipeline works without running the full evaluation:

```bash
uv run python -m paper.run_evaluation \
    --quick --output-dir paper_results/quick
```

## Output structure

After running the full pipeline, `paper_results/` contains:

```text
paper_results/
  campaign_summary.json
  campaign_runs.csv
  run_00_seed_20260325/
    summary.json
    straight_suite_rows.csv
    mixed_turn_rows.csv
    near_threshold_rows.csv
    table_1_experiment_design.csv
    table_2_straight_exactness.csv
    table_3_mixed_turn_results.csv
    table_4_ablation.csv
    figure_1_crossing.pdf
    figure_2_proxy_miss.pdf
    figure_3_stress_maps.pdf
    figure_4_ablation.pdf
    figure_sampled_proxy_miss.pdf
  run_01_seed_20270325/
    ...
  projection/
    summary.json
    straight_rows_lat_plus_51p0.csv
    mixed_turn_rows_lat_plus_51p0.csv
    straight_by_excursion.csv
    mixed_turn_by_excursion.csv
    plots/
      projection_min_error_vs_excursion.pdf
```

### Figure-to-manuscript mapping

| Script output | Manuscript figure |
| ------------- | ----------------- |
| `figure_1_crossing.pdf` | `fig_crossing.pdf` |
| `figure_2_proxy_miss.pdf` | `fig_nominal_proxy_miss.pdf` |
| `figure_3_stress_maps.pdf` | `fig_stress_maps.pdf` |
| `figure_4_ablation.pdf` | `fig_ablation.pdf` |
| `figure_sampled_proxy_miss.pdf` | `fig_sampled_proxy_miss.pdf` |
| `projection_min_error_vs_excursion.pdf` | `fig_projection_excursion.pdf` |

## Key paper claims and where to verify them

| Claim | Where to check |
| ----- | -------------- |
| 100% straight exactness | `run_*/summary.json` > `straight` |
| 0 false-safe (mixed-turn) | `run_*/summary.json` > `mixed_turn` |
| 99.83% certification | `run_*/summary.json` > `mixed_turn` |
| Rule-of-three 5.6e-5 | `campaign_summary.json` > `aggregate` |
| 95.41% near-threshold | `run_*/summary.json` > `near_threshold` |
| 1,047 / 535 flips | `projection/summary.json` |

## Additional diagnostic scripts

These are not needed for paper reproduction but are available for
further investigation:

| Command | Purpose |
| ------- | ------- |
| `uv run python -m paper.run_projection_sanity` | Multi-latitude validation |
| `uv run python -m paper.run_projection_anchors` | Anchor-mode comparison |

## Regenerating figures from existing data

To re-render main figures without re-running the evaluation:

```bash
uv run python -m paper.run_evaluation \
    --render-figures-from \
    paper_results/run_00_seed_20260325
```
