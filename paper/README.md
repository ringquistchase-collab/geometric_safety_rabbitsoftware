# Paper evaluation

This directory contains the evaluation code for the paper:

> **Certified Pairwise Lateral Safety for Bounded-Speed Aircraft
> Encounters with Single-Turn Manoeuvres**
> George De Ath and Ben Carvell

It is separate from the core `geometric_safety` solver so that
downstream users of the library are not affected by paper-specific
infrastructure.

## Reproducing the empirical paper results

### Prerequisites

```bash
git clone https://github.com/project-bluebird/geometric_safety.git
cd geometric_safety
uv sync
```

### Full pipeline

The paper pipeline is split into a data stage and a rendering stage.
The data stage saves compact summaries, the row-level data needed for
figure regeneration, and terminal logs.
The rendering stage reads those saved outputs to generate figures and
the versioned capped projection archive. It also regenerates the two
explanatory application figures used in the manuscript, so the public
code repository is the source for all paper figures.

The full data stage is CPU-intensive. As a reference point, a
128-worker Linux server completes the run in roughly 30--60 minutes.
On smaller machines, reduce `N_JOBS` or run the smoke test first with
`QUICK=1`.

#### Full data stage

Run the expensive data/log generation stage with:

```bash
N_JOBS=128 ./paper/run_full_results.sh \
    paper_eval_outputs/paper_full_$(date +%Y%m%d_%H%M%S)
```

This writes the following key files for the default five-seed full run:

```text
paper_eval_outputs/<run>/
  logs/
    main_evaluation.log
    projection_sweep.log
  main_evaluation/
    campaign_summary.json
    campaign_runs.csv
    run_00_seed_<seed>/
      summary.json
      run_record.json
      stress_map_source.csv
      near_threshold_default_margin_summary.csv
      table_*.csv
    run_01_seed_<seed+step>/
      ...
    ...
  projection_full/
    summary.json
    projection_rows.csv.gz
    *_summary_by_latitude.csv
    *_by_excursion.csv
    mixed_turn_by_excursion_and_turn_count.csv
```

With the script defaults, the first run directory is
`run_00_seed_20260325`. If `REPEAT_SEEDS=1`, `paper.run_evaluation`
writes the same per-run files directly under `main_evaluation/` rather
than inside `run_00_seed_<seed>/`.

#### Rendering stage

After the data stage completes, render figures from the saved outputs:

```bash
./paper/render_paper_figures.sh paper_eval_outputs/<run>
```

This writes the empirical paper-evaluation PDFs under:

```text
paper_eval_outputs/<run>/rendered_figures/
  application/
    fig_lateral_overlap_schematic.pdf
    fig_clearance_grid.pdf
    fig_clearance_grid_source.json.gz
  main/
    fig_crossing.pdf
    fig_nominal_proxy_miss.pdf
    fig_stress_maps.pdf
    fig_ablation.pdf
    fig_sampled_proxy_miss.pdf
  projection_deterministic_51_cap_100nmi/
    projection_min_error_vs_excursion.pdf
    ...
```

It also writes the versioned supplementary projection artifact:

```text
paper/results/projection_deterministic_51_cap_100nmi.tar.gz
```

and a compact archive of the full saved run:

```text
paper/results/<run>.tar.gz
```

See `paper/reproducibility_manifest.md` for the mapping from saved
artifacts to manuscript figures and numerical claims.

To check that a completed run contains the expected source artifacts:

```bash
uv run python -m paper.check_reproducibility_artifacts paper_eval_outputs/<run>
uv run python -m paper.check_reproducibility_artifacts \
    paper_eval_outputs/<run> --require-rendered
```

#### Manual data commands

#### Step 1: Main evaluation (5 seeds, 100k encounters per suite)

This produces all numerical results without rendering figures. It writes
compact figure-source tables, including the stress-map cell counts and
rates and the default near-threshold margin-bin summary, but avoids the
much larger straight, mixed-turn, and near-threshold row dumps.

```bash
uv run python -m paper.run_evaluation \
    --profile large100k \
    --repeat-seeds 5 \
    --n-jobs 128 \
    --output-dir paper_eval_outputs/paper_full/main_evaluation \
    --row-output-mode none \
    --skip-plots
```

#### Step 2: Projection sensitivity sweep

This produces the saved data for the supplementary projection figure.

```bash
uv run python -m paper.run_projection_sweep \
    --n-jobs 128 \
    --output-dir paper_eval_outputs/paper_full/projection_full \
    --combined-rows \
    --compress-rows
```

#### Step 3: Render figures

This reads the saved outputs and renders figures without rerunning the
expensive evaluation. The path below is the first seed run produced by
the default full pipeline; use the matching `run_00_seed_<seed>` path if
the seed is changed.

```bash
uv run python -m paper.run_evaluation \
    --render-figures-from paper_eval_outputs/paper_full/main_evaluation/run_00_seed_20260325 \
    --figure-output-dir paper_eval_outputs/paper_full/rendered_figures/main \
    --output-dir paper_eval_outputs/paper_full/render_work/main \
    --figure-formats pdf
```

The projection figure is rendered separately, with the 100 NMI
excursion cap used in the manuscript:

```bash
uv run python -m paper.plot_projection_sweep \
    --input-dir paper_eval_outputs/paper_full/projection_full \
    --output-dir paper_eval_outputs/paper_full/rendered_figures/projection_deterministic_51_cap_100nmi \
    --max-excursion-nmi 100 \
    --write-rows \
    --combined-rows \
    --archive-output paper/results/projection_deterministic_51_cap_100nmi.tar.gz
```

The application figures can also be rendered directly:

```bash
uv run python -m paper.render_lateral_overlap_figure \
    --output-pdf paper_eval_outputs/paper_full/rendered_figures/application/fig_lateral_overlap_schematic.pdf

uv run python -m paper.render_clearance_grid_figure \
    --output-pdf paper_eval_outputs/paper_full/rendered_figures/application/fig_clearance_grid.pdf \
    --source-json paper_eval_outputs/paper_full/rendered_figures/application/fig_clearance_grid_source.json.gz \
    --write-source-json
```

To inspect or extract the archived result:

```bash
tar -tzf paper/results/projection_deterministic_51_cap_100nmi.tar.gz
mkdir -p paper_eval_outputs/projection_deterministic_51_cap_100nmi_extracted
tar -xzf paper/results/projection_deterministic_51_cap_100nmi.tar.gz \
    -C paper_eval_outputs/projection_deterministic_51_cap_100nmi_extracted
```

### Quick smoke test

To verify the pipeline works without running the full evaluation:

```bash
QUICK=1 N_JOBS=4 ./paper/run_full_results.sh paper_results/quick
PROJECTION_ARCHIVE=paper_results/quick_projection.tar.gz \
FULL_ARCHIVE=paper_results/quick.tar.gz \
    ./paper/render_paper_figures.sh paper_results/quick
uv run python -m paper.check_reproducibility_artifacts \
    paper_results/quick --require-rendered \
    --projection-archive paper_results/quick_projection.tar.gz
```

## Output structure

After running the full pipeline, the relevant output directories contain:

```text
paper_eval_outputs/<run>/main_evaluation/
  campaign_summary.json
  campaign_runs.csv
  run_00_seed_<seed>/
    summary.json
    run_record.json
    stress_map_source.csv
    near_threshold_default_margin_summary.csv
    table_1_experiment_design.csv
    table_2_straight_exactness.csv
    table_3_mixed_turn_results.csv
    table_4_runtime_summary.csv
  run_01_seed_<seed+step>/
    ...
paper_eval_outputs/<run>/rendered_figures/
  application/
    fig_lateral_overlap_schematic.pdf
    fig_clearance_grid.pdf
    fig_clearance_grid_source.json.gz
  main/
    fig_crossing.pdf
    fig_nominal_proxy_miss.pdf
    fig_stress_maps.pdf
    fig_ablation.pdf
    fig_sampled_proxy_miss.pdf
  projection_deterministic_51_cap_100nmi/
    projection_min_error_vs_excursion.pdf
    projection_error_by_range_and_horizon.pdf
    projection_orientation_disagreement.pdf
    projection_disagreement_by_excursion.pdf
paper/results/
  projection_deterministic_51_cap_100nmi.tar.gz
    summary.json
    projection_rows.csv
    straight_summary_by_latitude.csv
    straight_by_excursion.csv
    mixed_turn_summary_by_latitude.csv
    mixed_turn_by_excursion.csv
    mixed_turn_by_excursion_and_turn_count.csv
    projection_min_error_vs_excursion.pdf
    projection_error_by_range_and_horizon.pdf
    projection_orientation_disagreement.pdf
    projection_disagreement_by_excursion.pdf
```

### Figure-to-manuscript mapping

The full rendering stage writes the empirical paper figures plus a few
diagnostic scenario figures retained for reproducibility. The current
manuscript uses the following empirical outputs:

| Script output | Manuscript item |
| ------------- | --------------- |
| `fig_lateral_overlap_schematic.pdf` | Main-paper introductory lateral-overlap schematic |
| `fig_clearance_grid.pdf` | Main-paper clearance-grid illustration |
| `fig_stress_maps.pdf` | Main-paper mixed-turn stress-map figure |
| `fig_ablation.pdf` | Main-paper near-threshold ablation figure |
| `fig_sampled_proxy_miss.pdf` | Supplementary sampled-heuristic figure |
| `projection_min_error_vs_excursion.pdf` | Supplementary projection-sensitivity figure |

The explanatory application figures are not stochastic experiment
outputs, but they are generated by this public repository so the
published manuscript figures can be reproduced from one codebase.

To populate a separate manuscript checkout with the generated PDFs:

```bash
uv run python -m paper.reproduce_paper_outputs \
    --manuscript-root /path/to/2026-03-geometric-safety-paper \
    --archive paper/results/<run>.tar.gz
```

## Key paper claims and where to verify them

| Claim | Where to check |
| ----- | -------------- |
| 100% straight exactness | `run_*/summary.json` > `straight` |
| 0 false-safe (mixed-turn) | `run_*/summary.json` > `mixed_turn` |
| 99.83% certification | `run_*/summary.json` > `mixed_turn` |
| Rule-of-three 5.6e-5 | `campaign_summary.json` > `aggregate` |
| 95.36% near-threshold | `run_*/summary.json` > `near_threshold` |
| Near-threshold margin-bin rates | `run_*/near_threshold_default_margin_summary.csv` |
| 1,047 / 535 flips | `summary.json` inside `paper/results/projection_deterministic_51_cap_100nmi.tar.gz` |
| 94 / 52 above-band projection flips | `projection_rows.csv` inside `paper/results/projection_deterministic_51_cap_100nmi.tar.gz` |
| 2.02 us fixed-time median runtime | `run_*/summary.json` > `kernel_runtime` |
| 5.16 / 13.41 / 23.37 us full-solver runtimes | `run_*/summary.json` > `mixed_turn.runtime` |

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
    paper_eval_outputs/<run>/main_evaluation/run_00_seed_20260325
```

For a non-default seed, replace `run_00_seed_20260325` with the first
per-seed run directory in `main_evaluation/`. For a single-seed run,
use `paper_eval_outputs/<run>/main_evaluation` directly.
