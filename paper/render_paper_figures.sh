#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ge 1 ]]; then
    run_root="$1"
elif [[ -f paper_eval_outputs/latest_paper_run.txt ]]; then
    run_root="$(cat paper_eval_outputs/latest_paper_run.txt)"
else
    echo "Usage: $0 <paper-run-root> [figure-output-dir]" >&2
    exit 2
fi

figure_root="${2:-$run_root/rendered_figures}"
application_dir="$figure_root/application"
if [[ -n "${MAIN_RUN_DIR:-}" ]]; then
    main_run_dir="$MAIN_RUN_DIR"
elif [[ -f "$run_root/main_evaluation/summary.json" ]]; then
    main_run_dir="$run_root/main_evaluation"
else
    shopt -s nullglob
    run_summaries=("$run_root"/main_evaluation/run_00_seed_*/summary.json)
    shopt -u nullglob
    if [[ ${#run_summaries[@]} -eq 0 ]]; then
        echo "Could not find a renderable evaluation summary under $run_root/main_evaluation" >&2
        exit 1
    fi
    main_run_dir="$(dirname "${run_summaries[0]}")"
fi
projection_archive="${PROJECTION_ARCHIVE:-paper/results/projection_deterministic_51_cap_100nmi.tar.gz}"
full_archive="${FULL_ARCHIVE:-paper/results/$(basename "$run_root").tar.gz}"
uv_bin="${UV:-uv}"

mkdir -p "$run_root/logs" "$figure_root" "$application_dir" "$(dirname "$full_archive")"

run_and_log() {
    local name="$1"
    shift
    local log_path="$run_root/logs/${name}.log"
    printf '[%s] starting %s\n' "$(date -Is)" "$name" | tee "$log_path"
    "$@" 2>&1 | tee -a "$log_path"
    printf '[%s] finished %s\n' "$(date -Is)" "$name" | tee -a "$log_path"
}

run_and_log render_main_figures \
    "$uv_bin" run python -m paper.run_evaluation \
        --render-figures-from "$main_run_dir" \
        --figure-output-dir "$figure_root/main" \
        --output-dir "$run_root/render_work/main" \
        --figure-formats pdf

run_and_log render_application_figures \
    "$uv_bin" run python -m paper.render_lateral_overlap_figure \
        --output-pdf "$application_dir/fig_lateral_overlap_schematic.pdf"

# The submitted figure uses a 600x600 grid; lower CLEARANCE_GRID_DENSITY for quick previews.
run_and_log render_clearance_grid \
    "$uv_bin" run python -m paper.render_clearance_grid_figure \
        --output-pdf "$application_dir/fig_clearance_grid.pdf" \
        --source-json "$application_dir/fig_clearance_grid_source.json.gz" \
        --grid-density "${CLEARANCE_GRID_DENSITY:-600}" \
        --write-source-json

run_and_log render_projection_figures \
    "$uv_bin" run python -m paper.plot_projection_sweep \
        --input-dir "$run_root/projection_full" \
        --output-dir "$figure_root/projection_deterministic_51_cap_100nmi" \
        --max-excursion-nmi 100 \
        --write-rows \
        --combined-rows \
        --archive-output "$projection_archive"

rm -f "$figure_root/projection_deterministic_51_cap_100nmi/projection_rows.csv"

run_and_log archive_full_run \
    tar -czf "$full_archive" -C "$(dirname "$run_root")" "$(basename "$run_root")"

printf 'Rendered figures from %s into %s\n' "$run_root" "$figure_root"
printf 'Archived full run to %s\n' "$full_archive"
