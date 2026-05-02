#!/usr/bin/env bash
set -euo pipefail

run_root="${1:-paper_eval_outputs/paper_full_$(date +%Y%m%d_%H%M%S)}"
n_jobs="${N_JOBS:-128}"
profile="${PAPER_PROFILE:-large100k}"
repeat_seeds="${REPEAT_SEEDS:-5}"
quick="${QUICK:-0}"
uv_bin="${UV:-uv}"

mkdir -p "$run_root/logs" paper_eval_outputs
printf '%s\n' "$run_root" > paper_eval_outputs/latest_paper_run.txt

run_and_log() {
    local name="$1"
    shift
    local log_path="$run_root/logs/${name}.log"
    printf '[%s] starting %s\n' "$(date -Is)" "$name" | tee "$log_path"
    "$@" 2>&1 | tee -a "$log_path"
    printf '[%s] finished %s\n' "$(date -Is)" "$name" | tee -a "$log_path"
}

main_args=(
    "$uv_bin" run python -m paper.run_evaluation
    --n-jobs "$n_jobs"
    --output-dir "$run_root/main_evaluation"
    --row-output-mode none
    --skip-plots
)
if [[ "$quick" == "1" ]]; then
    main_args+=(--quick --repeat-seeds "${REPEAT_SEEDS:-1}")
else
    main_args+=(--profile "$profile" --repeat-seeds "$repeat_seeds")
fi

run_and_log main_evaluation "${main_args[@]}"

projection_args=(
    "$uv_bin" run python -m paper.run_projection_sweep
    --n-jobs "$n_jobs"
    --output-dir "$run_root/projection_full"
    --combined-rows
    --compress-rows
)
if [[ "$quick" == "1" || "${PROJECTION_QUICK:-0}" == "1" ]]; then
    projection_args+=(--quick)
fi

run_and_log projection_sweep "${projection_args[@]}"

printf 'Saved data-only paper run to %s\n' "$run_root"
