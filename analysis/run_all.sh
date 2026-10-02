#!/usr/bin/env bash
# Regenerate every result file and figure from the data in ../runs/.
# Needs Python 3 with numpy, scipy, matplotlib (see ../requirements.txt).
# Usage: bash analysis/run_all.sh
# The printed output of each script is kept in analysis/output/<script>.txt.
set -e
cd "$(dirname "$0")"
export MPLBACKEND=Agg
mkdir -p output
run() {
    local name="${1%.py}"
    [ $# -gt 1 ] && name="${name}_${2##*/}" && name="${name%.yaml}"
    echo "== $*"
    python3 "$@" > "output/${name}.txt"
}

# Early exploration
run plot_run.py
run plot_run2.py

# Experiment 1: reactive navigation, v1 baseline vs v2
run analyze_batch.py
run compare_v1_v2.py            # -> runs/compare_rows.json
run plot_compare.py
run plot_curve.py
run plot_before_after.py

# Experiment 2: walking people
run analyze_dynamic.py          # -> runs/dynamic_rows.json
run diagnose_kills.py
run plot_cost_of_dynamics.py
run plot_dynamic.py
run analyze_batch2.py           # -> runs/batch2_rows.json
run plot_phase2_final.py
run analyze_xroom.py            # -> runs/xroom_rows.json
run plot_xroom.py

# Experiment 3: mapping while people walk
run analyze_slam_batch.py       # -> runs/slam_batch_rows.json
run plot_slam_batch.py
run compare_maps.py
run plot_ablation.py
run evaluate_map.py ../runs/world_659927.yaml ../runs/world_659927.csv ../runs/map_eval_659927.png

# Experiments 4 and 5: missions and the paired AMCL study
run predict_feasibility.py      # -> runs/missions_feasibility.json
run analyze_missions.py A       # -> runs/missions_results_v8.json
run analyze_missions.py B       # -> runs/missions_results_v9.json
run verify_collapse.py
run plot_paired_missions.py

echo "All figures regenerated in runs/."
