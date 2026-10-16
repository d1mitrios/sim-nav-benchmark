#!/usr/bin/env bash
# Regenerate every result file and figure from the data in ../runs/.
# Needs Python 3 with numpy, scipy, matplotlib and PyYAML (see ../requirements.txt).
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

# Experiments 6, 7, 8 and 5b (October): configuration x map, 120 simulated seconds per attempt
judge() {   # judge <output name> <analyze_missions2.py arguments>
    local name=$1
    shift
    echo "== analyze_missions2.py $*"
    python3 analyze_missions2.py "$@" > "output/missions2_${name}.txt"
}
judge v9_truth nav2_params_v9 truth 20261012 20261012
judge v9_slam nav2_params_v9 slam 20261014 --md5=b7f0bb38
judge rev_truth nav2_params truth 20261013 20261014 --md5=9f696af1
judge rev_slam nav2_params slam 20261014 --md5=9f696af1
run stats_missions.py           # -> runs/missions_stats.json
run safety_missions.py          # -> runs/missions_safety.json
run unknown_paths.py
run amcl_map_coverage.py       # -> runs/missions_amcl_coverage.json
run failure_mechanisms.py       # -> runs/missions_failures.json
run plot_missions_2x2.py
run plot_failure_mechanisms.py
run plot_amcl_turns.py

# Checks: held-out worlds, repeated worlds, leave-one-out for change 7a
run stats_batch_b.py            # -> runs/missions_batch_b.json
run plot_missions_headline.py

echo "All figures regenerated in runs/."
