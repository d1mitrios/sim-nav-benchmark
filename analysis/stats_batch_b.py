#!/usr/bin/env python3
# Batch B (October 2026): held-out worlds, replicates and the leave-one-out for change 7a.
# Usage: python3 stats_batch_b.py
# All runs on ground-truth maps with 120 simulated seconds per attempt, judged as in
# stats_missions.py (real arrival = true position within 0.6 m of the goal). Arms:
#   rev_heldout, no7a_heldout   revised configuration and its leave-one-out (nav2_params_no7a.yaml,
#                               AMCL back on base_link) on 10 worlds never used before
#   rev_repeat, no7a_repeat     the same two on 8 benchmark worlds run again: the three that were
#                               not OK in experiment 8 (012, 005, 015) and five OK ones drawn at random
# Paired comparisons by world: revised against the leave-one-out (held-out, repeat, all 18
# worlds), and the repeat against experiment 8 on the same 8 worlds (test-retest).
# Writes runs/missions_batch_b.json.
import os
import sys
import json

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import analyze_missions2 as am
from stats_missions import ARMS, run_arm, summarize, compare

HELDOUT = [str(20261015000 + k) for k in range(1, 11)]
REPEAT = [str(20260723000 + k) for k in (12, 5, 15, 1, 2, 8, 18, 19)]
REV_MD5, NO7A_MD5 = "9f696af1", "a5e38846"
B_ARMS = {
    "rev_heldout": ("nav2_params", "truth", "20261015", None, REV_MD5, HELDOUT),
    "no7a_heldout": ("nav2_params_no7a", "truth", "20261015", None, NO7A_MD5, HELDOUT),
    "rev_repeat": ("nav2_params", "truth", "20261015", None, REV_MD5, REPEAT),
    "no7a_repeat": ("nav2_params_no7a", "truth", "20261015", None, NO7A_MD5, REPEAT),
}


def main():
    arms = {name: run_arm(*spec) for name, spec in B_ARMS.items()}
    cfg, mk, d0, d1, md5 = ARMS["rev_truth"]
    arms["exp8_repeat"] = run_arm(cfg, mk, d0, d1, md5, REPEAT)
    arms["rev_all"] = {**arms["rev_heldout"], **arms["rev_repeat"]}
    arms["no7a_all"] = {**arms["no7a_heldout"], **arms["no7a_repeat"]}
    print()
    summary = {name: summarize(name, res) for name, res in arms.items() if res}
    print()
    comps = {}
    for a, b, label in [("rev_heldout", "no7a_heldout", "change 7a, held-out worlds"),
                        ("rev_repeat", "no7a_repeat", "change 7a, repeated worlds"),
                        ("rev_all", "no7a_all", "change 7a, all 18 worlds"),
                        ("rev_repeat", "exp8_repeat", "test-retest, revised, same 8 worlds")]:
        c = compare(a, b, label, arms)
        if c:
            comps[f"{a}_vs_{b}"] = c
    json.dump({"arms": summary, "comparisons": comps},
              open(os.path.join(am.RUNS, "missions_batch_b.json"), "w"), indent=1)
    print("\nresults -> runs/missions_batch_b.json")


if __name__ == "__main__":
    main()
