#!/usr/bin/env bash
# === Nav2 mission-batch orchestrator (WSL), experiments 4 to 8 ===
# Usage:  bash nav_batch.sh                          experiments 4-5 protocol:
#                                                    nav2_params_v9.yaml, SLAM maps, 240 s wall-clock
#                                                    timeout per attempt
#         bash nav_batch.sh <params.yaml> <slam|truth> [<params2.yaml> <slam|truth>]
#                                                    experiment 6 on: the given parameter file, the
#                                                    SLAM map or the ground-truth map of each world,
#                                                    a budget of 120 simulated seconds per attempt
#                                                    (240 s at the intended real-time factor of 0.5)
#                                                    and an AMCL pose log. The configuration name
#                                                    (the parameter file's name) and the map kind
#                                                    are written into every row of the missions CSV.
#                                                    With a second pair, run i of the Isaac batch
#                                                    (batch_runner.py REPEAT = 2) uses pair 1 for
#                                                    odd i and pair 2 for even i: both configurations
#                                                    run back to back in every world.
# For each world signalled by batch_runner.py (nav_go.txt), brings Nav2 up on the saved map,
# waits for two readiness gates (bt_navigator active, first /amcl_pose), runs the
# mission_runner tour, writes missions_done_<seed>.txt and takes Nav2 down.
# DDS hygiene per cycle: repeated bring-up and kill cycles left FastDDS shared-memory
# segments in /dev/shm and a slow ros2 daemon, until the readiness probes timed out. So:
#  - patient kill (INT 15 s, TERM 8 s) so the -9 stays rare
#  - after the kill: ros2 daemon stop + rm /dev/shm/fastrtps_* + fast_datasharing_*
#  - every ros2 CLI probe runs under a timeout
#  - mission_runner under a 2400 s timeout (below the 45 min cap of batch_runner), 7200 s
#    from experiment 6 on (set NAV_MAX_MINUTES = 130 in batch_runner.py for those batches)
# Start whenever, before or during the Isaac batch. Stop: Ctrl+C.
source /opt/ros/jazzy/setup.bash
RUNS=/mnt/c/isaac_project/runs
MAPS=$HOME/robot_project/maps
JOB_PARAMS=("${1:-/mnt/c/isaac_project/ros/nav2_params_v9.yaml}")
JOB_MAPS=("${2:-}")
if [ -n "$3" ]; then JOB_PARAMS+=("$3"); JOB_MAPS+=("${4:-}"); fi
PARAMS=${JOB_PARAMS[0]}
MAPKIND=${JOB_MAPS[0]}
CONFIG=$(basename "$PARAMS" .yaml)
SIM_BUDGET=120
TRUTH=/mnt/c/isaac_project/analysis/make_truth_map.py
RUNNER=/mnt/c/isaac_project/ros/mission_runner.py
LOGS=$HOME/robot_project/navlogs
GO=$RUNS/nav_go.txt
TIMEOUT_S=240
mkdir -p "$LOGS"
for k in "${!JOB_PARAMS[@]}"; do
  m=${JOB_MAPS[$k]}
  if { [ -n "$m" ] && [ "$m" != slam ] && [ "$m" != truth ]; } || { [ "$k" -gt 0 ] && [ -z "$m" ]; } \
     || { [ -n "${JOB_MAPS[0]}" ] && [ -z "$m" ]; }; then
    echo "usage: bash nav_batch.sh [params.yaml slam|truth [params2.yaml slam|truth]]"; exit 1
  fi
  [ -f "${JOB_PARAMS[$k]}" ] || { echo "no parameter file ${JOB_PARAMS[$k]}"; exit 1; }
done
rm -f "$RUNS"/missions_done_*.txt

dds_hygiene() {
  ros2 daemon stop >/dev/null 2>&1
  rm -f /dev/shm/fastrtps_* /dev/shm/fast_datasharing_* 2>/dev/null
  sleep 2
}

if [ -n "$MAPKIND" ]; then
  for k in "${!JOB_PARAMS[@]}"; do
    echo "[nav-batch] v6 up - job $((k + 1)): config $(basename "${JOB_PARAMS[$k]}" .yaml), ${JOB_MAPS[$k]} maps, $SIM_BUDGET sim-s per attempt"
  done
else
  echo "[nav-batch] v5 up - experiments 4-5 protocol (SLAM maps, ${TIMEOUT_S}s wall-clock per attempt)"
fi
echo "[nav-batch] waiting for nav_go.txt signals (Ctrl+C to stop)"
dds_hygiene   # clean start (leftovers of earlier runs)
while true; do
  if [ -f "$GO" ]; then
    read -r seed idx < <(tr -d '\r' < "$GO")
    rm -f "$GO"
    # job of this run: pair 1 for odd run indices, pair 2 for even ones (one pair: always 1)
    k=0
    if [ "${#JOB_PARAMS[@]}" -gt 1 ] && [ -n "$idx" ]; then k=$(( (idx - 1) % ${#JOB_PARAMS[@]} )); fi
    PARAMS=${JOB_PARAMS[$k]}
    MAPKIND=${JOB_MAPS[$k]}
    CONFIG=$(basename "$PARAMS" .yaml)
    if [ -z "$seed" ]; then
      echo "[nav-batch] WARN: empty nav_go signal - ignoring"
      continue
    fi
    MAP="$MAPS/world_${seed}.yaml"
    if [ "$MAPKIND" = truth ]; then
      MAP="$RUNS/truth_${seed}.yaml"
      # rendered every time from the manifest that generate_world.py has just written, so
      # the map always shows the world that is actually running
      /usr/bin/python3 "$TRUTH" "$seed" "$RUNS" >/dev/null 2>&1
    fi
    if [ ! -f "$MAP" ]; then
      echo "[nav-batch] world $seed: no map ($MAP) - skipping"
      echo NO_MAP > "$RUNS/missions_done_${seed}.txt"
      continue
    fi
    LOG="$LOGS/nav_${seed}.log"
    [ -n "$MAPKIND" ] && LOG="$LOGS/nav_${seed}_${CONFIG}_${MAPKIND}_$(date +%Y%m%d_%H%M%S).log"
    echo "[nav-batch] $(date +%H:%M:%S) GO world $seed (run ${idx:-?}, config $CONFIG${MAPKIND:+, $MAPKIND maps}) - nav2 starting in 10s"
    sleep 10
    pkill -9 -f "component_container_isolated|nav2_container|lifecycle_manager" 2>/dev/null
    # frame of the wheel axle, 0.10 m behind the chassis centre (base_link = the lidar): the
    # point a differential drive turns about. The revised configuration (nav2_params.yaml) and its
    # variants refer to it, v9 ignores it.
    TF=""
    if [ -n "$MAPKIND" ]; then
      setsid ros2 run tf2_ros static_transform_publisher --x -0.10 --y 0 --z 0 \
          --frame-id base_link --child-frame-id base_axle > "$LOG.tf" 2>&1 &
      TF=$!
    fi
    setsid ros2 launch nav2_bringup bringup_launch.py \
        map:="$MAP" params_file:="$PARAMS" \
        use_sim_time:=false autostart:=true > "$LOG" 2>&1 &
    NAV=$!
    # gate 1: bt_navigator ACTIVE - every probe bounded by a 6s timeout
    up=0
    for _ in $(seq 1 24); do
      if timeout 6 ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q "^active"; then up=1; break; fi
      sleep 4
    done
    # gate 2: first /amcl_pose = amcl active AND scans flowing (sim alive)
    pose=0
    if [ "$up" = 1 ]; then
      echo "[nav-batch] $(date +%H:%M:%S) bt_navigator active - waiting for /amcl_pose"
      for _ in $(seq 1 30); do
        if timeout 8 ros2 topic echo --once /amcl_pose >/dev/null 2>&1; then pose=1; break; fi
      done
    fi
    if [ "$pose" = 1 ]; then
      echo "[nav-batch] $(date +%H:%M:%S) nav2 ready + scans flowing - starting the tour"
      if [ -n "$MAPKIND" ]; then
        timeout 7200 /usr/bin/python3 "$RUNNER" "$seed" "$TIMEOUT_S" \
            --sim-budget "$SIM_BUDGET" --config "$CONFIG" --map "$MAPKIND" \
            --params-md5 "$(md5sum "$PARAMS" | cut -c1-8)"
        [ $? -eq 124 ] && echo "[nav-batch] WARN: mission_runner timed out (7200s)"
      else
        timeout 2400 /usr/bin/python3 "$RUNNER" "$seed" "$TIMEOUT_S"
        [ $? -eq 124 ] && echo "[nav-batch] WARN: mission_runner timed out (2400s)"
      fi
    else
      echo "[nav-batch] $(date +%H:%M:%S) WARN: world $seed not ready (bt=$up pose=$pose) - skipping tour"
      echo "NOT_READY bt=$up pose=$pose" > "$RUNS/missions_skip_${seed}.txt"
    fi
    echo done > "$RUNS/missions_done_${seed}.txt"
    echo "[nav-batch] $(date +%H:%M:%S) world $seed finished - taking nav2 down"
    kill -INT -- -$NAV 2>/dev/null
    for _ in $(seq 1 15); do kill -0 $NAV 2>/dev/null || break; sleep 1; done
    kill -TERM -- -$NAV 2>/dev/null
    for _ in $(seq 1 8); do kill -0 $NAV 2>/dev/null || break; sleep 1; done
    kill -KILL -- -$NAV 2>/dev/null
    wait $NAV 2>/dev/null
    [ -n "$TF" ] && { kill -INT -- -$TF 2>/dev/null; sleep 1; kill -KILL -- -$TF 2>/dev/null; wait $TF 2>/dev/null; }
    pkill -9 -f "component_container_isolated|nav2_container|lifecycle_manager" 2>/dev/null
    dds_hygiene   # clean DDS before the next world
    echo "[nav-batch] $(date +%H:%M:%S) nav2 down + dds clean - waiting for next nav_go"
  fi
  sleep 2
done
