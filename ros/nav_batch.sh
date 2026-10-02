#!/usr/bin/env bash
# === Nav2 mission-batch orchestrator (WSL), experiments 4 and 5 ===
# For each world signalled by batch_runner.py (nav_go.txt), brings Nav2 up on the saved map,
# waits for two readiness gates (bt_navigator active, first /amcl_pose), runs the
# mission_runner tour, writes missions_done_<seed>.txt and takes Nav2 down.
# DDS hygiene per cycle: repeated bring-up and kill cycles left FastDDS shared-memory
# segments in /dev/shm and a slow ros2 daemon, until the readiness probes timed out. So:
#  - patient kill (INT 15 s, TERM 8 s) so the -9 stays rare
#  - after the kill: ros2 daemon stop + rm /dev/shm/fastrtps_* + fast_datasharing_*
#  - every ros2 CLI probe runs under a timeout
#  - mission_runner under a 2400 s timeout (below the 45 min cap of batch_runner)
# Start whenever, before or during the Isaac batch. Stop: Ctrl+C.
source /opt/ros/jazzy/setup.bash
RUNS=/mnt/c/isaac_project/runs
MAPS=$HOME/robot_project/maps
PARAMS=/mnt/c/isaac_project/ros/nav2_params.yaml
RUNNER=/mnt/c/isaac_project/ros/mission_runner.py
LOGS=$HOME/robot_project/navlogs
GO=$RUNS/nav_go.txt
TIMEOUT_S=240
mkdir -p "$LOGS"
rm -f "$RUNS"/missions_done_*.txt

dds_hygiene() {
  ros2 daemon stop >/dev/null 2>&1
  rm -f /dev/shm/fastrtps_* /dev/shm/fast_datasharing_* 2>/dev/null
  sleep 2
}

echo "[nav-batch] v4 up - waiting for nav_go.txt signals (Ctrl+C to stop)"
dds_hygiene   # clean start (leftovers of earlier runs)
while true; do
  if [ -f "$GO" ]; then
    seed=$(tr -d ' \r\n' < "$GO")
    rm -f "$GO"
    if [ -z "$seed" ]; then
      echo "[nav-batch] WARN: empty nav_go signal - ignoring"
      continue
    fi
    if [ ! -f "$MAPS/world_${seed}.yaml" ]; then
      echo "[nav-batch] world $seed: no map - skipping"
      echo NO_MAP > "$RUNS/missions_done_${seed}.txt"
      continue
    fi
    echo "[nav-batch] $(date +%H:%M:%S) GO world $seed - nav2 starting in 10s"
    sleep 10
    pkill -9 -f "component_container_isolated|nav2_container|lifecycle_manager" 2>/dev/null
    setsid ros2 launch nav2_bringup bringup_launch.py \
        map:="$MAPS/world_${seed}.yaml" params_file:="$PARAMS" \
        use_sim_time:=false autostart:=true > "$LOGS/nav_${seed}.log" 2>&1 &
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
      timeout 2400 /usr/bin/python3 "$RUNNER" "$seed" "$TIMEOUT_S"
      [ $? -eq 124 ] && echo "[nav-batch] WARN: mission_runner timed out (2400s)"
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
    pkill -9 -f "component_container_isolated|nav2_container|lifecycle_manager" 2>/dev/null
    dds_hygiene   # clean DDS before the next world
    echo "[nav-batch] $(date +%H:%M:%S) nav2 down + dds clean - waiting for next nav_go"
  fi
  sleep 2
done
