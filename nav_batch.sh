#!/usr/bin/env bash
# === Nav2 mission-batch orchestrator v4 (WSL) — Phase 3, Step 2 ===
# v3 -> v4 (lesson, 30/7, runs 10-25 lost): after ~9 bringup+kill cycles the
# ros2 CLI crawled and the gates went blind (017: bt=0 pose=0, durations 1291s
# from CLI latency). Cause: FastDDS shared-memory segments piling up in /dev/shm
# from the kill -9s + a ros2 daemon that bogs down. v4 hygiene PER CYCLE:
#  - more patient kill (INT 15s, TERM 8s) so the -9 stays rare
#  - after the kill: ros2 daemon stop + rm /dev/shm/fastrtps_* + fast_datasharing_*
#  - all ros2 CLI probes with a timeout (none can crawl forever)
#  - mission_runner with timeout 2400s (never blocks the batch < 45' cap)
# Trigger: nav_go.txt (batch_runner v5.1). Start whenever. Stop: Ctrl+C.
source /opt/ros/jazzy/setup.bash
RUNS=/mnt/c/isaac_project/runs
MAPS=$HOME/robot_project/maps
PARAMS=/mnt/c/isaac_project/nav2_params.yaml
RUNNER=/mnt/c/isaac_project/mission_runner.py
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

echo "[nav-batch] v4 up — waiting for nav_go.txt signals (Ctrl+C to stop)"
dds_hygiene   # clean start (junk piled up from the morning batch)
while true; do
  if [ -f "$GO" ]; then
    seed=$(tr -d ' \r\n' < "$GO")
    rm -f "$GO"
    if [ -z "$seed" ]; then
      echo "[nav-batch] WARN: empty nav_go signal — ignoring"
      continue
    fi
    if [ ! -f "$MAPS/world_${seed}.yaml" ]; then
      echo "[nav-batch] world $seed: no map — skipping"
      echo NO_MAP > "$RUNS/missions_done_${seed}.txt"
      continue
    fi
    echo "[nav-batch] $(date +%H:%M:%S) GO world $seed — nav2 starting in 10s"
    sleep 10
    pkill -9 -f "component_container_isolated|nav2_container|lifecycle_manager" 2>/dev/null
    setsid ros2 launch nav2_bringup bringup_launch.py \
        map:="$MAPS/world_${seed}.yaml" params_file:="$PARAMS" \
        use_sim_time:=false autostart:=true > "$LOGS/nav_${seed}.log" 2>&1 &
    NAV=$!
    # gate 1: bt_navigator ACTIVE — every probe bounded by a 6s timeout
    up=0
    for _ in $(seq 1 24); do
      if timeout 6 ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q "^active"; then up=1; break; fi
      sleep 4
    done
    # gate 2: first /amcl_pose = amcl active AND scans flowing (sim alive)
    pose=0
    if [ "$up" = 1 ]; then
      echo "[nav-batch] $(date +%H:%M:%S) bt_navigator active — waiting for /amcl_pose"
      for _ in $(seq 1 30); do
        if timeout 8 ros2 topic echo --once /amcl_pose >/dev/null 2>&1; then pose=1; break; fi
      done
    fi
    if [ "$pose" = 1 ]; then
      echo "[nav-batch] $(date +%H:%M:%S) nav2 ready + scans flowing — starting the tour"
      timeout 2400 /usr/bin/python3 "$RUNNER" "$seed" "$TIMEOUT_S"
      [ $? -eq 124 ] && echo "[nav-batch] WARN: mission_runner timed out (2400s)"
    else
      echo "[nav-batch] $(date +%H:%M:%S) WARN: world $seed not ready (bt=$up pose=$pose) — skipping tour"
      echo "NOT_READY bt=$up pose=$pose" > "$RUNS/missions_skip_${seed}.txt"
    fi
    echo done > "$RUNS/missions_done_${seed}.txt"
    echo "[nav-batch] $(date +%H:%M:%S) world $seed finished — taking nav2 down"
    kill -INT -- -$NAV 2>/dev/null
    for _ in $(seq 1 15); do kill -0 $NAV 2>/dev/null || break; sleep 1; done
    kill -TERM -- -$NAV 2>/dev/null
    for _ in $(seq 1 8); do kill -0 $NAV 2>/dev/null || break; sleep 1; done
    kill -KILL -- -$NAV 2>/dev/null
    wait $NAV 2>/dev/null
    pkill -9 -f "component_container_isolated|nav2_container|lifecycle_manager" 2>/dev/null
    dds_hygiene   # v4: the key — clean DDS before the next world
    echo "[nav-batch] $(date +%H:%M:%S) nav2 down + dds clean — waiting for next nav_go"
  fi
  sleep 2
done
