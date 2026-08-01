#!/usr/bin/env bash
# === Mapping-batch orchestrator (WSL) — Phase 3, Step 1 ===
# Runs ALONGSIDE batch_runner v4 (Isaac): for each new world (change of
# current_seed.txt) it starts a fresh slam_toolbox; when the save_map_now.txt
# signal appears (the runner writes it 30s before STOP), it saves the map as
# maps/world_<seed>.pgm/.yaml, copies it to C:\, deletes the signal (=ACK
# to the runner) and kills slam so it starts clean on the next world.
# Start BEFORE pressing Run on the batch_runner. Stop: Ctrl+C.
source /opt/ros/jazzy/setup.bash
RUNS=/mnt/c/isaac_project/runs
MAPS=$HOME/robot_project/maps
PARAMS=$HOME/robot_project/slam_params.yaml
mkdir -p "$MAPS"
rm -f "$RUNS/save_map_now.txt"
last=""
echo "[slam-batch] orchestrator up — waiting for worlds (Ctrl+C to stop)"
while true; do
  seed=$(cat "$RUNS/current_seed.txt" 2>/dev/null)
  if [ -n "$seed" ] && [ "$seed" != "$last" ]; then
    last="$seed"
    echo "[slam-batch] world $seed — slam starting in 10s"
    sleep 10
    setsid ros2 launch slam_toolbox online_async_launch.py \
        slam_params_file:="$PARAMS" > "$MAPS/slam_$seed.log" 2>&1 &
    SLAM=$!
    echo "[slam-batch] mapping world $seed (slam pid $SLAM)"
    while true; do
      if [ -f "$RUNS/save_map_now.txt" ]; then
        s=$(cat "$RUNS/save_map_now.txt")
        echo "[slam-batch] save signal ($s)"
        ros2 run nav2_map_server map_saver_cli -f "$MAPS/world_${s}" \
            --ros-args -p save_map_timeout:=10.0 >> "$MAPS/slam_$seed.log" 2>&1 \
          && echo "[slam-batch] saved world_${s}" \
          || echo "[slam-batch] WARN: map_saver failed for $s"
        cp "$MAPS/world_${s}".pgm "$MAPS/world_${s}".yaml "$RUNS/" 2>/dev/null
        rm -f "$RUNS/save_map_now.txt"
        break
      fi
      now=$(cat "$RUNS/current_seed.txt" 2>/dev/null)
      if [ "$now" != "$seed" ]; then
        echo "[slam-batch] WARN: seed changed without save signal"
        break
      fi
      sleep 2
    done
    echo "[slam-batch] restarting slam"
    # v2: escalating kill on the WHOLE process group (ros2 launch often ignores a bare INT)
    kill -INT -- -$SLAM 2>/dev/null
    for _ in 1 2 3 4 5 6 7 8; do kill -0 $SLAM 2>/dev/null || break; sleep 1; done
    kill -TERM -- -$SLAM 2>/dev/null
    for _ in 1 2 3 4; do kill -0 $SLAM 2>/dev/null || break; sleep 1; done
    kill -KILL -- -$SLAM 2>/dev/null
    wait $SLAM 2>/dev/null
    pkill -9 -f async_slam_toolbox_node 2>/dev/null   # orphans = they hold /map
    echo "[slam-batch] slam down"
  fi
  sleep 2
done
