#!/usr/bin/env python3
# === Mission runner — Phase 3 Step 2 (WSL side) ===
# Usage:  source /opt/ros/jazzy/setup.bash && python3 mission_runner.py <seed> [timeout_s]
# Reads the world manifest (world_<seed>.csv), builds a DETERMINISTIC tour of
# 4 goals: centers of the 3 other quadrants (adjacent-x, adjacent-y, diagonal) and
# return to the spawn (0,0). Sends NavigateToPose serially, with a timeout per mission,
# v3: RETRY-ONCE on every failure (human-blocks clear up; each attempt = a row
# in the CSV with an attempt column). Writes missions_<seed>_<ts>.csv: goal, result, wall
# times (for offline join with the metrics CSV / ground truth). CAUTION: runs with
# system python (NOT ai_env), needs Nav2 active with the map of the SAME seed.
import csv
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose

RUNS = "/mnt/c/isaac_project/runs"
STATUS_NAMES = {4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}
RETRY_WAIT_S = 20.0   # v3: ~10 sim-s at RTF 0.5 — enough for a walker to move on


def load_walls(seed):
    pxw = pyw = None
    obs = []
    for line in open(f"{RUNS}/world_{seed}.csv"):
        p = line.strip().split(",")
        if p[0] == "door" and p[5] == "x":
            pxw = float(p[2])
        elif p[0] == "door" and p[5] == "y":
            pyw = float(p[3])
        elif p[0] == "box" and not p[1].startswith("partition"):
            obs.append((float(p[2]), float(p[3]),
                        0.5 * math.hypot(float(p[4]), float(p[5]))))
        elif p[0] == "cyl":
            obs.append((float(p[2]), float(p[3]), float(p[4])))
    if pxw is None or pyw is None:
        raise SystemExit(f"manifest without quad doors: world_{seed}.csv")
    return pxw, pyw, obs


def adjust_goal(gx, gy, obs, pxw, pyw):
    """v2: shift to the nearest CLEAR point (quadrant centers sometimes
    land on furniture — lesson from 025). Deterministic spiral,
    stays in the same quadrant, ≥0.6 m from obstacles, ≥0.9 m from walls."""
    sx, sy = gx > pxw, gy > pyw
    for r in [0.0, 0.3, 0.6, 0.9, 1.2, 1.5, 1.8, 2.1, 2.4, 2.7, 3.0]:
        for k in range(12):
            a = 2.0 * math.pi * k / 12.0
            x, y = gx + r * math.cos(a), gy + r * math.sin(a)
            if abs(x) > 8.6 or abs(y) > 8.6:
                continue
            if (x > pxw) != sx or (y > pyw) != sy:
                continue
            if abs(x - pxw) < 0.9 or abs(y - pyw) < 0.9:
                continue
            if all(math.hypot(x - ox, y - oy) - orr >= 0.6 for ox, oy, orr in obs):
                return x, y
    return gx, gy


def build_tour(pxw, pyw):
    xE, xW = (pxw + 10.0) / 2.0, (pxw - 10.0) / 2.0
    yN, yS = (pyw + 10.0) / 2.0, (pyw - 10.0) / 2.0
    ex = "W" if 0.0 < pxw else "E"          # x-side of the spawn (0,0)
    ey = "S" if 0.0 < pyw else "N"          # y-side of the spawn
    other_x = "E" if ex == "W" else "W"
    other_y = "N" if ey == "S" else "S"
    cx = {"E": xE, "W": xW}
    cy = {"N": yN, "S": yS}
    tour = [
        ("adjacent-x", cx[other_x], cy[ey]),      # across the vertical wall
        ("adjacent-y", cx[ex], cy[other_y]),      # across the horizontal one
        ("diagonal",   cx[other_x], cy[other_y]), # diagonal quadrant
        ("home",       0.0, 0.0),                 # return to the spawn
    ]
    return tour


class MissionRunner(Node):
    def __init__(self):
        super().__init__("mission_runner")
        self.cli = ActionClient(self, NavigateToPose, "navigate_to_pose")

    def run_one(self, gx, gy, timeout_s):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x = float(gx)
        goal.pose.pose.position.y = float(gy)
        goal.pose.pose.orientation.w = 1.0
        if not self.cli.wait_for_server(timeout_sec=10.0):
            return "NO_SERVER", 0.0
        t0 = time.time()
        # v4: a rejection at SEND is a lifecycle state (bt_navigator not yet
        # active), NOT a navigation result — retry internally up to 60s without
        # consuming an attempt (smoke lesson, 29/7: 8/8 REJECTED in 0.0s).
        gh = None
        while True:
            send = self.cli.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send, timeout_sec=15.0)
            gh = send.result()
            if gh is not None and gh.accepted:
                break
            if time.time() - t0 > 60.0:
                return "REJECTED", time.time() - t0
            print("[missions]    (goal rejected - stack not active yet, retry in 5s)",
                  flush=True)
            time.sleep(5.0)
        res_fut = gh.get_result_async()
        while time.time() - t0 < timeout_s:
            rclpy.spin_once(self, timeout_sec=0.5)
            if res_fut.done():
                status = res_fut.result().status
                return STATUS_NAMES.get(status, f"STATUS_{status}"), time.time() - t0
        # timeout -> cancel
        cancel = gh.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel, timeout_sec=10.0)
        t_end = time.time()
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.5)
            if res_fut.done():
                break
        return "TIMEOUT", t_end - t0


def main():
    if len(sys.argv) < 2:
        raise SystemExit("usage: mission_runner.py <seed> [timeout_s]")
    seed = sys.argv[1]
    timeout_s = float(sys.argv[2]) if len(sys.argv) > 2 else 300.0
    pxw, pyw, obs = load_walls(seed)
    tour = [(nm, *adjust_goal(gx, gy, obs, pxw, pyw))
            for nm, gx, gy in build_tour(pxw, pyw)]
    rclpy.init()
    node = MissionRunner()
    out = f"{RUNS}/missions_{seed}_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    f = open(out, "w", newline="")
    w = csv.writer(f)
    w.writerow(["mission", "goal_x", "goal_y", "attempt", "result", "wall_s",
                "epoch_start", "epoch_end"])
    f.flush()
    print(f"[missions] world {seed} (walls x={pxw:.2f}, y={pyw:.2f}) — "
          f"{len(tour)} goals, timeout {timeout_s:.0f}s, retry-once on failure")
    ok = first_try = 0
    for name, gx, gy in tour:
        result = None
        for attempt in (1, 2):
            print(f"[missions] -> {name} (attempt {attempt}): ({gx:.2f}, {gy:.2f}) ...",
                  flush=True)
            e0 = time.time()
            result, dur = node.run_one(gx, gy, timeout_s)
            e1 = time.time()
            print(f"[missions]    {result} in {dur:.0f}s")
            w.writerow([name, f"{gx:.2f}", f"{gy:.2f}", attempt, result, f"{dur:.1f}",
                        f"{e0:.1f}", f"{e1:.1f}"])
            f.flush()
            if result == "SUCCEEDED":
                first_try += attempt == 1
                break
            if attempt == 1:
                # v3: retry-once — human-blocks often clear up while the walkers
                # continue their loop; a correct refusal of an impassable door will
                # simply fail a second time too (stronger negative evidence).
                print(f"[missions]    retry in {RETRY_WAIT_S:.0f}s ...", flush=True)
                time.sleep(RETRY_WAIT_S)
        ok += result == "SUCCEEDED"
        time.sleep(3.0)   # short breather between missions
    f.close()
    print(f"[missions] DONE: {ok}/{len(tour)} SUCCEEDED "
          f"({first_try} first-try, {ok - first_try} with retry) — log: {out}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
