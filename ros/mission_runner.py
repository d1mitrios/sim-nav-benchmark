#!/usr/bin/env python3
# === Mission runner, experiments 4-8 (WSL side) ===
# Usage:  source /opt/ros/jazzy/setup.bash && python3 mission_runner.py <seed> [timeout_s]
#             [--sim-budget S] [--config NAME] [--map slam|truth] [--params-md5 HEX]
# Reads the world manifest (world_<seed>.csv), builds a DETERMINISTIC tour of
# 4 goals: centers of the 3 other quadrants (adjacent-x, adjacent-y, diagonal) and
# return to the spawn (0,0). Sends NavigateToPose serially, with a timeout per mission,
# retrying once on every failure (a person in the way often moves on, and each attempt is
# a row in the CSV with an attempt column). Writes missions_<seed>_<ts>.csv: goal, result,
# wall times (for the offline join with the metrics CSV and ground truth). Runs with the
# system python (not the ultralytics venv of the navigator) and needs Nav2 active with
# the map of the same seed.
# Experiments 4 and 5 ran with a wall-clock timeout (timeout_s). The simulator's real-time
# factor drifted between 0.15 and 0.5 during those batches, so a wall-clock timeout gave each
# attempt between about 36 and 120 simulated seconds. From experiment 6 on, --sim-budget S
# measures the timeout and the retry pause in simulated seconds instead, read from the metrics
# CSV that Isaac is writing for this world (run_<seed>_<ts>.csv, line-buffered). The same file
# gives every row its simulated start and end time, so the ground-truth join no longer has to
# estimate the real-time factor. A second file, amcl_<seed>_<ts>.csv, keeps every /amcl_pose
# estimate with its wall and simulated time, which makes the localization error measurable.
# A third, cmdvel_<seed>_<ts>.csv, samples the velocity command at the three stages of the
# Nav2 chain (controller, smoother, collision monitor) twice a second, so that a robot that
# stands still can be told apart: no command, a command cut on the way, or a command the
# simulated robot does not execute.
import argparse
import csv
import glob
import math
import os
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

RUNS = "/mnt/c/isaac_project/runs"
STATUS_NAMES = {4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}
RETRY_WAIT_S = 20.0   # ~10 sim-s at RTF 0.5, enough for a walker to move on
RETRY_WAIT_SIM = 10.0  # the same pause in simulated seconds (--sim-budget mode)
WALL_CAP_S = 1500.0    # safety cap per attempt in --sim-budget mode (RTF 0.08 at 120 sim-s)


class SimClock:
    """Simulated time of the running world, from the last line of its metrics CSV."""

    def __init__(self, seed, not_before):
        self.seed = seed
        self.not_before = not_before - 5.0
        self.path = None

    def _find(self):
        fs = [f for f in glob.glob(f"{RUNS}/run_{self.seed}_*.csv")
              if os.path.getmtime(f) >= self.not_before]
        self.path = max(fs, key=os.path.getmtime) if fs else None

    def now(self):
        """Last logged sim_t, or None when no metrics file of this world is being written."""
        if self.path is None:
            self._find()
            if self.path is None:
                return None
        try:
            with open(self.path, "rb") as f:
                f.seek(0, os.SEEK_END)
                f.seek(max(0, f.tell() - 400))
                lines = f.read().decode(errors="ignore").strip().splitlines()
            for line in reversed(lines):
                try:
                    return float(line.split(",")[0])
                except ValueError:
                    continue
        except OSError:
            pass
        return None


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
    land on furniture, as in world 025). Deterministic spiral,
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
    def __init__(self, clock=None, amcl_csv=None, cmd_csv=None):
        super().__init__("mission_runner")
        self.cli = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.clock = clock
        self.amcl_w = None
        if amcl_csv:
            self.amcl_f = open(amcl_csv, "w", newline="")
            self.amcl_w = csv.writer(self.amcl_f)
            self.amcl_w.writerow(["epoch", "sim_t", "x", "y", "yaw_deg", "cov_xx", "cov_yy", "cov_aa"])
            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE)
            self.create_subscription(PoseWithCovarianceStamped, "amcl_pose", self.on_amcl, qos)
        self.cmd_w = None
        if cmd_csv:
            self.cmd_f = open(cmd_csv, "w", newline="")
            self.cmd_w = csv.writer(self.cmd_f)
            self.cmd_w.writerow(["epoch", "sim_t", "stage", "vx", "wz"])
            self.cmd_last = {}
            for stage, topic in (("controller", "cmd_vel_nav"), ("smoother", "cmd_vel_smoothed"),
                                 ("monitor", "cmd_vel")):
                self.create_subscription(Twist, topic,
                                         lambda m, s=stage: self.on_cmd(s, m), 10)

    def on_cmd(self, stage, msg):
        now = time.time()
        vx, wz = msg.linear.x, msg.angular.z
        last = self.cmd_last.get(stage)
        # twice a second, and at once when the command switches between moving and stopped
        moving = abs(vx) > 1e-3 or abs(wz) > 1e-3
        if last and now - last[0] < 0.5 and moving == last[1]:
            return
        self.cmd_last[stage] = (now, moving)
        s = self.clock.now() if self.clock else None
        self.cmd_w.writerow([f"{now:.2f}", "" if s is None else f"{s:.2f}", stage,
                             f"{vx:.3f}", f"{wz:.3f}"])
        self.cmd_f.flush()

    def on_amcl(self, msg):
        p, q, c = msg.pose.pose.position, msg.pose.pose.orientation, msg.pose.covariance
        yaw = math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
        s = self.clock.now() if self.clock else None
        stamp = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec   # wall clock (no /clock)
        self.amcl_w.writerow([f"{stamp:.2f}", "" if s is None else f"{s:.2f}",
                              f"{p.x:.3f}", f"{p.y:.3f}", f"{yaw:.1f}",
                              f"{c[0]:.4f}", f"{c[7]:.4f}", f"{c[35]:.4f}"])
        self.amcl_f.flush()

    def sim_now(self):
        return self.clock.now() if self.clock else None

    def pause(self, wall_s=0.0, sim_s=0.0, wall_cap=0.0):
        """Wait while still serving the /amcl_pose callback: wall_s wall seconds, or sim_s
        simulated seconds (bounded by wall_cap) when the simulated clock is available."""
        t0, s0 = time.time(), self.sim_now() if sim_s else None
        while True:
            if s0 is not None:
                s = self.sim_now()
                if (s is not None and s - s0 >= sim_s) or time.time() - t0 >= wall_cap:
                    return
            elif time.time() - t0 >= wall_s:
                return
            rclpy.spin_once(self, timeout_sec=0.2)

    def run_one(self, gx, gy, timeout_s, sim_budget=None):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x = float(gx)
        goal.pose.pose.position.y = float(gy)
        goal.pose.pose.orientation.w = 1.0
        if not self.cli.wait_for_server(timeout_sec=10.0):
            return "NO_SERVER", 0.0
        t0 = time.time()
        # a rejection at send time is a lifecycle state (bt_navigator not yet
        # active), NOT a navigation result - retry internally up to 60s without
        # consuming an attempt (a test run saw 8 of 8 goals rejected at 0.0 s).
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
        s0 = self.sim_now() if sim_budget else None
        while True:
            if s0 is not None:
                s = self.sim_now()
                if (s is not None and s - s0 >= sim_budget) or time.time() - t0 >= WALL_CAP_S:
                    break
            elif time.time() - t0 >= timeout_s:
                break
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
    ap = argparse.ArgumentParser(description="Nav2 four-goal tour of one world")
    ap.add_argument("seed")
    ap.add_argument("timeout_s", nargs="?", type=float, default=300.0,
                    help="wall-clock timeout per attempt (experiments 4-5)")
    ap.add_argument("--sim-budget", type=float, default=None,
                    help="timeout per attempt in simulated seconds (experiment 6 on)")
    ap.add_argument("--config", default="", help="navigation configuration, written to every row")
    ap.add_argument("--map", default="", help="map kind (slam or truth), written to every row")
    ap.add_argument("--params-md5", default="", help="first 8 hex digits of the parameter file's md5")
    a = ap.parse_args()
    seed, timeout_s = a.seed, a.timeout_s
    pxw, pyw, obs = load_walls(seed)
    tour = [(nm, *adjust_goal(gx, gy, obs, pxw, pyw))
            for nm, gx, gy in build_tour(pxw, pyw)]
    rclpy.init()
    stamp = time.strftime('%Y%m%d_%H%M%S')
    clock = SimClock(seed, time.time() - 3600.0) if a.sim_budget else None
    if clock and clock.now() is None:
        print("[missions] WARN: no metrics CSV of this world found - falling back to wall-clock "
              f"timeouts of {timeout_s:.0f}s", flush=True)
        clock = None
    node = MissionRunner(clock, f"{RUNS}/amcl_{seed}_{stamp}.csv" if a.sim_budget else None,
                         f"{RUNS}/cmdvel_{seed}_{stamp}.csv" if a.sim_budget else None)
    sim_budget = a.sim_budget if clock else None
    out = f"{RUNS}/missions_{seed}_{stamp}.csv"
    f = open(out, "w", newline="")
    w = csv.writer(f)
    header = ["mission", "goal_x", "goal_y", "attempt", "result", "wall_s",
              "epoch_start", "epoch_end"]
    if a.sim_budget:
        header += ["sim_start", "sim_end", "config", "map", "params_md5"]
    w.writerow(header)
    f.flush()
    budget = f"{sim_budget:.0f} sim-s" if sim_budget else f"{timeout_s:.0f}s"
    print(f"[missions] world {seed} (walls x={pxw:.2f}, y={pyw:.2f}) - "
          f"{len(tour)} goals, timeout {budget}, retry-once on failure"
          + (f", config {a.config}, map {a.map}" if a.config else ""))
    ok = first_try = 0
    for name, gx, gy in tour:
        result = None
        for attempt in (1, 2):
            print(f"[missions] -> {name} (attempt {attempt}): ({gx:.2f}, {gy:.2f}) ...",
                  flush=True)
            e0, s0 = time.time(), node.sim_now()
            result, dur = node.run_one(gx, gy, timeout_s, sim_budget)
            e1, s1 = time.time(), node.sim_now()
            print(f"[missions]    {result} in {dur:.0f}s"
                  + (f" ({s1 - s0:.0f} sim-s)" if s0 is not None and s1 is not None else ""))
            row = [name, f"{gx:.2f}", f"{gy:.2f}", attempt, result, f"{dur:.1f}",
                   f"{e0:.1f}", f"{e1:.1f}"]
            if a.sim_budget:
                row += ["" if s0 is None else f"{s0:.2f}", "" if s1 is None else f"{s1:.2f}",
                        a.config, a.map, a.params_md5]
            w.writerow(row)
            f.flush()
            if result == "SUCCEEDED":
                first_try += attempt == 1
                break
            if attempt == 1:
                # retry once: a person in the way often moves on while the walkers
                # continue their loop, while a correct refusal of an impassable door will
                # simply fail a second time too (stronger negative evidence).
                if clock:
                    print(f"[missions]    retry in {RETRY_WAIT_SIM:.0f} sim-s ...", flush=True)
                    node.pause(wall_s=RETRY_WAIT_S, sim_s=RETRY_WAIT_SIM, wall_cap=10.0 * RETRY_WAIT_S)
                else:
                    print(f"[missions]    retry in {RETRY_WAIT_S:.0f}s ...", flush=True)
                    node.pause(wall_s=RETRY_WAIT_S)
        ok += result == "SUCCEEDED"
        node.pause(wall_s=3.0)   # short breather between missions
    f.close()
    print(f"[missions] DONE: {ok}/{len(tour)} SUCCEEDED "
          f"({first_try} first-try, {ok - first_try} with retry) - log: {out}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
