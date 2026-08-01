# yolo_navigator_v2.py — v2 of the tuned yolo_navigator.py (the original stays UNTOUCHED)
# Changes vs v1 (all the FTG logic/scoring/hysteresis/inflation SAME):
#  (1) Progress watchdog: |v|<0.05 & |w|<0.3 for >2.5s -> recovery (kills the
#      "asymptotic creep" at STOP_DIST that stuck the robot at 0.011 m/s).
#  (2) Creep-band ban: cruise<0.08 & front<0.6 -> rotate toward open_angle
#      instead of creeping at 1 cm/s.
#  (3) Stuck detection: extra condition "front near STOP_DIST + zero progress".
#  (4) Observability: logs on every mode change + periodic status every 5s.
#  (5) Person braking graded by distance (bbox height as proxy):
#      h>320px (~<3m) -> STOP, h>200 (~<5m) -> up to 0.1, h>120 (~<8m) -> -0.15, else nothing.
#      (v1 braked -0.2 at every distance and had a floor max(0.1,...) that PUSHED forward.)
#  (6) v2.1: Anti-livelock for narrow passages: oscillation detector (>=3 REVERSE/TRAP in 12s) ->
#      COMMIT mode (decisive slow passage if W>=0.55) or ESCAPE_NARROW (turn elsewhere +
#      temporary "no narrow gaps" filter 12s so hysteresis doesn't pull it back in).
#  (8) v2.5: Fix "rotational livelock" in a locked REVERSE (live failure 2026-07-21:
#      40+ minutes v=0, w=±0.8 flip-flop in a wall↔cylinder pocket, R marginally >0.35 so
#      it never becomes TRAP, |w|>0.3 so the watchdog sees nothing, mode unchanged so
#      the oscillation detector never counts):
#      (a) the turn direction LOCKS at REVERSE entry (no flip-flop with L/R noise),
#      (b) reversing is allowed for front<STOP+0.20 with an explicit rear check (the old guard
#          front>=STOP_DIST cancelled reverse EXACTLY in the 0.32-0.44 band where it was needed),
#      (c) continuous REVERSE >6s -> [ESCAPE_POCKET]: same path as ESCAPE_NARROW
#          (3s turn + LEAVE 4-8s + 20s narrow filter).
#  (9) v2.5: [SENSOR_STALE] guard: if no /scan arrives for >2.5s (Isaac freeze on the 2080 Ti,
#      pause, Stop, dead publisher) -> zero velocity + wait, NOT driving on
#      stale data and NOT blind recovery (observed: 3.5+ minutes of frozen
#      L/R/front with the robot "stopped at a person" who was no longer there).
# (10) v2.6: Escape that does not come back (live failure: the escape turned at the wall and
#      then WEDGE re-entered the pocket chasing the "deepest" open_angle — which in
#      pockets is the dead end itself):
#      (a) escape_turn_dir toward the side with lateral space (d_L/d_R instead of open_angle),
#          and on a relapse <30s it is REVERSED — breaks the same-direction loops,
#      (b) while the narrow filter holds (cooldown 20s): WEDGE = move away almost straight
#          (turn ±0.3 only), ROTATE_OPEN turns toward the open side — no depth-seeking.
# (11) v2.7: A fair chance at COMMIT before giving up (live failure: it gave up on a narrow
#      that fit): the 6s REVERSE-escalation FIRST enters COMMIT if there is a live
#      passable gap (W>=0.55, not wedged, <2 recent failures) — continuous REVERSE
#      counted as ONE entry in the osc-detector, so COMMIT never got the chance to
#      trigger and ESCAPE_POCKET "gave up". ESCAPE only if no gap exists.
#      Also: COMMIT crawls at 0.06 m/s when front<0.30 (fewer unfair aborts at 0.15).
# (12) v2.8: [STUCK_PHYSICAL] detector + WIGGLE recovery (live failure, dense world:
#      rear wheel hooked on a box corner, FTG commands forward, zero motion —
#      invisible to ALL the detectors because they watched commands/scan, not displacement).
#      Without odometry, displacement is inferred from the scan: if we command motion
#      (|v|>=0.12 or |w|>=0.5) and the scan stays unchanged (mean |dr|<0.015 m over
#      a 1.2s window) for >3s -> WIGGLE 3s: alternating diagonal pushes
#      forward/backward with opposite turns to unhook, with a phase offset per repeat.
# (13) v2.9: [WALL_FOLLOW] escape for J-pockets (live failure, world 59259 SE corner:
#      ~10s-period loop REVERSE→WEDGE→FTG(pseudo-gap W≈2 at an oblique corner)→ROTATE→REVERSE,
#      outside ALL the detection windows — minutes of dancing in place, net displacement 0.08m):
#      (a) osc-detector window 12s→35s so the slow loops count,
#      (b) 2 escapes in 90s OR 5 REVERSE entries in 60s -> WALL_FOLLOW 25s: follow
#          the NEAR wall at 0.5m distance (P-control, v=0.22) — a topological guarantee
#          of exit from any pocket (a pocket's "deep openings" are deceptive),
#      (c) afterwards: clear the osc/escape memories + hysteresis for a fresh FTG.
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import cv2
import time
from ultralytics import YOLO


class YoloNavigator(Node):
    def __init__(self):
        super().__init__('yolo_navigator')

        self.get_logger().info('v2: loading YOLOv8 for autonomous navigation...')
        self.model = YOLO('yolov8n.pt')

        self.callback_group = ReentrantCallbackGroup()
        self.timer_cb_group = MutuallyExclusiveCallbackGroup()

        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.image_sub = self.create_subscription(
            Image, '/camera', self.image_callback, qos_profile_sensor_data, callback_group=self.callback_group)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data, callback_group=self.callback_group)

        self.control_timer = self.create_timer(0.05, self.control_loop, callback_group=self.timer_cb_group)

        self.person_in_front = False
        self.person_h = 0            # (5) bbox height in px (distance proxy)
        self.SHOW_GUI = False

        self.best_gap_angle = 0.0
        self.best_gap_width = 0.0
        self.best_gap_axis_angle = 0.0
        self.best_gap_is_real_gate = False
        self.is_wedged = False
        self.front_dist = 999.0
        self.rear_dist = 999.0
        self.d_L = 10.0
        self.d_R = 10.0

        self.stuck_start_time = 0.0
        self.is_recovering = False
        self.recovery_start_time = 0.0

        self.prev_best_gap_score = 0.0
        self.prev_best_gap_angle = 0.0

        self.prev_ang = 0.0
        self.reverse_latch = False
        self.escape_until = 0.0
        self.open_angle = 0.0

        self.last_yolo_time = time.time()

        # (1) watchdog + (3) stuck-no-progress + (4) observability
        self.watchdog_start = 0.0
        self.noprog_start = 0.0
        self.mode = 'INIT'
        self.person_level = 0        # 0 none, 1 soft, 2 strong, 3 stop
        self.last_status_log = 0.0

        # (6) anti-livelock for narrow passages
        self.osc_times = []              # timestamps of REVERSE/TRAP entries
        self.commit_until = 0.0          # COMMIT mode active until...
        self.narrow_filter_until = 0.0   # while active: ignore gaps with W < NARROW_W
        self.escape_turn_until = 0.0     # escape turn active until...
        self.escape_turn_dir = 1.0
        self.NARROW_W = 0.55             # minimum "comfortable" width (robot ~0.42 m)
        self.recent_gap_w = 0.0          # best W of the last ~3s (for the COMMIT decision)
        self.recent_gap_t = 0.0
        self.leave_until = 0.0           # LEAVE mode: actively moving away from the pocket
        self.last_escape_t = 0.0
        self.commit_fail_count = 0       # v2.3: failed commits -> two-strikes -> escape
        self.commit_fail_t = 0.0
        self.person_stop_since = 0.0     # v2.4: how long we've been stopped in front of a person
        self.person_bypass = False       # v2.4: "polite bypass" active
        self.reverse_since = 0.0         # v2.5: when we entered continuous REVERSE
        self.reverse_dir = 0.0           # v2.5: locked turn direction in REVERSE
        self.last_scan_time = time.time()  # v2.5: staleness guard (Isaac freeze / lost /scan)
        self.scan_snapshot = None        # v2.8: physical-stuck detector
        self.scan_snapshot_t = 0.0
        self.scan_change = 999.0         # mean |dr| vs the snapshot ~1.2s ago
        self.motion_cmd_since = 0.0      # since when we command motion with no scan change
        self.wiggle_until = 0.0
        self.wiggle_start = 0.0
        self.wiggle_count = 0
        self.wall_follow_until = 0.0     # v2.9: wall-following escape active
        self.wall_follow_side = 1.0      # +1: wall on the left, -1: on the right
        self.escape_times = []           # v2.9: recent escapes (90s window)
        self.reverse_times = []          # v2.9: REVERSE/TRAP entries (60s window)

        self.get_logger().info('v2.9.1: system active! + WALL_FOLLOW + RUN_RESET (fresh state per batch run).')

    # ---------- (4) helpers ----------
    def _set_mode(self, mode, detail=''):
        if mode != self.mode:
            self.mode = mode
            self.get_logger().info(f'[{mode}] {detail}')
            # (6) oscillation bookkeeping: REVERSE/TRAP entries
            if mode in ('REVERSE', 'TRAP'):
                now = time.time()
                # v2.9 (a): 35s window — the slow loops (~10s period) now count
                self.osc_times = [t for t in self.osc_times if now - t < 35.0]
                self.osc_times.append(now)
                # v2.9 (b): 5 entries in 60s -> WALL_FOLLOW directly
                self.reverse_times = [t for t in self.reverse_times if now - t < 60.0]
                self.reverse_times.append(now)
                if len(self.reverse_times) >= 5 and now > self.wall_follow_until:
                    self._start_wall_follow(now, '5x REVERSE/TRAP in 60s')
                    return
                if len(self.osc_times) >= 3 and now > self.commit_until and now > self.escape_turn_until and now > self.leave_until:
                    self.osc_times.clear()
                    if now - self.commit_fail_t > 30.0:
                        self.commit_fail_count = 0
                    # v2.3: commit ONLY with a live gap NOW (not wedged, no stale memory)
                    # and not after 2 failures at the same spot
                    if (not self.is_wedged) and self.best_gap_width >= self.NARROW_W and self.commit_fail_count < 2:
                        self.commit_until = now + 15.0
                        self.get_logger().info(
                            f'[COMMIT] oscillating at a passable narrow (W={self.best_gap_width:.2f}) -> decisive pass')
                    else:
                        repeat = (now - self.last_escape_t) < 30.0
                        leave_dur = 8.0 if repeat else 4.0
                        self.last_escape_t = now
                        self.narrow_filter_until = now + 20.0
                        self.escape_turn_until = now + 3.0
                        self.leave_until = now + 3.0 + leave_dur
                        # v2.6: direction toward the side with space; on a relapse, reversed
                        if repeat and self.escape_turn_dir != 0.0:
                            self.escape_turn_dir = -self.escape_turn_dir
                        else:
                            self.escape_turn_dir = 1.0 if self.d_L > self.d_R else -1.0
                        self.prev_best_gap_score = 0.0   # erase the hysteresis memory
                        self.get_logger().info(
                            f'[ESCAPE_NARROW] narrow/dead end (W={self.best_gap_width:.2f} wedged={self.is_wedged} '
                            f'failedCommits={self.commit_fail_count}) -> leaving '
                            f'(leave={leave_dur:.0f}s{", relapse" if repeat else ""}), narrow filter 20s')
                        # v2.9 (b): second escape within 90s = the pocket beats the escapes -> WALL_FOLLOW
                        self.escape_times = [t for t in self.escape_times if now - t < 90.0]
                        self.escape_times.append(now)
                        if len(self.escape_times) >= 2:
                            self._start_wall_follow(now, 'double escape in 90s')

    def _start_wall_follow(self, now, why):
        # v2.9: lock in 25s of wall-following toward the NEAR wall and clear whatever
        # would compete with it (escape/leave/commit timers, latch, memories, hysteresis)
        self.wall_follow_until = now + 25.0
        self.wall_follow_side = 1.0 if self.d_L < self.d_R else -1.0
        self.escape_turn_until = 0.0
        self.leave_until = 0.0
        self.commit_until = 0.0
        self.reverse_latch = False
        self.osc_times.clear()
        self.reverse_times.clear()
        self.escape_times.clear()
        self.prev_best_gap_score = 0.0
        self.get_logger().info(
            f'[WALL_FOLLOW] {why} -> following the {"left" if self.wall_follow_side > 0 else "right"} '
            f'wall for 25s (L={self.d_L:.2f} R={self.d_R:.2f})')

    def _status_log(self, lin, ang, now):
        if now - self.last_status_log > 5.0:
            self.last_status_log = now
            self.get_logger().info(
                f'[STATUS] mode={self.mode} front={self.front_dist:.2f} L={self.d_L:.2f} R={self.d_R:.2f} '
                f'gap={self.best_gap_angle:+.2f} v={lin:+.2f} w={ang:+.2f} personLvl={self.person_level}')

    # ---------- scan (SAME as v1) ----------
    def scan_callback(self, msg):
        now_scan = time.time()
        if now_scan - self.last_scan_time > 10.0:
            # v2.9.1: a long /scan gap = run change (batch Stop->worldgen->Play) or a heavy
            # freeze -> pristine state so each run is an independent sample
            self.osc_times.clear()
            self.reverse_times.clear()
            self.escape_times.clear()
            self.narrow_filter_until = 0.0
            self.commit_until = 0.0
            self.escape_turn_until = 0.0
            self.leave_until = 0.0
            self.wall_follow_until = 0.0
            self.wiggle_until = 0.0
            self.reverse_latch = False
            self.reverse_dir = 0.0
            self.reverse_since = 0.0
            self.is_recovering = False
            self.stuck_start_time = 0.0
            self.watchdog_start = 0.0
            self.commit_fail_count = 0
            self.prev_best_gap_score = 0.0
            self.prev_ang = 0.0
            self.person_stop_since = 0.0
            self.person_bypass = False
            self.scan_snapshot = None
            self.scan_change = 999.0
            self.motion_cmd_since = 0.0
            self.get_logger().info('[RUN_RESET] empty /scan >10s -> fresh state for a new run')
        self.last_scan_time = now_scan
        ranges = np.array(msg.ranges)
        valid_mask = (ranges > 0.1) & (ranges < msg.range_max)
        ranges[~valid_mask] = 10.0

        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        X = ranges * np.cos(angles)
        Y = ranges * np.sin(angles)

        rear_mask = (X < -0.1) & (X > -1.0) & (np.abs(Y) < 0.18)
        self.rear_dist = np.min(np.abs(X[rear_mask])) if np.any(rear_mask) else 10.0

        front_mask = (X > 0.1) & (X < 1.2) & (np.abs(Y) < 0.22)
        self.front_dist = np.min(X[front_mask]) if np.any(front_mask) else 10.0

        left_mask = (angles > 0.3) & (angles < 1.5)
        right_mask = (angles < -0.3) & (angles > -1.5)
        self.d_L = np.min(ranges[left_mask]) if np.any(left_mask) else 10.0
        self.d_R = np.min(ranges[right_mask]) if np.any(right_mask) else 10.0

        # v2.8: scan change over a ~1.2s window (proxy for real displacement — we have no odom)
        now_t8 = time.time()
        ds8 = ranges[::5]
        if self.scan_snapshot is None or len(ds8) != len(self.scan_snapshot):
            self.scan_snapshot = ds8.copy()
            self.scan_snapshot_t = now_t8
        elif now_t8 - self.scan_snapshot_t >= 1.2:
            self.scan_change = float(np.mean(np.abs(ds8 - self.scan_snapshot)))
            self.scan_snapshot = ds8.copy()
            self.scan_snapshot_t = now_t8

        fov_mask = (angles > -1.5) & (angles < 1.5)
        fov_ranges_raw = ranges[fov_mask].copy()
        fov_angles = angles[fov_mask]

        if len(fov_ranges_raw) < 2:
            return

        inflation_radius = 0.12
        inflated_ranges = fov_ranges_raw.copy()
        angle_inc = msg.angle_increment

        for i in range(len(fov_ranges_raw)):
            r = fov_ranges_raw[i]
            if r < 3.0:
                val = np.clip(inflation_radius / max(r, inflation_radius), 0.0, 1.0)
                delta_theta = np.arcsin(val)
                num_indices = int(delta_theta / angle_inc)
                start_idx = max(0, i - num_indices)
                end_idx = min(len(fov_ranges_raw), i + num_indices + 1)
                inflated_ranges[start_idx:end_idx] = np.minimum(inflated_ranges[start_idx:end_idx], r)

        diffs = np.diff(inflated_ranges)
        threshold = 0.3
        edge_indices = np.where(np.abs(diffs) > threshold)[0]
        all_edges = [0] + list(edge_indices) + [len(inflated_ranges) - 2]

        valid_gaps = []
        for i in range(len(all_edges) - 1):
            idx1 = all_edges[i]
            idx2 = all_edges[i + 1] + 1
            segment_ranges_raw = fov_ranges_raw[idx1 + 1:idx2]
            if len(segment_ranges_raw) == 0:
                continue
            segment_mean = np.mean(segment_ranges_raw)
            r1 = fov_ranges_raw[idx1]
            r2 = fov_ranges_raw[idx2]
            deep_fraction = np.mean(segment_ranges_raw > 1.0)

            if (segment_mean > r1 + 0.2 or segment_mean > r2 + 0.2) and deep_fraction > 0.3:
                theta1 = fov_angles[idx1]
                theta2 = fov_angles[idx2]
                W = np.sqrt(r1**2 + r2**2 - 2 * r1 * r2 * np.cos(abs(theta1 - theta2)))
                # (6) while the narrow filter holds, narrow gaps are not even candidates
                if time.time() < self.narrow_filter_until and W < self.NARROW_W:
                    continue
                if W > 0.15:
                    x1, y1 = r1 * np.cos(theta1), r1 * np.sin(theta1)
                    x2, y2 = r2 * np.cos(theta2), r2 * np.sin(theta2)
                    mid_x = (x1 + x2) / 2.0
                    mid_y = (y1 + y2) / 2.0
                    target_angle = np.arctan2(mid_y, mid_x)
                    dx = x2 - x1
                    dy = y2 - y1
                    nx, ny = -dy, dx
                    if nx < 0:
                        nx, ny = dy, -dx
                    axis_angle = np.arctan2(ny, nx)
                    is_real_gate = (idx1 != 0) and (idx2 != len(fov_ranges_raw) - 1)
                    effective_width = min(W, 2.0)
                    score = effective_width / (1.0 + 4.0 * abs(target_angle))
                    valid_gaps.append({
                        'width': W, 'angle': target_angle, 'axis_angle': axis_angle,
                        'score': score, 'is_real_gate': is_real_gate
                    })

        if len(valid_gaps) > 0:
            best_candidate = max(valid_gaps, key=lambda g: g['score'])
            if self.prev_best_gap_score > 0 and abs(best_candidate['angle'] - self.prev_best_gap_angle) > 0.4:
                if best_candidate['score'] < self.prev_best_gap_score * 1.5:
                    for g in valid_gaps:
                        if abs(g['angle'] - self.prev_best_gap_angle) < 0.4:
                            best_candidate = g
                            break
            self.best_gap_angle = best_candidate['angle']
            self.best_gap_axis_angle = best_candidate['axis_angle']
            self.best_gap_width = best_candidate['width']
            self.best_gap_is_real_gate = best_candidate['is_real_gate']
            self.is_wedged = False
            self.prev_best_gap_score = best_candidate['score']
            self.prev_best_gap_angle = best_candidate['angle']
            # (6) rolling-max width for the COMMIT decision
            now_t = time.time()
            if now_t - self.recent_gap_t > 3.0:
                self.recent_gap_w = 0.0
            if best_candidate['width'] > self.recent_gap_w:
                self.recent_gap_w = best_candidate['width']
                self.recent_gap_t = now_t
        else:
            self.best_gap_angle = 0.0
            self.best_gap_axis_angle = 0.0
            self.best_gap_width = 0.0
            self.best_gap_is_real_gate = False
            self.is_wedged = True
            self.prev_best_gap_score = 0.0

        k = 5
        smooth = np.convolve(fov_ranges_raw, np.ones(k) / k, mode='same')
        self.open_angle = fov_angles[int(np.argmax(smooth))]

        # (6/v2.3) the rolling-max expiry runs ALWAYS (even in wedge) - no stale values
        if time.time() - self.recent_gap_t > 3.0:
            self.recent_gap_w = 0.0

    # ---------- camera / YOLO ----------
    def image_callback(self, msg):
        current_time = time.time()
        if current_time - self.last_yolo_time > 0.15:
            self.last_yolo_time = current_time
            img = np.array(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
            img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

            results = self.model(img_bgr, verbose=False)
            person_found = False
            person_h = 0

            if len(results[0].boxes) > 0:
                for box in results[0].boxes:
                    if int(box.cls[0]) == 0:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        box_width = x2 - x1
                        person_center = x1 + (box_width // 2)
                        cv2.rectangle(img_bgr, (x1, y1), (x2, y2), (255, 0, 0), 2)
                        if 213 < person_center < 426:
                            person_found = True
                            person_h = max(person_h, y2 - y1)   # (5) keep the nearest one
            self.person_in_front = person_found
            self.person_h = person_h if person_found else 0

            if self.SHOW_GUI:
                try:
                    cv2.putText(img_bgr, "GUI ACTIVE", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                    cv2.imshow("YOLO Navigator v2", img_bgr)
                    cv2.waitKey(1)
                except Exception:
                    self.get_logger().error("cv2.imshow failed, disabling GUI.")
                    self.SHOW_GUI = False

    # ---------- control ----------
    def control_loop(self):
        vel_msg = Twist()
        target_linear_x = 0.4
        target_angular_z = 0.0
        current_time = time.time()

        # 0. (v2.5) Scan staleness guard: if /scan froze (Isaac freeze/pause/Stop),
        # do NOT drive on stale data — stand still and wait. Before ALL the
        # state machines (otherwise watchdog/recovery would drive blind).
        if current_time - self.last_scan_time > 2.5:
            self.velocity_publisher.publish(vel_msg)   # zero Twist
            self.prev_ang = 0.0
            self._set_mode('SENSOR_STALE', f'no /scan for {current_time - self.last_scan_time:.1f}s')
            self._status_log(0.0, 0.0, current_time)
            return

        V_MAX, SLOW_START, STOP_DIST = 0.4, 0.7, 0.32
        cruise = np.clip(V_MAX * (self.front_dist - STOP_DIST) / (SLOW_START - STOP_DIST), 0.0, V_MAX)

        # 0b. (v2.8) WIGGLE: alternating diagonal pushes to mechanically unhook
        if current_time < self.wiggle_until:
            t8 = current_time - self.wiggle_start
            phase = (int(t8 / 0.7) + self.wiggle_count) % 4
            v8, w8 = ((-0.25, -1.0), (0.25, 1.0), (-0.25, 1.0), (0.25, -1.0))[phase]
            vel_msg.linear.x = v8
            vel_msg.angular.z = w8
            self.velocity_publisher.publish(vel_msg)
            self.prev_ang = w8
            self._set_mode('STUCK_WIGGLE', f'phase={phase} n={self.wiggle_count}')
            self._status_log(v8, w8, current_time)
            return

        # 0c. (v2.9) WALL_FOLLOW: pocket exit by wall-following — a pocket's "deep
        # openings" are deceptive, but the wall always leads to the exit.
        if current_time < self.wall_follow_until:
            d_side = self.d_L if self.wall_follow_side > 0 else self.d_R
            if self.front_dist < 0.45:
                v9, w9 = 0.0, -1.0 * self.wall_follow_side      # turn AWAY from the wall
            else:
                err = d_side - 0.5
                v9 = 0.22
                w9 = float(np.clip(1.5 * err, -1.0, 1.0)) * self.wall_follow_side
            vel_msg.linear.x = v9
            vel_msg.angular.z = w9
            self.velocity_publisher.publish(vel_msg)
            self.prev_ang = w9
            self._set_mode('WALL_FOLLOW',
                           f'side={"L" if self.wall_follow_side > 0 else "R"} d={d_side:.2f} front={self.front_dist:.2f}')
            self._status_log(v9, w9, current_time)
            return

        # 1. Recovery State Machine (SAME)
        if self.is_recovering:
            if current_time - self.recovery_start_time < 2.5:
                vel_msg.linear.x = -0.3 if self.rear_dist > 0.4 else (0.1 if self.front_dist > 0.5 else 0.0)
                vel_msg.angular.z = 1.0 if self.d_L > self.d_R else -1.0
                self.velocity_publisher.publish(vel_msg)
                self.prev_ang = vel_msg.angular.z
                self._set_mode('RECOVERY', f'front={self.front_dist:.2f} rear={self.rear_dist:.2f}')
                self._status_log(vel_msg.linear.x, vel_msg.angular.z, current_time)
                return
            else:
                self.is_recovering = False
                self.stuck_start_time = 0.0
                self.watchdog_start = 0.0
                self.noprog_start = 0.0

        # 1b. (6) Escape turn out of a too-narrow passage
        if current_time < self.escape_turn_until:
            vel_msg.linear.x = 0.0 if self.rear_dist < 0.4 else -0.1
            vel_msg.angular.z = 1.0 * self.escape_turn_dir
            self.velocity_publisher.publish(vel_msg)
            self.prev_ang = vel_msg.angular.z
            self._set_mode('ESCAPE_NARROW', f'dir={self.escape_turn_dir:+.0f}')
            self._status_log(vel_msg.linear.x, vel_msg.angular.z, current_time)
            return

        # 1b2. (6) LEAVE mode: actively moving away from the pocket after the escape turn
        if current_time < self.leave_until:
            vel_msg.linear.x = 0.25 if self.front_dist > 0.6 else 0.05
            vel_msg.angular.z = float(np.clip(1.0 * self.open_angle, -0.8, 0.8))
            self.velocity_publisher.publish(vel_msg)
            self.prev_ang = vel_msg.angular.z
            self._set_mode('LEAVE', f'open={self.open_angle:+.2f} front={self.front_dist:.2f}')
            self._status_log(vel_msg.linear.x, vel_msg.angular.z, current_time)
            return

        # 1c. (6) COMMIT mode: decisive slow passage of a narrow (bypasses trap/latch)
        if current_time < self.commit_until:
            if self.front_dist < 0.15:
                self.commit_until = 0.0          # hard abort -> normal protections
                self.commit_fail_count += 1      # v2.3: two-strikes bookkeeping
                self.commit_fail_t = current_time
                self.get_logger().info(f'[COMMIT] failed (front<0.15), failure #{self.commit_fail_count}')
            elif self.front_dist > 1.2 and self.best_gap_width > 1.0:
                self.commit_until = 0.0          # made it! back to normal FTG
                self.commit_fail_count = 0
                self.get_logger().info('[COMMIT] pass completed')
            else:
                steer = self.best_gap_angle if not self.is_wedged else self.open_angle
                axis = self.best_gap_axis_angle if not self.is_wedged else 0.0
                # v2.7: crawl at the threshold — fewer unfair aborts at front<0.15
                vel_msg.linear.x = 0.10 if self.front_dist > 0.30 else 0.06
                vel_msg.angular.z = float(np.clip(1.5 * steer + 0.5 * axis, -1.0, 1.0))
                # person braking applies here TOO (safety)
                if self.person_in_front and self.person_h > 320:
                    vel_msg.linear.x = 0.0
                self.velocity_publisher.publish(vel_msg)
                self.prev_ang = vel_msg.angular.z
                self.reverse_latch = False
                self._set_mode('COMMIT', f'steer={vel_msg.angular.z:+.2f} front={self.front_dist:.2f}')
                self._status_log(vel_msg.linear.x, vel_msg.angular.z, current_time)
                return

        # 2. Wedge trap escape (SAME)
        in_wedge = self.front_dist < 0.5 and self.d_L < 0.35 and self.d_R < 0.35
        if in_wedge or current_time < self.escape_until:
            if in_wedge and current_time >= self.escape_until:
                self.escape_until = current_time + 3.0
            vel_msg.linear.x = -0.25 if self.rear_dist > 0.4 else 0.0
            vel_msg.angular.z = 1.2 if self.d_L > self.d_R else -1.2
            self.velocity_publisher.publish(vel_msg)
            self.prev_ang = vel_msg.angular.z
            self._set_mode('TRAP', f'L={self.d_L:.2f} R={self.d_R:.2f}')
            self._status_log(vel_msg.linear.x, vel_msg.angular.z, current_time)
            return

        # 3. Stuck Detection — v1 condition + (3) no-progress near STOP_DIST
        near_wall_noprog = (self.front_dist < STOP_DIST + 0.06) and (abs(self.prev_ang) < 0.3)
        if self.front_dist < 0.25 or near_wall_noprog:
            if self.stuck_start_time == 0.0:
                self.stuck_start_time = current_time
            elif current_time - self.stuck_start_time > 3.0:
                self.is_recovering = True
                self.recovery_start_time = current_time
                self.stuck_start_time = 0.0
                self._set_mode('RECOVERY', 'stuck detector (front/no-progress)')
                return
        else:
            self.stuck_start_time = 0.0

        # 4. Schmitt trigger (SAME)
        if self.front_dist < STOP_DIST:
            self.reverse_latch = True
        elif self.front_dist > STOP_DIST + 0.12:
            self.reverse_latch = False
            self.reverse_dir = 0.0       # v2.5: clear the locked direction
            self.reverse_since = 0.0

        # 5. Basic Motion
        person_stop_active = False
        if self.reverse_latch:
            # v2.5 (a): lock the direction at entry — no flip-flop with the L/R noise
            if self.mode != 'REVERSE' or self.reverse_dir == 0.0:
                self.reverse_dir = 1.0 if self.d_L > self.d_R else -1.0
                self.reverse_since = current_time
            if max(self.d_L, self.d_R) > 0.6:
                target_linear_x = 0.05
                target_angular_z = 1.2 * self.reverse_dir
            else:
                # v2.5 (b): reverse only with a free back
                target_linear_x = -0.3 if self.rear_dist > 0.45 else 0.0
                target_angular_z = 0.8 * self.reverse_dir
            self._set_mode('REVERSE', f'front={self.front_dist:.2f} dir={self.reverse_dir:+.0f}')
            # v2.5 (c) + v2.7: continuous REVERSE >6s -> FIRST a fair chance at COMMIT
            # if a live passable gap exists, else ESCAPE_POCKET (the ESCAPE_NARROW path)
            if current_time - self.reverse_since > 6.0:
                if current_time - self.commit_fail_t > 30.0:
                    self.commit_fail_count = 0
                if (not self.is_wedged) and self.best_gap_width >= self.NARROW_W and self.commit_fail_count < 2:
                    self.commit_until = current_time + 15.0
                    self.reverse_latch = False
                    self.reverse_since = 0.0
                    self.reverse_dir = 0.0
                    self.get_logger().info(
                        f'[COMMIT] REVERSE>6s with a passable gap (W={self.best_gap_width:.2f}) -> decisive pass')
                    return
                repeat = (current_time - self.last_escape_t) < 30.0
                leave_dur = 8.0 if repeat else 4.0
                self.last_escape_t = current_time
                self.narrow_filter_until = current_time + 20.0
                self.escape_turn_until = current_time + 3.0
                self.leave_until = current_time + 3.0 + leave_dur
                # v2.6: direction toward the side with space; on a relapse, reversed
                if repeat and self.escape_turn_dir != 0.0:
                    self.escape_turn_dir = -self.escape_turn_dir
                else:
                    self.escape_turn_dir = 1.0 if self.d_L > self.d_R else -1.0
                self.prev_best_gap_score = 0.0
                self.reverse_latch = False
                self.reverse_since = 0.0
                self.reverse_dir = 0.0
                self.get_logger().info(
                    f'[ESCAPE_POCKET] REVERSE>6s with no way out (front={self.front_dist:.2f} '
                    f'L={self.d_L:.2f} R={self.d_R:.2f} rear={self.rear_dist:.2f}) -> '
                    f'turn+leave (leave={leave_dur:.0f}s{", relapse" if repeat else ""}), narrow filter 20s')
                # v2.9 (b): second escape within 90s -> WALL_FOLLOW
                self.escape_times = [t for t in self.escape_times if current_time - t < 90.0]
                self.escape_times.append(current_time)
                if len(self.escape_times) >= 2:
                    self._start_wall_follow(current_time, 'double escape in 90s')
                return

        elif self.is_wedged:
            if current_time < self.narrow_filter_until:
                # v2.6: cooldown after an escape — do NOT chase depth (open_angle):
                # in a pocket the "deepest" point is the very narrow you just left.
                # Move away almost straight; if the front closes, turn toward the open side.
                if self.front_dist > 0.6:
                    target_linear_x = 0.25
                    target_angular_z = float(np.clip(0.5 * self.open_angle, -0.3, 0.3))
                else:
                    target_linear_x = 0.0
                    target_angular_z = 1.0 if self.d_L > self.d_R else -1.0
                self._set_mode('WEDGE', f'cooldown open={self.open_angle:+.2f}')
            elif self.front_dist > 1.0:
                target_linear_x = min(0.4, cruise)
                target_angular_z = np.clip(1.2 * self.open_angle, -1.0, 1.0)
                self._set_mode('WEDGE', f'open={self.open_angle:+.2f}')
            else:
                target_linear_x = 0.05
                target_angular_z = 1.0 if self.open_angle > 0 else -1.0
                self._set_mode('WEDGE', f'open={self.open_angle:+.2f}')

        else:
            base_angular_z = 1.0 * self.best_gap_angle + 0.3 * self.best_gap_axis_angle
            heading_err = abs(self.best_gap_angle)
            align_factor = max(0.25, 1.0 - 1.2 * heading_err)
            target_linear_x = min(0.4, cruise) * align_factor
            target_angular_z = base_angular_z

            min_side_dist = min(self.d_L, self.d_R)
            if min_side_dist < 0.4 and abs(self.best_gap_angle) < 0.25:
                diff = self.d_L - self.d_R
                if abs(diff) > 0.15:
                    target_angular_z += 0.4 * diff
            if min_side_dist < 0.25:
                target_linear_x = min(target_linear_x, 0.15)

            # (2) Creep-band ban: instead of creeping, turn toward the open
            if cruise < 0.08 and self.front_dist < 0.6:
                target_linear_x = 0.0
                if current_time < self.narrow_filter_until:
                    # v2.6: in cooldown turn toward the open SIDE, not toward depth
                    target_angular_z = 1.0 if self.d_L > self.d_R else -1.0
                else:
                    target_angular_z = 1.0 if self.open_angle > 0 else -1.0
                self._set_mode('ROTATE_OPEN', f'front={self.front_dist:.2f} open={self.open_angle:+.2f}')
            else:
                self._set_mode('FTG', f'gap={self.best_gap_angle:+.2f} W={self.best_gap_width:.2f}')

            # (5) Graded person braking (bbox height ~ distance: 969/d px)
            # (7/v2.4) + "polite bypass": after 8s of waiting at a motionless person,
            # continue SLOWLY (0.1 m/s) around them — the lidar sees them as an obstacle anyway.
            new_level = 0
            if self.person_in_front:
                if self.person_h > 320:      # ~<3.0 m
                    if self.person_stop_since == 0.0:
                        self.person_stop_since = current_time
                        self.person_bypass = False
                    if current_time - self.person_stop_since > 8.0:
                        if not self.person_bypass:
                            self.person_bypass = True
                            self.get_logger().info('[PERSON] 8s waiting on a standing person -> polite slow bypass')
                        target_linear_x = min(target_linear_x, 0.1)
                        new_level = 2
                    else:
                        target_linear_x = 0.0
                        person_stop_active = True
                        new_level = 3
                elif self.person_h > 200:    # ~<4.8 m
                    target_linear_x = min(target_linear_x, 0.1)
                    new_level = 2
                    self.person_stop_since = 0.0
                    self.person_bypass = False
                elif self.person_h > 120:    # ~<8 m
                    target_linear_x = max(0.0, target_linear_x - 0.15)
                    new_level = 1
                    self.person_stop_since = 0.0
                    self.person_bypass = False
            else:
                self.person_stop_since = 0.0
                self.person_bypass = False
            if new_level != self.person_level:
                self.person_level = new_level
                self.get_logger().info(f'[PERSON] level={new_level} h={self.person_h}px')

        # 6. Safety and Publish (SAME)
        target_angular_z = np.clip(target_angular_z, -1.5, 1.5)
        MAX_DANG = 0.35
        target_angular_z = np.clip(target_angular_z, self.prev_ang - MAX_DANG, self.prev_ang + MAX_DANG)
        self.prev_ang = target_angular_z

        # v2.5 (b): the guard cancelled reverse for front>=0.32, i.e. EXACTLY in the
        # 0.32-0.44 band where the REVERSE latch is active and reverse is the only way out.
        # Now it is cancelled only when there is comfortable space ahead (reverse is pointless);
        # back safety is guaranteed by the explicit rear check in step 5.
        if target_linear_x < 0.0 and self.front_dist >= STOP_DIST + 0.20 and not self.is_recovering:
            target_linear_x = 0.0

        # (v2.8) Physical-stuck: we command motion but the scene does NOT change -> WIGGLE
        if (abs(target_linear_x) >= 0.12 or abs(target_angular_z) >= 0.5) and \
           current_time - self.last_scan_time < 1.0 and self.scan_change < 0.015:
            if self.motion_cmd_since == 0.0:
                self.motion_cmd_since = current_time
            elif current_time - self.motion_cmd_since > 3.0:
                self.wiggle_until = current_time + 3.0
                self.wiggle_start = current_time
                self.wiggle_count += 1
                self.motion_cmd_since = 0.0
                self.get_logger().info(
                    f'[STUCK_PHYSICAL] motion commands with no scan change (D={self.scan_change:.3f}m, '
                    f'mode={self.mode}) -> WIGGLE #{self.wiggle_count}')
                return
        else:
            self.motion_cmd_since = 0.0

        # (1) Progress watchdog: zero command with no reason (not person-stop) for >2.5s -> recovery
        if abs(target_linear_x) < 0.05 and abs(target_angular_z) < 0.3 and not person_stop_active:
            if self.watchdog_start == 0.0:
                self.watchdog_start = current_time
            elif current_time - self.watchdog_start > 2.5:
                self.is_recovering = True
                self.recovery_start_time = current_time
                self.watchdog_start = 0.0
                self._set_mode('RECOVERY', 'WATCHDOG: no progress >2.5s')
                return
        else:
            self.watchdog_start = 0.0

        vel_msg.linear.x = target_linear_x
        vel_msg.angular.z = target_angular_z
        self.velocity_publisher.publish(vel_msg)
        self._status_log(target_linear_x, target_angular_z, current_time)


def main(args=None):
    rclpy.init(args=args)
    navigator = YoloNavigator()
    executor = MultiThreadedExecutor()
    executor.add_node(navigator)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        stop_msg = Twist()
        navigator.velocity_publisher.publish(stop_msg)
        navigator.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
