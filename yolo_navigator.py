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
        
        self.get_logger().info('Loading YOLOv8 for autonomous navigation...')
        self.model = YOLO('yolov8n.pt') 
        
        # Use ReentrantCallbackGroup for the Subscriptions (so image and scan run in parallel)
        self.callback_group = ReentrantCallbackGroup()
        # Use MutuallyExclusiveCallbackGroup for the Timer, so control loops don't overlap
        self.timer_cb_group = MutuallyExclusiveCallbackGroup()

        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.image_sub = self.create_subscription(
            Image, '/camera', self.image_callback, qos_profile_sensor_data, callback_group=self.callback_group)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data, callback_group=self.callback_group)
            
        # Control timer at 20Hz, decoupled from the camera
        self.control_timer = self.create_timer(0.05, self.control_loop, callback_group=self.timer_cb_group)
        
        self.person_in_front = False
        self.SHOW_GUI = False  # By default False because imshow is not thread-safe in worker threads
        
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
        
        self.get_logger().info('System Active! Moving ahead with Follow-the-Gap...')

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        valid_mask = (ranges > 0.1) & (ranges < msg.range_max)
        ranges[~valid_mask] = 10.0 

        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        X = ranges * np.cos(angles)
        Y = ranges * np.sin(angles)

        # 1. Rear side
        rear_mask = (X < -0.1) & (X > -1.0) & (np.abs(Y) < 0.18)
        self.rear_dist = np.min(np.abs(X[rear_mask])) if np.any(rear_mask) else 10.0

        # 2. Front zone (Wider and longer cone)
        front_mask = (X > 0.1) & (X < 1.2) & (np.abs(Y) < 0.22)
        self.front_dist = np.min(X[front_mask]) if np.any(front_mask) else 10.0

        # 3. Side Distance Computation (Dynamic Centering)
        left_mask = (angles > 0.3) & (angles < 1.5)
        right_mask = (angles < -0.3) & (angles > -1.5)
        self.d_L = np.min(ranges[left_mask]) if np.any(left_mask) else 10.0
        self.d_R = np.min(ranges[right_mask]) if np.any(right_mask) else 10.0

        # 4. Gap Detection (Follow-the-Gap)
        fov_mask = (angles > -1.5) & (angles < 1.5)
        fov_ranges_raw = ranges[fov_mask].copy()
        fov_angles = angles[fov_mask]

        if len(fov_ranges_raw) < 2:
            return
            
        # 4.1 1D Obstacle Inflation (Inflation Radius locked)
        inflation_radius = 0.12
        inflated_ranges = fov_ranges_raw.copy()
        angle_inc = msg.angle_increment

        for i in range(len(fov_ranges_raw)):
            r = fov_ranges_raw[i]
            if r < 3.0: 
                # np.clip to avoid Invalid values if r < inflation_radius
                val = np.clip(inflation_radius / max(r, inflation_radius), 0.0, 1.0)
                delta_theta = np.arcsin(val)
                num_indices = int(delta_theta / angle_inc)
                
                start_idx = max(0, i - num_indices)
                end_idx = min(len(fov_ranges_raw), i + num_indices + 1)
                
                inflated_ranges[start_idx:end_idx] = np.minimum(inflated_ranges[start_idx:end_idx], r)

        # Find the discontinuities (edges) on the INFLATED data
        diffs = np.diff(inflated_ranges)
        threshold = 0.3
        edge_indices = np.where(np.abs(diffs) > threshold)[0]
        
        all_edges = [0] + list(edge_indices) + [len(inflated_ranges) - 2]

        valid_gaps = []
        for i in range(len(all_edges) - 1):
            idx1 = all_edges[i]
            idx2 = all_edges[i+1] + 1
            
            # The segment must be checked against the RAW data for the depth validation
            segment_ranges_raw = fov_ranges_raw[idx1+1:idx2]
            if len(segment_ranges_raw) == 0:
                continue
                
            segment_mean = np.mean(segment_ranges_raw)
            r1 = fov_ranges_raw[idx1]
            r2 = fov_ranges_raw[idx2]
            
            # FIX A: the gap must be deep across the WHOLE width (not 1 ray)
            deep_fraction = np.mean(segment_ranges_raw > 1.0)
            
            if (segment_mean > r1 + 0.2 or segment_mean > r2 + 0.2) and deep_fraction > 0.3:
                theta1 = fov_angles[idx1]
                theta2 = fov_angles[idx2]
                
                # Width W (Law of Cosines)
                W = np.sqrt(r1**2 + r2**2 - 2*r1*r2*np.cos(abs(theta1 - theta2)))
                if W > 0.15: # Since we inflate, W here can be small
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
                    
                    # New Scoring: We put a "cap" on W (2.0m) so the vast open areas to the sides don't 
                    # beat a narrower (but sufficient) passage that is right in front!
                    effective_width = min(W, 2.0)
                    score = effective_width / (1.0 + 4.0 * abs(target_angle))
                    
                    valid_gaps.append({
                        'width': W, 'angle': target_angle, 'axis_angle': axis_angle, 
                        'score': score, 'is_real_gate': is_real_gate
                    })

        # Hysteresis + Best Gap Selection (strong: 0.4 window, 1.5 ratio -> commit to one side)
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
        else:
            self.best_gap_angle = 0.0
            self.best_gap_axis_angle = 0.0
            self.best_gap_width = 0.0
            self.best_gap_is_real_gate = False
            self.is_wedged = True
            self.prev_best_gap_score = 0.0
            
        # Most open direction (smoothed) — for reorientation when it finds no gap
        k = 5
        smooth = np.convolve(fov_ranges_raw, np.ones(k)/k, mode='same')
        self.open_angle = fov_angles[int(np.argmax(smooth))]

    def image_callback(self, msg):
        current_time = time.time()
        
        # Run YOLO only every 0.15s (~6-7 Hz) so we don't consume 100% CPU nonstop
        if current_time - self.last_yolo_time > 0.15:
            self.last_yolo_time = current_time
            img = np.array(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
            img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            
            results = self.model(img_bgr, verbose=False)
            person_found = False
            
            if len(results[0].boxes) > 0:
                for box in results[0].boxes:
                    if int(box.cls[0]) == 0:  
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        box_width = x2 - x1
                        person_center = x1 + (box_width // 2)
                        cv2.rectangle(img_bgr, (x1, y1), (x2, y2), (255, 0, 0), 2)
                        
                        if 213 < person_center < 426:
                            person_found = True
                            break
            self.person_in_front = person_found

            if self.SHOW_GUI:
                try:
                    status_text = "GUI ACTIVE"
                    cv2.putText(img_bgr, status_text, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
                    cv2.imshow("YOLO Navigator", img_bgr)
                    cv2.waitKey(1)
                except Exception:
                    self.get_logger().error("cv2.imshow failed, disabling GUI.")
                    self.SHOW_GUI = False

    def control_loop(self):
        vel_msg = Twist()
        target_linear_x = 0.4
        target_angular_z = 0.0
        current_time = time.time()

        # Proportional braking
        V_MAX, SLOW_START, STOP_DIST = 0.4, 0.7, 0.32
        cruise = np.clip(V_MAX * (self.front_dist - STOP_DIST) / (SLOW_START - STOP_DIST), 0.0, V_MAX)

        # 1. Recovery State Machine
        if self.is_recovering:
            if current_time - self.recovery_start_time < 2.5:
                vel_msg.linear.x = -0.3 if self.rear_dist > 0.4 else (0.1 if self.front_dist > 0.5 else 0.0)
                vel_msg.angular.z = 1.0 if self.d_L > self.d_R else -1.0
                self.velocity_publisher.publish(vel_msg)
                self.prev_ang = vel_msg.angular.z
                return
            else:
                self.is_recovering = False
                self.stuck_start_time = 0.0

        # 2. FIX B: Wedge trap escape (tight on BOTH sides + blocked)
        in_wedge = self.front_dist < 0.5 and self.d_L < 0.35 and self.d_R < 0.35
        if in_wedge or current_time < self.escape_until:
            if in_wedge and current_time >= self.escape_until:
                self.escape_until = current_time + 3.0   # exit commitment, ~200° turn
            vel_msg.linear.x = -0.25 if self.rear_dist > 0.4 else 0.0
            vel_msg.angular.z = 1.2 if self.d_L > self.d_R else -1.2
            self.velocity_publisher.publish(vel_msg)
            self.prev_ang = vel_msg.angular.z
            return

        # 3. Stuck Detection (facing a wall)
        if self.front_dist < 0.25:
            if self.stuck_start_time == 0.0:
                self.stuck_start_time = current_time
            elif current_time - self.stuck_start_time > 3.0:
                self.is_recovering = True
                self.recovery_start_time = current_time
                self.stuck_start_time = 0.0
                return
        else:
            self.stuck_start_time = 0.0

        # 4. Schmitt trigger for reverse (no chattering at the threshold)
        if self.front_dist < STOP_DIST:
            self.reverse_latch = True
        elif self.front_dist > STOP_DIST + 0.12:
            self.reverse_latch = False

        # 5. Basic Motion
        if self.reverse_latch:
            if max(self.d_L, self.d_R) > 0.6:        # room to the side -> turn
                target_linear_x = 0.05
                target_angular_z = 1.2 if self.d_L > self.d_R else -1.2
            else:                                     # closed in -> reverse
                target_linear_x = -0.3
                target_angular_z = 0.8 if self.d_L > self.d_R else -0.8

        elif self.is_wedged:
            if self.front_dist > 1.0:                 # open space -> straight
                target_linear_x = min(0.4, cruise)
                target_angular_z = np.clip(1.2 * self.open_angle, -1.0, 1.0)  # curve toward the open
            else:
                target_linear_x = 0.05
                target_angular_z = 1.0 if self.open_angle > 0 else -1.0       # turn toward the open

        else:
            base_angular_z = 1.0 * self.best_gap_angle + 0.3 * self.best_gap_axis_angle

            # Speed/angle coupling: large error -> slow (smooth arc, no back-and-forth)
            heading_err = abs(self.best_gap_angle)
            align_factor = max(0.25, 1.0 - 1.2 * heading_err)
            target_linear_x = min(0.4, cruise) * align_factor
            target_angular_z = base_angular_z

            # Centering ONLY when going nearly straight (otherwise it fights the gap-steering)
            min_side_dist = min(self.d_L, self.d_R)
            if min_side_dist < 0.4 and abs(self.best_gap_angle) < 0.25:
                diff = self.d_L - self.d_R
                if abs(diff) > 0.15:
                    target_angular_z += 0.4 * diff
            if min_side_dist < 0.25:
                target_linear_x = min(target_linear_x, 0.15)

            if self.person_in_front:
                target_linear_x = max(0.1, target_linear_x - 0.2)

        # 4. Safety and Publishing
        target_angular_z = np.clip(target_angular_z, -1.5, 1.5)

        # CHANGE #2: looser slew-rate so it can turn in time (hysteresis keeps the jitter in check)
        MAX_DANG = 0.35
        target_angular_z = np.clip(target_angular_z, self.prev_ang - MAX_DANG, self.prev_ang + MAX_DANG)
        self.prev_ang = target_angular_z
        
        if target_linear_x < 0.0 and self.front_dist >= STOP_DIST and not self.is_recovering:
            target_linear_x = 0.0

        vel_msg.linear.x = target_linear_x
        vel_msg.angular.z = target_angular_z
        self.velocity_publisher.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)
    navigator = YoloNavigator()
    # Use MultiThreadedExecutor so YOLO doesn't block the control loop
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
