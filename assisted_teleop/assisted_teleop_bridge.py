import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
import math
import numpy as np


class AssistedTeleopBridge(Node):

    N_BINS  = 72
    BIN_RAD = 2.0 * math.pi / N_BINS
    FWD_IDX = N_BINS // 2              

    def __init__(self):
        super().__init__('assisted_teleop_bridge')

        self.declare_parameter('scan_topic',   '/scan')
        self.declare_parameter('input_topic',  '/joy_vel')
        self.declare_parameter('output_topic', '/cmd_vel')

        self.declare_parameter('enable_threshold', 0.05)
        self.declare_parameter('idle_timeout',      0.5)

        self.declare_parameter('laser_x_offset', 0.0)
        self.declare_parameter('laser_y_offset', 0.0)
        self.declare_parameter('robot_front_m',  0.17)
        self.declare_parameter('robot_half_w',   0.09)

        self.declare_parameter('free_dist_m',    0.50)
        self.declare_parameter('guard_dist_m',   0.10) 
        self.declare_parameter('min_speed_frac', 0.35)  

        self.declare_parameter('steer_gain',     2.0)   
        self.declare_parameter('max_correction', 0.80)
        self.declare_parameter('clearance_m',    0.25) 
        self.declare_parameter('w_smooth_alpha', 0.40)

        scan_topic        = self.get_parameter('scan_topic').value
        input_topic       = self.get_parameter('input_topic').value
        output_topic      = self.get_parameter('output_topic').value
        self.threshold    = self.get_parameter('enable_threshold').value
        self.idle_timeout = self.get_parameter('idle_timeout').value
        self.laser_x      = self.get_parameter('laser_x_offset').value
        self.laser_y      = self.get_parameter('laser_y_offset').value
        self.robot_front  = self.get_parameter('robot_front_m').value
        self.robot_half_w = self.get_parameter('robot_half_w').value
        self.free_dist    = self.get_parameter('free_dist_m').value
        self.guard_dist   = self.get_parameter('guard_dist_m').value
        self.min_speed    = self.get_parameter('min_speed_frac').value
        self.steer_gain   = self.get_parameter('steer_gain').value
        self.max_corr     = self.get_parameter('max_correction').value
        self.clearance    = self.get_parameter('clearance_m').value
        self.w_alpha      = self.get_parameter('w_smooth_alpha').value

        self.cb_group = ReentrantCallbackGroup()

        # State
        self.histogram = None
        self.fwd_gap   = 999.0
        self.rear_gap  = 999.0
        self.left_gap  = 999.0
        self.right_gap = 999.0

        self.prev_w          = 0.0
        self.prev_best_angle = 0.0
        self.last_input_time = self.get_clock().now()
        self.is_active       = False

        self.scan_sub = self.create_subscription(
            LaserScan, scan_topic, self._scan_cb,
            qos_profile_sensor_data, callback_group=self.cb_group)
        self.joy_sub = self.create_subscription(
            Twist, input_topic, self._joy_cb,
            10, callback_group=self.cb_group)
        self.cmd_pub = self.create_publisher(
            Twist, output_topic, 10, callback_group=self.cb_group)
        self.create_timer(0.1, self._idle_check, callback_group=self.cb_group)

        self.get_logger().info(
            f"AssistedTeleop ready | scan={scan_topic} -> {output_topic} | "
            f"guard={self.guard_dist}m | max_corr={self.max_corr}rad/s")

    def _scan_cb(self, msg: LaserScan):
        n = len(msg.ranges)
        angles = msg.angle_min + np.arange(n, dtype=np.float32) * msg.angle_increment
        ranges = np.array(msg.ranges, dtype=np.float32)

        valid = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= 8.0)
        angles = angles[valid]
        ranges = ranges[valid]

        if len(ranges) == 0:
            self.histogram = None
            return

        bx = ranges * np.cos(angles) + self.laser_x
        by = ranges * np.sin(angles) + self.laser_y

        body_mask = ((bx > -0.60) & (bx < self.robot_front) &
                     (np.abs(by) < self.robot_half_w + 0.05))

        valid_obs = ~body_mask
        bx = bx[valid_obs]
        by = by[valid_obs]
      
        bp_angles = np.arctan2(by, bx)
        bp_dists  = np.sqrt(bx * bx + by * by)

        hist = np.full(self.N_BINS, 8.0, dtype=np.float32)
        indices = ((bp_angles + math.pi) / self.BIN_RAD).astype(int) % self.N_BINS
        np.minimum.at(hist, indices, bp_dists)

        k   = np.array([0.1, 0.2, 0.4, 0.2, 0.1], dtype=np.float32)
        ext = np.concatenate([hist[-2:], hist, hist[:2]])
        self.histogram = np.convolve(ext, k, mode='same')[2:-2]

        margin_y = self.robot_half_w - 0.02
        fwd_mask = ((bx > self.robot_front) & (bx < self.robot_front + 2.0) & (np.abs(by) < margin_y))
        self.fwd_gap = float(np.min(bx[fwd_mask]) - self.robot_front) if np.any(fwd_mask) else 999.0

        # 후방 거리
        rear_mask = ((bx < -0.40) & (bx > -2.40) & (np.abs(by) < margin_y))
        self.rear_gap = float(-0.40 - np.max(bx[rear_mask])) if np.any(rear_mask) else 999.0

        # 좌측면 거리
        left_mask = ((by > self.robot_half_w) & (by < self.robot_half_w + 1.0) &
                     (bx > -0.40) & (bx < self.robot_front))
        self.left_gap = float(np.min(by[left_mask]) - self.robot_half_w) if np.any(left_mask) else 999.0

        # 우측면 거리
        right_mask = ((by < -self.robot_half_w) & (by > -self.robot_half_w - 1.0) &
                      (bx > -0.40) & (bx < self.robot_front))
        self.right_gap = float(-self.robot_half_w - np.max(by[right_mask])) if np.any(right_mask) else 999.0
      
    def _idle_check(self):
        if not self.is_active:
            return
        elapsed = (self.get_clock().now() - self.last_input_time).nanoseconds / 1e9
        if elapsed > self.idle_timeout:
            self.get_logger().info(f"Idle {elapsed:.1f}s — released.")
            self.is_active = False
            self.prev_w = 0.0
            self.cmd_pub.publish(Twist())

    def _joy_cb(self, msg: Twist):
        active = abs(msg.linear.x) > self.threshold or abs(msg.angular.z) > self.threshold
        if active:
            self.last_input_time = self.get_clock().now()
            if not self.is_active:
                self.get_logger().info("Joystick active — VFH ADAS ON.")
                self.is_active = True
            self._adas(msg)
        else:
            if self.is_active:
                self.prev_w = 0.0
                self.cmd_pub.publish(Twist())

    def _publish(self, v: float, w: float, smooth: bool = True):
        if smooth:
            w = self.w_alpha * w + (1.0 - self.w_alpha) * self.prev_w
        self.prev_w = w

        out = Twist()
        out.linear.x  = max(-1.4, min(1.4, v))
        out.angular.z = max(-1.5, min(1.5, w))
        self.cmd_pub.publish(out)

    def _adas(self, user_cmd: Twist):
        u_v = user_cmd.linear.x
        u_w = user_cmd.angular.z

        v_out = u_v
        w_out = u_w

        run_vfh = u_v > self.threshold

        hist = self.histogram
        is_vfh_active_correction = False

        need_assist = (self.fwd_gap <= self.free_dist) or (self.left_gap < 0.50) or (self.right_gap < 0.50)

        if run_vfh and hist is not None and need_assist:
            SEARCH   = 18
            ROBOT_W  = 2 
            MAX_DIST = 1.5
            W_DIST   = 5.0
            W_ANGLE  = 4.5
            W_PREV   = 1.0

            best_score = -999.0
            best_angle = 0.0

            for delta in range(-SEARCH, SEARCH + 1):
                center = self.FWD_IDX + delta

                min_d = 8.0
                for b in range(center - ROBOT_W, center + ROBOT_W + 1):
                    d = hist[b % self.N_BINS]
                    if d < min_d:
                        min_d = d

                effective_dist = min_d - self.robot_front

                if effective_dist < self.clearance:
                    continue

                angle = delta * self.BIN_RAD

                dist_score  = min(max(effective_dist, 0.0), MAX_DIST) / MAX_DIST
                angle_ratio = abs(angle) / (math.pi / 2.0)
                score = (dist_score * W_DIST) - (angle_ratio * W_ANGLE)

                if abs(angle - self.prev_best_angle) < 0.15:
                    score += W_PREV

                if score > best_score:
                    best_score = score
                    best_angle = angle

            if best_score <= -999.0:
                v_out = 0.0
                self.get_logger().warning("All paths blocked.", throttle_duration_sec=0.3)
            else:
                self.prev_best_angle = best_angle

                correction = self.steer_gain * best_angle
                correction = max(-self.max_corr, min(self.max_corr, correction))
                w_out = u_w + correction

                t = (self.fwd_gap - self.guard_dist) / max(self.free_dist - self.guard_dist, 0.01)
                v_out = u_v * max(self.min_speed, min(1.0, t))
                v_out = v_out * max(0.40, math.cos(best_angle))

                is_vfh_active_correction = (abs(best_angle) > 0.01)

                dw = w_out - u_w
                if abs(dw) > 0.05 or v_out < u_v * 0.9:
                    side = "L" if best_angle > 0.05 else ("R" if best_angle < -0.05 else "F")
                    self.get_logger().info(
                        f"VFH {side} tgt={math.degrees(best_angle):+.0f}° v:{v_out:.2f}",
                        throttle_duration_sec=0.15)
        else:
            if run_vfh:
                self.prev_best_angle = 0.0

        stop_msgs = []
        is_guard  = False

        if v_out > 0.01 and self.fwd_gap < self.guard_dist:
            v_out = 0.0
            stop_msgs.append(f"FWD({self.fwd_gap:.2f})")
            is_guard = True
        elif v_out < -0.01 and self.rear_gap < self.guard_dist:
            v_out = 0.0
            stop_msgs.append(f"REAR({self.rear_gap:.2f})")
            is_guard = True
          
        SIDE_GUARD = 0.04

        if w_out > 0.01 and self.left_gap < SIDE_GUARD:
            w_out = 0.0
            stop_msgs.append(f"LEFT({self.left_gap:.2f})")
            is_guard = True
        elif w_out < -0.01 and self.right_gap < SIDE_GUARD:
            w_out = 0.0
            stop_msgs.append(f"RIGHT({self.right_gap:.2f})")
            is_guard = True

        if is_guard:
            self.get_logger().warning(f"GUARD STOP: {' '.join(stop_msgs)}", throttle_duration_sec=0.2)

        should_smooth = (is_vfh_active_correction and not is_guard)
        self._publish(v_out, w_out, smooth=should_smooth)


def main(args=None):
    rclpy.init(args=args)
    node = AssistedTeleopBridge()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
