"""
VFH-lite + 4-Way Absolute Guard Stop
═══════════════════════════════════════════════════════════════════
[개선점]
  1. 깊이 확장 & 히스테리시스: 넓은 구역 선호 및 좁은 구석(corner) 회피
  2. 회전 상한(max_correction): 과도한 오버스티어 방지
  3. 4방향 철통 방어 (Guard Stop): 후진, 제자리 회전 시에도 좌, 우, 후방 장애물 감지하여 절대 충돌 차단
  4. 후진 버그 수정: 속도 하한을 0.0에서 -1.4m/s로 복원하여 후진 정상 작동

[포팅 이력]
  - v1.0: Orange Pi 5 + 스쿠터 플랫폼 (LiDAR 위치 오프셋, 큰 footprint)
  - v1.1: scan_topic / output_topic 파라미터화, TurtleBot3 기본값 적용
"""

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

    N_BINS  = 72                        # 360° / 5° = 72 bins
    BIN_RAD = 2.0 * math.pi / N_BINS    # ~0.087 rad per bin (5°)
    FWD_IDX = N_BINS // 2               # bin 36 = 0° = forward

    def __init__(self):
        super().__init__('assisted_teleop_bridge')

        # ── 토픽 ──────────────────────────────────────────────────────────────
        # TurtleBot3: scan_topic='/scan', output_topic='/cmd_vel'
        # 스쿠터(Orange Pi): scan_topic='/scan_filtered', output_topic='/cmd_vel_assisted_teleop'
        self.declare_parameter('scan_topic',   '/scan')
        self.declare_parameter('input_topic',  '/joy_vel')
        self.declare_parameter('output_topic', '/cmd_vel')

        self.declare_parameter('enable_threshold', 0.05)
        self.declare_parameter('idle_timeout',      0.5)

        # ── 로봇 Footprint ────────────────────────────────────────────────────
        # TurtleBot3 Burger 기본값:
        #   laser_x_offset=0.0, laser_y_offset=0.0 (LiDAR가 중앙에 위치)
        #   robot_front_m=0.17, robot_half_w=0.09
        #
        # 스쿠터(Orange Pi) 값 (참고):
        #   laser_x_offset=0.44, laser_y_offset=0.24
        #   robot_front_m=0.75,  robot_half_w=0.40
        self.declare_parameter('laser_x_offset', 0.0)
        self.declare_parameter('laser_y_offset', 0.0)
        self.declare_parameter('robot_front_m',  0.17)
        self.declare_parameter('robot_half_w',   0.09)

        # ── 제동 거리 (범퍼 기준) ──────────────────────────────────────────────
        # TurtleBot3 기본값 (스쿠터: free=1.00, guard=0.15)
        self.declare_parameter('free_dist_m',    0.50)  # 이상: 100% 투과
        self.declare_parameter('guard_dist_m',   0.10)  # 이하: 긴급 정지
        self.declare_parameter('min_speed_frac', 0.35)  # 회피 중 최소 속도 비율

        # ── VFH ───────────────────────────────────────────────────────────────
        # TurtleBot3 기본값 (스쿠터: clearance=0.55)
        self.declare_parameter('steer_gain',     2.0)   # 조향 강도 (rad/s per rad)
        self.declare_parameter('max_correction', 0.80)  # 급회전 방지 최대 조향량
        self.declare_parameter('clearance_m',    0.25)  # 방향 통과 최소 거리
        self.declare_parameter('w_smooth_alpha', 0.40)  # 각속도 스무딩 계수

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

    # ─────────────────────────────────────────────────────────────────────────
    def _scan_cb(self, msg: LaserScan):
        """
        LiDAR -> base_footprint 프레임 변환 후:
          (1) 72-bin 극좌표 히스토그램 생성
          (2) 전, 후, 좌, 우 4방향 최단 이격 거리 계산 (가드스톱용)
        """
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

        # 로봇 footprint 내부 LiDAR 노이즈 필터링
        # (스쿠터: 팔걸이·사용자 다리가 히스토그램을 막아 특정 방향 회피 불가 현상 방지용)
        body_mask = ((bx > -0.60) & (bx < self.robot_front) &
                     (np.abs(by) < self.robot_half_w + 0.05))

        valid_obs = ~body_mask
        bx = bx[valid_obs]
        by = by[valid_obs]

        # --- 극좌표 히스토그램 ---
        bp_angles = np.arctan2(by, bx)
        bp_dists  = np.sqrt(bx * bx + by * by)

        hist = np.full(self.N_BINS, 8.0, dtype=np.float32)
        indices = ((bp_angles + math.pi) / self.BIN_RAD).astype(int) % self.N_BINS
        np.minimum.at(hist, indices, bp_dists)

        k   = np.array([0.1, 0.2, 0.4, 0.2, 0.1], dtype=np.float32)
        ext = np.concatenate([hist[-2:], hist, hist[:2]])
        self.histogram = np.convolve(ext, k, mode='same')[2:-2]

        # 전면 거리 (base_link 기준, 통로 평행벽 오인 방지로 폭 -2cm 좁게 잡음)
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

    # ─────────────────────────────────────────────────────────────────────────
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

    # ─────────────────────────────────────────────────────────────────────────
    def _adas(self, user_cmd: Twist):
        u_v = user_cmd.linear.x
        u_w = user_cmd.angular.z

        v_out = u_v
        w_out = u_w

        # VFH 회피 로직은 전진 시에만 구동
        run_vfh = u_v > self.threshold

        hist = self.histogram
        is_vfh_active_correction = False

        need_assist = (self.fwd_gap <= self.free_dist) or (self.left_gap < 0.50) or (self.right_gap < 0.50)

        if run_vfh and hist is not None and need_assist:
            SEARCH   = 18   # ±18 bins = ±90°
            ROBOT_W  = 2    # ±2 bins = ±10°
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

                # 범퍼 기준 유효 거리 (robot_front를 빼서 base_link → 범퍼 기준으로 통일)
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

        # --- 4방향 절대 철통 방어 (Guard Stop) ---
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

        # 측면 가드스톱은 4cm로 분리 (차선 라인 유지 허용)
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

        # VFH가 능동 개입한 경우에만 스무딩 적용 (순수 수동 조작은 즉각 응답)
        should_smooth = (is_vfh_active_correction and not is_guard)
        self._publish(v_out, w_out, smooth=should_smooth)


# ─────────────────────────────────────────────────────────────────────────────
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
