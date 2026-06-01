import subprocess
import time
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist


class VoiceAlertNode(Node):

    def __init__(self):
        super().__init__('voice_alert_node')

        self.declare_parameter('obstacle_dist_m',  0.4)
        self.declare_parameter('forward_threshold', 0.05)
        self.declare_parameter('cooldown_sec',     3.0)

        self.obs_thr   = self.get_parameter('obstacle_dist_m').value
        self.fwd_thr   = self.get_parameter('forward_threshold').value
        self.cooldown  = self.get_parameter('cooldown_sec').value

        self.last_alert_time = 0.0
        self.current_range   = float('inf')
        self.moving_forward  = False
        self._lock = threading.Lock()

        self.create_subscription(Range,  '/tof_distance', self._tof_cb,  10)
        self.create_subscription(Twist,  '/joy_vel',      self._joy_cb,  10)

        self.get_logger().info(
            f'voice_alert 시작 | 장애물:{self.obs_thr}m | 쿨다운:{self.cooldown}s')

    def _tof_cb(self, msg: Range):
        with self._lock:
            self.current_range = msg.range
            obstacle = msg.range < self.obs_thr

        if not obstacle:
            return

        now = time.monotonic()
        if now - self.last_alert_time < self.cooldown:
            return

        with self._lock:
            fwd = self.moving_forward

        if fwd:
            text = '전진 버튼을 누르지 마세요'
        else:
            text = '장애물이 감지되었습니다'

        self.last_alert_time = now
        self.get_logger().info(f'TTS: {text} ({msg.range:.2f}m)')
        threading.Thread(target=self._speak, args=(text,), daemon=True).start()

    def _joy_cb(self, msg: Twist):
        with self._lock:
            self.moving_forward = msg.linear.x > self.fwd_thr

    def _speak(self, text: str):
        try:
            subprocess.run(
                ['espeak-ng', '-v', 'ko', '-s', '130', '-a', '180', text],
                timeout=5,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except FileNotFoundError:
            self.get_logger().error('espeak-ng 없음. 설치: sudo apt install espeak-ng')
        except Exception as e:
            self.get_logger().error(f'TTS 오류: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = VoiceAlertNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
