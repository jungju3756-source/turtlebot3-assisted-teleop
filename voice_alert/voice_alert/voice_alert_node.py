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

        self.declare_parameter('obstacle_dist_m',  1.2)
        self.declare_parameter('forward_threshold', 0.05)
        self.declare_parameter('cooldown_sec',     3.0)

        self.obs_thr   = self.get_parameter('obstacle_dist_m').value
        self.fwd_thr   = self.get_parameter('forward_threshold').value
        self.cooldown  = self.get_parameter('cooldown_sec').value

        self.last_alert_time = 0.0
        self.current_range   = float('inf')
        self.moving_forward  = False
        self._lock = threading.Lock()

        self.declare_parameter('vel_topic', '/cmd_vel')   # 실제 전진 속도 (/joy_vel 은 미발행)
        vel_topic = self.get_parameter('vel_topic').value

        self.create_subscription(Range,  '/tof_distance', self._tof_cb,  10)
        self.create_subscription(Twist,  vel_topic,       self._joy_cb,  10)

        self.get_logger().info(
            f'voice_alert start | obstacle:{self.obs_thr}m | cooldown:{self.cooldown}s '
            f'| 전진 중일 때만 경보 (vel:{vel_topic})')

    def _tof_cb(self, msg: Range):
        with self._lock:
            self.current_range = msg.range
            obstacle = msg.range < self.obs_thr

        if not obstacle:
            return

        with self._lock:
            fwd = self.moving_forward

        # 전진 중일 때만 경보 (정지/후진 시에는 침묵)
        if not fwd:
            return

        now = time.monotonic()
        if now - self.last_alert_time < self.cooldown:
            return

        text = 'Obstacle ahead, stop'
        self.last_alert_time = now
        self.get_logger().info(f'TTS: {text} ({msg.range:.2f}m)')
        threading.Thread(target=self._speak, args=(text,), daemon=True).start()

    def _joy_cb(self, msg: Twist):
        with self._lock:
            self.moving_forward = msg.linear.x > self.fwd_thr

    def _speak(self, text: str):
        try:
            subprocess.run(
                ['espeak-ng', '-v', 'en', '-s', '150', '-a', '180', text],
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
