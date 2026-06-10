"""Depth image → /depth/front_distance (Float32, metres).

Subscribes to RealSense aligned depth image, extracts the 5th-percentile
depth value inside a central ROI, and publishes it as a scalar distance.
This lets obstacle_detector_node use the camera as a 3rd confirmation
sensor alongside ToF and LiDAR without full PointCloud2 processing.
"""
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

BEST_EFFORT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class DepthBridgeNode(Node):

    def __init__(self):
        super().__init__('depth_bridge')

        self.declare_parameter('depth_topic',  '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('output_topic', '/depth/front_distance')
        self.declare_parameter('roi_h_frac',   0.4)   # 중앙 세로 비율
        self.declare_parameter('roi_w_frac',   0.4)   # 중앙 가로 비율
        self.declare_parameter('min_depth_m',  0.15)  # 최소 유효 거리 (m)
        self.declare_parameter('max_depth_m',  3.0)   # 최대 유효 거리 (m)
        self.declare_parameter('percentile',   5.0)   # 노이즈 제거용 퍼센타일
        self.declare_parameter('skip',         3)     # N번째 프레임만 처리

        depth_topic  = self.get_parameter('depth_topic').value
        output_topic = self.get_parameter('output_topic').value
        self._roi_h  = self.get_parameter('roi_h_frac').value
        self._roi_w  = self.get_parameter('roi_w_frac').value
        self._min_d  = int(self.get_parameter('min_depth_m').value * 1000)   # mm
        self._max_d  = int(self.get_parameter('max_depth_m').value * 1000)   # mm
        self._pct    = float(self.get_parameter('percentile').value)
        self._skip   = int(self.get_parameter('skip').value)
        self._count  = 0

        self.create_subscription(Image, depth_topic, self._depth_cb, BEST_EFFORT_QOS)
        self._pub = self.create_publisher(Float32, output_topic, 10)

        self.get_logger().info(
            f'depth_bridge: {depth_topic} → {output_topic} '
            f'| ROI {self._roi_w*100:.0f}%×{self._roi_h*100:.0f}% '
            f'| range {self._min_d/1000:.2f}–{self._max_d/1000:.1f}m')

    def _depth_cb(self, msg: Image):
        self._count += 1
        if self._count % self._skip != 0:
            return

        # Z16 depth image (uint16, mm)
        depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)

        # 중앙 ROI 추출
        h, w = msg.height, msg.width
        r0 = int(h * (0.5 - self._roi_h / 2))
        r1 = int(h * (0.5 + self._roi_h / 2))
        c0 = int(w * (0.5 - self._roi_w / 2))
        c1 = int(w * (0.5 + self._roi_w / 2))
        roi = depth[r0:r1, c0:c1]

        valid = roi[(roi >= self._min_d) & (roi < self._max_d)]

        if valid.size > 0:
            dist_m = float(np.percentile(valid, self._pct)) / 1000.0
        else:
            dist_m = float('inf')

        out = Float32()
        out.data = dist_m
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DepthBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
