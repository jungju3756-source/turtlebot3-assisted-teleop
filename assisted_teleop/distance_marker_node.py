import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Range
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

try:
    from sensor_msgs.msg import Image as DepthImage
    import struct
    _DEPTH_OK = True
except ImportError:
    _DEPTH_OK = False


class DistanceMarkerNode(Node):

    def __init__(self):
        super().__init__('distance_marker_node')

        self.tof_dist   = None
        self.depth_dist = None
        self.scan_data  = None

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data)
        self.tof_sub = self.create_subscription(
            Range, '/tof_distance', self._tof_cb, 10)
        self.depth_sub = self.create_subscription(
            DepthImage, '/camera/camera/aligned_depth_to_color/image_raw',
            self._depth_cb, qos_profile_sensor_data)

        self.marker_pub = self.create_publisher(MarkerArray, '/distance_markers', 10)
        self.create_timer(0.2, self._publish_markers)

        self.get_logger().info('distance_marker_node ready')

    # ── 콜백 ──────────────────────────────────────────────────────
    def _scan_cb(self, msg):
        self.scan_data = msg

    def _tof_cb(self, msg):
        self.tof_dist = msg.range

    def _depth_cb(self, msg):
        try:
            w, h = msg.width, msg.height
            cx = w // 2
            cy = h // 2
            roi = 40  # 중앙 80x80 px 영역
            raw = bytes(msg.data)
            dists = []
            for row in range(max(0, cy - roi), min(h, cy + roi)):
                for col in range(max(0, cx - roi), min(w, cx + roi)):
                    idx = (row * w + col) * 2
                    val = struct.unpack_from('<H', raw, idx)[0]
                    if val > 0:
                        dists.append(val / 1000.0)
            self.depth_dist = min(dists) if dists else None
        except Exception:
            pass

    # ── 마커 생성 헬퍼 ─────────────────────────────────────────────
    def _line_marker(self, mid, x_end, y_end, r, g, b, ns):
        m = Marker()
        m.header.frame_id = 'base_footprint'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = mid
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.02
        m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 1.0
        m.lifetime.sec = 1
        p0 = Point(); p0.x = 0.0; p0.y = 0.0; p0.z = 0.05
        p1 = Point(); p1.x = x_end; p1.y = y_end; p1.z = 0.05
        m.points = [p0, p1]
        return m

    def _text_marker(self, mid, x, y, text, r, g, b, ns):
        m = Marker()
        m.header.frame_id = 'base_footprint'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns + '_txt'
        m.id = mid
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.20
        m.scale.z = 0.10
        m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 1.0
        m.text = text
        m.lifetime.sec = 1
        return m

    # ── 라이다: 전/후/좌/우 최근접 거리 ──────────────────────────
    def _scan_sector_min(self, msg, angle_center_deg, half_deg=20.0):
        n = len(msg.ranges)
        results = []
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min or r > 4.0:
                continue
            ang_deg = math.degrees(msg.angle_min + i * msg.angle_increment)
            diff = (ang_deg - angle_center_deg + 180) % 360 - 180
            if abs(diff) <= half_deg:
                x = r * math.cos(math.radians(ang_deg))
                y = r * math.sin(math.radians(ang_deg))
                results.append((r, x, y))
        if not results:
            return None
        return min(results, key=lambda v: v[0])

    # ── 메인 퍼블리시 ──────────────────────────────────────────────
    def _publish_markers(self):
        markers = []
        mid = 0

        # ── 라이다 4방향 ─────────────────────────────────────────
        if self.scan_data is not None:
            sectors = [
                (0,   'FRONT', 1.0, 0.3, 0.3),   # 빨강
                (180, 'REAR',  0.3, 0.3, 1.0),   # 파랑
                (90,  'LEFT',  0.3, 1.0, 0.3),   # 초록
                (-90, 'RIGHT', 1.0, 1.0, 0.3),   # 노랑
            ]
            for ang, label, r, g, b in sectors:
                res = self._scan_sector_min(self.scan_data, ang)
                if res:
                    dist, x, y = res
                    markers.append(self._line_marker(mid, x, y, r, g, b, 'lidar'))
                    markers.append(self._text_marker(
                        mid, x, y, f'{label}\n{dist:.2f}m', r, g, b, 'lidar'))
                    mid += 1

        # ── ToF 전방 ─────────────────────────────────────────────
        if self.tof_dist is not None:
            d = self.tof_dist
            markers.append(self._line_marker(mid, d, 0.0, 1.0, 0.5, 0.0, 'tof'))
            markers.append(self._text_marker(
                mid, d + 0.05, 0.08, f'ToF\n{d:.2f}m', 1.0, 0.5, 0.0, 'tof'))
            mid += 1

        # ── 뎁스 카메라 전방 ──────────────────────────────────────
        if self.depth_dist is not None:
            d = self.depth_dist
            markers.append(self._line_marker(mid, d, 0.05, 0.5, 0.0, 1.0, 'depth'))
            markers.append(self._text_marker(
                mid, d + 0.05, 0.15, f'Depth\n{d:.2f}m', 0.5, 0.0, 1.0, 'depth'))
            mid += 1

        if markers:
            arr = MarkerArray()
            arr.markers = markers
            self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = DistanceMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
