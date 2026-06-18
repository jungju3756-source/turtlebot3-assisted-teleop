"""Distance HUD — ToF/Depth/LiDAR 전방 거리를 화면 코너 이미지로 퍼블리시."""

import math
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Float32

# ToF(중앙 2x2)·Depth 가 보는 "정면 중앙"에 맞춤.
# VL53L8CX ~45° FoV / 8존 = 존당 ~5.6°, 중앙 2x2 ≈ ±6° → LiDAR 도 거의 똑바로 앞만 본다.
LIDAR_SECTOR = math.radians(6.0)    # ±6° 섹터 (ToF 중앙 콘에 정렬)
# LiDAR 0°(스캔 기준 정면)는 카메라가 보는 방향과 180° 반대로 장착됨.
# → ToF/Depth 가 보는 정면 = LiDAR 좌표계의 180° 방향이므로 섹터 중심을 π 로 둔다.
LIDAR_FORWARD = math.pi             # 카메라 정면에 해당하는 LiDAR 각도
LIDAR_MIN_R  = 0.12                  # LDS-02 최소 유효 거리 (m)
# LiDAR 는 ToF/Depth 보다 5cm 뒤에 장착됨 → 같은 물체를 5cm 더 멀게 측정.
# 표시값을 ToF/Depth 기준면에 맞추기 위해 측정거리에서 5cm 차감.
LIDAR_OFFSET = 0.05                  # m


class DistanceHudNode(Node):

    def __init__(self):
        super().__init__('distance_hud_node')
        self._tof   = float('inf')
        self._depth = float('inf')
        self._lidar = float('inf')

        self.create_subscription(Float32, '/tof/front_distance',   self._tof_cb,   10)
        self.create_subscription(Float32, '/depth/front_distance', self._depth_cb, 10)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data)

        self._pub = self.create_publisher(Image, '/distance_hud', 10)
        self.create_timer(0.1, self._publish)
        self.get_logger().info('distance_hud_node ready → /distance_hud')

    def _tof_cb(self, msg: Float32):
        self._tof = float(msg.data)

    def _depth_cb(self, msg: Float32):
        self._depth = float(msg.data)

    def _scan_cb(self, msg: LaserScan):
        if not msg.ranges:
            self._lidar = float('inf')
            return
        rmin = max(LIDAR_MIN_R, msg.range_min)
        rmax = msg.range_max if msg.range_max > 0 else float('inf')
        best = float('inf')
        ang  = msg.angle_min
        for r in msg.ranges:
            # 카메라 정면(LIDAR_FORWARD=π) 기준 각도차를 [-π,π] 로 정규화
            d = math.atan2(math.sin(ang - LIDAR_FORWARD), math.cos(ang - LIDAR_FORWARD))
            if abs(d) <= LIDAR_SECTOR and math.isfinite(r) and rmin <= r <= rmax:
                if r < best:
                    best = r
            ang += msg.angle_increment
        # 5cm 뒤 장착분 보정 (ToF/Depth 기준면 정렬)
        self._lidar = (best - LIDAR_OFFSET) if math.isfinite(best) else best

    def _publish(self):
        img = np.zeros((165, 340, 3), dtype=np.uint8)

        tof_val   = f'{self._tof:.2f} m'   if math.isfinite(self._tof)   else '  --'
        depth_val = f'{self._depth:.2f} m' if math.isfinite(self._depth) else '  --'
        lidar_val = f'{self._lidar:.2f} m' if math.isfinite(self._lidar) else '  --'

        cv2.putText(img, f'ToF   : {tof_val}',   (12, 45),  cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 150,  50), 2)
        cv2.putText(img, f'Depth : {depth_val}', (12, 95),  cv2.FONT_HERSHEY_SIMPLEX, 0.9, (150,  80, 255), 2)
        cv2.putText(img, f'LiDAR : {lidar_val}', (12, 145), cv2.FONT_HERSHEY_SIMPLEX, 0.9, ( 80, 255, 120), 2)

        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        out = Image()
        out.header.stamp    = self.get_clock().now().to_msg()
        out.header.frame_id = ''
        out.height, out.width = rgb.shape[:2]
        out.encoding     = 'rgb8'
        out.is_bigendian = False
        out.step         = out.width * 3
        out.data         = rgb.tobytes()
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DistanceHudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
