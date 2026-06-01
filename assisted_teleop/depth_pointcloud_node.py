import struct
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField


VOLATILE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


class DepthPointCloudNode(Node):

    def __init__(self):
        super().__init__('depth_pointcloud_node')

        self.declare_parameter('depth_topic',  '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('info_topic',   '/camera/camera/aligned_depth_to_color/camera_info')
        self.declare_parameter('output_topic', '/camera/points')
        self.declare_parameter('skip',         2)   # publish every N-th frame to reduce load

        depth_topic  = self.get_parameter('depth_topic').value
        info_topic   = self.get_parameter('info_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.skip    = self.get_parameter('skip').value

        self._fx = self._fy = self._cx = self._cy = None
        self._frame_count = 0

        self.create_subscription(CameraInfo, info_topic,  self._info_cb,  VOLATILE_QOS)
        self.create_subscription(Image,      depth_topic, self._depth_cb, VOLATILE_QOS)
        self._pub = self.create_publisher(PointCloud2, output_topic, 5)

        self.get_logger().info(
            f'depth_pointcloud_node 시작 | depth={depth_topic} → {output_topic}')

    def _info_cb(self, msg: CameraInfo):
        if self._fx is None:
            self.get_logger().info(f'카메라 정보 수신: fx={msg.k[0]:.1f}')
        self._fx = msg.k[0]
        self._fy = msg.k[4]
        self._cx = msg.k[2]
        self._cy = msg.k[5]
        self._frame_id = msg.header.frame_id

    def _depth_cb(self, msg: Image):
        if self._fx is None:
            self.get_logger().warn('카메라 정보 미수신 — 대기 중', throttle_duration_sec=2.0)
            return

        self._frame_count += 1
        if self._frame_count % self.skip != 0:
            return

        # Z16 = uint16, depth in mm
        depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)

        # Pixel coordinates
        v, u = np.meshgrid(np.arange(msg.height), np.arange(msg.width), indexing='ij')

        z = depth.astype(np.float32) / 1000.0  # mm → m
        valid = (depth > 0) & (depth < 65000)

        x = np.where(valid, (u - self._cx) * z / self._fx, np.nan).astype(np.float32)
        y = np.where(valid, (v - self._cy) * z / self._fy, np.nan).astype(np.float32)
        z = np.where(valid, z, np.nan).astype(np.float32)

        points = np.stack([x, y, z], axis=-1).reshape(-1, 3)

        cloud = PointCloud2()
        cloud.header.stamp    = msg.header.stamp
        cloud.header.frame_id = self._frame_id
        cloud.height = 1
        cloud.width  = len(points)
        cloud.fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step   = 12
        cloud.row_step     = 12 * len(points)
        cloud.is_dense     = False
        cloud.data         = points.tobytes()

        self._pub.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = DepthPointCloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
