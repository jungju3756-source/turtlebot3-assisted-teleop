import re
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

try:
    import serial
except ImportError:
    serial = None


class TofSensorNode(Node):

    def __init__(self):
        super().__init__('tof_sensor_node')

        self.declare_parameter('serial_port',          '/dev/ttyACM1')
        self.declare_parameter('obstacle_dist_mm',     400)
        self.declare_parameter('front_rows',           [2, 3, 4, 5])
        self.declare_parameter('front_cols',           [2, 3, 4, 5])

        port       = self.get_parameter('serial_port').value
        self.thr   = self.get_parameter('obstacle_dist_mm').value
        self.f_rows = self.get_parameter('front_rows').value
        self.f_cols = self.get_parameter('front_cols').value

        self.pub = self.create_publisher(Range, '/tof_distance', 10)

        self.grid      = [[None] * 8 for _ in range(8)]
        self.in_frame  = False

        if serial is None:
            self.get_logger().error("pyserial not installed — run: pip3 install pyserial")
            return

        try:
            self.ser = serial.Serial(port, 115200, timeout=1.0)
            self.get_logger().info(f"ToF opened: {port} | threshold={self.thr}mm")
        except Exception as e:
            self.get_logger().error(f"Cannot open {port}: {e}")
            return

        t = threading.Thread(target=self._read_loop, daemon=True)
        t.start()

    # ------------------------------------------------------------------ #
    def _read_loop(self):
        while rclpy.ok():
            try:
                raw = self.ser.readline()
                line = raw.decode('utf-8', errors='ignore').strip()
                self._parse_line(line)
            except Exception:
                pass

    def _parse_line(self, line: str):
        if line.startswith('Frame #'):
            self.in_frame = True
            self.grid = [[None] * 8 for _ in range(8)]
            return

        if not self.in_frame:
            return

        m = re.match(r'^R(\d)\s+(.*)', line)
        if not m:
            return

        row_idx = int(m.group(1))
        parts   = m.group(2).split()
        values  = []
        for p in parts:
            if p == '----':
                values.append(None)
            else:
                try:
                    values.append(int(p))
                except ValueError:
                    values.append(None)

        if len(values) == 8:
            self.grid[row_idx] = values

        if row_idx == 7:
            self._process_frame()

    def _process_frame(self):
        dists = []
        for r in self.f_rows:
            for c in self.f_cols:
                v = self.grid[r][c]
                if v is not None and v > 0:
                    dists.append(v)

        if not dists:
            return

        min_mm = min(dists)

        msg = Range()
        msg.header.stamp       = self.get_clock().now().to_msg()
        msg.header.frame_id    = 'tof_sensor'
        msg.radiation_type     = Range.INFRARED
        msg.field_of_view      = 0.785
        msg.min_range          = 0.04
        msg.max_range          = 4.0
        msg.range              = min_mm / 1000.0

        self.pub.publish(msg)

        if min_mm < self.thr:
            self.get_logger().warning(
                f"ToF 장애물 감지: {min_mm}mm (임계값 {self.thr}mm)",
                throttle_duration_sec=0.3)


def main(args=None):
    rclpy.init(args=args)
    node = TofSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
