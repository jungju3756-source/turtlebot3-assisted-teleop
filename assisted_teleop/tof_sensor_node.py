import re
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range, Image
from std_msgs.msg import Float32MultiArray, UInt8MultiArray

try:
    import serial
except ImportError:
    serial = None

try:
    from PIL import Image as PILImage, ImageDraw, ImageFont
    _PIL_OK = True
except ImportError:
    _PIL_OK = False


class TofSensorNode(Node):

    def __init__(self):
        super().__init__('tof_sensor_node')

        self.declare_parameter('serial_port',          '/dev/ttyACM1')
        self.declare_parameter('obstacle_dist_mm',     400)
        self.declare_parameter('front_rows',           [2, 3, 4, 5])
        self.declare_parameter('front_cols',           [2, 3, 4, 5])
        self.declare_parameter('image_scale',          20)
        self.declare_parameter('distance_scale',       10.0)  # cm→mm (Pico 출력이 cm인 경우)

        port        = self.get_parameter('serial_port').value
        self.thr    = self.get_parameter('obstacle_dist_mm').value
        self.f_rows = self.get_parameter('front_rows').value
        self.f_cols = self.get_parameter('front_cols').value
        self.scale  = self.get_parameter('image_scale').value
        self.dist_scale = self.get_parameter('distance_scale').value

        self.pub          = self.create_publisher(Range, '/tof_distance', 10)
        self.img_pub      = self.create_publisher(Image, '/tof_image', 10)
        self.dist_pub   = self.create_publisher(Float32MultiArray, '/tof/distances', 10)
        self.status_pub = self.create_publisher(UInt8MultiArray, '/tof/status', 10)

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
                    values.append(int(round(int(p) * self.dist_scale)))
                except ValueError:
                    values.append(None)

        if len(values) == 8:
            self.grid[row_idx] = values

        if row_idx == 7:
            self._process_frame()

    def _dist_to_color(self, dist_mm):
        """Jet colormap: red=close(0mm) → green → blue=far(2000mm), gray=invalid"""
        if dist_mm is None or dist_mm <= 0:
            return (60, 60, 60)
        d = min(float(dist_mm), 2000.0)
        ratio = d / 2000.0
        h = ratio * 240.0
        h6 = h / 60.0
        i = int(h6)
        f = h6 - i
        if i == 0:
            r, g, b = 255, int(255 * f), 0
        elif i == 1:
            r, g, b = int(255 * (1 - f)), 255, 0
        elif i == 2:
            r, g, b = 0, 255, int(255 * f)
        elif i == 3:
            r, g, b = 0, int(255 * (1 - f)), 255
        elif i == 4:
            r, g, b = int(255 * f), 0, 255
        else:
            r, g, b = 255, 0, int(255 * (1 - f))
        return (r, g, b)

    def _publish_image(self, stamp):
        s = max(self.scale, 50)  # 숫자 표시를 위해 최소 50px
        h, w = 8 * s, 8 * s

        if _PIL_OK:
            pil = PILImage.new('RGB', (w, h))
            draw = ImageDraw.Draw(pil)
            try:
                font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf', 11)
            except Exception:
                font = ImageFont.load_default()

            for row in range(8):
                for col in range(8):
                    r, g, b = self._dist_to_color(self.grid[row][col])
                    is_front = (row in self.f_rows and col in self.f_cols)
                    x0, y0 = col * s, row * s
                    x1, y1 = x0 + s - 1, y0 + s - 1
                    draw.rectangle([x0, y0, x1, y1], fill=(r, g, b))
                    if is_front:
                        draw.rectangle([x0, y0, x1, y1], outline=(255, 255, 255), width=2)

                    val = self.grid[row][col]
                    if val is not None and val > 0:
                        text = f'{val}'
                    else:
                        text = '----'
                    brightness = 0.299 * r + 0.587 * g + 0.114 * b
                    txt_color = (0, 0, 0) if brightness > 128 else (255, 255, 255)
                    bbox = draw.textbbox((0, 0), text, font=font)
                    tw = bbox[2] - bbox[0]
                    th = bbox[3] - bbox[1]
                    tx = x0 + (s - tw) // 2
                    ty = y0 + (s - th) // 2
                    draw.text((tx, ty), text, fill=txt_color, font=font)

            data = bytes(pil.tobytes())
        else:
            # PIL 없을 때 — 기존 컬러 히트맵
            data = bytearray(h * w * 3)
            for row in range(8):
                for col in range(8):
                    r, g, b = self._dist_to_color(self.grid[row][col])
                    is_front = (row in self.f_rows and col in self.f_cols)
                    for sr in range(s):
                        for sc in range(s):
                            if is_front and (sr == 0 or sr == s - 1 or sc == 0 or sc == s - 1):
                                pr, pg, pb = 255, 255, 255
                            else:
                                pr, pg, pb = r, g, b
                            px = ((row * s + sr) * w + (col * s + sc)) * 3
                            data[px] = pr
                            data[px + 1] = pg
                            data[px + 2] = pb
            data = bytes(data)

        img = Image()
        img.header.stamp = stamp
        img.header.frame_id = 'tof_sensor'
        img.height = h
        img.width = w
        img.encoding = 'rgb8'
        img.is_bigendian = False
        img.step = w * 3
        img.data = data
        self.img_pub.publish(img)

    def _process_frame(self):
        now = self.get_clock().now().to_msg()

        dists = []
        for r in self.f_rows:
            for c in self.f_cols:
                v = self.grid[r][c]
                if v is not None and v > 0:
                    dists.append(v)

        if dists:
            min_mm = min(dists)
            msg = Range()
            msg.header.stamp    = now
            msg.header.frame_id = 'tof_sensor'
            msg.radiation_type  = Range.INFRARED
            msg.field_of_view   = 0.785
            msg.min_range       = 0.04
            msg.max_range       = 4.0
            msg.range           = min_mm / 1000.0
            self.pub.publish(msg)

            if min_mm < self.thr:
                self.get_logger().warning(
                    f"ToF 장애물 감지: {min_mm}mm (임계값 {self.thr}mm)",
                    throttle_duration_sec=0.3)

        self._publish_image(now)
        self._publish_grid(now)

    def _publish_grid(self, stamp):
        flat_dist   = []
        flat_status = []
        for row in range(8):
            for col in range(8):
                v = self.grid[row][col]
                if v is not None and v > 0:
                    flat_dist.append(float(v))
                    flat_status.append(5)     # VL53L8CX valid
                else:
                    flat_dist.append(float('nan'))
                    flat_status.append(255)   # invalid

        d_msg = Float32MultiArray()
        d_msg.data = flat_dist
        self.dist_pub.publish(d_msg)

        s_msg = UInt8MultiArray()
        s_msg.data = flat_status
        self.status_pub.publish(s_msg)


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
