"""YOLO 객체 감지 노드 — RealSense 컬러 영상에서 학습 모델(best.pt)로 객체 감지.

입력:  /camera/camera/color/image_raw/compressed (CompressedImage, CamPi)
출력:  /yolo/image      (Image, 바운딩박스 그려진 영상 → RViz2 Image 디스플레이)
       /yolo/detections (String, JSON: [{cls, conf, x1, y1, x2, y2}, ...])

추론은 별도 스레드에서 최신 프레임만 처리 (프레임 밀림 방지).
"""

import json
import threading
import time

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String


class YoloDetectNode(Node):

    def __init__(self):
        super().__init__('yolo_detect_node')

        self.declare_parameter('model_path', '/home/jungju/turtlebot3_ws/best.pt')
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw/compressed')
        self.declare_parameter('use_compressed', True)
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('imgsz', 320)
        self.declare_parameter('output_width', 424)

        model_path     = self.get_parameter('model_path').value
        image_topic    = self.get_parameter('image_topic').value
        use_compressed = self.get_parameter('use_compressed').value
        self.conf      = float(self.get_parameter('conf_threshold').value)
        self.imgsz     = int(self.get_parameter('imgsz').value)
        self.out_w     = int(self.get_parameter('output_width').value)

        self.img_pub = self.create_publisher(Image, '/yolo/image', 10)
        self.det_pub = self.create_publisher(String, '/yolo/detections', 10)

        self._latest_frame = None
        self._lock = threading.Lock()

        try:
            from ultralytics import YOLO
            self.model = YOLO(model_path)
            self.get_logger().info(f'YOLO model loaded: {model_path}')
        except Exception as e:
            self.get_logger().error(f'YOLO load failed: {e}')
            self.model = None
            return

        if use_compressed:
            self.create_subscription(
                CompressedImage, image_topic, self._compressed_cb,
                qos_profile_sensor_data)
        else:
            self.create_subscription(
                Image, image_topic, self._raw_cb,
                qos_profile_sensor_data)

        t = threading.Thread(target=self._inference_loop, daemon=True)
        t.start()
        self.get_logger().info(
            f'yolo_detect started | topic={image_topic} conf={self.conf}')

    # ── 입력 콜백: 최신 프레임만 보관 ──────────────────────────────
    def _compressed_cb(self, msg: CompressedImage):
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if frame is not None:
            with self._lock:
                self._latest_frame = frame

    def _raw_cb(self, msg: Image):
        if msg.encoding not in ('rgb8', 'bgr8'):
            return
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3)
        if msg.encoding == 'rgb8':
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        with self._lock:
            self._latest_frame = frame.copy()

    # ── 추론 루프 (최신 프레임만, 밀림 없음) ──────────────────────
    def _inference_loop(self):
        while rclpy.ok():
            with self._lock:
                frame = self._latest_frame
                self._latest_frame = None
            if frame is None:
                time.sleep(0.05)
                continue
            try:
                self._process(frame)
            except Exception as e:
                self.get_logger().warning(f'inference error: {e}',
                                          throttle_duration_sec=5.0)

    def _process(self, frame_bgr):
        results = self.model.predict(
            frame_bgr, conf=self.conf, imgsz=self.imgsz, verbose=False)
        r = results[0]

        annotated = r.plot()  # BGR, 박스+라벨 그려짐

        dets = []
        for box in r.boxes:
            x1, y1, x2, y2 = [float(v) for v in box.xyxy[0]]
            dets.append({
                'cls':  r.names[int(box.cls[0])],
                'conf': round(float(box.conf[0]), 3),
                'x1': round(x1), 'y1': round(y1),
                'x2': round(x2), 'y2': round(y2),
            })

        now = self.get_clock().now().to_msg()

        h, w = annotated.shape[:2]
        out_h = int(h * self.out_w / w)
        annotated = cv2.resize(annotated, (self.out_w, out_h))
        rgb = cv2.cvtColor(annotated, cv2.COLOR_BGR2RGB)
        img = Image()
        img.header.stamp = now
        img.header.frame_id = 'camera_color_frame'
        img.height, img.width = rgb.shape[:2]
        img.encoding = 'rgb8'
        img.is_bigendian = False
        img.step = img.width * 3
        img.data = rgb.tobytes()
        self.img_pub.publish(img)

        s = String()
        s.data = json.dumps(dets)
        self.det_pub.publish(s)

        if dets:
            self.get_logger().info(
                f'detected: {[(d["cls"], d["conf"]) for d in dets]}',
                throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectNode()
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
