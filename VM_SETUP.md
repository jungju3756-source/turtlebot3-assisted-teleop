# VM 시각화 및 TTS 실행 가이드

## 환경 정보
- Pi IP: 192.168.0.36 (ROS2 Jazzy, aarch64)
- VM IP: 192.168.0.72 (Ubuntu 24.04, x86_64)
- ROS_DOMAIN_ID: 40

---

## VM 기본 설정 (.bashrc에 추가)

```bash
source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash
export ROS_DOMAIN_ID=40
export TURTLEBOT3_MODEL=burger
```

---

## Pi에서 실행 (한 번에)

```bash
source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash
ros2 launch assisted_teleop full_system_launch.py
```

> 기본값: `joy_type:=8bitdo_micro`  
> Xbox 컨트롤러 사용 시: `ros2 launch assisted_teleop full_system_launch.py joy_type:=xbox`

이 명령 하나로 아래 4가지가 동시에 실행됩니다:
- TurtleBot3 Bringup (모터 + 라이다)
- 조이스틱 + ToF + 장애물 회피
- RealSense D405 뎁스 카메라
- 포인트클라우드 변환 노드 (`/camera/points`)

---

## Pi에서 개별 실행 (문제 발생 시)

### 터미널 1 — TurtleBot3 Bringup
```bash
source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_bringup robot.launch.py
```

### 터미널 2 — 조이스틱 + ToF + 장애물 회피
```bash
source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash
ros2 launch assisted_teleop assisted_teleop_launch.py joy_type:=8bitdo_micro
```

### 터미널 3 — RealSense D405 (뎁스 카메라)
```bash
source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash
ros2 launch realsense2_camera rs_launch.py \
  align_depth.enable:=true \
  depth_module.depth_profile:=848x480x30 \
  rgb_camera.color_profile:=848x480x30
```

### 터미널 4 — 포인트클라우드 변환 노드
```bash
source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash
ros2 run assisted_teleop depth_pointcloud_node
```

---

## VM에서 실행 순서

### 터미널 1 — TTS 음성 알림 (ToF 장애물 감지)
```bash
source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash
ros2 run voice_alert voice_alert_node
```
> 장애물 0.4m 이내 감지 시 "장애물이 감지되었습니다" 음성 출력

### 터미널 2 — RViz2 시각화
```bash
source /opt/ros/jazzy/setup.bash
rviz2
```

---

## RViz2 설정

### 라이다 단독 시각화
1. **Fixed Frame** → `base_scan`
2. Add → **LaserScan**
   - Topic: `/scan`
   - Color Transformer: `Intensity` 또는 `Flat Color`
   - Size: `0.03`

### 뎁스 포인트클라우드 단독 시각화
1. **Fixed Frame** → `camera_color_optical_frame`
2. Add → **PointCloud2**
   - Topic: `/camera/points`
   - Color Transformer: `AxisColor` (Axis: Z)
   - Size: `0.01`

### ToF 히트맵 시각화
1. Add → **Image**
   - Topic: `/tof_image`
   - (Fixed Frame 무관)

### 전체 통합 (라이다 + 뎁스 + ToF 동시)
1. **Fixed Frame** → `base_footprint`
2. Add → **LaserScan** → `/scan`
3. Add → **PointCloud2** → `/camera/points`
4. Add → **Image** → `/tof_image`
5. Add → **Image** → `/camera/camera/color/image_raw` (RGB 영상, 선택)

> 통합 시각화는 TF 연결이 필요합니다.  
> `base_footprint` 기준으로 `base_scan`, `camera_color_optical_frame` TF가 bringup에서 broadcast됩니다.

---

## 토픽 확인 명령

```bash
# VM에서 Pi 토픽이 보이는지 확인
export ROS_DOMAIN_ID=40
ros2 topic hz /scan                  # 라이다 (5Hz)
ros2 topic hz /camera/points         # 뎁스 포인트클라우드
ros2 topic hz /tof_image             # ToF 히트맵
ros2 topic hz /tof_distance          # ToF 거리값
```

---

## 8BitDo Micro 조작법

| 입력 | 동작 |
|------|------|
| D-pad ↑ | 전진 (0.20 m/s) |
| D-pad ↓ | 후진 |
| D-pad ← | 좌회전 |
| D-pad → | 우회전 |
| R 버튼 + D-pad | 터보 (0.22 m/s) |

---

## 블루투스 컨트롤러 재연결

8BitDo Micro가 자동 연결 안 될 경우:
```bash
bluetoothctl connect E4:17:D8:E6:37:F9
```
