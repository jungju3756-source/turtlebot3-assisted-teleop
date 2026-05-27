# turtlebot3-assisted-teleop

ROS 2 Jazzy 기반 TurtleBot3 Burger 보조 텔레오퍼레이션 패키지.

Xbox 컨트롤러로 조종하면서 LiDAR(VFH-lite)와 ToF 센서(VL53L8CX)가 자동으로 장애물을 회피하고 충돌 직전 강제 정지합니다.

---

## 시스템 구성

```
Xbox 컨트롤러 (BT 동글)
        ↓
    joy_node → teleop_twist_joy → /joy_vel
                                        ↓
                           assisted_teleop_bridge
                           ├── /scan (LiDAR, VFH-lite 회피)
                           ├── /tof_distance (ToF Guard Stop)
                           └── Guard Stop: 전/후/좌/우 + ToF
                                        ↓
                                   /cmd_vel → TurtleBot3
```

---

## 하드웨어

| 장치 | 용도 |
|------|------|
| Raspberry Pi 5 | 메인 컴퓨터 |
| TurtleBot3 Burger | 로봇 플랫폼 |
| LDS-02 LiDAR | 360° 장애물 감지 (VFH) |
| VL53L8CX (RP2350 Zero) | 전방 ToF 근거리 감지 |
| Xbox Series X\|S 컨트롤러 | 조종 입력 (BT 동글 필수) |

---

## 소프트웨어 환경

- OS: Ubuntu 24.04 LTS (Raspberry Pi 5, aarch64)
- ROS 2: Jazzy Jalisco
- Python: 3.12

---

## 설치

### 1. 의존 패키지 설치

```bash
sudo apt install -y ros-jazzy-joy ros-jazzy-teleop-twist-joy
pip3 install pyserial
```

### 2. 워크스페이스에 클론

```bash
cd ~/turtlebot3_ws/src
git clone https://github.com/jungju3756-source/turtlebot3-assisted-teleop assisted-teleop
```

### 3. 빌드

```bash
cd ~/turtlebot3_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select assisted_teleop
source install/setup.bash
```

---

## 실행 방법

### 준비물 확인

- TurtleBot3 전원 ON (배터리)
- OpenCR micro-USB → Pi 연결 (`/dev/ttyACM0`)
- LiDAR USB → Pi 연결 (`/dev/ttyUSB0`)
- RP2350 Zero (VL53L8CX) USB → Pi 연결 (`/dev/ttyACM1`)
- Xbox 컨트롤러 → USB 블루투스 동글로 연결

### 터미널 1 — 로봇 bringup

```bash
ros2 launch turtlebot3_bringup robot.launch.py
```

아래 메시지가 나올 때까지 대기:
```
[turtlebot3_node]: Run!
LDS-02 started successfully
```

### 터미널 2 — 모터 파워 ON (매번 필수)

```bash
ros2 service call /motor_power std_srvs/srv/SetBool "{data: true}"
```

> 이 명령 없이는 cmd_vel을 보내도 바퀴가 돌지 않습니다.

### 터미널 3 — 조이스틱 + 장애물 회피

```bash
ros2 launch assisted_teleop assisted_teleop_launch.py joy_type:=xbox
```

성공 시 출력:
```
[joy_node]: Opened joystick: Xbox Series X Controller.
[tof_sensor_node]: ToF opened: /dev/ttyACM1 | threshold=400mm
[assisted_teleop_bridge]: AssistedTeleop ready
```

---

## 조종 방법 (Xbox 컨트롤러)

| 입력 | 동작 |
|------|------|
| 왼쪽 스틱 위 | 전진 (0.20 m/s) |
| 왼쪽 스틱 아래 | 후진 |
| 왼쪽 스틱 좌우 | 좌/우 회전 |
| RB (오른쪽 범퍼) | 터보 (0.22 m/s) |
| 스틱 놓으면 | 자동 정지 |

---

## 장애물 회피 동작

| 거리 | 동작 |
|------|------|
| 50cm 이상 | ADAS 미개입 |
| 10~50cm | 속도 감속 + VFH 자동 조향 |
| 10cm 이하 | LiDAR Guard Stop (강제 정지) |
| 40cm 이하 (ToF) | ToF Guard Stop (강제 정지) |

터미널 로그 예시:
```
[assisted_teleop_bridge]: VFH L tgt=+15° v:0.18
[tof_sensor_node]: ToF 장애물 감지: 253mm (임계값 400mm)
[assisted_teleop_bridge]: GUARD STOP: TOF(0.25m)
```

---

## 파라미터 튜닝

### `config/turtlebot3_params.yaml`

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `free_dist_m` | 0.50m | VFH 개입 시작 거리 |
| `guard_dist_m` | 0.10m | LiDAR 긴급 정지 거리 |
| `clearance_m` | 0.25m | VFH 통과 최소 폭 |
| `tof_guard_m` | 0.30m | ToF 긴급 정지 거리 |

### `launch/assisted_teleop_launch.py`

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `serial_port` | `/dev/ttyACM1` | RP2350 시리얼 포트 |
| `obstacle_dist_mm` | 400 | ToF 감지 임계값 (mm) |

---

## 파일 구조

```
assisted_teleop/
├── assisted_teleop/
│   ├── assisted_teleop_bridge.py   # VFH + Guard Stop + ToF 통합 노드
│   └── tof_sensor_node.py          # VL53L8CX 시리얼 읽기 → /tof_distance
├── config/
│   ├── turtlebot3_params.yaml      # 로봇 파라미터
│   ├── joystick_xbox.yaml          # Xbox 컨트롤러 설정
│   └── joystick_default.yaml       # 기본 조이스틱 설정
├── launch/
│   └── assisted_teleop_launch.py   # 통합 런치 파일
├── package.xml
└── setup.py
```
