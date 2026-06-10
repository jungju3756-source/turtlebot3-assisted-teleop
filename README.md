# TurtleBot3 Assisted Teleop — 구현 정리

## 시스템 블록도

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         Raspberry Pi 5  (192.168.0.36)                      │
│                                                                             │
│  ┌──────────────┐   /joy    ┌─────────────────────┐   /joy_vel (Twist)     │
│  │   joy_node   │──────────▶│ teleop_twist_joy    │──────────────┐         │
│  │  (joy pkg)   │           │      _node          │              │         │
│  └──────────────┘           └─────────────────────┘              ▼         │
│        ▲                                               ┌────────────────┐   │
│        │ /dev/input/js0                                │   assisted_    │   │
│  ┌─────┴──────┐                                        │ teleop_bridge  │   │
│  │ 8BitDo     │  Bluetooth                             │                │   │
│  │ Micro BT   │◀ ─ ─ ─ ─ ─ ─ hci0                    │  ┌──────────┐  │   │
│  └────────────┘                                        │  │VFH-lite │  │   │
│                                                        │  │장애물회피│  │   │
│  ┌──────────────┐  /dev/tof  ┌──────────────┐         │  └────┬─────┘  │   │
│  │ Pico 2 ToF   │──────────▶│ tof_sensor   │─────────▶│       │        │   │
│  │  (8x8 grid)  │  115200   │    _node     │/tof_dist │  ┌────▼─────┐  │   │
│  └──────────────┘           └──────┬───────┘          │  │  Guard   │  │   │
│                                    │ /tof_image        │  │  Stop    │  │   │
│                                    │                   │  └──────────┘  │   │
│  ┌──────────────┐  /dev/lidar ┌────▼──────────┐       └───────┬────────┘   │
│  │  LiDAR       │──────────▶│  ld08_driver   │               │            │
│  │  LDS-02      │            │               │──/scan─────────▶            │
│  └──────────────┘            └───────────────┘        /cmd_vel (TwistStamped)│
│                                                               │            │
│  ┌──────────────┐  /dev/opencr ┌─────────────────┐           │            │
│  │   OpenCR     │◀────────────│ turtlebot3_node  │◀──────────┘            │
│  │  (Dynamixel) │             │  (turtlebot3_node│                        │
│  └──────┬───────┘             │     pkg)         │──/odom, /joint_states  │
│         │                     └─────────────────┘                         │
│  ┌──────▼───────┐                                                          │
│  │ 좌/우 모터   │   /tf, /tf_static ◀── robot_state_publisher             │
│  │ (Dynamixel)  │                                                          │
│  └──────────────┘                                                          │
│                                                                             │
│  ┌─────────────────────────────────────────────────────┐                   │
│  │  ROS2 DDS (Fast DDS) — ROS_DOMAIN_ID=40             │                   │
│  │  토픽 브로드캐스트: /scan /tof_distance /tof_image   │                   │
│  │                    /cmd_vel /odom /tf /tf_static     │                   │
│  └─────────────────────────────────────────────────────┘                   │
└─────────────────────────────────────┬───────────────────────────────────────┘
                                      │  WiFi  (192.168.x.x LAN)
                                      │  ROS2 DDS 멀티캐스트
┌─────────────────────────────────────▼───────────────────────────────────────┐
│                     VM Ubuntu 24.04  (192.168.0.72)                         │
│                                                                             │
│  ┌───────────────────────┐       ┌──────────────────────────────────┐      │
│  │   voice_alert_node    │       │            RViz2                 │      │
│  │                       │       │                                  │      │
│  │  /tof_distance ──────▶│       │  Fixed Frame: base_footprint     │      │
│  │  range < 0.4m?        │       │                                  │      │
│  │       │               │       │  ┌────────────┐ /scan            │      │
│  │       ▼               │       │  │ LaserScan  │◀─────────────────│      │
│  │  "장애물이             │       │  │ (Squares,  │  Best Effort QoS │      │
│  │   감지되었습니다"      │       │  │  0.05m)    │                  │      │
│  │  (TTS espeak)         │       │  └────────────┘                  │      │
│  └───────────────────────┘       │  ┌────────────┐ /tof_image       │      │
│                                  │  │   Image    │◀─────────────────│      │
│                                  │  │ (8x8 히트맵│                  │      │
│                                  │  │  컬러맵)   │                  │      │
│                                  │  └────────────┘                  │      │
│                                  └──────────────────────────────────┘      │
└─────────────────────────────────────────────────────────────────────────────┘
```

## 시스템 개요

| 항목 | 내용 |
|------|------|
| 로봇 | TurtleBot3 Waffle |
| 컴퓨터 | Raspberry Pi 5 (ROS2 Jazzy, aarch64) |
| 시각화 VM | Ubuntu 24.04 VMware (x86_64) |
| 캠라파 | 192.168.0.43 (RealSense D405) |
| Pi IP | 192.168.0.36 |
| VM IP | 192.168.0.72 |
| ROS_DOMAIN_ID | 40 |

---

## 하드웨어 구성

| 장치 | 연결 | udev 심링크 |
|------|------|-------------|
| OpenCR (Dynamixel 모터 컨트롤러) | USB (VID:0483 PID:5740) | `/dev/opencr` |
| LiDAR LDS-02 | USB (VID:10c4 PID:ea60) | `/dev/lidar` |
| ToF 센서 (Raspberry Pi Pico 2) | USB (VID:2e8a PID:000f) | `/dev/tof` |
| 8BitDo Micro 게임패드 | Pi 내장 블루투스 (hci0) | `/dev/input/js0` |
| RealSense D405 | USB 3.0 필수 (5000M) | — (별도 실행) |

### USB 허브 구성
- VIA Labs VL812 허브 사용
- OpenCR, LiDAR, ToF → 허브 경유
- RealSense D405 → Pi USB 3.0 직결 (대역폭 필요)

---

## udev 규칙

`/etc/udev/rules.d/99-pico-tof.rules` (ToF 심링크 + ModemManager 무시):
```
SUBSYSTEM=="tty", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="000f", SYMLINK+="tof", ENV{ID_MM_DEVICE_IGNORE}="1"
SUBSYSTEM=="tty", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="000b", ENV{ID_MM_DEVICE_IGNORE}="1"
```

재적용:
```bash
sudo udevadm control --reload-rules && sudo udevadm trigger --subsystem-match=tty
```

---

## 블루투스 컨트롤러

- **장치**: 8BitDo Micro (MAC: E4:17:D8:E6:37:F9)
- **모드**: X-input 모드 (Start 버튼 3초 누름)
- **Pi BT 어댑터**: hci0 (D8:3A:DD:CD:D0:4F)

재연결:
```bash
bluetoothctl connect E4:17:D8:E6:37:F9
```

페어링 모드 진입: 전원 ON 후 **Start 버튼 3초** 길게 누름 (LED 빠르게 깜빡임)

---

## ROS2 패키지

### 1. `assisted_teleop`
경로: `src/assisted-teleop/`

| 파일 | 역할 |
|------|------|
| `tof_sensor_node.py` | ToF 직렬 읽기 → `/tof_distance`, `/tof_image`, `/tof/distances`, `/tof/status`<br>**`distance_scale=10.0`** 파라미터: Pico 출력 cm → mm 변환 |
| `depth_bridge_node.py` | RealSense aligned depth → `/depth/front_distance` (Float32, m)<br>중앙 ROI 40%×40%, 5th percentile, `skip=3` |
| `distance_marker_node.py` | LaserScan + ToF + Depth → `/distance_markers` (RViz2 거리선) |
| `config/joystick_8bitdo_micro.yaml` | 8BitDo Micro D-pad 매핑 |
| `config/joystick_xbox.yaml` | Xbox 컨트롤러 매핑 |
| `launch/assisted_teleop_launch.py` | 조이스틱 + ToF + Depth + 마커 런치 |

### 2. `voice_alert`
경로: `src/voice_alert/`

| 파일 | 역할 |
|------|------|
| `voice_alert/voice_alert_node.py` | `/tof_distance` 구독 → TTS 음성 알림 |

---

## 토픽 구조

```
[8BitDo Micro] → /dev/input/js0
    → joy_node → /joy
    → teleop_twist_joy_node → /joy_vel (Twist)
    → assisted_teleop_bridge → /cmd_vel (TwistStamped)
    → turtlebot3_node → 모터

[Pico 2 ToF] → /dev/tof
    → tof_sensor_node → /tof_distance (Range)
                      → /tof_image (Image)
    → assisted_teleop_bridge (Guard Stop 입력)
    → [VM] voice_alert_node → TTS

[LiDAR LDS-02] → /dev/lidar
    → ld08_driver → /scan (LaserScan)
    → assisted_teleop_bridge (VFH 입력)
    → [VM] RViz2 시각화
```

---

## 장애물 회피 로직

### VFH-lite (라이다 기반)
- 전방 72개 bin으로 히스토그램 생성
- 전방 0.5m 이내 장애물 감지 시 우회 경로 계산
- 모든 경로 막히면 `[WARN] All paths blocked.` 출력 후 정지

### Guard Stop (ToF + 라이다)
| 조건 | 동작 |
|------|------|
| ToF < 0.30m + 전진 중 | `GUARD STOP: TOF(x.xxm)` → 즉시 정지 |
| 전방 라이다 < 0.10m | `GUARD STOP: FWD(x.xx)` → 즉시 정지 |
| 후방 라이다 < 0.10m + 후진 중 | `GUARD STOP: REAR(x.xx)` → 즉시 정지 |

### ToF 경보 (터미널 출력)
- 장애물 400mm 이내: `[WARN] ToF 장애물 감지: xxxmm`

---

## 조작법 (8BitDo Micro)

| 입력 | 동작 | 속도 |
|------|------|------|
| D-pad ↑ | 전진 | 0.20 m/s |
| D-pad ↓ | 후진 | 0.20 m/s |
| D-pad ← | 좌회전 | 1.0 rad/s |
| D-pad → | 우회전 | 1.0 rad/s |
| R 버튼 + D-pad | 터보 | 0.22 m/s / 1.5 rad/s |

---

## 수정한 외부 파일

### `src/turtlebot3/turtlebot3_bringup/launch/robot.launch.py`
```python
usb_port = LaunchConfiguration('usb_port', default='/dev/opencr')  # ttyACM0 → /dev/opencr
# LiDAR port
launch_arguments={'port': '/dev/lidar', ...}  # ttyUSB0 → /dev/lidar
```

---

## 실행 방법

### Pi 원커맨드 (alias)

```bash
rob        # 전체 시작 (CamPi + VM + Pi 스택)
rob_stop   # 전체 종료
rob_log    # tmux 세션 재접속
```

### 스크립트 직접 실행

```bash
~/turtlebot3_ws/start.sh   # 전체 시작
~/turtlebot3_ws/stop.sh    # 전체 종료
tmux attach -t robot       # 로그 확인
```

### 캠라파 원커맨드

```bash
cam   # RealSense D405 시작 (alias)
```

### VM 원커맨드

```bash
mon   # 시각화 시작 (alias)
```

---

## 알려진 이슈

| 문제 | 원인 | 해결 |
|------|------|------|
| BT 연결 시 SSH 끊김 | WiFi/BT 2.4GHz 간섭 | BT 먼저 연결 후 launch, tmux 사용 |
| RealSense "Device busy" | USB 2.0 대역폭 부족 | USB 3.0(파란 포트) 직결 필수 |
| 4개 노드 동시 실행 시 Pi 꺼짐 | 과열/과부하 | RealSense는 별도 실행 |
| RViz2 LaserScan 0 points | QoS 불일치 | Reliability → `Best Effort` 로 변경 |

---

## GitHub

- 레포: https://github.com/jungju3756-source/turtlebot3-assisted-teleop
- 브랜치: `main`
