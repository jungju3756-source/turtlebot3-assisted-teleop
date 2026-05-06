# assisted_teleop

VFH-lite(Vector Field Histogram) 기반 Assisted Teleoperation ROS 2 패키지.

조이스틱 입력을 그대로 전달하되, 장애물 접근 시 자동으로 방향을 보정하고 충돌 직전 긴급 정지(Guard Stop)를 수행한다.

---

## 알고리즘 개요

```
조이스틱 → [teleop_twist_joy] → /joy_vel
                                      ↓
                         [assisted_teleop_bridge]
                          ├── LiDAR(/scan) 수신
                          ├── VFH-lite: 빈 방향 탐색 → 조향 보정
                          └── Guard Stop: 4방향 물리 충돌 차단
                                      ↓
                                  /cmd_vel → 로봇 구동
```

### 주요 기능
| 기능 | 설명 |
|------|------|
| VFH-lite | 72-bin 극좌표 히스토그램으로 가장 열린 방향 탐색, 조향 보정 |
| 4방향 Guard Stop | 전/후/좌/우 범퍼 기준 거리 측정, 충돌 직전 강제 정지 |
| 속도 감속 | 장애물 접근 시 거리 비례 감속 (min_speed_frac 이하로는 내려가지 않음) |
| 스무딩 | VFH 개입 시에만 각속도 스무딩 적용 (수동 조작은 즉각 응답) |

---

## 개발 이력

- **v1.0** — Orange Pi 5 + 스쿠터 플랫폼에서 구현 및 실주행 검증
  - LiDAR 위치 오프셋 보정 (laser_x=0.44m, laser_y=0.24m)
  - 스쿠터 footprint 기준 (front=0.75m, half_w=0.40m)
  - 휠체어 본체 노이즈(팔걸이·다리) 필터링
  - 갈지자(Zig-zag) 주행 방지 히스테리시스 적용

- **v1.1** — TurtleBot3 포팅 준비
  - `scan_topic` / `output_topic` 파라미터화
  - TB3 Burger 기본값 적용 (`config/turtlebot3_params.yaml`)
  - `use_sim_time` launch 인수 추가 (Gazebo 시뮬 지원)

---

## TurtleBot3 포팅 시 확인 사항

### 1. 파라미터 (`config/turtlebot3_params.yaml`)

현재 TB3 Burger 기준 기본값이 설정되어 있다. 실제 로봇에 맞게 검증 필요:

| 파라미터 | TB3 기본값 | 스쿠터 값 | 설명 |
|----------|-----------|----------|------|
| `laser_x_offset` | 0.0 | 0.44 | LiDAR X 위치 (base_link 기준, m) |
| `laser_y_offset` | 0.0 | 0.24 | LiDAR Y 위치 (base_link 기준, m) |
| `robot_front_m` | 0.17 | 0.75 | base_link → 앞 범퍼 거리 (m) |
| `robot_half_w` | 0.09 | 0.40 | 로봇 반폭 (m) |
| `free_dist_m` | 0.50 | 1.00 | 이 거리 이상이면 ADAS 미개입 (m) |
| `guard_dist_m` | 0.10 | 0.15 | 긴급 정지 거리 (m) |
| `clearance_m` | 0.25 | 0.55 | VFH 방향 통과 최소 거리 (m) |

### 2. LiDAR 좌표계 확인

TB3의 `/scan` 토픽이 `base_link` 기준으로 발행되는지, `laser_link` 기준인지 확인.  
`laser_link`라면 `laser_x_offset` / `laser_y_offset`에 실제 tf 변환값을 입력해야 한다.

```bash
ros2 run tf2_ros tf2_echo base_link laser_link
```

### 3. 아직 미완성인 부분 (개발 필요)

- [ ] TB3 실제 주행 파라미터 튜닝 (`free_dist_m`, `clearance_m` 등)
- [ ] Gazebo 시뮬레이션 검증
- [ ] `scan_topic` TB3가 필터링된 스캔을 쓰는 경우 처리 (`scan_filtered` 여부 확인)
- [ ] 속도 제한 하드코딩 (`±1.4m/s`) → TB3 최대속도(0.22m/s)에 맞게 파라미터화 필요

---

## 빌드 및 실행

### 의존 패키지 설치
```bash
sudo apt install ros-jazzy-joy ros-jazzy-teleop-twist-joy
```

### 빌드
```bash
# 워크스페이스 루트에서
colcon build --packages-select assisted_teleop
source install/setup.bash
```

### 실행
```bash
# 기본 조이스틱
ros2 launch assisted_teleop assisted_teleop_launch.py

# Mocute 052 조이스틱
ros2 launch assisted_teleop assisted_teleop_launch.py joy_type:=mocute_052

# Gazebo 시뮬레이션
ros2 launch assisted_teleop assisted_teleop_launch.py use_sim_time:=true
```

### 토픽 확인
```bash
ros2 topic echo /cmd_vel          # 출력 확인
ros2 topic hz /scan               # LiDAR 수신 확인
ros2 run rqt_graph rqt_graph      # 전체 노드 구성 확인
```

---

## 파일 구조

```
assisted_teleop/
├── assisted_teleop/
│   └── assisted_teleop_bridge.py   # 핵심 노드 (VFH + Guard Stop)
├── config/
│   ├── turtlebot3_params.yaml      # TB3용 파라미터
│   ├── joystick_default.yaml       # 기본 조이스틱 설정
│   └── joystick_mocute_052.yaml    # Mocute 052 설정
├── launch/
│   └── assisted_teleop_launch.py   # joy + teleop_twist_joy + 이 노드 통합 실행
├── package.xml
├── setup.py
└── README.md
```
