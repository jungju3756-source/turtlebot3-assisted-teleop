# TurtleBot3 Assisted-Teleop / Shared-Control 워크스페이스

TurtleBot3 Burger 기반 **3-센서 융합(ToF · Depth · LiDAR) 보조 주행 + 원격 모니터링** 시스템.
본 문서는 현재 구현·튜닝된 전체 구성을 정리합니다. 패키지별 상세는
[`src/shared_control/README.md`](src/shared_control/README.md) 참고.

---

## 1. 시스템 구성 (3대 머신)

| 머신 | IP | 역할 | 실행 |
|---|---|---|---|
| **Pi** (로봇) | 192.168.0.36 | 주행·ToF·LiDAR·중재 | `shared_control bringup.launch.py`, `turtlebot3_bringup`, `voice_alert` |
| **CamPi** | 192.168.0.43 | RealSense D405 + depth 추출 | RealSense `~/start.sh` + `depth_bridge_node.py` |
| **VM** | 192.168.0.72 | RViz2 · HUD · YOLO 모니터링 | `tb3_monitor vm_monitor.launch.py` |

- 공통: **ROS2 Jazzy**, `ROS_DOMAIN_ID=40`, `rmw_fastrtps_cpp`
- 한 줄 기동: 각 머신에서 `bash ~/start.sh` (Pi에서 실행 시 CamPi·VM 자동 원격 기동)

```
[8BitDo] → Pi(teleop) ─┐
                       ▼
         ToF ─┐  arbitrator → /cmd_vel → 로봇
   CamPi Depth┤      ▲ (현재 회피 OFF → 항상 MANUAL 통과)
        LiDAR ┘      │
                  VM HUD / RViz2 / YOLO
```

---

## 2. 센서 측정 기준 (현재 튜닝값)

세 거리값은 서로 다른 머신에서 계산되어 VM HUD(`/distance_hud`)에 ToF / Depth / LiDAR 3줄로 표시됩니다.

| 센서 | 토픽 | 계산 위치 | 보는 영역 | 집계 | 보정 |
|---|---|---|---|---|---|
| **ToF** | `/tof/front_distance` | Pi `tof_bridge` | 8×8 그리드 **정중앙 2×2** (행3-4·열3-4) | 최솟값 | 없음 |
| **Depth** | `/depth/front_distance` | CamPi `depth_bridge` | 이미지 **중앙 40%×40%** | 5퍼센타일 | 없음 |
| **LiDAR** | `/scan` | VM `distance_hud` | **카메라 정면 ±6°** | 최솟값 | **−5cm** |

### LiDAR 정렬 (중요)
- LDS-02는 카메라(ToF/Depth)와 **180° 반대로 장착** → 스캔 0°가 로봇 뒤를 향함.
  HUD는 정면을 `LIDAR_FORWARD = π` 기준 ±6°로 샘플링하여 카메라와 같은 방향을 본다.
- LiDAR는 ToF/Depth보다 **5cm 뒤**에 장착 → 측정거리에서 `LIDAR_OFFSET = 0.05 m` 차감해 기준면을 맞춘다.
- 세 값을 동일 정면·동일 기준면으로 맞췄으나, LiDAR(수평면) vs 카메라(장착 높이) 차이로 인한 미세 편차는 잔존.

### ToF 스케일
- VL53L8CX는 **mm 단위 그대로 출력** → `distance_scale = 1.0` (10이면 10배 뻥튀기 버그).

---

## 3. 장애물 회피 (현재 **비활성화**)

사용자 요청으로 자동 회피를 꺼둔 상태. `bringup.launch.py`에서 다음 3개 노드가 주석 처리됨:

- `obstacle_detector` (장애물 감지 → `/obstacle/detected`)
- `gap_analyzer` (갭 분석)
- `avoidance` (회피 명령 → `/cmd_vel_auto`)

→ `/obstacle/detected`가 발행되지 않아 `arbitrator`가 항상 **MANUAL** 상태,
조이스틱 명령(`/cmd_vel_manual`)이 그대로 `/cmd_vel`로 통과. 로봇이 스스로 회피/정지하지 않음.

**다시 켜기:** `bringup.launch.py`에서 위 3개 `Node(...)` 블록 주석 해제 후 재빌드.
(주의: `obstacle_detector`의 LiDAR 정면 섹터도 0° 기준이라, 재활성화 시 카메라 정면(180°)으로 동일 반전 필요.)

---

## 4. 음성 경보 (voice_alert, 활성)

- Pi에서 동작. **전진 중일 때만** 경보 (`/cmd_vel`의 `linear.x > 0.05`).
- `/tof_distance < 1.2 m` & 쿨다운 3초마다 espeak-ng로 "Obstacle ahead, stop".
- 정지/후진 시에는 침묵.

---

## 5. 빌드 · 실행

```bash
# 빌드 (각 머신 워크스페이스에서)
cd ~/turtlebot3_ws
colcon build --symlink-install
source install/setup.bash

# 통합 기동 (Pi에서 실행하면 CamPi·VM도 원격 자동 기동)
bash ~/start.sh
```

### CamPi RealSense 설정 (필수)
- `align_depth.enable:=false` — **반드시 OFF**. ON이면 Pi CPU 과부하로 depth 스트림이 멈춤(0.3fps).
  → raw depth 토픽 `/camera/camera/depth/image_rect_raw` 사용.
- depth_module 수동 노출: `enable_auto_exposure:=false exposure:=50000 gain:=16`
  (자동 노출은 IR/depth용으로 튜닝돼 컬러가 검게 나옴).
- D405는 color·depth가 **동일 depth 모듈**에서 나옴 (별도 RGB 센서 없음, `rgb_camera.*` 무효).

### VM 모니터링
- DISPLAY=`:1`, `XAUTHORITY=/run/user/1000/gdm/Xauthority` (x11 세션).
- RViz2 "카메라" = YOLO Detection 디스플레이(`/yolo/image`), 압축 컬러를 VM 로컬에서 추론.

---

## 6. 트러블슈팅 메모

| 증상 | 원인 | 해결 |
|---|---|---|
| depth 0.3fps로 멈춤 | `align_depth=true` Pi 과부하 | raw depth + align OFF |
| 원격 기동 SSH exit 255 | `nohup … &`가 SSH fd 점유 | tmux(CamPi) / `setsid … </dev/null`(VM) |
| VM RViz2 "display :0" 실패 | VM은 `:1` | `DISPLAY=:1` + gdm XAUTHORITY |
| 카메라 검은 화면 | D405 자동노출(IR용) | depth_module 수동 노출 50000/gain 16 |
| ToF 8000~31000mm | `distance_scale=10` | `1.0` (mm 그대로) |
| 노드 중복/이름 충돌 | `ros2 launch` 부모 미종료 | launch 부모까지 종료 후 1회 재기동 |
| `pgrep -f`로 카운트 2배 | 패턴 문자열이 자기 셸 cmdline 매칭 | PID 기반 확인, 또는 `ps pid/ppid` 트리로 검증 |
| TTS 계속 울림 | 정지 중에도 경보 | 전진 중(`linear.x>0.05`)에만 발화 |

---

## 7. 주요 토픽

| 토픽 | 타입 | 발행 | 설명 |
|---|---|---|---|
| `/tof/front_distance` | Float32 | Pi | ToF 중앙 2×2 최소거리(m) |
| `/depth/front_distance` | Float32 | CamPi | depth 중앙 ROI 5퍼센타일(m) |
| `/scan` | LaserScan | Pi | LDS-02 360° |
| `/distance_hud` | Image | VM | ToF/Depth/LiDAR 3줄 오버레이 |
| `/cmd_vel_manual` | Twist | Pi | 조이스틱 입력 |
| `/cmd_vel` | TwistStamped | Pi | arbitrator 최종 출력 |
| `/robot/state` | String | Pi | MANUAL / AVOIDANCE / STOPPED |
