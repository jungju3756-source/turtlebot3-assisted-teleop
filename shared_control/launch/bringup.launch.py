"""Full TurtleBot3 + VFH shared_control stack launch.

실행 노드:
  * joy_node          (컨트롤러 → /joy)
  * teleop_twist_joy  (/joy → /cmd_vel_manual)
  * tof_sensor_node   (VL53L8CX 8x8 → /tof/distances, /tof/status, /tof_image)
  * depth_bridge_node (RealSense aligned depth → /depth/front_distance)
  * distance_marker_node (RViz2 마커)
  * tof_bridge        (/tof/distances → /tof/front_distance)
  * obstacle_detector (ToF + LiDAR + Depth → /obstacle/detected)
  * gap_analyzer      (LiDAR 갭 폭 → /gap/width, /gap/passable)
  * avoidance         (회피 명령 → /cmd_vel_auto)
  * arbitrator        (상태머신 → /cmd_vel, /robot/state)

별도로 실행 (사전 조건):
  TURTLEBOT3_MODEL=burger LDS_MODEL=LDS-02 ros2 launch turtlebot3_bringup robot.launch.py
  CamPi: ~/start.sh  (RealSense + depth_bridge_node, align_depth=OFF, raw depth 사용)
         align_depth 는 Pi CPU 과부하로 depth 스트림이 멈추므로 반드시 OFF.
"""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sc_share = FindPackageShare('shared_control')

    params   = PathJoinSubstitution([sc_share, 'config', 'shared_control_params.yaml'])
    joy_yaml = PathJoinSubstitution([sc_share, 'config', 'teleop_twist_joy.yaml'])

    return LaunchDescription([
        # ── 조이스틱 ─────────────────────────────────────────────────
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{'device_id': 0, 'deadzone': 0.05, 'autorepeat_rate': 20.0}]),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            output='screen',
            parameters=[joy_yaml],
            remappings=[('/cmd_vel', '/cmd_vel_manual')]),

        # ── ToF 센서 (/tof/distances, /tof/status, /tof_image) ──────
        Node(
            package='assisted_teleop',
            executable='tof_sensor_node',
            name='tof_sensor_node',
            output='screen',
            parameters=[{
                'serial_port':      '/dev/tof',
                'obstacle_dist_mm': 1200,
                'front_rows':       [1, 2, 3, 4, 5, 6],
                'front_cols':       [1, 2, 3, 4, 5, 6],
                'distance_scale':   1.0,   # VL53L8CX 는 mm 출력 → 스케일 불필요 (10이면 10배 뻥튀기)
            }]),

        # ── shared_control 스택 ──────────────────────────────────────
        Node(
            package='shared_control',
            executable='tof_bridge',
            name='tof_bridge',
            output='screen',
            parameters=[params]),
        # ── 장애물 회피 비활성화 (사용자 요청 2026-06-18) ──────────────
        # 아래 3개 노드를 끄면 /obstacle/detected 가 발행되지 않아
        # arbitrator 가 항상 MANUAL → teleop(/cmd_vel_manual)이 그대로 통과.
        # 회피를 다시 켜려면 obstacle_detector/gap_analyzer/avoidance 주석 해제.
        # Node(
        #     package='shared_control',
        #     executable='obstacle_detector',
        #     name='obstacle_detector',
        #     output='screen',
        #     parameters=[params]),
        # Node(
        #     package='shared_control',
        #     executable='gap_analyzer',
        #     name='gap_analyzer',
        #     output='screen',
        #     parameters=[params]),
        # Node(
        #     package='shared_control',
        #     executable='avoidance',
        #     name='avoidance',
        #     output='screen',
        #     parameters=[params]),
        Node(
            package='shared_control',
            executable='arbitrator',
            name='arbitrator',
            output='screen',
            parameters=[params]),
    ])
