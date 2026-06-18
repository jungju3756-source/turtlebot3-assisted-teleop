"""VM 전용 통합 launch — 명령어 하나로 모든 시각화 실행.

  ros2 launch tb3_monitor vm_monitor.launch.py

실행되는 것:
  * tb3_monitor   — 터미널 상태 표시 + /monitor/markers (RViz2 마커)
  * voice_alert   — TTS 음성 알림 (장애물 0.4m 이내)
  * yolo_detect   — YOLO 객체 감지 (best.pt → /yolo/image 바운딩박스)
  * distance_hud  — ToF/LiDAR 거리 코너 패널 (/distance_hud)
  * rviz2         — 통합 설정 (지도 + 라이다 + 마커 + 카메라 + YOLO)

사전 조건:
  export ROS_DOMAIN_ID=40
  source /opt/ros/jazzy/setup.bash
  source ~/turtlebot3_ws/install/setup.bash
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('tb3_monitor')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'full_system.rviz')

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='RViz2 실행 여부 (헤드리스 환경에서는 false)')
    use_rviz = LaunchConfiguration('use_rviz')

    nodes = [
        use_rviz_arg,

        # ── 터미널 상태 모니터 + /monitor/markers ─────────────────────
        Node(
            package='tb3_monitor',
            executable='monitor_node',
            name='tb3_monitor',
            output='screen',
            emulate_tty=True),

        # ── TTS 음성 알림 (/tof_distance 기준 0.4m 이내) ──────────────
        Node(
            package='voice_alert',
            executable='voice_alert_node',
            name='voice_alert',
            output='screen'),

        # ── YOLO 객체 감지 (best.pt → /yolo/image 바운딩박스) ─────────
        Node(
            package='tb3_monitor',
            executable='yolo_detect',
            name='yolo_detect',
            output='screen',
            parameters=[{
                'model_path':     '/home/jungju/turtlebot3_ws/best.pt',
                'image_topic':    '/camera/camera/color/image_raw/compressed',
                'use_compressed': True,
                'conf_threshold': 0.5,
                'imgsz':          320,
                'output_width':   424,
            }]),

        # ── ToF/LiDAR 거리 코너 HUD ──────────────────────────────────
        Node(
            package='tb3_monitor',
            executable='distance_hud',
            name='distance_hud',
            output='screen'),

        # ── RViz2 통합 시각화 ─────────────────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'),
    ]

    return LaunchDescription(nodes)
