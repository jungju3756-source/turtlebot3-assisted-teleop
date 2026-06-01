import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    joy_type_arg = DeclareLaunchArgument(
        'joy_type', default_value='8bitdo_micro',
        description='Joystick config name')

    joy_type = LaunchConfiguration('joy_type')

    # ── 환경 변수 ──────────────────────────────────────────────
    set_tb3_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger')
    set_lds_model = SetEnvironmentVariable('LDS_MODEL',        'LDS-02')

    # ── 1. TurtleBot3 Bringup ──────────────────────────────────
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('turtlebot3_bringup'),
            '/launch/robot.launch.py']),
    )

    # ── 2. Assisted Teleop (joy + ToF + bridge) ────────────────
    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('assisted_teleop'),
            '/launch/assisted_teleop_launch.py']),
        launch_arguments={'joy_type': joy_type}.items(),
    )

    return LaunchDescription([
        joy_type_arg,
        set_tb3_model,
        set_lds_model,
        bringup,
        teleop,
    ])
