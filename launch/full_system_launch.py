import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    SetEnvironmentVariable, TimerAction, ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    joy_type_arg = DeclareLaunchArgument(
        'joy_type', default_value='8bitdo_micro',
        description='Joystick config name')
    joy_type = LaunchConfiguration('joy_type')

    set_tb3_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger')
    set_lds_model = SetEnvironmentVariable('LDS_MODEL', 'LDS-02')

    # ── 1. TurtleBot3 Bringup ──────────────────────────────────────────
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('turtlebot3_bringup'),
            '/launch/robot.launch.py']))

    # ── 2. 조이스틱 + ToF + 거리마커 ──────────────────────────────────
    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('assisted_teleop'),
            '/launch/assisted_teleop_launch.py']),
        launch_arguments={'joy_type': joy_type}.items())

    # ── 3. SLAM (lifecycle 자동 활성화) ────────────────────────────────
    slam_params = os.path.join(
        get_package_share_directory('assisted_teleop'), 'config', 'slam_toolbox.yaml')
    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_params],
        output='screen')

    slam_configure = TimerAction(
        period=12.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'configure'],
            output='screen')])
    slam_activate = TimerAction(
        period=17.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'activate'],
            output='screen')])

    # ── 4. shared_control (VFH 자동회피 스택) ─────────────────────────
    sc_params = os.path.join(
        get_package_share_directory('shared_control'), 'config',
        'shared_control_params.yaml')

    tof_bridge = Node(
        package='shared_control', executable='tof_bridge',
        name='tof_bridge', parameters=[sc_params], output='screen')

    obstacle_detector = Node(
        package='shared_control', executable='obstacle_detector',
        name='obstacle_detector', parameters=[sc_params], output='screen')

    gap_analyzer = Node(
        package='shared_control', executable='gap_analyzer',
        name='gap_analyzer', parameters=[sc_params], output='screen')

    avoidance = Node(
        package='shared_control', executable='avoidance',
        name='avoidance', parameters=[sc_params], output='screen')

    arbitrator = Node(
        package='shared_control', executable='arbitrator',
        name='arbitrator', parameters=[sc_params], output='screen')

    return LaunchDescription([
        joy_type_arg,
        set_tb3_model,
        set_lds_model,
        bringup,
        teleop,
        slam,
        slam_configure,
        slam_activate,
        tof_bridge,
        obstacle_detector,
        gap_analyzer,
        avoidance,
        arbitrator,
    ])
