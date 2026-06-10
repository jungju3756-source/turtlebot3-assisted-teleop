from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    joy_type_arg = DeclareLaunchArgument(
        'joy_type', default_value='default',
        description='Joystick config name (default | 8bitdo_micro | ...)')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock (true for Gazebo)')

    joy_type     = LaunchConfiguration('joy_type')
    use_sim_time = LaunchConfiguration('use_sim_time')

    joy_config_file = PythonExpression(["'joystick_' + '", joy_type, "' + '.yaml'"])

    joy_params = PathJoinSubstitution([
        FindPackageShare('assisted_teleop'), 'config', joy_config_file])

    # ── 조이스틱 드라이버 ──────────────────────────────────────────────
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[joy_params, {'use_sim_time': use_sim_time}])

    # ── 조이스틱 → /cmd_vel_manual (arbitrator가 수신) ────────────────
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[joy_params, {'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel', '/cmd_vel_manual')])

    # ── ToF 센서 (/tof_distance, /tof_image, /tof/distances, /tof/front_distance)
    tof_node = Node(
        package='assisted_teleop',
        executable='tof_sensor_node',
        name='tof_sensor_node',
        parameters=[{'serial_port': '/dev/tof',
                     'obstacle_dist_mm': 400}],
        output='screen')

    # ── 거리 마커 (RViz2 /distance_markers 표시용) ────────────────────
    distance_marker_node = Node(
        package='assisted_teleop',
        executable='distance_marker_node',
        name='distance_marker_node',
        output='screen')

    # ── 뎁스 → PointCloud2 변환 (캠라파 D405 → /camera/points) ────────
    depth_pointcloud_node = Node(
        package='assisted_teleop',
        executable='depth_pointcloud_node',
        name='depth_pointcloud_node',
        parameters=[{'skip': 4}],   # Pi 부하 감소: 4프레임마다 1개 발행
        output='screen')

    return LaunchDescription([
        joy_type_arg,
        use_sim_time_arg,
        joy_node,
        teleop_node,
        tof_node,
        distance_marker_node,
        depth_pointcloud_node,
    ])
