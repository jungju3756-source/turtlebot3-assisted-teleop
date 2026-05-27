from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    joy_type_arg = DeclareLaunchArgument(
        'joy_type', default_value='default',
        description='Joystick config name (default | mocute_052 | ...)')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock (true for Gazebo)')

    joy_type       = LaunchConfiguration('joy_type')
    use_sim_time   = LaunchConfiguration('use_sim_time')

    joy_config_file = PythonExpression(["'joystick_' + '", joy_type, "' + '.yaml'"])

    joy_params = PathJoinSubstitution([
        FindPackageShare('assisted_teleop'), 'config', joy_config_file])

    tb3_params = PathJoinSubstitution([
        FindPackageShare('assisted_teleop'), 'config', 'turtlebot3_params.yaml'])

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[joy_params, {'use_sim_time': use_sim_time}])

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[joy_params, {'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel', '/joy_vel')])

    assisted_teleop_node = Node(
        package='assisted_teleop',
        executable='assisted_teleop_bridge',
        name='assisted_teleop_bridge',
        parameters=[tb3_params, {'use_sim_time': use_sim_time}],
        output='screen')

    tof_node = Node(
        package='assisted_teleop',
        executable='tof_sensor_node',
        name='tof_sensor_node',
        parameters=[{'serial_port': '/dev/ttyACM1',
                     'obstacle_dist_mm': 400}],
        output='screen')

    return LaunchDescription([
        joy_type_arg,
        use_sim_time_arg,
        joy_node,
        teleop_node,
        assisted_teleop_node,
        tof_node,
    ])
