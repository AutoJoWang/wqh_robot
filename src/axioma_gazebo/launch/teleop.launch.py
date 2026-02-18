from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch joystick teleop (joy_node + teleop_twist_joy)."""

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{'dev': '/dev/input/js0'}]
    )

    teleop_joy = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'axis_linear.x': 1,
            'axis_angular.yaw': 0,
            'scale_linear.x': 0.5,
            'scale_angular.yaw': 2.0
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        joy_node,
        teleop_joy,
    ])
