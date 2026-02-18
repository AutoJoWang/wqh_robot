#!/usr/bin/env python3
"""Orchestrator: simulation + Nav2 + RViz."""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_axioma_gazebo = get_package_share_directory('axioma_gazebo')
    pkg_axioma_navigation = get_package_share_directory('axioma_navigation')

    rviz_config = os.path.join(pkg_axioma_navigation, 'rviz', 'navigation.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_axioma_gazebo, 'launch', 'simulation.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_axioma_navigation, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Static TF map->odom (until AMCL initializes)
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation clock'),
        simulation,
        static_transform_publisher,
        navigation,
        rviz,
    ])
