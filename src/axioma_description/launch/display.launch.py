#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError


def check_package_exists(package_name):
    """Check if a ROS 2 package exists."""
    try:
        get_package_share_directory(package_name)
        return True
    except PackageNotFoundError:
        return False


def generate_launch_description():
    # Obtener el directorio del paquete
    pkg_axioma_description = get_package_share_directory('axioma_description')

    # Ruta al archivo URDF
    urdf_file = os.path.join(pkg_axioma_description, 'urdf', 'axioma.urdf')

    # Ruta al archivo de configuración de RViz2
    rviz_config_file = os.path.join(pkg_axioma_description, 'rviz', 'display.rviz')

    # Leer el contenido del URDF
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Argumentos del launch
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start joint_state_publisher_gui if true (requires ros-humble-joint-state-publisher-gui)'
    )

    # Nodo robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': robot_desc
        }]
    )

    # Check if joint_state_publisher_gui is available
    gui_available = check_package_exists('joint_state_publisher_gui')
    
    # Nodo joint_state_publisher_gui (only if gui=True and package is available)
    if gui_available:
        joint_state_publisher_gui = Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
            condition=IfCondition(LaunchConfiguration('gui'))
        )
    else:
        joint_state_publisher_gui = None
        print("[WARNING] joint_state_publisher_gui package not found. Using joint_state_publisher instead.")
        print("[INFO] To install: sudo apt install ros-humble-joint-state-publisher-gui")

    # Nodo joint_state_publisher (fallback if GUI is not available or gui=False)
    if gui_available:
        # If GUI available, use non-GUI version only when gui=False
        joint_state_publisher = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
            condition=UnlessCondition(LaunchConfiguration('gui'))
        )
    else:
        # If GUI not available, always use non-GUI version
        joint_state_publisher = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        )

    # Nodo RViz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    launch_description = [
        use_sim_time_arg,
        gui_arg,
        robot_state_publisher,
        rviz2,
    ]
    
    # Add joint state publisher nodes
    if joint_state_publisher_gui is not None:
        launch_description.append(joint_state_publisher_gui)
    launch_description.append(joint_state_publisher)

    return LaunchDescription(launch_description)
