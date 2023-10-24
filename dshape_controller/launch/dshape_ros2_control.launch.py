#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro


def generate_launch_description():
    controller_pkg = get_package_share_directory('dshape_controller')
    description_pkg = get_package_share_directory('dshape_description')
    
    bot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        description_pkg,'launch','description_ros2_control.launch.py'
        )]), launch_arguments={'use_sim_time': 'false'}.items()
    )

    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        controller_pkg,'launch','controller_spawner.launch.py'
        )]), 
        # launch_arguments={'use_sim_time': 'false'}.items()
    )


    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(description_pkg, 'rviz', 'slam.rviz')],
        condition=IfCondition(LaunchConfiguration('open_rviz'))
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('open_rviz', default_value='false', description='Open RViz.'),
            bot,
            rviz_node,
            controller,
        ]
    )