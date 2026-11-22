#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    xacro_file = os.path.join(get_package_share_directory('line_maze_solver'), 'urdf', 'robot.urdf.xacro')
    # value_type=str stops newer ROS from trying to parse the URDF as YAML
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        ),
    ])
