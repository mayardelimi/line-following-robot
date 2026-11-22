#!/usr/bin/env python3
"""Full simulation: Gazebo + robot + chosen controller (+ optional viewer and RViz).

    ros2 launch line_maze_solver maze_bot_launch.py                  # CNN controller (default)
    ros2 launch line_maze_solver maze_bot_launch.py controller:=pid  # PID controller
    ros2 launch line_maze_solver maze_bot_launch.py viewer:=false use_rviz:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

PACKAGE = 'line_maze_solver'


def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory(PACKAGE), 'launch')

    def include(name, **arguments):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, name)),
            launch_arguments=arguments.items())

    controller = LaunchConfiguration('controller')

    return LaunchDescription([
        DeclareLaunchArgument('controller', default_value='cnn', description='cnn | pid'),
        DeclareLaunchArgument('viewer', default_value='true', description='Open the camera / overlay windows'),
        DeclareLaunchArgument('use_rviz', default_value='true', description='Start RViz2'),

        include('robot_state_publisher_launch.py'),
        include('gazebo_launch.py'),

        Node(package=PACKAGE, executable='cnn_follower.py', name='cnn_follower', output='screen',
             condition=IfCondition(PythonExpression(["'", controller, "' == 'cnn'"]))),
        Node(package=PACKAGE, executable='pid_follower.py', name='pid_follower', output='screen',
             condition=IfCondition(PythonExpression(["'", controller, "' == 'pid'"]))),
        Node(package=PACKAGE, executable='frame_viewer.py', name='frame_viewer', output='screen',
             condition=IfCondition(LaunchConfiguration('viewer'))),

        include('rviz_launch.py', use_rviz=LaunchConfiguration('use_rviz')),
    ])
