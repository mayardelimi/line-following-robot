#!/usr/bin/env python3
"""Start new Gazebo (gz sim) with the line-follower world, spawn the robot and bridge topics."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_path = get_package_share_directory('line_maze_solver')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    world_path = os.path.join(pkg_path, 'worlds', 'line_follower.world')

    # lets "model://line_path_ground_plane" in the world file be found
    set_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(pkg_path, 'models'))

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r -v 3 {world_path}'}.items(),
    )

    # robot_description is published by robot_state_publisher; wait a few seconds for gz to be up
    spawn = TimerAction(
        period=6.0,
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', 'robot_description',
                '-name', 'maze_bot',
                '-x', '9.464182',
                '-y', '13.545395',
                '-z', '0.034947',
            ],
            output='screen',
        )],
    )

    # '[' = gz -> ROS, ']' = ROS -> gz
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/camera_sensor/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
        ],
        output='screen',
    )

    return LaunchDescription([set_resource_path, gz_sim, bridge, spawn])
