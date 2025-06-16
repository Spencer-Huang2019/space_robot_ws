#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable, LaunchConfiguration
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('publish_frequency', default_value='15.0'))
    ld.add_action(DeclareBooleanLaunchArg('use_gazebo', default_value=True))
    
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([FindPackageShare('space_robot_learning'), 'description', 'urdf', 'space_robot.urdf.xacro']),
            ' ',
            'use_gazebo:=',
            LaunchConfiguration('use_gazebo'),
        ]
    )

    robot_description = {'robot_description': robot_description_content}

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        respawn=True,
        output='screen',
        parameters=[
            robot_description,
            {
                'publish_frequency': LaunchConfiguration('publish_frequency'),
            },
        ]
    )
    ld.add_action(robot_state_publisher_node)

    return ld